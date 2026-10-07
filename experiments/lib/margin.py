"""
Layer 2 (consequence estimation) safety-margin primitives — see
docs/archive/research_notes/open_world_safety_reframe_2026-08-20.md §4 and the
approved plan this session for the full design. Two independent margin
sources, combined via min(): distance to the nearest lane boundary (static
map geometry) and a worst-case object-clearance bound.

margin() values are a placeholder proxy, not a validated safety definition
-- the real margin-violation definition is Kalpit's to set once he's back
at the lab with live AWSIM access (see the plan's "explicitly bounded"
scope note). This module exists to make the Layer-2 MECHANISM (counterfactual
rollout -> P(violation)) testable now, on existing nominal data.
"""

import math

import lanelet2
import lanelet2.geometry as lg


def load_map(map_file: str):
    """Same MGRSProjector convention as
    st_gat/pipeline/State_Estimator/MapProcessor.py -- see that module's
    docstring for why this specific projector is required (generic
    projectors don't line up with the AWSIM/Autoware map frame)."""
    from autoware_lanelet2_extension_python.projection import MGRSProjector
    projector = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
    map_data, _load_errors = lanelet2.io.loadRobust(map_file, projector)
    return map_data


def lane_boundary_distance(x: float, y: float, map_data, n_candidates: int = 5) -> float:
    """Distance (metres) from (x, y) to the nearest lanelet boundary
    (left OR right edge, whichever is closer) among the n_candidates
    geometrically-nearest lanelets -- NOT just the single nearest lanelet
    by centroid, since the closest EDGE can belong to an adjacent lanelet
    (e.g. near a lane-change boundary). Checking a handful of candidates
    (findNearest) rather than every lanelet in the map keeps this cheap.

    Verified live against this repo's map (2026-08-21): a real recorded
    ego position (lane-centered) shows ~1.0-1.5m to each edge, both
    plausible for typical lane width -- lanelet2.geometry.distance()
    against leftBound/rightBound (converted to 2D via lg.to2D, the raw
    linestrings are 3D) gives a genuine distance-to-edge-line regardless
    of inside/outside, unlike lanelet2.geometry.distance(point, lanelet)
    itself (that one returns 0 for any point INSIDE the lanelet -- a cliff
    at the edge, not the continuous shrinking-margin-from-center signal
    Layer 2 needs).

    Returns float('inf') if no lanelets exist near the point at all (e.g.
    a position far off the mapped area) -- the caller's margin() then
    falls through to whatever the object-clearance term gives.
    """
    pt = lanelet2.core.BasicPoint2d(float(x), float(y))
    nearest = lg.findNearest(map_data.laneletLayer, pt, n_candidates)
    if not nearest:
        return float('inf')
    best = float('inf')
    for _dist, ll in nearest:
        left2d = lg.to2D(ll.leftBound)
        right2d = lg.to2D(ll.rightBound)
        best = min(best, lg.distance(pt, left2d), lg.distance(pt, right2d))
    return best


def object_clearance(dx: float, dy: float, speed: float, t_ahead: float,
                      ego_displacement: float = 0.0) -> float:
    """Worst-case clearance (metres) to one tracked object, t_ahead seconds
    from now.

    NOT literal constant-velocity dead-reckoning -- objects_set
    (sequence_builder.py's _build_object_set) deliberately stores only a
    frame-invariant SPEED MAGNITUDE, not a velocity vector (to avoid a
    twist-frame-mismatch bug, see that function's own docstring), and
    carries no persistent per-object identity across timesteps to infer
    heading by differencing consecutive frames either -- objects are
    re-sorted by distance fresh at every frame. A real directional
    extrapolation isn't supported by this data without either a fragile
    cross-frame ID-matching heuristic or reading raw bag data directly
    (out of scope for this "naive" v1 proxy).

    Conservative fallback used instead: assume the object COULD be closing
    directly along the line to ego at its currently observed speed
    (dx, dy, speed all as observed NOW, t_ahead=0), and that ego's own
    predicted displacement magnitude over the same window could ALSO be
    entirely toward the object in the worst case. Both are worst-case,
    direction-agnostic bounds -- this systematically UNDERESTIMATES
    clearance (reports more risk than a true vector calculation would),
    which is the conservative direction to be wrong in for a safety proxy.

    dx, dy: object's position relative to ego, AT THE CURRENT (t=0) frame,
        real metres (de-normalize from objects_set's [0,1] encoding via
        cfg.OBJECT_REL_POS_RANGE before calling this).
    speed: object's observed speed magnitude, real m/s (de-normalize via
        cfg.OBJECT_SPEED_RANGE).
    t_ahead: seconds into the counterfactual trajectory.
    ego_displacement: magnitude (metres) of how far the counterfactual ego
        trajectory has moved from its t=0 position by t_ahead -- pass 0.0
        to ignore this term (object-closing-rate only).
    """
    current_dist = math.hypot(dx, dy)
    return max(0.0, current_dist - speed * t_ahead - ego_displacement)


def margin(lane_dist: float, object_dists) -> float:
    """Combined proxy margin: the tightest of the lane-boundary distance
    and every tracked object's worst-case clearance. `object_dists` is an
    iterable of already-computed object_clearance() values (possibly
    empty, if no objects were tracked at this frame)."""
    candidates = [lane_dist] + list(object_dists)
    return min(candidates) if candidates else float('inf')


# ── v2 safe-set primitives (2026-10-04) ─────────────────────────────────────
# Why v2 exists: applying the v1 margin above to the ACTUAL recorded future of
# nominal calibration windows (no bootstrap at all) flagged ~71% of real
# nominal driving as a "violation" (lane term ~55%, object term ~34%). The
# v1 lane term measures distance to the nearest edge of ANY of the 5 nearest
# lanelets -- which includes the shared line between two adjacent same-
# direction lanes and the crossing lanelets inside every intersection -- so a
# lane-centered vehicle sits ~1.0-1.5m from "a boundary" constantly; the v1
# object term assumes every object AND ego close head-on, so any object
# within ~35m flags over a 3s horizon. A margin that the realized nominal
# trajectory violates is not a safety set, and P(violation) built on it is
# not a risk measure. v2 replaces both terms with sets the realized nominal
# trajectory actually stays inside (acceptance test: realized-nominal
# violation rate ~0, see layer2_consequence_estimation.py --check-safe-set).

EGO_HALF_WIDTH_M = 0.948   # awsim_labs_vehicle_description vehicle_info: (wheel_tread 1.64 + 2*0.128 overhang)/2
DRIVABLE_SUBTYPES = ('road', 'road_shoulder')   # NOT crosswalk/walkway (those are pedestrian space)


class DrivableArea:
    """Union of every road/road_shoulder lanelet polygon, rasterized once
    into a boolean grid, with a signed distance field (metres) to its
    boundary: positive inside the drivable area, negative outside. Unlike
    v1's lane_boundary_distance(), internal lanelet splits (adjacent lanes,
    consecutive lanelets, overlapping intersection lanelets) are NOT
    boundaries here -- only the edge of the road surface itself is.

    A small morphological closing fills sub-`close_m` gaps between adjacent
    lanelet polygons (rasterization seams, and the slivers between turning
    lanelets inside an intersection, which are still paved road). Lookups
    are vectorized nearest-cell reads, so evaluating K counterfactuals x T
    steps is a single numpy index, not K*T lanelet2 queries."""

    def __init__(self, map_data, resolution_m: float = 0.2, close_m: float = 1.0, pad_m: float = 20.0):
        from PIL import Image, ImageDraw
        from scipy import ndimage

        polys = []
        for ll in map_data.laneletLayer:
            subtype = ll.attributes['subtype'] if 'subtype' in ll.attributes else ''
            if subtype not in DRIVABLE_SUBTYPES:
                continue
            polys.append([(p.x, p.y) for p in ll.polygon2d()])
        xs = [x for poly in polys for x, _ in poly]
        ys = [y for poly in polys for _, y in poly]
        self.res = resolution_m
        self.x0 = min(xs) - pad_m
        self.y0 = min(ys) - pad_m
        self.nx = int(math.ceil((max(xs) + pad_m - self.x0) / resolution_m))
        self.ny = int(math.ceil((max(ys) + pad_m - self.y0) / resolution_m))

        img = Image.new('1', (self.nx, self.ny), 0)
        draw = ImageDraw.Draw(img)
        for poly in polys:
            draw.polygon([((x - self.x0) / resolution_m, (y - self.y0) / resolution_m) for x, y in poly],
                         fill=1, outline=1)
        import numpy as np
        mask = np.array(img, dtype=bool)   # (ny, nx), row = y index
        if close_m > 0:
            r = max(1, int(round(close_m / resolution_m)))
            yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
            mask = ndimage.binary_closing(mask, structure=(xx ** 2 + yy ** 2) <= r * r)
        self.mask = mask
        inside = ndimage.distance_transform_edt(mask) * resolution_m
        outside = ndimage.distance_transform_edt(~mask) * resolution_m
        self.sdf = (inside - outside).astype('float32')

    def signed_distance(self, x, y):
        """Vectorized: x, y arrays (any matching shape) of map-frame metres ->
        signed distance to the drivable-area edge, same shape. Points off the
        rasterized extent read as far outside (-1e3)."""
        import numpy as np
        ix = np.floor((np.asarray(x) - self.x0) / self.res).astype(int)
        iy = np.floor((np.asarray(y) - self.y0) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        out = np.full(ix.shape, -1e3, dtype='float32')
        out[ok] = self.sdf[iy[ok], ix[ok]]
        return out


def road_margin(sdf_values, half_width: float = EGO_HALF_WIDTH_M):
    """Clearance (metres) between the ego footprint's lateral extent and the
    road edge, given signed_distance() of the ego reference point. <0 means
    part of the vehicle is off the drivable area. Lateral-only disc
    approximation (the rear-axle reference point with a half-width radius) --
    ignores the front/rear overhang corners, which can clip a curb on a
    tight turn without the reference point's disc doing so."""
    return sdf_values - half_width


# Oriented multi-disc footprints. Ego reference point is base_link (rear-axle
# centre); the body spans -rear_overhang (1.1m) to wheel_base+front_overhang
# (3.79m), width 1.896m. Three discs of radius 1.25m centred every 1.63m
# along the heading cover that rectangle (sqrt(0.815^2 + 0.948^2) = 1.25).
# Tracked vehicles are treated as the same-size rectangle centred on their
# reported position; VRUs / unknown / other as single discs.
EGO_DISC_OFFSETS = (-0.285, 1.345, 2.975)
EGO_DISC_R = 1.25
OBJ_DISCS = {   # class group (cfg.OBJECT_CLASS_GROUP) -> (offsets along heading, radius)
    0: ((0.0,), 1.0),                  # unknown
    1: ((-1.63, 0.0, 1.63), 1.25),     # vehicle
    2: ((0.0,), 0.5),                  # vulnerable road user
    3: ((0.0,), 1.0),                  # other
}


def lanelet_heading(x: float, y: float, map_data) -> float:
    """Direction of travel (rad) of the nearest road lanelet's centerline
    segment at (x, y). Used for objects whose heading can't be read from
    their own motion (stationary, or no association) -- lanelets are
    directional, so this also resolves the sign for a moving object."""
    pt = lanelet2.core.BasicPoint2d(float(x), float(y))
    nearest = lg.findNearest(map_data.laneletLayer, pt, 1)
    if not nearest:
        return 0.0
    cl = [(p.x, p.y) for p in nearest[0][1].centerline]
    best, heading = float('inf'), 0.0
    for (ax, ay), (bx, by) in zip(cl[:-1], cl[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-9:
            continue
        u = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        d = math.hypot(ax + u * dx - x, ay + u * dy - y)
        if d < best:
            best, heading = d, math.atan2(dy, dx)
    return heading


def disc_centers(xy, heading, offsets):
    """xy: (..., 2), heading: (...) -> (..., n_discs, 2)."""
    import numpy as np
    off = np.asarray(offsets)
    c, s = np.cos(heading)[..., None], np.sin(heading)[..., None]
    return np.stack([xy[..., 0:1] + off * c, xy[..., 1:2] + off * s], axis=-1)


def object_clearance_v2(ego_xy, ego_heading, obj_xy, obj_heading, obj_class):
    """Minimum disc-to-disc clearance (metres; <0 = footprints overlap)
    between the ego footprint and every object, at every step.

    ego_xy: (K, T, 2), ego_heading: (K, T) -- K counterfactuals, T steps.
    obj_xy: (M, T, 2), obj_heading: (M, T), obj_class: (M,) int groups.
    Returns (K, T) min clearance over objects (inf when M == 0)."""
    import numpy as np
    K, T = ego_xy.shape[:2]
    if len(obj_class) == 0:
        return np.full((K, T), np.inf)
    ego_c = disc_centers(ego_xy, ego_heading, EGO_DISC_OFFSETS)          # (K, T, 3, 2)
    out = np.full((K, T), np.inf)
    for m in range(len(obj_class)):
        offs, r = OBJ_DISCS.get(int(obj_class[m]), OBJ_DISCS[3])
        oc = disc_centers(obj_xy[m], obj_heading[m], offs)               # (T, n, 2)
        d = np.linalg.norm(ego_c[:, :, :, None, :] - oc[None, :, None, :, :], axis=-1)  # (K,T,3,n)
        out = np.minimum(out, d.min(axis=(2, 3)) - EGO_DISC_R - r)
    return out
