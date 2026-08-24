"""
Layer 2 (consequence estimation) safety-margin primitives — see
docs/research_notes/open_world_safety_reframe_2026-08-20.md §4 and the
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
