"""
SequenceBuilder: adapts the T-ITS SequenceProcessor for our pipeline.

Key differences from the reference SequenceProcessor:
  1. Accepts our list-of-frame-dicts from bag_reader (no timestamp keys)
  2. Adds position_uncertainty (x_var, y_var) to every timestep's feature dict
  3. Reads route from the rosbag (LaneletRoute message) instead of route.json
  4. Uses GraphBuilder and MapProcessor (in Data_Curator/ and State_Estimator/, same
     package/class names as the original T-ITS codebase — copied in, not imported live)

The output sequence format is identical to the reference:
    {
        'past':   [processed_timestep, ...],   # INPUT_SEQ_LEN items
        'future': [processed_timestep, ...],   # OUTPUT_SEQ_LEN items
        'graph':  networkx.Graph,
        'graph_bounds': [x_min, x_max, y_min, y_max],
    }

Each processed_timestep (12 flat features + object set):
    {
        'position':                 [scaled_x, scaled_y],
        'velocity':                 [scaled_vx, scaled_vy],
        'steering':                 float,
        'acceleration':             float,
        'traffic_light_color':      float,    # perceived color of the most-restrictive
                                                # reported element, [0=none, 1=red]
        'traffic_light_confidence': float,    # that SAME element's confidence [0, 1]
        'traffic_light_discrepancy': 0 or 1,  # map (real TL graph node) expects a TL,
                                                # perception found none
        'has_adjacent_lane':        0.0/1.0,  # lanelet2 routing graph adjacency
        'uncertainty':              [scaled_x_var, scaled_y_var],
        'objects_set':              (K, OBJECT_FEATURE_DIM) array, per-object
                                     [rel_x, rel_y, speed, *class_onehot] — NOT
                                     collapsed to a nearest-object scalar (see
                                     docs/stgat_pipeline_plan.md Stage 0's
                                     entity-collapse warning; model.py's
                                     ObjectSetEncoder does the pooling, not
                                     this preprocessing step).
        'objects_mask':             (K,) array, 1.0 for real objects, 0.0 padding.
    }
"""

import sys
import os
import math
from typing import List, Tuple, Optional

import numpy as np

from .Data_Curator.Point import Point
from .State_Estimator.GraphBuilder import GraphBuilder, regulatory_element_position, _reg_attr

from . import config as cfg


def _load_map_processor():
    """Lazy import: MapProcessor requires lanelet2, only available with Autoware workspace sourced."""
    from .State_Estimator.MapProcessor import MapProcessor
    return MapProcessor(cfg.MAP_FILE)


# ── Route extraction ───────────────────────────────────────────────────────

def extract_route_from_bag(bag_dir: str) -> List[int]:
    """
    Read the first LaneletRoute message from the bag and return its lanelet IDs.
    Falls back to [] if the topic is absent (graph nodes will have path_node=0).
    """
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message

    try:
        from autoware_planning_msgs.msg import LaneletRoute
    except ImportError:
        return []

    storage_options   = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/planning/mission_planning/route':
            msg = deserialize_message(data, LaneletRoute)
            return [seg.preferred_primitive.id for seg in msg.segments]
    return []


# ── Feature scaling helpers ────────────────────────────────────────────────

def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _scale_velocity(vel_dict: dict) -> List[float]:
    vx_min, vx_max = cfg.VELOCITY_X_RANGE
    vy_min, vy_max = cfg.VELOCITY_Y_RANGE
    return [
        _clamp01((vel_dict['longitudinal'] - vx_min) / (vx_max - vx_min)),
        _clamp01((vel_dict['lateral']      - vy_min) / (vy_max - vy_min)),
    ]


def _scale_steering(steering: float) -> float:
    s_min, s_max = cfg.STEERING_RANGE
    return _clamp01((steering - s_min) / (s_max - s_min))


def _scale_acceleration(accel: float) -> float:
    a_min, a_max = cfg.ACCEL_RANGE
    return _clamp01((accel - a_min) / (a_max - a_min))


def _scale_uncertainty(x_var: float, y_var: float) -> List[float]:
    """
    Scale EKF covariance values to [0, 1].
    UNCERTAINTY_CAP (0.5 m²) maps to 1.0.
    Nominal (0.003 m²) → 0.006.
    """
    cap = cfg.UNCERTAINTY_CAP
    return [
        _clamp01(x_var / cap),
        _clamp01(y_var / cap),
    ]


def _scale_position_relative(point: Point, ref_x: float, ref_y: float, max_range: float) -> List[float]:
    """
    The 'position' feature/target: real-metre displacement from a FIXED
    reference point (this window's last-observed/'current' frame), scaled to
    [-1, 1] by a fixed constant (cfg.POSITION_DISPLACEMENT_RANGE_M) — not the
    window's own bounding box.

    Fixed 2026-08-02: the previous window-bbox-relative [0,1] scaling (a
    _scale_position helper, removed 2026-08-05 once traffic_light_detected —
    its last remaining caller — was retired as an output feature) gave the
    model no way to know the metric scale of its own position target — two
    windows with identical SCALED position sequences could correspond to very
    different real displacements depending on that window's own (never
    given to the model) bbox extent. Confirmed via a baseline check: the
    trained model's 1-step position error (2.5m) was ~30x worse than a
    trivial constant-velocity extrapolation baseline (0.08m) — not possible
    if the scale were consistent and learnable. Real-metre displacement from
    a fixed reference point has the same units in every training example, so
    the same network weights can learn one generalizable dynamics-to-
    displacement mapping instead of an unobservable per-window rescaling.
    """
    dx = point.x - ref_x
    dy = point.y - ref_y
    return [
        _clamp(dx / max_range, -1.0, 1.0),
        _clamp(dy / max_range, -1.0, 1.0),
    ]


def _object_class_onehot(label: int) -> List[float]:
    group = cfg.OBJECT_CLASS_GROUP.get(label, cfg.OBJECT_CLASS_GROUPS - 1)
    onehot = [0.0] * cfg.OBJECT_CLASS_GROUPS
    onehot[group] = 1.0
    return onehot


def _build_object_set(ego_pos_raw: dict, objects_raw: list) -> Tuple['np.ndarray', 'np.ndarray']:
    """
    Per-object feature set for this timestep, NOT collapsed to a
    nearest-object scalar (see docs/stgat_pipeline_plan.md Stage 0's
    entity-collapse warning — the same pattern already found and fixed once
    for traffic lights, §1.11). Padded/masked to cfg.MAX_TRACKED_OBJECTS so
    every timestep has a fixed-shape (K, F) tensor regardless of how many
    objects are actually tracked; model.py's ObjectSetEncoder (a
    permutation-invariant pooling layer) decides what to attend to across
    them, not this preprocessing step.

    Per-object features (OBJECT_FEATURE_DIM-dim): relative position (dx, dy —
    both already in the same bag/map frame as ego, no transform needed, see
    _process_frame's coordinate-frame fix), speed (frame-invariant magnitude
    — deliberately NOT an ego-relative velocity vector, since TrackedObjects'
    twist frame and VelocityReport's longitudinal/lateral frame aren't
    confirmed to match, and silently getting that subtraction wrong would
    repeat this session's position-frame bug in a new, harder-to-notice
    place), and a 4-way classification one-hot (vehicle / vulnerable road
    user / unknown / other).

    Objects beyond K are dropped by distance (nearest K kept) — this bounds
    the tensor size, it does not collapse the K that remain into one value.
    """
    ex, ey = ego_pos_raw['x'], ego_pos_raw['y']
    k = cfg.MAX_TRACKED_OBJECTS
    feats = np.zeros((k, cfg.OBJECT_FEATURE_DIM), dtype=np.float32)
    mask  = np.zeros((k,), dtype=np.float32)

    with_dist = sorted(
        objects_raw,
        key=lambda o: (o['position']['x'] - ex) ** 2 + (o['position']['y'] - ey) ** 2,
    )[:k]

    rx_min, rx_max = cfg.OBJECT_REL_POS_RANGE
    sp_min, sp_max = cfg.OBJECT_SPEED_RANGE
    for i, o in enumerate(with_dist):
        dx = o['position']['x'] - ex
        dy = o['position']['y'] - ey
        speed = math.hypot(o['velocity']['x'], o['velocity']['y'])
        feats[i, 0] = _clamp01((dx - rx_min) / (rx_max - rx_min))
        feats[i, 1] = _clamp01((dy - rx_min) / (rx_max - rx_min))
        feats[i, 2] = _clamp01((speed - sp_min) / (sp_max - sp_min))
        feats[i, 3:3 + cfg.OBJECT_CLASS_GROUPS] = _object_class_onehot(o['classification'])
        mask[i] = 1.0

    return feats, mask


# color from bag_reader: 1=RED, 2=AMBER, 3=GREEN
_LIGHT_COLOR_VALUE = {1: 1.0, 2: 0.67, 3: 0.33}


def _traffic_light_reading(traffic_lights: list) -> Tuple[float, float]:
    """(color, confidence) of the single most-restrictive reported element,
    kept self-consistent — both values describe the SAME physical reading,
    not independently maxed across elements (redesigned 2026-08-05,
    replacing the old traffic_light_state's color*confidence collapse — see
    docs/research_notes/ablation_study_2026.md). Reporting color and
    confidence as two separate features lets the model distinguish a pure
    confidence-degradation fault (tl_confidence — fault_injector.py) from a
    color-changing one, which the old multiplied scalar could not: halving
    confidence while leaving color untouched produced the same collapsed
    value as an undamaged reading of a more-permissive color at higher
    confidence — the two faults were indistinguishable downstream.
    (0.0, 0.0) if nothing reported."""
    if not traffic_lights:
        return 0.0, 0.0
    most_restrictive = max(traffic_lights, key=lambda tl: _LIGHT_COLOR_VALUE.get(tl['color'], 0.0))
    return (_LIGHT_COLOR_VALUE.get(most_restrictive['color'], 0.0),
            float(most_restrictive['confidence']))


def _traffic_light_discrepancy(map_expects: int, traffic_lights: list) -> int:
    """1 if the map (via a real traffic-light graph node — see
    GraphBuilder._add_traffic_light_nodes / TrafficLightExpectationChecker
    above) expects a TL to be governing ego's position right now, but
    perception reports nothing usable — complete signal loss (blackout),
    all-UNKNOWN classification, or any other detection failure collapse to
    the same empty `traffic_lights` list, so this doesn't distinguish which;
    it's the explicit map-vs-perception mismatch signal. Without it, the
    model has to infer the same relationship implicitly from map structure
    and the perception-report features alone — this makes it a first-class,
    directly observable feature instead."""
    return int(map_expects == 1 and not traffic_lights)


# ── Lanelet adjacency ──────────────────────────────────────────────────────

class LaneletAdjacencyChecker:
    """
    Precomputes has_adjacent_lane for all lanelets in the map using the
    lanelet2 routing graph. Nearest-lanelet-centroid lookup uses a KD-tree
    (was a linear O(n_lanelets) scan via Python's min()/lambda per call --
    audited 2026-08-05: even after fixing the ~60x per-frame call
    redundancy from STRIDE=1 windowing, this linear scan alone was ~24s of a
    106s trial's total extraction time, see
    docs/research_notes/ablation_study_2026.md).

    Coordinate frame: this map is loaded via MGRSProjector (MapProcessor.py),
    the same bag/MGRS frame ego and object positions from the rosbag are
    already in — query with RAW positions, never through
    Point.convert_coordinate_frame (see _process_frame's fix comment: that
    conversion targets an incompatible, now-unused frame).
    """

    def __init__(self, map_data, routing_graph=None):
        self._centroid_ll_ids: List[int] = []
        self._adjacency: dict = {}                            # ll_id -> bool
        self._tree = None
        self._available = False
        self._build(map_data, routing_graph)

    def _build(self, map_data, routing_graph=None):
        try:
            import lanelet2
            from scipy.spatial import cKDTree

            # Shared, not rebuilt: routing_graph only depends on map_data
            # (invariant across trials), so SequenceBuilder.__init__ builds
            # it once and passes the same object to both this class and
            # GraphBuilder rather than each constructing its own.
            if routing_graph is None:
                traffic_rules = lanelet2.traffic_rules.create(
                    lanelet2.traffic_rules.Locations.Germany,
                    lanelet2.traffic_rules.Participants.Vehicle,
                )
                routing_graph = lanelet2.routing.RoutingGraph(map_data, traffic_rules)

            centroids = []
            for ll in map_data.laneletLayer:
                adj_l = routing_graph.adjacentLeft(ll)
                adj_r = routing_graph.adjacentRight(ll)
                self._adjacency[ll.id] = (adj_l is not None) or (adj_r is not None)
                cl = ll.centerline
                if len(cl) > 0:
                    mid = cl[len(cl) // 2]
                    centroids.append((mid.x, mid.y))
                    self._centroid_ll_ids.append(ll.id)
            if centroids:
                self._tree = cKDTree(np.array(centroids))
            self._available = True
        except Exception as exc:
            print(f"  [seq_builder] WARNING: has_adjacent_lane disabled ({exc})")

    def query(self, ref_x: float, ref_y: float) -> float:
        if not self._available or self._tree is None:
            return 0.0
        _, idx = self._tree.query([ref_x, ref_y])
        ll_id = self._centroid_ll_ids[idx]
        return 1.0 if self._adjacency.get(ll_id, False) else 0.0


class TrafficLightExpectationChecker:
    """
    Precomputes real-world (x, y) positions of every distinct traffic-light
    regulatory element governing a lanelet on the route, and answers "does
    the map expect a TL to be governing this raw position right now" via a
    fixed real-world proximity radius.

    Deliberately works in RAW map coordinates, not a per-window graph's
    [0,1]-normalized frame: GraphBuilder._scale_graph's normalization factor
    depends on that window's own bounding box, so a fixed proximity
    threshold evaluated there would correspond to a different real-world
    distance every time a graph rebuilds — this checker instead uses the
    same raw bag/MGRS frame ego positions are already in (same convention as
    LaneletAdjacencyChecker), so one fixed metres threshold means the same
    thing on every call, independent of which window's graph happens to be
    cached. Reuses GraphBuilder.regulatory_element_position (the same
    computation compute_tl_zones.py/fault_injector.py use to pick which
    real-world light a TL fault targets) so "the light this checker
    considers governing" and "the light GraphBuilder adds a node for" are
    always the same real-world entity, not two independently-derived
    approximations of it.

    Ground-truth LABEL input for traffic_light_discrepancy only — not
    exposed as its own predicted feature (see config.py's FEATURE_SIZES doc
    for why traffic_light_detected was retired as a model output).
    """

    _RADIUS_M = 25.0   # generous stopping-distance proximity — how far out
                        # perception would plausibly still be reporting the
                        # light governing an approach

    def __init__(self, map_data, route):
        self._positions: List[Tuple[float, float]] = []
        seen_reg_ids = set()
        for lid in route:
            ll = map_data.laneletLayer[lid]
            for reg in ll.regulatoryElements:
                if _reg_attr(reg, 'subtype') != 'traffic_light' or reg.id in seen_reg_ids:
                    continue
                pos = regulatory_element_position(reg)
                if pos is None:
                    continue
                seen_reg_ids.add(reg.id)
                self._positions.append(pos)

    def query(self, x: float, y: float) -> int:
        if not self._positions:
            return 0
        nearest_sq = min((px - x) ** 2 + (py - y) ** 2 for px, py in self._positions)
        return int(nearest_sq <= self._RADIUS_M ** 2)


# ── Scale graph ────────────────────────────────────────────────────────────

def _scale_graph(G):
    xs = [d['x'] for _, d in G.nodes(data=True)]
    ys = [d['y'] for _, d in G.nodes(data=True)]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    for node_id, data in G.nodes(data=True):
        G.nodes[node_id]['x'] = _clamp01((data['x'] - x_min) / (x_max - x_min))
        G.nodes[node_id]['y'] = _clamp01((data['y'] - y_min) / (y_max - y_min))
    return G, x_min, x_max, y_min, y_max


# ── Per-timestep processing ────────────────────────────────────────────────

def _process_frame_static(
    frame: dict,
    adj_checker: Optional['LaneletAdjacencyChecker'] = None,
    tl_checker: Optional['TrafficLightExpectationChecker'] = None,
) -> dict:
    """Convert one raw frame dict → scaled feature dict, EXCLUDING 'position'
    (the one feature that depends on which window is currently being built —
    see _with_position below). Every other feature here depends only on the
    frame itself plus two small, per-trial, position-invariant checkers, so
    it can be computed exactly once per unique frame instead of once per
    overlapping window.

    Added 2026-08-05 (see docs/research_notes/ablation_study_2026.md's
    runtime audit): previously this all lived in one _process_frame() called
    once per (window, frame-in-window) pair — at STRIDE=1 with a
    60-frame window, ~60x redundant per unique frame. That redundancy used
    to be harder to remove because traffic_light_detected's map-expectation
    check depended on the per-window graph's [0,1]-scaled coordinates;
    retiring traffic_light_detected as an output feature (see config.py's
    FEATURE_SIZES doc) and moving the map-expectation check to
    TrafficLightExpectationChecker (which works in fixed real-world
    coordinates, not per-window graph-scaled ones) removed that dependency
    entirely, so nothing here needs the graph or window context anymore.
    """
    ego = frame['ego']

    # Raw bag/MGRS-frame position (NOT Point.convert_coordinate_frame — see
    # this file's git history, 2026-08-02, for that fix). The map
    # (MapProcessor.py) and this pipeline's graph (GraphBuilder, built
    # directly from the map's own lanelet points) are both in the bag frame
    # ego positions from the rosbag are already in.
    ego_point = Point(ego['position']['x'], ego['position']['y'])

    unc = ego['position_uncertainty']
    tl_list = frame.get('traffic_lights', [])
    tl_color, tl_confidence = _traffic_light_reading(tl_list)
    map_expects = tl_checker.query(ego_point.x, ego_point.y) if tl_checker else 0
    obj_feats, obj_mask = _build_object_set(ego['position'], frame['objects'])

    return {
        'velocity':                 _scale_velocity(ego['velocity']),
        'steering':                 _scale_steering(ego['steering']),
        'acceleration':             _scale_acceleration(ego['acceleration']),
        'traffic_light_color':      tl_color,
        'traffic_light_confidence': tl_confidence,
        'traffic_light_discrepancy': _traffic_light_discrepancy(map_expects, tl_list),
        'has_adjacent_lane':        adj_checker.query(ego_point.x, ego_point.y) if adj_checker else 0.0,
        'uncertainty':              _scale_uncertainty(unc['x_var'], unc['y_var']),
        'objects_set':              obj_feats,
        'objects_mask':             obj_mask,
    }


def _with_position(static_feats: dict, frame: dict, ref_x: float, ref_y: float) -> dict:
    """Add the one genuinely window-dependent feature (position, relative to
    this window's reference frame) to an already-computed static feature
    dict. Cheap arithmetic, no lookups — safe to redo per window even though
    everything else in static_feats is cached.

    ref_x, ref_y: this window's reference point (its last-observed/'current'
    frame's raw position) — see _scale_position_relative. Same reference for
    every frame in a window (past and future both), so past positions read
    as "how far was I before now" and future positions as "how far will I be
    after now", both anchored to the same present moment.
    """
    ego_point = Point(frame['ego']['position']['x'], frame['ego']['position']['y'])
    out = dict(static_feats)
    out['position'] = _scale_position_relative(ego_point, ref_x, ref_y, cfg.POSITION_DISPLACEMENT_RANGE_M)
    return out


# ── Main builder ───────────────────────────────────────────────────────────

class SequenceBuilder:
    """
    Builds sliding-window sequences from a list of 10Hz frame dicts.

    Usage:
        builder = SequenceBuilder.from_bag(bag_dir)
        sequences = builder.build(frames)
    """

    def __init__(self, map_data, route: List[int]):
        self.map_data = map_data
        self.route = route

        # Built ONCE and shared between GraphBuilder and LaneletAdjacencyChecker
        # (added 2026-08-05) — routing_graph only depends on map_data, which
        # never changes across trials, so constructing it twice per trial
        # (each class used to build its own) was pure waste. See
        # docs/research_notes/ablation_study_2026.md.
        routing_graph = None
        try:
            import lanelet2
            traffic_rules = lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                lanelet2.traffic_rules.Participants.Vehicle,
            )
            routing_graph = lanelet2.routing.RoutingGraph(map_data, traffic_rules)
        except Exception as exc:
            print(f"  [seq_builder] WARNING: lanelet2 routing graph unavailable "
                  f"({exc}) — topology-aware graph connectivity and "
                  f"has_adjacent_lane both disabled.")

        # Exposed so callers that swap route/graph_builder per trial without
        # re-running __init__ (run_pipeline.py's shared_builder pattern) can
        # reuse the same routing graph instead of rebuilding it, and must
        # also rebuild _tl_checker per trial the same way — see
        # run_pipeline.py's process_dataset() for why both are route-
        # dependent and graph_builder-swap-only would silently leave
        # _tl_checker (and therefore every window's traffic_light_discrepancy
        # label) built from an empty route.
        self.routing_graph = routing_graph

        self.graph_builder = GraphBuilder(
            map_data              = map_data,
            route                 = route,
            min_dist_between_node = cfg.MIN_DIST_BETWEEN_NODES,
            connection_threshold  = cfg.CONNECTION_THRESHOLD,
            max_nodes             = cfg.MAX_GRAPH_NODES,
            radius_m              = cfg.GRAPH_RADIUS_M,
            routing_graph         = routing_graph,
        )
        self._adj_checker = LaneletAdjacencyChecker(map_data, routing_graph=routing_graph)
        self._tl_checker  = TrafficLightExpectationChecker(map_data, route)

    @classmethod
    def from_bag(cls, bag_dir: str) -> 'SequenceBuilder':
        """Load map data (expensive) and extract route from the bag."""
        map_processor = _load_map_processor()
        route = extract_route_from_bag(bag_dir)
        return cls(map_processor.map_data, route)

    # Rebuild the graph only when the window centre moves more than this
    # (in local map coordinates, metres). At 10 Hz with ~3 m/s average speed,
    # one stride = ~0.3 m — so we rebuild roughly every 10 strides.
    _GRAPH_CACHE_DIST = 5.0   # metres

    # Nominal frame cadence is 10Hz (0.1s); allow some jitter (observed p99
    # inter-frame gap ~120ms on the master-clock topic) but reject anything
    # that indicates a real hole in the frame stream from a dropped/stale
    # topic — see build()'s temporal-contiguity gate comment for the finding
    # this guards against (up to 31s gaps observed, silently spliced before
    # this fix).
    _MAX_FRAME_GAP_SEC = 0.20

    def build(self, frames: List[dict], verbose: bool = False,
              filter_mrm: bool = False) -> List[dict]:
        """
        Create sliding-window sequences from the frame list.

        Graph rebuilds are cached: the same graph is reused for consecutive
        windows whose centre has moved less than _GRAPH_CACHE_DIST metres.
        This reduces graph builds from O(frames) to O(frames / 10) for typical
        urban driving speeds (≈3 m/s at 10 Hz).

        filter_mrm: if True, skip any window that contains a frame where MRM
            was non-NORMAL. Use this when building calibration sequences from
            nominal bags so that control-gate transients don't inflate the
            nominal residual distribution. Leave False for inference so the
            CUSUM signal sees MRM co-occurring with obstacle events.

        Returns list of sequence dicts with 'past', 'future', 'graph', 'graph_bounds'.
        """
        n = len(frames)
        total = cfg.INPUT_SEQ_LEN + cfg.OUTPUT_SEQ_LEN

        if n < total:
            if verbose:
                print(f"  [seq_builder] too few frames ({n} < {total}), skipping")
            return []

        # Temporal-contiguity gate (added 2026-08-05 — found via direct
        # inspection that bag_reader.py's per-topic staleness check drops
        # individual frames when a topic goes stale (correct), but the
        # dropped frames leave a HOLE in the returned list — this loop used
        # to slice `frames[i:i+total]` as if array-adjacency implied ~0.1s
        # real-time adjacency. Confirmed concretely on one ordinary nominal
        # trial: gaps up to 31.4s between array-adjacent frames (e.g. a
        # control_cmd outage), meaning ~12% of that trial's windows silently
        # spliced together frames from tens of seconds apart and fed the
        # model a "3s past / 3s future" window that was actually spanning a
        # large, arbitrary real-time jump — corrupting training data AND
        # every residual/discriminability/lead-time analysis built on top of
        # it, silently. Window is now rejected outright if any consecutive
        # frame pair inside it exceeds _MAX_FRAME_GAP_SEC (an intra-window
        # check, independent of and stricter than bag_reader's own
        # per-topic MAX_STALENESS_SEC=0.3, which only bounds one topic's
        # staleness relative to the master clock, not frame-to-frame gaps
        # in the resulting sequence).
        frame_gaps = [frames[j + 1]['t_sim'] - frames[j]['t_sim'] for j in range(n - 1)]

        # Static (window-independent) per-frame features, computed ONCE per
        # unique frame instead of once per overlapping window (added
        # 2026-08-05 — see docs/research_notes/ablation_study_2026.md's
        # runtime audit: at STRIDE=1 with a 60-frame window, the old
        # per-(window, frame) call pattern recomputed every frame's features
        # ~60x redundantly; profiled at ~31s of a 106s trial before this
        # fix, mostly LaneletAdjacencyChecker.query()).
        static_feats = [_process_frame_static(f, self._adj_checker, self._tl_checker)
                        for f in frames]

        sequences   = []
        n_windows   = 0
        n_rebuilds  = 0
        n_gap_rejected = 0

        # Graph cache
        G_cached     = None
        G_seq_cached = None   # deep copy of G_cached, reused for every window
                              # sharing this rebuild (see below)
        bounds_cache = None
        last_init_pt = None
        last_last_pt = None

        for i in range(0, n - total + 1, cfg.STRIDE):
            window = frames[i : i + total]

            if max(frame_gaps[i : i + total - 1]) > self._MAX_FRAME_GAP_SEC:
                n_gap_rejected += 1
                continue

            if filter_mrm and any(f.get('mrm_active', False) for f in window):
                continue

            init_frame = window[0]['ego']
            last_frame  = window[-1]['ego']

            # Raw bag-frame Points (see _process_frame_static's fix comment) —
            # this center_position feeds GraphBuilder._get_sorted_lanelets,
            # which measures distance against raw (bag-frame) lanelet
            # centerline points, so it must be in the same frame or every
            # lanelet "distance" is meaningless.
            init_pt = Point(init_frame['position']['x'], init_frame['position']['y'])
            last_pt = Point(last_frame['position']['x'], last_frame['position']['y'])

            # Rebuild graph only if window centre shifted significantly
            rebuild = (
                G_cached is None
                or math.sqrt(
                    (init_pt.x - last_init_pt.x) ** 2 +
                    (init_pt.y - last_init_pt.y) ** 2
                ) > self._GRAPH_CACHE_DIST
            )

            if rebuild:
                import copy
                G_raw = self.graph_builder.create_expanded_graph(init_pt, last_pt)
                G_cached, x_min, x_max, y_min, y_max = _scale_graph(G_raw)
                bounds_cache  = (x_min, x_max, y_min, y_max)
                last_init_pt  = init_pt
                last_last_pt  = last_pt
                n_rebuilds   += 1
                # Deep-copy once per rebuild, not once per window (added
                # 2026-08-05 — was ~6s of a 106s trial). Every window sharing
                # this rebuild period gets the SAME graph object reference;
                # safe because nothing downstream (dataset.py's
                # _build_graph_tensors, all analysis/plotting scripts) ever
                # mutates a sequence's 'graph' in place, only reads it.
                G_seq_cached = copy.deepcopy(G_cached)

            x_min, x_max, y_min, y_max = bounds_cache
            G_seq = G_seq_cached

            # Reference point for the 'position' feature (see
            # _scale_position_relative): this window's last-observed/'current'
            # frame — the boundary between past and future — so every frame's
            # position reads as "how far from now", in real metres, at a
            # fixed scale shared by every window (not that window's own bbox).
            current_frame = window[cfg.INPUT_SEQ_LEN - 1]['ego']
            ref_x, ref_y = current_frame['position']['x'], current_frame['position']['y']

            past_seq   = [_with_position(static_feats[j], frames[j], ref_x, ref_y)
                          for j in range(i, i + cfg.INPUT_SEQ_LEN)]
            future_seq = [_with_position(static_feats[j], frames[j], ref_x, ref_y)
                          for j in range(i + cfg.INPUT_SEQ_LEN, i + total)]

            sequences.append({
                'past':         past_seq,
                'future':       future_seq,
                'graph':        G_seq,
                'graph_bounds': [x_min, x_max, y_min, y_max],
                # Reference point (raw bag-frame metres) the 'position'
                # feature is relative to — added 2026-08-02 alongside the
                # position-representation fix, so any consumer that needs
                # real map coordinates back (plotting, residual analysis)
                # can reconstruct them without separately tracking raw
                # frame indices: real = ref + scaled * POSITION_DISPLACEMENT_RANGE_M.
                'position_ref': [ref_x, ref_y],
            })
            n_windows += 1

        if verbose or n_gap_rejected:
            print(f"  [seq_builder] {n_windows} sequences, {n_rebuilds} graph builds, "
                  f"{n_gap_rejected} windows rejected for a >{self._MAX_FRAME_GAP_SEC}s frame gap")

        return sequences
