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
        'traffic_light_detected':   0 or 1,   # graph node: upcoming lane has TL (map)
        'traffic_light_state':      float,    # perceived color × confidence [0, 1]
        'traffic_light_discrepancy': 0 or 1,  # map expects a TL, perception found none
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
from .State_Estimator.GraphBuilder import GraphBuilder

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


def _scale_position(point: Point, x_min, x_max, y_min, y_max) -> List[float]:
    """Graph-frame-relative [0,1] position — used ONLY internally, to look
    up the nearest graph node for traffic_light_detected/has_adjacent_lane
    (whose coordinates come from the same per-window _scale_graph bounds).
    NOT used for the 'position' feature/target — see _scale_position_relative
    below for why."""
    return [
        _clamp01((point.x - x_min) / (x_max - x_min)),
        _clamp01((point.y - y_min) / (y_max - y_min)),
    ]


def _scale_position_relative(point: Point, ref_x: float, ref_y: float, max_range: float) -> List[float]:
    """
    The 'position' feature/target: real-metre displacement from a FIXED
    reference point (this window's last-observed/'current' frame), scaled to
    [-1, 1] by a fixed constant (cfg.POSITION_DISPLACEMENT_RANGE_M) — not the
    window's own bounding box.

    Fixed 2026-08-02: the previous window-bbox-relative [0,1] scaling
    (_scale_position, still used above for graph lookups) gave the model no
    way to know the metric scale of its own position target — two windows
    with identical SCALED position sequences could correspond to very
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


def _traffic_light_state(traffic_lights: list) -> float:
    """Most restrictive visible traffic light color, normalized to [0,1] and
    weighted by confidence. Confidence matters here (not just color): a fault
    that degrades confidence but leaves color untouched (e.g. tl_confidence)
    must not look identical to an undamaged reading of the same color — a
    plain color lookup would be blind to that fault entirely."""
    if not traffic_lights:
        return 0.0
    return max(_LIGHT_COLOR_VALUE.get(tl['color'], 0.0) * tl['confidence'] for tl in traffic_lights)


def _traffic_light_discrepancy(detected: int, traffic_lights: list) -> int:
    """1 if the HD map says a traffic light should be in range right now but
    perception reports nothing usable — complete signal loss (blackout),
    all-UNKNOWN classification, or any other detection failure collapse to
    the same empty `traffic_lights` list, so this doesn't distinguish which;
    it's the explicit map-vs-perception mismatch signal. Without it, the
    model has to infer the same relationship implicitly from
    traffic_light_detected and traffic_light_state alone — this makes it a
    first-class, directly observable feature instead."""
    return int(detected == 1 and not traffic_lights)


def _traffic_light_detected(G, ego_pos_scaled) -> int:
    closest = min(
        G.nodes(data=True),
        key=lambda n: (n[1]['x'] - ego_pos_scaled[0])**2 + (n[1]['y'] - ego_pos_scaled[1])**2
    )
    return int(closest[1].get('traffic_light_detection_node', 0))


# ── Lanelet adjacency ──────────────────────────────────────────────────────

class LaneletAdjacencyChecker:
    """
    Precomputes has_adjacent_lane for all lanelets in the map using the
    lanelet2 routing graph.  Query is O(n_lanelets) at init, O(n) per frame
    where n is number of lanelets (979 for nishishinjuku ≈ fast enough).

    Coordinate frame: this map is loaded via MGRSProjector (MapProcessor.py),
    the same bag/MGRS frame ego and object positions from the rosbag are
    already in — query with RAW positions, never through
    Point.convert_coordinate_frame (see _process_frame's fix comment: that
    conversion targets an incompatible, now-unused frame).
    """

    def __init__(self, map_data):
        self._centroids: List[Tuple[float, float, int]] = []  # (x, y, ll_id)
        self._adjacency: dict = {}                            # ll_id -> bool
        self._available = False
        self._build(map_data)

    def _build(self, map_data):
        try:
            import lanelet2
            traffic_rules = lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                lanelet2.traffic_rules.Participants.Vehicle,
            )
            graph = lanelet2.routing.RoutingGraph(map_data, traffic_rules)
            for ll in map_data.laneletLayer:
                adj_l = graph.adjacentLeft(ll)
                adj_r = graph.adjacentRight(ll)
                self._adjacency[ll.id] = (adj_l is not None) or (adj_r is not None)
                cl = ll.centerline
                if len(cl) > 0:
                    mid = cl[len(cl) // 2]
                    self._centroids.append((mid.x, mid.y, ll.id))
            self._available = True
        except Exception as exc:
            print(f"  [seq_builder] WARNING: has_adjacent_lane disabled ({exc})")

    def query(self, ref_x: float, ref_y: float) -> float:
        if not self._available or not self._centroids:
            return 0.0
        _, _, ll_id = min(
            self._centroids,
            key=lambda c: (c[0] - ref_x) ** 2 + (c[1] - ref_y) ** 2,
        )
        return 1.0 if self._adjacency.get(ll_id, False) else 0.0


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

def _process_frame(
    frame: dict,
    G,
    x_min, x_max, y_min, y_max,
    ref_x: float, ref_y: float,
    adj_checker: Optional['LaneletAdjacencyChecker'] = None,
) -> dict:
    """Convert one raw frame dict → scaled feature dict.

    ref_x, ref_y: this window's reference point (its last-observed/'current'
    frame's raw position) — see _scale_position_relative. Same reference for
    every frame in a window (past and future both), so past positions read
    as "how far was I before now" and future positions as "how far will I be
    after now", both anchored to the same present moment.
    """
    ego = frame['ego']

    # Raw bag/MGRS-frame position (NOT Point.convert_coordinate_frame — see
    # this file's git history, 2026-08-02, for that fix). The map
    # (MapProcessor.py) and this pipeline's graph (GraphBuilder, built
    # directly from the map's own lanelet points) are both in the bag frame
    # ego positions from the rosbag are already in.
    ego_point = Point(ego['position']['x'], ego['position']['y'])

    # Graph-frame-scaled position — ONLY for the nearest-graph-node lookups
    # below (traffic_light_detected/has_adjacent_lane compare against graph
    # node coordinates, which live in this same per-window [0,1] frame). NOT
    # the 'position' feature/target — see _scale_position_relative's
    # docstring for why conflating the two made the model's own position
    # target's units inconsistent across training examples.
    ego_pos_graph_scaled = _scale_position(ego_point, x_min, x_max, y_min, y_max)
    position_out = _scale_position_relative(ego_point, ref_x, ref_y, cfg.POSITION_DISPLACEMENT_RANGE_M)

    unc = ego['position_uncertainty']
    tl_list      = frame.get('traffic_lights', [])
    tl_detected  = _traffic_light_detected(G, ego_pos_graph_scaled)
    obj_feats, obj_mask = _build_object_set(ego['position'], frame['objects'])

    return {
        'position':                 position_out,
        'velocity':                 _scale_velocity(ego['velocity']),
        'steering':                 _scale_steering(ego['steering']),
        'acceleration':             _scale_acceleration(ego['acceleration']),
        'traffic_light_detected':   tl_detected,
        'traffic_light_state':      _traffic_light_state(tl_list),
        'traffic_light_discrepancy': _traffic_light_discrepancy(tl_detected, tl_list),
        'has_adjacent_lane':        adj_checker.query(ego_point.x, ego_point.y) if adj_checker else 0.0,
        'uncertainty':              _scale_uncertainty(unc['x_var'], unc['y_var']),
        'objects_set':              obj_feats,
        'objects_mask':             obj_mask,
    }


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
        self.graph_builder = GraphBuilder(
            map_data              = map_data,
            route                 = route,
            min_dist_between_node = cfg.MIN_DIST_BETWEEN_NODES,
            connection_threshold  = cfg.CONNECTION_THRESHOLD,
            max_nodes             = cfg.MAX_GRAPH_NODES,
            min_nodes             = cfg.MIN_GRAPH_NODES,
        )
        self._adj_checker = LaneletAdjacencyChecker(map_data)

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

        sequences   = []
        n_windows   = 0
        n_rebuilds  = 0

        # Graph cache
        G_cached     = None
        bounds_cache = None
        last_init_pt = None
        last_last_pt = None

        for i in range(0, n - total + 1, cfg.STRIDE):
            window = frames[i : i + total]

            if filter_mrm and any(f.get('mrm_active', False) for f in window):
                continue

            init_frame = window[0]['ego']
            last_frame  = window[-1]['ego']

            # Raw bag-frame Points (see _process_frame's fix comment) — this
            # center_position feeds GraphBuilder._get_sorted_lanelets, which
            # measures distance against raw (bag-frame) lanelet centerline
            # points, so it must be in the same frame or every lanelet
            # "distance" is meaningless.
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

            x_min, x_max, y_min, y_max = bounds_cache

            # Deep-copy the cached graph so each sequence owns an independent copy
            # (needed because pickle serialises the graph and we don't want aliasing)
            import copy
            G_seq = copy.deepcopy(G_cached)

            # Reference point for the 'position' feature (see
            # _scale_position_relative): this window's last-observed/'current'
            # frame — the boundary between past and future — so every frame's
            # position reads as "how far from now", in real metres, at a
            # fixed scale shared by every window (not that window's own bbox).
            current_frame = window[cfg.INPUT_SEQ_LEN - 1]['ego']
            ref_x, ref_y = current_frame['position']['x'], current_frame['position']['y']

            past_seq   = [_process_frame(f, G_cached, x_min, x_max, y_min, y_max,
                                         ref_x, ref_y, self._adj_checker)
                          for f in window[:cfg.INPUT_SEQ_LEN]]
            future_seq = [_process_frame(f, G_cached, x_min, x_max, y_min, y_max,
                                         ref_x, ref_y, self._adj_checker)
                          for f in window[cfg.INPUT_SEQ_LEN:]]

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

        if verbose:
            print(f"  [seq_builder] {n_windows} sequences, {n_rebuilds} graph builds")

        return sequences
