"""
SequenceBuilder: adapts the T-ITS SequenceProcessor for our pipeline.

Key differences from the reference SequenceProcessor:
  1. Accepts our list-of-frame-dicts from bag_reader (no timestamp keys)
  2. Adds position_uncertainty (x_var, y_var) to every timestep's feature dict
  3. Reads route from the rosbag (LaneletRoute message) instead of route.json
  4. Uses GraphBuilder and MapProcessor vendored from the reference codebase (see vendor/)

The output sequence format is identical to the reference:
    {
        'past':   [processed_timestep, ...],   # INPUT_SEQ_LEN items
        'future': [processed_timestep, ...],   # OUTPUT_SEQ_LEN items
        'graph':  networkx.Graph,
        'graph_bounds': [x_min, x_max, y_min, y_max],
    }

Each processed_timestep (13 features + objects):
    {
        'position':                 [scaled_x, scaled_y],
        'velocity':                 [scaled_vx, scaled_vy],
        'steering':                 float,
        'acceleration':             float,
        'object_distance':          float,
        'traffic_light_detected':   0 or 1,   # graph node: upcoming lane has TL
        'traffic_light_state':      float,    # perception color [0, 0.33, 0.67, 1.0]
        'closest_object_velocity':  float,    # |velocity| nearest object, scaled
        'has_adjacent_lane':        0.0/1.0,  # lanelet2 routing graph adjacency
        'uncertainty':              [scaled_x_var, scaled_y_var],
        'objects':                  [...],    # scaled, kept for compatibility
    }
"""

import sys
import os
import math
from typing import List, Tuple, Optional

from .vendor.point import Point
from .vendor.graph_builder import GraphBuilder

from . import config as cfg


def _load_map_processor():
    """Lazy import: MapProcessor requires lanelet2, only available with Autoware workspace sourced."""
    from .vendor.map_processor import MapProcessor
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
    return [
        _clamp01((point.x - x_min) / (x_max - x_min)),
        _clamp01((point.y - y_min) / (y_max - y_min)),
    ]


def _scale_object(obj_dict: dict, x_min, x_max, y_min, y_max) -> dict:
    pos = Point.convert_coordinate_frame(
        obj_dict['position']['x'], obj_dict['position']['y'],
        cfg.REFERENCE_POINTS
    )
    return {
        'position': _scale_position(pos, x_min, x_max, y_min, y_max),
        'velocity': [
            _clamp01((obj_dict['velocity']['x'] - cfg.VELOCITY_X_RANGE[0]) /
                     (cfg.VELOCITY_X_RANGE[1] - cfg.VELOCITY_X_RANGE[0])),
            _clamp01((obj_dict['velocity']['y'] - cfg.VELOCITY_Y_RANGE[0]) /
                     (cfg.VELOCITY_Y_RANGE[1] - cfg.VELOCITY_Y_RANGE[0])),
        ],
        'classification': obj_dict['classification'],
    }


def _closest_object_distance(ego_pos_scaled, objects_scaled) -> float:
    if not objects_scaled:
        return 1.0
    min_d = float('inf')
    for obj in objects_scaled:
        op = obj.get('position', [])
        if len(op) >= 2:
            d = math.sqrt((ego_pos_scaled[0] - op[0])**2 + (ego_pos_scaled[1] - op[1])**2)
            if d < min_d:
                min_d = d
    return min_d if min_d != float('inf') else 1.0


# color from bag_reader: 1=RED, 2=AMBER, 3=GREEN
_LIGHT_COLOR_VALUE = {1: 1.0, 2: 0.67, 3: 0.33}


def _traffic_light_state(traffic_lights: list) -> float:
    """Most restrictive visible traffic light color, normalized to [0,1]."""
    if not traffic_lights:
        return 0.0
    return max(_LIGHT_COLOR_VALUE.get(tl['color'], 0.0) for tl in traffic_lights)


def _closest_object_velocity(ego_pos_raw: dict, objects_raw: list) -> float:
    """Scaled |longitudinal velocity| of the nearest tracked object."""
    if not objects_raw:
        return 0.0
    ex, ey = ego_pos_raw['x'], ego_pos_raw['y']
    nearest = min(
        objects_raw,
        key=lambda o: (o['position']['x'] - ex)**2 + (o['position']['y'] - ey)**2,
    )
    vx_min, vx_max = cfg.VELOCITY_X_RANGE
    return _clamp01((abs(nearest['velocity']['x']) - vx_min) / (vx_max - vx_min))


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

    Coordinate frame: lanelet2 LocalCartesian ≡ the reference frame produced
    by Point.convert_coordinate_frame — same origin, nearly identity scale.
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
    adj_checker: Optional['LaneletAdjacencyChecker'] = None,
) -> dict:
    """Convert one raw frame dict → scaled feature dict."""
    ego = frame['ego']

    ego_point = Point.convert_coordinate_frame(
        ego['position']['x'], ego['position']['y'],
        cfg.REFERENCE_POINTS
    )
    ego_pos_scaled = _scale_position(ego_point, x_min, x_max, y_min, y_max)

    objects_scaled = [_scale_object(o, x_min, x_max, y_min, y_max) for o in frame['objects']]
    unc = ego['position_uncertainty']

    return {
        'position':                 ego_pos_scaled,
        'velocity':                 _scale_velocity(ego['velocity']),
        'steering':                 _scale_steering(ego['steering']),
        'acceleration':             _scale_acceleration(ego['acceleration']),
        'object_distance':          _closest_object_distance(ego_pos_scaled, objects_scaled),
        'traffic_light_detected':   _traffic_light_detected(G, ego_pos_scaled),
        'traffic_light_state':      _traffic_light_state(frame.get('traffic_lights', [])),
        'closest_object_velocity':  _closest_object_velocity(ego['position'], frame['objects']),
        'has_adjacent_lane':        adj_checker.query(ego_point.x, ego_point.y) if adj_checker else 0.0,
        'uncertainty':              _scale_uncertainty(unc['x_var'], unc['y_var']),
        'objects':                  objects_scaled,
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

            init_pt = Point.convert_coordinate_frame(
                init_frame['position']['x'], init_frame['position']['y'],
                cfg.REFERENCE_POINTS
            )
            last_pt = Point.convert_coordinate_frame(
                last_frame['position']['x'], last_frame['position']['y'],
                cfg.REFERENCE_POINTS
            )

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

            past_seq   = [_process_frame(f, G_cached, x_min, x_max, y_min, y_max,
                                         self._adj_checker)
                          for f in window[:cfg.INPUT_SEQ_LEN]]
            future_seq = [_process_frame(f, G_cached, x_min, x_max, y_min, y_max,
                                         self._adj_checker)
                          for f in window[cfg.INPUT_SEQ_LEN:]]

            sequences.append({
                'past':         past_seq,
                'future':       future_seq,
                'graph':        G_seq,
                'graph_bounds': [x_min, x_max, y_min, y_max],
            })
            n_windows += 1

        if verbose:
            print(f"  [seq_builder] {n_windows} sequences, {n_rebuilds} graph builds")

        return sequences
