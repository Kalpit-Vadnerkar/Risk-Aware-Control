"""
BagReader: reads a ROS2 bag and returns a synchronized 10 Hz time series.

Output format matches DataReader.read_scene_data() from the T-ITS reference
codebase, extended with position_uncertainty fields.

Each frame dict:
    {
        'ego': {
            'position':             {'x', 'y', 'z'},
            'orientation':          {'x', 'y', 'z', 'w'},
            'velocity':             {'longitudinal', 'lateral', 'yaw_rate'},
            'steering':             float,      # actual tire angle (rad)
            'acceleration':         float,      # commanded (m/s²)
            'position_uncertainty': {'x_var', 'y_var'},  # EKF covariance (m²)
        },
        'objects': [
            {'position': {'x','y','z'}, 'orientation': {...},
             'velocity':  {'x','y','z'}, 'classification': int}
        ],
        'traffic_lights': [
            {'id': int, 'color': int, 'confidence': float}
        ],
    }
"""

import json
import math
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

# Requires Autoware workspace sourced
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import TrackedObjects, TrafficLightGroupArray
from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport
from autoware_adapi_v1_msgs.msg import MrmState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from . import config as cfg


_MSG_TYPES = {
    cfg.TOPICS['kinematic_state']:  Odometry,
    cfg.TOPICS['pose_covariance']:  PoseWithCovarianceStamped,
    cfg.TOPICS['velocity']:         VelocityReport,
    cfg.TOPICS['steering']:         SteeringReport,
    cfg.TOPICS['control']:          Control,
    cfg.TOPICS['objects']:          TrackedObjects,
    cfg.TOPICS['traffic_lights']:   TrafficLightGroupArray,
}

# MRM state is recorded but not a required field — forward-fill independently.
_MRM_TOPIC = '/system/fail_safe/mrm_state'
_MRM_NORMAL = 1  # MrmState.state value for NORMAL

# Reverse map: topic string → key name
_TOPIC_KEY = {v: k for k, v in cfg.TOPICS.items()}


def _ns_to_sec(ns: int) -> float:
    return ns * 1e-9


def _quaternion_to_yaw(qz: float, qw: float) -> float:
    return 2.0 * math.atan2(qz, qw)


def _extract_kinematic_state(msg: Odometry) -> dict:
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    t = msg.twist.twist.linear
    return {
        'position':    {'x': p.x, 'y': p.y, 'z': p.z},
        'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w},
        'yaw':         _quaternion_to_yaw(o.z, o.w),
    }


def _extract_pose_covariance(msg: PoseWithCovarianceStamped) -> dict:
    cov = msg.pose.covariance  # 36-element row-major 6×6 matrix
    return {
        'x_var':   float(cov[0]),   # (0,0)
        'y_var':   float(cov[7]),   # (1,1)
        'yaw_var': float(cov[35]),  # (5,5)
    }


def _extract_velocity(msg: VelocityReport) -> dict:
    return {
        'longitudinal': float(msg.longitudinal_velocity),
        'lateral':      float(msg.lateral_velocity),
        'yaw_rate':     float(msg.heading_rate),
    }


def _extract_steering(msg: SteeringReport) -> float:
    return float(msg.steering_tire_angle)


def _extract_control(msg: Control) -> dict:
    return {
        'steering_angle': float(msg.lateral.steering_tire_angle),
        'velocity':       float(msg.longitudinal.velocity),
        'acceleration':   float(msg.longitudinal.acceleration),
    }


def _extract_objects(msg: TrackedObjects) -> list:
    objects = []
    for obj in msg.objects:
        pos  = obj.kinematics.pose_with_covariance.pose.position
        ori  = obj.kinematics.pose_with_covariance.pose.orientation
        vel  = obj.kinematics.twist_with_covariance.twist.linear
        cls  = obj.classification[0].label if obj.classification else 0
        objects.append({
            'position':    {'x': pos.x, 'y': pos.y, 'z': pos.z},
            'orientation': {'x': ori.x, 'y': ori.y, 'z': ori.z, 'w': ori.w},
            'velocity':    {'x': vel.x, 'y': vel.y, 'z': vel.z},
            'classification': int(cls),
        })
    return objects


def _extract_traffic_lights(msg: TrafficLightGroupArray,
                             group_id: Optional[int] = None) -> list:
    """
    Extract traffic light info from the new TrafficLightGroupArray format.
    Returns only elements with a known color (RED=1, AMBER=2, GREEN=3).

    group_id: if given, only elements from that one traffic_light_group_id are
    returned (see _load_tl_group_zones / docs/stgat_pipeline_plan.md §1.11) —
    a TL fault only ever mutates the ONE group governing the vehicle's current
    lanelet, and pooling across every currently-tracked group (2-6 at once)
    lets the fault's effect be outvoted by other, unfaulted groups in the same
    message. None (default) keeps the old pooled-across-all-groups behavior.
    """
    lights = []
    for group in msg.traffic_light_groups:
        if group_id is not None and int(group.traffic_light_group_id) != group_id:
            continue
        for element in group.elements:
            if element.color in (1, 2, 3):   # RED, AMBER, GREEN
                lights.append({
                    'id':         int(group.traffic_light_group_id),
                    'color':      int(element.color),
                    'confidence': float(element.confidence),
                })
    return lights


# ── Traffic-light group scoping (docs/stgat_pipeline_plan.md §1.11) ────────

_tl_zones_cache: Dict[str, dict] = {}


def _load_tl_zones_file(zones_file: str) -> dict:
    """Load and cache tl_zones.json (shared across many read_bag() calls in a
    batch pipeline run). {} on any failure — callers treat that as 'no zones
    available', same fallback as compare_fault_vs_nominal.py."""
    if zones_file not in _tl_zones_cache:
        try:
            with open(zones_file) as f:
                _tl_zones_cache[zones_file] = json.load(f)
        except Exception:
            _tl_zones_cache[zones_file] = {}
    return _tl_zones_cache[zones_file]


def _load_tl_group_zones(goal_id: str, zones_file: str = cfg.TL_ZONES_FILE
                          ) -> List[Tuple[float, float, int]]:
    """(x, y, group_id) zones for one goal, from tl_zones.json. [] if the
    goal has no entry (e.g. it's never had a TL fault campaign computed for
    it) — same fallback path as fault_injector.py/compare_fault_vs_nominal.py.
    Deliberately does NOT filter on the 'reachable' field — see CLAUDE.md's
    note that 'reachable' is informational for gating, not a filter."""
    data = _load_tl_zones_file(zones_file)
    goal = data.get('goals', {}).get(goal_id)
    if not goal:
        return []
    return [(z['x'], z['y'], z['group_id']) for z in goal.get('tl_zones', [])]


def _nearest_group_id(x: float, y: float,
                       zones: List[Tuple[float, float, int]],
                       radius: float) -> Optional[int]:
    """Nearest zone's group_id within `radius` metres of (x, y), or None."""
    best_d, best_gid = radius, None
    for zx, zy, gid in zones:
        d = math.hypot(x - zx, y - zy)
        if d <= best_d:
            best_d, best_gid = d, gid
    return best_gid


def read_bag(bag_dir: str, goal_id: Optional[str] = None,
             verbose: bool = False) -> List[dict]:
    """
    Read a rosbag and return a list of synchronized 10 Hz frames.

    Strategy: read all messages in timestamp order. Maintain a "latest value"
    cache for each topic. Every time a TrackedObjects message arrives (master
    clock, ~10 Hz), emit one frame using the cached values. Drop frames where
    any topic is stale beyond MAX_STALENESS_SEC.

    goal_id: if given, traffic-light extraction is scoped to the one
    traffic_light_group_id governing the vehicle's current lanelet (via
    experiments/configs/tl_zones.json — see docs/stgat_pipeline_plan.md §1.11),
    instead of pooling every currently-tracked group. Pass the run's goal
    (e.g. 'goal_007') whenever it's known; leave None only when it genuinely
    isn't (falls back to the old pooled behavior).

    Returns list of frame dicts in chronological order. Stopped/idle periods
    are NOT filtered out (removed 2026-08-05 — the old MessageCleaner-derived
    filter deleted any >3s continuous near-zero-speed segment unconditionally,
    including fault-induced stuck periods and MRM-triggered stops, which are
    exactly the behavior the fault-detection pipeline needs to observe; the
    nominal dataset has no idle/parked segments to begin with, so the filter
    had no remaining upside once traffic_light_state stopped needing it).
    """
    storage_options   = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}

    # Bags collected before the fault_injector topic-wiring fix (2026-07-22,
    # see README.md item 8) never recorded traffic_signals_faulted at all —
    # without this fallback, 'traffic_lights' would never appear in `cache`,
    # `missing` below would never clear, and EVERY frame in those bags (i.e.
    # all of nom_v11, the actual training set) would be silently dropped.
    _tl_topic_unfaulted = '/perception/traffic_light_recognition/traffic_signals'
    _tl_topic = (cfg.TOPICS['traffic_lights']
                 if cfg.TOPICS['traffic_lights'] in topic_type_map
                 else _tl_topic_unfaulted)
    msg_types = dict(_MSG_TYPES)
    topic_key = dict(_TOPIC_KEY)
    if _tl_topic != cfg.TOPICS['traffic_lights']:
        msg_types[_tl_topic] = TrafficLightGroupArray
        topic_key[_tl_topic] = 'traffic_lights'
        if verbose:
            print(f"  [bag_reader] {cfg.TOPICS['traffic_lights']} not in this bag — "
                  f"falling back to {_tl_topic} (pre-fix bag; never actually faulted)")

    # Latest extracted payload per topic key
    cache: Dict[str, object]  = {}
    # Timestamp of latest message per topic key (seconds)
    cache_ts: Dict[str, float] = {}

    # MRM state tracked separately — optional, not required for frame emission
    _mrm_active: bool = False

    # TL group-scoping (§1.11): last known ego position, used to pick which
    # traffic_light_group_id governs "right now" whenever a TL message arrives.
    group_zones = _load_tl_group_zones(goal_id) if goal_id else []
    last_ego_xy: Optional[Tuple[float, float]] = None

    frames: List[dict] = []

    master_topic = cfg.TOPICS[cfg.MASTER_CLOCK_TOPIC]

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()

        if topic == _MRM_TOPIC:
            try:
                msg = deserialize_message(data, MrmState)
                _mrm_active = (msg.state != _MRM_NORMAL)
            except Exception:
                pass
            continue

        if topic not in msg_types:
            continue

        msg_cls = msg_types[topic]
        try:
            msg = deserialize_message(data, msg_cls)
        except Exception:
            continue

        t_sec   = _ns_to_sec(timestamp_ns)
        key     = topic_key[topic]

        # Update cache with extracted payload
        if topic == cfg.TOPICS['kinematic_state']:
            cache[key]    = _extract_kinematic_state(msg)
            pos = cache[key]['position']
            last_ego_xy = (pos['x'], pos['y'])
        elif topic == cfg.TOPICS['pose_covariance']:
            cache[key]    = _extract_pose_covariance(msg)
        elif topic == cfg.TOPICS['velocity']:
            cache[key]    = _extract_velocity(msg)
        elif topic == cfg.TOPICS['steering']:
            cache[key]    = _extract_steering(msg)
        elif topic == cfg.TOPICS['control']:
            cache[key]    = _extract_control(msg)
        elif topic == cfg.TOPICS['objects']:
            cache[key]    = _extract_objects(msg)
        elif topic == _tl_topic:
            current_gid = None
            if group_zones and last_ego_xy is not None:
                current_gid = _nearest_group_id(
                    last_ego_xy[0], last_ego_xy[1], group_zones, cfg.TL_ZONE_RADIUS_M)
            cache[key]    = _extract_traffic_lights(msg, group_id=current_gid)

        cache_ts[key] = t_sec

        # Emit frame when master clock (objects) fires
        if topic != master_topic:
            continue

        # Check all required keys are present and fresh
        required_keys = list(cfg.TOPICS.keys())
        missing = [k for k in required_keys if k not in cache]
        if missing:
            continue

        stale = [k for k in required_keys
                 if t_sec - cache_ts[k] > cfg.MAX_STALENESS_SEC]
        if stale:
            if verbose:
                print(f"  [bag_reader] dropping frame t={t_sec:.2f}s, stale: {stale}")
            continue

        kin  = cache['kinematic_state']
        cov  = cache['pose_covariance']
        vel  = cache['velocity']
        ctrl = cache['control']

        frame = {
            # Real wall-clock time (seconds) of this frame's master-clock
            # message, from rosbag2's own recorded timestamp — needed to build
            # the divergence trace schema's timestamps (docs/theoretical_framework.md
            # §5) and to join fault_log.jsonl's `wall_time` events (NOT
            # `sim_time_sec` — a different, unrelated clock; see
            # compare_fault_vs_nominal.py's extract_fault_windows docstring
            # for the bug this distinction already caused once).
            't_sim': t_sec,
            'ego': {
                'position':    kin['position'],
                'orientation': kin['orientation'],
                'velocity': {
                    'longitudinal': vel['longitudinal'],
                    'lateral':      vel['lateral'],
                    'yaw_rate':     vel['yaw_rate'],
                },
                'steering':     cache['steering'],
                'acceleration': ctrl['acceleration'],
                'position_uncertainty': {
                    'x_var': cov['x_var'],
                    'y_var': cov['y_var'],
                },
            },
            'objects':        list(cache['objects']),   # copy
            'traffic_lights': list(cache['traffic_lights']),
            'mrm_active':     _mrm_active,
        }
        frames.append(frame)

    if verbose:
        print(f"  [bag_reader] raw frames: {len(frames)}")

    return frames
