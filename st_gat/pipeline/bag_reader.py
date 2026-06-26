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

import math
import sys
from collections import defaultdict
from typing import Dict, List, Optional

# Requires Autoware workspace sourced
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import TrackedObjects, TrafficLightGroupArray
from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport
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


def _extract_traffic_lights(msg: TrafficLightGroupArray) -> list:
    """
    Extract traffic light info from the new TrafficLightGroupArray format.
    Returns only elements with a known color (RED=1, AMBER=2, GREEN=3).
    """
    lights = []
    for group in msg.traffic_light_groups:
        for element in group.elements:
            if element.color in (1, 2, 3):   # RED, AMBER, GREEN
                lights.append({
                    'id':         int(group.traffic_light_group_id),
                    'color':      int(element.color),
                    'confidence': float(element.confidence),
                })
    return lights


def _filter_stopped_periods(frames: List[dict]) -> List[dict]:
    """
    Remove sequences of consecutive frames where vehicle speed is below
    STOPPED_THRESHOLD_MS for longer than MAX_STOPPED_DURATION_SEC.
    Short stops (red lights) are kept; long stops (stuck) are removed.
    Mirrors the original MessageCleaner.process_velocity_data() logic.
    """
    if not frames:
        return frames

    keep = [True] * len(frames)
    stopped_start_idx = None

    for i, frame in enumerate(frames):
        speed = abs(frame['ego']['velocity']['longitudinal'])
        if speed < cfg.STOPPED_THRESHOLD_MS:
            if stopped_start_idx is None:
                stopped_start_idx = i
        else:
            if stopped_start_idx is not None:
                # How long was the stop? Each frame is ~0.1s
                stop_duration = (i - stopped_start_idx) * 0.1
                if stop_duration > cfg.MAX_STOPPED_DURATION_SEC:
                    for j in range(stopped_start_idx, i):
                        keep[j] = False
                stopped_start_idx = None

    # Handle stop extending to end of run
    if stopped_start_idx is not None:
        stop_duration = (len(frames) - stopped_start_idx) * 0.1
        if stop_duration > cfg.MAX_STOPPED_DURATION_SEC:
            for j in range(stopped_start_idx, len(frames)):
                keep[j] = False

    return [f for f, k in zip(frames, keep) if k]


def read_bag(bag_dir: str, verbose: bool = False) -> List[dict]:
    """
    Read a rosbag and return a list of synchronized 10 Hz frames.

    Strategy: read all messages in timestamp order. Maintain a "latest value"
    cache for each topic. Every time a TrackedObjects message arrives (master
    clock, ~10 Hz), emit one frame using the cached values. Drop frames where
    any topic is stale beyond MAX_STALENESS_SEC.

    Returns list of frame dicts in chronological order with stopped periods removed.
    """
    storage_options   = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}

    # Latest extracted payload per topic key
    cache: Dict[str, object]  = {}
    # Timestamp of latest message per topic key (seconds)
    cache_ts: Dict[str, float] = {}

    frames: List[dict] = []

    master_topic = cfg.TOPICS[cfg.MASTER_CLOCK_TOPIC]

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()

        if topic not in _MSG_TYPES:
            continue

        msg_cls = _MSG_TYPES[topic]
        try:
            msg = deserialize_message(data, msg_cls)
        except Exception:
            continue

        t_sec   = _ns_to_sec(timestamp_ns)
        key     = _TOPIC_KEY[topic]

        # Update cache with extracted payload
        if topic == cfg.TOPICS['kinematic_state']:
            cache[key]    = _extract_kinematic_state(msg)
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
        elif topic == cfg.TOPICS['traffic_lights']:
            cache[key]    = _extract_traffic_lights(msg)

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
            'objects':       list(cache['objects']),   # copy
            'traffic_lights': list(cache['traffic_lights']),
        }
        frames.append(frame)

    if verbose:
        print(f"  [bag_reader] raw frames: {len(frames)}")

    frames = _filter_stopped_periods(frames)

    if verbose:
        print(f"  [bag_reader] after stop-filtering: {len(frames)}")

    return frames
