"""
Metrics computation for experiment analysis.

Computes safety, reliability, and fail-operational metrics from rosbag data.
"""

import os
import json
import math
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Any, Optional, Tuple
from collections import defaultdict

# ROS2 imports
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# Message types (requires sourcing Autoware)
from autoware_adapi_v1_msgs.msg import MrmState, RouteState
from autoware_system_msgs.msg import AutowareState
from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport
from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray


# Message type mapping
MESSAGE_TYPES = {
    'autoware_adapi_v1_msgs/msg/MrmState': MrmState,
    'autoware_adapi_v1_msgs/msg/RouteState': RouteState,
    'autoware_system_msgs/msg/AutowareState': AutowareState,
    'autoware_vehicle_msgs/msg/VelocityReport': VelocityReport,
    'autoware_vehicle_msgs/msg/SteeringReport': SteeringReport,
    'autoware_control_msgs/msg/Control': Control,
    'autoware_perception_msgs/msg/PredictedObjects': PredictedObjects,
    'nav_msgs/msg/Odometry': Odometry,
    'geometry_msgs/msg/PoseWithCovarianceStamped': PoseWithCovarianceStamped,
    'diagnostic_msgs/msg/DiagnosticArray': DiagnosticArray,
}


@dataclass
class SafetyMetrics:
    """Safety-related metrics."""
    min_object_distance: float = float('inf')
    mean_object_distance: float = 0.0
    collision_proxy_count: int = 0  # Times distance < threshold
    collision_threshold: float = 1.0  # meters
    near_miss_count: int = 0  # TTC < threshold
    near_miss_threshold: float = 2.0  # seconds
    min_ttc: float = float('inf')
    mean_ttc: float = 0.0


@dataclass
class ReliabilityMetrics:
    """Reliability-related metrics."""
    goal_reached: bool = False
    completion_time: float = 0.0
    driving_time: float = 0.0
    mean_velocity: float = 0.0
    max_velocity: float = 0.0
    velocity_std: float = 0.0
    # Trajectory tracking (requires planned trajectory)
    lateral_rmse: float = 0.0
    longitudinal_rmse: float = 0.0


@dataclass
class FailOperationalMetrics:
    """Fail-operational metrics."""
    mrm_trigger_count: int = 0
    mrm_total_duration: float = 0.0
    emergency_stop_count: int = 0
    comfortable_stop_count: int = 0
    mrm_recovery_count: int = 0
    diagnostic_error_count: int = 0
    diagnostic_warn_count: int = 0


@dataclass
class ComfortMetrics:
    """Comfort-related metrics (secondary)."""
    max_acceleration: float = 0.0
    max_deceleration: float = 0.0
    max_jerk: float = 0.0
    max_lateral_acceleration: float = 0.0
    max_steering_rate: float = 0.0


@dataclass
class ExperimentMetrics:
    """Complete metrics for an experiment."""
    safety: SafetyMetrics = field(default_factory=SafetyMetrics)
    reliability: ReliabilityMetrics = field(default_factory=ReliabilityMetrics)
    fail_operational: FailOperationalMetrics = field(default_factory=FailOperationalMetrics)
    comfort: ComfortMetrics = field(default_factory=ComfortMetrics)

    def to_dict(self) -> dict:
        return {
            'safety': asdict(self.safety),
            'reliability': asdict(self.reliability),
            'fail_operational': asdict(self.fail_operational),
            'comfort': asdict(self.comfort),
        }


class MetricsCollector:
    """Collects and computes metrics from rosbag data."""

    def __init__(self, bag_path: str, goal_position: Optional[Dict[str, float]] = None):
        self.bag_path = bag_path
        self.goal_position = goal_position

        # Data storage
        self.velocities: List[Tuple[float, float]] = []  # (timestamp, velocity)
        self.accelerations: List[Tuple[float, float]] = []
        self.steerings: List[Tuple[float, float]] = []
        self.positions: List[Tuple[float, float, float]] = []  # (timestamp, x, y)
        self.object_distances: List[Tuple[float, float]] = []  # (timestamp, min_distance)
        self.mrm_states: List[Tuple[float, int, int]] = []  # (timestamp, state, behavior)
        self.autoware_states: List[Tuple[float, int]] = []
        self.diagnostic_issues: List[Tuple[float, int]] = []  # (timestamp, level)

        self.start_time_ns: Optional[int] = None
        self.driving_start_ns: Optional[int] = None
        self.driving_end_ns: Optional[int] = None

    def _get_msg_type(self, type_str: str):
        """Get message class from type string."""
        return MESSAGE_TYPES.get(type_str)

    def _ns_to_sec(self, ns: int) -> float:
        """Convert nanoseconds to relative seconds."""
        if self.start_time_ns is None:
            return 0.0
        return (ns - self.start_time_ns) / 1e9

    def read_bag(self):
        """Read and parse rosbag data."""
        storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}

        prev_velocity = None
        prev_velocity_time = None
        prev_steering = None
        prev_steering_time = None
        prev_mrm_state = 1  # NORMAL

        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if self.start_time_ns is None:
                self.start_time_ns = timestamp

            time_sec = self._ns_to_sec(timestamp)
            msg_type = self._get_msg_type(topic_types.get(topic, ''))

            if msg_type is None:
                continue

            try:
                msg = deserialize_message(data, msg_type)
            except Exception:
                continue

            # Process by topic
            if topic == '/vehicle/status/velocity_status':
                vel = msg.longitudinal_velocity
                self.velocities.append((time_sec, vel))

                # Compute acceleration
                if prev_velocity is not None and prev_velocity_time is not None:
                    dt = time_sec - prev_velocity_time
                    if dt > 0.001:
                        accel = (vel - prev_velocity) / dt
                        self.accelerations.append((time_sec, accel))
                prev_velocity = vel
                prev_velocity_time = time_sec

            elif topic == '/vehicle/status/steering_status':
                steer = msg.steering_tire_angle
                self.steerings.append((time_sec, steer))

                # Compute steering rate
                if prev_steering is not None and prev_steering_time is not None:
                    dt = time_sec - prev_steering_time
                    if dt > 0.001:
                        steer_rate = abs(steer - prev_steering) / dt
                        # Store for comfort metrics
                prev_steering = steer
                prev_steering_time = time_sec

            elif topic == '/localization/kinematic_state':
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                self.positions.append((time_sec, x, y))

            elif topic == '/perception/object_recognition/objects':
                min_dist = self._compute_min_object_distance(msg)
                if min_dist is not None:
                    self.object_distances.append((time_sec, min_dist))

            elif topic == '/system/fail_safe/mrm_state':
                state = msg.state
                behavior = msg.behavior
                self.mrm_states.append((time_sec, state, behavior))

                # Track state transitions
                if prev_mrm_state == 1 and state == 2:  # NORMAL -> MRM_OPERATING
                    pass  # Transition counted in metrics
                prev_mrm_state = state

            elif topic == '/autoware/state':
                state = msg.state
                self.autoware_states.append((time_sec, state))

                # Track driving time
                if state == 5 and self.driving_start_ns is None:  # DRIVING
                    self.driving_start_ns = timestamp
                elif state == 6:  # ARRIVED_GOAL
                    self.driving_end_ns = timestamp

            elif topic == '/diagnostics':
                for status in msg.status:
                    # status.level is byte type (b'\x00'), convert to int for comparison
                    level = status.level
                    if isinstance(level, bytes):
                        level = level[0] if level else 0
                    if level > 0:
                        self.diagnostic_issues.append((time_sec, level))

    def _compute_min_object_distance(self, msg) -> Optional[float]:
        """Compute minimum distance to any detected object."""
        if not hasattr(msg, 'objects') or len(msg.objects) == 0:
            return None

        min_dist = float('inf')
        for obj in msg.objects:
            # Get object position (assuming it's relative to ego or in map frame)
            x = obj.kinematics.initial_pose_with_covariance.pose.position.x
            y = obj.kinematics.initial_pose_with_covariance.pose.position.y
            dist = math.sqrt(x*x + y*y)
            min_dist = min(min_dist, dist)

        return min_dist if min_dist < float('inf') else None

    def compute_metrics(self) -> ExperimentMetrics:
        """Compute all metrics from collected data."""
        metrics = ExperimentMetrics()

        # Safety metrics
        self._compute_safety_metrics(metrics.safety)

        # Reliability metrics
        self._compute_reliability_metrics(metrics.reliability)

        # Fail-operational metrics
        self._compute_fail_operational_metrics(metrics.fail_operational)

        # Comfort metrics
        self._compute_comfort_metrics(metrics.comfort)

        return metrics

    def _compute_safety_metrics(self, m: SafetyMetrics):
        """Compute safety metrics."""
        if self.object_distances:
            distances = [d for _, d in self.object_distances]
            m.min_object_distance = min(distances)
            m.mean_object_distance = sum(distances) / len(distances)
            m.collision_proxy_count = sum(1 for d in distances if d < m.collision_threshold)

        # TTC computation would require velocity of approaching objects
        # For now, use distance as proxy

    def _compute_reliability_metrics(self, m: ReliabilityMetrics):
        """Compute reliability metrics."""
        # Check if goal was ever reached (not just final state)
        if self.autoware_states:
            m.goal_reached = any(state == 6 for _, state in self.autoware_states)  # ARRIVED_GOAL

        # Driving time
        if self.driving_start_ns and self.driving_end_ns:
            m.driving_time = (self.driving_end_ns - self.driving_start_ns) / 1e9
        elif self.driving_start_ns and self.start_time_ns:
            # Use last timestamp if goal not reached
            if self.velocities:
                last_time = self.velocities[-1][0]
                driving_start_sec = (self.driving_start_ns - self.start_time_ns) / 1e9
                m.driving_time = last_time - driving_start_sec

        # Total completion time
        if self.start_time_ns and self.velocities:
            m.completion_time = self.velocities[-1][0]

        # Velocity statistics
        if self.velocities:
            vels = [v for _, v in self.velocities]
            m.mean_velocity = sum(vels) / len(vels)
            m.max_velocity = max(vels)
            mean_v = m.mean_velocity
            m.velocity_std = math.sqrt(sum((v - mean_v)**2 for v in vels) / len(vels))

    def _compute_fail_operational_metrics(self, m: FailOperationalMetrics):
        """Compute fail-operational metrics."""
        if self.mrm_states:
            prev_state = 1  # NORMAL
            mrm_start_time = None

            for time_sec, state, behavior in self.mrm_states:
                # Count triggers (NORMAL -> MRM_OPERATING)
                if prev_state == 1 and state == 2:
                    m.mrm_trigger_count += 1
                    mrm_start_time = time_sec

                    if behavior == 2:  # EMERGENCY_STOP
                        m.emergency_stop_count += 1
                    elif behavior == 3:  # COMFORTABLE_STOP
                        m.comfortable_stop_count += 1

                # Count recoveries (MRM_OPERATING -> NORMAL)
                if prev_state == 2 and state == 1:
                    m.mrm_recovery_count += 1
                    if mrm_start_time is not None:
                        m.mrm_total_duration += time_sec - mrm_start_time
                    mrm_start_time = None

                prev_state = state

        # Diagnostic issues
        for _, level in self.diagnostic_issues:
            if level == 2:  # ERROR
                m.diagnostic_error_count += 1
            elif level == 1:  # WARN
                m.diagnostic_warn_count += 1

    def _compute_comfort_metrics(self, m: ComfortMetrics):
        """Compute comfort metrics."""
        if self.accelerations:
            accels = [a for _, a in self.accelerations]
            m.max_acceleration = max(a for a in accels if a > 0) if any(a > 0 for a in accels) else 0.0
            m.max_deceleration = abs(min(a for a in accels if a < 0)) if any(a < 0 for a in accels) else 0.0

            # Compute jerk
            prev_accel = None
            prev_time = None
            max_jerk = 0.0
            for time_sec, accel in self.accelerations:
                if prev_accel is not None and prev_time is not None:
                    dt = time_sec - prev_time
                    if dt > 0.001:
                        jerk = abs(accel - prev_accel) / dt
                        max_jerk = max(max_jerk, jerk)
                prev_accel = accel
                prev_time = time_sec
            m.max_jerk = max_jerk


def compute_metrics_from_bag(bag_path: str, goal_position: Optional[Dict[str, float]] = None) -> ExperimentMetrics:
    """Convenience function to compute metrics from a rosbag."""
    collector = MetricsCollector(bag_path, goal_position)
    collector.read_bag()
    return collector.compute_metrics()


def save_metrics(metrics: ExperimentMetrics, output_file: str):
    """Save metrics to JSON file."""
    with open(output_file, 'w') as f:
        json.dump(metrics.to_dict(), f, indent=2)


def load_metrics(input_file: str) -> dict:
    """Load metrics from JSON file."""
    with open(input_file, 'r') as f:
        return json.load(f)
