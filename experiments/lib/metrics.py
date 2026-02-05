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
from bisect import bisect_left

# ROS2 imports
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# Message types (requires sourcing Autoware)
from autoware_adapi_v1_msgs.msg import MrmState, RouteState
from autoware_system_msgs.msg import AutowareState
from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport
from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import PredictedObjects, TrackedObjects
from autoware_planning_msgs.msg import Trajectory
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
    'autoware_perception_msgs/msg/TrackedObjects': TrackedObjects,
    'autoware_planning_msgs/msg/Trajectory': Trajectory,
    'nav_msgs/msg/Odometry': Odometry,
    'geometry_msgs/msg/PoseWithCovarianceStamped': PoseWithCovarianceStamped,
    'diagnostic_msgs/msg/DiagnosticArray': DiagnosticArray,
}


@dataclass
class SafetyMetrics:
    """Safety-related metrics."""
    min_object_distance: float = float('inf')
    clearance_p5: float = float('inf')  # 5th percentile clearance (more robust)
    mean_object_distance: float = 0.0
    collision_proxy_count: int = 0  # Times distance < threshold
    collision_threshold: float = 2.0  # meters (vehicle half-width + margin)
    near_miss_count: int = 0  # TTC < threshold
    near_miss_rate: float = 0.0  # Near misses per km
    critical_ttc_count: int = 0  # TTC < 1s (critical)
    critical_ttc_rate: float = 0.0  # Critical TTC per km
    near_miss_threshold: float = 2.0  # seconds
    min_ttc: float = float('inf')
    mean_ttc: float = 0.0
    object_count_total: int = 0  # Total object detections


@dataclass
class ReliabilityMetrics:
    """Reliability-related metrics."""
    goal_reached: bool = False
    completion_time: float = 0.0
    driving_time: float = 0.0
    expected_time: float = 0.0  # Based on route distance and planned velocity
    time_ratio: float = 0.0  # actual / expected (>1 means slower than expected)
    time_efficiency: float = 0.0  # expected / actual (1.0 = optimal, <1 = slower)
    mean_velocity: float = 0.0
    max_velocity: float = 0.0
    velocity_std: float = 0.0
    # Trajectory tracking - NOTE: lateral_rmse from trajectory is unreliable
    # Should use /control/control_performance/error when available
    lateral_rmse: float = 0.0
    longitudinal_rmse: float = 0.0
    total_distance_traveled: float = 0.0
    distance_km: float = 0.0  # Distance in km for normalization
    route_distance: float = 0.0  # Planned route distance


@dataclass
class FailOperationalMetrics:
    """Fail-operational metrics."""
    mrm_trigger_count: int = 0
    mrm_rate: float = 0.0  # MRM triggers per km (normalized)
    mrm_total_duration: float = 0.0
    mrm_time_fraction: float = 0.0  # Fraction of driving time in MRM
    emergency_stop_count: int = 0
    emergency_stop_rate: float = 0.0  # E-stops per km
    comfortable_stop_count: int = 0
    comfortable_stop_rate: float = 0.0  # Comfortable stops per km
    estop_ratio: float = 0.0  # emergency / (emergency + comfortable)
    mrm_recovery_count: int = 0
    mrm_recovery_rate: float = 0.0  # recoveries / triggers
    avg_mrm_duration: float = 0.0
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
    hard_brake_count: int = 0  # decel > 3 m/s^2


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

        # Data storage with timestamps for synchronization
        self.velocities: List[Tuple[float, float]] = []  # (timestamp, velocity)
        self.accelerations: List[Tuple[float, float]] = []
        self.steerings: List[Tuple[float, float]] = []
        self.steering_rates: List[Tuple[float, float]] = []
        self.positions: List[Tuple[float, float, float, float]] = []  # (timestamp, x, y, heading)

        # Object tracking with ego-relative distances
        self.object_distances: List[Tuple[float, float]] = []  # (timestamp, min_distance)
        self.ttc_values: List[Tuple[float, float]] = []  # (timestamp, ttc)

        # State tracking
        self.mrm_states: List[Tuple[float, int, int]] = []  # (timestamp, state, behavior)
        self.autoware_states: List[Tuple[float, int]] = []
        self.diagnostic_issues: List[Tuple[float, int]] = []  # (timestamp, level)

        # Trajectory tracking
        self.trajectory_points: List[Tuple[float, float, float]] = []  # (timestamp, x, y)
        self.lateral_errors: List[float] = []

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

    def _get_ego_pose_at_time(self, time_sec: float) -> Optional[Tuple[float, float, float]]:
        """Get interpolated ego pose (x, y, heading) at given time."""
        if not self.positions:
            return None

        times = [p[0] for p in self.positions]

        # Find closest position (simple nearest-neighbor for now)
        idx = bisect_left(times, time_sec)
        if idx == 0:
            return (self.positions[0][1], self.positions[0][2], self.positions[0][3])
        if idx >= len(times):
            return (self.positions[-1][1], self.positions[-1][2], self.positions[-1][3])

        # Use closer one
        if time_sec - times[idx-1] < times[idx] - time_sec:
            return (self.positions[idx-1][1], self.positions[idx-1][2], self.positions[idx-1][3])
        return (self.positions[idx][1], self.positions[idx][2], self.positions[idx][3])

    def _get_ego_velocity_at_time(self, time_sec: float) -> Optional[float]:
        """Get interpolated ego velocity at given time."""
        if not self.velocities:
            return None

        times = [v[0] for v in self.velocities]
        idx = bisect_left(times, time_sec)

        if idx == 0:
            return self.velocities[0][1]
        if idx >= len(times):
            return self.velocities[-1][1]

        if time_sec - times[idx-1] < times[idx] - time_sec:
            return self.velocities[idx-1][1]
        return self.velocities[idx][1]

    def _quaternion_to_yaw(self, qz: float, qw: float) -> float:
        """Convert quaternion (z, w components) to yaw angle."""
        return 2.0 * math.atan2(qz, qw)

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
                        self.steering_rates.append((time_sec, steer_rate))
                prev_steering = steer
                prev_steering_time = time_sec

            elif topic == '/localization/kinematic_state':
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                heading = self._quaternion_to_yaw(qz, qw)
                self.positions.append((time_sec, x, y, heading))

            elif topic in ['/perception/object_recognition/objects',
                          '/perception/object_recognition/tracking/objects']:
                self._process_objects(msg, time_sec)

            elif topic == '/planning/scenario_planning/trajectory':
                self._process_trajectory(msg, time_sec)

            elif topic == '/system/fail_safe/mrm_state':
                state = msg.state
                behavior = msg.behavior
                self.mrm_states.append((time_sec, state, behavior))
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
                    level = status.level
                    if isinstance(level, bytes):
                        level = level[0] if level else 0
                    if level > 0:
                        self.diagnostic_issues.append((time_sec, level))

    def _process_objects(self, msg, time_sec: float):
        """Process object detection message and compute distances/TTC."""
        objects = msg.objects if hasattr(msg, 'objects') else []
        if not objects:
            return

        ego_pose = self._get_ego_pose_at_time(time_sec)
        ego_vel = self._get_ego_velocity_at_time(time_sec)

        if ego_pose is None:
            return

        ego_x, ego_y, ego_heading = ego_pose

        min_dist = float('inf')
        min_ttc = float('inf')

        for obj in objects:
            # Handle both PredictedObjects and TrackedObjects message types
            # PredictedObjects uses: kinematics.initial_pose_with_covariance
            # TrackedObjects uses: kinematics.pose_with_covariance
            try:
                if hasattr(obj.kinematics, 'initial_pose_with_covariance'):
                    # PredictedObjects
                    obj_x = obj.kinematics.initial_pose_with_covariance.pose.position.x
                    obj_y = obj.kinematics.initial_pose_with_covariance.pose.position.y
                    obj_vx = obj.kinematics.initial_twist_with_covariance.twist.linear.x
                    obj_vy = obj.kinematics.initial_twist_with_covariance.twist.linear.y
                elif hasattr(obj.kinematics, 'pose_with_covariance'):
                    # TrackedObjects
                    obj_x = obj.kinematics.pose_with_covariance.pose.position.x
                    obj_y = obj.kinematics.pose_with_covariance.pose.position.y
                    obj_vx = obj.kinematics.twist_with_covariance.twist.linear.x
                    obj_vy = obj.kinematics.twist_with_covariance.twist.linear.y
                else:
                    continue
            except AttributeError:
                continue

            # Compute relative position
            rel_x = obj_x - ego_x
            rel_y = obj_y - ego_y
            distance = math.sqrt(rel_x*rel_x + rel_y*rel_y)
            min_dist = min(min_dist, distance)

            # Compute TTC if we have velocity info
            if ego_vel is not None and ego_vel > 0.1:
                # Project relative position onto ego heading direction
                cos_h = math.cos(ego_heading)
                sin_h = math.sin(ego_heading)

                # Longitudinal distance (in front of ego)
                long_dist = rel_x * cos_h + rel_y * sin_h

                # Lateral distance (perpendicular to ego heading)
                lat_dist = -rel_x * sin_h + rel_y * cos_h

                # Only consider objects:
                # 1. In front of ego (long_dist > 0)
                # 2. Within lane width (|lat_dist| < 2.5m for ~lane width + vehicle width)
                LANE_WIDTH_THRESHOLD = 2.5  # meters

                if long_dist > 0 and abs(lat_dist) < LANE_WIDTH_THRESHOLD:
                    # Relative velocity along ego heading
                    obj_speed_along_ego = obj_vx * cos_h + obj_vy * sin_h
                    closing_speed = ego_vel - obj_speed_along_ego

                    # TTC = distance / closing_speed (only if closing)
                    # Only record TTC < 10s (anything higher isn't safety-relevant)
                    if closing_speed > 0.1:
                        ttc = long_dist / closing_speed
                        if ttc < 10.0:  # Cap at 10 seconds
                            min_ttc = min(min_ttc, ttc)

        if min_dist < float('inf'):
            self.object_distances.append((time_sec, min_dist))
        if min_ttc < float('inf'):
            self.ttc_values.append((time_sec, min_ttc))

    def _process_trajectory(self, msg, time_sec: float):
        """Process planned trajectory for tracking error computation."""
        if not hasattr(msg, 'points') or len(msg.points) == 0:
            return

        ego_pose = self._get_ego_pose_at_time(time_sec)
        if ego_pose is None:
            return

        ego_x, ego_y, ego_heading = ego_pose

        # Find closest point on trajectory
        min_dist = float('inf')
        closest_point = None

        for point in msg.points:
            traj_x = point.pose.position.x
            traj_y = point.pose.position.y

            dist = math.sqrt((traj_x - ego_x)**2 + (traj_y - ego_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_point = (traj_x, traj_y)

        if closest_point is not None:
            # Compute lateral error (perpendicular distance to trajectory)
            # This is a simplified version - proper implementation would use
            # the tangent of the trajectory at the closest point
            self.lateral_errors.append(min_dist)

    def compute_metrics(self) -> ExperimentMetrics:
        """Compute all metrics from collected data."""
        metrics = ExperimentMetrics()

        # Reliability metrics first (to get distance_km for normalization)
        self._compute_reliability_metrics(metrics.reliability)

        # Get distance in km for normalizing other metrics
        distance_km = metrics.reliability.distance_km
        driving_time = metrics.reliability.driving_time

        # Safety metrics (with distance normalization)
        self._compute_safety_metrics(metrics.safety, distance_km)

        # Fail-operational metrics (with distance and time normalization)
        self._compute_fail_operational_metrics(metrics.fail_operational, distance_km, driving_time)

        # Comfort metrics
        self._compute_comfort_metrics(metrics.comfort)

        return metrics

    def _compute_safety_metrics(self, m: SafetyMetrics, distance_km: float = 0.0):
        """Compute safety metrics."""
        if self.object_distances:
            distances = [d for _, d in self.object_distances]
            m.min_object_distance = min(distances)
            m.mean_object_distance = sum(distances) / len(distances)
            m.collision_proxy_count = sum(1 for d in distances if d < m.collision_threshold)
            m.object_count_total = len(distances)

            # 5th percentile clearance (more robust than min)
            sorted_distances = sorted(distances)
            p5_idx = max(0, int(len(sorted_distances) * 0.05))
            m.clearance_p5 = sorted_distances[p5_idx]

        if self.ttc_values:
            ttcs = [t for _, t in self.ttc_values]
            m.min_ttc = min(ttcs)
            m.mean_ttc = sum(ttcs) / len(ttcs)

            # Count distinct events (not every frame)
            # An event starts when TTC drops below threshold and ends when it rises above
            # Minimum 2s gap between separate events to avoid double-counting
            MIN_EVENT_GAP = 2.0  # seconds

            near_miss_events = 0
            critical_ttc_events = 0
            last_near_miss_time = -float('inf')
            last_critical_time = -float('inf')

            for timestamp, ttc in self.ttc_values:
                # Near miss event (TTC < 2s)
                if ttc < m.near_miss_threshold:
                    if timestamp - last_near_miss_time > MIN_EVENT_GAP:
                        near_miss_events += 1
                        last_near_miss_time = timestamp

                # Critical TTC event (TTC < 1s)
                if ttc < 1.0:
                    if timestamp - last_critical_time > MIN_EVENT_GAP:
                        critical_ttc_events += 1
                        last_critical_time = timestamp

            m.near_miss_count = near_miss_events
            m.critical_ttc_count = critical_ttc_events

            # Normalize by distance
            if distance_km > 0:
                m.near_miss_rate = m.near_miss_count / distance_km
                m.critical_ttc_rate = m.critical_ttc_count / distance_km

    def _compute_reliability_metrics(self, m: ReliabilityMetrics):
        """Compute reliability metrics."""
        # Check if goal was ever reached
        if self.autoware_states:
            m.goal_reached = any(state == 6 for _, state in self.autoware_states)

        # Driving time
        if self.driving_start_ns and self.driving_end_ns:
            m.driving_time = (self.driving_end_ns - self.driving_start_ns) / 1e9
        elif self.driving_start_ns and self.start_time_ns:
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

        # Total distance traveled
        if len(self.positions) > 1:
            total_dist = 0.0
            for i in range(1, len(self.positions)):
                dx = self.positions[i][1] - self.positions[i-1][1]
                dy = self.positions[i][2] - self.positions[i-1][2]
                total_dist += math.sqrt(dx*dx + dy*dy)
            m.total_distance_traveled = total_dist
            m.distance_km = total_dist / 1000.0  # Convert to km for normalization
            m.route_distance = total_dist  # Use traveled as route distance estimate

        # Trajectory tracking error
        # NOTE: The computed lateral_errors from trajectory are unreliable
        # because the trajectory is in vehicle frame (future path) not a reference line.
        # Only use if values are reasonable (< 5m)
        if self.lateral_errors:
            # Filter out unreasonable values
            reasonable_errors = [e for e in self.lateral_errors if e < 5.0]
            if reasonable_errors:
                m.lateral_rmse = math.sqrt(sum(e*e for e in reasonable_errors) / len(reasonable_errors))
            # If no reasonable values, leave as 0 - should use control_performance topic instead

        # Expected time calculation
        # Use mean velocity from successful portion to estimate expected time
        # Assume planned avg velocity is 60% of max velocity (accounting for stops, turns)
        if m.max_velocity > 0 and m.total_distance_traveled > 0:
            planned_avg_velocity = m.max_velocity * 0.6  # Conservative estimate
            m.expected_time = m.total_distance_traveled / planned_avg_velocity
            if m.driving_time > 0:
                m.time_ratio = m.driving_time / m.expected_time
                m.time_efficiency = m.expected_time / m.driving_time  # Inverted: 1.0 = optimal

    def _compute_fail_operational_metrics(self, m: FailOperationalMetrics, distance_km: float = 0.0, driving_time: float = 0.0):
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

            # Compute derived metrics
            if m.mrm_trigger_count > 0:
                m.mrm_recovery_rate = m.mrm_recovery_count / m.mrm_trigger_count
                m.avg_mrm_duration = m.mrm_total_duration / m.mrm_trigger_count

            # E-stop ratio
            total_stops = m.emergency_stop_count + m.comfortable_stop_count
            if total_stops > 0:
                m.estop_ratio = m.emergency_stop_count / total_stops

            # Normalized rates (per km)
            if distance_km > 0:
                m.mrm_rate = m.mrm_trigger_count / distance_km
                m.emergency_stop_rate = m.emergency_stop_count / distance_km
                m.comfortable_stop_rate = m.comfortable_stop_count / distance_km

            # MRM time fraction
            if driving_time > 0:
                m.mrm_time_fraction = m.mrm_total_duration / driving_time

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
            positive_accels = [a for a in accels if a > 0]
            negative_accels = [a for a in accels if a < 0]

            m.max_acceleration = max(positive_accels) if positive_accels else 0.0
            m.max_deceleration = abs(min(negative_accels)) if negative_accels else 0.0
            m.hard_brake_count = sum(1 for a in negative_accels if abs(a) > 3.0)

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

        # Steering rate
        if self.steering_rates:
            rates = [r for _, r in self.steering_rates]
            m.max_steering_rate = max(rates)

        # Lateral acceleration (approximate: v^2 * curvature, or v * steering_rate)
        # For now, estimate from velocity and steering
        if self.velocities and self.steerings:
            max_lat_accel = 0.0
            # Wheelbase approximation (typical passenger car)
            wheelbase = 2.7  # meters

            for (t_v, vel), (t_s, steer) in zip(self.velocities, self.steerings):
                if abs(steer) > 0.001:  # Non-zero steering
                    # lateral_accel = v^2 * tan(steering) / wheelbase
                    lat_accel = abs(vel * vel * math.tan(steer) / wheelbase)
                    max_lat_accel = max(max_lat_accel, lat_accel)

            m.max_lateral_acceleration = max_lat_accel


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


def generate_summary_table(metrics_list: List[Dict], goal_ids: List[str]) -> str:
    """
    Generate a formatted summary table of experiment results.

    Args:
        metrics_list: List of metrics dictionaries
        goal_ids: List of goal IDs corresponding to metrics

    Returns:
        Formatted table string
    """
    lines = []

    # Header
    lines.append("=" * 140)
    lines.append("EXPERIMENT RESULTS SUMMARY")
    lines.append("=" * 140)
    lines.append("")

    # Main results table - new format with normalized metrics
    header = f"{'Goal':<12} {'Status':<8} {'Dist(km)':>9} {'Effic':>6} {'NrMiss/km':>10} {'MRM/km':>8} {'MRM%':>6} {'E-Stop%':>8} {'Clr(P5)':>8} {'PathDev':>8}"
    lines.append(header)
    lines.append("-" * 140)

    total_success = 0
    total_experiments = len(metrics_list)

    for goal_id, m in zip(goal_ids, metrics_list):
        rel = m.get('reliability', {})
        safe = m.get('safety', {})
        fail = m.get('fail_operational', {})

        status = "SUCCESS" if rel.get('goal_reached', False) else "FAILED"
        if rel.get('goal_reached', False):
            total_success += 1

        # Get metrics with new names
        distance_km = rel.get('distance_km', rel.get('total_distance_traveled', 0) / 1000)
        efficiency = rel.get('time_efficiency', 0)
        near_miss_rate = safe.get('near_miss_rate', 0)
        mrm_rate = fail.get('mrm_rate', 0)
        mrm_time_frac = fail.get('mrm_time_fraction', 0)
        estop_ratio = fail.get('estop_ratio', 0)
        clearance_p5 = safe.get('clearance_p5', safe.get('min_object_distance', float('inf')))
        path_dev = rel.get('lateral_rmse', 0)  # Path deviation from planned trajectory

        # Format values
        dist_str = f"{distance_km:.3f}" if distance_km else "-"
        eff_str = f"{efficiency:.2f}" if efficiency else "-"
        nmr_str = f"{near_miss_rate:.1f}" if near_miss_rate else "0.0"
        mrm_str = f"{mrm_rate:.0f}" if mrm_rate else "-"
        mrm_pct_str = f"{mrm_time_frac*100:.1f}%" if mrm_time_frac else "-"
        estop_str = f"{estop_ratio*100:.0f}%" if estop_ratio else "-"
        clr_str = f"{clearance_p5:.1f}m" if clearance_p5 < 1000 else "-"
        pathdev_str = f"{path_dev:.2f}m" if path_dev > 0 else "-"

        line = f"{goal_id:<12} {status:<8} {dist_str:>9} {eff_str:>6} {nmr_str:>10} {mrm_str:>8} {mrm_pct_str:>6} {estop_str:>8} {clr_str:>8} {pathdev_str:>8}"
        lines.append(line)

    lines.append("-" * 140)

    # Summary statistics
    lines.append("")
    lines.append("SUMMARY STATISTICS")
    lines.append("-" * 50)
    lines.append(f"Total experiments: {total_experiments}")
    lines.append(f"Successful: {total_success} ({100*total_success/total_experiments:.1f}%)")
    lines.append(f"Failed: {total_experiments - total_success} ({100*(total_experiments-total_success)/total_experiments:.1f}%)")

    # Compute aggregate metrics
    if metrics_list:
        distances_km = [m['reliability'].get('distance_km', m['reliability'].get('total_distance_traveled', 0)/1000)
                       for m in metrics_list if m['reliability'].get('total_distance_traveled', 0) > 0]
        times = [m['reliability'].get('driving_time', 0) for m in metrics_list if m['reliability'].get('driving_time', 0) > 0]
        efficiencies = [m['reliability'].get('time_efficiency', 0) for m in metrics_list if m['reliability'].get('time_efficiency', 0) > 0]
        clearances = [m['safety'].get('clearance_p5', m['safety'].get('min_object_distance', float('inf')))
                     for m in metrics_list if m['safety'].get('min_object_distance', float('inf')) < 1000]
        mrm_rates = [m['fail_operational'].get('mrm_rate', 0) for m in metrics_list if m['fail_operational'].get('mrm_rate', 0) > 0]
        mrm_time_fracs = [m['fail_operational'].get('mrm_time_fraction', 0) for m in metrics_list if m['fail_operational'].get('mrm_time_fraction', 0) > 0]
        estop_ratios = [m['fail_operational'].get('estop_ratio', 0) for m in metrics_list if m['fail_operational'].get('estop_ratio', 0) > 0]
        mrm_durations = [m['fail_operational'].get('avg_mrm_duration', 0) for m in metrics_list if m['fail_operational'].get('avg_mrm_duration', 0) > 0]

        lines.append("")
        if distances_km:
            lines.append(f"Total distance: {sum(distances_km):.2f} km")
            lines.append(f"Average distance: {sum(distances_km)/len(distances_km):.3f} km")
        if times:
            lines.append(f"Total driving time: {sum(times):.1f}s ({sum(times)/60:.1f} min)")
        if efficiencies:
            lines.append(f"Average time efficiency: {sum(efficiencies)/len(efficiencies):.2f} (1.0 = optimal)")
        if clearances:
            lines.append(f"Overall min clearance (P5): {min(clearances):.2f}m")
        if mrm_rates:
            lines.append(f"Average MRM rate: {sum(mrm_rates)/len(mrm_rates):.1f} MRM/km")
        if mrm_time_fracs:
            lines.append(f"Average MRM time: {sum(mrm_time_fracs)/len(mrm_time_fracs)*100:.1f}% of driving time")
        if estop_ratios:
            lines.append(f"Average E-stop ratio: {sum(estop_ratios)/len(estop_ratios)*100:.0f}%")
        if mrm_durations:
            lines.append(f"Average MRM duration: {sum(mrm_durations)/len(mrm_durations)*1000:.1f}ms")

    lines.append("")
    lines.append("=" * 140)
    lines.append("")
    lines.append("Column Legend:")
    lines.append("  Dist(km)   - Distance traveled in kilometers")
    lines.append("  Effic      - Time efficiency: expected/actual (1.0 = optimal, <1 = slower)")
    lines.append("  NrMiss/km  - Near miss events (TTC < 2s) per kilometer")
    lines.append("  MRM/km     - MRM triggers per kilometer")
    lines.append("  MRM%       - Percentage of driving time in MRM state")
    lines.append("  E-Stop%    - Emergency stops as percentage of all MRMs")
    lines.append("  Clr(P5)    - 5th percentile minimum clearance (meters)")
    lines.append("  PathDev    - Path deviation from planned trajectory (meters, lower = better tracking)")
    lines.append("")

    return "\n".join(lines)


def compute_all_metrics(data_dir: str, output_file: Optional[str] = None) -> Tuple[List[Dict], str]:
    """
    Compute metrics for all experiments in a directory and generate summary.

    Args:
        data_dir: Directory containing experiment folders
        output_file: Optional file to save summary to

    Returns:
        Tuple of (metrics_list, summary_table_string)
    """
    import os

    metrics_list = []
    goal_ids = []

    # Find all experiment directories
    exp_dirs = sorted([d for d in os.listdir(data_dir)
                      if os.path.isdir(os.path.join(data_dir, d)) and d.startswith('goal_')])

    print(f"Found {len(exp_dirs)} experiments in {data_dir}")

    for exp_dir in exp_dirs:
        exp_path = os.path.join(data_dir, exp_dir)
        rosbag_path = os.path.join(exp_path, 'rosbag')
        metrics_file = os.path.join(exp_path, 'metrics.json')

        # Extract goal ID
        parts = exp_dir.split('_')
        goal_id = f"{parts[0]}_{parts[1]}" if len(parts) >= 2 else exp_dir

        # Check if metrics already computed
        if os.path.exists(metrics_file):
            print(f"  Loading existing metrics for {goal_id}")
            metrics = load_metrics(metrics_file)
        elif os.path.exists(rosbag_path):
            print(f"  Computing metrics for {goal_id}...")
            try:
                exp_metrics = compute_metrics_from_bag(rosbag_path)
                metrics = exp_metrics.to_dict()
                save_metrics(exp_metrics, metrics_file)
            except Exception as e:
                print(f"    ERROR: {e}")
                continue
        else:
            print(f"  Skipping {goal_id} - no rosbag found")
            continue

        metrics_list.append(metrics)
        goal_ids.append(goal_id)

    # Generate summary table
    summary = generate_summary_table(metrics_list, goal_ids)

    if output_file:
        with open(output_file, 'w') as f:
            f.write(summary)
        print(f"\nSummary saved to: {output_file}")

    return metrics_list, summary
