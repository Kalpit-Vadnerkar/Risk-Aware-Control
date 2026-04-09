"""
PerceptionInterceptor ROS2 node for RISE validation scenarios.

Sits between Autoware's perception output and planning input.
Subscribes to the real perception topic, optionally injects fake objects
or applies faults, and publishes to the filtered topic that planning consumes.

Strategies (--strategy):
  - passthrough:          No modification (baseline)
  - static_obstacle:      Appends a stationary PredictedObject ahead of ego, on the planned route
  - cut_in:               Appends a time-varying object that moves into ego's lane
  - perception_delay:     Buffers messages and releases after N ms
  - perception_dropout:   Randomly removes objects from each message
  - position_noise:       Adds Gaussian noise to object positions

Fault overlay (--fault-strategy):
  Applied ON TOP of the scenario strategy.  Used for Phase 2/3 experiments where a
  scenario (obstacle) is combined with a sensor fault.  The fault is applied to ALL
  objects including the injected scenario object, so the planner may intermittently
  lose the obstacle.

  Supported fault overlay strategies: perception_dropout, position_noise
  (perception_delay as overlay is not supported — its buffering model is incompatible
  with synchronous scenario injection)

3-Experiment Pattern:
  Exp 1 (nominal):  --strategy static_obstacle --params '{"distance":30}'
  Exp 2 (fault):    --strategy static_obstacle --params '{"distance":30}'
                    --fault-strategy perception_dropout --fault-params '{"dropout_rate":0.3}'
  Exp 3 (RISE):     same as Exp 2, with RISE controller also active (future work)

Usage:
  python3 perception_interceptor.py --strategy passthrough
  python3 perception_interceptor.py --strategy static_obstacle --params '{"distance":30, "obj_type":"CAR"}'
  python3 perception_interceptor.py --strategy static_obstacle --params '{"distance":30}' \
      --fault-strategy perception_dropout --fault-params '{"dropout_rate":0.3}'
"""

import argparse
import copy
import json
import math
import random
import sys
import os
import time
import threading
from collections import deque
from typing import Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from autoware_perception_msgs.msg import PredictedObjects, ObjectClassification
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry

# Add lib to path for ObjectFactory/ObjectPlacer
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenarios import ObjectFactory, ObjectPlacer, ScenarioType


# QoS profile matching Autoware perception output
PERCEPTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


OBJ_TYPE_MAP = {
    'CAR': ObjectClassification.CAR,
    'TRUCK': ObjectClassification.TRUCK,
    'PEDESTRIAN': ObjectClassification.PEDESTRIAN,
    'BICYCLE': ObjectClassification.BICYCLE,
    'MOTORCYCLE': ObjectClassification.MOTORCYCLE,
}


class PerceptionInterceptor(Node):
    """ROS2 node that intercepts and modifies perception messages."""

    def __init__(self, strategy: str = 'passthrough', params: Optional[Dict[str, Any]] = None,
                 fault_strategy: Optional[str] = None,
                 fault_params: Optional[Dict[str, Any]] = None,
                 fault_delay: float = 0.0):
        super().__init__('perception_interceptor')

        self.strategy = strategy
        self.params = params or {}

        # Fault overlay (applied on top of scenario injection).
        # If fault_delay > 0, the fault starts in passthrough and activates after fault_delay
        # seconds. This avoids degrading the planning pipeline during cold-start initialization
        # (goal-set → trajectory → engage), which can trigger MRM before driving begins.
        if fault_delay > 0.0 and fault_strategy is not None:
            self.fault_strategy = None   # passthrough until timer fires
            self.fault_params = {}
            self._pending_fault_strategy = fault_strategy
            self._pending_fault_params = fault_params or {}
            self._fault_delay_timer = threading.Timer(fault_delay, self._activate_fault_overlay)
            self._fault_delay_timer.daemon = True
            self._fault_delay_timer.start()
            self.get_logger().info(
                f'Fault overlay scheduled in {fault_delay:.0f}s: '
                f'{fault_strategy} {fault_params}'
            )
        else:
            self.fault_strategy = fault_strategy
            self.fault_params = fault_params or {}
            self._pending_fault_strategy = None
            self._pending_fault_params = {}
            self._fault_delay_timer = None

        self._lock = threading.Lock()

        # Ego state (updated from localization)
        self._ego_x = 0.0
        self._ego_y = 0.0
        self._ego_z = 0.0
        self._ego_heading = 0.0
        self._ego_vx = 0.0
        self._ego_ready = False

        # Planned trajectory (used for route-based obstacle placement)
        self._latest_trajectory: Optional[Trajectory] = None

        # Strategy-specific state
        self._start_time = None
        self._delay_buffer = deque()
        self._injected_object_id = None  # Persistent UUID for injected objects
        self._msg_count = 0
        self._obstacle_world_pos = None   # Static obstacle: fixed world (x, y, z), set once
        self._obstacle_heading = 0.0      # Static obstacle: heading at placement time
        self._cutin_anchor = None          # Cut-in: (x, y, z, heading) captured at trigger time
        self._start_pos: Optional[Tuple[float, float]] = None  # Ego spawn position

        # Publisher: filtered topic that planning reads
        self._pub = self.create_publisher(
            PredictedObjects,
            '/perception/object_recognition/objects_filtered',
            PERCEPTION_QOS,
        )

        # Subscriber: real perception output
        self._sub = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self._on_perception,
            PERCEPTION_QOS,
        )

        # Subscriber: ego pose for object placement
        self._ego_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self._on_ego_pose,
            PERCEPTION_QOS,
        )

        # Subscriber: planned trajectory for route-based obstacle placement
        self._traj_sub = self.create_subscription(
            Trajectory,
            '/planning/scenario_planning/trajectory',
            self._on_trajectory,
            PERCEPTION_QOS,
        )

        # For delay strategy: timer to flush buffer
        if strategy == 'perception_delay':
            delay_ms = self.params.get('delay_ms', 0)
            if delay_ms > 0:
                # Check at 10ms intervals
                self._delay_timer = self.create_timer(0.01, self._flush_delay_buffer)

        self.get_logger().info(
            f'PerceptionInterceptor started: strategy={strategy}, params={params}, '
            f'fault_strategy={fault_strategy}, fault_params={fault_params}'
        )

    def _activate_fault_overlay(self):
        """Activate the deferred fault overlay. Called from a threading.Timer."""
        with self._lock:
            self.fault_strategy = self._pending_fault_strategy
            self.fault_params = self._pending_fault_params
        self.get_logger().info(
            f'Fault overlay activated: {self.fault_strategy} {self.fault_params}'
        )

    def _on_ego_pose(self, msg: Odometry):
        """Update ego state from localization."""
        with self._lock:
            self._ego_x = msg.pose.pose.position.x
            self._ego_y = msg.pose.pose.position.y
            self._ego_z = msg.pose.pose.position.z
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self._ego_heading = 2.0 * math.atan2(qz, qw)
            self._ego_vx = msg.twist.twist.linear.x
            if not self._ego_ready:
                self._start_pos = (self._ego_x, self._ego_y)
            self._ego_ready = True

    def _on_trajectory(self, msg: Trajectory):
        """Store latest planned trajectory for route-based placement."""
        with self._lock:
            self._latest_trajectory = msg

    def _find_trajectory_point_at_distance(self, distance: float,
                                             margin: float = 30.0) -> Optional[Tuple[float, float, float, float]]:
        """Return (x, y, z, yaw) at arc-length `distance` ahead of ego along the trajectory.

        Finds the trajectory point closest to ego first, then walks forward `distance`
        metres from that point.  This avoids the systematic under-estimation that
        occurs when measuring from trajectory[0], which is typically 10-30 m behind ego.

        Returns None if:
          - trajectory unavailable / too short
          - the remaining trajectory ahead of ego is shorter than `distance + margin`
            (prevents placing at the planning-horizon boundary / intersection node)
        """
        traj = self._latest_trajectory
        if traj is None or len(traj.points) < 2:
            return None

        points = traj.points
        ego_x, ego_y = self._ego_x, self._ego_y

        # --- Step 1: find trajectory index closest to ego ---
        min_d2 = float('inf')
        ego_idx = 0
        for i, pt in enumerate(points):
            dx = pt.pose.position.x - ego_x
            dy = pt.pose.position.y - ego_y
            d2 = dx * dx + dy * dy
            if d2 < min_d2:
                min_d2 = d2
                ego_idx = i

        # --- Step 2: walk forward from ego_idx ---
        arc_len = 0.0
        result = None

        for i in range(ego_idx, len(points) - 1):
            p0 = points[i].pose.position
            p1 = points[i + 1].pose.position
            seg_len = math.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2)

            if result is None and arc_len + seg_len >= distance:
                t = (distance - arc_len) / seg_len if seg_len > 1e-6 else 0.0
                x = p0.x + t * (p1.x - p0.x)
                y = p0.y + t * (p1.y - p0.y)
                z = p0.z + t * (p1.z - p0.z)
                yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)
                result = (x, y, z, yaw)

            arc_len += seg_len

        # Require at least `margin` metres of trajectory beyond placement point.
        # This prevents placing the obstacle right at the planning-horizon boundary.
        if result is not None and arc_len >= distance + margin:
            return result

        return None

    def _on_perception(self, msg: PredictedObjects):
        """Process incoming perception message based on strategy."""
        if self._start_time is None:
            self._start_time = time.monotonic()

        self._msg_count += 1

        if self.strategy == 'passthrough':
            if self.fault_strategy:
                msg = self._apply_fault_to_msg(msg)
            self._pub.publish(msg)

        elif self.strategy == 'static_obstacle':
            self._apply_static_obstacle(msg)

        elif self.strategy == 'cut_in':
            self._apply_cut_in(msg)

        elif self.strategy == 'perception_delay':
            self._apply_delay(msg)

        elif self.strategy == 'perception_dropout':
            self._apply_dropout(msg)

        elif self.strategy == 'position_noise':
            self._apply_position_noise(msg)

        else:
            self.get_logger().warn(f'Unknown strategy: {self.strategy}, using passthrough')
            self._pub.publish(msg)

    # ── Fault overlay (applied on top of scenario injection) ─────────

    def _apply_fault_to_msg(self, msg: PredictedObjects) -> PredictedObjects:
        """Apply fault transformation to all objects in message (including injected ones).

        Returns a new message with fault applied.  The input message is not modified.
        perception_delay as overlay is not supported here.
        """
        if not msg.objects:
            return msg

        if self.fault_strategy == 'perception_dropout':
            rate = self.fault_params.get('dropout_rate', 0.0)
            if rate <= 0.0:
                return msg
            out = copy.deepcopy(msg)
            out.objects = [obj for obj in out.objects if random.random() > rate]
            return out

        elif self.fault_strategy == 'position_noise':
            std = self.fault_params.get('noise_std', 0.0)
            if std <= 0.0:
                return msg
            out = copy.deepcopy(msg)
            for obj in out.objects:
                pose = obj.kinematics.initial_pose_with_covariance.pose
                pose.position.x += random.gauss(0, std)
                pose.position.y += random.gauss(0, std)
            return out

        return msg

    # ── Static Obstacle ──────────────────────────────────────────────

    def _apply_static_obstacle(self, msg: PredictedObjects):
        """Append a stationary object ahead of ego, placed on the planned route.

        Position is locked in world space on the first call after the planned
        trajectory becomes available.  This guarantees the obstacle lands on the
        actual road segment the vehicle intends to follow, not just along the
        current heading which may not follow road curvature.

        Falls back to heading-projection if the trajectory is unavailable.
        """
        with self._lock:
            if not self._ego_ready:
                self._pub.publish(msg)
                return

            obj_type_str = self.params.get('obj_type', 'CAR')
            obj_type = OBJ_TYPE_MAP.get(obj_type_str, ObjectClassification.CAR)

            # Lock position on first call — obstacle must not follow ego.
            # Prefer trajectory-based placement; fall back to heading projection.
            if self._obstacle_world_pos is None:
                distance = self.params.get('distance', 100.0)
                lateral_offset = self.params.get('lateral_offset', 0.0)
                min_travel = self.params.get('min_travel_before_placement', 80.0)

                # Defer placement until ego has cleared the spawn area.
                # This ensures the trajectory is settled and the obstacle lands
                # on a meaningful road segment rather than in the start intersection.
                if self._start_pos is not None:
                    ego_travel = math.sqrt(
                        (self._ego_x - self._start_pos[0]) ** 2 +
                        (self._ego_y - self._start_pos[1]) ** 2
                    )
                    if ego_travel < min_travel:
                        self._pub.publish(msg)
                        return

                traj_point = self._find_trajectory_point_at_distance(distance)

                if traj_point is not None:
                    import uuid as _uuid
                    ox, oy, oz, traj_yaw = traj_point
                    # Apply lateral offset along the trajectory direction
                    if lateral_offset != 0.0:
                        ox -= lateral_offset * math.sin(traj_yaw)
                        oy += lateral_offset * math.cos(traj_yaw)
                    self._obstacle_world_pos = (ox, oy, oz)
                    self._obstacle_heading = traj_yaw
                    # Generate UUID now so it's stable and can be written to info file
                    self._injected_object_id = _uuid.uuid4().bytes
                    self.get_logger().info(
                        f'Static obstacle placed on trajectory at world ({ox:.1f}, {oy:.1f}) '
                        f'({distance:.0f}m arc-length ahead of ego)'
                    )
                    self._write_obstacle_info(ox, oy, oz, traj_yaw)
                else:
                    # Trajectory not yet available — defer placement
                    self._pub.publish(msg)
                    return

            ox, oy, oz = self._obstacle_world_pos
            obstacle_heading = self._obstacle_heading

        fake_obj = ObjectFactory.create_object(
            x=ox, y=oy, z=oz,
            yaw=obstacle_heading,
            obj_type=obj_type,
            object_id=self._injected_object_id,
        )

        out = copy.deepcopy(msg)
        out.objects.append(fake_obj)

        # Apply fault overlay to all objects including the injected one
        if self.fault_strategy:
            out = self._apply_fault_to_msg(out)

        self._pub.publish(out)

    def _write_obstacle_info(self, x: float, y: float, z: float, heading: float):
        """Write injected obstacle info to a temp file for post-experiment metadata."""
        info = {
            'uuid_hex': bytes(self._injected_object_id).hex(),
            'world_x': x,
            'world_y': y,
            'world_z': z,
            'heading': heading,
            'placement_time': time.time(),
            'strategy': self.strategy,
            'params': self.params,
        }
        try:
            with open('/tmp/rise_obstacle_info.json', 'w') as f:
                json.dump(info, f)
        except Exception as exc:
            self.get_logger().warn(f'Could not write obstacle info: {exc}')

    # ── Cut-in ────────────────────────────────────────────────────────

    def _apply_cut_in(self, msg: PredictedObjects):
        """Inject an object that cuts into ego's lane over time."""
        elapsed = time.monotonic() - self._start_time
        trigger_time = self.params.get('trigger_time', 5.0)
        duration = self.params.get('cut_in_duration', 3.0)

        if elapsed < trigger_time:
            self._pub.publish(msg)
            return

        cut_in_elapsed = elapsed - trigger_time

        if cut_in_elapsed > duration + 5.0:
            # Cut-in complete + 5s linger, stop injecting
            self._pub.publish(msg)
            return

        progress = min(1.0, cut_in_elapsed / duration)

        with self._lock:
            if not self._ego_ready:
                self._pub.publish(msg)
                return

            longitudinal_distance = self.params.get('distance', 60.0)
            lateral_start = self.params.get('lateral_start', 3.5)
            lateral_end = self.params.get('lateral_end', 0.0)
            obj_type_str = self.params.get('obj_type', 'CAR')
            obj_type = OBJ_TYPE_MAP.get(obj_type_str, ObjectClassification.CAR)

            # Lock longitudinal anchor at trigger time using trajectory if available
            if self._cutin_anchor is None:
                traj_point = self._find_trajectory_point_at_distance(longitudinal_distance)
                if traj_point is not None:
                    ax, ay, az, anchor_yaw = traj_point
                    self._cutin_anchor = (ax, ay, az, anchor_yaw)
                    self.get_logger().info(
                        f'Cut-in anchored on trajectory at ({ax:.1f}, {ay:.1f}), '
                        f'{longitudinal_distance:.0f}m ahead'
                    )
                else:
                    # Fallback: anchor relative to ego heading at trigger time
                    self._cutin_anchor = (
                        self._ego_x, self._ego_y, self._ego_z, self._ego_heading
                    )
                    self.get_logger().info(
                        f'Cut-in anchored at ego pos ({self._ego_x:.1f}, {self._ego_y:.1f}), '
                        f'{longitudinal_distance:.0f}m ahead (heading fallback)'
                    )

            anchor_x, anchor_y, anchor_z, anchor_heading = self._cutin_anchor

            ox, oy, oz = ObjectPlacer.cut_in_position(
                anchor_x, anchor_y, anchor_z, anchor_heading,
                longitudinal_distance, lateral_start, lateral_end, progress,
            )

            if progress < 1.0:
                lateral_speed = abs(lateral_start - lateral_end) / duration
            else:
                lateral_speed = 0.0

            obj_vx = self._ego_vx
            obj_vy = -lateral_speed if lateral_start > 0 else lateral_speed

        if self._injected_object_id is None:
            import uuid
            self._injected_object_id = uuid.uuid4().bytes

        fake_obj = ObjectFactory.create_object(
            x=ox, y=oy, z=oz,
            yaw=self._ego_heading,
            vx=obj_vx, vy=obj_vy,
            obj_type=obj_type,
            object_id=self._injected_object_id,
        )

        out = copy.deepcopy(msg)
        out.objects.append(fake_obj)

        if self.fault_strategy:
            out = self._apply_fault_to_msg(out)

        self._pub.publish(out)

    # ── Perception Delay ──────────────────────────────────────────────

    def _apply_delay(self, msg: PredictedObjects):
        """Buffer messages and release after delay_ms."""
        delay_ms = self.params.get('delay_ms', 0)
        if delay_ms <= 0:
            self._pub.publish(msg)
            return

        release_time = time.monotonic() + delay_ms / 1000.0
        self._delay_buffer.append((release_time, msg))

    def _flush_delay_buffer(self):
        """Timer callback: publish any messages whose delay has expired."""
        now = time.monotonic()
        while self._delay_buffer and self._delay_buffer[0][0] <= now:
            _, msg = self._delay_buffer.popleft()
            self._pub.publish(msg)

    # ── Perception Dropout ────────────────────────────────────────────

    def _apply_dropout(self, msg: PredictedObjects):
        """Randomly remove objects from the message."""
        dropout_rate = self.params.get('dropout_rate', 0.0)

        if dropout_rate <= 0.0 or not msg.objects:
            self._pub.publish(msg)
            return

        out = copy.deepcopy(msg)
        out.objects = [obj for obj in out.objects if random.random() > dropout_rate]
        self._pub.publish(out)

    # ── Position Noise ────────────────────────────────────────────────

    def _apply_position_noise(self, msg: PredictedObjects):
        """Add Gaussian noise to object positions."""
        noise_std = self.params.get('noise_std', 0.0)

        if noise_std <= 0.0 or not msg.objects:
            self._pub.publish(msg)
            return

        out = copy.deepcopy(msg)
        for obj in out.objects:
            pose = obj.kinematics.initial_pose_with_covariance.pose
            pose.position.x += random.gauss(0, noise_std)
            pose.position.y += random.gauss(0, noise_std)
        self._pub.publish(out)


def main():
    parser = argparse.ArgumentParser(description='Perception Interceptor for RISE experiments')
    parser.add_argument('--strategy', type=str, default='passthrough',
                        choices=['passthrough', 'static_obstacle', 'cut_in',
                                 'perception_delay', 'perception_dropout', 'position_noise'],
                        help='Scenario injection strategy')
    parser.add_argument('--params', type=str, default='{}',
                        help='JSON string of scenario strategy parameters')
    parser.add_argument('--fault-strategy', type=str, default=None,
                        choices=['perception_dropout', 'position_noise'],
                        help='Fault overlay applied on top of scenario (for fault+scenario experiments)')
    parser.add_argument('--fault-params', type=str, default='{}',
                        help='JSON string of fault overlay parameters')
    parser.add_argument('--fault-delay', type=float, default=0.0,
                        help='Seconds to wait before activating fault overlay (0 = immediate). '
                             'Use ~45s to avoid stressing planning during cold-start engage.')
    args = parser.parse_args()

    params = json.loads(args.params)
    fault_params = json.loads(args.fault_params)

    rclpy.init()
    node = PerceptionInterceptor(
        strategy=args.strategy,
        params=params,
        fault_strategy=args.fault_strategy,
        fault_params=fault_params,
        fault_delay=args.fault_delay,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish an empty objects message so planning clears its object buffer
        # immediately rather than waiting for its internal stale-object timeout.
        try:
            from autoware_perception_msgs.msg import PredictedObjects
            import std_msgs.msg as std_msgs
            empty_msg = PredictedObjects()
            empty_msg.header.stamp = node.get_clock().now().to_msg()
            empty_msg.header.frame_id = 'map'
            node._pub.publish(empty_msg)
        except Exception:
            pass
        node.get_logger().info(
            f'Shutting down. Processed {node._msg_count} messages.'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
