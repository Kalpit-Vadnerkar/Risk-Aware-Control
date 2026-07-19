#!/usr/bin/env python3
"""
Set goal pose via the Autoware ADAPI routing service.

Uses /api/routing/set_route_points (the correct ADAPI service) instead of
publishing to /planning/mission_planning/goal (which does NOT trigger routing).

Also verifies route acceptance by monitoring /api/routing/state.

Exit codes:
  0 = Route set and accepted
  1 = Route setting failed (service returned error)
  2 = Route not accepted within timeout
  3 = Error (missing messages, service unavailable, etc.)

Usage:
  python3 set_goal.py <goal_x> <goal_y> <goal_z> <ori_z> <ori_w> [timeout]
  Default timeout: 60 seconds
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import sys
import time

try:
    from autoware_adapi_v1_msgs.srv import SetRoutePoints
    from autoware_adapi_v1_msgs.msg import RouteState
    HAS_ADAPI = True
except ImportError:
    HAS_ADAPI = False

from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class GoalSetter(Node):
    def __init__(self, goal_x, goal_y, goal_z, ori_z, ori_w, timeout):
        super().__init__('goal_setter')

        if not HAS_ADAPI:
            self.get_logger().error(
                "autoware_adapi_v1_msgs not found. "
                "Source autoware/install/setup.bash first."
            )
            raise SystemExit(3)

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_z = goal_z
        self.ori_z = ori_z
        self.ori_w = ori_w
        self.timeout = timeout
        self.start_time = time.time()
        self.route_state = None
        self.route_set = False
        self.service_called = False
        self.done = False

        STATE_NAMES = {0: "UNKNOWN", 1: "UNSET", 2: "SET", 3: "ARRIVED", 4: "CHANGING"}
        self.STATE_NAMES = STATE_NAMES

        # Subscribe to route state - must match publisher QoS (TRANSIENT_LOCAL)
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            RouteState,
            '/api/routing/state',
            self.route_state_callback,
            state_qos
        )

        # Service client
        self.client = self.create_client(
            SetRoutePoints,
            '/api/routing/set_route_points'
        )

        # Timer to drive the state machine
        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info(
            f'Goal setter: ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f}) '
            f'ori=({ori_z:.4f}, {ori_w:.4f}) timeout={timeout}s'
        )

    def route_state_callback(self, msg):
        new_state = msg.state
        if new_state != self.route_state:
            old_name = self.STATE_NAMES.get(self.route_state, "NONE")
            new_name = self.STATE_NAMES.get(new_state, f"STATE_{new_state}")
            elapsed = time.time() - self.start_time
            print(f'  [{elapsed:.0f}s] Route state: {old_name} -> {new_name}')
            self.route_state = new_state

        # Success: route is now SET after we called the service
        if self.service_called and new_state == 2:  # SET
            self.route_set = True
            if not self.done:
                self.done = True
                elapsed = time.time() - self.start_time
                print(f'\nRoute SET successfully after {elapsed:.1f}s')
                raise SystemExit(0)

    def tick(self):
        elapsed = time.time() - self.start_time

        # Timeout check
        if elapsed >= self.timeout:
            if not self.service_called:
                print(f'\nTimeout: Could not call set_route_points service within {self.timeout}s')
                raise SystemExit(3)
            elif not self.route_set:
                state_name = self.STATE_NAMES.get(self.route_state, "UNKNOWN")
                print(f'\nTimeout: Route not accepted within {self.timeout}s (state: {state_name})')
                raise SystemExit(2)

        # Wait for route state to be known and UNSET before calling service
        if not self.service_called:
            if self.route_state is None:
                if elapsed > 5:
                    print(f'  [{elapsed:.0f}s] Waiting for /api/routing/state topic...')
                return

            if self.route_state != 1:  # Not UNSET
                state_name = self.STATE_NAMES.get(self.route_state, "UNKNOWN")
                print(f'  [{elapsed:.0f}s] Route state is {state_name}, waiting for UNSET...')
                return

            # Route is UNSET, call the service
            self.call_set_route()

    def call_set_route(self):
        if not self.client.service_is_ready():
            elapsed = time.time() - self.start_time
            print(f'  [{elapsed:.0f}s] Waiting for /api/routing/set_route_points service...')
            return

        # Build request
        request = SetRoutePoints.Request()

        # Header
        request.header = Header()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'

        # Goal pose
        request.goal = Pose()
        request.goal.position = Point(
            x=self.goal_x, y=self.goal_y, z=self.goal_z
        )
        request.goal.orientation = Quaternion(
            x=0.0, y=0.0, z=self.ori_z, w=self.ori_w
        )

        # No waypoints (direct route to goal)
        request.waypoints = []

        # Route option - allow goal modification for better lanelet snapping
        request.option.allow_goal_modification = True

        print(f'\n  Calling /api/routing/set_route_points...')
        print(f'    Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_z:.2f})')
        print(f'    Orientation: (z={self.ori_z:.4f}, w={self.ori_w:.4f})')

        self.service_called = True
        future = self.client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.status.success:
                print(f'  Service response: SUCCESS - {response.status.message}')
                # Wait for route state to confirm SET (handled in route_state_callback)
            else:
                print(f'  Service response: FAILED')
                print(f'    Code: {response.status.code}')
                print(f'    Message: {response.status.message}')
                raise SystemExit(1)
        except SystemExit:
            raise
        except Exception as e:
            print(f'  Service call exception: {e}')
            raise SystemExit(3)


def main():
    if len(sys.argv) < 6:
        print("Usage: python3 set_goal.py <goal_x> <goal_y> <goal_z> <ori_z> <ori_w> [timeout]")
        print("  Timeout default: 30 seconds")
        sys.exit(3)

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_z = float(sys.argv[3])
    ori_z = float(sys.argv[4])
    ori_w = float(sys.argv[5])
    timeout = float(sys.argv[6]) if len(sys.argv) > 6 else 60.0

    rclpy.init()
    node = GoalSetter(goal_x, goal_y, goal_z, ori_z, ori_w, timeout)

    exit_code = 3
    try:
        rclpy.spin(node)
    except SystemExit as e:
        exit_code = e.code if e.code is not None else 0
    except KeyboardInterrupt:
        exit_code = 3
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
