#!/usr/bin/env python3
"""
Capture multiple goals from an interactive RViz session.

Usage:
  1. Launch AWSIM and Autoware with RViz
  2. Run this script: python3 capture_goals_session.py
  3. Set goals in RViz using "2D Goal Pose" tool
  4. Each accepted goal is automatically recorded
  5. Press Ctrl+C when done to save all goals

The script captures:
  - Each goal that gets accepted (route state -> SET)
  - Start position when goal was set
  - Goal ID for easy reference

Output: experiments/configs/captured_goals.json
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import json
import os
from datetime import datetime

try:
    from autoware_adapi_v1_msgs.msg import RouteState
    HAS_ADAPI = True
except ImportError:
    HAS_ADAPI = False
    print("WARNING: autoware_adapi_v1_msgs not found. Route state monitoring disabled.")


class MultiGoalCapture(Node):
    def __init__(self):
        super().__init__('multi_goal_capture')

        # All captured goals
        self.goals = []
        self.goal_counter = 0

        # Current capture state
        self.pending_goal = None  # Goal waiting to be accepted
        self.current_position = None
        self.route_state = 0
        self.last_route_state = 0

        # QoS profiles
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to echo-back goal (what Autoware accepts after lanelet snapping)
        self.echo_sub = self.create_subscription(
            PoseStamped,
            '/planning/mission_planning/echo_back_goal_pose',
            self.echo_goal_callback,
            transient_qos
        )

        # Subscribe to route state
        if HAS_ADAPI:
            self.route_sub = self.create_subscription(
                RouteState,
                '/api/routing/state',
                self.route_state_callback,
                transient_qos
            )

        # Subscribe to current pose (ground truth from AWSIM)
        self.pose_sub = self.create_subscription(
            Odometry,
            '/awsim/ground_truth/localization/kinematic_state',
            self.pose_callback,
            best_effort_qos
        )

        # Timer for status display
        self.status_timer = self.create_timer(5.0, self.print_status)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Multi-Goal Capture Session Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('Instructions:')
        self.get_logger().info('  1. Set a goal in RViz using "2D Goal Pose"')
        self.get_logger().info('  2. Wait for "GOAL CAPTURED" message')
        self.get_logger().info('  3. Clear route: ros2 service call /api/routing/clear_route ...')
        self.get_logger().info('     Or use: python3 clear_route.py')
        self.get_logger().info('  4. Repeat for more goals')
        self.get_logger().info('  5. Press Ctrl+C when done to save all goals')
        self.get_logger().info('')

    def echo_goal_callback(self, msg):
        """Capture what Autoware actually accepts (after lanelet snapping)"""
        pose = msg.pose

        # Store as pending until route is confirmed
        self.pending_goal = {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'z': pose.orientation.z,
                'w': pose.orientation.w
            },
            'frame_id': msg.header.frame_id,
            'start_position': self.current_position.copy() if self.current_position else None,
            'timestamp': datetime.now().isoformat()
        }

        self.get_logger().info(f'\nGoal received: ({pose.position.x:.2f}, {pose.position.y:.2f})')
        self.get_logger().info('Waiting for route acceptance...')

    def route_state_callback(self, msg):
        """Monitor route state to know when route is accepted"""
        STATE_NAMES = {0: "UNKNOWN", 1: "UNSET", 2: "SET", 3: "ARRIVED", 4: "CHANGING"}

        self.last_route_state = self.route_state
        self.route_state = msg.state

        # Detect transition to SET (route accepted)
        if self.last_route_state != 2 and msg.state == 2 and self.pending_goal:
            self.goal_counter += 1
            goal_id = f"goal_{self.goal_counter:03d}"

            goal_entry = {
                'id': goal_id,
                'goal': self.pending_goal,
                'captured_at': datetime.now().isoformat()
            }

            # Calculate distance from start if we have start position
            if self.pending_goal.get('start_position'):
                start = self.pending_goal['start_position']
                end = self.pending_goal['position']
                import math
                dist = math.sqrt((end['x'] - start['x'])**2 + (end['y'] - start['y'])**2)
                goal_entry['estimated_distance'] = round(dist, 1)

            self.goals.append(goal_entry)
            self.pending_goal = None

            self.get_logger().info('')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'*** GOAL CAPTURED: {goal_id} ***')
            self.get_logger().info(f"    Position: ({goal_entry['goal']['position']['x']:.2f}, "
                                  f"{goal_entry['goal']['position']['y']:.2f})")
            if 'estimated_distance' in goal_entry:
                self.get_logger().info(f"    Est. distance: {goal_entry['estimated_distance']:.0f}m")
            self.get_logger().info(f'    Total goals captured: {len(self.goals)}')
            self.get_logger().info('=' * 60)
            self.get_logger().info('')
            self.get_logger().info('Clear the route before setting the next goal:')
            self.get_logger().info('  ros2 service call /api/routing/clear_route autoware_adapi_v1_msgs/srv/ClearRoute')
            self.get_logger().info('')

        elif msg.state == 1:  # UNSET
            if self.last_route_state == 2:
                self.get_logger().info('Route cleared. Ready for next goal.')

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.current_position = {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z
        }

    def print_status(self):
        """Print periodic status"""
        state_names = {0: "UNKNOWN", 1: "UNSET", 2: "SET", 3: "ARRIVED", 4: "CHANGING"}
        state_name = state_names.get(self.route_state, f"?{self.route_state}")

        pos_str = "unknown"
        if self.current_position:
            pos_str = f"({self.current_position['x']:.0f}, {self.current_position['y']:.0f})"

        print(f"\r[Status] Goals: {len(self.goals)} | Route: {state_name} | Position: {pos_str}    ",
              end='', flush=True)

    def save_data(self):
        """Save all captured goals to file"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_file = os.path.join(script_dir, '..', 'configs', 'captured_goals.json')
        os.makedirs(os.path.dirname(output_file), exist_ok=True)

        output_data = {
            'capture_session': datetime.now().isoformat(),
            'total_goals': len(self.goals),
            'goals': self.goals
        }

        with open(output_file, 'w') as f:
            json.dump(output_data, f, indent=2)

        print('')
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'SESSION COMPLETE')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Total goals captured: {len(self.goals)}')
        self.get_logger().info(f'Saved to: {output_file}')
        self.get_logger().info('')

        if self.goals:
            self.get_logger().info('Captured Goals:')
            for g in self.goals:
                pos = g['goal']['position']
                dist_str = f" (~{g['estimated_distance']:.0f}m)" if 'estimated_distance' in g else ""
                self.get_logger().info(f"  {g['id']}: ({pos['x']:.2f}, {pos['y']:.2f}){dist_str}")

            self.get_logger().info('')
            self.get_logger().info('To run experiments on these goals:')
            self.get_logger().info('  python3 run_batch_experiments.py')


def main():
    rclpy.init()
    node = MultiGoalCapture()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
