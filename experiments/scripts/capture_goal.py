#!/usr/bin/env python3
"""
Capture goal coordinates from a manual RViz session.

Usage:
  1. Launch AWSIM and Autoware manually
  2. Run this script: python3 capture_goal.py
  3. Set goal in RViz using "2D Goal Pose" tool
  4. Script will print and save the coordinates

The script also captures initial pose for reference.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import json
import os
from datetime import datetime


class GoalCapture(Node):
    def __init__(self):
        super().__init__('goal_capture')

        self.captured_data = {
            'timestamp': datetime.now().isoformat(),
            'initial_pose': None,
            'goal_pose': None,
            'current_pose': None
        }

        # Subscribe to goal topic
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/planning/mission_planning/goal',
            self.goal_callback,
            10
        )

        # Alternative goal topic
        self.goal_sub2 = self.create_subscription(
            PoseStamped,
            '/rviz/goal_pose',
            self.goal_callback,
            10
        )

        # QoS for AWSIM ground truth (publishes BEST_EFFORT)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to current pose (ground truth)
        self.pose_sub = self.create_subscription(
            Odometry,
            '/awsim/ground_truth/localization/kinematic_state',
            self.pose_callback,
            best_effort_qos
        )

        # Subscribe to localization pose
        self.loc_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_with_covariance',
            self.localization_callback,
            10
        )

        self.get_logger().info('Goal Capture Node started')
        self.get_logger().info('Waiting for goal... Set a goal in RViz using "2D Goal Pose"')
        self.get_logger().info('Press Ctrl+C when done to save captured data')

    def goal_callback(self, msg):
        pose = msg.pose
        self.captured_data['goal_pose'] = {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            },
            'frame_id': msg.header.frame_id
        }
        self.get_logger().info(f'\n=== GOAL CAPTURED ===')
        self.get_logger().info(f'Position: x={pose.position.x:.2f}, y={pose.position.y:.2f}, z={pose.position.z:.2f}')
        self.get_logger().info(f'Orientation: z={pose.orientation.z:.4f}, w={pose.orientation.w:.4f}')
        self.get_logger().info(f'Frame: {msg.header.frame_id}')

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.captured_data['current_pose'] = {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            }
        }

    def localization_callback(self, msg):
        if self.captured_data['initial_pose'] is None:
            pose = msg.pose.pose
            self.captured_data['initial_pose'] = {
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                }
            }
            self.get_logger().info(f'Initial pose captured: x={pose.position.x:.2f}, y={pose.position.y:.2f}')

    def save_data(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_file = os.path.join(script_dir, '..', 'configs', 'captured_route.json')

        with open(output_file, 'w') as f:
            json.dump(self.captured_data, f, indent=2)

        self.get_logger().info(f'\nData saved to: {output_file}')

        # Also print the goal command for easy copy-paste
        if self.captured_data['goal_pose']:
            g = self.captured_data['goal_pose']
            cmd = f"""
# To set this goal programmatically:
ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped "{{
  header: {{frame_id: 'map'}},
  pose: {{
    position: {{x: {g['position']['x']}, y: {g['position']['y']}, z: {g['position']['z']}}},
    orientation: {{x: {g['orientation']['x']}, y: {g['orientation']['y']}, z: {g['orientation']['z']}, w: {g['orientation']['w']}}}
  }}
}}" --once
"""
            print(cmd)


def main():
    rclpy.init()
    node = GoalCapture()

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
