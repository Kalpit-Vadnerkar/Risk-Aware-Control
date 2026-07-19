#!/usr/bin/env python3
"""
Capture the initial pose set via RViz's "2D Pose Estimate" tool.

Usage:
  1. Start AWSIM and Autoware with RViz
  2. Run this script: python3 capture_initial_pose.py
  3. In RViz, click "2D Pose Estimate" and set the pose
  4. The pose will be saved to captured_initial_pose.json

This allows you to later run headless with set_initial_pose.py

NOTE: RViz's 2D Pose Estimate sets Z=0. This script automatically gets
the correct Z from AWSIM's ground truth topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import json
import os
import sys


class InitialPoseCapture(Node):
    def __init__(self, output_file):
        super().__init__('initial_pose_capture')
        self.output_file = output_file
        self.captured = False
        self.awsim_z = None  # Z coordinate from AWSIM ground truth

        # Best effort QoS for AWSIM topics
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to AWSIM ground truth to get correct Z
        self.awsim_sub = self.create_subscription(
            Odometry,
            '/awsim/ground_truth/localization/kinematic_state',
            self.awsim_callback,
            best_effort_qos
        )

        # RViz publishes to /initialpose when you use 2D Pose Estimate
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_callback,
            10
        )

        self.get_logger().info('Waiting for initial pose from RViz...')
        self.get_logger().info('Use the "2D Pose Estimate" tool in RViz to set the pose.')
        self.get_logger().info('(Z coordinate will be auto-corrected from AWSIM ground truth)')

    def awsim_callback(self, msg):
        # Get Z from AWSIM ground truth
        self.awsim_z = msg.pose.pose.position.z

    def pose_callback(self, msg):
        pose = msg.pose.pose
        covariance = list(msg.pose.covariance)

        # Use AWSIM Z if available, otherwise use 41.58 (Shinjuku default)
        z_coord = self.awsim_z if self.awsim_z is not None else 41.58
        if self.awsim_z is None:
            self.get_logger().warn(f'AWSIM ground truth not available, using default Z={z_coord}')
        else:
            self.get_logger().info(f'Using Z={z_coord} from AWSIM ground truth')

        data = {
            'frame_id': msg.header.frame_id,
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': z_coord  # Use corrected Z, not RViz's Z=0
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            },
            'covariance': covariance
        }

        with open(self.output_file, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f'Initial pose captured!')
        self.get_logger().info(f'  Position: ({pose.position.x:.2f}, {pose.position.y:.2f}, {z_coord:.2f})')
        self.get_logger().info(f'  Orientation: (z={pose.orientation.z:.4f}, w={pose.orientation.w:.4f})')
        self.get_logger().info(f'  (Z corrected from {pose.position.z:.2f} to {z_coord:.2f})')
        self.get_logger().info(f'  Saved to: {self.output_file}')

        self.captured = True


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(script_dir, '..', 'configs')
    output_file = os.path.join(config_dir, 'captured_initial_pose.json')

    print("=" * 60)
    print("Initial Pose Capture Tool")
    print("=" * 60)
    print()
    print("Instructions:")
    print("  1. Make sure AWSIM and Autoware (with RViz) are running")
    print("  2. In RViz, click the '2D Pose Estimate' button")
    print("  3. Click and drag on the map to set vehicle position/heading")
    print("  4. The pose will be saved automatically")
    print()
    print("Press Ctrl+C to exit")
    print()

    rclpy.init()
    node = InitialPoseCapture(output_file)

    try:
        while rclpy.ok() and not node.captured:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.captured:
            print()
            print("=" * 60)
            print("SUCCESS: Initial pose captured and saved!")
            print(f"File: {output_file}")
            print()
            print("You can now use set_initial_pose.py to restore this pose")
            print("in headless mode (without RViz).")
            print("=" * 60)

    except KeyboardInterrupt:
        print("\nCapture cancelled.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
