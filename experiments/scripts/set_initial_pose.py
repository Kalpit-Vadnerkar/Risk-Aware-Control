#!/usr/bin/env python3
"""
Set the initial pose programmatically (without RViz).

This script:
1. Publishes to /initialpose (same as RViz's "2D Pose Estimate")
2. AWSIM receives this and teleports the vehicle
3. Autoware re-initializes localization at the new position
4. Verifies the vehicle actually moved to the target position

Usage:
  python3 set_initial_pose.py [--config path/to/pose.json] [--wait] [--timeout 30]

Options:
  --config   Path to JSON file with pose data (default: captured_initial_pose.json)
  --wait     Wait for localization to stabilize after setting pose
  --timeout  Timeout in seconds for localization wait (default: 30)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import json
import os
import sys
import argparse
import time
import math


class InitialPoseSetter(Node):
    def __init__(self, pose_data, wait_for_localization=False, timeout=30.0):
        super().__init__('initial_pose_setter')
        self.pose_data = pose_data
        self.wait_for_localization = wait_for_localization
        self.timeout = timeout

        # Position tracking
        self.current_position = None
        self.awsim_position = None
        self.odom_count = 0
        self.last_odom_time = None

        # QoS matching RViz's initialpose publisher
        # RViz uses: reliable, volatile, depth 1
        qos_initialpose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher for initial pose - same topic RViz uses
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_initialpose
        )

        # Best effort QoS for AWSIM ground truth
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to AWSIM ground truth to verify teleport
        self.awsim_sub = self.create_subscription(
            Odometry,
            '/awsim/ground_truth/localization/kinematic_state',
            self.awsim_callback,
            qos_best_effort
        )

        # Subscribe to Autoware localization to verify it updated
        self.odom_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            qos_best_effort
        )

        self.get_logger().info('InitialPoseSetter initialized')
        self.get_logger().info('Subscribed to /awsim/ground_truth/localization/kinematic_state')
        self.get_logger().info('Subscribed to /localization/kinematic_state')

    def awsim_callback(self, msg):
        """Track AWSIM ground truth position."""
        self.awsim_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    def odom_callback(self, msg):
        """Track Autoware localization position."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        self.odom_count += 1
        self.last_odom_time = time.time()

    def get_current_position(self, timeout=5.0):
        """Get current position from AWSIM ground truth."""
        self.awsim_position = None
        start = time.time()

        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.awsim_position is not None:
                return self.awsim_position

        return None

    def calculate_distance(self, pos1, pos2):
        """Calculate 2D distance between two positions."""
        if pos1 is None or pos2 is None:
            return float('inf')
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def publish_pose(self):
        """Publish the initial pose message."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.pose_data.get('frame_id', 'map')

        pos = self.pose_data['position']
        ori = self.pose_data['orientation']

        msg.pose.pose.position.x = float(pos['x'])
        msg.pose.pose.position.y = float(pos['y'])
        msg.pose.pose.position.z = float(pos['z'])

        msg.pose.pose.orientation.x = float(ori.get('x', 0.0))
        msg.pose.pose.orientation.y = float(ori.get('y', 0.0))
        msg.pose.pose.orientation.z = float(ori['z'])
        msg.pose.pose.orientation.w = float(ori['w'])

        # Use saved covariance or default
        if 'covariance' in self.pose_data:
            msg.pose.covariance = list(self.pose_data['covariance'])
        else:
            # Default covariance (same as RViz default)
            cov = [0.0] * 36
            cov[0] = 0.25   # x variance
            cov[7] = 0.25   # y variance
            cov[14] = 0.0   # z variance (fixed)
            cov[21] = 0.0   # roll variance (fixed)
            cov[28] = 0.0   # pitch variance (fixed)
            cov[35] = 0.07  # yaw variance
            msg.pose.covariance = cov

        self.get_logger().info(f'Publishing initial pose to /initialpose:')
        self.get_logger().info(f'  Target position: ({pos["x"]:.2f}, {pos["y"]:.2f}, {pos["z"]:.2f})')
        self.get_logger().info(f'  Orientation: (z={ori["z"]:.4f}, w={ori["w"]:.4f})')

        # Publish the message
        self.pose_pub.publish(msg)

        return (float(pos['x']), float(pos['y']), float(pos['z']))

    def set_initial_pose(self):
        """
        Set initial pose and verify the vehicle teleported.

        Returns True if vehicle position changed to target, False otherwise.
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('SETTING INITIAL POSE')
        self.get_logger().info('=' * 60)

        # Step 1: Get current position before reset
        self.get_logger().info('Step 1: Getting current position from AWSIM...')
        pos_before = self.get_current_position(timeout=3.0)

        if pos_before:
            self.get_logger().info(f'  Current position: ({pos_before[0]:.2f}, {pos_before[1]:.2f}, {pos_before[2]:.2f})')
        else:
            self.get_logger().warn('  Could not get current position from AWSIM')
            self.get_logger().warn('  Is AWSIM running and publishing ground truth?')

        # Step 2: Publish initial pose
        self.get_logger().info('Step 2: Publishing initial pose...')
        target_pos = self.publish_pose()

        # Give AWSIM time to process and teleport
        self.get_logger().info('  Waiting for AWSIM to process...')
        time.sleep(1.0)

        # Publish again to ensure delivery
        self.publish_pose()
        time.sleep(0.5)

        # Step 3: Verify position changed
        self.get_logger().info('Step 3: Verifying position change...')

        # Wait for new position
        verification_start = time.time()
        verification_timeout = 5.0
        verified = False

        while time.time() - verification_start < verification_timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.awsim_position is not None:
                dist_to_target = self.calculate_distance(self.awsim_position, target_pos)

                if dist_to_target < 5.0:  # Within 5 meters of target
                    self.get_logger().info(f'  SUCCESS: Vehicle at ({self.awsim_position[0]:.2f}, {self.awsim_position[1]:.2f})')
                    self.get_logger().info(f'  Distance to target: {dist_to_target:.2f}m')
                    verified = True
                    break

        if not verified:
            if self.awsim_position is not None:
                dist_to_target = self.calculate_distance(self.awsim_position, target_pos)
                self.get_logger().error(f'  FAILED: Vehicle still at ({self.awsim_position[0]:.2f}, {self.awsim_position[1]:.2f})')
                self.get_logger().error(f'  Distance to target: {dist_to_target:.2f}m')
                self.get_logger().error('  Vehicle did NOT teleport!')
            else:
                self.get_logger().error('  FAILED: Could not verify position (no AWSIM data)')

            self.get_logger().info('')
            self.get_logger().info('Troubleshooting:')
            self.get_logger().info('  1. Is AWSIM running and not paused?')
            self.get_logger().info('  2. Check if /initialpose topic has subscribers:')
            self.get_logger().info('     ros2 topic info /initialpose')
            self.get_logger().info('  3. Try setting pose manually in RViz to verify AWSIM works')
            return False

        self.get_logger().info('=' * 60)
        return True

    def wait_for_stable_localization(self):
        """Wait for Autoware localization to stabilize after teleport."""
        self.get_logger().info(f'Waiting for localization to stabilize (timeout: {self.timeout}s)...')

        start_time = time.time()
        stable_count = 0
        required_stable = 30  # Need 30 consecutive odom messages
        last_progress = 0

        while time.time() - start_time < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.last_odom_time and (time.time() - self.last_odom_time) < 0.5:
                stable_count += 1
                if stable_count >= required_stable:
                    elapsed = time.time() - start_time
                    self.get_logger().info(f'Localization stable after {elapsed:.1f}s ({self.odom_count} messages)')
                    if self.current_position:
                        self.get_logger().info(f'  Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f})')
                    return True
            else:
                stable_count = 0

            # Progress indicator every 5 seconds
            elapsed = int(time.time() - start_time)
            if elapsed > last_progress and elapsed % 5 == 0:
                last_progress = elapsed
                pos_str = f'({self.current_position[0]:.1f}, {self.current_position[1]:.1f})' if self.current_position else 'unknown'
                self.get_logger().info(f'  [{elapsed}s] Odom: {self.odom_count}, stable: {stable_count}/{required_stable}, pos: {pos_str}')

        self.get_logger().warn(f'Localization did not stabilize within {self.timeout}s')
        return False


def load_pose_config(config_path):
    """Load pose from JSON config file."""
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, 'r') as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser(description='Set initial pose for Autoware/AWSIM')
    parser.add_argument('--config', type=str, help='Path to pose JSON file')
    parser.add_argument('--wait', action='store_true', help='Wait for localization to stabilize')
    parser.add_argument('--timeout', type=float, default=30.0, help='Timeout for localization wait (seconds)')
    args = parser.parse_args()

    # Determine config path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(script_dir, '..', 'configs')

    if args.config:
        config_path = args.config
    else:
        config_path = os.path.join(config_dir, 'captured_initial_pose.json')

    # Load pose
    try:
        pose_data = load_pose_config(config_path)
        print(f"Loaded initial pose from: {config_path}")
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        print()
        print("You need to capture an initial pose first:")
        print("  1. Run Autoware with RViz")
        print("  2. Run: python3 capture_initial_pose.py")
        print("  3. Use RViz 2D Pose Estimate to set the pose")
        sys.exit(1)

    # Initialize ROS
    rclpy.init()
    node = InitialPoseSetter(pose_data, wait_for_localization=args.wait, timeout=args.timeout)

    try:
        # Set initial pose and verify
        success = node.set_initial_pose()

        if not success:
            print("\nERROR: Failed to set initial pose - vehicle did not teleport")
            sys.exit(1)

        # Optionally wait for localization
        if args.wait:
            loc_success = node.wait_for_stable_localization()
            if loc_success:
                print("\nSUCCESS: Initial pose set and localization stabilized.")
            else:
                print("\nWARNING: Initial pose set but localization may not be stable.")
            sys.exit(0 if loc_success else 1)
        else:
            print("\nSUCCESS: Initial pose set. Use --wait to verify localization.")
            sys.exit(0)

    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
