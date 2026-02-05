#!/usr/bin/env python3
"""
Comprehensive system diagnostics for Autoware + AWSIM.

Monitors all critical subsystems and provides detailed status for headless operation.
Run this to understand why the vehicle isn't moving or why MRM is triggered.

Usage:
  python3 diagnose_system.py [--continuous] [--timeout 60]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from autoware_system_msgs.msg import AutowareState
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState, MrmState
from sensor_msgs.msg import PointCloud2
from autoware_internal_debug_msgs.msg import Float32Stamped
import time
import sys
import argparse
from collections import defaultdict
from datetime import datetime


class SystemDiagnostics(Node):
    def __init__(self):
        super().__init__('system_diagnostics')

        self.data = {
            'autoware_state': None,
            'autoware_state_time': None,
            'route_state': None,
            'route_state_time': None,
            'mrm_state': None,
            'mrm_state_time': None,
            'operation_mode': None,
            'operation_mode_time': None,
            'odom_count': 0,
            'odom_last_time': None,
            'odom_rate': 0.0,
            'lidar_count': 0,
            'lidar_last_time': None,
            'ndt_score': None,
            'ndt_score_time': None,
            'diagnostics': {},
            'diag_last_time': None,
        }

        self.odom_times = []
        self.start_time = time.time()

        # Best effort QoS for AWSIM topics
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Autoware state
        self.create_subscription(
            AutowareState,
            '/autoware/state',
            self.autoware_state_cb,
            10
        )

        # Route state
        self.create_subscription(
            RouteState,
            '/api/routing/state',
            self.route_state_cb,
            10
        )

        # MRM state
        self.create_subscription(
            MrmState,
            '/system/fail_safe/mrm_state',
            self.mrm_state_cb,
            10
        )

        # Operation mode
        self.create_subscription(
            OperationModeState,
            '/api/operation_mode/state',
            self.operation_mode_cb,
            10
        )

        # Localization odometry
        self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_cb,
            10
        )

        # AWSIM ground truth (best effort)
        self.create_subscription(
            Odometry,
            '/awsim/ground_truth/localization/kinematic_state',
            self.awsim_odom_cb,
            best_effort_qos
        )

        # LiDAR point cloud
        self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw',
            self.lidar_cb,
            best_effort_qos
        )

        # NDT score
        self.create_subscription(
            Float32Stamped,
            '/localization/pose_estimator/transform_probability',
            self.ndt_score_cb,
            10
        )

        # Diagnostics
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_cb,
            10
        )

    def autoware_state_cb(self, msg):
        state_names = {
            1: 'INITIALIZING',
            2: 'WAITING_FOR_ROUTE',
            3: 'PLANNING',
            4: 'WAITING_FOR_ENGAGE',
            5: 'DRIVING',
            6: 'ARRIVED_GOAL'
        }
        self.data['autoware_state'] = state_names.get(msg.state, f'UNKNOWN({msg.state})')
        self.data['autoware_state_time'] = time.time()

    def route_state_cb(self, msg):
        state_names = {
            0: 'UNKNOWN',
            1: 'UNSET',
            2: 'SET',
            3: 'ARRIVED',
            4: 'CHANGING'
        }
        self.data['route_state'] = state_names.get(msg.state, f'UNKNOWN({msg.state})')
        self.data['route_state_time'] = time.time()

    def mrm_state_cb(self, msg):
        state_names = {
            0: 'UNKNOWN',
            1: 'NORMAL',
            2: 'MRM_OPERATING',
            3: 'MRM_SUCCEEDED',
            4: 'MRM_FAILED'
        }
        behavior_names = {
            0: 'UNKNOWN',
            1: 'NONE',
            2: 'EMERGENCY_STOP',
            3: 'COMFORTABLE_STOP',
            4: 'PULL_OVER'
        }
        self.data['mrm_state'] = state_names.get(msg.state, f'UNKNOWN({msg.state})')
        self.data['mrm_behavior'] = behavior_names.get(msg.behavior, f'UNKNOWN({msg.behavior})')
        self.data['mrm_state_time'] = time.time()

    def operation_mode_cb(self, msg):
        mode_names = {
            0: 'UNKNOWN',
            1: 'STOP',
            2: 'AUTONOMOUS',
            3: 'LOCAL',
            4: 'REMOTE'
        }
        self.data['operation_mode'] = mode_names.get(msg.mode, f'UNKNOWN({msg.mode})')
        self.data['autonomous_available'] = msg.is_autonomous_mode_available
        self.data['operation_mode_time'] = time.time()

    def odom_cb(self, msg):
        now = time.time()
        self.data['odom_count'] += 1
        self.data['odom_last_time'] = now

        # Calculate rate
        self.odom_times.append(now)
        self.odom_times = [t for t in self.odom_times if now - t < 1.0]
        self.data['odom_rate'] = len(self.odom_times)

        # Store position
        self.data['position'] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        self.data['velocity'] = msg.twist.twist.linear.x

    def awsim_odom_cb(self, msg):
        self.data['awsim_position'] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        self.data['awsim_odom_time'] = time.time()

    def lidar_cb(self, msg):
        self.data['lidar_count'] += 1
        self.data['lidar_last_time'] = time.time()
        self.data['lidar_points'] = msg.width * msg.height

    def ndt_score_cb(self, msg):
        self.data['ndt_score'] = msg.data
        self.data['ndt_score_time'] = time.time()

    def diagnostics_cb(self, msg):
        self.data['diag_last_time'] = time.time()
        for status in msg.status:
            # Convert level from bytes to int if needed (ROS2 Python quirk)
            level = status.level
            if isinstance(level, bytes):
                level = int.from_bytes(level, byteorder='little')

            # Track ERROR and WARN diagnostics
            if level >= 1:  # WARN or ERROR
                level_name = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}.get(level, 'UNKNOWN')
                self.data['diagnostics'][status.name] = {
                    'level': level_name,
                    'message': status.message,
                    'time': time.time()
                }
            elif status.name in self.data['diagnostics']:
                # Clear if it's now OK
                del self.data['diagnostics'][status.name]

    def get_status_line(self, name, value, ok_condition, stale_time=None):
        """Generate a colored status line."""
        if value is None:
            return f"  {name}: NOT PUBLISHING"

        if stale_time and (time.time() - stale_time > 2.0):
            return f"  {name}: STALE ({value})"

        if ok_condition:
            return f"  {name}: OK ({value})"
        else:
            return f"  {name}: PROBLEM ({value})"

    def print_status(self):
        """Print comprehensive system status."""
        elapsed = time.time() - self.start_time
        timestamp = datetime.now().strftime('%H:%M:%S')

        print(f"\n{'='*70}")
        print(f"System Diagnostics - {timestamp} (elapsed: {elapsed:.0f}s)")
        print(f"{'='*70}")

        # Autoware State
        print("\n[AUTOWARE STATE]")
        state = self.data.get('autoware_state')
        if state:
            ok = state in ['WAITING_FOR_ROUTE', 'PLANNING', 'WAITING_FOR_ENGAGE', 'DRIVING', 'ARRIVED_GOAL']
            print(self.get_status_line('State', state, ok, self.data.get('autoware_state_time')))
        else:
            print("  State: NOT PUBLISHING - Autoware may not be running")

        # Route State
        route = self.data.get('route_state')
        if route:
            print(self.get_status_line('Route', route, route in ['SET', 'ARRIVED'], self.data.get('route_state_time')))

        # Operation Mode
        op_mode = self.data.get('operation_mode')
        if op_mode:
            auto_avail = self.data.get('autonomous_available', False)
            print(f"  Operation Mode: {op_mode} (autonomous_available: {auto_avail})")

        # MRM State
        print("\n[SAFETY SYSTEM]")
        mrm = self.data.get('mrm_state')
        mrm_behavior = self.data.get('mrm_behavior', 'UNKNOWN')
        if mrm:
            ok = mrm == 'NORMAL'
            print(self.get_status_line('MRM State', f"{mrm} / {mrm_behavior}", ok, self.data.get('mrm_state_time')))
            if mrm != 'NORMAL':
                print(f"    WARNING: MRM is active - vehicle will not move!")
        else:
            print("  MRM State: NOT PUBLISHING")

        # Localization
        print("\n[LOCALIZATION]")
        odom_rate = self.data.get('odom_rate', 0)
        odom_ok = odom_rate > 10
        if self.data.get('odom_last_time'):
            age = time.time() - self.data['odom_last_time']
            if age > 1.0:
                print(f"  Odometry: STALE (last {age:.1f}s ago)")
            else:
                print(f"  Odometry: {odom_rate} Hz ({self.data['odom_count']} total)")
        else:
            print("  Odometry: NOT PUBLISHING - Localization failed!")

        # Position
        if 'position' in self.data:
            pos = self.data['position']
            vel = self.data.get('velocity', 0)
            print(f"  Position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
            print(f"  Velocity: {vel:.2f} m/s")

        # NDT Score
        ndt = self.data.get('ndt_score')
        if ndt is not None:
            ndt_ok = ndt >= 2.3
            print(self.get_status_line('NDT Score', f"{ndt:.2f} (threshold: 2.3)", ndt_ok, self.data.get('ndt_score_time')))
        else:
            print("  NDT Score: NOT PUBLISHING")

        # Sensing
        print("\n[SENSING]")
        if self.data.get('lidar_last_time'):
            age = time.time() - self.data['lidar_last_time']
            points = self.data.get('lidar_points', 0)
            if age > 1.0:
                print(f"  LiDAR: STALE (last {age:.1f}s ago)")
            else:
                print(f"  LiDAR: OK ({points} points, {self.data['lidar_count']} frames)")
        else:
            print("  LiDAR: NOT PUBLISHING - Check AWSIM!")

        # AWSIM Ground Truth
        if self.data.get('awsim_odom_time'):
            age = time.time() - self.data['awsim_odom_time']
            if age > 1.0:
                print(f"  AWSIM Ground Truth: STALE")
            else:
                pos = self.data.get('awsim_position', (0, 0, 0))
                print(f"  AWSIM Ground Truth: OK ({pos[0]:.1f}, {pos[1]:.1f})")
        else:
            print("  AWSIM Ground Truth: NOT PUBLISHING")

        # Active Diagnostics Issues
        active_diags = {k: v for k, v in self.data['diagnostics'].items()
                        if time.time() - v['time'] < 5.0}
        if active_diags:
            print("\n[ACTIVE DIAGNOSTIC ISSUES]")
            for name, info in sorted(active_diags.items())[:10]:  # Show top 10
                print(f"  [{info['level']}] {name}")
                if info['message']:
                    print(f"         {info['message'][:60]}")

        # Recommendations
        print("\n[RECOMMENDATIONS]")
        issues = []

        if not self.data.get('autoware_state'):
            issues.append("Start Autoware: ./Run_Autoware.sh")

        if not self.data.get('lidar_last_time') or (time.time() - self.data.get('lidar_last_time', 0) > 2):
            issues.append("Check AWSIM is running and publishing sensor data")

        if odom_rate < 10 and self.data.get('autoware_state'):
            issues.append("Localization failing - try setting initial pose: python3 set_initial_pose.py --wait")

        if ndt is not None and ndt < 2.3:
            issues.append("NDT score low - initial pose may be wrong, or map doesn't match environment")

        if mrm and mrm != 'NORMAL':
            issues.append("MRM active - fix localization/planning issues before engaging")

        if not self.data.get('autonomous_available', True) and op_mode:
            issues.append("Autonomous mode unavailable - check diagnostic issues above")

        if issues:
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("  System appears healthy. Ready to run experiments.")

        print(f"\n{'='*70}")


def main():
    parser = argparse.ArgumentParser(description='System diagnostics for Autoware + AWSIM')
    parser.add_argument('--continuous', '-c', action='store_true', help='Continuous monitoring mode')
    parser.add_argument('--interval', type=float, default=3.0, help='Update interval in seconds')
    parser.add_argument('--timeout', type=float, default=0, help='Exit after N seconds (0=forever)')
    args = parser.parse_args()

    rclpy.init()
    node = SystemDiagnostics()

    print("Starting system diagnostics...")
    print("Collecting data from topics...")

    start_time = time.time()
    last_print = 0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            elapsed = time.time() - start_time

            # Print status at intervals
            if elapsed - last_print >= args.interval:
                node.print_status()
                last_print = elapsed

                if not args.continuous:
                    break

            # Timeout check
            if args.timeout > 0 and elapsed >= args.timeout:
                print(f"\nTimeout reached ({args.timeout}s)")
                break

    except KeyboardInterrupt:
        print("\nDiagnostics stopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
