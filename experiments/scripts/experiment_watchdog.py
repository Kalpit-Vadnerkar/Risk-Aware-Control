#!/usr/bin/env python3
"""
Experiment Watchdog - monitors a running experiment and detects stuck/completion.

Replaces passive `sleep $DURATION` in run_experiment.sh with active monitoring.

Monitors:
  - Time elapsed (hard timeout)
  - Vehicle velocity (stuck detection)
  - Distance to goal (completion detection)

Exit codes:
  0 = Goal reached or duration completed normally
  1 = Vehicle stuck (no movement for stuck_timeout seconds)
  2 = Error

Usage:
  python3 experiment_watchdog.py <max_duration> [goal_x] [goal_y] [stuck_timeout]

  max_duration:  Maximum experiment time in seconds
  goal_x/y:     Goal coordinates (optional, enables completion detection)
  stuck_timeout: Seconds without movement before declaring stuck (default: 30)

Output: Writes status to stdout and a JSON result file.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import sys
import os
import json
import math
import time
from datetime import datetime


class ExperimentWatchdog(Node):
    def __init__(self, max_duration, goal_x=None, goal_y=None, stuck_timeout=30.0):
        super().__init__('experiment_watchdog')

        self.max_duration = max_duration
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.stuck_timeout = stuck_timeout
        self.goal_threshold = 5.0  # meters from goal to consider "reached"
        self.movement_threshold = 0.3  # m/s below which we consider "not moving"

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_vel = 0.0
        self.last_moving_time = time.time()
        self.start_time = time.time()
        self.has_received_odom = False
        self.result = None  # Will be set when experiment ends

        # Track position history for stuck detection (more robust than velocity alone)
        self.last_position = None
        self.last_position_change_time = time.time()
        self.position_change_threshold = 1.0  # meters

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            Odometry,
            '/awsim/ground_truth/localization/kinematic_state',
            self.odom_callback,
            best_effort_qos
        )

        # Check at 2 Hz
        self.timer = self.create_timer(0.5, self.check_status)

        # Status display at 1 Hz
        self.display_timer = self.create_timer(1.0, self.display)

        self.get_logger().info(f'Watchdog started: max_duration={max_duration}s, stuck_timeout={stuck_timeout}s')
        if goal_x is not None:
            self.get_logger().info(f'Goal: ({goal_x:.1f}, {goal_y:.1f})')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_vel = math.sqrt(vx**2 + vy**2)
        self.has_received_odom = True

        # Track movement via position change
        now = time.time()
        if self.last_position is not None:
            dx = self.current_x - self.last_position[0]
            dy = self.current_y - self.last_position[1]
            dist_moved = math.sqrt(dx**2 + dy**2)
            if dist_moved > self.position_change_threshold:
                self.last_position = (self.current_x, self.current_y)
                self.last_position_change_time = now
        else:
            self.last_position = (self.current_x, self.current_y)

        # Update last moving time based on velocity
        if self.current_vel > self.movement_threshold:
            self.last_moving_time = now

    def check_status(self):
        now = time.time()
        elapsed = now - self.start_time

        # Check hard timeout
        if elapsed >= self.max_duration:
            self.finish('timeout', 'Maximum duration reached')
            return

        # Don't check stuck/goal until we've received odometry data
        if not self.has_received_odom:
            return

        # Check goal reached
        if self.goal_x is not None and self.goal_y is not None:
            dist_to_goal = math.sqrt(
                (self.goal_x - self.current_x)**2 +
                (self.goal_y - self.current_y)**2
            )
            if dist_to_goal < self.goal_threshold and self.current_vel < 0.5:
                self.finish('goal_reached', f'Goal reached (dist={dist_to_goal:.1f}m)')
                return

        # Check stuck - use both velocity and position-based detection
        # Only check after initial 15s (allow time for engagement and planning)
        if elapsed > 15.0:
            time_since_movement = now - self.last_moving_time
            time_since_position_change = now - self.last_position_change_time

            # Stuck if BOTH velocity has been low AND position hasn't changed
            if time_since_movement > self.stuck_timeout and time_since_position_change > self.stuck_timeout:
                self.finish('stuck', f'No movement for {self.stuck_timeout:.0f}s')
                return

    def display(self):
        if not self.has_received_odom:
            print('\r[Watchdog] Waiting for odometry...', end='', flush=True)
            return

        elapsed = time.time() - self.start_time
        remaining = self.max_duration - elapsed
        time_since_move = time.time() - self.last_moving_time

        goal_str = ''
        if self.goal_x is not None:
            dist = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
            goal_str = f' | Goal: {dist:.0f}m'

        stuck_warning = ''
        if time_since_move > 10:
            stuck_warning = f' | IDLE {time_since_move:.0f}s'

        print(f'\r[Watchdog] {elapsed:.0f}s/{self.max_duration}s | '
              f'Vel: {self.current_vel:.1f} m/s | '
              f'Pos: ({self.current_x:.0f}, {self.current_y:.0f})'
              f'{goal_str}{stuck_warning}     ',
              end='', flush=True)

    def finish(self, status, message):
        elapsed = time.time() - self.start_time
        self.result = {
            'status': status,
            'message': message,
            'elapsed_seconds': round(elapsed, 1),
            'final_position': {'x': self.current_x, 'y': self.current_y},
            'final_velocity': round(self.current_vel, 3),
            'timestamp': datetime.now().isoformat()
        }

        if self.goal_x is not None:
            dist = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
            self.result['distance_to_goal'] = round(dist, 1)

        print(f'\n\n[Watchdog] FINISHED: {status} - {message}')
        print(f'  Elapsed: {elapsed:.1f}s')
        print(f'  Position: ({self.current_x:.1f}, {self.current_y:.1f})')
        print(f'  Velocity: {self.current_vel:.2f} m/s')

        # Write result to file for run_experiment.sh to read
        script_dir = os.path.dirname(os.path.abspath(__file__))
        result_file = os.path.join(script_dir, '..', 'data', 'watchdog_result.json')
        os.makedirs(os.path.dirname(result_file), exist_ok=True)
        with open(result_file, 'w') as f:
            json.dump(self.result, f, indent=2)

        # Shutdown ROS
        raise SystemExit(0 if status in ('goal_reached', 'timeout') else 1)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 experiment_watchdog.py <max_duration> [goal_x] [goal_y] [stuck_timeout]")
        sys.exit(2)

    max_duration = float(sys.argv[1])
    goal_x = float(sys.argv[2]) if len(sys.argv) > 3 else None
    goal_y = float(sys.argv[3]) if len(sys.argv) > 3 else None
    stuck_timeout = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0

    rclpy.init()
    node = ExperimentWatchdog(max_duration, goal_x, goal_y, stuck_timeout)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        exit_code = 0
        if node.result:
            exit_code = 0 if node.result['status'] in ('goal_reached', 'timeout') else 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
