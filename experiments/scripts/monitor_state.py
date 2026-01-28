#!/usr/bin/env python3
"""
Monitor vehicle state during experiments.

Shows:
- Current position and velocity
- Distance to goal
- Autoware state
- Basic collision proximity warning

Usage: python3 monitor_state.py [goal_x] [goal_y]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_perception_msgs.msg import PredictedObjects
import math
import sys
import json
import os


class StateMonitor(Node):
    def __init__(self, goal_x=None, goal_y=None):
        super().__init__('state_monitor')

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_vel = 0.0
        self.min_obstacle_dist = float('inf')
        self.autoware_state = 'UNKNOWN'

        # QoS for AWSIM ground truth (publishes BEST_EFFORT)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to ground truth pose
        self.pose_sub = self.create_subscription(
            Odometry,
            '/awsim/ground_truth/localization/kinematic_state',
            self.pose_callback,
            best_effort_qos
        )

        # Subscribe to velocity
        self.vel_sub = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.velocity_callback,
            10
        )

        # Subscribe to objects for collision proximity
        self.obj_sub = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self.objects_callback,
            10
        )

        # Timer for display update
        self.timer = self.create_timer(1.0, self.display_status)

        self.get_logger().info('State Monitor started')
        if goal_x and goal_y:
            self.get_logger().info(f'Goal: ({goal_x:.2f}, {goal_y:.2f})')

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def velocity_callback(self, msg):
        self.current_vel = msg.longitudinal_velocity

    def objects_callback(self, msg):
        if not msg.objects:
            self.min_obstacle_dist = float('inf')
            return

        min_dist = float('inf')
        for obj in msg.objects:
            obj_x = obj.kinematics.initial_pose_with_covariance.pose.position.x
            obj_y = obj.kinematics.initial_pose_with_covariance.pose.position.y
            dist = math.sqrt((obj_x - self.current_x)**2 + (obj_y - self.current_y)**2)
            min_dist = min(min_dist, dist)

        self.min_obstacle_dist = min_dist

    def display_status(self):
        # Calculate distance to goal
        if self.goal_x and self.goal_y:
            dist_to_goal = math.sqrt(
                (self.goal_x - self.current_x)**2 +
                (self.goal_y - self.current_y)**2
            )
            goal_str = f'Goal dist: {dist_to_goal:.1f}m'
        else:
            goal_str = 'Goal: not set'

        # Collision warning
        if self.min_obstacle_dist < 3.0:
            warning = ' ⚠️  CLOSE!'
        elif self.min_obstacle_dist < 5.0:
            warning = ' ⚡'
        else:
            warning = ''

        print(f'\r[Pos: ({self.current_x:.1f}, {self.current_y:.1f}) | '
              f'Vel: {self.current_vel:.1f} m/s | '
              f'{goal_str} | '
              f'Obstacle: {self.min_obstacle_dist:.1f}m{warning}]     ',
              end='', flush=True)

        # Check if goal reached
        if self.goal_x and self.goal_y:
            dist_to_goal = math.sqrt(
                (self.goal_x - self.current_x)**2 +
                (self.goal_y - self.current_y)**2
            )
            if dist_to_goal < 5.0 and self.current_vel < 0.5:
                print('\n\n🎯 GOAL REACHED!')


def main():
    rclpy.init()

    # Try to load goal from config
    goal_x, goal_y = None, None

    if len(sys.argv) >= 3:
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
    else:
        # Try to load from captured_route.json
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_file = os.path.join(script_dir, '..', 'configs', 'captured_route.json')
        if os.path.exists(config_file):
            with open(config_file) as f:
                data = json.load(f)
                if data.get('goal_pose'):
                    goal_x = data['goal_pose']['position']['x']
                    goal_y = data['goal_pose']['position']['y']

    node = StateMonitor(goal_x, goal_y)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print('\n\nMonitor stopped')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
