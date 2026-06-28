#!/usr/bin/env python3
"""
Live route feasibility tester.

For each goal in captured_goals.json:
  1. Set the route via Autoware API
  2. Wait for route to be accepted
  3. Check if behavior planner generates a valid (non-zero velocity) path
  4. Clear the route
  5. Report result

AWSIM + Autoware must be running. Run as:
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Kalpit/autoware/install/setup.bash
  python3 test_route_feasibility.py [--goals goal_001,goal_007,...]
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose
from autoware_planning_msgs.msg import Path
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ClearRoute
from autoware_adapi_v1_msgs.msg import RouteState

GOALS_FILE = os.path.join(os.path.dirname(__file__),
                          'experiments/configs/captured_goals.json')
RESET_POSE_FILE = os.path.join(os.path.dirname(__file__),
                               'experiments/configs/reset_pose.json')

# Thresholds for "feasible" path
MIN_PATH_POINTS = 15          # behavior path must have at least this many points
MIN_MEAN_VELOCITY = 0.1       # m/s — at least some points must have positive velocity
CHECK_WINDOW = 8.0            # seconds to wait and sample path after route set
ROUTE_SET_TIMEOUT = 15.0      # seconds to wait for route to be accepted


def load_goals(names=None):
    with open(GOALS_FILE) as f:
        data = json.load(f)
    goals = data['goals']
    if names:
        goals = [g for g in goals if g['id'] in names]
    return goals


class FeasibilityChecker(Node):
    def __init__(self):
        super().__init__('feasibility_checker')

        # QoS for route state (transient local so we get the current state)
        tl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        volatile_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._route_state = None
        self._path_samples = []
        self._path_sub = None

        self._route_state_sub = self.create_subscription(
            RouteState, '/api/routing/state',
            self._on_route_state, tl_qos)

        self._set_route_cli = self.create_client(
            SetRoutePoints, '/api/routing/set_route_points')
        self._clear_route_cli = self.create_client(
            ClearRoute, '/api/routing/clear_route')

    def _on_route_state(self, msg):
        self._route_state = msg.state

    def _on_path(self, msg):
        self._path_samples.append(msg)

    def wait_spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def clear_route(self):
        if not self._clear_route_cli.wait_for_service(timeout_sec=5.0):
            return False
        req = ClearRoute.Request()
        fut = self._clear_route_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        self.wait_spin(1.0)
        return True

    def set_route(self, goal):
        if not self._set_route_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('set_route_points service not available')
            return False

        req = SetRoutePoints.Request()
        req.header.frame_id = goal['goal'].get('frame_id', 'map')
        req.header.stamp = self.get_clock().now().to_msg()
        req.option.allow_goal_modification = False

        goal_pose = Pose()
        p = goal['goal']['position']
        o = goal['goal']['orientation']
        goal_pose.position.x = p['x']
        goal_pose.position.y = p['y']
        goal_pose.position.z = p['z']
        goal_pose.orientation.x = o.get('x', 0.0)
        goal_pose.orientation.y = o.get('y', 0.0)
        goal_pose.orientation.z = o['z']
        goal_pose.orientation.w = o['w']
        req.goal = goal_pose

        fut = self._set_route_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.done():
            return False
        resp = fut.result()
        return resp.status.success

    def wait_for_route_set(self, timeout=ROUTE_SET_TIMEOUT):
        # RouteState: UNSET=0, SET=1, ARRIVED=2, CHANGING=3
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._route_state == RouteState.SET:
                return True
        return False

    def sample_path(self, duration=CHECK_WINDOW):
        """Subscribe to behavior path and sample for `duration` seconds."""
        volatile_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._path_samples = []
        self._path_sub = self.create_subscription(
            Path,
            '/planning/scenario_planning/lane_driving/behavior_planning/path',
            self._on_path, volatile_qos)
        self.wait_spin(duration)
        self.destroy_subscription(self._path_sub)
        self._path_sub = None
        return self._path_samples

    def evaluate_samples(self, samples):
        """Return (n_points_max, mean_lead_velocity, verdict)."""
        if not samples:
            return 0, 0.0, 'NO_PATH'
        best_n = 0
        best_vel = 0.0
        for msg in samples:
            pts = msg.points
            n = len(pts)
            if n > best_n:
                best_n = n
            # mean velocity of first 10 points
            window = min(10, n)
            if window > 0:
                mean_v = sum(abs(pt.longitudinal_velocity_mps) for pt in pts[:window]) / window
                if mean_v > best_vel:
                    best_vel = mean_v
        if best_n < MIN_PATH_POINTS:
            verdict = 'STOP_PATH'
        elif best_vel < MIN_MEAN_VELOCITY:
            verdict = 'ZERO_VEL'
        else:
            verdict = 'OK'
        return best_n, best_vel, verdict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goals', default='', help='Comma-separated goal IDs to test (default: all)')
    args = parser.parse_args()

    names = [g.strip() for g in args.goals.split(',') if g.strip()] or None
    goals = load_goals(names)

    rclpy.init()
    node = FeasibilityChecker()

    print(f'\nRoute Feasibility Test — {len(goals)} goals')
    print('=' * 72)
    print(f'{"Goal":<12} {"Dist":>6}  {"RouteSet":>9}  {"PathPts":>7}  {"LeadVel":>8}  Result')
    print('-' * 72)

    results = {}

    for g in goals:
        gid = g['id']
        dist = g.get('estimated_distance', '?')

        # Clear any existing route
        node.clear_route()
        node.wait_spin(1.0)

        # Set route
        ok = node.set_route(g)
        if not ok:
            print(f'{gid:<12} {str(dist):>6}  {"SVC_FAIL":>9}  {"—":>7}  {"—":>8}  SKIP')
            results[gid] = 'SVC_FAIL'
            continue

        # Wait for route to be accepted
        route_set = node.wait_for_route_set(ROUTE_SET_TIMEOUT)
        if not route_set:
            print(f'{gid:<12} {str(dist):>6}  {"TIMEOUT":>9}  {"—":>7}  {"—":>8}  REJECT')
            results[gid] = 'ROUTE_REJECT'
            node.clear_route()
            node.wait_spin(1.0)
            continue

        # Sample behavior path
        samples = node.sample_path(CHECK_WINDOW)
        n_pts, lead_vel, verdict = node.evaluate_samples(samples)

        symbol = '✓' if verdict == 'OK' else '✗'
        print(f'{gid:<12} {str(dist):>6}  {"SET":>9}  {n_pts:>7}  {lead_vel:>8.2f}  {symbol} {verdict}')
        results[gid] = verdict

        node.clear_route()
        node.wait_spin(2.0)

    print('=' * 72)
    ok_goals = [g for g, v in results.items() if v == 'OK']
    bad_goals = [g for g, v in results.items() if v != 'OK']
    print(f'\nFEASIBLE ({len(ok_goals)}): {", ".join(ok_goals)}')
    print(f'FAILED   ({len(bad_goals)}): {", ".join(bad_goals)}')

    # Save results
    out = os.path.join(os.path.dirname(__file__), 'route_feasibility.json')
    with open(out, 'w') as f:
        json.dump({'results': results, 'feasible': ok_goals, 'failed': bad_goals}, f, indent=2)
    print(f'\nResults saved to: {out}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
