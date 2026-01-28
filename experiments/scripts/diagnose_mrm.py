#!/usr/bin/env python3
"""
MRM Diagnostic Investigation Script

Monitors all topics related to MRM (Minimum Risk Maneuver) to understand:
- What diagnostic conditions trigger MRM
- When autonomous mode becomes unavailable
- What MRM behavior is selected and why
- Timeline of events leading to emergency stop

Usage:
  1. Launch AWSIM and Autoware
  2. Source ROS2 + Autoware:
       source /opt/ros/humble/setup.bash
       source /path/to/autoware/install/setup.bash
  3. Run: python3 diagnose_mrm.py
  4. Set goal in RViz and engage autonomous mode
  5. Watch output for diagnostic failures and MRM triggers
  6. Ctrl+C to stop and save report

Output: Saves timestamped report to experiments/data/mrm_diagnosis_<timestamp>.txt
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import sys
import os
import json
from datetime import datetime
from collections import defaultdict

# Try importing Autoware-specific messages
try:
    from tier4_system_msgs.msg import MrmState, OperationModeAvailability
    from autoware_adapi_v1_msgs.msg import OperationModeState
    HAS_AUTOWARE_MSGS = True
except ImportError:
    HAS_AUTOWARE_MSGS = False
    print("WARNING: Autoware messages not found.")
    print("Source autoware workspace first:")
    print("  source /opt/ros/humble/setup.bash")
    print("  source <autoware>/install/setup.bash")
    print("")
    print("Falling back to generic subscribers (limited info).\n")


class MrmDiagnostics(Node):
    def __init__(self):
        super().__init__('mrm_diagnostics')

        self.log_lines = []
        self.start_time = self.get_clock().now()
        self.diagnostic_status = {}          # name -> (level, message)
        self.failing_diagnostics = {}        # name -> (level, message, first_seen, last_seen)
        self.mrm_events = []                 # timeline of MRM state changes
        self.mode_availability = {}          # field -> bool
        self.current_vel = 0.0
        self.current_pos = (0.0, 0.0)
        self.mrm_state_str = "UNKNOWN"
        self.mrm_behavior_str = "NONE"
        self.operation_mode_str = "UNKNOWN"
        self.diag_error_count = 0
        self.diag_warn_count = 0
        self.diag_stale_count = 0
        self.last_mrm_state = None
        self.last_mode_avail = None

        # QoS profiles
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Core diagnostic topic ---
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )

        # Also try /diagnostics_agg (aggregated)
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics_agg',
            self.diagnostics_callback,
            10
        )

        # --- Ground truth velocity/position ---
        self.create_subscription(
            Odometry,
            '/awsim/ground_truth/localization/kinematic_state',
            self.odom_callback,
            best_effort_qos
        )

        # --- Autoware-specific topics ---
        if HAS_AUTOWARE_MSGS:
            # MRM state
            self.create_subscription(
                MrmState,
                '/system/fail_safe/mrm_state',
                self.mrm_state_callback,
                10
            )

            # Operation mode availability (what MRM handler reads)
            self.create_subscription(
                OperationModeAvailability,
                '/system/operation_mode/availability',
                self.mode_availability_callback,
                10
            )

            # Current operation mode
            self.create_subscription(
                OperationModeState,
                '/api/operation_mode/state',
                self.operation_mode_callback,
                transient_local_qos
            )

        # Display timer (1 Hz)
        self.create_timer(1.0, self.display_status)

        # Detailed log timer (0.5 Hz - logs failing diagnostics periodically)
        self.create_timer(2.0, self.log_failing_diagnostics)

        self._log("=" * 60)
        self._log("MRM DIAGNOSTIC INVESTIGATION")
        self._log(f"Started: {datetime.now().isoformat()}")
        self._log(f"Autoware msgs available: {HAS_AUTOWARE_MSGS}")
        self._log("=" * 60)
        self._log("")
        self._log("Waiting for data... Set goal in RViz and engage autonomous.")
        self._log("")

        self.get_logger().info("MRM Diagnostics started. Monitoring all diagnostic topics.")
        if not HAS_AUTOWARE_MSGS:
            self.get_logger().warn("Running without Autoware messages - limited MRM info.")

    def _log(self, msg):
        """Log to both console and internal buffer."""
        timestamp = self._elapsed()
        line = f"[{timestamp}] {msg}"
        print(line)
        self.log_lines.append(line)

    def _elapsed(self):
        """Seconds since start."""
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        return f"{elapsed:8.1f}s"

    def _parse_level(self, level):
        """Convert diagnostic level to int (handles bytes from ROS2 Python)."""
        if isinstance(level, bytes):
            return int.from_bytes(level, byteorder='little')
        return int(level)

    def diagnostics_callback(self, msg):
        """Process /diagnostics messages. Track all statuses, flag errors."""
        for status in msg.status:
            name = status.name
            level = self._parse_level(status.level)
            message = status.message

            prev = self.diagnostic_status.get(name)
            self.diagnostic_status[name] = (level, message)

            # Detect transitions to error/warn/stale
            if level != 0:  # Not OK
                level_str = {1: "WARN", 2: "ERROR", 3: "STALE"}.get(level, f"LEVEL_{level}")
                now_str = datetime.now().isoformat()

                if name not in self.failing_diagnostics:
                    # New failure
                    self.failing_diagnostics[name] = {
                        'level': level,
                        'level_str': level_str,
                        'message': message,
                        'first_seen': now_str,
                        'last_seen': now_str,
                        'count': 1
                    }
                    self._log(f"DIAG {level_str}: {name}")
                    self._log(f"  Message: {message}")

                    # Log key-value pairs from diagnostic
                    for kv in status.values:
                        if kv.key and kv.value:
                            self._log(f"  {kv.key}: {kv.value}")
                else:
                    entry = self.failing_diagnostics[name]
                    entry['last_seen'] = now_str
                    entry['count'] += 1
                    # Log if level escalated
                    if level > entry['level']:
                        entry['level'] = level
                        entry['level_str'] = level_str
                        entry['message'] = message
                        self._log(f"DIAG ESCALATED to {level_str}: {name}")
                        self._log(f"  Message: {message}")

            elif name in self.failing_diagnostics:
                # Recovered
                entry = self.failing_diagnostics[name]
                self._log(f"DIAG RECOVERED: {name} (was {entry['level_str']} for {entry['count']} msgs)")
                del self.failing_diagnostics[name]

        # Update counts
        levels = [v['level'] for v in self.failing_diagnostics.values()]
        self.diag_error_count = levels.count(2)
        self.diag_warn_count = levels.count(1)
        self.diag_stale_count = levels.count(3)

    def mrm_state_callback(self, msg):
        """Track MRM state changes."""
        state_map = {0: "UNKNOWN", 1: "NORMAL", 2: "MRM_OPERATING", 3: "MRM_SUCCEEDED", 4: "MRM_FAILED"}
        behavior_map = {0: "NONE", 1: "PULL_OVER", 2: "COMFORTABLE_STOP", 3: "EMERGENCY_STOP"}

        state_str = state_map.get(msg.state, f"STATE_{msg.state}")
        behavior_str = behavior_map.get(msg.behavior, f"BEHAVIOR_{msg.behavior}")

        if state_str != self.mrm_state_str or behavior_str != self.mrm_behavior_str:
            self._log("")
            self._log(f">>> MRM STATE CHANGE: {self.mrm_state_str} -> {state_str}")
            self._log(f">>> MRM BEHAVIOR: {self.mrm_behavior_str} -> {behavior_str}")
            self._log(f">>> Vehicle: pos=({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f}), vel={self.current_vel:.2f} m/s")

            # Log current failing diagnostics at time of MRM change
            if self.failing_diagnostics:
                self._log(f">>> Failing diagnostics at this moment ({len(self.failing_diagnostics)}):")
                for name, entry in sorted(self.failing_diagnostics.items()):
                    self._log(f">>>   [{entry['level_str']}] {name}: {entry['message']}")

            # Log mode availability at time of MRM change
            if self.mode_availability:
                self._log(f">>> Mode availability: {self.mode_availability}")

            self._log("")

            self.mrm_events.append({
                'time': datetime.now().isoformat(),
                'elapsed': self._elapsed(),
                'prev_state': self.mrm_state_str,
                'new_state': state_str,
                'prev_behavior': self.mrm_behavior_str,
                'new_behavior': behavior_str,
                'velocity': self.current_vel,
                'position': self.current_pos,
                'failing_diagnostics': {k: v['message'] for k, v in self.failing_diagnostics.items()},
                'mode_availability': dict(self.mode_availability)
            })

            self.mrm_state_str = state_str
            self.mrm_behavior_str = behavior_str

    def mode_availability_callback(self, msg):
        """Track operation mode availability changes."""
        avail = {
            'autonomous': msg.autonomous,
            'stop': msg.stop,
            'local': msg.local,
            'remote': msg.remote,
            'emergency_stop': msg.emergency_stop,
            'comfortable_stop': msg.comfortable_stop,
            'pull_over': msg.pull_over,
        }

        if avail != self.mode_availability:
            changes = []
            for key, val in avail.items():
                old = self.mode_availability.get(key)
                if old is not None and old != val:
                    changes.append(f"{key}: {old} -> {val}")

            if changes and self.mode_availability:
                self._log("")
                self._log(f"MODE AVAILABILITY CHANGED:")
                for c in changes:
                    self._log(f"  {c}")
                self._log(f"  Full: {avail}")
                self._log("")

            self.mode_availability = avail

    def operation_mode_callback(self, msg):
        """Track current operation mode."""
        mode_map = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}
        mode_str = mode_map.get(msg.mode, f"MODE_{msg.mode}")

        if mode_str != self.operation_mode_str:
            self._log(f"OPERATION MODE: {self.operation_mode_str} -> {mode_str}")
            self.operation_mode_str = mode_str

    def odom_callback(self, msg):
        """Track vehicle state."""
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_vel = (vx**2 + vy**2)**0.5

    def log_failing_diagnostics(self):
        """Periodically log persistent failures."""
        if not self.failing_diagnostics:
            return

        # Only log if we have persistent failures (seen > 5 times)
        persistent = {k: v for k, v in self.failing_diagnostics.items() if v['count'] > 5}
        if persistent and (len(persistent) != getattr(self, '_last_persistent_count', 0)):
            self._last_persistent_count = len(persistent)
            self._log(f"Persistent failing diagnostics ({len(persistent)}):")
            for name, entry in sorted(persistent.items()):
                self._log(f"  [{entry['level_str']}] {name} (x{entry['count']})")

    def display_status(self):
        """Terminal status line."""
        total_diag = len(self.diagnostic_status)
        failing = len(self.failing_diagnostics)

        status = (
            f"\r[MRM: {self.mrm_state_str}/{self.mrm_behavior_str} | "
            f"Mode: {self.operation_mode_str} | "
            f"Diag: {total_diag} total, "
            f"{self.diag_error_count}E/{self.diag_warn_count}W/{self.diag_stale_count}S | "
            f"Vel: {self.current_vel:.1f} m/s | "
            f"Pos: ({self.current_pos[0]:.0f}, {self.current_pos[1]:.0f})]     "
        )
        print(status, end='', flush=True)

    def save_report(self):
        """Save full diagnostic report on exit."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(script_dir, '..', 'data')
        os.makedirs(data_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_file = os.path.join(data_dir, f'mrm_diagnosis_{timestamp}.txt')
        json_file = os.path.join(data_dir, f'mrm_diagnosis_{timestamp}.json')

        # --- Text report ---
        with open(report_file, 'w') as f:
            f.write("=" * 60 + "\n")
            f.write("MRM DIAGNOSTIC REPORT\n")
            f.write(f"Generated: {datetime.now().isoformat()}\n")
            f.write("=" * 60 + "\n\n")

            # Timeline
            f.write("--- EVENT LOG ---\n\n")
            for line in self.log_lines:
                f.write(line + "\n")

            # MRM events summary
            f.write("\n\n--- MRM EVENTS SUMMARY ---\n\n")
            if self.mrm_events:
                for i, event in enumerate(self.mrm_events):
                    f.write(f"Event {i+1}: {event['prev_state']} -> {event['new_state']}\n")
                    f.write(f"  Behavior: {event['prev_behavior']} -> {event['new_behavior']}\n")
                    f.write(f"  Time: {event['elapsed']}\n")
                    f.write(f"  Velocity: {event['velocity']:.2f} m/s\n")
                    f.write(f"  Position: ({event['position'][0]:.1f}, {event['position'][1]:.1f})\n")
                    if event['failing_diagnostics']:
                        f.write(f"  Failing diagnostics ({len(event['failing_diagnostics'])}):\n")
                        for name, msg in event['failing_diagnostics'].items():
                            f.write(f"    - {name}: {msg}\n")
                    if event['mode_availability']:
                        f.write(f"  Mode availability: {event['mode_availability']}\n")
                    f.write("\n")
            else:
                f.write("No MRM events recorded.\n")

            # Diagnostic summary
            f.write("\n--- DIAGNOSTIC SUMMARY ---\n\n")
            f.write(f"Total unique diagnostics seen: {len(self.diagnostic_status)}\n")
            f.write(f"Currently failing: {len(self.failing_diagnostics)}\n\n")

            if self.failing_diagnostics:
                f.write("Failing at shutdown:\n")
                for name, entry in sorted(self.failing_diagnostics.items()):
                    f.write(f"  [{entry['level_str']}] {name}\n")
                    f.write(f"    Message: {entry['message']}\n")
                    f.write(f"    First seen: {entry['first_seen']}\n")
                    f.write(f"    Occurrences: {entry['count']}\n\n")

            # All diagnostics seen
            f.write("\n--- ALL DIAGNOSTICS SEEN ---\n\n")
            for name, (level, message) in sorted(self.diagnostic_status.items()):
                lvl = self._parse_level(level) if isinstance(level, bytes) else level
                level_str = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}.get(lvl, f"L{lvl}")
                f.write(f"  [{level_str}] {name}: {message}\n")

        # --- JSON data ---
        json_data = {
            'timestamp': datetime.now().isoformat(),
            'mrm_events': self.mrm_events,
            'final_state': {
                'mrm_state': self.mrm_state_str,
                'mrm_behavior': self.mrm_behavior_str,
                'operation_mode': self.operation_mode_str,
                'mode_availability': self.mode_availability,
                'velocity': self.current_vel,
                'position': self.current_pos,
            },
            'failing_diagnostics': {
                k: {
                    'level_str': v['level_str'],
                    'message': v['message'],
                    'first_seen': v['first_seen'],
                    'count': v['count']
                } for k, v in self.failing_diagnostics.items()
            },
            'all_diagnostics': {
                k: {'level': v[0], 'message': v[1]}
                for k, v in self.diagnostic_status.items()
            }
        }

        with open(json_file, 'w') as f:
            json.dump(json_data, f, indent=2)

        print(f"\n\nReport saved to: {report_file}")
        print(f"JSON data saved to: {json_file}")


def main():
    rclpy.init()
    node = MrmDiagnostics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n\nSaving report...")
        node.save_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
