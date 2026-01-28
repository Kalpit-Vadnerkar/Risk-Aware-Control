#!/usr/bin/env python3
"""
Wait for Autoware to be fully initialized and ready for route setting.

Monitors /autoware/state and exits when state reaches WAITING_FOR_ROUTE.

Exit codes:
  0 = Autoware ready (WAITING_FOR_ROUTE)
  1 = Timeout (Autoware did not become ready)
  2 = Error

Usage:
  python3 wait_for_autoware.py [timeout_seconds]
  Default timeout: 120 seconds
"""

import rclpy
from rclpy.node import Node
import sys
import time

try:
    from autoware_system_msgs.msg import AutowareState
    HAS_MSG = True
except ImportError:
    HAS_MSG = False


# State constants (from AutowareState.msg)
STATE_NAMES = {
    1: "INITIALIZING",
    2: "WAITING_FOR_ROUTE",
    3: "PLANNING",
    4: "WAITING_FOR_ENGAGE",
    5: "DRIVING",
    6: "ARRIVED_GOAL",
    7: "FINALIZING",
}


class AutowareReadinessWaiter(Node):
    def __init__(self, timeout):
        super().__init__('autoware_readiness_waiter')
        self.timeout = timeout
        self.start_time = time.time()
        self.current_state = 0
        self.ready = False

        if HAS_MSG:
            self.create_subscription(
                AutowareState,
                '/autoware/state',
                self.state_callback,
                10
            )
        else:
            # Fallback: use generic subscriber
            from std_msgs.msg import String
            self.get_logger().warn(
                "AutowareState msg not found. "
                "Make sure to source autoware/install/setup.bash"
            )
            raise SystemExit(2)

        self.timer = self.create_timer(1.0, self.check_timeout)
        self.get_logger().info(f'Waiting for Autoware (timeout: {timeout}s)...')

    def state_callback(self, msg):
        new_state = msg.state
        if new_state != self.current_state:
            old_name = STATE_NAMES.get(self.current_state, "UNKNOWN")
            new_name = STATE_NAMES.get(new_state, f"STATE_{new_state}")
            elapsed = time.time() - self.start_time
            print(f'  [{elapsed:.0f}s] Autoware: {old_name} -> {new_name}')
            self.current_state = new_state

        # Ready when WAITING_FOR_ROUTE or later operational states
        if new_state >= 2:  # WAITING_FOR_ROUTE or beyond
            if not self.ready:
                self.ready = True
                elapsed = time.time() - self.start_time
                state_name = STATE_NAMES.get(new_state, f"STATE_{new_state}")
                print(f'\nAutoware READY ({state_name}) after {elapsed:.0f}s')
                raise SystemExit(0)

    def check_timeout(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.timeout:
            state_name = STATE_NAMES.get(self.current_state, "UNKNOWN")
            print(f'\nTimeout after {self.timeout}s. Last state: {state_name}')
            raise SystemExit(1)

        # Progress indicator
        if self.current_state == 0:
            print(f'\r  [{elapsed:.0f}s] Waiting for /autoware/state topic...', end='', flush=True)


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0

    rclpy.init()
    node = AutowareReadinessWaiter(timeout)

    try:
        rclpy.spin(node)
    except SystemExit as e:
        exit_code = e.code
    except KeyboardInterrupt:
        exit_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
