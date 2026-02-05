"""
ROS2 utility functions for experiment control.

Uses rclpy for reliable ROS2 communication instead of subprocess calls.
"""

import subprocess
import time
import threading
from typing import Optional, Tuple
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor

# Autoware message types
from autoware_system_msgs.msg import AutowareState as AutowareStateMsg
from autoware_adapi_v1_msgs.msg import RouteState as RouteStateMsg
from autoware_adapi_v1_msgs.msg import MrmState as MrmStateMsg
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_vehicle_msgs.msg import Engage, VelocityReport
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header


class AutowareState(IntEnum):
    """Autoware state machine states."""
    UNKNOWN = 0
    INITIALIZING = 1
    WAITING_FOR_ROUTE = 2
    PLANNING = 3
    WAITING_FOR_ENGAGE = 4
    DRIVING = 5
    ARRIVED_GOAL = 6


class RouteState(IntEnum):
    """Route state machine states."""
    UNKNOWN = 0
    UNSET = 1
    SET = 2
    ARRIVED = 3
    CHANGING = 4


class MrmState(IntEnum):
    """MRM state machine states."""
    UNKNOWN = 0
    NORMAL = 1
    MRM_OPERATING = 2
    MRM_SUCCEEDED = 3
    MRM_FAILED = 4


class MrmBehavior(IntEnum):
    """MRM behavior types."""
    UNKNOWN = 0
    NONE = 1
    EMERGENCY_STOP = 2
    COMFORTABLE_STOP = 3
    PULL_OVER = 4


# Global ROS2 state monitor node
_monitor_node = None
_executor = None
_spin_thread = None


class StateMonitorNode(Node):
    """ROS2 node for monitoring Autoware states."""

    def __init__(self):
        super().__init__('experiment_state_monitor')

        # State storage with thread-safe access
        self._lock = threading.Lock()
        self._autoware_state = None
        self._route_state = None
        self._mrm_state = None
        self._mrm_behavior = None
        self._velocity = None
        self._velocity_time = None

        # QoS for volatile topics (autoware/state, mrm_state)
        volatile_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for transient local topics (routing/state)
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions with appropriate QoS
        self.create_subscription(
            AutowareStateMsg,
            '/autoware/state',
            self._autoware_state_callback,
            volatile_qos
        )

        self.create_subscription(
            RouteStateMsg,
            '/api/routing/state',
            self._route_state_callback,
            transient_qos
        )

        self.create_subscription(
            MrmStateMsg,
            '/system/fail_safe/mrm_state',
            self._mrm_state_callback,
            volatile_qos
        )

        self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self._velocity_callback,
            volatile_qos
        )

        self.get_logger().info('State monitor initialized')

    def _autoware_state_callback(self, msg):
        with self._lock:
            self._autoware_state = msg.state

    def _route_state_callback(self, msg):
        with self._lock:
            self._route_state = msg.state

    def _mrm_state_callback(self, msg):
        with self._lock:
            self._mrm_state = msg.state
            self._mrm_behavior = msg.behavior

    def _velocity_callback(self, msg):
        with self._lock:
            self._velocity = msg.longitudinal_velocity
            self._velocity_time = time.time()

    def get_autoware_state(self) -> Tuple[Optional[AutowareState], str]:
        with self._lock:
            if self._autoware_state is None:
                return None, "UNKNOWN"
            try:
                state = AutowareState(self._autoware_state)
                return state, state.name
            except ValueError:
                return None, f"UNKNOWN({self._autoware_state})"

    def get_route_state(self) -> Tuple[Optional[RouteState], str]:
        with self._lock:
            if self._route_state is None:
                return None, "UNKNOWN"
            try:
                state = RouteState(self._route_state)
                return state, state.name
            except ValueError:
                return None, f"UNKNOWN({self._route_state})"

    def get_mrm_state(self) -> Tuple[Optional[MrmState], Optional[MrmBehavior]]:
        with self._lock:
            state = None
            behavior = None
            if self._mrm_state is not None:
                try:
                    state = MrmState(self._mrm_state)
                except ValueError:
                    pass
            if self._mrm_behavior is not None:
                try:
                    behavior = MrmBehavior(self._mrm_behavior)
                except ValueError:
                    pass
            return state, behavior

    def get_velocity(self) -> Tuple[Optional[float], Optional[float]]:
        """Get current velocity and time since last update."""
        with self._lock:
            if self._velocity is None:
                return None, None
            age = time.time() - self._velocity_time if self._velocity_time else None
            return self._velocity, age


def _ensure_node_running():
    """Ensure the state monitor node is running and valid."""
    global _monitor_node, _executor, _spin_thread

    # Check if node exists and context is still valid
    if _monitor_node is not None:
        try:
            # Try to check if context is valid by accessing the node
            if rclpy.ok():
                return
        except Exception:
            pass
        # Context invalid, need to reinitialize
        print("ROS2 context invalid, reinitializing...")
        _monitor_node = None
        _executor = None
        _spin_thread = None

    # Initialize ROS2 if needed
    try:
        if not rclpy.ok():
            rclpy.init()
    except RuntimeError:
        # Already initialized but in bad state, try shutdown and reinit
        try:
            rclpy.shutdown()
        except:
            pass
        rclpy.init()

    _monitor_node = StateMonitorNode()
    _executor = SingleThreadedExecutor()
    _executor.add_node(_monitor_node)

    def spin():
        while rclpy.ok():
            _executor.spin_once(timeout_sec=0.1)

    _spin_thread = threading.Thread(target=spin, daemon=True)
    _spin_thread.start()

    # Wait for initial state messages (volatile topics need time to receive)
    time.sleep(1.0)


def shutdown_ros():
    """Shutdown ROS2 cleanly."""
    global _monitor_node, _executor, _spin_thread

    if _executor is not None:
        _executor.shutdown()
    if _monitor_node is not None:
        _monitor_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    _monitor_node = None
    _executor = None
    _spin_thread = None


def wait_for_topic(topic: str, timeout: float = 30.0) -> bool:
    """Wait for a topic to be available."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'info', topic],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1)
    return False


def get_autoware_state(timeout: float = 5.0) -> Tuple[Optional[AutowareState], str]:
    """Get current Autoware state."""
    _ensure_node_running()

    start_time = time.time()
    while time.time() - start_time < timeout:
        state, name = _monitor_node.get_autoware_state()
        if state is not None:
            return state, name
        time.sleep(0.1)

    return None, "TIMEOUT"


def get_route_state(timeout: float = 5.0) -> Tuple[Optional[RouteState], str]:
    """Get current route state."""
    _ensure_node_running()

    start_time = time.time()
    while time.time() - start_time < timeout:
        state, name = _monitor_node.get_route_state()
        if state is not None:
            return state, name
        time.sleep(0.1)

    return None, "TIMEOUT"


def get_mrm_state(timeout: float = 5.0) -> Tuple[Optional[MrmState], Optional[MrmBehavior]]:
    """Get current MRM state and behavior."""
    _ensure_node_running()

    start_time = time.time()
    while time.time() - start_time < timeout:
        state, behavior = _monitor_node.get_mrm_state()
        if state is not None:
            return state, behavior
        time.sleep(0.1)

    return None, None


def get_velocity(timeout: float = 2.0) -> Optional[float]:
    """Get current vehicle velocity."""
    _ensure_node_running()

    start_time = time.time()
    while time.time() - start_time < timeout:
        velocity, age = _monitor_node.get_velocity()
        if velocity is not None and age is not None and age < 1.0:
            return velocity
        time.sleep(0.1)

    return None


def clear_route(timeout: float = 10.0) -> bool:
    """Clear the current route."""
    try:
        from autoware_adapi_v1_msgs.srv import ClearRoute

        _ensure_node_running()

        client = _monitor_node.create_client(ClearRoute, '/api/routing/clear_route')
        if not client.wait_for_service(timeout_sec=5.0):
            print("WARNING: clear_route service not available")
            return False

        request = ClearRoute.Request()
        future = client.call_async(request)

        start_time = time.time()
        while not future.done() and time.time() - start_time < timeout:
            time.sleep(0.1)

        if future.done():
            response = future.result()
            return response.status.success
        return False

    except Exception as e:
        print(f"WARNING: clear_route failed: {e}")
        return False


def wait_for_mrm_recovery(timeout: float = 30.0) -> bool:
    """Wait for MRM to return to NORMAL state."""
    _ensure_node_running()

    print(f"Waiting for MRM recovery (timeout: {timeout}s)...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        mrm_state, mrm_behavior = get_mrm_state(timeout=2.0)

        if mrm_state == MrmState.NORMAL:
            elapsed = time.time() - start_time
            print(f"  MRM recovered to NORMAL after {elapsed:.1f}s")
            return True

        elapsed = time.time() - start_time
        state_name = mrm_state.name if mrm_state else "UNKNOWN"
        behavior_name = mrm_behavior.name if mrm_behavior else "?"
        if int(elapsed) % 5 == 0:
            print(f"  [{elapsed:.0f}s] MRM state: {state_name} ({behavior_name})")

        time.sleep(1.0)

    print(f"  MRM did not recover within {timeout}s")
    return False


def recover_from_emergency(max_attempts: int = 3) -> bool:
    """Attempt to recover from emergency/MRM state.

    This clears the route and waits for MRM to recover, which should
    allow planning to restart properly.
    """
    _ensure_node_running()

    for attempt in range(max_attempts):
        print(f"\nRecovery attempt {attempt + 1}/{max_attempts}...")

        # Check current state
        mrm_state, _ = get_mrm_state()
        if mrm_state == MrmState.NORMAL:
            print("  Already in NORMAL state")
            return True

        # Clear route to stop planning errors
        print("  Clearing route...")
        clear_route()
        time.sleep(2)

        # Wait for MRM to recover
        if wait_for_mrm_recovery(timeout=15.0):
            return True

        # If still not recovered, wait longer
        print("  Waiting additional time...")
        time.sleep(5)

    print("Recovery failed after all attempts")
    return False


def engage_autonomous(timeout: float = 10.0) -> bool:
    """Engage autonomous driving mode."""
    try:
        from autoware_adapi_v1_msgs.srv import ChangeOperationMode

        _ensure_node_running()

        # Try ADAPI service first
        client = _monitor_node.create_client(
            ChangeOperationMode,
            '/api/operation_mode/change_to_autonomous'
        )

        if client.wait_for_service(timeout_sec=5.0):
            request = ChangeOperationMode.Request()
            future = client.call_async(request)

            start_time = time.time()
            while not future.done() and time.time() - start_time < timeout:
                time.sleep(0.1)

        # Also publish engage topic for compatibility
        engage_pub = _monitor_node.create_publisher(
            Engage,
            '/autoware/engage',
            10
        )
        engage_msg = Engage()
        engage_msg.engage = True
        engage_pub.publish(engage_msg)
        time.sleep(0.5)  # Let the message propagate

        return True

    except Exception as e:
        print(f"WARNING: engage_autonomous failed: {e}")
        return False


def set_goal(x: float, y: float, z: float, qz: float, qw: float,
             timeout: float = 60.0) -> bool:
    """Set goal via ADAPI service.

    Uses proper ROS2 service client with correct message types.
    Waits for route state to become SET before returning.
    """
    _ensure_node_running()

    try:
        # Wait for route to be UNSET first
        start_time = time.time()
        while time.time() - start_time < 10.0:
            route_state, _ = get_route_state(timeout=2.0)
            if route_state == RouteState.UNSET:
                break
            if route_state in [RouteState.SET, RouteState.ARRIVED]:
                print("  Route already set, clearing first...")
                clear_route()
                time.sleep(1)
            time.sleep(0.5)

        # Create service client
        client = _monitor_node.create_client(
            SetRoutePoints,
            '/api/routing/set_route_points'
        )

        if not client.wait_for_service(timeout_sec=10.0):
            print("ERROR: set_route_points service not available")
            return False

        # Build request with proper message types
        request = SetRoutePoints.Request()
        request.header = Header()
        request.header.stamp = _monitor_node.get_clock().now().to_msg()
        request.header.frame_id = 'map'

        request.goal = Pose()
        request.goal.position = Point(x=x, y=y, z=z)
        request.goal.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        request.waypoints = []
        request.option.allow_goal_modification = True

        print(f"  Calling /api/routing/set_route_points...")
        print(f"    Goal: ({x:.2f}, {y:.2f}, {z:.2f})")
        print(f"    Orientation: (z={qz:.4f}, w={qw:.4f})")

        # Call service
        future = client.call_async(request)

        # Wait for response
        call_start = time.time()
        while not future.done() and time.time() - call_start < 30.0:
            time.sleep(0.1)

        if not future.done():
            print("ERROR: Service call timed out")
            return False

        response = future.result()
        if not response.status.success:
            print(f"ERROR: Service returned failure: {response.status.message}")
            return False

        print(f"  Service response: SUCCESS")

        # Wait for route state to become SET
        wait_start = time.time()
        while time.time() - wait_start < timeout - 30:
            route_state, state_name = get_route_state(timeout=2.0)
            if route_state == RouteState.SET:
                print(f"  Route state: SET")
                return True
            elapsed = time.time() - wait_start
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                print(f"  [{elapsed:.0f}s] Waiting for route SET (current: {state_name})")
            time.sleep(0.5)

        print("ERROR: Route did not become SET within timeout")
        return False

    except Exception as e:
        print(f"ERROR: set_goal failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def start_rosbag_recording(output_dir: str, topics: list) -> Optional[subprocess.Popen]:
    """Start rosbag recording in background."""
    cmd = ['ros2', 'bag', 'record', '-o', output_dir] + topics
    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        time.sleep(2)  # Wait for recording to start
        if process.poll() is None:  # Still running
            return process
    except Exception:
        pass
    return None


def stop_rosbag_recording(process: subprocess.Popen, timeout: float = 10.0):
    """Stop rosbag recording gracefully."""
    if process is None:
        return

    try:
        process.terminate()
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)
    except Exception:
        pass
