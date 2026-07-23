#!/usr/bin/env python3
"""
Main experiment runner for RISE validation.

Runs experiments on all or selected goals, collects data, and computes metrics.
Supports scenario-based experiments with perception interceptor.

Usage:
  python3 run_experiments.py [options]

Options:
  --goals "GOAL1,GOAL2,..."  Run specific goals (default: all) - USE QUOTES!
  --condition CONDITION      Experiment condition: baseline, fault_xxx (default: baseline)
  --stuck-timeout SECONDS    Timeout for stuck detection (default: 150)
  --trials N                 Number of trials per goal (default: 1)
  --skip-reset               Don't reset vehicle between experiments
  --skip-existing            Skip goals that already have successful results
  --dry-run                  Show what would run without executing
  --scenario YAML            Path to scenario YAML for interceptor config
  --scenario-params JSON     JSON override for scenario params (single run)

Examples:
  # Run all experiments (passthrough interceptor)
  python3 run_experiments.py

  # Run with scenario
  python3 run_experiments.py --scenario static_obstacle.yaml --goals "goal_007"

  # Run with inline scenario params
  python3 run_experiments.py --scenario-params '{"strategy":"static_obstacle","distance":100}' --goals "goal_007"

  # Run specific goals (QUOTE the comma-separated list!)
  python3 run_experiments.py --goals "goal_007,goal_008,goal_009"

  # Resume from goal_007, skipping already completed goals
  python3 run_experiments.py --skip-existing

  # Run with longer timeout for distant goals
  python3 run_experiments.py --goals "goal_020,goal_021" --stuck-timeout 200

  # Run multiple trials
  python3 run_experiments.py --condition baseline --trials 3
"""

import argparse
import math
import os
import signal
import sys
import time
import subprocess
import json
from datetime import datetime
from typing import Tuple

# Add lib to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

from config import (
    ExperimentConfig, GoalConfig, load_goals,
    get_recording_topics, DATA_DIR, SCRIPTS_DIR, CONFIG_DIR, campaign_meta_dir
)
from ros_utils import (
    wait_for_topic, get_autoware_state, get_mrm_state, get_velocity,
    get_vehicle_position,
    clear_route, engage_autonomous, change_to_stop, set_goal,
    start_rosbag_recording, stop_rosbag_recording,
    wait_for_mrm_recovery, recover_from_emergency,
    wait_for_clean_diagnostics,
    set_velocity_limit,
    AutowareState, MrmState, MrmBehavior, shutdown_ros
)
from metrics import compute_metrics_from_bag, save_metrics


class ExperimentRunner:
    """Runs and monitors a single experiment."""

    def __init__(self, config: ExperimentConfig):
        self.config = config
        self.rosbag_process = None
        self.result = {
            'status': 'unknown',
            'goal_reached': False,
            'driving_time': 0.0,
            'mrm_trigger_count': 0,
        }

    def setup(self, max_init_wait: float = 60.0) -> bool:
        """Setup experiment (create directories, check prerequisites)."""
        print(f"\n{'='*60}")
        print(f"Setting up experiment: {self.config.experiment_id}")
        print(f"{'='*60}")

        # Create directories first so we can save results even if setup fails
        self.config.create_directories()

        # Check Autoware is running and ready
        state, state_name = get_autoware_state()
        if state is None:
            print(f"ERROR: Cannot get Autoware state")
            return False

        print(f"Autoware state: {state_name}")

        # Handle INITIALIZING state - wait for it to complete
        if state == AutowareState.INITIALIZING:
            print(f"Autoware is initializing, waiting up to {max_init_wait}s...")
            start_wait = time.time()
            while time.time() - start_wait < max_init_wait:
                time.sleep(2)
                state, state_name = get_autoware_state()
                elapsed = time.time() - start_wait
                print(f"  [{elapsed:.0f}s] State: {state_name}")
                if state is not None and state != AutowareState.INITIALIZING:
                    print(f"Autoware ready: {state_name}")
                    break
            else:
                print("ERROR: Autoware still initializing after timeout")
                print("TIP: Try running reset_vehicle.py manually")
                return False

        if state in [AutowareState.DRIVING, AutowareState.PLANNING]:
            print("WARNING: Vehicle is already driving/planning, clearing route...")
            clear_route()
            time.sleep(3)

        # Save metadata
        self.config.save_metadata()

        return True

    def start_recording(self) -> bool:
        """Start rosbag recording."""
        print("Starting rosbag recording...")
        topics = get_recording_topics()
        self.rosbag_process = start_rosbag_recording(
            self.config.rosbag_dir, topics
        )
        if self.rosbag_process is None:
            print("ERROR: Failed to start recording")
            return False
        print(f"Recording to: {self.config.rosbag_dir}")
        return True

    def stop_recording(self):
        """Stop rosbag recording."""
        if self.rosbag_process:
            print("Stopping rosbag recording...")
            stop_rosbag_recording(self.rosbag_process)
            self.rosbag_process = None

    def set_goal_and_engage(self) -> bool:
        """Set goal and engage autonomous mode."""
        goal = self.config.goal

        print(f"Setting goal: ({goal.position['x']:.2f}, {goal.position['y']:.2f})")

        # Set goal
        success = set_goal(
            goal.position['x'],
            goal.position['y'],
            goal.position.get('z', 0.0),
            goal.orientation.get('z', 0.0),
            goal.orientation.get('w', 1.0),
            timeout=60.0
        )

        if not success:
            print("ERROR: Failed to set goal")
            return False

        # Check MRM state before engaging
        mrm_state, mrm_behavior = get_mrm_state()
        if mrm_state is not None and mrm_state != MrmState.NORMAL:
            print(f"WARNING: MRM state is {mrm_state.name}, waiting for NORMAL...")
            wait_start = time.time()
            while time.time() - wait_start < 30:
                mrm_state, _ = get_mrm_state()
                if mrm_state == MrmState.NORMAL:
                    print("MRM state is now NORMAL")
                    break
                time.sleep(1)
            else:
                print(f"ERROR: MRM state still not NORMAL after 30s (current: {mrm_state})")
                return False

        # Wait for planning to produce a trajectory before engaging.
        # On a fresh system start, the control_command_gate has a timeout_builtin
        # diagnostic: if the controller receives no commands within its timeout after
        # engage, the gate fires ERROR → /autoware/system fails → MRM fires immediately.
        # Waiting for a non-empty trajectory guarantees the planning→control pipeline
        # is ready before we activate the controller.
        #
        # If planning doesn't respond in 60s, the system is likely in a contaminated
        # post-MRM state. Do NOT engage — that causes 79 rapid MRM oscillations over
        # 200s with the vehicle never moving (worse than failing fast here).
        # Instead, attempt one route re-set to force the behavior planner to restart
        # planning from scratch, which often clears the contaminated state without a
        # full Autoware restart.
        print("Waiting for planning trajectory before engage...")
        if not self._wait_for_trajectory(timeout=60.0):
            print("No trajectory after 60s — attempting route re-set to recover planning...")
            # Give the behavior planner time to fully unwind from the previous
            # (possibly stuck) trial before we try to re-plan.  A 3s sleep wasn't
            # enough after a 200s stuck scenario; 20s lets the planner settle.
            clear_route()
            time.sleep(20.0)
            # Re-set the same goal to re-trigger the behavior planner
            success = set_goal(
                goal.position['x'],
                goal.position['y'],
                goal.position.get('z', 0.0),
                goal.orientation.get('z', 0.0),
                goal.orientation.get('w', 1.0),
                timeout=60.0
            )
            if not success:
                print("ERROR: Route re-set failed — aborting trial (engage_failed)")
                return False
            print("Route re-set succeeded. Waiting up to 90s for trajectory...")
            if not self._wait_for_trajectory(timeout=90.0):
                print("ERROR: No trajectory after route re-set — aborting trial (engage_failed)")
                print("       System needs a reset before next trial.")
                return False
            print("Trajectory received after route re-set.")

        # Engage autonomous mode
        print("Engaging autonomous mode...")
        engage_autonomous()
        time.sleep(5)  # give control pipeline time to start before MRM check

        # Check MRM state after engage. MRM_SUCCEEDED fires transiently in the
        # first few seconds as the control_command_gate timeout fires before the
        # first control command arrives — this is self-clearing and the vehicle
        # will still drive. Only abort on MRM_FAILED (terminal) or if
        # MRM_SUCCEEDED does not clear within 15 seconds.
        mrm_state, _ = get_mrm_state()
        if mrm_state not in (None, MrmState.NORMAL):
            if mrm_state == MrmState.MRM_FAILED:
                print(f"ERROR: MRM_FAILED after engage — aborting")
                return False
            print(f"  Transient MRM ({mrm_state.name}) after engage — waiting for recovery...")
            if not wait_for_mrm_recovery(timeout=20.0):
                print(f"ERROR: MRM did not recover within 20s after engage — aborting")
                return False
            print("  MRM recovered — proceeding")

        return True

    def _wait_for_trajectory(self, timeout: float = 30.0) -> bool:
        """Wait for planning to publish a non-empty trajectory.

        Polls the trajectory monitor in ros_utils. Falls back to a timed wait
        if the monitor does not support trajectory checking.
        """
        from ros_utils import wait_for_trajectory
        return wait_for_trajectory(timeout=timeout)

    def wait_for_driving(self, timeout: float = 30.0) -> bool:
        """Wait for vehicle to enter DRIVING state."""
        print("Waiting for DRIVING state...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            state, _ = get_autoware_state()
            if state == AutowareState.DRIVING:
                return True
            if state == AutowareState.ARRIVED_GOAL:
                return True
            time.sleep(0.5)

        print("WARNING: Timeout waiting for DRIVING state")
        return False

    def monitor_experiment(self) -> dict:
        """Monitor experiment until completion or stuck."""
        print(f"\nMonitoring experiment (stuck timeout: {self.config.stuck_timeout}s)...")

        driving_start_time = time.time()
        last_moving_time = time.time()
        mrm_trigger_count = 0
        prev_mrm_state = MrmState.NORMAL
        consecutive_zero_velocity = 0
        VELOCITY_THRESHOLD = 0.1  # m/s - below this is considered stopped
        STUCK_VELOCITY_COUNT = 30  # 30 seconds of near-zero velocity = stuck

        while True:
            elapsed = time.time() - driving_start_time

            # Check Autoware state
            state, state_name = get_autoware_state()

            if state == AutowareState.ARRIVED_GOAL:
                print(f"\nGoal reached! Driving time: {elapsed:.1f}s")
                return {
                    'status': 'goal_reached',
                    'goal_reached': True,
                    'driving_time': elapsed,
                    'mrm_trigger_count': mrm_trigger_count,
                }

            # Check MRM state
            mrm_state, mrm_behavior = get_mrm_state()
            if mrm_state is not None:
                if prev_mrm_state == MrmState.NORMAL and mrm_state == MrmState.MRM_OPERATING:
                    mrm_trigger_count += 1
                    print(f"\r[{elapsed:.0f}s] MRM triggered (#{mrm_trigger_count})     ", end='')
                prev_mrm_state = mrm_state

                # Check for terminal MRM states (vehicle stopped and won't recover)
                if mrm_state in [MrmState.MRM_SUCCEEDED, MrmState.MRM_FAILED]:
                    # MRM completed - check if this is permanent
                    consecutive_zero_velocity += 1
                    if consecutive_zero_velocity >= STUCK_VELOCITY_COUNT:
                        behavior_name = mrm_behavior.name if mrm_behavior else "UNKNOWN"
                        print(f"\nMRM completed ({mrm_state.name}, {behavior_name}) - vehicle stopped for {consecutive_zero_velocity}s")
                        return {
                            'status': f'mrm_{mrm_state.name.lower()}',
                            'goal_reached': False,
                            'driving_time': elapsed,
                            'mrm_trigger_count': mrm_trigger_count,
                            'mrm_final_state': mrm_state.name,
                            'mrm_final_behavior': behavior_name,
                        }

            # Check actual velocity for stuck detection
            velocity = get_velocity()
            if velocity is not None:
                if abs(velocity) > VELOCITY_THRESHOLD:
                    last_moving_time = time.time()
                    consecutive_zero_velocity = 0
                else:
                    consecutive_zero_velocity += 1

            time_since_movement = time.time() - last_moving_time

            # Stuck detection: no movement for stuck_timeout seconds
            if time_since_movement > self.config.stuck_timeout:
                print(f"\nStuck detected: no movement for {time_since_movement:.0f}s (velocity: {velocity:.3f} m/s)")
                return {
                    'status': 'stuck',
                    'goal_reached': False,
                    'driving_time': elapsed,
                    'mrm_trigger_count': mrm_trigger_count,
                    'time_since_movement': time_since_movement,
                    'final_velocity': velocity,
                }

            # Safety timeout (absolute maximum time)
            if elapsed > self.config.stuck_timeout * 3:  # 3x stuck timeout as safety
                print(f"\nSafety timeout reached ({elapsed:.1f}s)")
                return {
                    'status': 'timeout',
                    'goal_reached': False,
                    'driving_time': elapsed,
                    'mrm_trigger_count': mrm_trigger_count,
                }

            # Status display with velocity
            mrm_str = f" MRM:{mrm_trigger_count}" if mrm_trigger_count > 0 else ""
            vel_str = f" v={velocity:.1f}" if velocity is not None else ""
            stopped_str = f" (stopped {consecutive_zero_velocity}s)" if consecutive_zero_velocity > 5 else ""
            print(f"\r[{elapsed:.0f}s] {state_name}{mrm_str}{vel_str}{stopped_str}       ", end='', flush=True)

            time.sleep(1.0)

    def run(self) -> bool:
        """Run the complete experiment."""
        # Clear stale obstacle info from any previous run
        try:
            import os as _os
            if _os.path.exists('/tmp/rise_obstacle_info.json'):
                _os.remove('/tmp/rise_obstacle_info.json')
        except Exception:
            pass

        try:
            # Setup
            if not self.setup():
                self.result['status'] = 'setup_failed'
                return False

            # Start recording
            if not self.start_recording():
                self.result['status'] = 'recording_failed'
                return False

            # Set goal and engage
            if not self.set_goal_and_engage():
                self.result['status'] = 'engage_failed'
                return False

            # Wait for driving
            if not self.wait_for_driving():
                self.result['status'] = 'engage_failed'
                return False

            # Optional: wait for stabilization
            if self.config.stabilization_delay > 0:
                print(f"Waiting {self.config.stabilization_delay}s for stabilization...")
                time.sleep(self.config.stabilization_delay)

            # Monitor experiment
            self.result = self.monitor_experiment()
            return self.result.get('goal_reached', False)

        except KeyboardInterrupt:
            print("\n\nExperiment interrupted by user")
            self.result['status'] = 'interrupted'
            return False

        finally:
            # Always stop recording
            self.stop_recording()
            # Save result
            self.config.save_result(self.result)
            # Clear route and try to recover from any emergency state
            print("\nCleaning up experiment...")
            clear_route()
            # Check if we're in emergency state and try to recover
            mrm_state, _ = get_mrm_state()
            if mrm_state is not None and mrm_state != MrmState.NORMAL:
                print(f"System in {mrm_state.name} state, attempting recovery...")
                recover_from_emergency(max_attempts=2)

    def compute_and_save_metrics(self):
        """Compute metrics from recorded data."""
        if not os.path.exists(self.config.rosbag_dir):
            print("No rosbag data to analyze")
            return

        # Try to load injected obstacle UUID written by the interceptor
        injected_uuid_bytes = None
        obstacle_info_path = '/tmp/rise_obstacle_info.json'
        if os.path.exists(obstacle_info_path):
            try:
                with open(obstacle_info_path) as f:
                    obstacle_info = json.load(f)
                uuid_hex = obstacle_info.get('uuid_hex', '')
                if uuid_hex:
                    injected_uuid_bytes = bytes.fromhex(uuid_hex)
                    # Append obstacle placement info to metadata for cross-experiment reproducibility
                    if os.path.exists(self.config.metadata_file):
                        with open(self.config.metadata_file) as mf:
                            metadata = json.load(mf)
                        metadata['injected_obstacle'] = obstacle_info
                        with open(self.config.metadata_file, 'w') as mf:
                            json.dump(metadata, mf, indent=2)
            except Exception as exc:
                print(f"WARNING: Could not load obstacle info: {exc}")

        print(f"\nComputing metrics from: {self.config.rosbag_dir}")
        try:
            metrics = compute_metrics_from_bag(
                self.config.rosbag_dir,
                goal_position=self.config.goal.position,
                injected_uuid_bytes=injected_uuid_bytes,
            )
            save_metrics(metrics, self.config.metrics_file)
            print(f"Metrics saved to: {self.config.metrics_file}")

            # Print summary
            print("\nMetrics Summary:")
            print(f"  Goal reached: {metrics.reliability.goal_reached}")
            print(f"  Driving time: {metrics.reliability.driving_time:.1f}s")
            print(f"  Mean velocity: {metrics.reliability.mean_velocity:.2f} m/s")
            print(f"  MRM triggers: {metrics.fail_operational.mrm_trigger_count}")
            print(f"  Min object distance (all): {metrics.safety.min_object_distance:.2f}m")
            if metrics.safety.min_injected_obstacle_distance < float('inf'):
                print(f"  Min injected obstacle distance: {metrics.safety.min_injected_obstacle_distance:.2f}m")

        except Exception as e:
            print(f"ERROR computing metrics: {e}")


def start_fault_injector(
    tl_fault: str | None,
    tl_params: dict,
    imu_fault: str | None,
    imu_params: dict,
    fault_delay: float,
    fault_duration: float,
    log_file: str,
) -> subprocess.Popen:
    """Launch the fault injector as a subprocess."""
    injector_script = os.path.join(
        os.path.dirname(__file__), '..', 'lib', 'fault_injector.py'
    )

    cmd = ['python3', injector_script, '--log-file', log_file]
    if tl_fault:
        cmd += ['--tl-fault', tl_fault, '--tl-params', json.dumps(tl_params)]
    if imu_fault:
        cmd += ['--imu-fault', imu_fault, '--imu-params', json.dumps(imu_params)]
    if fault_delay > 0:
        cmd += ['--fault-delay', str(fault_delay)]
    if fault_duration > 0:
        cmd += ['--fault-duration', str(fault_duration)]

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(2.0)

    if proc.poll() is not None:
        stderr = proc.stderr.read().decode() if proc.stderr else ''
        raise RuntimeError(f'FaultInjector failed to start: {stderr}')

    print(f"FaultInjector started (PID {proc.pid}): tl={tl_fault}, imu={imu_fault}, delay={fault_delay:.0f}s, duration={fault_duration:.0f}s")
    return proc


def stop_fault_injector(proc: subprocess.Popen):
    """Stop the fault injector gracefully."""
    if proc is None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)
    except Exception:
        pass
    print("FaultInjector stopped")


def start_interceptor(strategy: str, params: dict) -> subprocess.Popen:
    """Launch the perception interceptor as a subprocess."""
    interceptor_script = os.path.join(
        os.path.dirname(__file__), '..', 'lib', 'perception_interceptor.py'
    )

    cmd = [
        'python3', interceptor_script,
        '--strategy', strategy,
        '--params', json.dumps(params),
    ]

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Wait for node to initialize
    time.sleep(2.0)

    if proc.poll() is not None:
        stderr = proc.stderr.read().decode() if proc.stderr else ''
        raise RuntimeError(f'Interceptor failed to start: {stderr}')

    print(f"Interceptor started (PID {proc.pid}): {strategy} {params}")
    return proc


def stop_interceptor(proc: subprocess.Popen):
    """Stop the interceptor gracefully."""
    if proc is None:
        return

    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)
    except Exception:
        pass
    print("Interceptor stopped")


def _get_start_xy() -> Tuple[float, float]:
    """Return (x, y) of the experiment starting position from baseline config."""
    baseline = os.path.join(CONFIG_DIR, 'baseline.json')
    try:
        with open(baseline) as f:
            cfg = json.load(f)
        pos = cfg['egoConfiguration']['egoPosition']
        return pos['x'], pos['y']
    except Exception:
        return 81384.60, 49922.00  # hard-coded fallback


# Vehicles within this distance of start don't need an AWSIM teleport.
_NEAR_START_THRESHOLD_M = 25.0


def reset_vehicle(max_retries: int = 2):
    """Reset vehicle to starting position with retry on failure.

    If the vehicle is already within _NEAR_START_THRESHOLD_M of the start pose,
    skip the AWSIM teleport and only clear the route + wait for diagnostics.
    The teleport triggers a full localization re-init which is expensive and
    unnecessary when the vehicle hasn't moved far from start (e.g. early MRM).
    """
    reset_script = os.path.join(SCRIPTS_DIR, 'reset_vehicle.py')
    if not os.path.exists(reset_script):
        print("WARNING: reset_vehicle.py not found")
        return False

    # First, try to clear any emergency state
    mrm_state, _ = get_mrm_state()
    if mrm_state is not None and mrm_state != MrmState.NORMAL:
        print(f"System in {mrm_state.name} state, clearing route first...")
        clear_route()
        time.sleep(2)
        wait_for_mrm_recovery(timeout=10.0)

    # Check whether the vehicle is already near the start position.
    # If so, skip the teleport — route is already cleared above.
    pos = get_vehicle_position()
    if pos is not None:
        sx, sy = _get_start_xy()
        dist = math.hypot(pos[0] - sx, pos[1] - sy)
        if dist <= _NEAR_START_THRESHOLD_M:
            print(f"  Vehicle {dist:.1f}m from start — skipping teleport, waiting for diagnostics")
            if wait_for_clean_diagnostics(timeout=60.0, stable_duration=5.0):
                return True
            print("  Diagnostics did not clear — falling back to full teleport reset")

    for attempt in range(max_retries):
        try:
            result = subprocess.run(
                ['python3', reset_script, '--wait'],
                timeout=90,
                capture_output=False
            )
            if result.returncode == 0:
                time.sleep(2)
                mrm_state, _ = get_mrm_state()
                if mrm_state != MrmState.NORMAL:
                    print(f"WARNING: MRM state after reset: {mrm_state}")
                    recover_from_emergency(max_attempts=1)
                # Wait for /diagnostics_agg to show zero ERROR entries for 5s.
                # MRM NORMAL is a symptom of clean diagnostics, not the cause —
                # checking diagnostics directly catches the contaminated state where
                # 14+ nodes are simultaneously in ERROR after a bad MRM cascade.
                if wait_for_clean_diagnostics(timeout=60.0, stable_duration=5.0):
                    return True
                print("  Diagnostics did not clear after reset — retrying...")
            else:
                print(f"WARNING: Reset returned non-zero exit code: {result.returncode}")
        except subprocess.TimeoutExpired:
            print(f"WARNING: Reset timed out (attempt {attempt + 1}/{max_retries})")
        except Exception as e:
            print(f"WARNING: Reset failed: {e}")

        if attempt < max_retries - 1:
            print("Retrying reset...")
            time.sleep(5)

    print("ERROR: All reset attempts failed")
    return False


def check_experiment_exists(goal_id: str, campaign_dir: str) -> bool:
    """Check if a successful experiment already exists for this goal."""
    goal_dir = os.path.join(campaign_dir, goal_id)
    if not os.path.isdir(goal_dir):
        return False
    for entry in os.listdir(goal_dir):
        result_file = os.path.join(goal_dir, entry, 'result.json')
        if os.path.exists(result_file):
            try:
                with open(result_file) as f:
                    result = json.load(f)
                if result.get('goal_reached') or result.get('status') == 'goal_reached':
                    return True
            except Exception:
                pass
    return False


def main():
    parser = argparse.ArgumentParser(description='Run RISE validation experiments')
    parser.add_argument('--goals', type=str, default=None,
                        help='Comma-separated goal IDs (default: all)')
    parser.add_argument('--goals-file', type=str, default=None,
                        help='Path to goals JSON, filename resolved against '
                             'experiments/configs/ if not absolute (default: captured_goals.json)')
    parser.add_argument('--condition', type=str, default='baseline',
                        help='Experiment condition (default: baseline)')
    parser.add_argument('--stuck-timeout', type=float, default=150.0,
                        help='Stuck detection timeout in seconds')
    parser.add_argument('--trials', type=int, default=1,
                        help='Number of trials per goal')
    parser.add_argument('--skip-reset', action='store_true',
                        help='Skip vehicle reset between experiments')
    parser.add_argument('--skip-existing', action='store_true',
                        help='Skip goals that already have successful results')
    parser.add_argument('--dry-run', action='store_true',
                        help='Show what would run without executing')
    parser.add_argument('--yes', '-y', action='store_true',
                        help='Skip the "Press Enter to start" confirmation (for unattended/scripted runs)')
    parser.add_argument('--compute-metrics-only', type=str, default=None,
                        help='Only compute metrics for existing experiment')
    parser.add_argument('--scenario', type=str, default=None,
                        help='Path to scenario YAML (enables interceptor)')
    parser.add_argument('--scenario-params', type=str, default=None,
                        help='JSON string of scenario params (inline, no YAML needed)')
    parser.add_argument('--campaign', type=str, default='default',
                        help='Campaign name (subdirectory under data/)')
    parser.add_argument('--velocity-limit', type=float, default=0.0,
                        help='Max velocity cap in m/s (0 = Autoware default ~20 m/s)')
    parser.add_argument('--tl-fault', type=str, default=None,
                        choices=['tl_confidence', 'tl_oscillate', 'tl_unknown', 'tl_blackout'],
                        help='Traffic light fault mode')
    parser.add_argument('--tl-params', type=str, default='{}',
                        help='JSON params for TL fault, e.g. {"dropout_rate":0.5}')
    parser.add_argument('--imu-fault', type=str, default=None,
                        choices=['imu_bias'],
                        help='IMU fault mode')
    parser.add_argument('--imu-params', type=str, default='{}',
                        help='JSON params for IMU fault, e.g. {"accel_bias_ms2":0.5,"gyro_bias_rads":0.1}')
    parser.add_argument('--fault-delay', type=float, default=30.0,
                        help='Seconds after engage before faults activate (default: 30)')
    parser.add_argument('--fault-duration', type=float, default=45.0,
                        help='Seconds the fault stays active before auto-deactivating (default: 45)')
    args = parser.parse_args()

    # Handle metrics-only mode
    if args.compute_metrics_only:
        bag_path = args.compute_metrics_only
        if not os.path.exists(bag_path):
            print(f"ERROR: Path not found: {bag_path}")
            return 1
        try:
            metrics = compute_metrics_from_bag(bag_path)
            output_file = os.path.join(os.path.dirname(bag_path), 'metrics.json')
            save_metrics(metrics, output_file)
            print(f"Metrics saved to: {output_file}")
            print(json.dumps(metrics.to_dict(), indent=2))
        except Exception as e:
            print(f"ERROR: {e}")
            return 1
        return 0

    # Load goals
    goals_file = args.goals_file
    if goals_file and not os.path.isabs(goals_file):
        goals_file = os.path.join(CONFIG_DIR, goals_file)
    try:
        all_goals = load_goals(goals_file)
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        print("Run capture_goals_session.py first to capture goals.")
        return 1

    # Resolve scenario configuration
    scenario_type = 'passthrough'
    scenario_params = {}
    if args.scenario_params:
        sp = json.loads(args.scenario_params)
        scenario_type = sp.pop('strategy', sp.pop('scenario_type', 'passthrough'))
        scenario_params = sp
    elif args.scenario:
        scenario_path = args.scenario
        if not os.path.isabs(scenario_path):
            candidate = os.path.join(CONFIG_DIR, 'scenarios', scenario_path)
            if os.path.exists(candidate):
                scenario_path = candidate
        if os.path.exists(scenario_path):
            from scenarios import load_scenario
            sc = load_scenario(scenario_path)
            scenario_type = sc.scenario_type.value
            scenario_params = sc.params
            if args.condition == 'baseline':
                args.condition = scenario_type
        else:
            print(f"WARNING: Scenario file not found: {scenario_path}")

    # Filter goals
    if args.goals:
        goal_ids = [g.strip() for g in args.goals.split(',')]
        goals = [g for g in all_goals if g.id in goal_ids]
        if not goals:
            print(f"ERROR: No matching goals found for: {args.goals}")
            print(f"Available: {', '.join(g.id for g in all_goals)}")
            return 1
    else:
        goals = all_goals

    # Display plan
    print("="*60)
    print("RISE EXPERIMENT RUNNER")
    print("="*60)
    print(f"Goals to run: {len(goals)}")
    print(f"Trials per goal: {args.trials}")
    print(f"Condition: {args.condition}")
    print(f"Scenario: {scenario_type} {scenario_params if scenario_params else ''}")
    print(f"Campaign: {args.campaign}")
    print(f"Velocity limit: {args.velocity_limit:.1f} m/s" if args.velocity_limit > 0 else "Velocity limit: default")
    print(f"Stuck timeout: {args.stuck_timeout}s")
    print()
    print("Goals:")
    for g in goals:
        dist_str = f" (~{g.estimated_distance:.0f}m)" if g.estimated_distance else ""
        print(f"  {g.id}: ({g.position['x']:.1f}, {g.position['y']:.1f}){dist_str}")

    if args.dry_run:
        print("\nDRY RUN - not executing")
        return 0

    print()
    if sys.stdin.isatty() and not args.yes:
        input("Press Enter to start experiments (Ctrl+C to cancel)...")

    # Apply velocity limit if specified
    if args.velocity_limit > 0:
        print(f"\nApplying velocity limit: {args.velocity_limit:.1f} m/s")
        set_velocity_limit(args.velocity_limit)
        time.sleep(0.5)

    # Initial reset to ensure clean state
    if not args.skip_reset:
        print("\nPerforming initial vehicle reset...")
        if not reset_vehicle():
            print("WARNING: Initial reset failed, continuing anyway...")
        time.sleep(3)

    # Start interceptor (runs for the entire batch)
    interceptor_proc = None
    try:
        print(f"\nStarting interceptor: {scenario_type}")
        interceptor_proc = start_interceptor(scenario_type, scenario_params)
    except RuntimeError as e:
        print(f"ERROR: Failed to start interceptor: {e}")
        return 1

    # Start fault injector (always-running relay; passthrough when no fault specified)
    fault_injector_proc = None
    tl_params_parsed  = json.loads(args.tl_params) if isinstance(args.tl_params, str) else args.tl_params
    imu_params_parsed = json.loads(args.imu_params) if isinstance(args.imu_params, str) else args.imu_params
    meta_dir          = campaign_meta_dir(args.campaign)
    batch_fault_log   = os.path.join(meta_dir, 'fault_log.jsonl')
    os.makedirs(meta_dir, exist_ok=True)
    try:
        print(f"\nStarting fault injector: tl={args.tl_fault or 'passthrough'}, imu={args.imu_fault or 'passthrough'}")
        fault_injector_proc = start_fault_injector(
            tl_fault=args.tl_fault,
            tl_params=tl_params_parsed,
            imu_fault=args.imu_fault,
            imu_params=imu_params_parsed,
            fault_delay=args.fault_delay,
            fault_duration=args.fault_duration,
            log_file=batch_fault_log,
        )
    except RuntimeError as e:
        print(f"ERROR: Failed to start fault injector: {e}")
        stop_interceptor(interceptor_proc)
        return 1

    # Run experiments
    results = []
    skipped_count = 0
    total_experiments = len(goals) * args.trials

    try:
        for trial in range(args.trials):
            for i, goal in enumerate(goals):
                exp_num = trial * len(goals) + i + 1
                print(f"\n{'#'*60}")
                print(f"# EXPERIMENT {exp_num}/{total_experiments}")
                print(f"# Goal: {goal.id}, Trial: {trial+1}/{args.trials}")
                print(f"{'#'*60}")

                # Check if we should skip this goal
                campaign_dir = os.path.join(DATA_DIR, args.campaign)
                if args.skip_existing and os.path.isdir(campaign_dir):
                    if check_experiment_exists(goal.id, campaign_dir):
                        print(f"SKIPPING: {goal.id} already has successful result")
                        skipped_count += 1
                        continue

                # Create experiment ID
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                exp_id = f"{goal.id}_{args.condition}_t{trial+1}_{timestamp}"

                # Re-apply velocity limit before each trial (Autoware's velocity
                # smoother can lose the external limit when the route is cleared
                # between experiments).
                if args.velocity_limit > 0:
                    set_velocity_limit(args.velocity_limit)
                    time.sleep(0.2)

                # Create config
                config = ExperimentConfig(
                    experiment_id=exp_id,
                    goal=goal,
                    trial_num=trial + 1,
                    timestamp=timestamp,
                    stuck_timeout=args.stuck_timeout,
                    condition=args.condition,
                    scenario_type=scenario_type,
                    scenario_params={**scenario_params,
                                     **({"velocity_limit_mps": args.velocity_limit}
                                        if args.velocity_limit > 0 else {})},
                    campaign=args.campaign,
                )

                # Run experiment
                runner = ExperimentRunner(config)
                success = runner.run()

                # Compute metrics
                runner.compute_and_save_metrics()

                # Copy fault log snapshot into this run's directory
                if os.path.exists(batch_fault_log):
                    import shutil
                    run_fault_log = os.path.join(runner.config.data_dir, 'fault_log.jsonl')
                    try:
                        shutil.copy2(batch_fault_log, run_fault_log)
                    except Exception as exc:
                        print(f"WARNING: Could not copy fault log: {exc}")

                results.append({
                    'experiment_id': exp_id,
                    'goal_id': goal.id,
                    'trial': trial + 1,
                    'success': success,
                    'result': runner.result,
                })

                # Reset vehicle before next experiment
                if not args.skip_reset and (i < len(goals) - 1 or trial < args.trials - 1):
                    print("\nResetting vehicle...")
                    if not reset_vehicle():
                        # First reset failed — the system is likely in the contaminated
                        # oscillation state. Wait for the diagnostic flood to decay, then
                        # try once more before giving up and moving on.
                        print("WARNING: Reset failed — waiting 20s for diagnostics to clear...")
                        time.sleep(20)
                        print("Attempting second reset...")
                        if not reset_vehicle():
                            print("ERROR: Second reset also failed — system in bad state.")
                            print("       Waiting 30s before next run (data may be contaminated).")
                            time.sleep(30)
                        else:
                            time.sleep(5)
                    else:
                        time.sleep(5)
    finally:
        # Always stop interceptor and fault injector
        stop_interceptor(interceptor_proc)
        stop_fault_injector(fault_injector_proc)

    # Summary
    print("\n" + "="*60)
    print("EXPERIMENT SUMMARY")
    print("="*60)

    success_count = sum(1 for r in results if r['success'])
    failed_count = len(results) - success_count
    print(f"Total run: {len(results)}, Successful: {success_count}, Failed: {failed_count}")
    if skipped_count > 0:
        print(f"Skipped (already completed): {skipped_count}")

    print("\nResults:")
    for r in results:
        status = "SUCCESS" if r['success'] else r['result'].get('status', 'FAILED')
        mrm = r['result'].get('mrm_trigger_count', 0)
        time_str = f"{r['result'].get('driving_time', 0):.0f}s"
        print(f"  {r['experiment_id']}: {status} (MRM: {mrm}, Time: {time_str})")

    # Save summary
    meta_dir = campaign_meta_dir(args.campaign)
    os.makedirs(meta_dir, exist_ok=True)
    summary_file = os.path.join(meta_dir, f"batch_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
    with open(summary_file, 'w') as f:
        json.dump({
            'timestamp': datetime.now().isoformat(),
            'condition': args.condition,
            'total_experiments': len(results),
            'successful': success_count,
            'results': results,
        }, f, indent=2)
    print(f"\nSummary saved to: {summary_file}")

    return 0 if success_count == len(results) else 1


if __name__ == '__main__':
    exit_code = 1
    try:
        exit_code = main()
    finally:
        try:
            shutdown_ros()
        except Exception:
            pass
    # rclpy C++ destructors throw during Python's normal exit path, producing
    # "terminate called without an active exception" + core dump. os._exit()
    # bypasses Python garbage collection and atexit handlers, exiting cleanly.
    # It also skips the normal stdio flush — without this, every print() since
    # the last buffer flush (stdout is block-buffered when not a TTY) is
    # silently lost, including error messages. Cost real debugging time
    # (2026-07-21) tracking down a silent exit-1 that was actually a clear
    # "ERROR: Goals file not found" message nobody ever saw.
    sys.stdout.flush()
    sys.stderr.flush()
    import os as _os
    _os._exit(exit_code)
