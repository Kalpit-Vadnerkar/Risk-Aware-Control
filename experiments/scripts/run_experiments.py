#!/usr/bin/env python3
"""
Main experiment runner for RISE validation.

Runs experiments on all or selected goals, collects data, and computes metrics.

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

Examples:
  # Run all experiments
  python3 run_experiments.py

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
import os
import sys
import time
import subprocess
import json
from datetime import datetime

# Add lib to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

from config import (
    ExperimentConfig, GoalConfig, load_goals,
    get_recording_topics, DATA_DIR, SCRIPTS_DIR
)
from ros_utils import (
    wait_for_topic, get_autoware_state, get_mrm_state, get_velocity,
    clear_route, engage_autonomous, set_goal,
    start_rosbag_recording, stop_rosbag_recording,
    wait_for_mrm_recovery, recover_from_emergency,
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

        time.sleep(2)

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

        # Engage autonomous mode
        print("Engaging autonomous mode...")
        engage_autonomous()
        time.sleep(3)  # Increased wait time

        # Verify we're not in MRM state after engage
        mrm_state, mrm_behavior = get_mrm_state()
        if mrm_state is not None and mrm_state != MrmState.NORMAL:
            print(f"WARNING: MRM triggered immediately after engage ({mrm_state.name})")
            # Give it time to recover
            time.sleep(5)

        return True

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
                self.result['status'] = 'goal_setting_failed'
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

        print(f"\nComputing metrics from: {self.config.rosbag_dir}")
        try:
            metrics = compute_metrics_from_bag(
                self.config.rosbag_dir,
                goal_position=self.config.goal.position
            )
            save_metrics(metrics, self.config.metrics_file)
            print(f"Metrics saved to: {self.config.metrics_file}")

            # Print summary
            print("\nMetrics Summary:")
            print(f"  Goal reached: {metrics.reliability.goal_reached}")
            print(f"  Driving time: {metrics.reliability.driving_time:.1f}s")
            print(f"  Mean velocity: {metrics.reliability.mean_velocity:.2f} m/s")
            print(f"  MRM triggers: {metrics.fail_operational.mrm_trigger_count}")
            print(f"  Min object distance: {metrics.safety.min_object_distance:.2f}m")

        except Exception as e:
            print(f"ERROR computing metrics: {e}")


def reset_vehicle(max_retries: int = 2):
    """Reset vehicle to starting position with retry on failure."""
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
        # Wait a bit for MRM to potentially recover
        wait_for_mrm_recovery(timeout=10.0)

    for attempt in range(max_retries):
        try:
            result = subprocess.run(
                ['python3', reset_script, '--wait'],
                timeout=90,  # Increased timeout
                capture_output=False
            )
            if result.returncode == 0:
                # After reset, verify MRM is NORMAL
                time.sleep(2)
                mrm_state, _ = get_mrm_state()
                if mrm_state == MrmState.NORMAL:
                    return True
                else:
                    print(f"WARNING: MRM state after reset: {mrm_state}")
                    # Try to recover
                    if recover_from_emergency(max_attempts=1):
                        return True
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


def check_experiment_exists(goal_id: str, condition: str, data_dir: str) -> bool:
    """Check if a successful experiment already exists for this goal."""
    # Look for existing experiment directories
    for entry in os.listdir(data_dir):
        if entry.startswith(f"{goal_id}_{condition}_"):
            result_file = os.path.join(data_dir, entry, 'result.json')
            if os.path.exists(result_file):
                try:
                    with open(result_file) as f:
                        result = json.load(f)
                    if result.get('goal_reached') or result.get('status') == 'goal_reached':
                        return True
                except:
                    pass
    return False


def main():
    parser = argparse.ArgumentParser(description='Run RISE validation experiments')
    parser.add_argument('--goals', type=str, default=None,
                        help='Comma-separated goal IDs (default: all)')
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
    parser.add_argument('--compute-metrics-only', type=str, default=None,
                        help='Only compute metrics for existing experiment')
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
    try:
        all_goals = load_goals()
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        print("Run capture_goals_session.py first to capture goals.")
        return 1

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
    input("Press Enter to start experiments (Ctrl+C to cancel)...")

    # Initial reset to ensure clean state
    if not args.skip_reset:
        print("\nPerforming initial vehicle reset...")
        if not reset_vehicle():
            print("WARNING: Initial reset failed, continuing anyway...")
        time.sleep(3)

    # Run experiments
    results = []
    skipped_count = 0
    total_experiments = len(goals) * args.trials

    for trial in range(args.trials):
        for i, goal in enumerate(goals):
            exp_num = trial * len(goals) + i + 1
            print(f"\n{'#'*60}")
            print(f"# EXPERIMENT {exp_num}/{total_experiments}")
            print(f"# Goal: {goal.id}, Trial: {trial+1}/{args.trials}")
            print(f"{'#'*60}")

            # Check if we should skip this goal
            if args.skip_existing:
                if check_experiment_exists(goal.id, args.condition, DATA_DIR):
                    print(f"SKIPPING: {goal.id} already has successful result")
                    skipped_count += 1
                    continue

            # Create experiment ID
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            exp_id = f"{goal.id}_{args.condition}_t{trial+1}_{timestamp}"

            # Create config
            config = ExperimentConfig(
                experiment_id=exp_id,
                goal=goal,
                stuck_timeout=args.stuck_timeout,
                condition=args.condition,
            )

            # Run experiment
            runner = ExperimentRunner(config)
            success = runner.run()

            # Compute metrics
            runner.compute_and_save_metrics()

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
                    print("WARNING: Reset failed, waiting 10s before continuing...")
                    time.sleep(10)
                else:
                    time.sleep(5)

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
    summary_file = os.path.join(DATA_DIR, f"batch_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
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
    try:
        exit_code = main()
    finally:
        # Clean shutdown of ROS2
        shutdown_ros()
    sys.exit(exit_code)
