#!/usr/bin/env python3
"""
Scenario sweep runner for RISE validation.

Loads a scenario YAML, expands the parameter grid, and runs each combination
across selected goals and trials. Produces a sweep summary with degradation curves.

Usage:
  python3 run_scenario_sweep.py SCENARIO_YAML [options]

Examples:
  # Dry run - show experiment matrix
  python3 run_scenario_sweep.py ../configs/scenarios/perception_delay_sweep.yaml --dry-run

  # Small sweep: 1 goal, 1 trial
  python3 run_scenario_sweep.py ../configs/scenarios/perception_delay_sweep.yaml \
      --goals "goal_007" --trials 1

  # Full sweep: 3 goals, 2 trials
  python3 run_scenario_sweep.py ../configs/scenarios/perception_delay_sweep.yaml \
      --goals "goal_007,goal_011,goal_021" --trials 2

  # With longer timeout for scenarios that may cause long stops
  python3 run_scenario_sweep.py ../configs/scenarios/static_obstacle.yaml \
      --goals "goal_007" --stuck-timeout 180
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from typing import Optional

# Add lib to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

from config import (
    ExperimentConfig, load_goals, DATA_DIR, SCRIPTS_DIR, CONFIG_DIR,
)
from scenarios import load_scenario


def build_experiment_matrix(scenario_yaml: str, goal_ids: list, trials: int):
    """Build the full experiment matrix from scenario + goals + trials.

    Returns list of dicts with keys: scenario_config, goal_id, trial, params, condition
    """
    scenario = load_scenario(scenario_yaml)
    configs = scenario.expand_sweep()

    matrix = []
    for config in configs:
        # Build a condition string from params
        param_str = '_'.join(f'{k}{v}' for k, v in sorted(config.params.items()))
        condition = f'{config.scenario_type.value}_{param_str}' if param_str else config.scenario_type.value

        for goal_id in goal_ids:
            for trial in range(1, trials + 1):
                matrix.append({
                    'scenario_type': config.scenario_type.value,
                    'params': dict(config.params),
                    'goal_id': goal_id,
                    'trial': trial,
                    'condition': condition,
                    'description': config.description,
                })

    return matrix


def print_matrix(matrix: list):
    """Pretty-print the experiment matrix."""
    print(f"\nExperiment Matrix: {len(matrix)} experiments")
    print(f"{'#':<4} {'Condition':<45} {'Goal':<12} {'Trial':<6}")
    print('-' * 70)
    for i, exp in enumerate(matrix, 1):
        print(f"{i:<4} {exp['condition']:<45} {exp['goal_id']:<12} {exp['trial']:<6}")
    print()


def start_interceptor(strategy: str, params: dict,
                       fault_strategy: Optional[str] = None,
                       fault_params: Optional[dict] = None) -> subprocess.Popen:
    """Launch the perception interceptor as a subprocess."""
    interceptor_script = os.path.join(
        os.path.dirname(__file__), '..', 'lib', 'perception_interceptor.py'
    )

    cmd = [
        'python3', interceptor_script,
        '--strategy', strategy,
        '--params', json.dumps(params),
    ]

    if fault_strategy:
        cmd += ['--fault-strategy', fault_strategy,
                '--fault-params', json.dumps(fault_params or {})]

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


def run_single_experiment(exp: dict, goals_map: dict, stuck_timeout: float,
                          campaign: str = 'default',
                          fault_strategy: Optional[str] = None,
                          fault_params: Optional[dict] = None) -> dict:
    """Run a single experiment with interceptor lifecycle.

    Returns result dict.
    """
    goal_config = goals_map[exp['goal_id']]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    exp_id = f"{exp['goal_id']}_{exp['condition']}_t{exp['trial']}_{timestamp}"

    config = ExperimentConfig(
        experiment_id=exp_id,
        goal=goal_config,
        stuck_timeout=stuck_timeout,
        condition=exp['condition'],
        scenario_type=exp['scenario_type'],
        scenario_params=exp['params'],
        campaign=campaign,
    )

    # Start interceptor
    interceptor_proc = None
    try:
        fault_str = f" + fault:{fault_strategy}" if fault_strategy else ""
        print(f"\nStarting interceptor: {exp['scenario_type']} {exp['params']}{fault_str}")
        interceptor_proc = start_interceptor(exp['scenario_type'], exp['params'],
                                              fault_strategy, fault_params)

        # Run the experiment using the existing runner
        # Import here to avoid ROS2 init conflicts
        from run_experiments import ExperimentRunner, reset_vehicle

        runner = ExperimentRunner(config)
        success = runner.run()
        runner.compute_and_save_metrics()

        # Record obstacle placement info in the sweep result for traceability
        obstacle_info = {}
        try:
            obstacle_info_path = '/tmp/rise_obstacle_info.json'
            if os.path.exists(obstacle_info_path):
                with open(obstacle_info_path) as _f:
                    obstacle_info = json.load(_f)
        except Exception:
            pass

        return {
            'experiment_id': exp_id,
            'goal_id': exp['goal_id'],
            'condition': exp['condition'],
            'scenario_type': exp['scenario_type'],
            'params': exp['params'],
            'trial': exp['trial'],
            'success': success,
            'result': runner.result,
            'obstacle_info': obstacle_info,
        }

    except Exception as e:
        print(f"ERROR in experiment {exp_id}: {e}")
        return {
            'experiment_id': exp_id,
            'goal_id': exp['goal_id'],
            'condition': exp['condition'],
            'scenario_type': exp['scenario_type'],
            'params': exp['params'],
            'trial': exp['trial'],
            'success': False,
            'result': {'status': 'error', 'error': str(e)},
        }

    finally:
        stop_interceptor(interceptor_proc)


def generate_sweep_summary(results: list, scenario_yaml: str) -> str:
    """Generate a text summary of sweep results."""
    lines = []
    lines.append('=' * 80)
    lines.append(f'SCENARIO SWEEP SUMMARY')
    lines.append(f'Scenario: {os.path.basename(scenario_yaml)}')
    lines.append(f'Timestamp: {datetime.now().isoformat()}')
    lines.append('=' * 80)

    total = len(results)
    successes = sum(1 for r in results if r['success'])
    lines.append(f'\nTotal experiments: {total}')
    lines.append(f'Successful: {successes} ({100*successes/total:.0f}%)')
    lines.append(f'Failed: {total - successes}')

    # Group by condition for degradation analysis
    by_condition = {}
    for r in results:
        cond = r['condition']
        if cond not in by_condition:
            by_condition[cond] = []
        by_condition[cond].append(r)

    lines.append(f'\n{"Condition":<45} {"Success":<8} {"MRM":<6} {"Time":<8}')
    lines.append('-' * 70)

    for cond in sorted(by_condition.keys()):
        runs = by_condition[cond]
        succ = sum(1 for r in runs if r['success'])
        mrms = [r['result'].get('mrm_trigger_count', 0) for r in runs]
        times = [r['result'].get('driving_time', 0) for r in runs]
        avg_mrm = sum(mrms) / len(mrms) if mrms else 0
        avg_time = sum(times) / len(times) if times else 0

        lines.append(
            f'{cond:<45} {succ}/{len(runs):<6} {avg_mrm:<6.1f} {avg_time:<8.0f}s'
        )

    lines.append('')
    lines.append('=' * 80)
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='Run scenario parameter sweep')
    parser.add_argument('scenario_yaml', type=str,
                        help='Path to scenario YAML file')
    parser.add_argument('--goals', type=str, default='goal_007',
                        help='Comma-separated goal IDs (default: goal_007)')
    parser.add_argument('--trials', type=int, default=1,
                        help='Number of trials per configuration (default: 1)')
    parser.add_argument('--stuck-timeout', type=float, default=150.0,
                        help='Stuck detection timeout in seconds')
    parser.add_argument('--dry-run', action='store_true',
                        help='Show experiment matrix without running')
    parser.add_argument('--skip-reset', action='store_true',
                        help='Skip vehicle reset between experiments')
    parser.add_argument('--campaign', type=str, default=None,
                        help='Campaign name (default: scenario name)')
    parser.add_argument('--fault-strategy', type=str, default=None,
                        choices=['perception_dropout', 'position_noise'],
                        help='Fault overlay applied on top of scenario (for fault+scenario experiments)')
    parser.add_argument('--fault-params', type=str, default='{}',
                        help='JSON string of fault overlay parameters, e.g. \'{"dropout_rate":0.3}\'')
    args = parser.parse_args()

    # Resolve scenario path
    scenario_yaml = args.scenario_yaml
    if not os.path.isabs(scenario_yaml):
        # Try relative to configs/scenarios
        candidate = os.path.join(CONFIG_DIR, 'scenarios', scenario_yaml)
        if os.path.exists(candidate):
            scenario_yaml = candidate
        elif not os.path.exists(scenario_yaml):
            print(f"ERROR: Scenario file not found: {scenario_yaml}")
            return 1

    goal_ids = [g.strip() for g in args.goals.split(',')]

    fault_strategy = args.fault_strategy
    fault_params = json.loads(args.fault_params)

    # Resolve campaign name (default: scenario file stem, with fault suffix if applicable)
    scenario_name = os.path.splitext(os.path.basename(scenario_yaml))[0]
    if args.campaign:
        campaign = args.campaign
    elif fault_strategy:
        campaign = f"{scenario_name}_{fault_strategy}"
    else:
        campaign = scenario_name

    # Build experiment matrix
    matrix = build_experiment_matrix(scenario_yaml, goal_ids, args.trials)

    print_matrix(matrix)

    if args.dry_run:
        print("DRY RUN - not executing")
        return 0

    # Load goal configs
    try:
        all_goals = load_goals()
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        return 1

    goals_map = {g.id: g for g in all_goals}
    missing = [gid for gid in goal_ids if gid not in goals_map]
    if missing:
        print(f"ERROR: Goals not found: {missing}")
        print(f"Available: {', '.join(g.id for g in all_goals)}")
        return 1

    input(f"\nPress Enter to start {len(matrix)} experiments (Ctrl+C to cancel)...")

    # Run experiments
    results = []
    for i, exp in enumerate(matrix, 1):
        print(f"\n{'#'*60}")
        print(f"# SWEEP EXPERIMENT {i}/{len(matrix)}")
        print(f"# Condition: {exp['condition']}")
        print(f"# Goal: {exp['goal_id']}, Trial: {exp['trial']}")
        print(f"{'#'*60}")

        result = run_single_experiment(exp, goals_map, args.stuck_timeout, campaign,
                                        fault_strategy, fault_params)
        results.append(result)

        # Reset between experiments
        if not args.skip_reset and i < len(matrix):
            print("\nResetting vehicle...")
            try:
                from run_experiments import reset_vehicle
                if not reset_vehicle():
                    print("WARNING: Reset failed, waiting 10s...")
                    time.sleep(10)
                else:
                    time.sleep(5)
            except Exception as e:
                print(f"WARNING: Reset error: {e}")
                time.sleep(10)

    # Generate and print summary
    summary = generate_sweep_summary(results, scenario_yaml)
    print(summary)

    # Save results
    campaign_dir = os.path.join(DATA_DIR, campaign)
    os.makedirs(campaign_dir, exist_ok=True)
    sweep_file = os.path.join(
        campaign_dir,
        f'sweep_{scenario_name}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
    )
    with open(sweep_file, 'w') as f:
        json.dump({
            'scenario_yaml': scenario_yaml,
            'scenario_name': scenario_name,
            'timestamp': datetime.now().isoformat(),
            'goals': goal_ids,
            'trials': args.trials,
            'total_experiments': len(results),
            'successful': sum(1 for r in results if r['success']),
            'results': results,
        }, f, indent=2)

    print(f"\nSweep results saved to: {sweep_file}")

    # Save text summary
    summary_file = sweep_file.replace('.json', '_summary.txt')
    with open(summary_file, 'w') as f:
        f.write(summary)
    print(f"Summary saved to: {summary_file}")

    return 0


if __name__ == '__main__':
    try:
        exit_code = main()
    except KeyboardInterrupt:
        print("\n\nSweep interrupted by user")
        exit_code = 1
    finally:
        try:
            from ros_utils import shutdown_ros
            shutdown_ros()
        except Exception:
            pass
    sys.exit(exit_code)
