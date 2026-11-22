#!/usr/bin/env python3
"""
Data-collection runner. Normally called through ./collect.sh.

Campaigns are defined in experiments/configs/campaigns.yaml. Each campaign
has a trial target per goal; the runner counts the valid trials already on
disk and runs only what is missing, at most --max-trials per invocation
(behavior_path_planner degrades after ~18-36 trials on one Autoware process,
so restart Autoware between invocations). Re-running the same command
continues where the last one stopped.

A trial is valid when it recorded a bag, the vehicle started the route (and,
in a nominal campaign, reached the goal), and (for a fault campaign) the
fault armed. Before each trial the runner watches the MRM state for 20 s
while parked and stops if this Autoware launch is flapping (see
HEALTH_WINDOW_S). An invalid trial is moved to
data/<campaign>/_invalid/ so its slot is retried; by default the invocation
stops there too, since an invalid trial usually means Autoware needs a
restart.

Usage:
  run_experiments.py --status [CAMPAIGN ...]
  run_experiments.py CAMPAIGN [CAMPAIGN ...] [--max-trials N] [--goals g1,g2]
                     [--trials N] [--arm A|B] [--on-invalid stop|continue]
                     [--dry-run] [--yes]
  CAMPAIGN may be 'all' (every campaign, in file order).
"""

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from collections import OrderedDict

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib'))

from config import (  # noqa: E402
    BAG_STORAGE, CONFIG_DIR, DATA_DIR, LIB_DIR, SCRIPTS_DIR,
    TrialConfig, campaign_meta_dir, load_campaigns, load_goals, recording_topics,
)

DEFAULT_MAX_TRIALS = 12
INVALID_DIR = '_invalid'
# Hard cap on one trial. The longest routes (goal_012, goal_026, ~1.15 km)
# take ~310 s in nominal driving; a fault that slows the vehicle needs more.
TRIAL_TIMEOUT_S = 900.0
# Statuses where the vehicle never started the route: never a fault effect.
NOT_STARTED = ('setup_failed', 'recording_failed', 'engage_failed')
# Session health check before each trial: some Autoware launches come up
# with the MRM state flapping NORMAL -> MRM_OPERATING for ~0.1 s every
# 2-4 s, parked or driving (nominal 2026-11-22: 2 of 7 launches, 33-108
# triggers per trial). Driving is unaffected, but every MRM-based measure
# is meaningless, so that launch is not used.
HEALTH_WINDOW_S = 20.0
HEALTH_MAX_TRIGGERS = 1


# ── Bookkeeping (no ROS needed) ──────────────────────────────────────────────

def valid_trials(campaign: str, goal_id: str) -> list:
    """Trial directories with a result.json that is marked valid."""
    goal_dir = os.path.join(DATA_DIR, campaign, goal_id)
    if not os.path.isdir(goal_dir):
        return []
    out = []
    for entry in sorted(os.listdir(goal_dir)):
        rf = os.path.join(goal_dir, entry, 'result.json')
        try:
            with open(rf) as f:
                if json.load(f).get('valid', False):
                    out.append(entry)
        except (OSError, json.JSONDecodeError):
            continue
    return out


def leftover_dirs(campaign: str, goal_id: str) -> list:
    """Trial directories without a valid result.json (crashed mid-trial)."""
    goal_dir = os.path.join(DATA_DIR, campaign, goal_id)
    if not os.path.isdir(goal_dir):
        return []
    done = set(valid_trials(campaign, goal_id))
    return [os.path.join(goal_dir, e) for e in sorted(os.listdir(goal_dir)) if e not in done]


def next_trial_num(campaign: str, goal_id: str) -> int:
    """1 + the highest t<N> used for this goal, valid or in _invalid/, so a
    re-run slot never reuses a number."""
    names = []
    goal_dir = os.path.join(DATA_DIR, campaign, goal_id)
    if os.path.isdir(goal_dir):
        names += os.listdir(goal_dir)
    inv_dir = os.path.join(DATA_DIR, campaign, INVALID_DIR)
    if os.path.isdir(inv_dir):
        names += [e[len(goal_id) + 1:] for e in os.listdir(inv_dir) if e.startswith(goal_id + '_')]
    nums = [int(n[1:].split('_')[0]) for n in names if n.startswith('t') and n[1:].split('_')[0].isdigit()]
    return max(nums, default=0) + 1


def build_plan(campaigns, goals_filter=None, trials_override=None):
    """Remaining (campaign, goal_id, trial_num), campaign by campaign; inside
    a campaign trial 1 on every goal, then trial 2, ... so an interrupted
    run leaves the goals balanced."""
    plan = []
    for c in campaigns:
        goals = [g for g in c.goals if not goals_filter or g in goals_filter]
        target = trials_override or c.trials
        have = {g: len(valid_trials(c.name, g)) for g in goals}
        for k in range(1, target + 1):
            plan += [(c, g, k) for g in goals if k > have[g]]
    return plan


def print_status(campaigns, goals_filter=None, trials_override=None):
    total_have = total_target = 0
    print(f"{'campaign':<16} {'valid/target':>13}  {'invalid':>7}  per goal (valid)")
    for c in campaigns:
        goals = [g for g in c.goals if not goals_filter or g in goals_filter]
        target = trials_override or c.trials
        have = {g: min(len(valid_trials(c.name, g)), target) for g in goals}
        inv_dir = os.path.join(DATA_DIR, c.name, INVALID_DIR)
        n_inv = len(os.listdir(inv_dir)) if os.path.isdir(inv_dir) else 0
        h, t = sum(have.values()), target * len(goals)
        total_have += h
        total_target += t
        short = ' '.join(f"{g.replace('goal_', '')}:{n}" for g, n in have.items())
        if len(short) > 90:
            short = f'{sum(n == target for n in have.values())}/{len(goals)} goals complete'
        print(f'{c.name:<16} {h:>6}/{t:<6}  {n_inv:>7}  {short}')
        for g in goals:
            for d in leftover_dirs(c.name, g):
                print(f'    leftover (no valid result.json, delete it): {d}')
    print(f"{'total':<16} {total_have:>6}/{total_target:<6}")


# ── Processes ────────────────────────────────────────────────────────────────

def _start(script, args, name, log_path):
    """Output to a log file: an unread PIPE fills up and blocks the process
    (the injector logs every fault message)."""
    with open(log_path, 'a') as log:
        proc = subprocess.Popen(['python3', os.path.join(LIB_DIR, script)] + args,
                                stdout=log, stderr=subprocess.STDOUT)
    time.sleep(2.0)
    if proc.poll() is not None:
        with open(log_path) as f:
            tail = f.read()[-2000:]
        raise RuntimeError(f'{name} failed to start, see {log_path}:\n{tail}')
    print(f'{name} started (PID {proc.pid}), log: {log_path}')
    return proc


def _stop(proc, name):
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
    print(f'{name} stopped')


def start_fault_injector(campaign, log_file, arm):
    """Always runs: it is also the relay that feeds imu_data_faulted
    (gyro_odometer) and traffic_signals_faulted (behavior planning).
    Without a fault it passes everything through."""
    f = campaign.fault
    args = ['--log-file', log_file, '--arm', arm]
    if f:   # zones only matter when there is a fault to gate
        args += ['--zone-goals', ','.join(campaign.goals)]
    if f.get('tl_fault'):
        args += ['--tl-fault', f['tl_fault'], '--tl-params', json.dumps(f.get('tl_params', {}))]
    if f.get('imu_fault'):
        args += ['--imu-fault', f['imu_fault'], '--imu-params', json.dumps(f.get('imu_params', {}))]
    if f.get('duration_s'):
        args += ['--fault-duration', str(f['duration_s'])]
    return _start('fault_injector.py', args, 'Fault injector',
                  os.path.join(os.path.dirname(log_file), 'fault_injector.log'))


def start_object_relay(meta_dir):
    """Copies perception objects -> objects_filtered (what planning reads)."""
    return _start('object_relay.py', [], 'Object relay', os.path.join(meta_dir, 'object_relay.log'))


def write_trial_fault_log(campaign_log, trial_log, t0, t1):
    """The injector's startup line plus this trial's events."""
    if not os.path.exists(campaign_log):
        return
    with open(campaign_log) as f, open(trial_log, 'w') as out:
        for i, line in enumerate(f):
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                continue
            if i == 0 or t0 <= e.get('wall_time', 0.0) <= t1:
                out.write(line)


# ── One trial ────────────────────────────────────────────────────────────────

class TrialRunner:
    def __init__(self, config: TrialConfig):
        self.config = config
        self.rosbag_process = None
        self.result = {'status': 'unknown', 'goal_reached': False,
                       'driving_time': 0.0, 'mrm_trigger_count': 0}

    def setup(self, max_init_wait=60.0) -> bool:
        print(f"\n{'=' * 60}\nTrial: {self.config.experiment_id}\n{'=' * 60}")
        self.config.create_directories()
        state, state_name = R.get_autoware_state()
        if state is None:
            print('ERROR: cannot get Autoware state')
            return False
        print(f'Autoware state: {state_name}')
        if state == R.AutowareState.INITIALIZING:
            print(f'Autoware is initializing, waiting up to {max_init_wait:.0f}s...')
            t0 = time.time()
            while time.time() - t0 < max_init_wait:
                time.sleep(2)
                state, state_name = R.get_autoware_state()
                if state is not None and state != R.AutowareState.INITIALIZING:
                    break
            else:
                print('ERROR: Autoware still initializing')
                return False
        if state in (R.AutowareState.DRIVING, R.AutowareState.PLANNING):
            print('WARNING: vehicle already driving/planning, clearing route...')
            R.clear_route()
            time.sleep(3)
        self.config.save_metadata()
        return True

    def start_recording(self) -> bool:
        topics = recording_topics(self.config.campaign.record)
        self.rosbag_process = R.start_rosbag_recording(
            self.config.rosbag_dir, topics, self.config.recorder_log_file, storage=BAG_STORAGE)
        if self.rosbag_process is None:
            print('ERROR: failed to start recording')
            return False
        print(f'Recording {len(topics)} topics to {self.config.rosbag_dir}')
        return True

    def stop_recording(self):
        if self.rosbag_process:
            print('Stopping recording...')
            R.stop_rosbag_recording(self.rosbag_process)
            self.rosbag_process = None

    def _set_goal(self) -> bool:
        g = self.config.goal
        return R.set_goal(g.position['x'], g.position['y'], g.position.get('z', 0.0),
                          g.orientation.get('z', 0.0), g.orientation.get('w', 1.0), timeout=60.0)

    def set_goal_and_engage(self) -> bool:
        g = self.config.goal
        print(f"Setting goal: ({g.position['x']:.2f}, {g.position['y']:.2f})")
        if not self._set_goal():
            print('ERROR: failed to set goal')
            return False

        mrm_state, _ = R.get_mrm_state()
        if mrm_state is not None and mrm_state != R.MrmState.NORMAL:
            print(f'WARNING: MRM state {mrm_state.name}, waiting for NORMAL...')
            t0 = time.time()
            while time.time() - t0 < 30:
                mrm_state, _ = R.get_mrm_state()
                if mrm_state == R.MrmState.NORMAL:
                    break
                time.sleep(1)
            else:
                print(f'ERROR: MRM state still {mrm_state} after 30s')
                return False

        # Engage only once planning publishes a trajectory: engaging earlier
        # trips the control gate timeout -> MRM. No trajectory in 60 s means a
        # contaminated post-MRM planner; one route re-set usually clears it.
        print('Waiting for planning trajectory before engage...')
        if not R.wait_for_trajectory(timeout=60.0):
            print('No trajectory after 60s, re-setting the route...')
            R.clear_route()
            time.sleep(20.0)
            if not self._set_goal() or not R.wait_for_trajectory(timeout=90.0):
                print('ERROR: no trajectory after route re-set (engage_failed)')
                return False

        print('Engaging autonomous mode...')
        R.engage_autonomous()
        time.sleep(5)
        # A transient MRM_SUCCEEDED right after engage is the control gate
        # timeout before the first command; it clears by itself.
        mrm_state, _ = R.get_mrm_state()
        if mrm_state not in (None, R.MrmState.NORMAL):
            if mrm_state == R.MrmState.MRM_FAILED:
                print('ERROR: MRM_FAILED after engage')
                return False
            print(f'  Transient MRM ({mrm_state.name}) after engage, waiting...')
            if not R.wait_for_mrm_recovery(timeout=20.0):
                print('ERROR: MRM did not recover within 20s after engage')
                return False
        return True

    def wait_for_driving(self, timeout=30.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            state, _ = R.get_autoware_state()
            if state in (R.AutowareState.DRIVING, R.AutowareState.ARRIVED_GOAL):
                return True
            time.sleep(0.5)
        print('WARNING: timeout waiting for DRIVING')
        return False

    def monitor(self) -> dict:
        timeout = self.config.stuck_timeout
        print(f'\nMonitoring (stuck timeout {timeout:.0f}s)...')
        t0 = last_moving = time.time()
        # Counted in the subscription callback: MRM flaps NORMAL<->OPERATING
        # in ~100 ms blips that a 1 Hz poll misses.
        R.reset_mrm_trigger_count()
        still = 0
        while True:
            elapsed = time.time() - t0
            state, state_name = R.get_autoware_state()
            if state == R.AutowareState.ARRIVED_GOAL:
                print(f'\nGoal reached in {elapsed:.1f}s')
                return {'status': 'goal_reached', 'goal_reached': True, 'driving_time': elapsed,
                        'mrm_trigger_count': R.get_mrm_trigger_count()}
            mrm_state, mrm_behavior = R.get_mrm_state()
            if mrm_state in (R.MrmState.MRM_SUCCEEDED, R.MrmState.MRM_FAILED):
                still += 1
                if still >= 30:
                    b = mrm_behavior.name if mrm_behavior else 'UNKNOWN'
                    print(f'\nMRM completed ({mrm_state.name}, {b}), stopped {still}s')
                    return {'status': f'mrm_{mrm_state.name.lower()}', 'goal_reached': False,
                            'driving_time': elapsed, 'mrm_trigger_count': R.get_mrm_trigger_count(),
                            'mrm_final_state': mrm_state.name, 'mrm_final_behavior': b}
            v = R.get_velocity()
            if v is not None:
                if abs(v) > 0.1:
                    last_moving, still = time.time(), 0
                else:
                    still += 1
            if time.time() - last_moving > timeout:
                print(f'\nStuck: no movement for {time.time() - last_moving:.0f}s')
                return {'status': 'stuck', 'goal_reached': False, 'driving_time': elapsed,
                        'mrm_trigger_count': R.get_mrm_trigger_count(), 'final_velocity': v}
            if elapsed > TRIAL_TIMEOUT_S:
                print(f'\nSafety timeout ({elapsed:.0f}s)')
                return {'status': 'timeout', 'goal_reached': False, 'driving_time': elapsed,
                        'mrm_trigger_count': R.get_mrm_trigger_count()}
            n = R.get_mrm_trigger_count()
            print(f"\r[{elapsed:.0f}s] {state_name}{f' MRM:{n}' if n else ''}"
                  f"{f' v={v:.1f}' if v is not None else ''}{f' (stopped {still}s)' if still > 5 else ''}      ",
                  end='', flush=True)
            time.sleep(1.0)

    def run(self):
        try:
            if not self.setup():
                self.result['status'] = 'setup_failed'
                return
            if not self.start_recording():
                self.result['status'] = 'recording_failed'
                return
            R.reset_mrm_trigger_count()
            if not self.set_goal_and_engage() or not self.wait_for_driving():
                self.result['status'] = 'engage_failed'
                return
            if self.config.stabilization_delay > 0:
                time.sleep(self.config.stabilization_delay)
            # Before the vehicle has moved, no fault can be the cause; 1 is
            # the usual post-engage transient, more means a flapping launch.
            pre = R.get_mrm_trigger_count()
            self.result = self.monitor()
            self.result['pre_drive_mrm_triggers'] = pre
        except KeyboardInterrupt:
            print('\nInterrupted')
            self.result['status'] = 'interrupted'
            raise
        finally:
            self.stop_recording()
            print('\nCleaning up...')
            R.clear_route()
            mrm_state, _ = R.get_mrm_state()
            if mrm_state is not None and mrm_state != R.MrmState.NORMAL:
                R.recover_from_emergency(max_attempts=2)

    def validate(self, start_wall_time) -> list:
        """Issues that make this trial unusable (empty list = valid)."""
        c = self.config
        if self.result.get('status') == 'interrupted':
            return ['interrupted']
        if not os.path.exists(os.path.join(c.rosbag_dir, 'metadata.yaml')):
            return [f"no complete rosbag (status={self.result.get('status')!r}): "
                    f'failed before or during recording; see {c.recorder_log_file}']
        print(f'\nComputing metrics from {c.rosbag_dir}')
        try:
            m = compute_metrics_from_bag(
                c.rosbag_dir, goal_position=c.goal.position,
                fault_log_path=c.fault_log_file,
                tl_fault=c.campaign.fault.get('tl_fault'),
                imu_fault=c.campaign.fault.get('imu_fault'),
                trial_start_wall_time=start_wall_time)
        except Exception as e:
            return [f'metrics failed: {e}']
        save_metrics(m, c.metrics_file)
        r, fv = m.reliability, m.fault_validation
        print(f'  goal reached {r.goal_reached}, {r.driving_time:.0f}s, mean v {r.mean_velocity:.1f} m/s, '
              f'MRM triggers {m.fail_operational.mrm_trigger_count}')
        issues = []
        status = self.result.get('status')
        if status in NOT_STARTED:
            issues.append(f'{status}: the vehicle never started the route')
        elif not c.campaign.fault and status != 'goal_reached':
            issues.append(f'{status}: a nominal trial must reach the goal')
        if (fv.tl_fault_configured or fv.imu_fault_configured) and not fv.valid:
            issues += fv.issues or ['fault never armed']
        dist_m = r.distance_km * 1000
        if not r.goal_reached and dist_m < 50 and r.driving_time > 0.5 * c.stuck_timeout:
            issues.append(f'vehicle covered only {dist_m:.0f} m in {r.driving_time:.0f}s: Autoware '
                          f'stuck/unresponsive, not a fault effect')
        return issues


# ── Vehicle reset between trials ─────────────────────────────────────────────

_NEAR_START_M = 25.0


def _start_xy():
    with open(os.path.join(CONFIG_DIR, 'baseline.json')) as f:
        pos = json.load(f)['egoConfiguration']['egoPosition']
    return pos['x'], pos['y']


def reset_vehicle(max_retries=2) -> bool:
    """Back to the start pose. Near the start (e.g. an early MRM) only the
    route is cleared; otherwise AWSIM teleports and localization re-inits."""
    mrm_state, _ = R.get_mrm_state()
    if mrm_state is not None and mrm_state != R.MrmState.NORMAL:
        R.clear_route()
        time.sleep(2)
        R.wait_for_mrm_recovery(timeout=10.0)
    pos = R.get_vehicle_position()
    if pos is not None and math.hypot(pos[0] - _start_xy()[0], pos[1] - _start_xy()[1]) <= _NEAR_START_M:
        print('  Near start, skipping teleport; waiting for clean diagnostics')
        if R.wait_for_clean_diagnostics(timeout=60.0, stable_duration=5.0):
            return True
    for attempt in range(max_retries):
        try:
            res = subprocess.run(['python3', os.path.join(SCRIPTS_DIR, 'reset_vehicle.py'), '--wait'], timeout=90)
            if res.returncode == 0:
                time.sleep(2)
                mrm_state, _ = R.get_mrm_state()
                if mrm_state != R.MrmState.NORMAL:
                    R.recover_from_emergency(max_attempts=1)
                if R.wait_for_clean_diagnostics(timeout=60.0, stable_duration=5.0):
                    return True
                print('  Diagnostics did not clear after reset')
        except subprocess.TimeoutExpired:
            print(f'WARNING: reset timed out (attempt {attempt + 1}/{max_retries})')
        time.sleep(5)
    return False


def mrm_flapping(window=HEALTH_WINDOW_S) -> int:
    """MRM triggers seen while parked for `window` seconds; more than
    HEALTH_MAX_TRIGGERS means this Autoware launch is in the flapping state."""
    R.reset_mrm_trigger_count()
    time.sleep(window)
    return R.get_mrm_trigger_count()


def reset_with_retry():
    if reset_vehicle():
        return
    print('WARNING: reset failed, waiting 20s for diagnostics to settle...')
    time.sleep(20)
    if not reset_vehicle():
        print('ERROR: second reset failed; system likely needs an Autoware restart')


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('campaigns', nargs='*', help="campaign names from campaigns.yaml, or 'all'")
    ap.add_argument('--status', action='store_true', help='show progress and exit (no ROS needed)')
    ap.add_argument('--max-trials', type=int, default=DEFAULT_MAX_TRIALS,
                    help=f'trials per invocation, i.e. per Autoware restart (default {DEFAULT_MAX_TRIALS})')
    ap.add_argument('--goals', type=str, default=None, help='restrict to these goals (comma-separated)')
    ap.add_argument('--trials', type=int, default=None, help="override the campaign's trial target")
    ap.add_argument('--arm', choices=['A', 'B'], default='A',
                    help='diagnostic-gate configuration Autoware runs under (label only; '
                         'switch with experiments/scripts/switch_diagnostic_arm.sh + restart)')
    ap.add_argument('--on-invalid', choices=['stop', 'continue'], default='stop')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--yes', '-y', action='store_true', help='no "Press Enter" prompt')
    args = ap.parse_args()

    all_campaigns = load_campaigns()
    names = args.campaigns or (list(all_campaigns) if args.status else [])
    if names == ['all']:
        names = list(all_campaigns)
    unknown = [n for n in names if n not in all_campaigns]
    if unknown or not names:
        print(f"Unknown or missing campaign: {unknown or '(none)'}\nAvailable: {' '.join(all_campaigns)} all")
        return 2
    campaigns = [all_campaigns[n] for n in names]
    goals_filter = set(g.strip() for g in args.goals.split(',')) if args.goals else None
    if args.arm == 'B':
        print('NOTE: Arm B data goes under <campaign>_armB/')
        for c in campaigns:
            c.name = f'{c.name}_armB'

    if args.status:
        print_status(campaigns, goals_filter, args.trials)
        return 0

    plan = build_plan(campaigns, goals_filter, args.trials)
    batch = plan[:args.max_trials]
    print('=' * 60)
    print(f'{len(plan)} trials remaining; this run: {len(batch)} (--max-trials {args.max_trials})')
    for (c, g, k) in batch:
        print(f'  {c.name:<16} {g}  trial {k}')
    if not batch:
        print('Nothing to do.')
        return 0
    if args.dry_run:
        print('DRY RUN, not executing')
        return 0
    if sys.stdin.isatty() and not args.yes:
        input('Press Enter to start (Ctrl+C to cancel)...')

    global R, compute_metrics_from_bag, save_metrics
    import ros_utils as R                                       # noqa: E402
    from metrics import compute_metrics_from_bag, save_metrics  # noqa: E402
    goals = {g.id: g for g in load_goals()}

    groups = OrderedDict()
    for item in batch:
        groups.setdefault(item[0].name, []).append(item)

    results, stopped = [], False
    try:
        for cname, items in groups.items():
            campaign = items[0][0]
            meta = campaign_meta_dir(cname)
            os.makedirs(meta, exist_ok=True)
            campaign_log = os.path.join(meta, 'fault_log.jsonl')
            print(f"\n{'#' * 60}\n# Campaign {cname}: {campaign.description}\n{'#' * 60}")
            # Relays first: the reset below needs IMU and TL data flowing
            relay = injector = None
            try:
                relay = start_object_relay(meta)
                injector = start_fault_injector(campaign, campaign_log, args.arm)
                print('\nInitial vehicle reset...')
                reset_with_retry()
                time.sleep(3)
                for i, (_, gid, _) in enumerate(items):
                    # Restrict the injector's zone matching to this trial's goal
                    R.set_fault_injector_goal(gid)
                    config = TrialConfig(campaign=campaign, goal=goals[gid], trial_num=next_trial_num(cname, gid), arm=args.arm)
                    n_mrm = mrm_flapping()
                    if n_mrm > HEALTH_MAX_TRIGGERS:
                        print('\n' + '!' * 70 + f'\n! {n_mrm} MRM triggers in {HEALTH_WINDOW_S:.0f}s '
                              f'while parked: this Autoware launch is in the MRM-flapping state.'
                              f'\n! No trial recorded. Restart Autoware, then re-run the same command.'
                              + '\n' + '!' * 70)
                        stopped = True
                        break
                    t0 = time.time()
                    runner = TrialRunner(config)
                    runner.run()
                    write_trial_fault_log(campaign_log, config.fault_log_file, t0, time.time())
                    issues = runner.validate(t0)
                    runner.result.update({'valid': not issues, 'issues': issues})
                    config.save_result(runner.result)
                    results.append((config, runner.result))
                    if issues:
                        inv = os.path.join(DATA_DIR, cname, INVALID_DIR)
                        os.makedirs(inv, exist_ok=True)
                        dest = os.path.join(inv, f'{gid}_{os.path.basename(config.data_dir)}')
                        shutil.move(config.data_dir, dest)
                        print('\n' + '!' * 70 + f'\n! INVALID trial, moved to {dest}')
                        for s in issues:
                            print(f'!   - {s}')
                        print('!' * 70)
                        if args.on_invalid == 'stop':
                            stopped = True
                            break
                    if i < len(items) - 1 or cname != list(groups)[-1]:
                        print('\nResetting vehicle...')
                        reset_with_retry()
                        time.sleep(5)
            finally:
                _stop(relay, 'Object relay')
                _stop(injector, 'Fault injector')
            if stopped:
                break
    except KeyboardInterrupt:
        print('\nStopped by user.')
        stopped = True

    print('\n' + '=' * 60 + '\nSUMMARY\n' + '=' * 60)
    for config, r in results:
        tag = 'valid  ' if r.get('valid') else 'INVALID'
        print(f"  {tag} {config.experiment_id}: {r.get('status')} "
              f"(MRM {r.get('mrm_trigger_count', 0)}, {r.get('driving_time', 0):.0f}s)")
    left = len(build_plan(campaigns, goals_filter, args.trials))
    print(f'\n{left} trials still to run for: {" ".join(names)}')
    if left:
        print('Restart Autoware (Terminal 2), then re-run the same command.')
    if stopped:
        print('Stopped early (invalid trial or Ctrl+C): check the messages above first.')
    return 1 if stopped else 0


if __name__ == '__main__':
    exit_code = 1
    try:
        exit_code = main()
    finally:
        if 'ros_utils' in sys.modules:
            try:
                sys.modules['ros_utils'].shutdown_ros()
            except Exception:
                pass
    # rclpy destructors throw on the normal exit path ("terminate called
    # without an active exception"); os._exit skips them, so flush first.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)
