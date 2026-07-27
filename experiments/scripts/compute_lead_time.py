#!/usr/bin/env python3
"""
Compute lead time for IMU fault trials collected under Arm B: seconds between
fault onset and Autoware's own MRM intervention.

Why not just use the raw MRM state topic / mrm_trigger_count: Arm B's
restored localization/perception diagnostics chatter constantly even in
near-nominal driving (confirmed 2026-07-25, goal_012 imu_fault_s1 control
condition: 92 NORMAL->MRM_OPERATING transitions in one 280s trial, EVERY one
recovering within 0.05-0.24s). Raw trigger count or "time of first trigger"
is dominated by that noise floor, not the fault. What's actually
distinguishable is PERSISTENCE: a transition that does NOT recover within
--min-sustain-s (default 1.0, well above the observed ~0.25s noise ceiling)
is a real, terminal intervention, not a blip.

Lead time = (time of the first sustained MRM_OPERATING transition) -
            (start time of the fault-activation window immediately preceding
            it, from fault_injector.py's own fault_log.jsonl via
            extract_fault_windows).

Trials with no sustained transition at all (goal reached, or only transient
blips) are reported as "no intervention" and excluded from lead-time stats —
not an error, just not a case where a lead-time number is definable yet.

Usage (must source ROS/Autoware, then the repo venv):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/compute_lead_time.py --campaign imu_fault_ramp_armB
  python3 experiments/scripts/compute_lead_time.py --campaign imu_fault_ramp_armB \
      --campaign imu_fault_scale_armB --campaign imu_fault_stuck_armB
"""

import argparse
import glob
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from metrics import MetricsCollector  # noqa: E402
from compare_fault_vs_nominal import read_gt_and_tl  # noqa: E402
from plot_fault_impact import fault_kind_and_windows  # noqa: E402

DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data')

MRM_NORMAL = 1
MRM_OPERATING = 2


def find_sustained_mrm(mrm_states, min_sustain_s=1.0):
    """First NORMAL->OPERATING (or worse) transition that does not return to
    NORMAL within min_sustain_s — a real, terminal intervention rather than
    one of the constant sub-quarter-second Arm B chatter blips. Returns the
    transition time, or None if every excursion in this trial recovered
    quickly (including the trivial case of zero excursions)."""
    prev_state = MRM_NORMAL
    for i, (t, state, _behavior) in enumerate(mrm_states):
        if prev_state == MRM_NORMAL and state != MRM_NORMAL:
            # Walk forward until state returns to NORMAL or the bag ends.
            j = i
            while j < len(mrm_states) and mrm_states[j][1] != MRM_NORMAL:
                j += 1
            recovered_at = mrm_states[j][0] if j < len(mrm_states) else None
            sustained = (recovered_at is None) or (recovered_at - t >= min_sustain_s)
            if sustained:
                return t
        prev_state = state
    return None


def find_trial_dirs(campaign, goal='*', trial=None):
    pattern = f't{trial}_*' if trial else 't*'
    return sorted(glob.glob(os.path.join(DATA_DIR, campaign, goal, pattern)))


def analyze_trial(campaign, trial_dir, min_sustain_s):
    goal_id = os.path.basename(os.path.dirname(trial_dir))
    trial_name = os.path.basename(trial_dir)
    bag_dir = os.path.join(trial_dir, 'rosbag')
    if not os.path.isdir(bag_dir):
        return None

    result = json.load(open(os.path.join(trial_dir, 'result.json')))
    mc = MetricsCollector(bag_dir)
    mc.read_bag()
    gt_tl = read_gt_and_tl(bag_dir)
    kind, windows = fault_kind_and_windows(trial_dir, gt_tl['bag_start_abs_sec'], gt_tl['bag_duration'])

    if kind != 'imu' or not windows:
        return {'campaign': campaign, 'goal': goal_id, 'trial': trial_name,
                'status': result.get('status'), 'outcome': 'no_imu_fault_windows',
                'lead_time_s': None}

    sustained_t = find_sustained_mrm(mc.mrm_states, min_sustain_s)
    if sustained_t is None:
        return {'campaign': campaign, 'goal': goal_id, 'trial': trial_name,
                'status': result.get('status'), 'outcome': 'no_sustained_mrm',
                'lead_time_s': None}

    # Fault-activation window immediately preceding the sustained MRM event —
    # the one causally responsible, not necessarily the first window in the
    # trial (periodic faults re-cycle every ~50s; ramp only has one).
    preceding = [w for w in windows if w['start'] <= sustained_t]
    if not preceding:
        return {'campaign': campaign, 'goal': goal_id, 'trial': trial_name,
                'status': result.get('status'), 'outcome': 'mrm_before_any_fault_window',
                'lead_time_s': None, 'sustained_mrm_t': sustained_t}

    onset = max(preceding, key=lambda w: w['start'])
    lead_time = sustained_t - onset['start']
    return {
        'campaign': campaign, 'goal': goal_id, 'trial': trial_name,
        'status': result.get('status'), 'outcome': 'measured',
        'fault_onset_t': onset['start'], 'fault_onset_reason': onset.get('reason'),
        'sustained_mrm_t': sustained_t, 'lead_time_s': lead_time,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', action='append', required=True)
    ap.add_argument('--goal', default='*')
    ap.add_argument('--min-sustain-s', type=float, default=1.0,
                     help='Minimum non-recovery duration to count as a real intervention, '
                          'not Arm B chatter (default: 1.0s, ~4-20x the observed noise ceiling).')
    args = ap.parse_args()

    rows = []
    for campaign in args.campaign:
        for trial_dir in find_trial_dirs(campaign, args.goal):
            r = analyze_trial(campaign, trial_dir, args.min_sustain_s)
            if r:
                rows.append(r)

    print(f'{"campaign":22s} {"goal":10s} {"trial":22s} {"status":20s} {"outcome":26s} {"lead_time_s":>11s}')
    for r in rows:
        lt = f'{r["lead_time_s"]:.2f}' if r['lead_time_s'] is not None else '-'
        print(f'{r["campaign"]:22s} {r["goal"]:10s} {r["trial"]:22s} {r["status"]:20s} {r["outcome"]:26s} {lt:>11s}')

    measured = [r['lead_time_s'] for r in rows if r['outcome'] == 'measured']
    print(f'\n{len(measured)}/{len(rows)} trials produced a measurable lead time '
          f'(min-sustain-s={args.min_sustain_s}).')
    if measured:
        measured.sort()
        n = len(measured)
        mean = sum(measured) / n
        median = measured[n // 2] if n % 2 else (measured[n // 2 - 1] + measured[n // 2]) / 2
        print(f'  lead_time_s: min={measured[0]:.2f}  median={median:.2f}  '
              f'mean={mean:.2f}  max={measured[-1]:.2f}')
        print(f'  all values: {[round(v, 2) for v in measured]}')

    by_outcome = {}
    for r in rows:
        by_outcome.setdefault(r['outcome'], 0)
        by_outcome[r['outcome']] += 1
    print(f'\nOutcome breakdown: {by_outcome}')


if __name__ == '__main__':
    main()
