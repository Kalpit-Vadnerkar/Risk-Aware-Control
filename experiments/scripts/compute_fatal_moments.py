#!/usr/bin/env python3
"""
Alternative fatal-moment anchors, computed offline from already-collected
data (no new AWSIM/Autoware runs) — a response to a real problem found in
this session's conformal-lead-time results: metrics.py's `permanent_stop_time_s`
(StaticCollisionMetrics) walks BACKWARD from the end of the recording to find
the LATEST point after which velocity never exceeds threshold again. That
answers "when did the vehicle finally, completely give up" — not "when did
things first go wrong." For trials with a long struggle period (repeated
stuck/inch-forward/replan cycles before finally settling), this inflates the
reported fatal moment far past the actual point of no return, which directly
inflates every lead-time number computed against it (conformal_lead_time.py's
lead_time_s = fatal_moment - alarm_crossing_time).

Looked at Autoware's own mrm_handler source
(autoware/src/universe/autoware_universe/system/autoware_mrm_handler/) for a
more principled anchor before inventing one: its `isEmergency()`/`isStopped()`
logic is REACTIVE (checks live diagnostic/operation-mode-availability health
each control cycle), not a retrospective "point of no return" judgment — there
is no numeric threshold there to borrow directly for an offline analysis. It's
also compromised as a ground truth in Arm A specifically, since perception/
planning/localization are deliberately stripped from the gate (CLAUDE.md) —
consistent with prior notes that MRM state flaps constantly even under nominal
Arm A driving. The one thing worth reusing: mrm_handler's own `isStopped()`
check uses th_stopped_velocity=0.001 m/s, much stricter than metrics.py's 0.3.

Two new candidate anchors, both computable from data already on disk:

1. first_stop_time_s: same "stopped for >= MIN_STOPPED_S" definition as
   metrics.py's existing heuristic, but scanning FORWARD for the FIRST
   qualifying window instead of backward from the end — catches the earliest
   genuine stop instead of the final one. Real trade-off: risks firing on a
   benign extended stop (e.g. heavy congestion) that happens to precede the
   actual permanent one. Combined with (2) below to reduce that risk.

2. lane_deviation_crossing_s: first SUSTAINED crossing of a lane-width-scale
   EKF-vs-ground-truth divergence threshold (reuses compare_fault_vs_nominal.py's
   ekf_gt_divergence — already computed there for the fault_impact/discriminability
   scripts, not re-derived here) — a physically-grounded "the vehicle is
   meaningfully off its intended path" moment, independent of whether it ever
   fully stops. Particularly suited to IMU faults, whose failure mode is
   divergence, not necessarily immobilization. "Sustained" (>= SUSTAIN_S
   continuously above threshold) to avoid firing on ordinary EKF noise spikes
   (confirmed present even in nominal driving in this session's fault_impact
   plots).

Combined candidate: earliest of the two, per trial, since either genuinely
represents "something went wrong" for a different reason (immobilization vs.
divergence).

Two things found investigating imu_fault_s1 (2026-08-04, the deliberate
negative control — supposed to be harmless) that changed this script:

1. fault_log.jsonl is written by ONE fault_injector node that stays alive
   across an entire campaign's goal_007->goal_012->goal_026 sequence
   (run_experiments.py starts it once, before the per-goal trial loop) — so
   each trial's own copy of fault_log.jsonl is a CUMULATIVE snapshot, not
   trimmed to that trial's own bag window. Naively taking "the earliest
   fault-onset event in the file" (as st_gat/residuals.py's
   _first_fault_onset_wall_time does) picks up a PREVIOUS goal's onset event
   for every trial except the campaign's first. Confirmed directly: goal_012's
   "generation 1" imu_bias cycles 1-3 all have negative bag-relative
   timestamps (they're goal_007's), and only "generation 2" is actually
   inside goal_012's own [0, bag_duration] window. extract_fault_windows()
   in compare_fault_vs_nominal.py already scopes correctly (filters by
   relative time, not "the earliest") — this script now does the same:
   _trial_fault_onset_rel_s() below finds the earliest onset whose
   bag-relative time falls in [0, bag_duration], ignoring earlier goals'
   leaked events.
2. A candidate stop/deviation BEFORE the fault has even armed can't be
   fault-caused by construction — found a real example (imu_fault_s1/
   goal_026: an 8.3s "stop" that occurs before imu_fault_activated at
   55.91s, clearly an engage/startup artifact, not a fault reaction).
   candidates are now gated to only count if they occur at or after this
   trial's own fault onset.

Autoware source check for a principled deviation threshold (2026-08-04, per
Kalpit's request to look at Arm-B-relevant thresholds without needing to
collect Arm B data): autoware_control_validator's own
max_distance_deviation=1.0m and yaw_deviation_error=1.0rad/warn=0.5rad
(autoware/src/universe/autoware_universe/control/autoware_control_validator/
config/control_validator.param.yaml) ARE already recorded in every bag's
/diagnostics topic (confirmed: `control_validator: control_validation_max_
distance_deviation` present, DiagnosticStatus per cycle) — no Arm B run
needed to read them. But checked the actual values in an imu_fault_s3 trial
and found level=OK throughout: control_validator/planning_validator check
SELF-CONSISTENCY (is the controller tracking the trajectory IT planned from
its own belief), not GROUND-TRUTH correctness — an IMU/localization fault
corrupts perception and planning together, so the whole stack stays
internally consistent even while diverging from reality. These validators
are structurally blind to exactly the fault class this dissertation studies
— which is itself a reinforcement of the map-grounded-independent-check
premise, not a dead end. Not directly reusable as OUR lane-deviation
threshold (different comparison), but 1.0m is a reasonable, non-arbitrary
calibration anchor for the sweep below instead of a guessed value.

Usage (must source ROS/Autoware, then the repo venv):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/compute_fatal_moments.py
  python3 experiments/scripts/compute_fatal_moments.py --sweep   # threshold sweep, no candidates.csv
"""

import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import pandas as pd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from metrics import MetricsCollector  # noqa: E402
from compare_fault_vs_nominal import read_gt_and_tl, ekf_gt_divergence, load_fault_log  # noqa: E402

DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data')
DEFAULT_OUTPUT = os.path.join(REPO_DIR, 'experiments', 'analysis', 'fatal_moments', 'candidates.csv')

FAULT_DATASETS = [
    'imu_fault_s1', 'imu_fault_s3', 'imu_fault_scale', 'imu_fault_stuck',
    'tl_fault_s2', 'tl_fault_s3', 'tl_fault_s4', 'tl_fault_ramp',
]

_FAULT_ONSET_EVENTS = {'tl_fault_start', 'imu_fault_activated'}
_ONSET_PAD_S = 2.0  # small slack for clock/event-ordering jitter around t=0

# Same stop definition as metrics.py's StaticCollisionMetrics, applied
# forward instead of backward.
STOP_SPEED_THRESHOLD_MPS = 0.3
MIN_STOPPED_S = 20.0

# Calibration anchor, not a direct threshold (see module docstring — Autoware's
# own control_validator checks a different comparison): its max_distance_deviation
# is 1.0m, so start the sweep around there rather than an arbitrary guess.
DEFAULT_LANE_DEVIATION_M = 1.0
SUSTAIN_S = 2.0                  # divergence must stay above threshold this long to count


def trial_fault_onset_rel_s(trial_dir: str, bag_start_abs_sec: float, bag_duration: float):
    """Earliest fault-onset event whose bag-relative time falls inside THIS
    trial's own [0, bag_duration] window — NOT "the earliest onset in the
    file", which picks up a previous goal's onset (fault_log.jsonl is a
    cumulative campaign-level log, see module docstring). None if no onset
    falls in this trial's own window (nominal trials; or a fault that never
    armed for this specific trial)."""
    events = load_fault_log(trial_dir)
    onsets_rel = [
        e['wall_time'] - bag_start_abs_sec for e in events
        if e.get('event') in _FAULT_ONSET_EVENTS and 'wall_time' in e
    ]
    in_window = [t for t in onsets_rel if -_ONSET_PAD_S <= t <= bag_duration]
    return min(in_window) if in_window else None


def _first_qualifying_window(series, predicate, min_duration, after_s=None):
    """First timestamp from which `predicate(value)` holds continuously for
    at least min_duration, optionally not considering any window starting
    before `after_s` (fault-onset gate — a candidate can't be fault-caused if
    it starts before the fault even armed, see module docstring)."""
    if not series:
        return None
    n = len(series)
    i = 0
    while i < n:
        t_i, v_i = series[i]
        if predicate(v_i) and (after_s is None or t_i >= after_s):
            start_t = t_i
            j = i
            while j < n and predicate(series[j][1]):
                j += 1
            end_t = series[j - 1][0]
            if end_t - start_t >= min_duration:
                return start_t
            i = j
        else:
            i += 1
    return None


def first_stop_time(velocities, min_stopped_s=MIN_STOPPED_S, speed_thresh=STOP_SPEED_THRESHOLD_MPS,
                     after_s=None):
    """Forward scan: first timestamp from which velocity stays below
    speed_thresh for at least min_stopped_s continuously (not necessarily
    through the end of the recording — that's the backward-scan/existing
    metrics.py definition)."""
    return _first_qualifying_window(velocities, lambda v: abs(v) < speed_thresh, min_stopped_s, after_s)


def lane_deviation_crossing_time(divergence_series, threshold_m, sustain_s=SUSTAIN_S, after_s=None):
    """First timestamp from which EKF-vs-GT divergence stays above
    threshold_m for at least sustain_s continuously (debounced against
    ordinary EKF noise spikes, confirmed present even in nominal driving)."""
    return _first_qualifying_window(divergence_series, lambda v: v >= threshold_m, sustain_s, after_s)


def gather_trial_data(campaign, goal_id, trial_dir):
    """One bag read per trial — everything downstream (candidates.csv AND
    the threshold sweep) reuses this instead of re-reading, since bag reads
    are the expensive part (~1-2 min/trial)."""
    trial_name = os.path.basename(trial_dir)
    bag_dir = os.path.join(trial_dir, 'rosbag')
    if not os.path.isdir(bag_dir):
        return None

    result_path = os.path.join(trial_dir, 'result.json')
    status = 'unknown'
    if os.path.exists(result_path):
        with open(result_path) as f:
            status = json.load(f).get('status', 'unknown')

    # Fault onset is a fact about the injection itself (fault_log.jsonl
    # records a real tl_fault_start/imu_fault_activated event whenever the
    # fault mechanism armed) — independent of whether the trial went on to
    # fail. Compute it unconditionally, BEFORE the goal_reached skip below
    # (fixed 2026-08-06, Kalpit: onset should always exist since every fault
    # campaign has real injection points; confirmed via fault_log.jsonl that
    # goal_reached trials still have real tl_fault_start events that were
    # previously discarded along with the "skip, no fatal moment" path).
    gt_tl = read_gt_and_tl(bag_dir, goal_id)
    onset_s = trial_fault_onset_rel_s(trial_dir, gt_tl['bag_start_abs_sec'], gt_tl['bag_duration'])

    # Same guard metrics.py's own StaticCollisionMetrics uses: a trial that
    # reached its goal has no "fatal moment" by definition, no matter what a
    # stop-detection heuristic finds in its tail (arriving and idling IS a
    # >=20s stopped window). Missing this guard was a real bug in this
    # script's first pass — it fired on the ordinary post-arrival stop for
    # tl_fault_s2/goal_026 and tl_fault_s3/goal_007, both goal_reached=True.
    # Still applies to the FATAL-MOMENT computation below, just not to onset
    # above — a trial can have a real fault onset and no fatal moment at the
    # same time (that combination is itself informative: the fault fired but
    # didn't derail the trial).
    if status == 'goal_reached':
        return {'campaign': campaign, 'goal_id': goal_id, 'trial': trial_name, 'status': status,
                'skip': True, 'onset_s': onset_s, 'bag_duration': gt_tl['bag_duration']}

    metrics_path = os.path.join(trial_dir, 'metrics.json')
    existing_stop_s = float('nan')
    if os.path.exists(metrics_path):
        with open(metrics_path) as f:
            sc = json.load(f).get('static_collision', {}) or {}
        stop_time = sc.get('permanent_stop_time_s')
        existing_stop_s = float(stop_time) if stop_time is not None else float('nan')

    mc = MetricsCollector(bag_dir)
    mc.read_bag()
    divergence = ekf_gt_divergence(mc, gt_tl['gt_positions'])

    return {
        'campaign': campaign, 'goal_id': goal_id, 'trial': trial_name, 'status': status, 'skip': False,
        'velocities': mc.velocities, 'divergence': divergence, 'onset_s': onset_s,
        'existing_stop_s': existing_stop_s, 'bag_duration': gt_tl['bag_duration'],
    }


def process_trial(data, lane_deviation_m):
    if data is None:
        return None
    if data['skip']:
        onset_s = data.get('onset_s')
        return {
            'campaign': data['campaign'], 'goal_id': data['goal_id'], 'trial': data['trial'],
            'status': data['status'],
            'fault_onset_s': onset_s if onset_s is not None else float('nan'),
            'existing_backward_stop_s': float('nan'), 'first_stop_s': float('nan'),
            'lane_deviation_crossing_s': float('nan'), 'combined_earliest_s': float('nan'),
            'bag_duration_s': data.get('bag_duration', float('nan')),
        }

    onset_s = data['onset_s']
    fwd_stop = first_stop_time(data['velocities'], after_s=onset_s)
    lane_dev = lane_deviation_crossing_time(data['divergence'], lane_deviation_m, after_s=onset_s)
    candidates = [t for t in (fwd_stop, lane_dev) if t is not None]
    combined = min(candidates) if candidates else float('nan')

    return {
        'campaign': data['campaign'], 'goal_id': data['goal_id'], 'trial': data['trial'],
        'status': data['status'], 'fault_onset_s': onset_s if onset_s is not None else float('nan'),
        'existing_backward_stop_s': data['existing_stop_s'],
        'first_stop_s': fwd_stop if fwd_stop is not None else float('nan'),
        'lane_deviation_crossing_s': lane_dev if lane_dev is not None else float('nan'),
        'combined_earliest_s': combined,
        'bag_duration_s': data['bag_duration'],
    }


SWEEP_THRESHOLDS_M = [0.5, 1.0, 1.75, 2.5, 3.5, 5.0]  # 1.0 = control_validator's own max_distance_deviation


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lane-deviation-m', type=float, default=DEFAULT_LANE_DEVIATION_M,
                    help=f'sustained EKF-GT divergence threshold, metres (default {DEFAULT_LANE_DEVIATION_M} '
                         '— control_validator\'s own max_distance_deviation, see module docstring)')
    ap.add_argument('--output', default=DEFAULT_OUTPUT)
    ap.add_argument('--sweep', action='store_true',
                    help='report crossing count/timing across SWEEP_THRESHOLDS_M instead of writing candidates.csv')
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    print('Reading all trials (one bag read each, reused for every threshold if --sweep)...')
    trial_data = []
    for campaign in FAULT_DATASETS:
        campaign_dir = os.path.join(DATA_DIR, campaign)
        if not os.path.isdir(campaign_dir):
            continue
        for goal_id in sorted(g for g in os.listdir(campaign_dir) if g.startswith('goal_')):
            goal_dir = os.path.join(campaign_dir, goal_id)
            for trial_name in sorted(os.listdir(goal_dir)):
                trial_dir = os.path.join(goal_dir, trial_name)
                print(f'  reading {campaign}/{goal_id}/{trial_name}')
                data = gather_trial_data(campaign, goal_id, trial_dir)
                if data:
                    trial_data.append(data)

    if args.sweep:
        pd.set_option('display.width', 200)
        print(f'\n=== Lane-deviation threshold sweep (sustain={SUSTAIN_S}s, gated to after fault onset) ===')
        non_skipped = [d for d in trial_data if not d['skip']]
        sweep_rows = []
        for thresh in SWEEP_THRESHOLDS_M:
            crossing_times = []
            n_cross = 0
            for d in non_skipped:
                t = lane_deviation_crossing_time(d['divergence'], thresh, after_s=d['onset_s'])
                if t is not None:
                    n_cross += 1
                    crossing_times.append(t)
            sweep_rows.append({
                'threshold_m': thresh,
                'n_trials_crossing': n_cross,
                'n_trials_total': len(non_skipped),
                'crossing_rate': n_cross / len(non_skipped) if non_skipped else float('nan'),
                'mean_crossing_s': float(np.mean(crossing_times)) if crossing_times else float('nan'),
                'median_crossing_s': float(np.median(crossing_times)) if crossing_times else float('nan'),
            })
        sweep_df = pd.DataFrame(sweep_rows)
        print(sweep_df.round(3).to_string(index=False))
        print('\nNote: control_validator\'s own max_distance_deviation=1.0m is a self-consistency check '
              '(plan vs. actual control), not this ground-truth comparison — included as a calibration '
              'anchor, not because it directly transfers.')
        return

    rows = [process_trial(d, args.lane_deviation_m) for d in trial_data]
    rows = [r for r in rows if r]
    for row in rows:
        if row['status'] == 'goal_reached':
            continue
        print(f'  {row["campaign"]}/{row["goal_id"]}/{row["trial"]}: '
              f'existing(backward)={row["existing_backward_stop_s"]:.1f}s  '
              f'first_stop(forward)={row["first_stop_s"]:.1f}s  '
              f'lane_deviation={row["lane_deviation_crossing_s"]:.1f}s  '
              f'combined_earliest={row["combined_earliest_s"]:.1f}s  '
              f'fault_onset={row["fault_onset_s"]:.1f}s  (duration={row["bag_duration_s"]:.0f}s)')

    df = pd.DataFrame(rows)
    df.to_csv(args.output, index=False)
    print(f'\nSaved {args.output}')

    pd.set_option('display.width', 200)
    valid = df.dropna(subset=['existing_backward_stop_s', 'combined_earliest_s'])
    if len(valid):
        delta = valid['existing_backward_stop_s'] - valid['combined_earliest_s']
        print(f'\nExisting (backward-scan) anchor vs. combined-earliest candidate, over '
              f'{len(valid)} trials with both defined:')
        print(f'  mean inflation:   {delta.mean():.1f}s')
        print(f'  median inflation: {delta.median():.1f}s')
        print(f'  max inflation:    {delta.max():.1f}s')


if __name__ == '__main__':
    main()
