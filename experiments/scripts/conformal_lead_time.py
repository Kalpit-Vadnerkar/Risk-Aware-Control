#!/usr/bin/env python3
"""
Stage 4, first pass: conformal calibration wrapping the model's own
residual/CUSUM traces (st_gat/residuals.py) into (a) a distribution-free
alarm threshold calibrated on held-out nominal trials, and (b) a lead-time-
to-fatal-moment measurement for fault trials, checked against that exact
threshold. This is the new-contribution artifact docs/theoretical_framework.md
calls "calibrated confidence + lead time" — NOT a bigger-z-wins comparison
like analyze_fault_discriminability.py, which is descriptive background, not
this.

Difference from the z-score calibration already reported in
model_improvement_notes_2026.md: that check verifies the model's OWN Gaussian
variance head roughly matches its error on in-distribution nominal data (an
assumption check). Conformal calibration makes no distributional assumption
at all — it takes an empirical (1-alpha) quantile of a statistic on held-out
nominal data and gets a distribution-free guarantee that a new, exchangeable
nominal trial exceeds it with probability <= alpha. That guarantee is what
"calibrated confidence" in the framework actually refers to, and it's what
lets a threshold-crossing time be called a CALIBRATED lead time instead of
just "the residual went up."

Scenario synchronization (both directions, per Kalpit 2026-08-02): a route-
derived zone-entry time (the same turn/bias-leadin/TL zone fault_injector.py
itself arms faults against, via experiments/configs/{turn,tl}_zones.json) is
used as t=0 for EVERY trial, nominal or fault. This applies symmetrically —
the calibration statistic (from held-out NOMINAL trials) is computed over the
identical window SHAPE that fault trials get tested against. Two statistics
computed over windows of different length/scenario-phase would not be
exchangeable, and the conformal guarantee would not actually hold.

CUSUM search window (fixed 2026-08-04, per Kalpit): originally [entry,
entry+window_s] for both calibration and test — but cusum_combined
accumulates from TRIAL START, not from zone entry, so a trial whose statistic
was already elevated before the window even opened had that accumulation
silently credited to the window's max without ever being searchable as a
crossing point — the reported crossing time (and therefore lead time) was a
lower bound, sometimes a badly wrong one (confirmed: two trials showed the
alarm "arriving after" the corrected fatal moment only because the search
literally could not look before entry). Window is now [0, entry+window_s] —
same end cutoff, same shape for calibration and test (still exchangeable),
but now actually searches the accumulation that happens before the zone.

Deliberately NOT optimized yet (Kalpit 2026-08-02: "let's see what results we
are getting before putting effort in optimizing") — alpha, the statistic
column, and the post-entry window length are all CLI flags with a first-
guess default, not tuned.

Data-scarcity caveat, found running this the first time: turn_zones.json/
tl_zones.json only have entries for goal_007/012/026 — the three goals the
fault campaigns actually ran on (per-goal zone files were only ever computed
for goals that needed a fault gated on one, see CLAUDE.md's "before running a
new TL or IMU fault campaign" section). So the true nominal calibration
sample, per zone kind, is 3 goals x 2 nom_v11 trials = 6 trials, not all 26
nominal goals. Too few to also hold out a separate empirical-false-alarm-rate
check without shrinking calibration further, so this first pass uses all 6
for calibration and skips the holdout check — flagged, not hidden. Revisit if
turn/TL zones ever get computed for the rest of the nominal goal set.

Ground truth caveat: no Arm B (safety-enabled) data exists yet, so the only
fatal-moment anchor available is Arm A's own metrics.json heuristic
(permanent_stop_rel_s / likely_static_collision) — self-referential (Arm A
has safety disabled, so the vehicle can actually reach that state), not the
independent oracle the dissertation eventually wants. mrm_first_trigger_rel_s
is carried through unchanged in the per-trial output so Arm B can be swapped
in as an independent ground truth later without reworking this script.

Usage (ROS/Autoware + repo venv sourced — needs bag_reader.read_bag for raw
ego (x, y) to compute zone-entry time; the precomputed trace CSVs only carry
the model's relative-displacement position representation, not absolute
coordinates usable for a zone-radius check):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/conformal_lead_time.py
  python3 experiments/scripts/conformal_lead_time.py --alpha 0.05 --statistic combined_nll --window-s 15
"""

import argparse
import glob
import math
import os
import sys

import numpy as np
import pandas as pd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.pipeline.bag_reader import read_bag  # noqa: E402
from fault_injector import (  # noqa: E402
    _load_turn_zones_by_goal, _load_tl_group_zones_by_goal,
    _DEFAULT_TL_ZONE_RADIUS_M, _DEFAULT_IMU_TURN_ZONE_RADIUS_M,
    _DEFAULT_IMU_LEADIN_ZONE_RADIUS_M,
)

TRACES_DIR = os.path.join(REPO_DIR, 'st_gat', 'results', cfg.HORIZON_TAG, 'traces')
DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'conformal_lead_time')
NOMINAL_DATASET = 'nom_v11'  # the reference distribution — see fault_scenario_table.md

# Which route-derived zone each campaign's fault actually arms against (see
# docs/fault_scenario_table.md's per-campaign table) — NOT an independent
# choice, this mirrors fault_injector.py's own arming logic exactly, so the
# zone-entry time computed here is the same moment that campaign's real fault
# would have armed at, for both fault trials AND nominal trials of the same
# goal/route.
CAMPAIGN_ZONE_KIND = {
    'imu_fault_s1':     'imu_leadin',
    'imu_fault_s3':     'imu_leadin',
    'imu_fault_scale':  'imu_turn',
    'imu_fault_stuck':  'imu_turn',
    'tl_fault_s2':      'tl',
    'tl_fault_s3':      'tl',
    'tl_fault_s4':      'tl',
    'tl_fault_ramp':    'tl',
}

_ZONE_RADIUS = {
    'tl':         _DEFAULT_TL_ZONE_RADIUS_M,
    'imu_leadin': _DEFAULT_IMU_LEADIN_ZONE_RADIUS_M,
    'imu_turn':   _DEFAULT_IMU_TURN_ZONE_RADIUS_M,
}


def _zone_points_for(zone_kind, goal_id, turn_zones_by_goal, tl_zones_by_goal):
    if zone_kind == 'tl':
        return [(x, y) for (x, y, _gid) in tl_zones_by_goal.get(goal_id, [])]
    turn_pts, _turn_end_pts, leadin_pts = turn_zones_by_goal.get(goal_id, ([], [], []))
    return leadin_pts if zone_kind == 'imu_leadin' else turn_pts


def _zone_entry_rel_s(frames: list, zone_points: list, radius_m: float) -> float | None:
    """First bag-relative second where ego enters within radius of ANY of the
    goal's zone points of the relevant kind — the same "first zone reached"
    rule fault_injector.py's own runtime arming logic uses, so this lines up
    with when a real fault trial would have actually armed."""
    if not frames or not zone_points:
        return None
    t0 = frames[0]['t_sim']
    radius_sq = radius_m ** 2
    for fr in frames:
        ex = fr['ego']['position']['x']
        ey = fr['ego']['position']['y']
        for zx, zy in zone_points:
            if (ex - zx) ** 2 + (ey - zy) ** 2 <= radius_sq:
                return fr['t_sim'] - t0
    return None


def conformal_quantile(scores: list, alpha: float) -> float:
    """Standard split-conformal finite-sample quantile: the k-th smallest of
    n calibration scores, k = ceil((n+1)(1-alpha)), gives P(new score <=
    threshold) >= 1-alpha for any new score exchangeable with the calibration
    set (Vovk et al.; see Angelopoulos & Bates' tutorial for this exact
    formula). Returns +inf if n is too small for the requested alpha (no
    finite threshold gives that guarantee from this few calibration points —
    a real constraint to report, not silently round away)."""
    scores = np.sort(np.asarray(scores, dtype=float))
    n = len(scores)
    if n == 0:
        return float('nan')
    k = math.ceil((n + 1) * (1 - alpha))
    if k > n:
        return float('inf')
    return float(scores[k - 1])


def _goal_from_run_name(run_name: str) -> str:
    parts = run_name.split('_')
    return f'{parts[0]}_{parts[1]}'


def _load_frames_cached(cache: dict, goal_id: str, trial_dir: str, verbose: bool) -> list:
    key = trial_dir
    if key not in cache:
        bag_dir = os.path.join(trial_dir, 'rosbag')
        cache[key] = read_bag(bag_dir, goal_id=goal_id, verbose=verbose) if os.path.isdir(bag_dir) else []
    return cache[key]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--alpha', type=float, default=0.2,
                    help='target false-alarm rate for the conformal threshold (default 0.2, not 0.1 — chosen to '
                         'be ACHIEVABLE given how few zone-tagged nominal trials exist right now, see module '
                         'docstring\'s data-scarcity note, not a target risk level someone picked for the claim)')
    ap.add_argument('--statistic', default='sprt_motion_combined',
                    help='trace column to conformalize (default sprt_motion_combined -- the '
                         'sequential-evidence/SPRT statistic, 2026-08-06, replacing CUSUM as the '
                         'primary detection signal; see st_gat/residuals.py\'s _sprt() docstring. '
                         'try sprt_tl_combined for TL/camera campaigns, sprt_position/velocity/'
                         'steering/acceleration/tl_color/tl_confidence/tl_discrepancy for single-'
                         'feature traces, or cusum_combined/cusum_tl_color/cusum_tl_confidence/'
                         'cusum_tl_discrepancy to compare against the old CUSUM statistic)')
    ap.add_argument('--window-s', type=float, default=20.0,
                    help='seconds after zone entry to look at, both for calibration and testing (default 20s, '
                         'roughly matching the fixed 15-20s fault caps in fault_scenario_table.md)')
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    ap.add_argument('--fatal-moments-csv', default=os.path.join(
        REPO_DIR, 'experiments', 'analysis', 'fatal_moments', 'candidates.csv'),
        help='compute_fatal_moments.py output — combined_earliest_s replaces the old inflated '
             'backward-scan permanent_stop_rel_s as the fatal-moment anchor (see that script\'s '
             'module docstring). Missing file or missing row for a trial falls back to the old anchor.')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    fatal_moments = {}
    fault_onsets = {}
    if os.path.exists(args.fatal_moments_csv):
        fm_df = pd.read_csv(args.fatal_moments_csv)
        for _, r in fm_df.iterrows():
            key = (r['campaign'], r['goal_id'], r['trial'])
            if pd.notna(r.get('combined_earliest_s')):
                fatal_moments[key] = float(r['combined_earliest_s'])
            if pd.notna(r.get('fault_onset_s')):
                fault_onsets[key] = float(r['fault_onset_s'])
        print(f'Loaded {len(fatal_moments)} fatal-moment candidates from {args.fatal_moments_csv}')
    else:
        print(f'WARNING: {args.fatal_moments_csv} not found — falling back to old backward-scan anchor '
              f'for every trial (run compute_fatal_moments.py first)')

    print('Loading route-derived zones...')
    nominal_goal_dir = os.path.join(cfg.DATA_ROOT, NOMINAL_DATASET)
    goal_ids = sorted(g for g in os.listdir(nominal_goal_dir) if g.startswith('goal_'))
    turn_zones_by_goal = _load_turn_zones_by_goal(goal_ids)
    tl_zones_by_goal   = _load_tl_group_zones_by_goal(goal_ids)
    print(f'  {len(goal_ids)} goals, turn zones for {len(turn_zones_by_goal)}, TL zones for {len(tl_zones_by_goal)}')

    frames_cache: dict = {}

    # ── Step 1: per zone-kind calibration (shared across campaigns of that kind) ──
    zone_kinds = sorted(set(CAMPAIGN_ZONE_KIND.values()))
    calibration = {}   # zone_kind -> {'threshold':, 'n_calib':, 'min_achievable_alpha':}
    for zone_kind in zone_kinds:
        radius_m = _ZONE_RADIUS[zone_kind]
        print(f'\n[calibration] zone_kind={zone_kind} (radius={radius_m:.0f}m)')
        calib_scores = []
        for goal_id in goal_ids:
            goal_dir = os.path.join(nominal_goal_dir, goal_id)
            trial_dirs = sorted(
                os.path.join(goal_dir, t) for t in os.listdir(goal_dir)
                if os.path.isdir(os.path.join(goal_dir, t, 'rosbag'))
            )
            zone_points = _zone_points_for(zone_kind, goal_id, turn_zones_by_goal, tl_zones_by_goal)
            for trial_dir in trial_dirs:
                trial_name = os.path.basename(trial_dir)
                trace_csv = os.path.join(TRACES_DIR, NOMINAL_DATASET, f'{goal_id}_{trial_name}.csv')
                if not os.path.exists(trace_csv):
                    continue
                frames = _load_frames_cached(frames_cache, goal_id, trial_dir, args.verbose)
                entry = _zone_entry_rel_s(frames, zone_points, radius_m)
                if entry is None:
                    continue
                df = pd.read_csv(trace_csv)
                # Search from trial START, not from zone entry (fixed
                # 2026-08-04 — see CUSUM_SEARCH_FROM_START docstring note):
                # end cutoff (entry + window_s) unchanged, so calibration and
                # test trials still use the identical window SHAPE, just one
                # that now actually captures pre-entry accumulation instead
                # of silently crediting whatever the statistic already was
                # the instant the old window started.
                window = df[(df['t_bag_rel'] >= 0) & (df['t_bag_rel'] <= entry + args.window_s)]
                if window.empty or args.statistic not in window.columns:
                    continue
                calib_scores.append(float(window[args.statistic].max()))

        threshold = conformal_quantile(calib_scores, args.alpha)
        n = len(calib_scores)
        min_alpha = 1.0 / (n + 1) if n > 0 else float('nan')
        calibration[zone_kind] = {
            'threshold': threshold, 'n_calib': n, 'min_achievable_alpha': min_alpha,
        }
        print(f'  n_calib={n}  threshold={threshold:.4f}  '
              f'(min achievable alpha given n={n} is {min_alpha:.3f}; requested alpha={args.alpha})')

    pd.DataFrame.from_dict(calibration, orient='index').to_csv(
        os.path.join(args.output_dir, 'calibration_by_zone_kind.csv'))

    # ── Step 2: per-fault-trial lead time against that zone-kind's threshold ──
    trial_rows = []
    for campaign, zone_kind in CAMPAIGN_ZONE_KIND.items():
        radius_m = _ZONE_RADIUS[zone_kind]
        threshold = calibration[zone_kind]['threshold']
        campaign_dir = os.path.join(cfg.DATA_ROOT, campaign)
        trace_campaign_dir = os.path.join(TRACES_DIR, campaign)
        if not os.path.isdir(campaign_dir) or not os.path.isdir(trace_campaign_dir):
            print(f'[lead-time] WARNING: no data for {campaign}, skipping')
            continue

        print(f'\n[lead-time] campaign={campaign}  zone_kind={zone_kind}  threshold={threshold:.4f}')
        for goal_id in sorted(g for g in os.listdir(campaign_dir) if g.startswith('goal_')):
            goal_dir = os.path.join(campaign_dir, goal_id)
            zone_points = _zone_points_for(zone_kind, goal_id, turn_zones_by_goal, tl_zones_by_goal)
            for trial_name in sorted(os.listdir(goal_dir)):
                trial_dir = os.path.join(goal_dir, trial_name)
                trace_csv = os.path.join(trace_campaign_dir, f'{goal_id}_{trial_name}.csv')
                if not os.path.exists(trace_csv):
                    continue
                frames = _load_frames_cached(frames_cache, goal_id, trial_dir, args.verbose)
                entry = _zone_entry_rel_s(frames, zone_points, radius_m)
                if entry is None:
                    print(f'  {goal_id}/{trial_name}: zone never reached, skipping')
                    continue

                key = (campaign, goal_id, trial_name)
                onset_rel_s = fault_onsets.get(key, float('nan'))

                df = pd.read_csv(trace_csv)
                # Same trial-start-to-(entry+window_s) shape as the
                # calibration side above — see that block's comment.
                window = df[(df['t_bag_rel'] >= 0) & (df['t_bag_rel'] <= entry + args.window_s)]
                if window.empty or args.statistic not in window.columns:
                    continue
                # Crossing must occur AT OR AFTER this trial's own fault
                # onset (fixed 2026-08-04, per Kalpit: most crossings were
                # firing before the fault even armed — traced one case to a
                # genuine, large residual anomaly unrelated to the fault,
                # confirmed absent in nominal trials of the same goal at the
                # same time. cusum_combined is a general behavioral-divergence
                # statistic, not fault-specific — it WILL fire on other real
                # anomalies, and crediting a pre-onset crossing as "detecting
                # the fault" is causally incoherent, not just optimistic.
                # Window itself still starts at 0 (unchanged, keeps the same
                # shape as calibration's window for exchangeability) — only
                # the crossing SEARCH is onset-gated.
                above = window[window[args.statistic] > threshold]
                crossing = above[above['t_bag_rel'] >= onset_rel_s] if not math.isnan(onset_rel_s) else above
                detected = not crossing.empty
                crossing_rel_s = float(crossing['t_bag_rel'].min()) if detected else float('nan')

                # Old anchor (backward-scan, kept for comparison — see
                # compute_fatal_moments.py's module docstring for why this
                # was found to be inflated, sometimes by 100+ seconds).
                old_fatal_rel_s = float(df['permanent_stop_rel_s'].iloc[0]) if len(df) else float('nan')
                likely_collision = bool(df['likely_static_collision'].iloc[0]) if len(df) else False
                mrm_rel_s = float(df['mrm_first_trigger_rel_s'].iloc[0]) if len(df) else float('nan')

                # New anchor (2026-08-04): combined_earliest_s from
                # compute_fatal_moments.py — forward-scan first-stop and
                # sustained lane-deviation crossing, both gated to after this
                # trial's own fault onset (not the campaign-cumulative log's
                # earliest onset — see that script's docstring). Falls back
                # to the old anchor if this trial has no fatal_moments row
                # (script covers fault campaigns only, matches this script's
                # own scope).
                new_fatal_rel_s = fatal_moments.get(key, float('nan'))
                fatal_rel_s = new_fatal_rel_s if not math.isnan(new_fatal_rel_s) else old_fatal_rel_s

                lead_time_s = (fatal_rel_s - crossing_rel_s
                                if detected and not math.isnan(fatal_rel_s) else float('nan'))

                # Staged timing, not just the single lead-time number (per
                # Kalpit 2026-08-04): every stage relative to fault onset, so
                # it's legible how much of the trial each gap actually covers
                # rather than three numbers with three different zero-points.
                detection_latency_s = (crossing_rel_s - onset_rel_s
                                        if detected and not math.isnan(onset_rel_s) else float('nan'))
                fatal_latency_s = (fatal_rel_s - onset_rel_s
                                    if not math.isnan(fatal_rel_s) and not math.isnan(onset_rel_s)
                                    else float('nan'))

                trial_rows.append({
                    'campaign': campaign, 'zone_kind': zone_kind, 'goal_id': goal_id, 'trial': trial_name,
                    'fault_onset_rel_s': onset_rel_s, 'zone_entry_rel_s': entry,
                    'detected': detected, 'crossing_rel_s': crossing_rel_s,
                    'detection_latency_s': detection_latency_s,   # onset -> model alarm
                    'fatal_moment_rel_s': fatal_rel_s, 'fatal_latency_s': fatal_latency_s,  # onset -> danger
                    'old_backward_scan_rel_s': old_fatal_rel_s,
                    'likely_static_collision': likely_collision,
                    'mrm_first_trigger_rel_s': mrm_rel_s,   # Arm B slot-in point — not used yet, no Arm B data
                    'lead_time_s': lead_time_s,   # = fatal_latency_s - detection_latency_s
                })

    per_trial = pd.DataFrame(trial_rows)
    per_trial.to_csv(os.path.join(args.output_dir, 'per_trial.csv'), index=False)

    if per_trial.empty:
        print('\nNo fault trials produced a usable row — nothing to summarize.')
        return

    def _summarize(g):
        n = len(g)
        n_detected = int(g['detected'].sum())
        lead_times = g.loc[g['lead_time_s'].notna(), 'lead_time_s']
        return pd.Series({
            'n_trials': n,
            'detection_rate': n_detected / n if n else float('nan'),
            'mean_lead_time_s': float(lead_times.mean()) if len(lead_times) else float('nan'),
            'median_lead_time_s': float(lead_times.median()) if len(lead_times) else float('nan'),
            'n_with_fatal_moment': int(len(lead_times)),
        })

    campaign_summary = per_trial.groupby('campaign')[
        ['detected', 'lead_time_s']].apply(_summarize)
    campaign_summary.to_csv(os.path.join(args.output_dir, 'campaign_summary.csv'))

    # Per-(campaign, goal) detail (per Kalpit 2026-08-04: campaign-level
    # aggregates were hiding which SPECIFIC goal each number came from).
    # Confirmed-bad trials are deleted outright now, not tracked via an
    # exclusion list (see config.py) — a goal with no data for a campaign
    # just doesn't produce a row here, no special marking needed.
    detail_cols = ['goal_id', 'trial', 'fault_onset_rel_s', 'detected', 'crossing_rel_s',
                   'detection_latency_s', 'fatal_moment_rel_s', 'fatal_latency_s', 'lead_time_s']
    per_goal_rows = []
    for campaign in CAMPAIGN_ZONE_KIND:
        campaign_trials = per_trial[per_trial.campaign == campaign]
        for _, r in campaign_trials.iterrows():
            per_goal_rows.append({'campaign': campaign, **{c: r[c] for c in detail_cols}})
    per_goal_detail = pd.DataFrame(per_goal_rows).sort_values(['campaign', 'goal_id']).reset_index(drop=True)
    per_goal_detail.to_csv(os.path.join(args.output_dir, 'per_goal_detail.csv'), index=False)

    pd.set_option('display.width', 200)
    print(f'\n=== Conformal calibration by zone kind (alpha={args.alpha}, statistic={args.statistic}, '
          f'window={args.window_s}s) ===')
    print(pd.DataFrame.from_dict(calibration, orient='index').round(4).to_string())
    print('\n=== Per-campaign lead time (only over trials with a real fatal moment) ===')
    print(campaign_summary.round(3).to_string())
    print('\n=== Per-goal detail (staged timing: onset -> detection -> danger) ===')
    print(per_goal_detail.round(2).to_string(index=False))
    print('\nCaveat: fatal moment = Arm A\'s own metrics.json heuristic (no Arm B ground truth')
    print('collected yet). mrm_first_trigger_rel_s is carried through in per_trial.csv for when')
    print('Arm B data exists and can replace it as an independent oracle.')


if __name__ == '__main__':
    main()
