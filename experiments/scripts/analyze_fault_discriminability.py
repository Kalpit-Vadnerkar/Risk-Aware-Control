#!/usr/bin/env python3
"""
Experiment A analysis (TODO.md Priority 0 / P0.1): is each divergence-trace
residual feature selectively discriminative for the fault class it should be,
or uniformly dominant regardless of fault type?

Run fresh against this repo's own retrained ST-GAT model and its own
newly-collected fault campaigns (via st_gat/residuals.py's per-timestep
traces) — not a recomputation of the published T-ITS paper's Fig. 9/10,
which TODO.md flags as living in a different, read-only reference repo.
Same falsifiable prediction either way: traffic_light_color/confidence/
discrepancy residuals should be selectively high for TL/camera faults and
much lower for IMU faults. If they're uniformly dominant across both fault
classes, that's a clean-signal artifact, not evidence for the map-grounded
negative-evidence mechanism (see docs/theoretical_framework.md §3).

For each fault trial, compares in-fault vs. out-of-fault z-scores against
THAT SAME GOAL's own pooled nominal baseline — not a cross-goal average.
Some goals (nom_v11/goal_026, goal_012) show elevated
traffic_light_color_nll even with zero fault, so a fair comparison has to
control for per-goal baseline differences, the same principle
compare_fault_vs_nominal.py already applies to raw signals; this script
reuses its fault-window extraction directly rather than re-deriving it.

Usage (must source ROS/Autoware, then the repo venv — needed by
compare_fault_vs_nominal.py's own imports, even though this script itself
only reads pre-computed trace CSVs and fault_log.jsonl, no rosbags):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/analyze_fault_discriminability.py

Requires st_gat.residuals to have already been run (produces the trace CSVs
this script reads from st_gat/results/<horizon_tag>/traces/).
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)                                        # so `import st_gat` resolves
sys.path.insert(0, SCRIPT_DIR)                                       # for compare_fault_vs_nominal
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))     # its own dependencies

from compare_fault_vs_nominal import load_fault_log, extract_fault_windows, in_any_window  # noqa: E402
from st_gat.pipeline import config as cfg  # noqa: E402

TRACES_DIR = os.path.join(REPO_DIR, 'st_gat', 'results', cfg.HORIZON_TAG, 'traces')
DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'fault_discriminability')

# Candidate residual features — raw per-feature NLLs/residuals, NOT the
# composite combined_nll (would muddy "which single feature wins").
CANDIDATE_FEATURES = [
    'position_nll', 'velocity_nll', 'steering_nll', 'acceleration_nll',
    'traffic_light_color_nll', 'traffic_light_confidence_nll',
    'traffic_light_discrepancy_residual',
]


def _goal_from_run_name(run_name: str) -> str:
    parts = run_name.split('_')
    return f'{parts[0]}_{parts[1]}'


def load_nominal_baselines() -> dict:
    """goal_id -> {feature: (mean, std)}, pooled across that goal's own
    nominal trace CSVs — the per-goal baseline every fault trial for that
    goal gets compared against, not a cross-goal pooled average."""
    per_goal_values: dict = {}
    for f in sorted(glob.glob(os.path.join(TRACES_DIR, 'nom_v11', '*.csv'))):
        run_name = os.path.splitext(os.path.basename(f))[0]
        goal_id = _goal_from_run_name(run_name)
        df = pd.read_csv(f)
        bucket = per_goal_values.setdefault(goal_id, {feat: [] for feat in CANDIDATE_FEATURES})
        for feat in CANDIDATE_FEATURES:
            if feat in df.columns:
                bucket[feat].extend(df[feat].dropna().tolist())

    baselines = {}
    for goal_id, feats in per_goal_values.items():
        baselines[goal_id] = {}
        for feat, values in feats.items():
            arr = np.asarray(values, dtype=float)
            arr = arr[np.isfinite(arr)]
            if arr.size == 0:
                baselines[goal_id][feat] = (0.0, 1.0)
            else:
                std = arr.std()
                baselines[goal_id][feat] = (float(arr.mean()), float(std if std > 1e-9 else 1e-9))
    return baselines


def analyze_trial(campaign: str, goal_id: str, trial_dirname: str, trace_csv: str, baselines: dict):
    """One row: per-feature discriminability (mean|z| in-fault - mean|z|
    out-of-fault) for one trial, against its own goal's nominal baseline."""
    df = pd.read_csv(trace_csv)
    if goal_id not in baselines:
        print(f'  WARNING: no nominal baseline for {goal_id} — skipping {campaign}/{trial_dirname}')
        return None

    trial_dir = os.path.join(cfg.DATA_ROOT, campaign, goal_id, trial_dirname)
    events = load_fault_log(trial_dir)
    kind = 'tl' if campaign.startswith('tl_fault') else 'imu'
    # bag_start_abs_sec / bag_duration derived from the trace's own t_sim/
    # t_bag_rel (t_bag_rel = t_sim - bag_start_abs_sec is a constant offset
    # for every row) — avoids re-reading the rosbag just for this.
    bag_start_abs_sec = float((df['t_sim'] - df['t_bag_rel']).iloc[0])
    bag_duration = float(df['t_bag_rel'].max())
    windows = extract_fault_windows(events, bag_start_abs_sec, bag_duration, kind)
    if not windows:
        print(f'  WARNING: no fault windows found for {campaign}/{goal_id}/{trial_dirname} — skipping')
        return None

    in_mask = df['t_bag_rel'].apply(lambda t: in_any_window(t, windows)).values
    if not in_mask.any() or in_mask.all():
        print(f'  WARNING: {campaign}/{goal_id}/{trial_dirname} in-fault mask is all-or-nothing '
              f'({int(in_mask.sum())}/{len(in_mask)} rows) — skipping')
        return None

    row = {'campaign': campaign, 'goal_id': goal_id, 'trial': trial_dirname, 'n_windows': len(windows)}
    for feat in CANDIDATE_FEATURES:
        if feat not in df.columns:
            continue
        mean_b, std_b = baselines[goal_id][feat]
        z = np.abs((df[feat].values - mean_b) / std_b)
        valid = np.isfinite(z)
        in_z  = z[valid & in_mask]
        out_z = z[valid & ~in_mask]
        if in_z.size == 0 or out_z.size == 0:
            continue
        row[f'{feat}_delta'] = float(in_z.mean() - out_z.mean())
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    print('Loading per-goal nominal baselines...')
    baselines = load_nominal_baselines()
    print(f'  {len(baselines)} goals with a nominal baseline\n')

    rows = []
    for campaign in cfg.FAULT_DATASETS:
        campaign_dir = os.path.join(TRACES_DIR, campaign)
        if not os.path.isdir(campaign_dir):
            print(f'WARNING: no traces for {campaign} — run st_gat.residuals first, skipping')
            continue
        for f in sorted(glob.glob(os.path.join(campaign_dir, '*.csv'))):
            run_name = os.path.splitext(os.path.basename(f))[0]
            goal_id = _goal_from_run_name(run_name)
            trial_dirname = run_name[len(goal_id) + 1:]
            print(f'  analyzing {campaign}/{run_name}')
            row = analyze_trial(campaign, goal_id, trial_dirname, f, baselines)
            if row:
                rows.append(row)

    per_trial = pd.DataFrame(rows)
    per_trial_path = os.path.join(args.output_dir, 'per_trial.csv')
    per_trial.to_csv(per_trial_path, index=False)
    print(f'\nSaved {per_trial_path}')

    delta_cols = [c for c in per_trial.columns if c.endswith('_delta')]
    per_campaign = per_trial.groupby('campaign')[delta_cols].mean()
    per_campaign.to_csv(os.path.join(args.output_dir, 'per_campaign_summary.csv'))

    per_trial['fault_class'] = per_trial['campaign'].apply(
        lambda c: 'TL/camera' if c.startswith('tl_fault') else 'IMU')
    per_class = per_trial.groupby('fault_class')[delta_cols].mean()
    per_class.to_csv(os.path.join(args.output_dir, 'per_class_summary.csv'))

    # Raw delta magnitudes aren't comparable ACROSS features (each feature's
    # NLL/residual has its own scale, and one outlier trial can dominate a
    # mean) — the actual discriminability test is which feature wins WITHIN
    # each row, not whose raw number is biggest.
    per_campaign_rank = per_campaign.rank(axis=1, ascending=False, method='min').astype(int)
    per_campaign_rank.to_csv(os.path.join(args.output_dir, 'per_campaign_rank.csv'))
    per_class_rank = per_class.rank(axis=1, ascending=False, method='min').astype(int)
    per_class_rank.to_csv(os.path.join(args.output_dir, 'per_class_rank.csv'))

    pd.set_option('display.width', 200)
    print('\n=== Per-campaign discriminability (mean|z| in-fault minus out-of-fault) ===')
    print(per_campaign.round(3).to_string())

    print('\n=== Per-campaign RANK (1 = most discriminative feature for that campaign) ===')
    print(per_campaign_rank.to_string())

    print('\n=== Per-FAULT-CLASS summary — the actual falsifiable test ===')
    print(per_class.round(3).to_string())
    print('\n=== Per-FAULT-CLASS RANK ===')
    print(per_class_rank.to_string())
    print('\nPrediction: a traffic_light_* feature should rank #1 (or near it) for TL/camera')
    print('and rank low for IMU; a velocity/steering/acceleration feature should rank #1 for')
    print('IMU. Uniform dominance of the same feature(s) in both rows would not support the')
    print('negative-evidence mechanism.')


if __name__ == '__main__':
    main()
