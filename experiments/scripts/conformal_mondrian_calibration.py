#!/usr/bin/env python3
"""
Mondrian (group-conditional) conformal calibration (2026-08-25).

Vanilla split-conformal (conformal_horizon_calibration.py) pools every
window into ONE global per-step quantile. That's provably valid on
average but can hide badly uneven per-scenario coverage underneath a fine
pooled number -- exactly what audit_minority_scenarios.py found: steering
coverage in tl_zones was 75.6% while the pooled figure read ~90%. This
script implements the standard, principled fix for that specific failure
mode: Mondrian conformal prediction (Vovk, "Conditional Validity of
Inductive Conformal Predictors," JMLR W&CP 25:475-490, 2012) -- partition
the calibration set into groups and calibrate a SEPARATE conformal
quantile per group, each with its own (1-alpha) coverage guarantee,
instead of one number averaged across groups of very different
difficulty.

This supersedes the ad hoc k-NN-in-h_last-space approach tried in
conformal_scene_conditioning.py (2026-08-24): that one worked directionally
but its coverage was noisy and non-monotonic in k. Mondrian grouping is
simpler, has an actual finite-sample coverage guarantee PER GROUP (not
just approximately, like the k-NN version), and reuses the exact same
zone geometry fault_injector.py gates faults on -- no new similarity
metric, no new failure mode to debug.

Groups (mutually exclusive, priority order -- see experiments/lib/
scenario_zones.py for the geometry): turn (turn_zones OR
bias_leadin_zones) > tl_zones > curved_road_zones > open_road. Turn and
TL merged from the 5 audit categories down to fewer, larger groups on
purpose: LOO-CV needs enough residuals per (group, fold) cell to fit a
stable quantile, and bias_leadin_zones/turn_zones showed near-identical
behavior in the audit anyway.

Usage (repo venv sourced, no ROS needed):
  source .venv/bin/activate
  python3 experiments/scripts/conformal_mondrian_calibration.py \
      --model st_gat/models/h30_30/st_gat_rise.pth
"""

import argparse
import json
import math
import os
import sys

import numpy as np
import torch
from torch.utils.data import DataLoader

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from conformal_horizon_calibration import (  # noqa: E402
    conformal_quantile, _trial_ids_for, _FEATURES, _SERIES, _extract_series, _collect_residuals,
)
import scenario_zones  # noqa: E402

DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'conformal_mondrian_calibration')
GROUPS = ['turn', 'tl_zones', 'curved_road_zones', 'open_road']
MIN_FOLD_GROUP_N = 30   # below this, fall back to the pooled/global fold quantile for that (group, fold, step)


def assign_groups(xy):
    """(N,2) real-metre positions -> (N,) array of group name strings, per
    the mutually-exclusive priority order in the module docstring."""
    trees = scenario_zones.load_zone_trees()
    membership = scenario_zones.zone_membership(xy, trees)
    n = len(xy)
    group = np.full(n, 'open_road', dtype=object)
    covered = np.zeros(n, dtype=bool)
    turn_mask = (membership['turn_zones'] | membership['bias_leadin_zones']) & ~covered
    group[turn_mask] = 'turn'
    covered |= turn_mask
    tl_mask = membership['tl_zones'] & ~covered
    group[tl_mask] = 'tl_zones'
    covered |= tl_mask
    curved_mask = membership['curved_road_zones'] & ~covered
    group[curved_mask] = 'curved_road_zones'
    covered |= curved_mask
    return group


def _load_model(model_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path} ({model.count_parameters():,} params)")
    return model, device


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--alpha', type=float, default=0.1)
    ap.add_argument('--min-fold-group-n', type=int, default=MIN_FOLD_GROUP_N)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    model, device = _load_model(args.model)
    ds = TrajectoryDataset(cfg.CAL_DIR)
    n = len(ds)
    trial_id, pkl_files = _trial_ids_for(cfg.CAL_DIR)
    n_trials = len(pkl_files)
    T_out = cfg.OUTPUT_SEQ_LEN
    series_keys = [s[0] for s in _SERIES]
    print(f"Calibration set: {n_trials} trials, {n} windows")

    loader = DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=0)
    print("Collecting residuals (one pass)...")
    all_resid = _extract_series(_collect_residuals(model, device, loader))

    print("Assigning Mondrian groups (turn / tl_zones / curved_road_zones / open_road)...")
    xy = scenario_zones.window_start_xy(ds, cfg)
    group = assign_groups(xy)
    for g in GROUPS:
        print(f"  {g}: {int((group == g).sum())} windows ({100*(group==g).mean():.1f}%)")

    # Per (feature, group): pooled out-of-fold coverage + mean fold quantile.
    report_groups = {g: {k: {} for k in series_keys} for g in GROUPS}
    fallback_count = 0
    total_count = 0

    for key in series_keys:
        resid = all_resid[key]   # (N, T_out)

        # global (fallback) per-fold quantile, same as vanilla conformal
        global_fold_q = np.zeros((n_trials, T_out))
        for fold, held_out in enumerate(range(n_trials)):
            fit_mask = trial_id != held_out
            global_fold_q[fold] = np.array([conformal_quantile(resid[fit_mask, t], args.alpha) for t in range(T_out)])

        for g in GROUPS:
            g_mask = group == g
            fold_q = np.zeros((n_trials, T_out))
            pooled_covered = [[] for _ in range(T_out)]
            for fold, held_out in enumerate(range(n_trials)):
                fit_mask  = (trial_id != held_out) & g_mask
                test_mask = (trial_id == held_out) & g_mask
                n_fit = int(fit_mask.sum())
                for t in range(T_out):
                    total_count += 1
                    if n_fit >= args.min_fold_group_n:
                        q = conformal_quantile(resid[fit_mask, t], args.alpha)
                    else:
                        q = global_fold_q[fold, t]   # fallback: not enough data for this (group, fold)
                        fallback_count += 1
                    fold_q[fold, t] = q
                if test_mask.sum() > 0:
                    covered = resid[test_mask] <= fold_q[fold][np.newaxis, :]
                    for t in range(T_out):
                        pooled_covered[t].append(covered[:, t])

            pooled_cov = np.array([
                np.concatenate(pooled_covered[t]).mean() if pooled_covered[t] else float('nan')
                for t in range(T_out)
            ])
            report_groups[g][key] = {
                'n_windows': int(g_mask.sum()),
                'mean_fold_quantile_by_step': fold_q.mean(axis=0).tolist(),
                'pooled_coverage_by_step': pooled_cov.tolist(),
            }
        print(f"  {key}: done")

    print(f"\nFallback to global quantile used for {fallback_count}/{total_count} (group, fold, step) cells "
          f"(min_fold_group_n={args.min_fold_group_n})")

    print(f"\n{'group':<20}{'feature':<26}{'cov @t=0':>10}{'@t=1.5s':>10}{'@t=3s':>8}   (target {100*(1-args.alpha):.0f}%)")
    print('-' * 84)
    for g in GROUPS:
        for key in series_keys:
            c = report_groups[g][key]['pooled_coverage_by_step']
            print(f"{g:<20}{key:<26}{c[0]:10.3f}{c[14]:10.3f}{c[29]:8.3f}")

    out_path = os.path.join(args.output_dir, 'conformal_mondrian_report.json')
    with open(out_path, 'w') as f:
        json.dump({
            'alpha': args.alpha, 'method': 'mondrian_leave_one_trial_out_cross_conformal',
            'groups': GROUPS, 'min_fold_group_n': args.min_fold_group_n,
            'n_trials': n_trials, 'n_windows': n, 'trials': pkl_files,
            'by_group': report_groups,
        }, f, indent=2)
    print(f'\nSaved {out_path}')

    # ── Width comparison plot: Mondrian per-group width vs. vanilla global width ──
    vanilla_report_path = os.path.join(REPO_DIR, 'experiments', 'analysis',
                                        'conformal_horizon_calibration', 'conformal_report.json')
    if os.path.exists(vanilla_report_path):
        with open(vanilla_report_path) as f:
            vanilla = json.load(f)
        vanilla_q = {row['feature']: row['mean_fold_quantile_by_step'] for row in vanilla['features']}

        ncols = 3
        nrows = math.ceil(len(series_keys) / ncols)
        fig, axes = plt.subplots(nrows, ncols, figsize=(15, 4.3 * nrows))
        steps = np.arange(T_out) * 0.1
        colors = {'turn': '#d62728', 'tl_zones': '#9467bd', 'curved_road_zones': '#2ca02c', 'open_road': '#1f77b4'}
        for ax, key in zip(axes.flat, series_keys):
            ax.plot(steps, vanilla_q[key], color='#7f7f7f', linewidth=2.2, linestyle='--', label='vanilla (pooled global)')
            for g in GROUPS:
                ax.plot(steps, report_groups[g][key]['mean_fold_quantile_by_step'], color=colors[g], linewidth=1.6, label=g)
            ax.set_title(key, fontsize=10)
            ax.set_xlabel('seconds into predicted horizon', fontsize=8)
            ax.tick_params(labelsize=7)
        for ax in axes.flat[len(series_keys):]:
            ax.axis('off')
        axes.flat[0].legend(loc='upper left', fontsize=7)
        fig.suptitle('Mondrian per-group calibrated width vs. vanilla pooled width', fontsize=12)
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        fig.savefig(os.path.join(args.output_dir, 'mondrian_vs_vanilla_width.png'), dpi=140)
        plt.close(fig)
        print(f"Saved {os.path.join(args.output_dir, 'mondrian_vs_vanilla_width.png')}")


if __name__ == '__main__':
    main()
