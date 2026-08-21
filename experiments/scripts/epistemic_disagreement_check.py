#!/usr/bin/env python3
"""
Epistemic-disagreement check: does cross-member disagreement between
independently-trained point predictors widen over the prediction horizon,
the way actual error does?

Why this exists (2026-08-20): the paused deep-ensemble design
(st_gat/model/ensemble.py, see the memory system's
project_ensemble_reconsideration_2026-08-19) originally planned to check
horizon-widening via a single Gaussian member's ALEATORIC scale -- but we
already know from the whole Student-t arc that single-network self-reported
scale stays flat regardless of family (Gaussian vs. Student-t, dof->inf is
the Gaussian limit). That's not a new test. The actually-untested mechanism
is EPISTEMIC uncertainty -- variance of independently-trained members' MEAN
predictions -- which is what ensembling is supposed to add on top of
whatever a single member's self-reported uncertainty already does (or
doesn't) do. This script tests that mechanism directly and cheaply: no
NLL/scale/dof head involved anywhere, just M independently-trained
(different seed/data order) point predictors (`st_gat/train.py
--warmup-only`), checking whether their pairwise disagreement grows with
horizon.

Note this is a DIFFERENT question from conformal_horizon_calibration.py's
(which asks "is a single point predictor's calibrated interval correct on
held-out NOMINAL data"). This one asks "does model disagreement behave like
a useful uncertainty signal at all" -- most relevant to whether ensembling
would help calibration UNDER DISTRIBUTION SHIFT (faults), which conformal
calibration (validated only under exchangeability, i.e. nominal data) does
not by itself address. See
docs/research_notes/nll_calibration_arc_and_conformal_pivot_2026-08-20.md's
closing section.

Method: law-of-total-variance style epistemic variance -- for M members'
mean predictions {mu_1, ..., mu_M} at a given sample/horizon-step,
epistemic_std = std across members of mu_i (population std, ddof=0),
matching st_gat/model/ensemble.py's own combination formula (kept
consistent in case this ever feeds into that ensemble design). Averaged
over all samples in cfg.CAL_DIR, per feature-series (same _SERIES
convention as conformal_horizon_calibration.py, reused not reimplemented),
per horizon step, plotted against actual RMSE the same way.

Usage (repo venv sourced; no ROS needed):
  source .venv/bin/activate
  python3 experiments/scripts/epistemic_disagreement_check.py \
      --models st_gat/checkpoints/h30_30_epistemic_m1/mean_warmup.pth \
               st_gat/checkpoints/h30_30_epistemic_m2/mean_warmup.pth
"""

import argparse
import json
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
sys.path.insert(0, SCRIPT_DIR)   # for conformal_horizon_calibration import below

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from conformal_horizon_calibration import _SERIES, _extract_series  # noqa: E402

DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'epistemic_disagreement')


def _load_model(model_path, device):
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded member: {model_path} ({model.count_parameters():,} params)")
    return model


def _collect_means(model, device, loader) -> dict:
    """Same convention as conformal_horizon_calibration.py's
    _collect_residuals, but returns the raw MEAN prediction per source
    feature (not actual-minus-mean) -- needed here since disagreement is
    between members' predictions, not against ground truth."""
    out = {k: [] for k in {s[1] for s in _SERIES}}
    with torch.no_grad():
        for past, _future, graph, _bounds in loader:
            past  = {k: v.to(device) for k, v in past.items()}
            graph = {k: v.to(device) for k, v in graph.items()}
            preds = model(past, graph)
            for key in out:
                out[key].append(preds[f'{key}_mean'].cpu().numpy())
    return {k: np.concatenate(v, axis=0) for k, v in out.items()}


def _collect_actuals(loader) -> dict:
    out = {k: [] for k in {s[1] for s in _SERIES}}
    for _past, future, _graph, _bounds in loader:
        for key in out:
            actual = future[key]
            if actual.dim() == 3 and actual.size(-1) == 1:
                actual = actual.squeeze(-1)
            out[key].append(actual.numpy())
    return {k: np.concatenate(v, axis=0) for k, v in out.items()}


def _rmse_by_step(residuals: np.ndarray) -> np.ndarray:
    return np.sqrt((residuals ** 2).mean(axis=0))


def plot_comparison(epi_std, actual_rmse, out_path):
    keys = list(epi_std.keys())
    T_out = epi_std[keys[0]].shape[0]
    steps = np.arange(T_out) * 0.1
    ncols = 3
    nrows = -(-len(keys) // ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(14, 4.3 * nrows))
    for ax, key in zip(axes.flat, keys):
        ax.plot(steps, epi_std[key], color='#d62728', linewidth=2, linestyle='--',
                label='cross-member epistemic std')
        ax.plot(steps, actual_rmse[key], color='#1f77b4', linewidth=2,
                label='actual RMSE (mean member)')
        ax.set_title(key, fontsize=10)
        ax.set_xlabel('seconds into predicted horizon', fontsize=8)
        ax.tick_params(labelsize=7)
    for ax in axes.flat[len(keys):]:
        ax.axis('off')
    axes.flat[0].legend(loc='upper left', fontsize=7)
    fig.suptitle('Epistemic disagreement — cross-member std vs. actual error growth '
                 '(independently-trained point predictors, no distributional head)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--models', nargs='+', required=True, help='paths to >=2 independently-trained mean_warmup.pth checkpoints')
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    if len(args.models) < 2:
        raise SystemExit('need at least 2 --models to measure cross-member disagreement')
    os.makedirs(args.output_dir, exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    ds = TrajectoryDataset(cfg.CAL_DIR)
    loader = DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=0)

    print("Collecting ground truth...")
    actuals = _collect_actuals(loader)

    member_means = []   # list of {source_feature: (N, T_out[, dims])} per member
    for path in args.models:
        model = _load_model(path, device)
        loader = DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=0)
        print(f"Collecting mean predictions ({path})...")
        member_means.append(_collect_means(model, device, loader))
        del model

    source_keys = list(actuals.keys())
    T_out = cfg.OUTPUT_SEQ_LEN
    M = len(member_means)

    # Cross-member epistemic std per source feature (population std across
    # the M members, matching ensemble.py's own law-of-total-variance
    # convention), and the mean-across-members prediction's own error vs.
    # ground truth, both computed at the raw (pre-series-reduction) level
    # so _extract_series can reuse the exact same l2/axis/scalar convention
    # conformal_horizon_calibration.py already validated.
    epi_errors    = {}   # {source_feature: (N, T_out[, dims]) std-across-members, reused as "error" input to _extract_series}
    mean_actual_err = {}
    for key in source_keys:
        stacked = np.stack([m[key] for m in member_means], axis=0)   # (M, N, T_out[, dims])
        epi_errors[key] = stacked.std(axis=0, ddof=0)                 # (N, T_out[, dims]) -- treated as "magnitude" directly below
        mean_pred = stacked.mean(axis=0)
        mean_actual_err[key] = actuals[key] - mean_pred

    # _extract_series expects (error) arrays it reduces via l2/axis/scalar;
    # epi_errors are already non-negative per-dim magnitudes (a std, not a
    # signed error), so reuse the same reduction logic directly rather than
    # re-deriving it -- 'l2' pools via norm (fine for a non-negative std
    # array too, gives combined-axis epistemic magnitude), 'axis'/'scalar'
    # select/abs as usual (abs is a no-op on an already-nonnegative std).
    epi_series    = _extract_series(epi_errors)
    actual_series = _extract_series(mean_actual_err)

    series_keys = [s[0] for s in _SERIES]
    epi_std      = {k: epi_series[k].mean(axis=0) for k in series_keys}   # avg epistemic magnitude per step
    actual_rmse  = {k: _rmse_by_step(actual_series[k]) for k in series_keys}

    print(f"\n{'feature':<26}{'epi_std @t=0':>14}{'@t=1.5s':>10}{'@t=3s':>8}   |  {'actualRMSE @t=0':>16}{'@t=1.5s':>10}{'@t=3s':>8}")
    print('-' * 100)
    rows = []
    for key in series_keys:
        e0, e15, e29 = epi_std[key][0], epi_std[key][14], epi_std[key][29]
        r0, r15, r29 = actual_rmse[key][0], actual_rmse[key][14], actual_rmse[key][29]
        print(f"{key:<26}{e0:14.4f}{e15:10.4f}{e29:8.4f}   |  {r0:16.4f}{r15:10.4f}{r29:8.4f}")
        rows.append({
            'feature': key,
            'epistemic_std_by_step': epi_std[key].tolist(),
            'actual_rmse_by_step': actual_rmse[key].tolist(),
            'epi_std_growth_ratio': float(epi_std[key][-1] / max(epi_std[key][0], 1e-9)),
            'actual_rmse_growth_ratio': float(actual_rmse[key][-1] / max(actual_rmse[key][0], 1e-9)),
        })

    report_path = os.path.join(args.output_dir, 'epistemic_report.json')
    with open(report_path, 'w') as f:
        json.dump({'n_members': M, 'models': args.models, 'features': rows}, f, indent=2)
    print(f'\nSaved {report_path}')

    plot_comparison(epi_std, actual_rmse, os.path.join(args.output_dir, 'epistemic_vs_actual.png'))


if __name__ == '__main__':
    main()
