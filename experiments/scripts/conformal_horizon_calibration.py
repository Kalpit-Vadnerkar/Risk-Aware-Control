#!/usr/bin/env python3
"""
Conformal-calibration prototype: per-feature, per-horizon-step calibrated
intervals wrapped around the model's own POINT prediction (the `_mean`
output only -- scale/dof/dof_reg are never touched), instead of relying on
the jointly-trained Student-t heads' self-reported uncertainty.

Why this exists (2026-08-20): the independent architecture review this
session (see the memory system's project_calibration_attempt_log_2026-08-20
and project_horizon_embedding_fix_2026-08-19) flagged that the whole
NLL-training pathology being fought (dof-collapse, now confirmed a
scale/validation-residual mismatch too -- see the same memory) is a
consequence of jointly training a network to BOTH predict the next state
AND self-report its own confidence in one objective. Conformal calibration
sidesteps that entirely: take a plain point predictor (Phase 1's mean-only
warmup already IS one, and its point accuracy has been solid throughout
every attempt this session), and derive a CALIBRATED interval directly from
held-out residuals -- no joint optimization, no distribution-family
assumption, no per-head collapse to fight.

Method: LEAVE-ONE-TRIAL-OUT CROSS-CONFORMAL (2026-08-20; standard split
conformal -- Vovk et al.; see Angelopoulos & Bates' tutorial -- run once per
fold, same finite-sample quantile formula already used in
experiments/scripts/conformal_lead_time.py, reimplemented here rather than
imported since that module pulls in ROS/bag_reader/fault_injector at import
time this script has no need for). `cfg.CAL_DIR` currently has only 7
trials (`nom_v11`) -- an earlier version of this script used one fixed
50/50 trial split, which an independent review + direct rerun showed gives
a noisy, low-power coverage estimate at this trial count (a single 3-fit/
4-test split let several features drift well off the 90% target just from
which trials happened to land where). Leave-one-trial-out fixes this
without needing more data: for each of the n_trials folds, fit the
per-horizon-step quantile on the OTHER n_trials-1 trials, check coverage on
the held-out trial (every trial serves as the held-out test trial exactly
once), then POOL every fold's out-of-fold coverage indicators into one
number per horizon step (the standard cross-conformal / CV+ aggregate,
Barber et al.) -- uses the full dataset's worth of test evidence while
every single check point stays genuinely out-of-fold. Per-fold coverage is
also kept and reported (min-max spread at t=0) so the remaining fold-to-
fold instability from having only 7 trials is visible, not hidden behind
one aggregate number.

Per feature-SERIES (position pooled L2 across x/y; velocity split into
velocity_longitudinal/velocity_lateral -- see _SERIES's docstring for why;
steering, acceleration, traffic_light_color, traffic_light_confidence as
scalars -- NOT traffic_light_discrepancy, a binary/probability output that
needs a different treatment, e.g. Platt/temperature scaling, already
explored in calibrate_tl_discrepancy.py) and per horizon step t=0..29:
  residual_t = |actual_t - predicted_mean_t|  (per _SERIES's l2/axis/scalar mode)
  q_t^(fold) = conformal_quantile(residuals on the other n_trials-1 trials, alpha)
  pooled_coverage_t = fraction of ALL out-of-fold residuals <= their OWN
                       fold's q_t^(fold) -- should be close to 1-alpha

Produces the same visual language as plot_calibration_diagrams.py's
horizon_widening.png (predicted vs. actual, seconds into horizon; the
plotted "predicted" curve is the across-fold MEAN quantile, a
representative interval width, not any single fold's) so the two are
directly comparable side by side, plus a coverage table.

Usage (repo venv sourced; no ROS needed -- TrajectoryDataset reads
precomputed .pkl sequences, not live bags):
  source .venv/bin/activate
  python3 experiments/scripts/conformal_horizon_calibration.py \
      --model st_gat/checkpoints/h30_30_dofreg_test/best_model.pth
"""

import argparse
import json
import math
import os
import pickle
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

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402

DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'conformal_horizon_calibration')

_FEATURES = {
    'position':                  2,
    'velocity':                  2,
    'steering':                  1,
    'acceleration':               1,
    'traffic_light_color':       1,
    'traffic_light_confidence':  1,
}

# Per-series calibration/reporting spec: (report_key, source_feature, mode).
# mode 'l2' pools all dims into one magnitude (valid when dims share a
# scale -- position's x/y both use one POSITION_DISPLACEMENT_RANGE_M, see
# sequence_builder.py); mode ('axis', i) calibrates dim i on its own.
# velocity split 2026-08-20 per an independent review's finding:
# VELOCITY_X_RANGE=(-0,12) vs VELOCITY_Y_RANGE=(-0.4,0.4) (config.py) is a
# 30x scale difference -- pooling into one L2 magnitude makes the resulting
# quantile essentially a longitudinal-error bound with lateral folded in
# almost invisibly (still a VALID bound per-axis, since |x_err| <=
# ||err||, just needlessly loose for the smaller-scale axis). position
# doesn't have this problem (shared scale), so stays pooled.
_SERIES = [
    ('position',                 'position',                 'l2'),
    ('velocity_longitudinal',    'velocity',                 ('axis', 0)),
    ('velocity_lateral',         'velocity',                 ('axis', 1)),
    ('steering',                 'steering',                 'scalar'),
    ('acceleration',             'acceleration',              'scalar'),
    ('traffic_light_color',      'traffic_light_color',       'scalar'),
    ('traffic_light_confidence', 'traffic_light_confidence',  'scalar'),
]

_SPLIT_SEED = 20260820  # fixed, so fit/test halves are reproducible run to run


def conformal_quantile(residuals: np.ndarray, alpha: float) -> float:
    """Standard split-conformal finite-sample quantile (Vovk et al.): the
    k-th smallest of n calibration residuals, k = ceil((n+1)(1-alpha)),
    gives P(new residual <= threshold) >= 1-alpha for any new point
    exchangeable with the calibration set. Same formula as
    conformal_lead_time.py's conformal_quantile(), reimplemented locally
    (see module docstring for why)."""
    residuals = np.sort(residuals)
    n = len(residuals)
    if n == 0:
        return float('nan')
    k = math.ceil((n + 1) * (1 - alpha))
    if k > n:
        return float('inf')
    return float(residuals[k - 1])


def _trial_ids_for(data_folder: str) -> np.ndarray:
    """Per-sequence trial id, in the SAME order TrajectoryDataset builds
    self.sequences (sorted filenames, extended in order) -- lets the
    fit/test split partition by TRIAL (pkl file) instead of by individual
    overlapping window. Added 2026-08-20 per an independent review's
    finding: sequence_builder.py builds windows at stride 1 (10Hz over 6s
    windows), so adjacent windows within one trial overlap by up to 59/60
    frames -- a per-WINDOW random split scatters near-duplicates across
    fit and test, so the resulting "held-out" coverage check mostly
    verifies the quantile reproduces on data it was drawn from, not that
    it generalizes to an unseen drive. This mirrors the trial/goal-level
    split discipline run_pipeline.py's _train_cal_split() already applies
    one level up (TRAIN vs. CAL) -- reapplied here for FIT vs. TEST within
    CAL specifically."""
    pkl_files = sorted(f for f in os.listdir(data_folder) if f.endswith('.pkl'))
    ids = []
    for i, fname in enumerate(pkl_files):
        with open(os.path.join(data_folder, fname), 'rb') as f:
            n = len(pickle.load(f))
        ids.append(np.full(n, i))
    return np.concatenate(ids), pkl_files


def _load_model(model_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path} ({model.count_parameters():,} params)")
    return model, device


def _collect_residuals(model, device, loader):
    """One pass over `loader`. Returns {source_feature: (N, T_out) or
    (N, T_out, dims) array of raw (actual - predicted_mean) errors} --
    POINT prediction only (the `_mean` output), scale/dof/dof_reg never
    touched. Reduction into a per-series residual magnitude (L2-pooled or
    single-axis, per _SERIES) happens in _extract_series, not here, so one
    pass over the data serves every series."""
    out = {k: [] for k in _FEATURES}
    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            preds  = model(past, graph)

            for key, dims in _FEATURES.items():
                mean   = preds[f'{key}_mean']       # (B, T_out) or (B, T_out, dims)
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)
                out[key].append((actual - mean).cpu().numpy())

    return {k: np.concatenate(v, axis=0) for k, v in out.items()}   # (N, T_out[, dims])


def _extract_series(errors: dict) -> dict:
    """errors: {source_feature: (N, T_out[, dims]) raw error array} (as
    returned by _collect_residuals) -> {report_key: (N, T_out) residual
    MAGNITUDE array}, per _SERIES's mode ('l2' pools all dims, ('axis', i)
    takes one dim, 'scalar' takes the array as-is)."""
    out = {}
    for report_key, source_key, mode in _SERIES:
        err = errors[source_key]
        if mode == 'l2':
            resid = np.linalg.norm(err, axis=-1)
        elif mode == 'scalar':
            resid = np.abs(err)
        else:
            _, axis = mode
            resid = np.abs(err[..., axis])
        out[report_key] = resid
    return out


def _rmse_by_step(residuals: np.ndarray) -> np.ndarray:
    """residuals: (N, T_out) raw (unsquared) magnitudes -> RMSE per step,
    matching plot_calibration_diagrams.py's horizon_widening.png metric
    exactly, for direct comparability."""
    return np.sqrt((residuals ** 2).mean(axis=0))


def plot_comparison(fit_q, test_rmse, out_path, alpha):
    T_out = fit_q[next(iter(fit_q))].shape[0]
    steps = np.arange(T_out) * 0.1
    keys = list(fit_q.keys())
    ncols = 3
    nrows = math.ceil(len(keys) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(14, 4.3 * nrows))
    for ax, key in zip(axes.flat, keys):
        ax.plot(steps, fit_q[key], color='#2ca02c', linewidth=2, linestyle='--',
                label=f'conformal interval half-width (target {100*(1-alpha):.0f}% coverage)')
        ax.plot(steps, test_rmse[key], color='#1f77b4', linewidth=2,
                label='actual RMSE (held-out test half)')
        ax.set_title(key, fontsize=10)
        ax.set_xlabel('seconds into predicted horizon', fontsize=8)
        ax.tick_params(labelsize=7)
    for ax in axes.flat[len(keys):]:
        ax.axis('off')
    axes.flat[0].legend(loc='upper left', fontsize=7)
    fig.suptitle('Conformal calibration — interval width vs. actual error growth '
                 '(point predictor only, no distributional head)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--alpha', type=float, default=0.1, help='miscoverage rate (default 0.1 -> 90% target coverage)')
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    model, device = _load_model(args.model)

    ds = TrajectoryDataset(cfg.CAL_DIR)
    n = len(ds)
    trial_id, pkl_files = _trial_ids_for(cfg.CAL_DIR)
    assert len(trial_id) == n, f"trial_id length {len(trial_id)} != dataset length {n} -- CAL_DIR changed between calls?"
    n_trials = len(pkl_files)
    print(f"Calibration set: {n_trials} trials, {n} windows -- leave-one-trial-out cross-conformal "
          f"(2026-08-20, replaces the original fixed 50/50 split: with only {n_trials} trials, a single "
          f"fixed fit/test split gives a noisy, low-power coverage estimate -- see the research note)")

    # One model forward pass over the WHOLE calibration set -- predictions
    # don't depend on which fold a sample is in, only the quantile-fit/
    # coverage-check split does, so this is both cheaper AND simpler than
    # re-running inference per fold.
    full_loader = DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=0)
    print("Collecting residuals (full calibration set, one pass)...")
    all_resid = _extract_series(_collect_residuals(model, device, full_loader))

    T_out = cfg.OUTPUT_SEQ_LEN
    series_keys = [s[0] for s in _SERIES]

    # Leave-one-trial-out: for each of the n_trials folds, fit the
    # per-step quantile on the OTHER n_trials-1 trials' residuals and check
    # coverage on the held-out trial's own residuals. Every trial serves as
    # test exactly once, so pooling every fold's held-out coverage
    # indicators together uses the full dataset's worth of test evidence
    # (the standard cross-conformal / CV+ aggregate, Barber et al.) while
    # every single check point is still genuinely out-of-fold.
    fold_cov  = {k: np.zeros((n_trials, T_out)) for k in series_keys}   # per-fold coverage, for spread/stability
    fold_q    = {k: np.zeros((n_trials, T_out)) for k in series_keys}   # per-fold quantile, averaged for the plot
    pooled_covered = {k: [[] for _ in range(T_out)] for k in series_keys}   # pooled out-of-fold indicators
    pooled_rmse_sq = {k: np.zeros(T_out) for k in series_keys}
    pooled_n       = {k: np.zeros(T_out) for k in series_keys}

    for fold, held_out in enumerate(range(n_trials)):
        fit_mask  = trial_id != held_out
        test_mask = trial_id == held_out
        for key in series_keys:
            resid = all_resid[key]
            q = np.array([conformal_quantile(resid[fit_mask, t], args.alpha) for t in range(T_out)])
            fold_q[key][fold] = q
            test_resid = resid[test_mask]
            covered = test_resid <= q[np.newaxis, :]        # (n_test_this_fold, T_out)
            fold_cov[key][fold] = covered.mean(axis=0)
            for t in range(T_out):
                pooled_covered[key][t].append(covered[:, t])
            pooled_rmse_sq[key] += (test_resid ** 2).sum(axis=0)
            pooled_n[key]       += test_resid.shape[0]
        print(f"  fold {fold+1}/{n_trials} (held out {pkl_files[held_out]}, "
              f"{test_mask.sum()} windows) done")

    pooled_cov  = {k: np.array([np.concatenate(pooled_covered[k][t]).mean() for t in range(T_out)])
                   for k in series_keys}
    pooled_rmse = {k: np.sqrt(pooled_rmse_sq[k] / pooled_n[k]) for k in series_keys}
    mean_q      = {k: fold_q[k].mean(axis=0) for k in series_keys}   # representative interval width for the plot

    print(f"\n{'feature':<26}{'pooled cov @t=0':>16}{'@t=1.5s':>10}{'@t=3s':>8}{'  fold spread (min-max, @t=0)':>32}"
          f"   (target {100*(1-args.alpha):.0f}%)")
    print('-' * 100)
    rows = []
    for key in series_keys:
        c0, c15, c29 = pooled_cov[key][0], pooled_cov[key][14], pooled_cov[key][29]
        spread = f"{fold_cov[key][:, 0].min():.2f}-{fold_cov[key][:, 0].max():.2f}"
        print(f"{key:<26}{c0:16.3f}{c15:10.3f}{c29:8.3f}{spread:>32}")
        rows.append({
            'feature': key,
            'pooled_coverage_by_step': pooled_cov[key].tolist(),
            'per_fold_coverage_by_step': fold_cov[key].tolist(),   # (n_trials, T_out)
            'pooled_rmse_by_step': pooled_rmse[key].tolist(),
            'mean_fold_quantile_by_step': mean_q[key].tolist(),   # avg across folds, what the plot shows
        })

    report_path = os.path.join(args.output_dir, 'conformal_report.json')
    with open(report_path, 'w') as f:
        json.dump({
            'alpha': args.alpha, 'method': 'leave_one_trial_out_cross_conformal',
            'n_trials': n_trials, 'n_windows': n, 'trials': pkl_files, 'features': rows,
        }, f, indent=2)
    print(f'\nSaved {report_path}')

    plot_comparison(mean_q, pooled_rmse, os.path.join(args.output_dir, 'conformal_vs_actual.png'), args.alpha)


if __name__ == '__main__':
    main()
