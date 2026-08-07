#!/usr/bin/env python3
"""
Visual calibration diagrams for the trained ST-GAT model — the actual
deliverable for step 1 of the 2026-08-06 reframe (see
docs/research_notes/ablation_study_2026.md and the memory this session
was briefed against): grounding the belief-divergence mechanism's own
credibility, not detection performance. check_calibration.py and
calibrate_tl_discrepancy.py already compute the right numbers on the
held-out CALIBRATION split; this script is those same numbers turned into
figures a reader (or Kalpit's committee) can actually look at and judge,
instead of a table of std(z) values.

REDESIGNED 2026-08-06, second pass (Gaussian -> Student-t heads): the
first version of this script used z = (actual-mean)/std checked against
N(0,1) — the coverage-curve/histogram/kurtosis results it produced are
exactly what proved that assumption wrong (leptokurtic residuals,
Anderson-Darling decisively rejects normality for all 6 features). Now
that model.py's heads predict a per-sample degrees-of-freedom alongside
mean/scale, there's no longer one fixed reference distribution to check a
raw z-score against — the correct generalization is the PROBABILITY
INTEGRAL TRANSFORM (PIT): u = F(actual; predicted params) using each
sample's OWN predicted Student-t CDF. A correctly-calibrated model gives
u ~ Uniform(0,1) regardless of family or how much the predicted dof varies
sample-to-sample (Gneiting et al., JRSS-B 2007) — this is what every plot
below is built on now, in place of the old N(0,1) comparison.

Produces one JSON report and four figures in --output-dir (default
experiments/analysis/calibration_diagrams/):

0. pit_uniformity_stats.json -- mean(u)/std(u) and a Kolmogorov-Smirnov
   test against Uniform(0,1) per Student-t-headed feature. The
   quantitative backing for whether the Student-t redesign actually fixed
   the shape mismatch the old Anderson-Darling numbers found — a
   well-calibrated model should NOT reject uniformity (large KS p-value).
1. coverage_curves.png — for each Student-t-headed feature (position,
   velocity, steering, acceleration, traffic_light_color,
   traffic_light_confidence), nominal confidence level p (x) vs. empirical
   coverage (y): what fraction of actual values fell inside the model's
   own p-confidence predicted interval, computed via PIT (u in
   [(1-p)/2, (1+p)/2]) rather than a Gaussian z-critical-value lookup — so
   this is valid regardless of the predicted dof.
2. pit_histograms.png — histogram of u per feature against the flat
   Uniform(0,1) density (replaces the old z-score-vs-N(0,1) histogram).
3. horizon_widening.png — does the model's own predicted uncertainty widen
   sensibly with prediction horizon, in a way that tracks how the actual
   error grows? Per feature: predicted std (now the Student-t IMPLIED std,
   sqrt(scale^2 * dof/(dof-2)), not the raw scale parameter) vs. actual
   RMSE (ground truth), both as a function of horizon step 0..T_out-1.
4. tl_discrepancy_reliability.png — the one Bernoulli head
   (traffic_light_discrepancy, unaffected by the Gaussian->Student-t
   change): mean predicted probability vs. mean actual frequency per bin,
   raw sigmoid output vs. temperature-scaled, both against the y=x
   diagonal. Reuses calibrate_tl_discrepancy.py's own
   reliability_bins()/temperature.json.

Usage (ROS/Autoware + repo venv sourced):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/plot_calibration_diagrams.py
"""

import argparse
import json
import os
import sys

import numpy as np
import torch
from scipy.stats import t as student_t, kstest
from torch.utils.data import DataLoader

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)
sys.path.insert(0, SCRIPT_DIR)   # for calibrate_tl_discrepancy import below

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from calibrate_tl_discrepancy import reliability_bins  # noqa: E402

DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'calibration_diagrams')

# (key, dims) — same Student-t-headed features check_calibration.py tracks.
_STUDENT_T_FEATURES = {
    'position':                  2,
    'velocity':                  2,
    'steering':                  1,
    'acceleration':               1,
    'traffic_light_color':       1,
    'traffic_light_confidence':  1,
}

_COVERAGE_LEVELS = np.linspace(0.05, 0.99, 40)


def _load_model(model_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.MODEL_CONFIG.copy()
    model_cfg.update({'d_model': 128, 'd_graph': 128, 'hidden_size': 128,
                      'num_layers': 2, 'nhead': 4, 'dropout_rate': 0.15, 'device': device})
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path} ({model.count_parameters():,} params)")
    return model, device


def _collect_calibration_stats(model, device, batch_size):
    """One pass over the calibration split. Returns:
    - u_by_feature: {key -> flat PIT array at t=0 (1-step-ahead)}
    - horizon: {key -> (implied_std_by_step (T_out,), actual_rmse_by_step (T_out,))}
    - tl_logits, tl_labels: raw arrays for the Bernoulli reliability diagram
    """
    ds = TrajectoryDataset(cfg.CAL_DIR)
    loader = DataLoader(ds, batch_size=batch_size, shuffle=False, num_workers=0, pin_memory=True)

    u_1step = {k: [] for k in _STUDENT_T_FEATURES}
    T_out = cfg.OUTPUT_SEQ_LEN
    sq_err_sum        = {k: np.zeros(T_out) for k in _STUDENT_T_FEATURES}
    implied_var_sum   = {k: np.zeros(T_out) for k in _STUDENT_T_FEATURES}
    n_count           = {k: np.zeros(T_out) for k in _STUDENT_T_FEATURES}

    tl_logits, tl_labels = [], []

    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            preds  = model(past, graph)

            for key, dims in _STUDENT_T_FEATURES.items():
                mean  = preds[f'{key}_mean']    # (B, T_out) or (B, T_out, dims)
                scale = preds[f'{key}_scale']
                dof   = preds[f'{key}_dof']      # (B, T_out) always
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)

                dof_b = dof.unsqueeze(-1) if mean.dim() == 3 else dof
                dof_full = dof_b.expand_as(mean)

                z = ((actual - mean) / scale).cpu().numpy()
                u = student_t.cdf(z, df=dof_full.cpu().numpy())
                # dims>1 (position/velocity): FLATTEN across dims, don't
                # average -- each dim's PIT is independently ~Uniform(0,1)
                # under correct calibration, but the MEAN of two independent
                # Uniform(0,1) values is NOT Uniform(0,1) (it's triangular,
                # concentrated near 0.5, std ~0.204 not ~0.289) -- averaging
                # was a real bug in an earlier version of this script that
                # manufactured apparent miscalibration for exactly these two
                # features. Flattening (treating x and y as two separate
                # pooled observations, same convention the old z-score
                # version used) preserves the correct marginal property.
                u_1step[key].append(u[:, 0].flatten())

                # Horizon widening: implied Student-t variance
                # (scale^2 * dof/(dof-2)), averaged over dims and batch.
                sq_err = (actual - mean) ** 2
                implied_var = scale.pow(2) * dof_b / (dof_b - 2)
                if dims > 1:
                    sq_err = sq_err.mean(dim=-1)
                    implied_var = implied_var.mean(dim=-1) if implied_var.dim() == mean.dim() else implied_var
                sq_err_sum[key]      += sq_err.sum(dim=0).cpu().numpy()
                implied_var_sum[key] += implied_var.sum(dim=0).cpu().numpy()
                n_count[key]         += actual.size(0)

            logit = preds['traffic_light_discrepancy_logit'][:, 0]
            actual_tl = future['traffic_light_discrepancy'][:, 0]
            if actual_tl.dim() == 2 and actual_tl.size(-1) == 1:
                actual_tl = actual_tl.squeeze(-1)
            tl_logits.append(logit.cpu().numpy())
            tl_labels.append(actual_tl.cpu().numpy())

    u_by_feature = {k: np.concatenate(v) for k, v in u_1step.items()}
    for k in u_by_feature:
        u_by_feature[k] = u_by_feature[k][np.isfinite(u_by_feature[k])]

    horizon = {}
    for k in _STUDENT_T_FEATURES:
        actual_rmse = np.sqrt(sq_err_sum[k] / n_count[k])
        implied_std = np.sqrt(implied_var_sum[k] / n_count[k])
        horizon[k] = (implied_std, actual_rmse)

    return u_by_feature, horizon, np.concatenate(tl_logits), np.concatenate(tl_labels)


def report_pit_uniformity_stats(u_by_feature, out_path):
    """Numeric evidence for whether the Student-t redesign fixed the shape
    mismatch: mean(u)/std(u) (want 0.5 / 0.289, the Uniform(0,1) moments)
    and a Kolmogorov-Smirnov test against Uniform(0,1) (large p-value =
    fails to reject calibration, which is what we WANT here — the opposite
    framing from the old Anderson-Darling-vs-normal check, where rejecting
    was the finding). Printed and saved to JSON so the numbers are citable
    without re-running this script."""
    rows = []
    print(f"\n{'feature':<26}{'mean(u)':>12}{'std(u)':>12}{'KS stat':>12}{'KS p-value':>14}")
    print('-' * 76)
    for key, u in u_by_feature.items():
        ks = kstest(u, 'uniform')
        rows.append({'feature': key, 'mean_u': float(u.mean()), 'std_u': float(u.std()),
                     'ks_statistic': float(ks.statistic), 'ks_pvalue': float(ks.pvalue),
                     'n': int(len(u))})
        print(f"{key:<26}{u.mean():12.3f}{u.std():12.3f}{ks.statistic:12.3f}{ks.pvalue:14.2e}")
    with open(out_path, 'w') as f:
        json.dump(rows, f, indent=2)
    print(f'\nSaved {out_path}')
    return rows


def plot_coverage_curves(u_by_feature, out_path):
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.5))
    for ax, (key, u) in zip(axes.flat, u_by_feature.items()):
        lo = (1 - _COVERAGE_LEVELS) / 2
        hi = (1 + _COVERAGE_LEVELS) / 2
        empirical = np.array([np.mean((u >= l) & (u <= h)) for l, h in zip(lo, hi)])
        ax.plot([0, 1], [0, 1], color='#999999', linestyle='--', linewidth=1, label='perfect calibration', zorder=1)
        ax.plot(_COVERAGE_LEVELS, empirical, color='#1f77b4', linewidth=2, zorder=2)
        ax.fill_between(_COVERAGE_LEVELS, _COVERAGE_LEVELS, empirical, color='#1f77b4', alpha=0.12)
        ax.set_title(key, fontsize=10)
        ax.set_xlabel('nominal confidence level', fontsize=8)
        ax.set_ylabel('empirical coverage (via PIT)', fontsize=8)
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)
        ax.tick_params(labelsize=7)
        ax.set_aspect('equal')
        max_gap = float(np.max(np.abs(empirical - _COVERAGE_LEVELS)))
        ax.text(0.03, 0.93, f'max gap: {max_gap:.3f}', fontsize=7.5, transform=ax.transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.7, linewidth=0))
    axes.flat[0].legend(loc='lower right', fontsize=7)
    fig.suptitle('Coverage calibration (PIT-based) — nominal vs. empirical (calibration split, 1-step-ahead)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def plot_pit_histograms(u_by_feature, out_path):
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.5))
    for ax, (key, u) in zip(axes.flat, u_by_feature.items()):
        ax.hist(u, bins=40, range=(0, 1), density=True, color='#1f77b4', alpha=0.6, label='observed PIT')
        ax.axhline(1.0, color='#d62728', linewidth=1.8, label='Uniform(0,1)')
        ks = kstest(u, 'uniform')
        ax.set_title(f'{key}  (KS p={ks.pvalue:.2e}, n={len(u)})', fontsize=9.5)
        ax.set_xlim(0, 1)
        ax.tick_params(labelsize=7)
    axes.flat[0].legend(loc='upper right', fontsize=7)
    fig.suptitle('PIT distribution vs. Uniform(0,1) (calibration split, 1-step-ahead)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def plot_horizon_widening(horizon, out_path):
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.5))
    for ax, (key, (implied_std, actual_rmse)) in zip(axes.flat, horizon.items()):
        steps = np.arange(len(implied_std)) * 0.1  # 10Hz -> seconds
        ax.plot(steps, implied_std, color='#d62728', linewidth=2, linestyle='--',
                label='predicted std (Student-t implied, model\'s own claim)')
        ax.plot(steps, actual_rmse, color='#1f77b4', linewidth=2, label='actual RMSE (ground truth)')
        ax.set_title(key, fontsize=10)
        ax.set_xlabel('seconds into predicted horizon', fontsize=8)
        ax.tick_params(labelsize=7)
    axes.flat[0].legend(loc='upper left', fontsize=7)
    fig.suptitle('Horizon widening — does predicted uncertainty track actual error growth? (calibration split)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def plot_tl_reliability(tl_logits, tl_labels, temperature_file, out_path):
    T = 1.0
    if os.path.exists(temperature_file):
        with open(temperature_file) as f:
            T = json.load(f)['temperature']
    probs_raw = 1 / (1 + np.exp(-tl_logits))
    probs_cal = 1 / (1 + np.exp(-tl_logits / T))

    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.plot([0, 1], [0, 1], color='#999999', linestyle='--', linewidth=1, label='perfect calibration', zorder=1)
    for probs, color, label in [(probs_raw, '#d62728', f'raw (T=1.0)'),
                                  (probs_cal, '#1f77b4', f'temperature-scaled (T={T:.2f})')]:
        rows = reliability_bins(probs, tl_labels)
        if not rows:
            continue
        xs = [r['mean_pred_prob'] for r in rows]
        ys = [r['mean_actual_freq'] for r in rows]
        ns = [r['n'] for r in rows]
        sizes = 30 + 300 * np.array(ns) / max(ns)
        ax.plot(xs, ys, color=color, linewidth=1.5, alpha=0.7, zorder=2)
        ax.scatter(xs, ys, color=color, s=sizes, alpha=0.85, label=label, zorder=3, edgecolors='white', linewidths=0.5)
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.set_xlabel('mean predicted probability (per bin)')
    ax.set_ylabel('mean actual frequency (per bin)')
    ax.set_title('traffic_light_discrepancy reliability diagram\n(marker size = bin count, calibration split)', fontsize=11)
    ax.legend(loc='upper left', fontsize=9)
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    ap.add_argument('--temperature-file', default=os.path.join(
        REPO_DIR, 'experiments', 'analysis', 'tl_calibration', 'temperature.json'))
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    model, device = _load_model(args.model)
    u_by_feature, horizon, tl_logits, tl_labels = _collect_calibration_stats(model, device, args.batch)

    report_pit_uniformity_stats(u_by_feature, os.path.join(args.output_dir, 'pit_uniformity_stats.json'))
    plot_coverage_curves(u_by_feature, os.path.join(args.output_dir, 'coverage_curves.png'))
    plot_pit_histograms(u_by_feature, os.path.join(args.output_dir, 'pit_histograms.png'))
    plot_horizon_widening(horizon, os.path.join(args.output_dir, 'horizon_widening.png'))
    plot_tl_reliability(tl_logits, tl_labels, args.temperature_file,
                        os.path.join(args.output_dir, 'tl_discrepancy_reliability.png'))


if __name__ == '__main__':
    main()
