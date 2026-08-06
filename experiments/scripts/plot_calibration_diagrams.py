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

Produces one JSON report and four figures in --output-dir (default
experiments/analysis/calibration_diagrams/):

0. normality_stats.json -- excess kurtosis and an Anderson-Darling
   normality test per Gaussian-headed feature, printed and saved. The
   quantitative backing for the "coverage curves overshoot because the
   z-distribution is leptokurtic" claim -- std(z) alone can't tell "right
   variance, right shape" from "right variance, wrong shape" apart.

1. coverage_curves.png — for each Gaussian-headed feature (position,
   velocity, steering, acceleration, traffic_light_color,
   traffic_light_confidence), nominal confidence level p (x) vs. empirical
   coverage (y): what fraction of actual values actually fell inside the
   model's own p-confidence predicted interval. A well-calibrated model
   tracks the y=x diagonal. This is the real generalization of
   check_calibration.py's "frac|z|<1 / frac|z|<2" spot checks (p=0.68,
   p=0.95) to the full curve.
2. zscore_histograms.png — same features, raw z = (actual - mean) / std
   histograms overlaid with the standard normal density they should match
   if the model's predicted variance is honest.
3. horizon_widening.png — does the model's own predicted uncertainty widen
   sensibly with prediction horizon, in a way that tracks how the actual
   error grows? Per feature: predicted std (model's own claim) vs. actual
   RMSE (ground truth), both as a function of horizon step 0..T_out-1.
   This is a trust check that's easy to get structurally wrong (e.g. a
   model that just predicts a constant huge variance would "cover" fine at
   any one horizon step without actually tracking anything) — plotting
   both curves together is what exposes that failure mode if it's there.
4. tl_discrepancy_reliability.png — the one Bernoulli head
   (traffic_light_discrepancy): mean predicted probability vs. mean actual
   frequency per bin, raw sigmoid output vs. temperature-scaled, both
   against the y=x diagonal. Reuses calibrate_tl_discrepancy.py's own
   reliability_bins()/temperature.json so this is the same computation as
   that script's printed table, not a re-derivation that could drift from it.

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
from scipy.stats import norm, kurtosis, anderson
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

# (key, dims) — same Gaussian-headed features check_calibration.py tracks.
_GAUSSIAN_FEATURES = {
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
    - z_by_feature: {key -> flat z-score array at t=0 (1-step-ahead)}
    - horizon: {key -> (pred_std_by_step (T_out,), actual_rmse_by_step (T_out,))}
    - tl_logits, tl_labels: raw arrays for the Bernoulli reliability diagram
    """
    ds = TrajectoryDataset(cfg.CAL_DIR)
    loader = DataLoader(ds, batch_size=batch_size, shuffle=False, num_workers=0, pin_memory=True)

    z_1step = {k: [] for k in _GAUSSIAN_FEATURES}
    # Horizon accumulators: sum of squared z (for RMSE-of-z... no, we want
    # real-unit actual error and the model's own predicted std, both per
    # step) -- accumulate sum of squared residual and sum of predicted var,
    # per horizon step, then take sqrt/mean at the end (streaming, so this
    # doesn't need to hold every window in memory).
    T_out = cfg.OUTPUT_SEQ_LEN
    sq_err_sum   = {k: np.zeros(T_out) for k in _GAUSSIAN_FEATURES}
    pred_std_sum = {k: np.zeros(T_out) for k in _GAUSSIAN_FEATURES}
    n_count      = {k: np.zeros(T_out) for k in _GAUSSIAN_FEATURES}

    tl_logits, tl_labels = [], []

    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            preds  = model(past, graph)

            for key, dims in _GAUSSIAN_FEATURES.items():
                mean = preds[f'{key}_mean']    # (B, T_out) or (B, T_out, dims)
                var  = preds[f'{key}_var']
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)

                z = (actual - mean) / torch.sqrt(var)
                z_1step[key].append(z[:, 0].flatten().cpu().numpy())

                # Horizon widening: per-step, averaged over dims and batch.
                sq_err = (actual - mean) ** 2
                if dims > 1:
                    sq_err = sq_err.mean(dim=-1)
                    var_step = var.mean(dim=-1)
                else:
                    var_step = var
                sq_err_sum[key]   += sq_err.sum(dim=0).cpu().numpy()
                pred_std_sum[key] += torch.sqrt(var_step).sum(dim=0).cpu().numpy()
                n_count[key]      += actual.size(0)

            logit = preds['traffic_light_discrepancy_logit'][:, 0]
            actual_tl = future['traffic_light_discrepancy'][:, 0]
            if actual_tl.dim() == 2 and actual_tl.size(-1) == 1:
                actual_tl = actual_tl.squeeze(-1)
            tl_logits.append(logit.cpu().numpy())
            tl_labels.append(actual_tl.cpu().numpy())

    z_by_feature = {k: np.concatenate(v) for k, v in z_1step.items()}
    for k in z_by_feature:
        z_by_feature[k] = z_by_feature[k][np.isfinite(z_by_feature[k])]

    horizon = {}
    for k in _GAUSSIAN_FEATURES:
        actual_rmse = np.sqrt(sq_err_sum[k] / n_count[k])
        mean_pred_std = pred_std_sum[k] / n_count[k]
        horizon[k] = (mean_pred_std, actual_rmse)

    return z_by_feature, horizon, np.concatenate(tl_logits), np.concatenate(tl_labels)


def report_normality_stats(z_by_feature, out_path):
    """Numeric evidence for the shape claim the coverage curves/histograms
    show visually: excess kurtosis (0 for a true Gaussian; the observed
    values here are large and positive, i.e. leptokurtic -- more mass near
    zero AND heavier tails than a Gaussian of the same variance) and the
    Anderson-Darling statistic against N(0,1) (a standard normality test --
    the AD statistic exceeding its 5% critical value rejects normality;
    check_calibration.py's std(z) alone cannot distinguish "correct
    variance, correct shape" from "correct variance, wrong shape", which is
    exactly the failure mode found here -- see
    docs/research_notes/trust_and_signal_behavior_2026-08-06.md). Printed
    and saved to JSON so the exact numbers are citable without re-running
    this script.
    """
    rows = []
    print(f"\n{'feature':<26}{'excess kurtosis':>18}{'AD stat':>14}{'AD crit@5%':>14}")
    print('-' * 72)
    for key, z in z_by_feature.items():
        ek = float(kurtosis(z, fisher=True))
        ad = anderson(z, dist='norm')
        crit5 = float(ad.critical_values[2])   # index 2 is the 5% significance level for dist='norm'
        rows.append({'feature': key, 'excess_kurtosis': ek, 'anderson_darling_stat': float(ad.statistic),
                     'anderson_darling_crit_5pct': crit5, 'n': int(len(z))})
        print(f"{key:<26}{ek:18.3f}{ad.statistic:14.3f}{crit5:14.3f}")
    with open(out_path, 'w') as f:
        json.dump(rows, f, indent=2)
    print(f'\nSaved {out_path}')
    return rows


def plot_coverage_curves(z_by_feature, out_path):
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.5))
    for ax, (key, z) in zip(axes.flat, z_by_feature.items()):
        z_crit = norm.ppf(0.5 + _COVERAGE_LEVELS / 2)   # two-sided critical z per nominal level
        empirical = np.array([np.mean(np.abs(z) <= zc) for zc in z_crit])
        ax.plot([0, 1], [0, 1], color='#999999', linestyle='--', linewidth=1, label='perfect calibration', zorder=1)
        ax.plot(_COVERAGE_LEVELS, empirical, color='#1f77b4', linewidth=2, zorder=2)
        ax.fill_between(_COVERAGE_LEVELS, _COVERAGE_LEVELS, empirical, color='#1f77b4', alpha=0.12)
        ax.set_title(key, fontsize=10)
        ax.set_xlabel('nominal confidence level', fontsize=8)
        ax.set_ylabel('empirical coverage', fontsize=8)
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)
        ax.tick_params(labelsize=7)
        ax.set_aspect('equal')
        max_gap = float(np.max(np.abs(empirical - _COVERAGE_LEVELS)))
        ax.text(0.03, 0.93, f'max gap: {max_gap:.3f}', fontsize=7.5, transform=ax.transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.7, linewidth=0))
    axes.flat[0].legend(loc='lower right', fontsize=7)
    fig.suptitle('Coverage calibration — nominal vs. empirical (calibration split, 1-step-ahead)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def plot_zscore_histograms(z_by_feature, out_path):
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.5))
    xs = np.linspace(-4, 4, 400)
    for ax, (key, z) in zip(axes.flat, z_by_feature.items()):
        z_clipped = np.clip(z, -6, 6)
        ax.hist(z_clipped, bins=60, density=True, color='#1f77b4', alpha=0.55, label='observed z')
        ax.plot(xs, norm.pdf(xs), color='#d62728', linewidth=1.8, label='N(0,1)')
        ax.set_title(f'{key}  (std={z.std():.2f}, n={len(z)})', fontsize=9.5)
        ax.set_xlim(-4, 4)
        ax.tick_params(labelsize=7)
    axes.flat[0].legend(loc='upper right', fontsize=7)
    fig.suptitle('z-score distribution vs. standard normal (calibration split, 1-step-ahead)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def plot_horizon_widening(horizon, out_path):
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.5))
    for ax, (key, (pred_std, actual_rmse)) in zip(axes.flat, horizon.items()):
        steps = np.arange(len(pred_std)) * 0.1  # 10Hz -> seconds
        ax.plot(steps, pred_std, color='#d62728', linewidth=2, linestyle='--', label='predicted std (model\'s own claim)')
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
    z_by_feature, horizon, tl_logits, tl_labels = _collect_calibration_stats(model, device, args.batch)

    report_normality_stats(z_by_feature, os.path.join(args.output_dir, 'normality_stats.json'))
    plot_coverage_curves(z_by_feature, os.path.join(args.output_dir, 'coverage_curves.png'))
    plot_zscore_histograms(z_by_feature, os.path.join(args.output_dir, 'zscore_histograms.png'))
    plot_horizon_widening(horizon, os.path.join(args.output_dir, 'horizon_widening.png'))
    plot_tl_reliability(tl_logits, tl_labels, args.temperature_file,
                        os.path.join(args.output_dir, 'tl_discrepancy_reliability.png'))


if __name__ == '__main__':
    main()
