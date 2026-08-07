#!/usr/bin/env python3
"""
Temperature scaling (Guo, Pleiss, Sun & Weinberger, "On Calibration of Modern
Neural Networks," ICML 2017 — docs/papers/2017_guo_temperature_scaling_calibration.pdf)
for the traffic_light_discrepancy Bernoulli head.

Why this head specifically needs its own calibration step (see
docs/research_notes/ablation_study_2026.md §4): it's a sigmoid/BCE head, not
a Gaussian mean/variance head — there's no notion of a predicted confidence
interval the way position/velocity have one. Running its raw residual
through the same continuous conformal-threshold machinery built for the
Gaussian features (as conformal_lead_time.py did) tests the wrong thing:
whether the residual magnitude clears an arbitrary bar, not whether the
predicted PROBABILITY is itself trustworthy (does "0.7" really mean 70%).
Confirmed empirically (2026-08-05): the frame-gap retrain measurably
improved every Gaussian-headed feature's calibration but left
traffic_light_discrepancy's conformal detection at 0/8 fault trials,
unchanged — a structural mismatch, not a training-noise problem.

Temperature scaling is the cheapest fix: a single learned scalar T fit by
minimizing BCE of sigmoid(logit / T) against true labels on the held-out
calibration split (never seen in training), no retraining needed. Requires
the model to expose the pre-sigmoid logit (added to STGAT.forward()'s output
dict 2026-08-05 as 'traffic_light_discrepancy_logit', alongside the existing
post-sigmoid 'traffic_light_discrepancy' key which training/loss still use
unchanged).

Usage (ROS/Autoware + repo venv sourced):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/calibrate_tl_discrepancy.py
"""

import argparse
import json
import os
import sys

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402

DEFAULT_OUTPUT = os.path.join(REPO_DIR, 'experiments', 'analysis', 'tl_calibration', 'temperature.json')
# T grid: 1.0 = no change; >1 flattens toward 0.5 (fixes overconfidence);
# <1 sharpens toward 0/1 (fixes underconfidence). Guo et al.'s own reported
# fitted temperatures for miscalibrated nets are mostly in [1, 5].
T_GRID = np.concatenate([np.arange(0.2, 1.0, 0.05), np.arange(1.0, 5.05, 0.1)])


def reliability_bins(probs: np.ndarray, labels: np.ndarray, n_bins: int = 10):
    """(bin center, mean predicted prob, mean actual frequency, count) per bin
    — the standard reliability-diagram table, reported as numbers instead of
    a plot since the point here is the BCE/ECE summary, not visual review."""
    edges = np.linspace(0, 1, n_bins + 1)
    rows = []
    for i in range(n_bins):
        lo, hi = edges[i], edges[i + 1]
        mask = (probs >= lo) & (probs < hi if i < n_bins - 1 else probs <= hi)
        if mask.sum() == 0:
            continue
        rows.append({
            'bin': f'[{lo:.1f}, {hi:.1f})', 'n': int(mask.sum()),
            'mean_pred_prob': float(probs[mask].mean()),
            'mean_actual_freq': float(labels[mask].mean()),
        })
    return rows


def expected_calibration_error(probs: np.ndarray, labels: np.ndarray, n_bins: int = 10) -> float:
    rows = reliability_bins(probs, labels, n_bins)
    n_total = sum(r['n'] for r in rows)
    return sum(r['n'] / n_total * abs(r['mean_pred_prob'] - r['mean_actual_freq']) for r in rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--output', default=DEFAULT_OUTPUT)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(args.model, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {args.model} ({model.count_parameters():,} params)")

    ds = TrajectoryDataset(cfg.CAL_DIR)
    loader = DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=0, pin_memory=True)

    all_logits, all_labels = [], []
    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            preds  = model(past, graph)

            logit = preds['traffic_light_discrepancy_logit'][:, 0]   # 1-step-ahead, t=0
            actual = future['traffic_light_discrepancy'][:, 0]
            if actual.dim() == 2 and actual.size(-1) == 1:
                actual = actual.squeeze(-1)
            all_logits.append(logit.cpu().numpy())
            all_labels.append(actual.cpu().numpy())

    logits = np.concatenate(all_logits)
    labels = np.concatenate(all_labels)
    print(f"\n{len(logits)} calibration examples, {labels.mean():.4f} positive rate")

    logits_t = torch.from_numpy(logits)
    labels_t = torch.from_numpy(labels)

    def bce_at(T: float) -> float:
        probs = torch.sigmoid(logits_t / T)
        return F.binary_cross_entropy(probs, labels_t).item()

    bce_by_T = [(T, bce_at(T)) for T in T_GRID]
    best_T, best_bce = min(bce_by_T, key=lambda x: x[1])
    base_bce = bce_at(1.0)

    probs_before = torch.sigmoid(logits_t).numpy()
    probs_after  = torch.sigmoid(logits_t / best_T).numpy()
    ece_before = expected_calibration_error(probs_before, labels)
    ece_after  = expected_calibration_error(probs_after, labels)

    print(f"\nBCE at T=1.0 (uncalibrated): {base_bce:.4f}")
    print(f"Best T: {best_T:.2f}  ->  BCE: {best_bce:.4f}")
    print(f"ECE (10-bin) before: {ece_before:.4f}   after: {ece_after:.4f}")

    print("\n=== Reliability bins, T=1.0 (uncalibrated) ===")
    for r in reliability_bins(probs_before, labels):
        print(f"  {r['bin']:>12}  n={r['n']:6d}  mean_pred={r['mean_pred_prob']:.3f}  "
              f"mean_actual={r['mean_actual_freq']:.3f}")

    print(f"\n=== Reliability bins, T={best_T:.2f} (calibrated) ===")
    for r in reliability_bins(probs_after, labels):
        print(f"  {r['bin']:>12}  n={r['n']:6d}  mean_pred={r['mean_pred_prob']:.3f}  "
              f"mean_actual={r['mean_actual_freq']:.3f}")

    with open(args.output, 'w') as f:
        json.dump({
            'temperature': float(best_T),
            'bce_before': base_bce, 'bce_after': best_bce,
            'ece_before': ece_before, 'ece_after': ece_after,
            'n_calibration_examples': int(len(logits)),
            'positive_rate': float(labels.mean()),
        }, f, indent=2)
    print(f"\nSaved {args.output}")


if __name__ == '__main__':
    main()
