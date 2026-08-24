#!/usr/bin/env python3
"""
Side-by-side comparison of the two Layer-1 (calibrated confidence)
approaches tried this project: (1) the paused jointly-trained Student-t/NLL
heads, (2) the current conformal calibration on a plain point predictor.
See docs/research_notes/nll_calibration_arc_and_conformal_pivot_2026-08-20.md
for the full history.

Approach 1's checkpoint (st_gat/checkpoints/h30_30/best_model.pth, the
7525b00 commit, still the last fully-trained Student-t model on disk) can
no longer be loaded by the CURRENT st_gat/model/model.py -- the 2026-08-19
per-head-horizon-embedding fix changed the model's architecture (state
dict keys don't match: horizon_embed.weight vs. horizon_embed.<feature>.weight).
This script loads that checkpoint using the model CLASS as it existed at
that commit (extracted via `git show 7525b00:st_gat/model/model.py`,
imported dynamically -- NOT by reverting the live model.py, which stays on
the current, fixed architecture), so approach 1 can be fairly evaluated
without losing today's architecture fix. See the module docstring's
"provenance" print at the top of main() for exactly what was extracted.

Reuses plot_calibration_diagrams.py's existing PIT-based diagnostic
functions directly (they operate on a model's forward() output dict, not
on which class produced it) rather than reimplementing them.

Usage (repo venv sourced; no ROS needed):
  source .venv/bin/activate
  python3 experiments/scripts/compare_layer1_approaches.py
"""

import importlib.util
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
sys.path.insert(0, SCRIPT_DIR)

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import TrajectoryDataset  # noqa: E402
import plot_calibration_diagrams as pcd  # noqa: E402 -- reused for its PIT/coverage/horizon-widening functions

OLD_MODEL_SRC = '/tmp/stgat_old_model_eval/old_model.py'   # git show 7525b00:st_gat/model/model.py
APPROACH1_CHECKPOINT = os.path.join(REPO_DIR, 'st_gat', 'checkpoints', 'h30_30', 'best_model.pth')
APPROACH2_CHECKPOINT = os.path.join(REPO_DIR, 'st_gat', 'checkpoints', 'h30_30_pointpred_v1', 'mean_warmup.pth')
OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'layer1_approach_comparison')


def _load_old_stgat_class():
    if not os.path.exists(OLD_MODEL_SRC):
        raise SystemExit(f"Run: git show 7525b00:st_gat/model/model.py > {OLD_MODEL_SRC}")
    spec = importlib.util.spec_from_file_location('old_model_7525b00', OLD_MODEL_SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.STGAT


def _point_accuracy(model, device, loader, is_old_arch: bool) -> dict:
    """Mean |actual - predicted_mean| per feature (raw, normalized units --
    same convention as trainer.py's position_l2_raw/velocity_l1_raw), plus
    the same for every other feature this model predicts."""
    T_out = cfg.OUTPUT_SEQ_LEN
    feats = {'position': 2, 'velocity': 2, 'steering': 1, 'acceleration': 1,
              'traffic_light_color': 1, 'traffic_light_confidence': 1}
    err_sum = {k: 0.0 for k in feats}
    n = 0
    with torch.no_grad():
        for past, future, graph, _b in loader:
            past  = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph = {k: v.to(device) for k, v in graph.items()}
            preds = model(past, graph)
            b = next(iter(future.values())).size(0)
            n += b
            for key, dims in feats.items():
                mean = preds[f'{key}_mean']
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)
                err = (actual - mean)
                mag = err.norm(dim=-1) if dims > 1 else err.abs()
                err_sum[key] += mag.sum().item()
    return {k: v / (n * T_out) for k, v in err_sum.items()}


def main():
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    ds = TrajectoryDataset(cfg.CAL_DIR)
    loader = DataLoader(ds, batch_size=256, shuffle=False, num_workers=0)

    # ── Approach 1: Student-t/NLL, old architecture, extracted class ──────
    print("=== Approach 1: Student-t/NLL joint head (h30_30/best_model.pth, commit 7525b00) ===")
    OldSTGAT = _load_old_stgat_class()
    model_cfg = cfg.build_inference_model_cfg(device)
    model1 = OldSTGAT(model_cfg).to(device)
    model1.load_state_dict(torch.load(APPROACH1_CHECKPOINT, map_location=device, weights_only=True))
    model1.eval()
    print(f"Loaded ({model1.count_parameters():,} params)")
    acc1 = _point_accuracy(model1, device, loader, is_old_arch=True)
    print("Point accuracy (mean |error|, normalized units):", acc1)

    print("\nRunning PIT-based calibration diagnostics (reusing plot_calibration_diagrams.py)...")
    u_by_feature, horizon, tl_logits, tl_labels = pcd._collect_calibration_stats(model1, device, batch_size=256)
    pit_stats = pcd.report_pit_uniformity_stats(u_by_feature, os.path.join(OUTPUT_DIR, 'approach1_pit_uniformity_stats.json'))
    pcd.plot_coverage_curves(u_by_feature, os.path.join(OUTPUT_DIR, 'approach1_coverage_curves.png'))
    pcd.plot_pit_histograms(u_by_feature, os.path.join(OUTPUT_DIR, 'approach1_pit_histograms.png'))
    pcd.plot_horizon_widening(horizon, os.path.join(OUTPUT_DIR, 'approach1_horizon_widening.png'))

    del model1
    if device.type == 'cuda':
        torch.cuda.empty_cache()

    # ── Approach 2: point predictor + conformal ────────────────────────────
    print("\n=== Approach 2: point predictor + conformal (h30_30_pointpred_v1) ===")
    from st_gat.model import STGAT
    model2 = STGAT(model_cfg).to(device)
    model2.load_state_dict(torch.load(APPROACH2_CHECKPOINT, map_location=device, weights_only=True))
    model2.eval()
    print(f"Loaded ({model2.count_parameters():,} params)")
    acc2 = _point_accuracy(model2, device, loader, is_old_arch=False)
    print("Point accuracy (mean |error|, normalized units):", acc2)
    print("\n(Calibration for approach 2 already computed via leave-one-trial-out cross-conformal --")
    print(" see experiments/analysis/conformal_horizon_calibration/{conformal_report.json,reliability_diagram.png,conformal_vs_actual.png}")
    print(" not re-run here; this script only adds the point-accuracy comparison + approach 1's fresh diagnostics.)")

    # ── Comparison table ────────────────────────────────────────────────────
    print(f"\n{'feature':<26}{'approach1 err':>16}{'approach2 err':>16}{'change':>10}")
    print('-' * 68)
    rows = []
    for key in acc1:
        a1, a2 = acc1[key], acc2[key]
        pct = 100 * (a2 - a1) / a1
        print(f"{key:<26}{a1:16.5f}{a2:16.5f}{pct:9.1f}%")
        rows.append({'feature': key, 'approach1_point_error': a1, 'approach2_point_error': a2,
                     'pct_change': pct})

    with open(os.path.join(OUTPUT_DIR, 'point_accuracy_comparison.json'), 'w') as f:
        json.dump({'approach1_checkpoint': APPROACH1_CHECKPOINT, 'approach2_checkpoint': APPROACH2_CHECKPOINT,
                    'rows': rows, 'approach1_pit_uniformity': pit_stats}, f, indent=2)
    print(f"\nSaved {OUTPUT_DIR}/point_accuracy_comparison.json")


if __name__ == '__main__':
    main()
