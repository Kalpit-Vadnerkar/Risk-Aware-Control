#!/usr/bin/env python3
"""
Diagnostic (2026-08-24): does the position head actually use route/map
context, or does it default to near-linear extrapolation of the observed
past heading regardless of the route ahead? Prompted by Kalpit noticing
window 3407/4523's position-map plots (experiments/analysis/
layer1_trust_examples/) show the predicted trajectory barely curving while
the actual trajectory takes a real turn.

Method: for every calibration window, compute (a) how much the ACTUAL
future path curves (heading change from first to last predicted step) and
(b) how much the PREDICTED mean path curves the same way, plus (c) the
mean position residual magnitude over the horizon. Bin windows by actual
curvature and report mean predicted curvature and mean residual per bin.
If the model has learned to use route context, predicted curvature should
track actual curvature and residual should stay roughly flat across bins.
If it's defaulting to linear extrapolation, predicted curvature will stay
near zero regardless of actual curvature, and residual will grow sharply
with actual curvature -- i.e. every real turn looks like an "anomaly" to
Layer 1's calibration even though it's normal driving.
"""

import os
import sys

import numpy as np
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402


def heading(vec):
    return np.arctan2(vec[1], vec[0])


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def main():
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model_path = os.path.join(REPO_DIR, 'st_gat', 'checkpoints', 'h30_30_pointpred_v1', 'mean_warmup.pth')
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path}")

    ds = TrajectoryDataset(cfg.CAL_DIR)
    n = len(ds)
    print(f"{n} calibration windows")

    rng = np.random.default_rng(0)
    idxs = rng.choice(n, size=min(1500, n), replace=False)

    actual_curv, pred_curv, resid_mean, resid_max, past_curv, disp_m = [], [], [], [], [], []
    K = 5  # average over K steps at each end to denoise the heading estimate

    def robust_turn_deg(xy, min_seg_m):
        start_vec = xy[K] - xy[0]
        end_vec   = xy[-1] - xy[-1 - K]
        if np.linalg.norm(start_vec) < min_seg_m or np.linalg.norm(end_vec) < min_seg_m:
            return None
        return np.degrees(abs(wrap(heading(end_vec) - heading(start_vec))))

    MIN_SEG_M = 0.5  # metres; below this a "heading" is noise, not signal

    with torch.no_grad():
        for idx in idxs:
            idx = int(idx)
            past, future, graph, _bounds = ds[idx]
            past_t  = {k: torch.as_tensor(v, device=device).unsqueeze(0) for k, v in past.items()}
            graph_t = {k: torch.as_tensor(v, device=device).unsqueeze(0) for k, v in graph.items()}
            preds = model(past_t, graph_t)

            mean = preds['position_mean'][0].cpu().numpy() * cfg.POSITION_DISPLACEMENT_RANGE_M
            actual = np.asarray(future['position']) * cfg.POSITION_DISPLACEMENT_RANGE_M
            past_pos = np.asarray(past['position']) * cfg.POSITION_DISPLACEMENT_RANGE_M

            act_turn = robust_turn_deg(actual, MIN_SEG_M)
            if act_turn is None:
                continue
            pred_turn = robust_turn_deg(mean, MIN_SEG_M)
            if pred_turn is None:
                pred_turn = 0.0
            p_turn = robust_turn_deg(past_pos, MIN_SEG_M)
            if p_turn is None:
                p_turn = 0.0

            total_disp = np.linalg.norm(actual[-1] - actual[0])
            resid = np.linalg.norm(mean - actual, axis=1)

            actual_curv.append(act_turn)
            pred_curv.append(pred_turn)
            past_curv.append(p_turn)
            resid_mean.append(resid.mean())
            resid_max.append(resid.max())
            disp_m.append(total_disp)

    actual_curv = np.array(actual_curv)
    pred_curv   = np.array(pred_curv)
    past_curv   = np.array(past_curv)
    resid_mean  = np.array(resid_mean)
    resid_max   = np.array(resid_max)

    print(f"\nUsable windows: {len(actual_curv)}")
    print(f"corr(actual_curvature, predicted_curvature) = {np.corrcoef(actual_curv, pred_curv)[0,1]:.3f}")
    print(f"corr(actual_curvature, mean_position_residual_m) = {np.corrcoef(actual_curv, resid_mean)[0,1]:.3f}")
    print(f"corr(actual_curvature, max_position_residual_m)  = {np.corrcoef(actual_curv, resid_max)[0,1]:.3f}")
    print(f"corr(past_curvature_already_observed, predicted_curvature) = {np.corrcoef(past_curv, pred_curv)[0,1]:.3f}")

    bins = np.percentile(actual_curv, [0, 20, 40, 60, 80, 100])
    print("\nBinned by ACTUAL future heading-change (deg), 5 quintiles:")
    print(f"{'bin (deg)':>18} {'n':>5} {'mean actual turn':>17} {'mean pred turn':>15} {'mean resid (m)':>15} {'max resid (m)':>14}")
    for i in range(5):
        lo, hi = bins[i], bins[i+1]
        mask = (actual_curv >= lo) & (actual_curv <= hi if i == 4 else actual_curv < hi)
        if mask.sum() == 0:
            continue
        print(f"[{lo:6.1f}, {hi:6.1f}] {mask.sum():5d} {actual_curv[mask].mean():17.2f} "
              f"{pred_curv[mask].mean():15.2f} {resid_mean[mask].mean():15.3f} {resid_max[mask].mean():14.3f}")

    # fraction of windows that are "real turns" at all (>15 deg heading change over 3s)
    turn_mask = actual_curv > 15.0
    print(f"\nWindows with >15 deg actual heading change over the 3s horizon: "
          f"{turn_mask.sum()} / {len(actual_curv)} ({100*turn_mask.mean():.1f}%)")
    if turn_mask.sum() > 0:
        print(f"  Of those, mean predicted heading change: {pred_curv[turn_mask].mean():.2f} deg "
              f"(vs. mean actual: {actual_curv[turn_mask].mean():.2f} deg)")
        print(f"  Of those, mean position residual: {resid_mean[turn_mask].mean():.3f} m "
              f"(vs. overall mean: {resid_mean.mean():.3f} m)")

    # The critical split: among real future turns, is the model only
    # extrapolating a turn ALREADY visible in the observed past (kinematic
    # continuation), or does it anticipate turns that haven't started yet
    # (which would require actually using route/map context)?
    already_turning = turn_mask & (past_curv > 10.0)
    not_yet_turning  = turn_mask & (past_curv <= 10.0)
    print(f"\nOf the {turn_mask.sum()} real future-turn windows:")
    print(f"  {already_turning.sum()} were ALREADY turning in the observed past (past turn >10 deg) — "
          f"mean predicted turn {pred_curv[already_turning].mean():.2f} deg vs actual {actual_curv[already_turning].mean():.2f} deg, "
          f"mean resid {resid_mean[already_turning].mean():.3f} m" if already_turning.sum() else "  0 were already turning in the observed past")
    print(f"  {not_yet_turning.sum()} had NOT started turning yet (past turn <=10 deg, i.e. the turn is genuinely upcoming) — "
          f"mean predicted turn {pred_curv[not_yet_turning].mean():.2f} deg vs actual {actual_curv[not_yet_turning].mean():.2f} deg, "
          f"mean resid {resid_mean[not_yet_turning].mean():.3f} m" if not_yet_turning.sum() else "  0 had not yet started turning")


if __name__ == '__main__':
    main()
