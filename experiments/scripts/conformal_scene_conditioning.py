#!/usr/bin/env python3
"""
Scene-embedding-conditioned conformal quantile (2026-08-24) -- the fix
Kalpit chose for the turn-anticipation gap found the same day (see
diagnose_turn_learning.py, CLAUDE.md's "Turn-anticipation gap found"
entry): the position head only partially anticipates turns from route/map
context, so ordinary turns produce systematically larger residuals than
calm driving. Vanilla conformal (conformal_horizon_calibration.py) pools
every window into ONE global per-step quantile, so it either stays wide
everywhere to remain valid on turns, or a real fault occurring near/during
a turn has its residual conflated with the turn's own residual. This was
already proposed in principle in docs/research_notes/
open_world_safety_reframe_2026-08-20.md §5 ("condition on the model's own
learned scene embedding... GraphEncoder's attention-pooled graph context
(or h_last after the LSTM)") -- this script is the first implementation.

Method: for each test window, take the model's own h_last (STGAT's shared
trunk output every head is conditioned on -- see model.py's encode_scene(),
factored out today specifically so this script doesn't need to duplicate
trunk logic), find its k nearest neighbors by Euclidean distance in
h_last-space among the FIT set (other trials only -- no leakage, same
leave-one-trial-out discipline as the vanilla script), and fit the
conformal quantile on just that neighborhood's residuals instead of the
whole fit set. This is a sample-selection rule (plumbing), not a new
statistical method -- see the reframe note for why this framing was
deliberately chosen over hand-labeled scenario tags.

Validates the hypothesis directly: does scene-conditioning widen the
interval specifically for turn-like windows (fixing the masking risk)
while tightening it for calm windows (recovering some of the slack vanilla
conformal wastes everywhere to stay valid on turns)?

Usage (repo venv sourced, no ROS needed):
  source .venv/bin/activate
  python3 experiments/scripts/conformal_scene_conditioning.py \
      --model st_gat/checkpoints/h30_30_pointpred_v1/mean_warmup.pth
"""

import argparse
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
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from conformal_horizon_calibration import (  # noqa: E402
    conformal_quantile, _trial_ids_for, _FEATURES, _SERIES, _extract_series,
)

DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'conformal_scene_conditioning')
_CHECK_SERIES = ['position', 'steering']   # the two most turn-relevant series; keeps runtime bounded

MIN_SEG_M = 0.5
K_TURN = 5


def _heading(vec):
    return np.arctan2(vec[1], vec[0])


def _wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def robust_turn_deg(xy_m, k=K_TURN, min_seg_m=MIN_SEG_M):
    """Same denoised heading-change metric as diagnose_turn_learning.py,
    duplicated (not imported) since that script isn't meant as a library --
    kept in sync deliberately, it's 6 lines."""
    start_vec = xy_m[k] - xy_m[0]
    end_vec   = xy_m[-1] - xy_m[-1 - k]
    if np.linalg.norm(start_vec) < min_seg_m or np.linalg.norm(end_vec) < min_seg_m:
        return 0.0
    return float(np.degrees(abs(_wrap(_heading(end_vec) - _heading(start_vec)))))


def _load_model(model_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path} ({model.count_parameters():,} params)")
    return model, device


def _collect(model, device, loader):
    """One pass: residual errors per source feature (as
    conformal_horizon_calibration.py's _collect_residuals) PLUS h_last per
    window -- combined into one pass since both come from the same forward
    call."""
    errs = {k: [] for k in _FEATURES}
    h_all = []
    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past   = {k: v.to(device) for k, v in past.items()}
            future = {k: v.to(device) for k, v in future.items()}
            graph  = {k: v.to(device) for k, v in graph.items()}
            h_last = model.encode_scene(past, graph)
            preds  = model(past, graph)
            h_all.append(h_last.cpu().numpy())
            for key, dims in _FEATURES.items():
                mean   = preds[f'{key}_mean']
                actual = future[key]
                if dims == 1 and actual.dim() == 3 and actual.size(-1) == 1:
                    actual = actual.squeeze(-1)
                errs[key].append((actual - mean).cpu().numpy())
    return ({k: np.concatenate(v, axis=0) for k, v in errs.items()},
            np.concatenate(h_all, axis=0))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--alpha', type=float, default=0.1)
    ap.add_argument('--k-neighbors', type=int, default=150)
    ap.add_argument('--test-sample-per-fold', type=int, default=200,
                     help='k-NN search cost scales with this; capped per fold')
    ap.add_argument('--seed', type=int, default=20260824)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    model, device = _load_model(args.model)
    ds = TrajectoryDataset(cfg.CAL_DIR)
    n = len(ds)
    trial_id, pkl_files = _trial_ids_for(cfg.CAL_DIR)
    n_trials = len(pkl_files)
    T_out = cfg.OUTPUT_SEQ_LEN
    print(f"Calibration set: {n_trials} trials, {n} windows")

    loader = DataLoader(ds, batch_size=args.batch, shuffle=False, num_workers=0)
    print("Collecting residuals + scene embeddings (one pass)...")
    errs, h_all = _collect(model, device, loader)
    all_resid = _extract_series(errs)   # {report_key: (N, T_out)}

    # actual future turn severity per window, for the turn-vs-calm breakdown
    print("Computing per-window turn severity...")
    turn_deg = np.zeros(n)
    for idx in range(n):
        future_pos = np.array([f['position'] for f in ds.sequences[idx]['future']]) * cfg.POSITION_DISPLACEMENT_RANGE_M
        turn_deg[idx] = robust_turn_deg(future_pos)

    rng = np.random.default_rng(args.seed)
    results = {key: {'global_q': [], 'scene_q': [], 'global_cov': [], 'scene_cov': [], 'turn_deg': [],
                      'global_cov_perstep': [], 'scene_cov_perstep': []}
               for key in _CHECK_SERIES}

    for fold, held_out in enumerate(range(n_trials)):
        fit_mask  = trial_id != held_out
        test_idx_all = np.where(trial_id == held_out)[0]
        n_sample = min(args.test_sample_per_fold, len(test_idx_all))
        test_idx = rng.choice(test_idx_all, size=n_sample, replace=False)

        fit_h = h_all[fit_mask]                      # (n_fit, d_h)
        fit_idx_global = np.where(fit_mask)[0]

        for key in _CHECK_SERIES:
            resid_fit_full = all_resid[key][fit_mask]     # (n_fit, T_out)
            q_global = np.array([conformal_quantile(resid_fit_full[:, t], args.alpha) for t in range(T_out)])

            for i in test_idx:
                # k nearest neighbors of window i's h_last among the fit set
                dists = np.linalg.norm(fit_h - h_all[i][None, :], axis=1)
                nn = np.argsort(dists)[:args.k_neighbors]
                neigh_resid = resid_fit_full[nn]           # (k, T_out)
                q_scene = np.array([conformal_quantile(neigh_resid[:, t], args.alpha) for t in range(T_out)])

                r = all_resid[key][i]
                results[key]['global_q'].append(q_global.mean())
                results[key]['scene_q'].append(q_scene.mean())
                # whole-horizon JOINT coverage (all 30 steps simultaneously in-band --
                # the operationally relevant "can I trust the whole predicted trajectory"
                # question, matching plot_layer1_trust_examples.py's "inside band the
                # whole horizon" label)
                results[key]['global_cov'].append(bool(np.all(r <= q_global)))
                results[key]['scene_cov'].append(bool(np.all(r <= q_scene)))
                # PER-STEP coverage (each of the 30 (window,step) pairs counted
                # independently, then pooled) -- the metric conformal_horizon_
                # calibration.py's already-validated ~90% figure actually used;
                # kept separate so the two aren't conflated.
                results[key]['global_cov_perstep'].append((r <= q_global).astype(float))
                results[key]['scene_cov_perstep'].append((r <= q_scene).astype(float))
                results[key]['turn_deg'].append(turn_deg[i])

        print(f"  fold {fold+1}/{n_trials} (held out {pkl_files[held_out]}, {n_sample} test windows) done")

    # ── Report ────────────────────────────────────────────────────────────
    for key in _CHECK_SERIES:
        r = {k: (np.array(v) if k not in ('global_cov_perstep', 'scene_cov_perstep')
                 else np.stack(v)) for k, v in results[key].items()}
        print(f"\n=== {key} ===")
        print(f"Whole-horizon JOINT coverage (all 30 steps at once) — "
              f"global: {r['global_cov'].mean():.3f}, scene: {r['scene_cov'].mean():.3f} "
              f"(target {100*(1-args.alpha):.0f}%; this is a STRICTER bar than the per-step "
              f"metric below, expect it to read lower)")
        print(f"PER-STEP pooled coverage (matches conformal_horizon_calibration.py's already- "
              f"validated ~90% figure) — global: {r['global_cov_perstep'].mean():.3f}, "
              f"scene: {r['scene_cov_perstep'].mean():.3f}")
        print(f"Overall mean quantile width — global: {r['global_q'].mean():.4f}, scene: {r['scene_q'].mean():.4f}")

        turn_mask = r['turn_deg'] > 15.0
        calm_mask = r['turn_deg'] <= 2.0
        if turn_mask.sum() > 0:
            print(f"TURN windows (n={turn_mask.sum()}, mean {r['turn_deg'][turn_mask].mean():.1f} deg): "
                  f"joint cov global={r['global_cov'][turn_mask].mean():.3f} scene={r['scene_cov'][turn_mask].mean():.3f}, "
                  f"per-step cov global={r['global_cov_perstep'][turn_mask].mean():.3f} scene={r['scene_cov_perstep'][turn_mask].mean():.3f}, "
                  f"width global={r['global_q'][turn_mask].mean():.4f} scene={r['scene_q'][turn_mask].mean():.4f}")
        if calm_mask.sum() > 0:
            print(f"CALM windows (n={calm_mask.sum()}, mean {r['turn_deg'][calm_mask].mean():.1f} deg): "
                  f"joint cov global={r['global_cov'][calm_mask].mean():.3f} scene={r['scene_cov'][calm_mask].mean():.3f}, "
                  f"per-step cov global={r['global_cov_perstep'][calm_mask].mean():.3f} scene={r['scene_cov_perstep'][calm_mask].mean():.3f}, "
                  f"width global={r['global_q'][calm_mask].mean():.4f} scene={r['scene_q'][calm_mask].mean():.4f}")

        # scatter: quantile width vs actual turn severity, global vs scene
        fig, ax = plt.subplots(figsize=(7, 5))
        order = np.argsort(r['turn_deg'])
        ax.scatter(r['turn_deg'], r['global_q'], s=6, alpha=0.35, color='#7f7f7f', label='global (vanilla) width')
        ax.scatter(r['turn_deg'], r['scene_q'], s=6, alpha=0.35, color='#2ca02c', label='scene-conditioned width')
        ax.set_xlabel('actual future turn severity (deg heading change over horizon)')
        ax.set_ylabel(f'mean-over-horizon conformal interval half-width ({key})')
        ax.set_title(f'{key}: does the interval widen for turns? (global vs. scene-conditioned)')
        ax.legend(fontsize=8)
        fig.tight_layout()
        out_path = os.path.join(args.output_dir, f'{key}_width_vs_turn.png')
        fig.savefig(out_path, dpi=140)
        plt.close(fig)
        print(f'Saved {out_path}')


if __name__ == '__main__':
    main()
