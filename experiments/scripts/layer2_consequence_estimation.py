#!/usr/bin/env python3
"""
Layer 2 v1 (consequence estimation) -- see the approved plan
(docs/research_notes/open_world_safety_reframe_2026-08-20.md §4 for the
architecture, and this session's plan file for the concrete design) and
experiments/lib/margin.py for the safety-margin primitives this wraps.

For each analysis window: take the point predictor's forecast, generate K
counterfactual position trajectories by bootstrap-resampling WHOLE residual
trajectories from Layer 1's held-out calibration pool (not independent
per-step/per-feature sampling -- see margin.py and the plan for why this
preserves realistic temporal correlation for free), check each
counterfactual trajectory against the combined lane-boundary/object-
clearance margin, and report P(violation within H).

Decomposition, revised from the original plan during implementation
(2026-08-21): the chosen v1 margin (lane-boundary distance + object
clearance) depends ONLY on the predicted ego POSITION -- not
velocity/steering/acceleration/TL, since those don't feed the margin
function at all. A "which of the 7 features drives this risk" breakdown
would therefore be vacuous (always "100% position"). Decomposing instead
along ALEATORIC (Layer 1's calibration-residual bootstrap) vs. EPISTEMIC
(cross-member disagreement, reusing the epistemic_disagreement_check.py
members) contribution to position uncertainty is a more meaningful
question given this margin's actual structure, and directly reuses both
of today's built signals rather than adding a new, uninformative one.

Validation scope, per the plan: NOMINAL data only. This is the same kind
of "does the signal stay quiet under normal conditions" sanity check
already established for the old SPRT signal
(plot_sprt_signal_behavior.py's nominal-floor pattern) -- NOT a full
reliability diagram, which needs real violation ground truth that barely
exists in nominal driving by construction. That's fault-data work, deferred
per Kalpit until he's back at the lab.

Usage (repo venv + ROS sourced -- margin.py needs lanelet2/MGRSProjector,
unlike conformal_horizon_calibration.py/epistemic_disagreement_check.py
which are ROS-free):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/layer2_consequence_estimation.py \
      --model st_gat/checkpoints/h30_30_pointpred_v1/mean_warmup.pth
"""

import argparse
import json
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
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))
sys.path.insert(0, SCRIPT_DIR)   # for conformal_horizon_calibration import below

import margin as margin_lib  # noqa: E402
from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
from conformal_horizon_calibration import _trial_ids_for  # noqa: E402

DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'layer2_consequence_estimation')


def _load_model(model_path, device):
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path} ({model.count_parameters():,} params)")
    return model


def _collect_position_means_and_raw(model, device, ds, batch_size):
    """One pass over the WHOLE dataset (order-preserving, shuffle=False --
    the returned arrays are index-aligned with ds.sequences). Returns:
    - pos_mean: (N, T_out, 2) normalized [-1,1] position prediction
    - pos_actual: (N, T_out, 2) normalized [-1,1] actual position (for the
      residual bootstrap pool)
    """
    loader = DataLoader(ds, batch_size=batch_size, shuffle=False, num_workers=0)
    means, actuals = [], []
    with torch.no_grad():
        for past, future, graph, _bounds in loader:
            past  = {k: v.to(device) for k, v in past.items()}
            graph = {k: v.to(device) for k, v in graph.items()}
            preds = model(past, graph)
            means.append(preds['position_mean'].cpu().numpy())
            actuals.append(future['position'].numpy())
    return np.concatenate(means, axis=0), np.concatenate(actuals, axis=0)


def _denorm_position(mean_norm: np.ndarray, ref_x: float, ref_y: float) -> np.ndarray:
    """mean_norm: (T_out, 2) normalized [-1,1] displacement -> (T_out, 2)
    real map-frame (x, y), per sequence_builder.py's _scale_position_relative
    (real = ref + normalized * POSITION_DISPLACEMENT_RANGE_M)."""
    real = mean_norm * cfg.POSITION_DISPLACEMENT_RANGE_M
    real[:, 0] += ref_x
    real[:, 1] += ref_y
    return real


def _current_objects_real(past_last_frame: dict) -> list:
    """De-normalized (dx, dy, speed) for every REAL (mask=1) tracked object
    at the last past frame, per sequence_builder.py's _build_object_set
    encoding (feats[:,0]=dx, [:,1]=dy via OBJECT_REL_POS_RANGE,
    [:,2]=speed via OBJECT_SPEED_RANGE)."""
    feats = np.asarray(past_last_frame['objects_set'])
    mask  = np.asarray(past_last_frame['objects_mask'])
    rx_min, rx_max = cfg.OBJECT_REL_POS_RANGE
    sp_min, sp_max = cfg.OBJECT_SPEED_RANGE
    out = []
    for i in range(feats.shape[0]):
        if mask[i] < 0.5:
            continue
        dx = feats[i, 0] * (rx_max - rx_min) + rx_min
        dy = feats[i, 1] * (rx_max - rx_min) + rx_min
        speed = feats[i, 2] * (sp_max - sp_min) + sp_min
        out.append((float(dx), float(dy), float(speed)))
    return out


def trajectory_margin_series(real_xy: np.ndarray, objects: list, map_data,
                              step_stride: int, dt: float = 0.1) -> np.ndarray:
    """real_xy: (T_out, 2) real map-frame trajectory. Returns the combined
    margin at every `step_stride`-th horizon step (checking all 30 steps is
    unnecessary for a "does this dip below threshold anywhere" check and
    dominates runtime -- see the module's compute-budget note)."""
    x0, y0 = real_xy[0]
    out = []
    for t in range(0, real_xy.shape[0], step_stride):
        x, y = real_xy[t]
        lane_dist = margin_lib.lane_boundary_distance(x, y, map_data)
        ego_disp = float(np.hypot(x - x0, y - y0))
        t_ahead = t * dt
        obj_clear = [margin_lib.object_clearance(dx, dy, speed, t_ahead, ego_disp)
                     for dx, dy, speed in objects]
        out.append(margin_lib.margin(lane_dist, obj_clear))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--k-draws', type=int, default=100, help='bootstrap counterfactual draws per window')
    ap.add_argument('--window-stride', type=int, default=5,
                     help='analyze every Nth window (compute-budget control, see module docstring)')
    ap.add_argument('--step-stride', type=int, default=3,
                     help='check every Nth horizon step within a trajectory (compute-budget control)')
    ap.add_argument('--horizon-s', type=float, default=3.0, help='risk horizon H, seconds')
    ap.add_argument('--threshold-m', type=float, default=1.0, help='margin violation threshold, metres')
    ap.add_argument('--n-example-trials', type=int, default=3, help='how many trials to render a time-series trace for')
    ap.add_argument('--seed', type=int, default=20260821)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = _load_model(args.model, device)
    map_data = margin_lib.load_map(cfg.MAP_FILE)

    ds = TrajectoryDataset(cfg.CAL_DIR)
    n = len(ds)
    trial_id, pkl_files = _trial_ids_for(cfg.CAL_DIR)
    assert len(trial_id) == n

    print("Collecting position predictions + actuals (one model pass)...")
    pos_mean, pos_actual = _collect_position_means_and_raw(model, device, ds, args.batch)
    residuals = pos_actual - pos_mean   # (N, T_out, 2), normalized units -- the bootstrap pool

    T_out = cfg.OUTPUT_SEQ_LEN
    n_horizon_steps = int(round(args.horizon_s / 0.1))  # steps within H
    rng = np.random.default_rng(args.seed)

    analyze_idx = np.arange(0, n, args.window_stride)
    print(f"Analyzing {len(analyze_idx)}/{n} windows (stride={args.window_stride}), "
          f"K={args.k_draws} draws, step_stride={args.step_stride}, H={args.horizon_s}s, "
          f"threshold={args.threshold_m}m")

    p_violation = np.full(n, np.nan)
    trial_of = {}   # trial_id -> list of (window_idx_within_trial, p_violation) for example plots

    for count, i in enumerate(analyze_idx):
        seq = ds.sequences[i]
        ref_x, ref_y = seq['position_ref']
        real_xy_pred = _denorm_position(pos_mean[i].copy(), ref_x, ref_y)   # (T_out, 2)
        objects = _current_objects_real(seq['past'][-1])

        # Bootstrap pool: exclude this window's OWN trial (LOO-CV discipline,
        # same as conformal_horizon_calibration.py) so a window is never
        # perturbed by a residual drawn from itself.
        pool_mask = trial_id != trial_id[i]
        pool_idx = np.where(pool_mask)[0]

        n_viol = 0
        for _k in range(args.k_draws):
            j = rng.choice(pool_idx)
            counterfactual_norm = pos_mean[i] + residuals[j]     # (T_out, 2), normalized
            counterfactual_real = _denorm_position(counterfactual_norm.copy(), ref_x, ref_y)
            m_series = trajectory_margin_series(
                counterfactual_real[:n_horizon_steps], objects, map_data, args.step_stride)
            if np.any(m_series < args.threshold_m):
                n_viol += 1
        p_violation[i] = n_viol / args.k_draws

        if (count + 1) % 200 == 0 or count == len(analyze_idx) - 1:
            print(f"  {count+1}/{len(analyze_idx)} windows done "
                  f"(mean P(violation) so far: {np.nanmean(p_violation):.4f})")

    analyzed = p_violation[~np.isnan(p_violation)]
    frac_above_01 = float((analyzed > 0.1).mean())
    frac_above_05 = float((analyzed > 0.5).mean())
    print(f"\n=== Nominal-driving sanity check (P(violation within {args.horizon_s}s)) ===")
    print(f"  mean:              {analyzed.mean():.4f}")
    print(f"  max:               {analyzed.max():.4f}")
    print(f"  fraction > 0.1:    {frac_above_01:.4f}")
    print(f"  fraction > 0.5:    {frac_above_05:.4f}")

    report = {
        'k_draws': args.k_draws, 'window_stride': args.window_stride,
        'step_stride': args.step_stride, 'horizon_s': args.horizon_s,
        'threshold_m': args.threshold_m, 'n_windows_analyzed': int(len(analyzed)),
        'mean_p_violation': float(analyzed.mean()), 'max_p_violation': float(analyzed.max()),
        'frac_above_0.1': frac_above_01, 'frac_above_0.5': frac_above_05,
    }
    report_path = os.path.join(args.output_dir, 'layer2_report.json')
    with open(report_path, 'w') as f:
        json.dump(report, f, indent=2)
    print(f'Saved {report_path}')

    # Example time-series traces, a few trials.
    fig, axes = plt.subplots(min(args.n_example_trials, len(pkl_files)), 1,
                              figsize=(12, 3 * min(args.n_example_trials, len(pkl_files))), squeeze=False)
    for row, t_id in enumerate(range(min(args.n_example_trials, len(pkl_files)))):
        idx_in_trial = analyze_idx[trial_id[analyze_idx] == t_id]
        vals = p_violation[idx_in_trial]
        ax = axes[row, 0]
        ax.plot(np.arange(len(vals)) * 0.1 * args.window_stride, vals, color='#d62728', linewidth=1.5)
        ax.axhline(0.1, color='#999999', linestyle='--', linewidth=1)
        ax.set_title(f'{pkl_files[t_id]} — P(violation within {args.horizon_s}s)', fontsize=9)
        ax.set_ylim(-0.02, 1.02)
        ax.set_xlabel('trial time (s, approx)', fontsize=8)
        ax.tick_params(labelsize=7)
    fig.suptitle('Layer 2: P(margin violation) over nominal driving — should stay near 0', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    plot_path = os.path.join(args.output_dir, 'p_violation_trace.png')
    fig.savefig(plot_path, dpi=140)
    plt.close(fig)
    print(f'Saved {plot_path}')


if __name__ == '__main__':
    main()
