#!/usr/bin/env python3
"""
Layer-1 trust visualization: for a handful of concrete example windows,
show what the model actually predicts, what the calibrated (conformal)
uncertainty band looks like around that prediction, and whether the real
outcome landed inside it. Complements the aggregate statistical evidence
(reliability_diagram.png, conformal_vs_actual.png) with the same story told
concretely, per Kalpit's request (2026-08-21): "I want to be able to show
through plots and diagrams what layer 1 is, what the model is predicting,
what the uncertainty looks like, and why can we trust the results."

Two views, both using the SAME calibrated quantile
(`mean_fold_quantile_by_step` from conformal_report.json -- already
computed by conformal_horizon_calibration.py's leave-one-trial-out
cross-conformal, not recomputed here):

1. Time-series band plots (6 non-position series: velocity_longitudinal/
   lateral, steering, acceleration, both TL heads) -- predicted mean +/- the
   calibrated half-width as a shaded band, actual value overlaid, per
   example window. Reuses experiments/lib/plotting.py's
   plot_mean_variance_band (pass half-width^2 as "variance", n_std=1 --
   the band IS the calibrated interval directly, not a distributional
   assumption).

2. Map-view spatial plot (position specifically, needs a 2D picture, not a
   1D band) -- past trajectory, predicted future trajectory, actual future
   trajectory, and the calibrated position uncertainty shown as a
   translucent disc of the calibrated radius at each horizon step, all
   drawn over the real map background. Reuses
   experiments/lib/plotting.py's map-loading/rendering conventions.

Usage (repo venv + ROS sourced -- needs the map for the position view):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/plot_layer1_trust_examples.py \
      --model st_gat/checkpoints/h30_30_pointpred_v1/mean_warmup.pth
"""

import argparse
import json
import os
import sys

import numpy as np
import torch

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.model import STGAT, TrajectoryDataset  # noqa: E402
import plotting as plib  # noqa: E402

DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'layer1_trust_examples')
CONFORMAL_REPORT = os.path.join(REPO_DIR, 'experiments', 'analysis',
                                 'conformal_horizon_calibration', 'conformal_report.json')
# Fixed canonical example windows, chosen by select_trust_example_windows.py
# (2026-08-24) to cover distinct real scenario types (turn, TL color change,
# hard braking, hard acceleration, calm baseline) rather than a random draw
# that might land on unremarkable windows or, by luck, unusually hard ones.
CANONICAL_WINDOWS = os.path.join(REPO_DIR, 'experiments', 'analysis', 'trust_example_windows.json')

# (report_key, source_feature, mode) for the 6 non-position series --
# mirrors conformal_horizon_calibration.py's _SERIES but this script needs
# the actual SIGNED predicted/actual values, not residual magnitudes, so
# it's kept separate rather than reusing _extract_series directly.
_SCALAR_SERIES = [
    ('velocity_longitudinal', 'velocity', 0),
    ('velocity_lateral',      'velocity', 1),
    ('steering',              'steering', None),
    ('acceleration',          'acceleration', None),
    ('traffic_light_color',       'traffic_light_color', None),
    ('traffic_light_confidence',  'traffic_light_confidence', None),
]


def _load_model(model_path, device):
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    print(f"Loaded model: {model_path} ({model.count_parameters():,} params)")
    return model


def _forward_one(model, device, ds, idx):
    past, future, graph, _bounds = ds[idx]
    past  = {k: torch.as_tensor(v, device=device).unsqueeze(0) for k, v in past.items()}
    graph = {k: torch.as_tensor(v, device=device).unsqueeze(0) for k, v in graph.items()}
    with torch.no_grad():
        preds = model(past, graph)
    return preds, future


def plot_scalar_bands(preds, future, quantiles: dict, idx: int, out_path: str):
    T_out = cfg.OUTPUT_SEQ_LEN
    steps = np.arange(T_out) * 0.1
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    for ax, (report_key, source_key, axis) in zip(axes.flat, _SCALAR_SERIES):
        mean = preds[f'{source_key}_mean'][0].cpu().numpy()
        actual = np.asarray(future[source_key])
        if axis is not None:
            mean, actual = mean[:, axis], actual[:, axis]
        else:
            mean = mean.reshape(-1)
            actual = actual.reshape(-1)
        q = np.array(quantiles[report_key])   # calibrated half-width per step
        plib.plot_mean_variance_band(ax, steps, mean, q ** 2, actual=actual,
                                      label='predicted mean', actual_label='actual')
        covered = np.all(np.abs(actual - mean) <= q)
        ax.set_title(f'{report_key}  ({"inside band" if covered else "band violated"} the whole horizon)',
                     fontsize=9.5)
        ax.set_xlabel('seconds into predicted horizon', fontsize=8)
        ax.tick_params(labelsize=7)
    axes.flat[0].legend(loc='best', fontsize=7)
    fig.suptitle(f'Layer 1 trust example, window {idx} — predicted mean ± calibrated conformal interval vs. actual',
                 fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def plot_position_map(preds, future, seq, quantile_position, map_data, idx: int, out_path: str,
                       n_circles: int = 6):
    ref_x, ref_y = seq['position_ref']
    past_frames = seq['past']
    past_xy = np.array([[ref_x + f['position'][0] * cfg.POSITION_DISPLACEMENT_RANGE_M,
                          ref_y + f['position'][1] * cfg.POSITION_DISPLACEMENT_RANGE_M]
                         for f in past_frames])

    mean_norm = preds['position_mean'][0].cpu().numpy()
    pred_xy = mean_norm * cfg.POSITION_DISPLACEMENT_RANGE_M
    pred_xy[:, 0] += ref_x
    pred_xy[:, 1] += ref_y

    actual_norm = np.asarray(future['position'])
    actual_xy = actual_norm * cfg.POSITION_DISPLACEMENT_RANGE_M
    actual_xy[:, 0] += ref_x
    actual_xy[:, 1] += ref_y

    all_x = np.concatenate([past_xy[:, 0], pred_xy[:, 0], actual_xy[:, 0]])
    all_y = np.concatenate([past_xy[:, 1], pred_xy[:, 1], actual_xy[:, 1]])
    xmin, xmax, ymin, ymax = plib.bbox_with_margin(all_x, all_y, margin=15.0)

    fig, ax = plt.subplots(figsize=(8, 8))
    plib.draw_map_background(ax, map_data, xmin, xmax, ymin, ymax)
    ax.plot(past_xy[:, 0], past_xy[:, 1], color='#555555', linewidth=1.5, label='observed past')
    ax.plot(pred_xy[:, 0], pred_xy[:, 1], color='#1f77b4', linewidth=1.5, linestyle='--', label='predicted future (mean)')
    ax.plot(actual_xy[:, 0], actual_xy[:, 1], color='#d62728', linewidth=1.5, label='actual future')

    T_out = pred_xy.shape[0]
    step_show = max(1, T_out // n_circles)
    for t in range(0, T_out, step_show):
        # quantile_position is in NORMALIZED units (same space as position_mean
        # before de-normalization above) -- must be scaled by the same
        # POSITION_DISPLACEMENT_RANGE_M to become a real-metre radius, or the
        # circle is ~100x too small to see against real map coordinates (bug
        # found 2026-08-24: circles were invisible because this scaling was
        # missing).
        r = quantile_position[t] * cfg.POSITION_DISPLACEMENT_RANGE_M
        circ = Circle((pred_xy[t, 0], pred_xy[t, 1]), r, facecolor='#1f77b4', alpha=0.25,
                       edgecolor='#1f77b4', linewidth=1.0, zorder=3)
        ax.add_patch(circ)
    ax.plot([], [], marker='o', markersize=10, markerfacecolor='#1f77b4', alpha=0.3,
             markeredgecolor='#1f77b4', linestyle='none', label='calibrated position uncertainty (radius = conformal quantile)')

    plib.style_map_axes(ax, xmin, xmax, ymin, ymax, legend_fontsize=8)
    ax.set_title(f'Layer 1 trust example, window {idx} — predicted vs. actual trajectory, calibrated uncertainty', fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'Saved {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--random', action='store_true',
                     help='draw a random sample instead of the fixed canonical windows '
                          '(experiments/analysis/trust_example_windows.json)')
    ap.add_argument('--n-examples', type=int, default=4)
    ap.add_argument('--seed', type=int, default=20260821)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    with open(CONFORMAL_REPORT) as f:
        report = json.load(f)
    quantiles = {row['feature']: row['mean_fold_quantile_by_step'] for row in report['features']}
    print(f"Loaded calibrated quantiles from {CONFORMAL_REPORT} "
          f"(leave-one-trial-out cross-conformal, alpha={report['alpha']})")

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = _load_model(args.model, device)
    ds = TrajectoryDataset(cfg.CAL_DIR)
    map_data = plib.load_map(cfg.MAP_FILE)

    if args.random or not os.path.exists(CANONICAL_WINDOWS):
        rng = np.random.default_rng(args.seed)
        idxs = [int(i) for i in rng.choice(len(ds), size=args.n_examples, replace=False)]
        names = {i: f'window{i}' for i in idxs}
    else:
        with open(CANONICAL_WINDOWS) as f:
            canonical = json.load(f)
        idxs = list(canonical.values())
        names = {v: f'{k}_window{v}' for k, v in canonical.items()}
        print(f"Using fixed canonical example windows from {CANONICAL_WINDOWS}: {canonical}")

    for idx in idxs:
        idx = int(idx)
        tag = names[idx]
        preds, future = _forward_one(model, device, ds, idx)
        seq = ds.sequences[idx]
        plot_scalar_bands(preds, future, quantiles, idx,
                           os.path.join(args.output_dir, f'{tag}_scalar_bands.png'))
        plot_position_map(preds, future, seq, quantiles['position'], map_data, idx,
                           os.path.join(args.output_dir, f'{tag}_position_map.png'))


if __name__ == '__main__':
    main()
