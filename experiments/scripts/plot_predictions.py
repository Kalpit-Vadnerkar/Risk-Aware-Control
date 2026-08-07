#!/usr/bin/env python3
"""
Visualizes what the trained ST-GAT model actually predicts for a handful of
example windows: predicted position trajectory (mean + uncertainty) overlaid
on the HD map next to the real driven trajectory, plus predicted-vs-actual
time series for a few other features (velocity, steering, traffic_light_color,
traffic_light_confidence, traffic_light_discrepancy).

Position is the map panel because it's the one prediction that ties directly
to the route/map plots the rest of this project already uses (plot_routes.py)
— seeing the predicted 3-second-ahead trajectory next to the real one, on
the actual road geometry, is a much more direct sanity check than a bare
number ever is. The other panels reuse plotting.py's plot_mean_variance_band
(shaded predicted mean +/- std, actual overlaid).

Selects up to --n-examples windows spread evenly across the trial (not just
the first N) so a single run shows the model's behavior at different points
along the route, not just near the start.

Updated 2026-08-06 for the Gaussian -> Student-t head redesign (see
model.py's docstring): every "±1 std" band below is now the Student-t
IMPLIED std (sqrt(scale^2 * dof/(dof-2))), not a raw predicted variance —
model.py no longer emits a `_var` key at all, only `_mean`/`_scale`/`_dof`.

Usage (must source ROS/Autoware, then the repo venv):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/plot_predictions.py --dataset nom_v11 --goal goal_012 --trial 1
  python3 experiments/scripts/plot_predictions.py --dataset imu_fault_stuck --goal goal_007 --trial 1 --n-examples 6
"""

import argparse
import glob
import os
import sys

import numpy as np
import torch
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, REPO_DIR)   # so `import st_gat` resolves when run as a plain script
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from plotting import (  # noqa: E402
    load_map, draw_map_background, style_map_axes, bbox_with_margin,
    plot_mean_variance_band, SPAWN,
)

from st_gat.pipeline import config as cfg  # noqa: E402
from st_gat.pipeline.bag_reader import read_bag  # noqa: E402
from st_gat.pipeline.sequence_builder import SequenceBuilder  # noqa: E402
from st_gat.model import STGAT  # noqa: E402
from st_gat.model.dataset import TrajectoryDataset  # noqa: E402

DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'predictions')

# Other-feature panels shown alongside position — (key, dims, label).
# velocity/steering/traffic_light_color/traffic_light_confidence have
# Gaussian (mean/var) heads; traffic_light_discrepancy is a sigmoid
# probability (no predicted variance, so its panel just overlays
# predicted-probability vs actual 0/1). traffic_light_color/confidence
# replace the old collapsed traffic_light_state (2026-08-05 redesign — see
# config.py's FEATURE_SIZES doc).
_SCALAR_PANELS = [
    ('velocity',                 2, 'velocity (scaled long., lat.)'),
    ('steering',                 1, 'steering (scaled)'),
    ('traffic_light_color',      1, 'TL color (perception channel)'),
    ('traffic_light_confidence', 1, 'TL confidence (perception channel)'),
    ('traffic_light_discrepancy', 1, 'TL discrepancy (map vs. perception)'),
]


def _find_trial_dir(dataset: str, goal: str, trial: int) -> str:
    base = os.path.join(cfg.DATA_ROOT, dataset, goal)
    matches = sorted(glob.glob(os.path.join(base, f't{trial}_*')))
    if not matches:
        raise FileNotFoundError(f'No trial {trial} found under {base}')
    return matches[0]


def _unscale_position(scaled_xy: np.ndarray, ref_xy) -> np.ndarray:
    """Inverse of sequence_builder._scale_position_relative — [-1,1]
    displacement-from-reference back to real bag-frame map coordinates,
    using this window's own position_ref (added 2026-08-02 alongside the
    position-representation fix) and the fixed cfg.POSITION_DISPLACEMENT_RANGE_M."""
    ref_x, ref_y = ref_xy
    out = np.empty_like(scaled_xy)
    out[..., 0] = scaled_xy[..., 0] * cfg.POSITION_DISPLACEMENT_RANGE_M + ref_x
    out[..., 1] = scaled_xy[..., 1] * cfg.POSITION_DISPLACEMENT_RANGE_M + ref_y
    return out


def plot_example(model, device, map_data, seq: dict, goal_xy, out_path: str, margin: float = 3.0):
    """One figure: map panel (position, tied to real map geometry) + a
    column of other-feature panels (predicted vs actual over the horizon)."""
    past_t   = TrajectoryDataset._build_feature_tensors(seq['past'])
    future_t = TrajectoryDataset._build_feature_tensors(seq['future'])
    graph_t  = TrajectoryDataset._build_graph_tensors(seq['graph'])

    batch_past  = {k: v.unsqueeze(0).to(device) for k, v in past_t.items()}
    batch_graph = {k: v.unsqueeze(0).to(device) for k, v in graph_t.items()}

    model.eval()
    with torch.no_grad():
        preds = model(batch_past, batch_graph)

    ref_xy = seq['position_ref']

    # ── Position: past (input), actual future, predicted future ────────────
    past_pos_scaled   = np.array([s['position'] for s in seq['past']])
    future_pos_scaled = np.array([s['position'] for s in seq['future']])
    pred_mean_scaled  = preds['position_mean'][0].cpu().numpy()          # (T_out, 2)
    pred_scale_scaled = preds['position_scale'][0].cpu().numpy()         # (T_out, 2)
    pred_dof          = preds['position_dof'][0].cpu().numpy()           # (T_out,) -- shared across x/y
    # Student-t implied variance (2026-08-06 redesign): scale^2 * dof/(dof-2),
    # not the raw scale parameter directly -- see model.py's docstring.
    pred_var_scaled = pred_scale_scaled ** 2 * (pred_dof / (pred_dof - 2))[:, None]

    past_xy   = _unscale_position(past_pos_scaled, ref_xy)
    actual_xy = _unscale_position(future_pos_scaled, ref_xy)
    pred_xy   = _unscale_position(pred_mean_scaled, ref_xy)
    # Std in real metres: fixed conversion factor, same for every window
    # (unlike the old per-window graph_bounds scale).
    pred_std_xy = np.sqrt(pred_var_scaled) * cfg.POSITION_DISPLACEMENT_RANGE_M

    fig = plt.figure(figsize=(15, 8.5))
    gs = fig.add_gridspec(len(_SCALAR_PANELS), 2, width_ratios=[1.3, 1])
    ax_map = fig.add_subplot(gs[:, 0])
    axes_ts = [fig.add_subplot(gs[i, 1]) for i in range(len(_SCALAR_PANELS))]

    # ── Map panel ────────────────────────────────────────────────────────────
    # Zoom to the trajectory itself, NOT the (usually much farther away) goal —
    # a single 3s-history/3s-future window only covers tens of metres, so
    # including the goal in the bbox (as plot_fault_impact.py does for a
    # FULL-TRIAL trajectory, where it's a reasonable end-of-range point)
    # would zoom out until the window shrinks to an invisible dot.
    all_x = np.concatenate([past_xy[:, 0], actual_xy[:, 0], pred_xy[:, 0]])
    all_y = np.concatenate([past_xy[:, 1], actual_xy[:, 1], pred_xy[:, 1]])
    # Margin scales with the window's OWN extent instead of a flat metres
    # value (found 2026-08-04): a fixed 40m margin is sized for a FULL-TRIAL
    # trajectory (hundreds of metres, plot_fault_impact.py's use case) — for
    # a single 3s+3s window, especially a slow/near-stationary one, 40m on
    # each side dwarfs the actual movement and shrinks the trajectory (and
    # the uncertainty circles on it) to an unreadable sliver in the middle of
    # a mostly-empty map. `margin` (default 3m, see --margin) is now just the
    # floor for a near-stationary window; 0.4x the window's own span is what
    # actually drives the zoom level for any window with real movement in it,
    # so a longer/faster horizon naturally gets more map shown around it.
    span = max(all_x.max() - all_x.min(), all_y.max() - all_y.min())
    dyn_margin = max(margin, 0.4 * span)
    xmin, xmax, ymin, ymax = bbox_with_margin(all_x, all_y, dyn_margin)
    draw_map_background(ax_map, map_data, xmin, xmax, ymin, ymax)

    ax_map.plot(past_xy[:, 0], past_xy[:, 1], color='#000000', linewidth=2.2,
                alpha=0.85, zorder=3, label='past (input, 3s)')
    ax_map.plot(actual_xy[:, 0], actual_xy[:, 1], color='#1f77b4', linewidth=1.8,
                alpha=0.85, zorder=4, label='actual future (3s)')
    ax_map.plot(pred_xy[:, 0], pred_xy[:, 1], color='#d62728', linewidth=1.6,
                linestyle='--', zorder=5, label='predicted future (mean)')
    # Uncertainty: sparse circles (every 5th predicted step, not all 30) —
    # radius = 1 std (combined x/y magnitude). Drawing every step makes
    # consecutive circles overlap almost completely whenever the vehicle is
    # slow (predicted points barely move between steps), so ~30 stacked
    # low-alpha circles visually merge into one solid blob and hide the
    # actual widening trend rather than showing it; a handful of distinct,
    # outlined circles reads as discrete uncertainty "snapshots" instead.
    step = max(1, len(pred_xy) // 6)
    for (px, py), (sx, sy) in list(zip(pred_xy, pred_std_xy))[::step]:
        radius = float(np.hypot(sx, sy))
        if radius > 0:
            ax_map.add_patch(plt.Circle((px, py), radius, facecolor='#d62728',
                                         alpha=0.12, edgecolor='#d62728',
                                         linewidth=0.8, zorder=2))
    ax_map.plot(past_xy[0, 0], past_xy[0, 1], marker='o', color='#555555',
                markersize=8, zorder=6, label='window start')
    ax_map.plot(*goal_xy, marker='*', color='black', markersize=14, zorder=6, label='goal')
    style_map_axes(ax_map, xmin, xmax, ymin, ymax)
    ax_map.set_title('Position: predicted vs. actual (shaded = predicted +/-1 std)')

    # ── Other-feature panels ─────────────────────────────────────────────────
    t = np.arange(len(seq['future'])) * 0.1   # seconds into the predicted horizon
    for ax, (key, dims, label) in zip(axes_ts, _SCALAR_PANELS):
        if dims == 1:
            actual = np.array([s[key] for s in seq['future']], dtype=float)
            if f'{key}_mean' in preds:
                mean  = preds[f'{key}_mean'][0].cpu().numpy()
                scale = preds[f'{key}_scale'][0].cpu().numpy()
                dof   = preds[f'{key}_dof'][0].cpu().numpy()
                var   = scale ** 2 * (dof / (dof - 2))   # Student-t implied variance
                plot_mean_variance_band(ax, t, mean, var, actual=actual)
            else:
                pred = preds[key][0].cpu().numpy()
                ax.plot(t, pred, color='#1f77b4', linewidth=1.3, label='predicted (p)')
                ax.plot(t, actual, color='#d62728', linewidth=1.1, alpha=0.85, label='actual')
        else:
            # velocity: plot longitudinal component only (dim 0) to keep the
            # panel readable — lateral is usually near-zero on this map.
            actual = np.array([s[key][0] for s in seq['future']], dtype=float)
            mean  = preds[f'{key}_mean'][0, :, 0].cpu().numpy()
            scale = preds[f'{key}_scale'][0, :, 0].cpu().numpy()
            dof   = preds[f'{key}_dof'][0].cpu().numpy()   # shared across dims
            var   = scale ** 2 * (dof / (dof - 2))
            plot_mean_variance_band(ax, t, mean, var, actual=actual)
        ax.set_ylabel(label, fontsize=8)
        ax.tick_params(labelsize=8)

    axes_ts[0].legend(loc='upper right', fontsize=7, framealpha=0.9)
    axes_ts[-1].set_xlabel('seconds into predicted horizon', fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dataset', default='nom_v11')
    ap.add_argument('--goal', required=True, help='e.g. goal_012')
    ap.add_argument('--trial', type=int, default=1)
    ap.add_argument('--n-examples', type=int, default=4, help='windows to plot, spread evenly across the trial')
    ap.add_argument('--model', default=cfg.MODEL_CONFIG['model_path'])
    ap.add_argument('--goals-file', default=os.path.join(REPO_DIR, 'experiments', 'configs', 'captured_goals.json'))
    ap.add_argument('--map-file', default=cfg.MAP_FILE)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    ap.add_argument('--margin', type=float, default=3.0,
                    help='floor, in metres, for a near-stationary window — the actual margin used is '
                         'max(this, 0.4 * the window\'s own trajectory span), see plot_example()')
    args = ap.parse_args()

    import json
    goals_data = json.load(open(args.goals_file))
    goal_entry = next((g for g in goals_data['goals'] if g['id'] == args.goal), None)
    if goal_entry is None:
        print(f'{args.goal} not found in {args.goals_file}', file=sys.stderr)
        sys.exit(1)
    goal_xy = (goal_entry['goal']['position']['x'], goal_entry['goal']['position']['y'])

    trial_dir = _find_trial_dir(args.dataset, args.goal, args.trial)
    bag_dir = os.path.join(trial_dir, 'rosbag')
    print(f'Reading: {trial_dir}')

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model_cfg = cfg.build_inference_model_cfg(device)
    model = STGAT(model_cfg).to(device)
    model.load_state_dict(torch.load(args.model, map_location=device, weights_only=True))
    model.eval()
    print(f'Loaded model: {args.model} ({model.count_parameters():,} params)')

    print(f'Loading map: {args.map_file}')
    map_data = load_map(args.map_file)

    frames = read_bag(bag_dir, goal_id=args.goal)
    builder = SequenceBuilder.from_bag(bag_dir)
    sequences = builder.build(frames)
    if not sequences:
        print('No sequences built (trial too short?)', file=sys.stderr)
        sys.exit(1)
    print(f'{len(sequences)} windows available')

    n = min(args.n_examples, len(sequences))
    indices = np.linspace(0, len(sequences) - 1, n, dtype=int)

    os.makedirs(args.output_dir, exist_ok=True)
    for i, idx in enumerate(indices):
        out_path = os.path.join(
            args.output_dir,
            f'{args.dataset}_{args.goal}_t{args.trial}_window{idx:04d}.png',
        )
        plot_example(model, device, map_data, sequences[idx], goal_xy, out_path, margin=args.margin)
        print(f'  [{i + 1}/{n}] window {idx}: saved {out_path}')


if __name__ == '__main__':
    main()
