#!/usr/bin/env python3
"""
Per-trial fault-impact visualization: WHERE the vehicle was and WHAT was
happening to it around a fault window, in one figure.

Combines plot_routes.py's map rendering with compare_fault_vs_nominal.py's
bag-reading (GT/EKF divergence, fault windows, feature series) and adds two
things neither of those scripts shows explicitly:
  1. Autoware's own MRM trigger timestamp(s) (from MetricsCollector.mrm_states,
     NORMAL -> MRM_OPERATING), marked on both the map and the time axis — the
     "did Autoware ever step in, and when" question.
  2. The trial's actual outcome per metrics.py's `static_collision` heuristic
     (permanently stopped + never reached goal = likely a real collision with
     map geometry) — distinct from a plain goal_reached/stuck split.

Which features get their own time-series panel is NOT fixed (revised
2026-07-25 — a fixed velocity/divergence/object-distance panel set showed
"no effect" for TL faults purely because none of those three are the signal
TL faults actually touch; tl_confidence is). Instead this reuses
compare_fault_vs_nominal.py's own discriminability recipe (z-score vs pooled
nominal stats, in-fault vs out-of-fault) to RANK all 6 candidate features per
trial and plots whichever 3 actually responded to this fault — for TL faults
that surfaces tl_confidence, for IMU faults it surfaces ekf_gt_divergence
and/or steering_rad, etc. Each panel also overlays the raw nominal trial(s)'
own series (not just a mean/std band) for direct visual comparison.

Usage (must source ROS/Autoware, then the repo venv):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/plot_fault_impact.py --campaign imu_fault_scale --goal goal_012
  python3 experiments/scripts/plot_fault_impact.py --campaign tl_fault_s3 --goal goal_012 --trial 1
"""

import argparse
import glob
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
WORKSPACE_DIR = os.path.dirname(REPO_DIR)
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from metrics import MetricsCollector  # noqa: E402
from plot_routes import (  # noqa: E402
    make_projector, load_map, ll_attr, lanelet_polygon_xy, bbox_hit,
    traffic_light_points, SPAWN,
)
from compare_fault_vs_nominal import (  # noqa: E402
    read_gt_and_tl, ekf_gt_divergence, load_fault_log, extract_fault_windows,
    resample, in_any_window,
)
from autoware_perception_msgs.msg import TrafficLightElement  # noqa: E402

DEFAULT_MAP_FILE = os.path.join(WORKSPACE_DIR, 'Map', 'nishishinjuku_autoware_map', 'lanelet2_map.osm')
DEFAULT_DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data')
DEFAULT_GOALS_FILE = os.path.join(REPO_DIR, 'experiments', 'configs', 'captured_goals.json')
DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'fault_impact')

# autoware_adapi_v1_msgs/msg/MrmState: state 1 = NORMAL, 2 = MRM_OPERATING
# (same values metrics.py's own _compute_fail_operational_metrics uses).
MRM_NORMAL = 1
MRM_OPERATING = 2

FAULT_COLOR = '#d55e00'
MRM_COLOR = '#9467bd'
TRAJ_COLOR = '#1f77b4'
NOMINAL_COLOR = '#999999'

# Binary outcome (simplified 2026-07-25 — collision-vs-stuck isn't a
# meaningful distinction for this dissertation's claim: either way the fault
# was fatal to completing the route. metrics.json's static_collision heuristic
# still exists as raw data if that distinction matters again later).
OUTCOME_STYLE = {
    'goal_reached': dict(color='#2ca02c', marker='o', label='goal reached'),
    'fatal':        dict(color='#d62728', marker='X', label='fatal (did not reach goal)'),
}

FEATURE_LABELS = {
    'velocity_mps':        'velocity (m/s)',
    'acceleration_mps2':   'acceleration (m/s²)',
    'steering_rad':        'steering (rad)',
    'obj_distance_m':      'min object dist (m)',
    'ekf_gt_divergence_m': 'EKF-GT divergence (m)',
    'tl_confidence':       'TL confidence',
    'tl_is_green':         'TL reported GREEN (0/1)',
}
LOG_SCALE_FEATURES = {'ekf_gt_divergence_m'}


def mrm_trigger_times(mrm_states):
    triggers = []
    prev = MRM_NORMAL
    for t, state, _behavior in mrm_states:
        if prev == MRM_NORMAL and state == MRM_OPERATING:
            triggers.append(t)
        prev = state
    return triggers


def fault_kind_and_windows(trial_dir, bag_start_abs_sec, bag_duration):
    events = load_fault_log(trial_dir)
    kind = None
    for e in events:
        if e.get('event') == 'startup':
            if e.get('tl_fault'):
                kind = 'tl'
            elif e.get('imu_fault'):
                kind = 'imu'
            break
    if kind is None:
        return None, []
    return kind, extract_fault_windows(events, bag_start_abs_sec, bag_duration, kind)


def nearest_gt_xy(t, gt_positions):
    """gt_positions: sorted (t, x, y) list."""
    if not gt_positions:
        return None
    times = np.array([p[0] for p in gt_positions])
    idx = int(np.searchsorted(times, t))
    idx = min(max(idx, 0), len(gt_positions) - 1)
    return gt_positions[idx][1], gt_positions[idx][2]


def classify_outcome(result):
    return 'goal_reached' if result.get('status') == 'goal_reached' else 'fatal'


def build_feats_from_mc(mc, gt_tl):
    """Same feature dict compare_fault_vs_nominal.py's build_feature_series
    constructs, but reusing an already-read MetricsCollector/gt_tl pair
    instead of re-reading the bag a second time (bag deserialization, not
    plotting, is the slow part of this pipeline). Plus `tl_is_green` (added
    2026-07-25): tl_oscillate/tl_unknown corrupt the reported COLOR, not
    confidence — tl_confidence alone barely moves for those (oscillate's
    "GREEN-forced" half still reports high confidence, just a wrong color),
    so ranking against confidence-only made a color-forcing fault look like
    it had no discriminating feature at all. This closes that gap."""
    divergence = ekf_gt_divergence(mc, gt_tl['gt_positions'])
    tl_confidence = [(t, c) for t, _col, c, d in gt_tl['tl_events'] if d]
    tl_is_green = [(t, 1.0 if col == TrafficLightElement.GREEN else 0.0)
                   for t, col, _c, d in gt_tl['tl_events'] if d]
    return {
        'velocity_mps':        mc.velocities,
        'acceleration_mps2':   mc.accelerations,
        'steering_rad':        mc.steerings,
        'obj_distance_m':      mc.object_distances,
        'ekf_gt_divergence_m': divergence,
        'tl_confidence':       tl_confidence,
        'tl_is_green':         tl_is_green,
    }


def nominal_stats_and_series(goal_id, nominal_campaign):
    """Pooled (mean, std) per feature across nominal trials for this goal,
    PLUS each nominal trial's own raw series (for direct overlay, not just a
    band) — mirrors compare_fault_vs_nominal.py's own pooling loop."""
    nominal_dirs = sorted(glob.glob(os.path.join(DEFAULT_DATA_DIR, nominal_campaign, goal_id, 't*')))
    pool = {}
    series_by_trial = []
    for nd in nominal_dirs:
        bag_dir = os.path.join(nd, 'rosbag')
        if not os.path.isdir(bag_dir):
            continue
        mc = MetricsCollector(bag_dir)
        mc.read_bag()
        gt_tl = read_gt_and_tl(bag_dir)
        feats = build_feats_from_mc(mc, gt_tl)
        series_by_trial.append(feats)
        for name, s in feats.items():
            pool.setdefault(name, []).extend(v for _t, v in s)

    stats = {}
    for name, values in pool.items():
        arr = np.array(values, dtype=float)
        arr = arr[np.isfinite(arr)]
        if arr.size == 0:
            stats[name] = (0.0, 1.0)
        else:
            std = arr.std()
            stats[name] = (arr.mean(), std if std > 1e-6 else 1e-6)
    return stats, series_by_trial


def rank_features(feats, windows, bag_duration, nominal_stats):
    """Same recipe as compare_fault_vs_nominal.py's discriminability table:
    resample onto a 0.5s grid, z-score against pooled nominal stats, split
    in-fault vs out-of-fault by the matched windows, rank by how much more
    each feature deviates in-fault than out-of-fault. Returns feature names,
    most-discriminating first; features with no signal (or no windows to
    split against) are simply omitted."""
    if not windows:
        return []
    grid = np.arange(0, bag_duration, 0.5)
    in_mask = np.array([in_any_window(t, windows) for t in grid])
    if not in_mask.any() or in_mask.all():
        return []
    rows = []
    for name, series in feats.items():
        mean, std = nominal_stats.get(name, (0.0, 1.0))
        vals = resample(series, grid)
        z = np.abs((vals - mean) / std)
        valid = np.isfinite(z)
        in_z = z[valid & in_mask]
        out_z = z[valid & ~in_mask]
        if not in_z.size or not out_z.size:
            continue
        rows.append((name, in_z.mean() - out_z.mean()))
    rows.sort(key=lambda r: -r[1])
    return [name for name, _delta in rows]


def choose_panel_features(feats, ranked, n=3):
    panels = [name for name in ranked if feats.get(name)][:n]
    if len(panels) < n:
        for name in feats:
            if name not in panels and feats.get(name):
                panels.append(name)
            if len(panels) == n:
                break
    return panels


def find_trial_dirs(campaign, goal, trial=None):
    base = os.path.join(DEFAULT_DATA_DIR, campaign, goal)
    pattern = f't{trial}_*' if trial else 't*'
    return sorted(glob.glob(os.path.join(base, pattern)))


def plot_trial(map_data, campaign, goal_id, goal_xy, trial_dir, nominal_stats,
                nominal_series_by_trial, output_dir, margin):
    trial_name = os.path.basename(trial_dir)
    bag_dir = os.path.join(trial_dir, 'rosbag')
    result = json.load(open(os.path.join(trial_dir, 'result.json')))
    metrics_d = json.load(open(os.path.join(trial_dir, 'metrics.json')))

    mc = MetricsCollector(bag_dir)
    mc.read_bag()
    gt_tl = read_gt_and_tl(bag_dir)
    gt = gt_tl['gt_positions']
    if not gt:
        print(f'  {trial_name}: no ground-truth points, skipping')
        return None

    feats = build_feats_from_mc(mc, gt_tl)
    kind, windows = fault_kind_and_windows(trial_dir, gt_tl['bag_start_abs_sec'], gt_tl['bag_duration'])
    triggers = mrm_trigger_times(mc.mrm_states)
    outcome = classify_outcome(result)
    stop_t = metrics_d.get('static_collision', {}).get('permanent_stop_time_s')

    ranked = rank_features(feats, windows, gt_tl['bag_duration'], nominal_stats)
    panel_features = choose_panel_features(feats, ranked, n=3)

    xs = [p[1] for p in gt]
    ys = [p[2] for p in gt]
    ts = [p[0] for p in gt]
    in_fault = [any(w['start'] <= t <= w['end'] for w in windows) for t in ts]

    fig = plt.figure(figsize=(15, 8.5))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.3, 1])
    ax_map = fig.add_subplot(gs[:, 0])
    axes_ts = [fig.add_subplot(gs[0, 1])]
    axes_ts.append(fig.add_subplot(gs[1, 1], sharex=axes_ts[0]))
    axes_ts.append(fig.add_subplot(gs[2, 1], sharex=axes_ts[0]))

    # ── Map panel ──────────────────────────────────────────────────────────
    all_x = xs + [SPAWN[0], goal_xy[0]]
    all_y = ys + [SPAWN[1], goal_xy[1]]
    xmin, xmax = min(all_x) - margin, max(all_x) + margin
    ymin, ymax = min(all_y) - margin, max(all_y) + margin

    for ll in map_data.laneletLayer:
        if ll_attr(ll, 'subtype') != 'road':
            continue
        if not bbox_hit(ll, xmin, xmax, ymin, ymax):
            continue
        ax_map.add_patch(Polygon(lanelet_polygon_xy(ll), closed=True,
                                  facecolor='#dddddd', edgecolor='#bbbbbb',
                                  linewidth=0.3, zorder=1))

    tl_pts = traffic_light_points(map_data, xmin, xmax, ymin, ymax)
    if tl_pts:
        ax_map.scatter([p[0] for p in tl_pts], [p[1] for p in tl_pts],
                        marker='s', color='#f1c40f', edgecolor='#333333',
                        linewidth=0.5, s=30, zorder=2, label='traffic light')

    ax_map.plot(xs, ys, color=TRAJ_COLOR, linewidth=1.5, alpha=0.6, zorder=3,
                label='trajectory')
    fx = [x for x, f in zip(xs, in_fault) if f]
    fy = [y for y, f in zip(ys, in_fault) if f]
    if fx:
        ax_map.scatter(fx, fy, color=FAULT_COLOR, s=10, zorder=4, label='fault active')

    for i, tt in enumerate(triggers):
        pos = nearest_gt_xy(tt, gt)
        if pos:
            ax_map.plot(*pos, marker='^', color=MRM_COLOR, markersize=10,
                        markeredgecolor='black', markeredgewidth=0.7, zorder=6,
                        label='MRM trigger' if i == 0 else None)

    ax_map.plot(*SPAWN, marker='o', color=TRAJ_COLOR, markersize=8, zorder=5, label='start')
    ax_map.plot(*goal_xy, marker='*', color='black', markersize=13, zorder=5, label='goal')

    style = OUTCOME_STYLE[outcome]
    ax_map.plot(xs[-1], ys[-1], marker=style['marker'], color=style['color'],
                markersize=9, markeredgewidth=1.5, zorder=7, label=style['label'])

    ax_map.set_xlim(xmin, xmax)
    ax_map.set_ylim(ymin, ymax)
    ax_map.set_aspect('equal')
    ax_map.set_xlabel('map x (m)')
    ax_map.set_ylabel('map y (m)')
    ax_map.legend(loc='best', fontsize=7)

    # ── Time-series panels: top-3 discriminating features for THIS fault ──
    def annotate(ax):
        for w in windows:
            ax.axvspan(w['start'], w['end'], color=FAULT_COLOR, alpha=0.15, zorder=0)
        for tt in triggers:
            ax.axvline(tt, color=MRM_COLOR, linestyle='--', linewidth=1.3, zorder=1)
        if stop_t is not None and result.get('status') != 'goal_reached':
            ax.axvline(stop_t, color='black', linestyle=':', linewidth=1.5, zorder=1)

    for panel_idx, (ax, feat_name) in enumerate(zip(axes_ts, panel_features)):
        for i, nfeats in enumerate(nominal_series_by_trial):
            nseries = nfeats.get(feat_name, [])
            if not nseries:
                continue
            nt = [p[0] for p in nseries]
            nv = [p[1] for p in nseries]
            ax.plot(nt, nv, color=NOMINAL_COLOR, linewidth=1.0, alpha=0.6, zorder=1,
                     label='nominal trial(s)' if i == 0 else None)

        series = feats.get(feat_name, [])
        ft = [p[0] for p in series]
        fv = [p[1] for p in series]
        ax.plot(ft, fv, color=TRAJ_COLOR, linewidth=1.3, zorder=3, label=trial_name)
        annotate(ax)
        rank_tag = f'  (rank #{panel_idx + 1})' if feat_name in ranked else '  (no clear fault signal)'
        ax.set_ylabel(FEATURE_LABELS.get(feat_name, feat_name) + rank_tag, fontsize=8)
        if feat_name in LOG_SCALE_FEATURES:
            ax.set_yscale('log')
        ax.tick_params(labelsize=8)

    axes_ts[0].legend(loc='upper right', fontsize=7, framealpha=0.9)
    axes_ts[-1].set_xlabel('time since trial start (s)', fontsize=9)

    fig.suptitle(
        f'{campaign} / {goal_id} / {trial_name}\n'
        f'status={result.get("status")}   outcome={outcome}   '
        f'mrm_triggers={len(triggers)}   fault_windows={len(windows)} ({kind or "none"})   '
        f'panels ranked by fault discriminability',
        fontsize=11,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.93])

    out_path = os.path.join(output_dir, f'{campaign}_{goal_id}_{trial_name}.png')
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'  {trial_name}: saved {out_path}  outcome={outcome}  '
          f'mrm_triggers={len(triggers)}  fault_windows={len(windows)}  '
          f'panels={panel_features}')
    return out_path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', required=True, help='e.g. imu_fault_scale, tl_fault_s3')
    ap.add_argument('--goal', required=True, help='e.g. goal_012')
    ap.add_argument('--trial', type=int, default=None, help='specific trial number; default = all trials')
    ap.add_argument('--nominal-campaign', default='nom_v11')
    ap.add_argument('--goals-file', default=DEFAULT_GOALS_FILE)
    ap.add_argument('--map-file', default=DEFAULT_MAP_FILE)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    ap.add_argument('--margin', type=float, default=40.0)
    args = ap.parse_args()

    goals_file = args.goals_file
    if not os.path.isabs(goals_file):
        goals_file = os.path.join(REPO_DIR, 'experiments', 'configs', goals_file)
    goals_data = json.load(open(goals_file))
    goal_entry = next((g for g in goals_data['goals'] if g['id'] == args.goal), None)
    if goal_entry is None:
        print(f'{args.goal} not found in {goals_file}', file=sys.stderr)
        sys.exit(1)
    goal_xy = (goal_entry['goal']['position']['x'], goal_entry['goal']['position']['y'])

    trial_dirs = find_trial_dirs(args.campaign, args.goal, args.trial)
    if not trial_dirs:
        print(f'No trials found for {args.campaign}/{args.goal} under {DEFAULT_DATA_DIR}/{args.campaign}/',
              file=sys.stderr)
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    print(f'Loading lanelet2 map: {args.map_file}')
    map_data = load_map(args.map_file)

    print(f'Computing nominal baseline (all features) from {args.nominal_campaign}/{args.goal}...')
    nominal_stats, nominal_series_by_trial = nominal_stats_and_series(args.goal, args.nominal_campaign)

    for trial_dir in trial_dirs:
        plot_trial(map_data, args.campaign, args.goal, goal_xy, trial_dir,
                   nominal_stats, nominal_series_by_trial, args.output_dir, args.margin)


if __name__ == '__main__':
    main()
