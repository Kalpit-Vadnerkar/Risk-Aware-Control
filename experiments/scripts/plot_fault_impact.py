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

Map panel now also draws the campaign's PLANNED gating zones (added
2026-07-28) — the same zone circles/injection stars/fault-end markers
plot_fault_plan.py draws, reusing its own drawing functions directly rather
than re-implementing them, so a plan plot and its matching impact plot are
directly comparable side by side: did the fault actually arm where/when it
was supposed to, on THIS specific trial's real driven trajectory. Zone
geometry comes from the goal's nominal-campaign route reconstruction (same
source plot_fault_plan.py uses), not the trial's own bag — routes are
deterministic per goal, and reusing the nominal reconstruction keeps this
consistent with the rest of the pipeline instead of re-deriving it per
trial. Campaigns not in plot_fault_plan.py's CAMPAIGN_ZONE_KIND (e.g. a
future campaign not yet wired into the plan plot) simply get no zone
overlay — this degrades gracefully, not an error.

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
    make_projector, load_map, ll_attr, lanelet_polygon_xy, bbox_hit, SPAWN,
)
from compare_fault_vs_nominal import (  # noqa: E402
    read_gt_and_tl, ekf_gt_divergence, load_fault_log, extract_fault_windows,
    resample, in_any_window,
)
from compute_turn_zones import (  # noqa: E402
    find_first_bag, read_route_ids, build_route_polyline, resample_by_arc_length,
    RESAMPLE_STEP_M,
)
from fault_injector import (  # noqa: E402
    _DEFAULT_TL_ZONE_RADIUS_M, _DEFAULT_IMU_TURN_ZONE_RADIUS_M, _DEFAULT_IMU_LEADIN_ZONE_RADIUS_M,
)
from plot_fault_plan import (  # noqa: E402
    CAMPAIGN_ZONE_KIND, CAMPAIGN_FAULT_END, draw_zones_with_reachability, draw_fault_end_points,
    first_entry_point_on_route,
    TL_COLOR, TURN_COLOR, LEADIN_COLOR, FAULT_END_COLOR,
    DEFAULT_TURN_ZONES_FILE, DEFAULT_TL_ZONES_FILE,
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
    'tl_detected':         'TL detected at all (0/1)',
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
    it had no discriminating feature at all. This closes that gap. Plus
    `tl_detected` (added 2026-07-28, unfiltered unlike the two above):
    tl_fault_s4 (blackout) empties the group's elements entirely, leaving
    almost no in-fault samples for tl_confidence/tl_is_green to compare —
    "was anything detected" needs to be measurable at every timestamp, not
    just when something was seen, or the resampler quietly falls back to
    nearby out-of-fault samples and hides the effect."""
    divergence = ekf_gt_divergence(mc, gt_tl['gt_positions'])
    tl_confidence = [(t, c) for t, _col, c, d in gt_tl['tl_events'] if d]
    tl_is_green = [(t, 1.0 if col == TrafficLightElement.GREEN else 0.0)
                   for t, col, _c, d in gt_tl['tl_events'] if d]
    tl_detected = [(t, 1.0 if d else 0.0) for t, _col, _c, d in gt_tl['tl_events']]
    return {
        'velocity_mps':        mc.velocities,
        'acceleration_mps2':   mc.accelerations,
        'steering_rad':        mc.steerings,
        'obj_distance_m':      mc.object_distances,
        'ekf_gt_divergence_m': divergence,
        'tl_confidence':       tl_confidence,
        'tl_is_green':         tl_is_green,
        'tl_detected':         tl_detected,
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
        gt_tl = read_gt_and_tl(bag_dir, goal_id)
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


def draw_planned_zones(ax, map_data, campaign, goal_id, goal_xy, zones, tl_zones_for_goal, nominal_campaign):
    """Overlay this campaign's PLANNED gating zones (same circles/injection
    stars/fault-end markers plot_fault_plan.py draws — reused directly, not
    reimplemented) onto an already-built map panel, so a trial's real
    trajectory can be checked directly against where the fault was SUPPOSED
    to arm. Silently draws nothing for campaigns plot_fault_plan.py doesn't
    know about (e.g. nominal) — this is a bonus overlay, never fatal to the
    impact plot itself.
    """
    zone_kind, _fault_label = CAMPAIGN_ZONE_KIND.get(campaign, (None, None))
    if zone_kind is None or zone_kind == 'none':
        return

    bag_dir = find_first_bag(nominal_campaign, goal_id)
    if bag_dir is None:
        print(f'  {goal_id}: no {nominal_campaign} trial to reconstruct the planned route from '
              f'— skipping zone overlay', file=sys.stderr)
        return
    route_ids = read_route_ids(bag_dir, goal_xy)
    if route_ids is None:
        print(f'  {goal_id}: no route message found in {nominal_campaign} bag — skipping zone overlay',
              file=sys.stderr)
        return
    route_pts = build_route_polyline(map_data, route_ids)
    resampled = resample_by_arc_length(route_pts, RESAMPLE_STEP_M)

    if zone_kind == 'tl':
        tl_zone_centers = [(z['x'], z['y']) for z in tl_zones_for_goal]
        tl_reachable = [z.get('reachable', True) for z in tl_zones_for_goal]
        tl_group_ids = [z['group_id'] for z in tl_zones_for_goal]
        tl_injection_pts = [first_entry_point_on_route(resampled, c, _DEFAULT_TL_ZONE_RADIUS_M)
                             for c in tl_zone_centers]
        tl_zones_xyr = [(x, y, r) for (x, y), r in zip(tl_zone_centers, tl_reachable)]
        draw_zones_with_reachability(ax, tl_zones_xyr, tl_injection_pts,
                                      _DEFAULT_TL_ZONE_RADIUS_M, TL_COLOR,
                                      f'planned TL zone (r={_DEFAULT_TL_ZONE_RADIUS_M:.0f}m)',
                                      'planned')
        for (x, y), gid, reachable in zip(tl_zone_centers, tl_group_ids, tl_reachable):
            if reachable:
                ax.annotate(str(gid), (x, y), textcoords='offset points', xytext=(6, 6), fontsize=6)

    elif zone_kind == 'turn':
        turn_pts = [(p['x'], p['y'], p.get('reachable', True)) for p in zones.get('turn_zones', [])]
        turn_injection_pts = [first_entry_point_on_route(resampled, (x, y), _DEFAULT_IMU_TURN_ZONE_RADIUS_M)
                               for x, y, _r in turn_pts]
        draw_zones_with_reachability(ax, turn_pts, turn_injection_pts,
                                      _DEFAULT_IMU_TURN_ZONE_RADIUS_M, TURN_COLOR,
                                      f'planned IMU turn zone (r={_DEFAULT_IMU_TURN_ZONE_RADIUS_M:.0f}m)',
                                      'planned')
        # Real end point (2026-07-28 fix) — see plot_fault_plan.py's own
        # turn_exit_pts comment for why this replaced the old "left the
        # entry radius" derivation.
        turn_end_pts_all = [((p['end_x'], p['end_y']) if 'end_x' in p else None)
                             for p in zones.get('turn_zones', [])]
        turn_exit_pts = [end_xy for (x, y, r), end_xy in zip(turn_pts, turn_end_pts_all) if r]
        draw_fault_end_points(ax, turn_exit_pts, FAULT_END_COLOR, 'planned fault end (zone exit)')

    elif zone_kind == 'leadin':
        leadin_pts = [(p['x'], p['y'], p.get('reachable', True)) for p in zones.get('bias_leadin_zones', [])]
        leadin_injection_pts = [first_entry_point_on_route(resampled, (x, y), _DEFAULT_IMU_LEADIN_ZONE_RADIUS_M)
                                 for x, y, _r in leadin_pts]
        draw_zones_with_reachability(ax, leadin_pts, leadin_injection_pts,
                                      _DEFAULT_IMU_LEADIN_ZONE_RADIUS_M, LEADIN_COLOR,
                                      f'planned IMU bias lead-in zone (r={_DEFAULT_IMU_LEADIN_ZONE_RADIUS_M:.0f}m)',
                                      'planned')


def plot_trial(map_data, campaign, goal_id, goal_xy, trial_dir, nominal_stats,
                nominal_series_by_trial, output_dir, margin, zones, tl_zones_for_goal, nominal_campaign):
    trial_name = os.path.basename(trial_dir)
    bag_dir = os.path.join(trial_dir, 'rosbag')
    result = json.load(open(os.path.join(trial_dir, 'result.json')))
    metrics_d = json.load(open(os.path.join(trial_dir, 'metrics.json')))

    mc = MetricsCollector(bag_dir)
    mc.read_bag()
    gt_tl = read_gt_and_tl(bag_dir, goal_id)
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

    # No raw all-lights scatter here (removed 2026-07-28) — draw_planned_zones
    # below already renders one marker per TL zone (matching plot_fault_plan.py's
    # convention), which is the relevant subset; every individual physical light
    # in the bbox was cluttering the map with positions the fault never targets.
    draw_planned_zones(ax_map, map_data, campaign, goal_id, goal_xy, zones, tl_zones_for_goal, nominal_campaign)

    ax_map.plot(xs, ys, color=TRAJ_COLOR, linewidth=1.5, alpha=0.6, zorder=3,
                label='trajectory')
    fx = [x for x, f in zip(xs, in_fault) if f]
    fy = [y for y, f in zip(ys, in_fault) if f]
    if fx:
        ax_map.scatter(fx, fy, color=FAULT_COLOR, s=10, zorder=4, label='fault active')

    # MRM trigger markers deliberately NOT plotted (removed 2026-07-28) — live
    # data showed the MRM state machine flaps NORMAL<->MRM_OPERATING dozens of
    # times per trial even under nominal (unfaulted) driving, self-recovering
    # in ~100ms each time (see docs/research_notes/ for the investigation).
    # Plotting every blip implied a significance individual triggers don't
    # have; deciding how to represent REAL sustained MRM engagement (vs. this
    # noise) is deliberately deferred, not solved by omission.

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
        # MRM trigger vertical lines deliberately not drawn — see the map
        # panel's comment above.
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
    ap.add_argument('--turn-zones-file', default=DEFAULT_TURN_ZONES_FILE)
    ap.add_argument('--tl-zones-file', default=DEFAULT_TL_ZONES_FILE)
    args = ap.parse_args()

    zones_data = json.load(open(args.turn_zones_file))
    tl_zones_data = json.load(open(args.tl_zones_file))
    zones = zones_data['goals'].get(args.goal, {})
    tl_zones_for_goal = tl_zones_data['goals'].get(args.goal, {}).get('tl_zones', [])

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
                   nominal_stats, nominal_series_by_trial, args.output_dir, args.margin,
                   zones, tl_zones_for_goal, args.nominal_campaign)


if __name__ == '__main__':
    main()
