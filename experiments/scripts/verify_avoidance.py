#!/usr/bin/env python3
"""
Trajectory verification: did avoidance maneuvers actually happen in obs_recovery runs?

For each goal (007, 011, 021), plots:
  1. 2D trajectory map (all runs overlaid, colored by condition)
  2. Cross-track deviation vs arc distance (lateral profile)

A lane change shows as a sustained ~3m cross-track excursion at the
obstacle placement zone (~150-200m arc distance).

Output: /tmp/avoidance_verification/
"""

import os, sys, json, glob, math
import numpy as np
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

DATA_DIR = Path("/home/df/Desktop/Kalpit-2026/Risk-Aware-Control/experiments/data")
OUTPUT_DIR = Path("/tmp/avoidance_verification")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

TOPIC_GROUND_TRUTH = '/awsim/ground_truth/localization/kinematic_state'
OBSTACLE_ZONE_START = 130   # m arc distance where obstacle could appear
OBSTACLE_ZONE_END   = 250   # m arc distance (30m ahead placement + avoidance distance)
LANE_WIDTH = 3.5            # m — Shinjuku lanes; excursion > this = lane change

@dataclass
class RunData:
    run_id: str
    goal_id: str
    condition: str          # 'baseline', 'obs_recovery', 'obs_noescape', 'obs_stuck'
    outcome: str            # 'goal_reached', 'stuck', 'mrm_mrm_succeeded', 'engage_failed'
    positions: np.ndarray   # shape (N, 2), columns: x, y
    arc_dist: np.ndarray    # shape (N,), cumulative arc distance in metres
    cross_track: Optional[np.ndarray] = None   # computed later against reference


# ── Rosbag reading ─────────────────────────────────────────────────────────

def read_positions(bag_dir: Path) -> np.ndarray:
    """Return (N, 2) array of ground-truth x,y from the rosbag."""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    pts = []
    with Reader(bag_dir) as reader:
        conns = [c for c in reader.connections if c.topic == TOPIC_GROUND_TRUTH]
        if not conns:
            return np.empty((0, 2))
        for conn, _, raw in reader.messages(connections=conns):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            pts.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
    return np.array(pts)


def arc_distance(positions: np.ndarray) -> np.ndarray:
    """Cumulative arc distance along the trajectory."""
    if len(positions) < 2:
        return np.zeros(len(positions))
    steps = np.sqrt(np.sum(np.diff(positions, axis=0) ** 2, axis=1))
    return np.concatenate([[0], np.cumsum(steps)])


# ── Cross-track deviation ──────────────────────────────────────────────────

def compute_cross_track(test_xy: np.ndarray, ref_xy: np.ndarray,
                        ref_arc: np.ndarray) -> np.ndarray:
    """
    For each point in test_xy, find the nearest point on the reference
    trajectory and return the signed perpendicular (cross-track) distance.
    Positive = left of reference route.
    """
    ct = np.full(len(test_xy), np.nan)
    for i, pt in enumerate(test_xy):
        dists = np.sqrt(np.sum((ref_xy - pt) ** 2, axis=1))
        j = int(np.argmin(dists))
        # Build local tangent from neighbours
        j0 = max(j - 2, 0)
        j1 = min(j + 2, len(ref_xy) - 1)
        tangent = ref_xy[j1] - ref_xy[j0]
        norm = np.linalg.norm(tangent)
        if norm < 1e-6:
            continue
        tangent /= norm
        perp = np.array([-tangent[1], tangent[0]])   # 90° left
        ct[i] = float(np.dot(pt - ref_xy[j], perp))
    return ct


# ── Data loading ───────────────────────────────────────────────────────────

def load_dataset(condition: str) -> List[RunData]:
    dataset_dir = DATA_DIR / condition
    if not dataset_dir.exists():
        return []

    runs = []
    for run_dir in sorted(dataset_dir.iterdir()):
        if not run_dir.is_dir():
            continue
        result_f  = run_dir / 'result.json'
        meta_f    = run_dir / 'metadata.json'
        bag_dir   = run_dir / 'rosbag'
        if not all(p.exists() for p in [result_f, meta_f, bag_dir]):
            continue

        with open(result_f)  as f: result   = json.load(f)
        with open(meta_f)    as f: metadata = json.load(f)

        goal_id = metadata.get('goal_id', 'unknown')
        outcome = result.get('status', 'unknown')

        try:
            positions = read_positions(bag_dir)
        except Exception as e:
            print(f"  SKIP {run_dir.name}: {e}")
            continue

        if len(positions) < 10:
            continue

        arc = arc_distance(positions)
        runs.append(RunData(
            run_id    = run_dir.name,
            goal_id   = goal_id,
            condition = condition,
            outcome   = outcome,
            positions = positions,
            arc_dist  = arc,
        ))

    return runs


# ── Analysis & plotting ────────────────────────────────────────────────────

CONDITION_STYLE = {
    # (color, linestyle, alpha, zorder)
    'baseline':     ('#2196F3', '-',  0.55, 2),   # blue
    'obs_recovery': (None,      '-',  0.85, 3),   # outcome-coloured
    'obs_noescape': ('#FF9800', '-',  0.70, 2),   # orange
    'obs_stuck':    ('#9E9E9E', '--', 0.50, 1),   # grey
}
OUTCOME_COLOR = {
    'goal_reached':      '#4CAF50',   # green
    'stuck':             '#F44336',   # red
    'mrm_mrm_succeeded': '#E91E63',   # pink-red
    'engage_failed':     '#9C27B0',   # purple
}


def run_color(run: RunData) -> str:
    if run.condition == 'obs_recovery':
        return OUTCOME_COLOR.get(run.outcome, '#607D8B')
    return CONDITION_STYLE[run.condition][0]


def plot_goal(goal_id: str, all_runs: List[RunData]):
    goal_runs = [r for r in all_runs if r.goal_id == goal_id]
    if not goal_runs:
        return

    baseline_runs  = [r for r in goal_runs if r.condition == 'baseline']
    recovery_runs  = [r for r in goal_runs if r.condition == 'obs_recovery']
    noescape_runs  = [r for r in goal_runs if r.condition == 'obs_noescape']

    # Pick reference trajectory: longest successful baseline run
    ref_candidates = [r for r in baseline_runs if r.outcome == 'goal_reached']
    if not ref_candidates:
        ref_candidates = baseline_runs
    if not ref_candidates:
        print(f"  No baseline for {goal_id}, skipping cross-track")
        ref = None
    else:
        ref = max(ref_candidates, key=lambda r: r.arc_dist[-1])

    # Compute cross-track for all non-baseline runs
    if ref is not None:
        for r in goal_runs:
            if r is not ref:
                r.cross_track = compute_cross_track(r.positions, ref.positions, ref.arc_dist)

    # ── Figure setup: 2 rows × 2 cols ──────────────────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(f'Trajectory Verification — {goal_id.upper()}\n'
                 f'Recovery: {len(recovery_runs)} runs  |  '
                 f'No-escape: {len(noescape_runs)} runs  |  '
                 f'Baseline: {len(baseline_runs)} runs',
                 fontsize=13, y=0.98)

    ax_map      = axes[0, 0]   # 2D trajectory map
    ax_ct_recov = axes[0, 1]   # cross-track: recovery runs
    ax_ct_noescape = axes[1, 0]  # cross-track: noescape runs
    ax_stats    = axes[1, 1]   # summary bar chart

    # Obstacle zone shading helper
    def shade_zone(ax, vertical=True):
        if vertical:
            ax.axvspan(OBSTACLE_ZONE_START, OBSTACLE_ZONE_END,
                       alpha=0.12, color='red', label='Obstacle zone')
        else:
            ax.axhspan(OBSTACLE_ZONE_START, OBSTACLE_ZONE_END,
                       alpha=0.12, color='red')
        ax.axhline(LANE_WIDTH, color='k', ls=':', lw=0.8, alpha=0.5)
        ax.axhline(-LANE_WIDTH, color='k', ls=':', lw=0.8, alpha=0.5)
        ax.axhline(0, color='grey', ls='-', lw=0.5, alpha=0.4)

    # ── 2D Map ──────────────────────────────────────────────────────────────
    for r in goal_runs:
        c   = run_color(r)
        ls  = CONDITION_STYLE.get(r.condition, ('grey', '-', 0.5, 1))
        lw  = 1.4 if r.condition == 'obs_recovery' else 0.9
        ax_map.plot(r.positions[:, 0], r.positions[:, 1],
                    color=c, ls=ls[1], alpha=ls[2], lw=lw, zorder=ls[3])

    # Mark spawn point
    if goal_runs:
        sp = goal_runs[0].positions[0]
        ax_map.plot(sp[0], sp[1], 'k^', ms=8, zorder=5, label='Spawn')

    ax_map.set_aspect('equal')
    ax_map.set_xlabel('X (m)')
    ax_map.set_ylabel('Y (m)')
    ax_map.set_title('2D Trajectory Map')

    # ── Cross-track: recovery runs ──────────────────────────────────────────
    shade_zone(ax_ct_recov)
    max_excursions_recov = []
    for r in recovery_runs:
        if r.cross_track is None:
            continue
        c = run_color(r)
        label = f"{r.run_id.split('_')[2].replace('t','T')} ({r.outcome[:10]})"
        ax_ct_recov.plot(r.arc_dist, r.cross_track, color=c, alpha=0.8, lw=1.2, label=label)
        # Max excursion in obstacle zone
        mask = (r.arc_dist >= OBSTACLE_ZONE_START) & (r.arc_dist <= OBSTACLE_ZONE_END)
        zone_ct = r.cross_track[mask]
        zone_ct = zone_ct[~np.isnan(zone_ct)]
        max_ex = float(np.max(np.abs(zone_ct))) if len(zone_ct) > 0 else 0.0
        max_excursions_recov.append((r.run_id, r.outcome, max_ex))

    ax_ct_recov.set_xlim(0, 500)
    ax_ct_recov.set_ylim(-8, 8)
    ax_ct_recov.set_xlabel('Arc distance (m)')
    ax_ct_recov.set_ylabel('Cross-track deviation (m)')
    ax_ct_recov.set_title('obs_recovery — Cross-track vs Route Distance')
    ax_ct_recov.legend(fontsize=6, loc='upper right', ncol=2)

    # ── Cross-track: noescape runs ──────────────────────────────────────────
    shade_zone(ax_ct_noescape)
    max_excursions_noescape = []
    for r in noescape_runs:
        if r.cross_track is None:
            continue
        c = run_color(r)
        ax_ct_noescape.plot(r.arc_dist, r.cross_track, color=c, alpha=0.7, lw=1.0)
        mask = (r.arc_dist >= OBSTACLE_ZONE_START) & (r.arc_dist <= OBSTACLE_ZONE_END)
        zone_ct = r.cross_track[mask]
        zone_ct = zone_ct[~np.isnan(zone_ct)]
        max_ex = float(np.max(np.abs(zone_ct))) if len(zone_ct) > 0 else 0.0
        max_excursions_noescape.append((r.run_id, r.outcome, max_ex))

    ax_ct_noescape.set_xlim(0, 500)
    ax_ct_noescape.set_ylim(-8, 8)
    ax_ct_noescape.set_xlabel('Arc distance (m)')
    ax_ct_noescape.set_ylabel('Cross-track deviation (m)')
    ax_ct_noescape.set_title('obs_noescape — Cross-track vs Route Distance')

    # ── Summary bar chart ──────────────────────────────────────────────────
    all_excursions = [(rid, cond, oc, ex) for (rid, oc, ex) in max_excursions_recov
                       for cond in ['obs_recovery']] + \
                     [(rid, 'obs_noescape', oc, ex) for (rid, oc, ex) in max_excursions_noescape]

    if all_excursions:
        labels = [f"{e[0].split('_')[2]} ({e[1][:5]})" for e in all_excursions]
        values = [e[3] for e in all_excursions]
        colors = [OUTCOME_COLOR.get(e[2], '#607D8B') if e[1] == 'obs_recovery'
                  else '#FF9800' for e in all_excursions]
        bars = ax_stats.barh(range(len(labels)), values, color=colors, edgecolor='white', height=0.7)
        ax_stats.set_yticks(range(len(labels)))
        ax_stats.set_yticklabels(labels, fontsize=7)
        ax_stats.axvline(LANE_WIDTH, color='red', ls='--', lw=1.2, label=f'Lane width ({LANE_WIDTH}m)')
        ax_stats.set_xlabel('Max |cross-track| in obstacle zone (m)')
        ax_stats.set_title('Obstacle Zone Lateral Excursion per Run')
        ax_stats.legend(fontsize=8)

    # ── Legend for map ──────────────────────────────────────────────────────
    legend_elements = [
        mpatches.Patch(color='#2196F3', label='Baseline (nominal)'),
        mpatches.Patch(color='#4CAF50', label='obs_recovery — goal_reached'),
        mpatches.Patch(color='#F44336', label='obs_recovery — stuck'),
        mpatches.Patch(color='#E91E63', label='obs_recovery — mrm_succeeded'),
        mpatches.Patch(color='#FF9800', label='obs_noescape'),
        mpatches.Patch(color='#9E9E9E', label='obs_stuck'),
    ]
    ax_map.legend(handles=legend_elements, fontsize=7, loc='lower right')

    plt.tight_layout()
    out_path = OUTPUT_DIR / f'verify_{goal_id}.png'
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {out_path}")

    # ── Console summary ────────────────────────────────────────────────────
    print(f"\n  Cross-track in obstacle zone ({OBSTACLE_ZONE_START}–{OBSTACLE_ZONE_END}m):")
    print(f"  {'Run':<55} {'Outcome':<22} {'Max |CT| (m)':>12}  {'Lane change?':>12}")
    print(f"  {'-'*105}")
    for rid, oc, ex in sorted(max_excursions_recov, key=lambda x: -x[2]):
        flag = '✓ YES' if ex >= LANE_WIDTH else '✗ no'
        print(f"  {rid:<55} {oc:<22} {ex:>12.2f}  {flag:>12}")
    if max_excursions_recov:
        print(f"  {'(obs_noescape)':<55}")
    for rid, oc, ex in sorted(max_excursions_noescape, key=lambda x: -x[2]):
        flag = '✓ YES' if ex >= LANE_WIDTH else '✗ no'
        print(f"  {rid:<55} {oc:<22} {ex:>12.2f}  {flag:>12}")

    return max_excursions_recov, max_excursions_noescape


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    print("Loading datasets...")
    conditions = ['baseline_all', 'obs_recovery', 'obs_noescape', 'obs_stuck']
    condition_map = {
        'baseline_all': 'baseline',
        'obs_recovery': 'obs_recovery',
        'obs_noescape': 'obs_noescape',
        'obs_stuck':    'obs_stuck',
    }

    all_runs = []
    for ds in conditions:
        print(f"  Loading {ds}...")
        runs = load_dataset(ds)
        for r in runs:
            r.condition = condition_map[ds]
        all_runs.extend(runs)
        print(f"    {len(runs)} runs loaded")

    goals = sorted(set(r.goal_id for r in all_runs
                       if r.goal_id in ['goal_007', 'goal_011', 'goal_021']))
    print(f"\nAnalysing goals: {goals}")

    all_recov_excursions = []
    all_noescape_excursions = []

    for goal in goals:
        print(f"\n{'='*60}")
        print(f"Goal: {goal}")
        print('='*60)
        r_ex, n_ex = plot_goal(goal, all_runs)
        all_recov_excursions.extend(r_ex)
        all_noescape_excursions.extend(n_ex)

    # ── Overall summary ────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("OVERALL SUMMARY")
    print('='*60)

    recov_excursions = [ex for _, _, ex in all_recov_excursions]
    noescape_excursions = [ex for _, _, ex in all_noescape_excursions]

    recov_lane_changes = sum(1 for ex in recov_excursions if ex >= LANE_WIDTH)
    recov_goal_reached = [ex for rid, oc, ex in all_recov_excursions if oc == 'goal_reached']

    print(f"\nobs_recovery runs with cross-track data: {len(recov_excursions)}")
    print(f"  Lane changes detected (excursion >= {LANE_WIDTH}m): "
          f"{recov_lane_changes}/{len(recov_excursions)}")
    if recov_excursions:
        print(f"  Max excursion: {max(recov_excursions):.2f}m")
        print(f"  Median excursion: {np.median(recov_excursions):.2f}m")
    if recov_goal_reached:
        print(f"  Among goal_reached runs: "
              f"lane change in {sum(1 for ex in recov_goal_reached if ex >= LANE_WIDTH)}"
              f"/{len(recov_goal_reached)}")
        print(f"  Max excursion (goal_reached): {max(recov_goal_reached):.2f}m")

    print(f"\nobs_noescape runs with cross-track data: {len(noescape_excursions)}")
    if noescape_excursions:
        print(f"  Lane changes detected: "
              f"{sum(1 for ex in noescape_excursions if ex >= LANE_WIDTH)}/{len(noescape_excursions)}")
        print(f"  Max excursion: {max(noescape_excursions):.2f}m")
        print(f"  (expected ~0 — vehicle should stop, not swerve)")

    print(f"\nPlots saved to: {OUTPUT_DIR}/")


if __name__ == '__main__':
    main()
