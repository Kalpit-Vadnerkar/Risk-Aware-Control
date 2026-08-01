#!/usr/bin/env python3
"""
Per-goal, per-CAMPAIGN fault-injection PLAN visualization: where THIS
campaign's fault is supposed to arm, computed purely from map/route geometry
and turn_zones.json — no rosbag / collected trial required. Companion to
plot_fault_impact.py, which shows what ACTUALLY happened in a specific
collected trial; this shows the plan those trials are checked against.

One campaign runs one fault type (collect.sh), so the plan plot for a
campaign only draws the zone kind that fault actually gates on:
  - tl_fault_s2/s3/s4/ramp   -> TL zone points (r=_DEFAULT_TL_ZONE_RADIUS_M)
  - imu_fault_scale/stuck    -> IMU turn zones (r=_DEFAULT_IMU_TURN_ZONE_RADIUS_M)
  - imu_fault_s1/s3          -> IMU bias lead-in zones (r=_DEFAULT_IMU_LEADIN_ZONE_RADIUS_M)
  - imu_fault_ramp           -> no zone gating (imu_bias_ramp accumulates
    continuously from runway-clear regardless of geometry) — the route from
    the runway-clear point onward is highlighted to show it explicitly
    includes straight segments, not just turns.

Every plot also marks the RUNWAY-CLEAR point: the earliest position ANY
fault (TL or IMU) is allowed to arm at all, computed the same way
fault_injector.py's own runway gate does (map-zone-based: first real TL zone
entered and then exited; flat 150m fallback otherwise) — a zone that would
fall before this point could never actually arm on a trial's first pass
through it, which is worth being able to see, not just assume away.

Fault END (de-injection) markers added 2026-07-28, alongside the existing
start/injection markers — see docs/fault_scenario_table.md's "fault start" /
"fault end" columns for the underlying design this mirrors:
  - `imu_fault_scale`/`imu_fault_stuck` (zone_kind='turn') end on turn-zone
    EXIT (`fault_injector.py`'s `_wait_for_zone_transit_end`) — a real route
    position, computed here the same way as the entry marker: the first
    point walking forward, after having entered, where the route leaves the
    zone radius again. Drawn as a distinct marker so a plan plot alone shows
    both ends of the on-window.
  - All fixed-timer fault types (`tl_fault_*`, `imu_fault_s1`/`s3`) end after
    a real elapsed-time cap, not a route position — where that lands on the
    route depends on the trial's actual driven speed, which this
    geometry-only plan plot does not have. Rather than draw a marker at a
    guessed position, these get a text note instead (see CAMPAIGN_FAULT_END).
  - `imu_fault_ramp` has no discrete "end" at all (continuous one-shot ramp,
    see docs/fault_scenario_table.md's "Not currently in scope") — no end
    marker or note drawn for it.

Background is the planned ROUTE's own lanelet geometry (same source
compute_turn_zones.py computes zones from), not a driven GT trajectory —
consistent with the rest of this pipeline. TL zones are read directly from
this goal's own entry in tl_zones.json (compute_tl_zones.py) — one zone per
real traffic_light regulatory element the route's lanelets are actually
governed by, each labeled with its real traffic_light_group_id, matching
exactly what fault_injector.py now uses to scope a TL fault to the one
light the vehicle is reacting to (see fault_injector.py's
_tl_fault_group_id). This replaced an earlier version that filtered/
clustered the cruder bbox-based TL points by proximity to the route — that
was a real improvement over showing every goal's pooled TLs, but was still
showing lights the vehicle's own lane isn't governed by (e.g. a cross-
street's signal within the proximity threshold); reading straight from the
route's own regulatory elements is exact, not a heuristic.

Usage (must source ROS/Autoware, then the repo venv):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/plot_fault_plan.py --campaign tl_fault_s2
  python3 experiments/scripts/plot_fault_plan.py --campaign imu_fault_scale --goals goal_012
"""

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from plotting import (  # noqa: E402
    load_map, ll_attr, lanelet_polygon_xy, bbox_hit, draw_map_background,
    draw_injection_points, draw_fault_end_points, draw_zones_with_reachability,
    TL_COLOR, TURN_COLOR, LEADIN_COLOR, LANE_CHANGE_COLOR, RUNWAY_COLOR,
    INJECTION_COLOR, FAULT_END_COLOR, ROUTE_COLOR, UNREACHABLE_COLOR,
)
from compute_turn_zones import (  # noqa: E402
    find_first_bag, read_route_ids, build_route_polyline, resample_by_arc_length,
    first_entry_point_on_route, RESAMPLE_STEP_M, DEFAULT_GOALS_FILE,
)
from fault_injector import (  # noqa: E402
    _load_tl_group_zones, _DEFAULT_ZONE_GOALS, _DEFAULT_TL_ZONE_RADIUS_M,
    _DEFAULT_IMU_TURN_ZONE_RADIUS_M, _DEFAULT_IMU_LEADIN_ZONE_RADIUS_M,
)

DEFAULT_MAP_FILE = os.path.join(os.path.dirname(REPO_DIR), 'Map',
                                 'nishishinjuku_autoware_map', 'lanelet2_map.osm')
DEFAULT_TURN_ZONES_FILE = os.path.join(REPO_DIR, 'experiments', 'configs', 'turn_zones.json')
DEFAULT_TL_ZONES_FILE = os.path.join(REPO_DIR, 'experiments', 'configs', 'tl_zones.json')
DEFAULT_NOMINAL_CAMPAIGN = 'nom_v11'
DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'fault_plan')
FLAT_RUNWAY_FALLBACK_M = 150.0    # matches fault_injector.py's --fault-min-runway-m default

# Zone/route colors (TL_COLOR, TURN_COLOR, LEADIN_COLOR, LANE_CHANGE_COLOR,
# RUNWAY_COLOR, INJECTION_COLOR, FAULT_END_COLOR, ROUTE_COLOR,
# UNREACHABLE_COLOR) now live in experiments/lib/plotting.py — this was the
# only place that defined them; plot_fault_impact.py used to import them
# from here transitively rather than from a real shared module.

# campaign name -> (zone kind, fault mechanism blurb for the title)
CAMPAIGN_ZONE_KIND = {
    'tl_fault_s2':     ('tl', 'tl_oscillate'),
    'tl_fault_s3':     ('tl', 'tl_unknown'),
    'tl_fault_s4':     ('tl', 'tl_blackout'),
    'tl_fault_ramp':   ('tl', 'tl_confidence_ramp'),
    'imu_fault_scale': ('turn', 'imu_scale_factor'),
    'imu_fault_stuck':  ('turn', 'imu_stuck_at'),
    'imu_fault_s1':    ('leadin', 'imu_bias (0.03 rad/s)'),
    'imu_fault_s3':    ('leadin', 'imu_bias (0.08 rad/s)'),
    'imu_fault_ramp':  ('none', 'imu_bias_ramp'),
}

# campaign name -> ('zone_exit', None) | ('fixed_timer', seconds) | ('continuous', None)
# Matches docs/fault_scenario_table.md's "Fault end" column exactly — the
# on_seconds/fault-duration values here are collect.sh's actual --imu-params/
# --tl-params/--fault-duration values, not independently guessed.
CAMPAIGN_FAULT_END = {
    'tl_fault_s2':     ('fixed_timer', 15),
    'tl_fault_s3':     ('fixed_timer', 15),
    'tl_fault_s4':     ('fixed_timer', 15),
    'tl_fault_ramp':   ('fixed_timer', 15),
    'imu_fault_scale': ('zone_exit', None),
    'imu_fault_stuck': ('zone_exit', None),
    'imu_fault_s1':    ('fixed_timer', 20),
    'imu_fault_s3':    ('fixed_timer', 15),
    'imu_fault_ramp':  ('continuous', None),
}


def min_dist_to_points(x, y, points):
    if not points:
        return float('inf')
    return min(math.hypot(x - px, y - py) for px, py in points)


def compute_runway_clear_point(resampled, tl_points, tl_radius, flat_fallback_m):
    """Mirrors fault_injector.py's runway gate exactly (see _on_gt_pose):
    map-zone based if tl_points is non-empty (sticky 'seen a zone' then
    'exited it'), flat arc-length distance from route start otherwise.
    Ignores the live speed>=_RUNWAY_MIN_SPEED_MPS criterion — irrelevant for
    static route geometry, assumed satisfied once moving."""
    if tl_points:
        seen_zone = False
        was_in_zone = False
        for x, y, _lid in resampled:
            in_zone = min_dist_to_points(x, y, tl_points) <= tl_radius
            if in_zone:
                seen_zone = True
            if seen_zone and was_in_zone and not in_zone:
                return (x, y), 'map_zone'
            was_in_zone = in_zone
        # Route never exits a zone after entering one (e.g. TL right at the
        # very end) — no clean runway-clear point on this route.
        return (resampled[-1][0], resampled[-1][1]), 'map_zone_never_cleared'

    acc = 0.0
    x0, y0, _ = resampled[0]
    for x1, y1, _lid in resampled[1:]:
        if math.hypot(x1 - x0, y1 - y0) >= flat_fallback_m:
            return (x1, y1), 'flat_fallback'
    return (resampled[-1][0], resampled[-1][1]), 'flat_fallback_never_reached'


def plot_goal(map_data, campaign, zone_kind, fault_label, fault_end, goal, goal_xy, pooled_tl_group_zones,
              zones, tl_zones_for_goal, nominal_campaign, output_dir, margin):
    bag_dir = find_first_bag(nominal_campaign, goal)
    if bag_dir is None:
        print(f'{goal}: no {nominal_campaign} trials found, skipping', file=sys.stderr)
        return
    route_ids = read_route_ids(bag_dir, goal_xy)
    if route_ids is None:
        print(f'{goal}: no route message matching goal_pose found, skipping', file=sys.stderr)
        return
    route_pts = build_route_polyline(map_data, route_ids)
    resampled = resample_by_arc_length(route_pts, RESAMPLE_STEP_M)
    xs = [p[0] for p in resampled]
    ys = [p[1] for p in resampled]

    # (x, y, reachable) — reachable=False means at/before the runway-clear
    # point (see compute_turn_zones.py/compute_tl_zones.py): kept in the
    # data, not dropped, but can never actually arm a fault on this route.
    # curved_road_zones is intentionally NOT read here — it's excluded from
    # gating (see docs/fault_scenario_table.md) and, per 2026-07-27
    # feedback, a single point is the wrong representation for it anyway
    # (it's a stretch of route, not a discrete corner) — future work if it
    # ever becomes an active scenario should draw it as a highlighted
    # sub-segment of the route line, not a marker.
    turn_pts = [(p['x'], p['y'], p.get('reachable', True)) for p in zones.get('turn_zones', [])]
    # Each turn run's own real end point (added 2026-07-28 alongside
    # fault_injector.py's fix — see compute_turn_zones.py) — None for older
    # turn_zones.json files predating this field, handled as "never exits"
    # the same as the old geometric derivation did.
    turn_end_pts = [((p['end_x'], p['end_y']) if 'end_x' in p else None)
                     for p in zones.get('turn_zones', [])]
    leadin_pts = [(p['x'], p['y'], p.get('reachable', True)) for p in zones.get('bias_leadin_zones', [])]
    lc_pts = [(p['x'], p['y']) for p in zones.get('lane_change_zones', [])]

    pooled_tl_points = [(x, y) for x, y, _gid in pooled_tl_group_zones]
    runway_xy, runway_method = compute_runway_clear_point(
        resampled, pooled_tl_points, _DEFAULT_TL_ZONE_RADIUS_M, FLAT_RUNWAY_FALLBACK_M)

    # TL zone centers are the average of real signal-head positions, which
    # sit beside/above the road, not on its centerline — plotting the
    # injection marker there made it look off-route (2026-07-27 feedback).
    # The real injection moment is the FIRST point along the driven route
    # (walking forward) that comes within the zone's radius of its center —
    # not the point of closest approach overall, which for a wide zone can
    # be a full diameter further along the route, effectively at/after the
    # intersection instead of on approach to it (2026-07-27 feedback: this
    # made injection points look like they were "too close to the light to
    # matter" — they were showing closest-approach, not first-entry). The
    # zone CIRCLE itself still stays at the true (possibly off-route)
    # geometric center, since that's the real position fault_injector.py
    # measures distance against; only the injection marker is adjusted.
    tl_zone_centers = [(z['x'], z['y']) for z in tl_zones_for_goal]
    tl_reachable = [z.get('reachable', True) for z in tl_zones_for_goal]
    tl_group_ids = [z['group_id'] for z in tl_zones_for_goal]
    tl_injection_pts = [first_entry_point_on_route(resampled, c, _DEFAULT_TL_ZONE_RADIUS_M)
                         for c in tl_zone_centers]

    zone_pts_for_bbox = {'tl': tl_zone_centers, 'turn': [(x, y) for x, y, _r in turn_pts],
                          'leadin': [(x, y) for x, y, _r in leadin_pts], 'none': []}[zone_kind]
    all_x = xs + [p[0] for p in zone_pts_for_bbox + lc_pts] + [runway_xy[0], goal_xy[0]]
    all_y = ys + [p[1] for p in zone_pts_for_bbox + lc_pts] + [runway_xy[1], goal_xy[1]]
    xmin, xmax = min(all_x) - margin, max(all_x) + margin
    ymin, ymax = min(all_y) - margin, max(all_y) + margin

    fig, ax = plt.subplots(figsize=(10, 11))
    # Was '#eeeeee'/'#cccccc' here (slightly lighter than plot_routes.py's/
    # plot_fault_impact.py's '#dddddd'/'#bbbbbb') for no functional reason —
    # unified via draw_map_background's shared default, per the same
    # consistency pass that moved this loop into plotting.py.
    draw_map_background(ax, map_data, xmin, xmax, ymin, ymax)

    # Runway (before runway-clear, NOTHING can arm) drawn distinctly from the
    # eligible remainder of the route.
    runway_idx = min(range(len(resampled)),
                      key=lambda i: math.hypot(resampled[i][0] - runway_xy[0], resampled[i][1] - runway_xy[1]))
    ax.plot(xs[:runway_idx + 1], ys[:runway_idx + 1], color=RUNWAY_COLOR, linewidth=1.3,
            alpha=0.5, linestyle=':', zorder=2, label='runway (nothing can arm yet)')
    eligible_label = ('eligible for continuous injection (any point, incl. straights)'
                       if zone_kind == 'none' else 'route (fault arms only within a zone below)')
    ax.plot(xs[runway_idx:], ys[runway_idx:], color=ROUTE_COLOR, linewidth=1.3,
            alpha=0.6, zorder=2, label=eligible_label)
    ax.plot(*runway_xy, marker='x', color=RUNWAY_COLOR, markersize=7, markeredgewidth=1.8,
            zorder=6, label=f'runway clears here ({runway_method})')

    ax.plot(resampled[0][0], resampled[0][1], marker='o', color='black', markersize=7,
            zorder=6, label='route start')
    ax.plot(*goal_xy, marker='D', color='black', markersize=8, zorder=6, label='goal')

    n_zones = 0
    if zone_kind == 'tl':
        tl_zones_xyr = [(x, y, r) for (x, y), r in zip(tl_zone_centers, tl_reachable)]
        draw_zones_with_reachability(ax, tl_zones_xyr, tl_injection_pts,
                                      _DEFAULT_TL_ZONE_RADIUS_M, TL_COLOR,
                                      f'TL fault zone (r={_DEFAULT_TL_ZONE_RADIUS_M:.0f}m)',
                                      'labeled: real traffic_light_group_id')
        # Explicit marker at the zone's real (possibly off-route) center —
        # the translucent circle alone didn't make clear WHAT it was
        # centered on (2026-07-27 feedback).
        for i, (x, y) in enumerate(tl_zone_centers):
            ax.plot(x, y, marker='s', color='black', markersize=6, zorder=5,
                    label='traffic light (real position)' if i == 0 else None)
        for (x, y), gid, reachable in zip(tl_zone_centers, tl_group_ids, tl_reachable):
            if reachable:
                ax.annotate(str(gid), (x, y), textcoords='offset points', xytext=(6, 6), fontsize=7)
        n_zones = sum(tl_reachable)
    elif zone_kind == 'turn':
        turn_injection_pts = [first_entry_point_on_route(resampled, (x, y), _DEFAULT_IMU_TURN_ZONE_RADIUS_M)
                               for x, y, _r in turn_pts]
        draw_zones_with_reachability(ax, turn_pts, turn_injection_pts,
                                      _DEFAULT_IMU_TURN_ZONE_RADIUS_M, TURN_COLOR,
                                      f'IMU turn zone (r={_DEFAULT_IMU_TURN_ZONE_RADIUS_M:.0f}m)',
                                      'start of turn')
        n_zones = sum(r for _, _, r in turn_pts)
        # Fault end (zone-exit-ended, 2026-07-28; corrected 2026-07-28 to
        # use the turn run's own real end point instead of "left the 15m
        # entry radius" — every measured turn run exceeds that radius, so
        # the old derivation understated where the fault actually turns
        # off now) — only meaningful for reachable zones, matching which
        # zones actually get an injection star above.
        turn_exit_pts = [end_xy for (x, y, r), end_xy in zip(turn_pts, turn_end_pts) if r]
        n_never_exited = sum(1 for p in turn_exit_pts if p is None)
        if n_never_exited:
            print(f'{goal}: WARNING — {n_never_exited} reachable turn zone(s) have no stored '
                  f'end point (stale turn_zones.json — rerun compute_turn_zones.py)', file=sys.stderr)
        draw_fault_end_points(ax, turn_exit_pts, FAULT_END_COLOR, 'fault end (turn complete)')
        if lc_pts:
            ax.scatter([p[0] for p in lc_pts], [p[1] for p in lc_pts],
                        marker='x', color=LANE_CHANGE_COLOR, s=50, zorder=5,
                        label='lane-change (detected, excluded from gating)')
    elif zone_kind == 'leadin':
        leadin_injection_pts = [first_entry_point_on_route(resampled, (x, y), _DEFAULT_IMU_LEADIN_ZONE_RADIUS_M)
                                 for x, y, _r in leadin_pts]
        draw_zones_with_reachability(ax, leadin_pts, leadin_injection_pts,
                                      _DEFAULT_IMU_LEADIN_ZONE_RADIUS_M, LEADIN_COLOR,
                                      f'IMU bias lead-in zone (r={_DEFAULT_IMU_LEADIN_ZONE_RADIUS_M:.0f}m)',
                                      'lead-in')
        n_zones = sum(r for _, _, r in leadin_pts)
        if lc_pts:
            ax.scatter([p[0] for p in lc_pts], [p[1] for p in lc_pts],
                        marker='x', color=LANE_CHANGE_COLOR, s=50, zorder=5,
                        label='lane-change (detected, excluded from gating)')
    elif zone_kind == 'none':
        draw_injection_points(ax, [runway_xy], INJECTION_COLOR, 'injection begins here')

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal')
    ax.set_xlabel('map x (m)')
    ax.set_ylabel('map y (m)')
    if zone_kind == 'none':
        subtitle = f'{fault_label} — no zone gating, continuous from runway-clear'
    else:
        subtitle = f'{fault_label} — {n_zones} zone(s)'
    end_mode, end_seconds = fault_end
    if end_mode == 'fixed_timer':
        subtitle += (f'\nfault end: fixed {end_seconds}s timer after activation '
                      '(not shown geometrically — depends on trial speed)')
    elif end_mode == 'zone_exit':
        subtitle += '\nfault end: turn-zone exit (teal marker below)'
    ax.set_title(f'Fault injection PLAN — {campaign} / {goal}\n{subtitle}')
    ax.legend(loc='best', fontsize=7)
    fig.tight_layout()

    out_path = os.path.join(output_dir, f'{campaign}_{goal}_fault_plan.png')
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f'{goal}: saved {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', required=True, choices=sorted(CAMPAIGN_ZONE_KIND))
    ap.add_argument('--goals', default=None, help='comma-separated; default = turn_zones.json goals')
    ap.add_argument('--turn-zones-file', default=DEFAULT_TURN_ZONES_FILE)
    ap.add_argument('--tl-zones-file', default=DEFAULT_TL_ZONES_FILE)
    ap.add_argument('--goals-file', default=DEFAULT_GOALS_FILE)
    ap.add_argument('--map-file', default=DEFAULT_MAP_FILE)
    ap.add_argument('--nominal-campaign', default=DEFAULT_NOMINAL_CAMPAIGN)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    ap.add_argument('--margin', type=float, default=40.0)
    args = ap.parse_args()

    zone_kind, fault_label = CAMPAIGN_ZONE_KIND[args.campaign]
    fault_end = CAMPAIGN_FAULT_END[args.campaign]

    zones_data = json.load(open(args.turn_zones_file))
    tl_zones_data = json.load(open(args.tl_zones_file))
    goals = args.goals.split(',') if args.goals else list(zones_data['goals'].keys())
    goals_by_id = {g['id']: g for g in json.load(open(args.goals_file))['goals']}

    os.makedirs(args.output_dir, exist_ok=True)
    print(f'Loading lanelet2 map: {args.map_file}')
    map_data = load_map(args.map_file)

    print(f'Loading pooled TL group zones (for runway-clear point, matching '
          f'fault_injector.py runtime behavior across {_DEFAULT_ZONE_GOALS})...')
    pooled_tl_group_zones = _load_tl_group_zones(_DEFAULT_ZONE_GOALS)

    for goal in goals:
        zones = zones_data['goals'].get(goal, {})
        tl_zones_for_goal = tl_zones_data['goals'].get(goal, {}).get('tl_zones', [])
        goal_xy = (goals_by_id[goal]['goal']['position']['x'], goals_by_id[goal]['goal']['position']['y'])
        plot_goal(map_data, args.campaign, zone_kind, fault_label, fault_end, goal, goal_xy, pooled_tl_group_zones,
                  zones, tl_zones_for_goal, args.nominal_campaign, args.output_dir, args.margin)


if __name__ == '__main__':
    main()
