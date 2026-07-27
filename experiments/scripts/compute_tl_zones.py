#!/usr/bin/env python3
"""
Precompute, per goal, which traffic light REGULATORY ELEMENT actually
governs the vehicle at each point along its planned route — not just "is
there some traffic light within Nm," but which specific one.

Background (2026-07-26): `_load_tl_zone_points`'s bbox-based TL point
extraction (used by fault_injector.py for both the runway gate and TL fault
zone-arming) answers "is the vehicle near ANY mapped traffic light," which
is sufficient for a generic runway-clear check but not for fault realism or
for an ST-GAT input feature — the vehicle's behavior planner reacts to the
ONE regulatory element attached to its current route lanelet, and the
perception message (`/perception/traffic_light_recognition/traffic_signals`)
commonly reports 2-6 groups simultaneously (multiple intersections/approaches
in the lookahead at once). A fault that blanket-corrupts every group in that
message is corrupting lights the vehicle isn't even reacting to yet.

Confirmed empirically: a lanelet's `traffic_light` regulatory element's own
id IS the `traffic_light_group_id` published in the perception message
(e.g. lanelet 493's regulatory element id 1489 matches a live
traffic_light_group_id=1489 in the topic) — so the route's own lanelet
sequence is sufficient to derive an exact, unambiguous "which TL matters
here" mapping, the same way compute_turn_zones.py derives turn geometry
from the route instead of guessing from position proximity.

Per goal, walks the recorded route's lanelet sequence (same route-matching
approach as compute_turn_zones.py: the route message whose goal_pose
matches this goal, not just the first one in the bag) and records one zone
per DISTINCT regulatory element encountered, at the average position of all
its 'refers' linestrings (matches fault_injector.py's
_traffic_light_points_in_bbox, which averages per element for the same
reason: one element commonly has 2-3 physical head linestrings for
different approaches/lanes, tens of metres apart).

Output: experiments/configs/tl_zones.json, consumed by fault_injector.py
for TL zone-arming AND for scoping which group_id a TL fault actually
mutates (see fault_injector.py's _tl_fault_group_id).

Usage (must source ROS/Autoware, then the repo venv):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/compute_tl_zones.py
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'lib'))

from compute_turn_zones import (  # noqa: E402
    load_map, ll_attr, find_first_bag, read_route_ids, build_route_polyline,
    resample_by_arc_length, runway_clear_arc_length, nearest_resampled_index,
    RESAMPLE_STEP_M, DEFAULT_MAP_FILE, DEFAULT_GOALS_FILE, DEFAULT_NOMINAL_CAMPAIGN, DEFAULT_GOALS,
)
from fault_injector import (  # noqa: E402
    _load_tl_zone_points, _DEFAULT_ZONE_GOALS, _DEFAULT_TL_ZONE_RADIUS_M,
)

DEFAULT_OUTPUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'configs', 'tl_zones.json')


def regulatory_element_position(reg):
    try:
        refers = reg.parameters['refers']
    except Exception:
        return None
    all_pts = [(p.x, p.y) for ls in refers for p in ls]
    if not all_pts:
        return None
    return sum(p[0] for p in all_pts) / len(all_pts), sum(p[1] for p in all_pts) / len(all_pts)


def route_tl_zones(map_data, route_ids):
    """[(x, y, traffic_light_group_id), ...] — one per distinct regulatory
    element encountered walking the route in order, first-seen position
    kept if (unusually) re-encountered."""
    seen = {}
    order = []
    for lid in route_ids:
        ll = map_data.laneletLayer[lid]
        for reg in ll.regulatoryElements:
            if ll_attr(reg, 'subtype') != 'traffic_light' or reg.id in seen:
                continue
            pos = regulatory_element_position(reg)
            if pos is None:
                continue
            seen[reg.id] = pos
            order.append(reg.id)
    return [(seen[gid][0], seen[gid][1], gid) for gid in order]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--nominal-campaign', default=DEFAULT_NOMINAL_CAMPAIGN)
    ap.add_argument('--goals', default=','.join(DEFAULT_GOALS))
    ap.add_argument('--goals-file', default=DEFAULT_GOALS_FILE)
    ap.add_argument('--map-file', default=DEFAULT_MAP_FILE)
    ap.add_argument('--output', default=DEFAULT_OUTPUT)
    args = ap.parse_args()

    goals = args.goals.split(',')
    goals_data = json.load(open(args.goals_file))
    goals_by_id = {g['id']: g for g in goals_data['goals']}

    print(f'Loading lanelet2 map: {args.map_file}')
    map_data = load_map(args.map_file)

    result = {}
    for goal in goals:
        bag_dir = find_first_bag(args.nominal_campaign, goal)
        if bag_dir is None:
            print(f'{goal}: no {args.nominal_campaign} trials found, skipping', file=sys.stderr)
            continue
        g = goals_by_id[goal]
        goal_xy = (g['goal']['position']['x'], g['goal']['position']['y'])

        route_ids = read_route_ids(bag_dir, goal_xy)
        if route_ids is None:
            print(f'{goal}: no route message matching goal_pose found, skipping', file=sys.stderr)
            continue

        zones = route_tl_zones(map_data, route_ids)

        # The runway gate (shared by TL and IMU fault arming — see
        # runway_clear_arc_length) only clears once the vehicle has entered
        # AND EXITED the first real TL zone it encounters, so that first
        # zone is — by construction, for every goal — always at or before
        # the runway-clear point and can never actually host a TL fault.
        # Reported via `reachable`, NOT dropped from the output (see
        # compute_turn_zones.py's docstring for the same policy and why:
        # this file is fault_injector.py's ONLY source for TL zone
        # positions, including the runway-clear check itself — dropping
        # the first zone here previously broke that check for every goal,
        # confirmed 2026-07-27 by the runway marker showing up a full
        # intersection later than it should on every plot).
        route_pts = build_route_polyline(map_data, route_ids)
        resampled = resample_by_arc_length(route_pts, RESAMPLE_STEP_M)
        pooled_tl_points = _load_tl_zone_points(_DEFAULT_ZONE_GOALS)
        runway_arc = runway_clear_arc_length(resampled, pooled_tl_points, _DEFAULT_TL_ZONE_RADIUS_M)

        annotated = []
        unreachable_count = 0
        for x, y, gid in zones:
            entry_arc = nearest_resampled_index(resampled, (x, y)) * RESAMPLE_STEP_M
            reachable = runway_arc is None or entry_arc > runway_arc
            if not reachable:
                unreachable_count += 1
            annotated.append((x, y, gid, reachable))

        result[goal] = {
            'tl_zones': [{'x': x, 'y': y, 'group_id': gid, 'reachable': r} for x, y, gid, r in annotated],
        }
        unreachable_note = f', {unreachable_count} unreachable (at/before runway-clear)' if unreachable_count else ''
        print(f'{goal}: {len(route_ids)} lanelet route, {len(annotated)} TL zone(s) '
              f'(group_ids={[gid for _, _, gid, _ in annotated]}){unreachable_note}')

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump({'source': 'route_geometry', 'source_campaign': args.nominal_campaign,
                    'goals': result}, f, indent=2)
    print(f'\nWrote {args.output}')


if __name__ == '__main__':
    main()
