#!/usr/bin/env python3
"""
Precompute per-goal IMU fault injection zones from the PLANNED ROUTE's own
map geometry, instead of a live sensor threshold, a wall-clock guess, or
(as this script did until 2026-07-26) yaw-rate detection on noisy driven
GT trajectories.

Background (2026-07-26): imu_scale_factor/imu_stuck_at are only consequential
when the vehicle is actually turning (their error is proportional to true yaw
rate, or diverges only once real motion departs from a frozen value) — same
"needs to coincide with the right geometry" shape as TL faults needing a real
intersection. imu_bias (constant additive) is different: it accumulates
heading error over elapsed on-time regardless of geometry, so what matters
for IT is having enough accumulation time before a moment where heading
accuracy is consequential (a turn) — not turning itself.

Redesigned 2026-07-26 (second pass) to use the route's own lanelet geometry
instead of driven GT: the driven-trajectory approach needed arc-length
resampling, careful threshold tuning, and gap-merging purely to fight GPS/
localization noise, and still produced false positives (real yaw-rate blips
during LANE CHANGES, not turns — visually confirmed 2026-07-26). Route
centerlines have zero position noise (they're exact map vector data), so
turn detection here is far more reliable, and turn-vs-lane-change
classification can use the map's own semantics (turn_direction tag +
routing-graph lanelet adjacency) instead of a distance-to-traffic-light
proxy, which turned out to be fragile: it depends on _load_tl_zone_points'
straight-line start->goal bounding box, which under-covers routes that loop
away from that line (confirmed: excludes 65% of goal_012's own route when
scoped to goal_012 alone — the live system avoids this by pooling all
production goals' boxes together, which this script now also does, but the
whole dependency was an unnecessary source of fragility for what is
fundamentally a route-geometry question).

Per goal, this reads the recorded /planning/mission_planning/route from one
nominal trial (there can be more than one route message in a bag — a stale
leftover from a previous trial's goal has been observed; the one actually
matching this goal's configured goal_pose is used, confirmed deterministic
across trials for the same start/goal), builds its lanelet-centerline
polyline, and finds turning regions by curvature (heading change per metre
of arc length — the route has no time axis, so this replaces the old yaw
RATE threshold with an equivalent yaw-per-DISTANCE threshold).

Each turning region is classified using the map itself, not a proximity
guess:
  - turn_zone: at least one spanned lanelet is explicitly tagged
    turn_direction=left/right — an unambiguous, sharp, real intersection
    turn. ONLY this category is used for IMU fault gating.
  - curved_road_zone: no spanned lanelet is tagged, but all of them are
    sequentially connected in the routing graph (a real, if gentle,
    curving road — confirmed to occur in this map: a diagonal street
    bending into the grid near goal_012/goal_026's shared start, tagged
    'straight'/untagged throughout despite ~20 degrees of real accumulated
    heading change). NOT used for gating (2026-07-27 feedback): the real
    yaw rate through a gentle curve is meaningfully smaller than through a
    tagged intersection turn, and folding both into the same "turn"
    scenario risks a weak/ambiguous divergence signal that's hard to tell
    from noise being labeled exactly the same as a strong, unambiguous
    one. Logged for future work as its own scenario, same reasoning as
    lane_change_zone below.
  - lane_change_zone: spanned lanelets are NOT all sequentially connected
    (a lateral/adjacent-lane relationship, or no direct relation at all) —
    excluded from IMU fault gating; kept in the output for visibility and
    future work (see plot_fault_plan.py and CLAUDE.md).

turn_zones and bias_leadin_zones are additionally filtered to exclude any
zone at or before the route's runway-clear point (see
runway_clear_arc_length) — fault_injector.py can never actually arm there
regardless of what's precomputed (runway_cleared gates ALL fault arming,
TL and IMU alike, and only becomes True after the vehicle has entered AND
EXITED the first real TL zone), so keeping such a zone in the output would
silently claim a scenario that can never be collected. Confirmed to matter
in practice: goal_007's first real turn (arc-length 22m) sits well before
its runway clears (62m) — every other current goal's first turn already
comes after their own runway-clear point, so this had gone unnoticed until
visualized (see plot_fault_plan.py; every TL zone is ALSO always before
runway-clear by construction, which is compute_tl_zones.py's problem to
filter, not this script's).

Output: experiments/configs/turn_zones.json, consumed by fault_injector.py.

Usage (must source ROS/Autoware, then the repo venv):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/compute_turn_zones.py
"""

import argparse
import glob
import json
import math
import os
import sys

import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from autoware_planning_msgs.msg import LaneletRoute

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
WORKSPACE_DIR = os.path.dirname(REPO_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, 'experiments', 'lib'))

from fault_injector import (  # noqa: E402
    _load_tl_zone_points, _DEFAULT_ZONE_GOALS, _DEFAULT_TL_ZONE_RADIUS_M,
)

DEFAULT_MAP_FILE = os.path.join(WORKSPACE_DIR, 'Map', 'nishishinjuku_autoware_map', 'lanelet2_map.osm')
DEFAULT_GOALS_FILE = os.path.join(REPO_DIR, 'experiments', 'configs', 'captured_goals.json')
DEFAULT_NOMINAL_CAMPAIGN = 'nom_v11'
DEFAULT_GOALS = ['goal_007', 'goal_012', 'goal_026']
DEFAULT_OUTPUT = os.path.join(REPO_DIR, 'experiments', 'configs', 'turn_zones.json')

ROUTE_TOPIC = '/planning/mission_planning/route'
ROUTE_GOAL_MATCH_TOL_M = 5.0      # a bag can contain more than one route message
                                   # (a stale one from a previous trial's goal has
                                   # been observed, ~1.4s before the real one) —
                                   # only messages whose goal_pose matches this
                                   # goal's configured position are considered,
                                   # and the LAST such match is used

CURVATURE_THRESHOLD_RAD_PER_M = 0.02   # empirically chosen 2026-07-26: p90-p95
                                        # of per-point curvature across all 3
                                        # goals' routes sits at 0.005-0.03, then
                                        # jumps to 0.05-0.1+ for real corners —
                                        # 0.02 sits cleanly in that gap. Route
                                        # geometry has zero position noise, so
                                        # (unlike the old GT-yaw-rate threshold)
                                        # this doesn't need to be conservative
                                        # against jitter.
MERGE_GAP_M = 20.0                # merge turning runs separated by a gap this
                                   # short or less into one continuous run —
                                   # same rationale as the old GT-based version:
                                   # one compound intersection can span several
                                   # short lanelets with a straight one between
MIN_RUN_LENGTH_M = 6.0            # a real turn is sustained over some real
                                   # distance (confirmed 2026-07-26: genuine
                                   # intersection turns span 14-28m at
                                   # peak curvature 0.06-0.12 rad/m); isolated
                                   # single-resample-step (2m) blips just over
                                   # CURVATURE_THRESHOLD_RAD_PER_M turned out to
                                   # be single-vertex digitization kinks in an
                                   # otherwise straight lanelet centerline, not
                                   # real turns — this filters those out
RESAMPLE_STEP_M = 2.0             # arc-length spacing for curvature computation


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ll_attr(ll, key, default=None):
    try:
        return ll.attributes[key]
    except Exception:
        return default


def load_map(map_file):
    projector = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
    map_data, _errs = lanelet2.io.loadRobust(map_file, projector)
    return map_data


def read_route_ids(bag_dir, goal_xy, tol=ROUTE_GOAL_MATCH_TOL_M):
    """Lanelet ID sequence (preferred_primitive per segment) of the route
    message whose goal_pose matches goal_xy, i.e. the actual route driven
    for THIS goal — not a stale leftover from a prior trial's goal recorded
    in the same bag. Returns the LAST match (most recent plan), None if no
    route message matches."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'), ConverterOptions('', ''))
    best = None
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != ROUTE_TOPIC:
            continue
        msg = deserialize_message(data, LaneletRoute)
        gx, gy = msg.goal_pose.position.x, msg.goal_pose.position.y
        if math.hypot(gx - goal_xy[0], gy - goal_xy[1]) <= tol:
            best = [seg.preferred_primitive.id for seg in msg.segments]
    return best


def find_first_bag(campaign, goal):
    base = os.path.join(REPO_DIR, 'experiments', 'data', campaign, goal)
    if not os.path.isdir(base):
        return None
    for t in sorted(os.listdir(base)):
        bag_dir = os.path.join(base, t, 'rosbag')
        if t.startswith('t') and os.path.isdir(bag_dir):
            return bag_dir
    return None


def build_route_polyline(map_data, lanelet_ids):
    """[(x, y, lanelet_id), ...] concatenating each lanelet's centerline in
    route order, deduping the shared boundary vertex between consecutive
    lanelets."""
    pts = []
    for lid in lanelet_ids:
        ll = map_data.laneletLayer[lid]
        for p in ll.centerline:
            if pts and math.hypot(p.x - pts[-1][0], p.y - pts[-1][1]) < 0.05:
                continue
            pts.append((p.x, p.y, lid))
    return pts


def resample_by_arc_length(route_points, step_m):
    """Like the driven-trajectory version this replaces, but (x, y, lanelet_id)
    instead of (t, x, y) — no time axis for static route geometry. Each
    resampled point inherits the lanelet_id of the original segment's later
    endpoint (fine-grained enough at step_m=2 relative to lanelet lengths)."""
    if len(route_points) < 2:
        return list(route_points)
    out = [route_points[0]]
    acc = 0.0
    for i in range(1, len(route_points)):
        x0, y0, _ = route_points[i - 1]
        x1, y1, lid = route_points[i]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len < 1e-6:
            continue
        pos_in_seg = 0.0
        while acc + (seg_len - pos_in_seg) >= step_m:
            pos_in_seg += step_m - acc
            frac = pos_in_seg / seg_len
            out.append((x0 + frac * (x1 - x0), y0 + frac * (y1 - y0), lid))
            acc = 0.0
        acc += seg_len - pos_in_seg
    return out


def turning_runs(resampled):
    """Contiguous index runs (into `resampled`) where curvature exceeds
    CURVATURE_THRESHOLD_RAD_PER_M, with nearby runs merged (see MERGE_GAP_M)
    and short runs discarded (see MIN_RUN_LENGTH_M)."""
    if len(resampled) < 3:
        return []

    turning = [False] * len(resampled)  # index-aligned to resampled; edges have no curvature
    for i in range(1, len(resampled) - 1):
        x0, y0, _ = resampled[i - 1]
        x1, y1, _ = resampled[i]
        x2, y2, _ = resampled[i + 1]
        h1 = math.atan2(y1 - y0, x1 - x0)
        h2 = math.atan2(y2 - y1, x2 - x1)
        curvature = abs(wrap_angle(h2 - h1)) / RESAMPLE_STEP_M
        turning[i] = curvature > CURVATURE_THRESHOLD_RAD_PER_M

    # Raw contiguous runs first, THEN merge nearby ones by the gap between
    # them — not by padding forward from a true point on the hope a future
    # true point exists within MERGE_GAP_M. That padding approach (this
    # script's first cut) blindly forced up to MERGE_GAP_M/RESAMPLE_STEP_M
    # points True after ANY true point regardless of whether a second real
    # run followed, silently inflating a single 2m curvature spike (a
    # digitization kink in one lanelet vertex, not a real turn — see
    # MIN_RUN_LENGTH_M) into a fake 22m run that then passed the length
    # filter it was supposed to be caught by. Found 2026-07-26 by noticing
    # known single-point noise spikes were still surviving after adding that
    # filter.
    raw_runs = []
    start = None
    for i, is_turning in enumerate(turning):
        if is_turning and start is None:
            start = i
        elif not is_turning and start is not None:
            raw_runs.append((start, i))
            start = None
    if start is not None:
        raw_runs.append((start, len(turning)))

    merged = []
    for run in raw_runs:
        if merged and (run[0] - merged[-1][1]) * RESAMPLE_STEP_M <= MERGE_GAP_M:
            merged[-1] = (merged[-1][0], run[1])
        else:
            merged.append(list(run))
    merged = [tuple(r) for r in merged]

    return [r for r in merged if (r[1] - r[0]) * RESAMPLE_STEP_M >= MIN_RUN_LENGTH_M]


def lanelet_ids_in_run(resampled, run):
    start, end = run
    ids = []
    for i in range(start, end):
        lid = resampled[i][2]
        if lid not in ids:
            ids.append(lid)
    return ids


def classify_lanelet_sequence(map_data, routing_graph, lanelet_ids):
    """turn_zone (explicitly tagged turn_direction=left/right — used for
    gating) vs curved_road_zone (untagged but sequentially connected — a
    real but gentle curve, not used for gating, future work) vs
    lane_change_zone (not sequentially connected, a lateral lane
    relationship — not used for gating, future work)."""
    for lid in lanelet_ids:
        if ll_attr(map_data.laneletLayer[lid], 'turn_direction') in ('left', 'right'):
            return 'turn_zone'

    for i in range(len(lanelet_ids) - 1):
        a = map_data.laneletLayer[lanelet_ids[i]]
        b_id = lanelet_ids[i + 1]
        following_ids = {ll.id for ll in routing_graph.following(a)}
        if b_id not in following_ids:
            return 'lane_change_zone'
    return 'curved_road_zone'


def runway_clear_arc_length(resampled, tl_points, tl_radius):
    """Arc-length (metres from route start) where fault_injector.py's own
    runway gate would clear: first entering, then exiting, any pooled TL
    zone point (mirrors _on_gt_pose's '# Runway' block exactly, minus the
    live speed>=_RUNWAY_MIN_SPEED_MPS criterion — irrelevant for static
    route geometry). None if the route never exits a zone after entering
    one. runway_cleared gates ALL fault arming (TL and IMU alike), so any
    zone at or before this point can never actually fire."""
    seen_zone = False
    was_in_zone = False
    for i, (x, y, _lid) in enumerate(resampled):
        in_zone = any(math.hypot(x - tx, y - ty) <= tl_radius for tx, ty in tl_points)
        if in_zone:
            seen_zone = True
        if seen_zone and was_in_zone and not in_zone:
            return i * RESAMPLE_STEP_M
        was_in_zone = in_zone
    return None


def run_entry_point(map_data, lanelet_ids):
    """The turn's injection point is the START of the run's first spanned
    lanelet (2026-07-26 — previously used the point where resampled curvature
    first crossed CURVATURE_THRESHOLD_RAD_PER_M, but for a gradual/untagged
    curve that threshold-crossing point can land noticeably into the curve,
    not at its start, since curvature ramps up rather than snapping on. The
    lanelet boundary is unambiguous and matches 'start of the segment', not
    an arbitrary threshold artifact)."""
    ll = map_data.laneletLayer[lanelet_ids[0]]
    p = ll.centerline[0]
    return p.x, p.y


def nearest_resampled_index(resampled, xy):
    x, y = xy
    best_i, best_d = 0, float('inf')
    for i, (px, py, _lid) in enumerate(resampled):
        d = math.hypot(px - x, py - y)
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def first_entry_point_on_route(resampled, center_xy, radius):
    """The first point along the route (walking forward from route start)
    that falls within `radius` of `center_xy` — i.e. where a zone centered
    on center_xy would ACTUALLY first arm, not the point of closest
    approach overall (2026-07-27 feedback: for TL zones especially, whose
    center is a real light's off-route position, "closest approach" and
    "first entry" can differ by the whole zone diameter — closest approach
    is typically right at/after the intersection, while first entry is
    where arming genuinely begins, with real approach distance still
    ahead). Falls back to nearest_resampled_index if the route never gets
    within radius of center_xy at all (shouldn't happen for zones actually
    derived from this route, but stay defensive)."""
    cx, cy = center_xy
    for x, y, _lid in resampled:
        if math.hypot(x - cx, y - cy) <= radius:
            return (x, y)
    i = nearest_resampled_index(resampled, center_xy)
    return resampled[i][0], resampled[i][1]


def leadin_point(resampled, entry_xy, leadin_m):
    """Walk backward along the resampled route's arc length from the turn
    entry by leadin_m, return that (x, y). Falls back to the route's first
    point if leadin_m exceeds distance already covered (turn very early)."""
    i = nearest_resampled_index(resampled, entry_xy)
    acc = 0.0
    while i > 0 and acc < leadin_m:
        acc += RESAMPLE_STEP_M
        i -= 1
    return resampled[i][0], resampled[i][1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--nominal-campaign', default=DEFAULT_NOMINAL_CAMPAIGN)
    ap.add_argument('--goals', default=','.join(DEFAULT_GOALS))
    ap.add_argument('--goals-file', default=DEFAULT_GOALS_FILE)
    ap.add_argument('--map-file', default=DEFAULT_MAP_FILE)
    ap.add_argument('--leadin-m', type=float, default=10.0,
                     help='Arc-length distance before a turn zone for the bias lead-in point (default: 10m)')
    ap.add_argument('--output', default=DEFAULT_OUTPUT)
    args = ap.parse_args()

    goals = args.goals.split(',')
    goals_data = json.load(open(args.goals_file))
    goals_by_id = {g['id']: g for g in goals_data['goals']}

    print(f'Loading lanelet2 map: {args.map_file}')
    map_data = load_map(args.map_file)
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
    routing_graph = lanelet2.routing.RoutingGraph(map_data, traffic_rules)

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
            print(f'{goal}: no route message matching goal_pose found in {bag_dir}, skipping',
                  file=sys.stderr)
            continue

        route_pts = build_route_polyline(map_data, route_ids)
        resampled = resample_by_arc_length(route_pts, RESAMPLE_STEP_M)
        runs = turning_runs(resampled)
        pooled_tl_points = _load_tl_zone_points(_DEFAULT_ZONE_GOALS)
        runway_arc = runway_clear_arc_length(resampled, pooled_tl_points, _DEFAULT_TL_ZONE_RADIUS_M)

        turn_zones, leadin_zones, lane_change_zones, curved_road_zones = [], [], [], []
        unreachable_count = 0
        for run in runs:
            lids = lanelet_ids_in_run(resampled, run)
            kind = classify_lanelet_sequence(map_data, routing_graph, lids)
            entry_xy = run_entry_point(map_data, lids)
            if kind == 'turn_zone':
                entry_arc = nearest_resampled_index(resampled, entry_xy) * RESAMPLE_STEP_M
                # Reachable is reported, NOT filtered out — see module
                # docstring. fault_injector.py loads this same list to
                # detect the runway-clear point itself; dropping the
                # unreachable entry here previously broke THAT detection
                # (it needs the complete picture to know where the first
                # real zone is), even though arming was never actually at
                # risk (the waiting_runway -> waiting_zone phase transition
                # already makes that zone unarmable on its own, by
                # construction — see _on_gt_pose/_on_tl).
                reachable = runway_arc is None or entry_arc > runway_arc
                if not reachable:
                    unreachable_count += 1
                turn_zones.append((entry_xy, reachable))
                leadin_zones.append((leadin_point(resampled, entry_xy, args.leadin_m), reachable))
            elif kind == 'curved_road_zone':
                curved_road_zones.append(entry_xy)
            else:
                lane_change_zones.append(entry_xy)

        result[goal] = {
            'turn_zones': [{'x': x, 'y': y, 'reachable': r} for (x, y), r in turn_zones],
            'bias_leadin_zones': [{'x': x, 'y': y, 'reachable': r} for (x, y), r in leadin_zones],
            'lane_change_zones': [{'x': x, 'y': y} for x, y in lane_change_zones],
            'curved_road_zones': [{'x': x, 'y': y} for x, y in curved_road_zones],
        }
        unreachable_note = (f', {unreachable_count} unreachable (at/before runway-clear — kept in '
                             f'the list, flagged reachable=false, not dropped: see module docstring)'
                             if unreachable_count else '')
        print(f'{goal}: {len(route_ids)} lanelet route, {len(turn_zones)} turn zone(s), '
              f'{len(leadin_zones)} lead-in zone(s), '
              f'{len(lane_change_zones)} lane-change zone(s), '
              f'{len(curved_road_zones)} curved-road zone(s) (neither used for gating)'
              f'{unreachable_note}')

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump({
            'curvature_threshold_rad_per_m': CURVATURE_THRESHOLD_RAD_PER_M,
            'leadin_m': args.leadin_m,
            'source_campaign': args.nominal_campaign,
            'source': 'route_geometry',
            'goals': result,
        }, f, indent=2)
    print(f'\nWrote {args.output}')


if __name__ == '__main__':
    main()
