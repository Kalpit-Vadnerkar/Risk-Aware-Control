#!/usr/bin/env python3
"""
HD Map explorer for the nishishinjuku_autoware_map (MGRS 54SUE frame).

Analyses lanelet topology, adjacency, and identifies single-lane segments
for scenario route selection.

Usage (must source ROS/Autoware first):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  python3 experiments/scripts/explore_map.py
"""

import math
import json
import lanelet2
from lanelet2.core import BasicPoint2d
from autoware_lanelet2_extension_python.projection import MGRSProjector

MAP_FILE = "/home/kvadner/Desktop/Dissertation/Map/nishishinjuku_autoware_map/lanelet2_map.osm"

# The map uses MGRS 54SUE projection. Autoware's own map loader
# (autoware_map_projection_loader) projects lanelet2 maps with MGRSProjector
# when no map_projector_info.yaml is present (the case here), deriving the
# MGRS grid cell straight from the map's own lat/lon points — that's what
# actually lines up with the AWSIM/ROS2 map frame (spawn position
# (81384.6, 49922.0) lands ~3.8m from the nearest road lanelet). A previous
# version of this file guessed a UtmProjector origin empirically, which was
# off by >1000m — do not reintroduce that pattern.
ORIGIN = lanelet2.io.Origin(0.0, 0.0)

GOALS_FILE = "/home/kvadner/Desktop/Dissertation/Risk-Aware-Control/experiments/configs/captured_goals.json"

# Spawn point in AWSIM/ROS2 map frame
SPAWN = (81384.53, 49921.95)


def dist2d(ax, ay, bx, by):
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


def lanelet_midpoint(ll):
    cl = ll.centerline
    mid = len(cl) // 2
    return cl[mid].x, cl[mid].y


def lanelet_length(ll):
    total = 0.0
    pts = list(ll.centerline)
    for i in range(1, len(pts)):
        total += dist2d(pts[i].x, pts[i].y, pts[i-1].x, pts[i-1].y)
    return total


def ll_attr(ll, key, default=""):
    try:
        return ll.attributes[key]
    except Exception:
        return default


def closest_lanelets(map_data, x, y, n=5, only_road=True):
    results = []
    for ll in map_data.laneletLayer:
        if only_road and ll_attr(ll, "subtype") != "road":
            continue
        mx, my = lanelet_midpoint(ll)
        d = dist2d(x, y, mx, my)
        results.append((d, ll))
    results.sort(key=lambda r: r[0])
    return results[:n]


def find_adjacent(routing_graph, ll):
    left = routing_graph.left(ll)
    right = routing_graph.right(ll)
    adj_l = routing_graph.adjacentLeft(ll)
    adj_r = routing_graph.adjacentRight(ll)
    return left, right, adj_l, adj_r


def has_adjacent(routing_graph, ll):
    return any(x is not None for x in find_adjacent(routing_graph, ll))


def print_section(title):
    print("\n" + "=" * 72)
    print(f"  {title}")
    print("=" * 72)


def main():
    print("Loading lanelet2 map (MGRS 54SUE / MGRSProjector)...")
    projector = MGRSProjector(ORIGIN)
    map_data, errs = lanelet2.io.loadRobust(MAP_FILE, projector)
    road_lls = [ll for ll in map_data.laneletLayer if ll_attr(ll, "subtype") == "road"]
    print(f"  Road lanelets: {len(road_lls)}  (total: {len(list(map_data.laneletLayer))})")

    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle
    )
    routing_graph = lanelet2.routing.RoutingGraph(map_data, traffic_rules)

    # ------------------------------------------------------------------ #
    # 0. Verify projection — spawn-point should be near road lanelets
    # ------------------------------------------------------------------ #
    print_section("0. Projection check — spawn-point neighbourhood")
    sx, sy = SPAWN
    spawn_near = closest_lanelets(map_data, sx, sy, n=5)
    print(f"  Spawn: ({sx:.1f}, {sy:.1f})")
    for d, ll in spawn_near:
        mx, my = lanelet_midpoint(ll)
        print(f"  LL {ll.id:6d}  dist={d:6.1f}m  mid=({mx:.1f},{my:.1f})  len={lanelet_length(ll):.1f}m")

    if spawn_near[0][0] > 50:
        print("  WARNING: closest lanelet is >50m from spawn — projection may be off.")
        print("  Map analysis below may be unreliable. Run test_route_feasibility.py for live testing.")

    # ------------------------------------------------------------------ #
    # 1. All goals — feasibility snapshot from map topology
    # ------------------------------------------------------------------ #
    print_section("1. All goals — map-topology feasibility")
    with open(GOALS_FILE) as f:
        gdata = json.load(f)

    goals = gdata['goals']
    spawn_lls = [ll for d, ll in closest_lanelets(map_data, sx, sy, n=10)]

    print(f"\n  {'Goal':<12} {'Dist':>6}  {'CloseLL':>8}  {'LLdist':>7}  NearAdj  Notes")
    print(f"  {'-'*70}")
    for g in goals:
        gid = g['id']
        px = g['goal']['position']['x']
        py = g['goal']['position']['y']
        dist = g.get('estimated_distance', '?')
        replaced = '[repl]' if g.get('original_goal_replaced') else ''

        goal_near = closest_lanelets(map_data, px, py, n=3)
        close_ll = goal_near[0][1] if goal_near else None
        close_d = goal_near[0][0] if goal_near else 9999

        if close_ll is not None:
            adj = has_adjacent(routing_graph, close_ll)
            adj_str = 'multi' if adj else 'SINGLE'
        else:
            adj_str = '?'

        print(f"  {gid:<12} {str(dist):>6}  {(close_ll.id if close_ll else '?'):>8}  "
              f"{close_d:>7.1f}  {adj_str:<7}  {replaced}")

    # ------------------------------------------------------------------ #
    # 2. Single-lane segment survey (for obs_singlelane scenario)
    # ------------------------------------------------------------------ #
    print_section("2. Single-lane segments across the entire map")
    single_lane = []
    for ll in road_lls:
        if not has_adjacent(routing_graph, ll):
            mx, my = lanelet_midpoint(ll)
            d_spawn = dist2d(sx, sy, mx, my)
            llen = lanelet_length(ll)
            single_lane.append((d_spawn, llen, ll))

    single_lane.sort(key=lambda x: x[0])
    print(f"  Found {len(single_lane)} single-lane road segments")
    print(f"\n  {'LL':>8}  {'DistSpawn':>10}  {'Length':>8}  mid-x     mid-y")
    print(f"  {'-'*55}")
    for d_s, llen, ll in single_lane[:30]:
        mx, my = lanelet_midpoint(ll)
        print(f"  {ll.id:>8}  {d_s:>10.1f}  {llen:>8.1f}  {mx:.1f}  {my:.1f}")

    # ------------------------------------------------------------------ #
    # 3. Route adjacency walk for selected goals (to find single-lane spots)
    # ------------------------------------------------------------------ #
    print_section("3. Route adjacency walk — goals closest to spawn")

    # Try first 5 spawn candidates as start lanelets
    spawn_candidates = [(d, ll) for d, ll in closest_lanelets(map_data, sx, sy, n=10)]

    target_goals = goals  # analyse all goals
    for g in target_goals:
        gid = g['id']
        px = g['goal']['position']['x']
        py = g['goal']['position']['y']
        goal_lls = closest_lanelets(map_data, px, py, n=5)

        found = False
        for sd, sll in spawn_candidates[:5]:
            for gd, gll in goal_lls[:3]:
                try:
                    route = routing_graph.getRoute(sll, gll, 0)
                    if route is None:
                        continue
                    path = route.shortestPath()
                    if path is None:
                        continue

                    n_single = sum(1 for rll in path if not has_adjacent(routing_graph, rll))
                    n_total = len(path)

                    # Summarise single-lane spans
                    single_spans = []
                    cum = 0.0
                    px2, py2 = sx, sy
                    for rll in path:
                        mx, my = lanelet_midpoint(rll)
                        cum += dist2d(px2, py2, mx, my)
                        px2, py2 = mx, my
                        if not has_adjacent(routing_graph, rll):
                            single_spans.append(f"~{cum:.0f}m(LL{rll.id})")

                    span_str = ', '.join(single_spans[:5]) if single_spans else 'none'
                    flag = ' ← SINGLE-LANE ROUTE' if n_single > 0 else ''
                    print(f"\n  {gid}: {n_total} lanelets, {n_single} single-lane segments{flag}")
                    print(f"    Single-lane at: {span_str}")
                    found = True
                    break
                except Exception:
                    pass
            if found:
                break
        if not found:
            print(f"\n  {gid}: route not computable from map (test live with Autoware)")

    # ------------------------------------------------------------------ #
    # 4. Traffic light coverage per route (goal neighbourhood)
    # ------------------------------------------------------------------ #
    print_section("4. Traffic light regulatory elements near each goal")
    tl_count = {}
    for g in goals:
        gid = g['id']
        px = g['goal']['position']['x']
        py = g['goal']['position']['y']
        goal_near_lls = [ll for d, ll in closest_lanelets(map_data, px, py, n=10)]
        tls = set()
        for ll in goal_near_lls:
            for reg in ll.regulatoryElements:
                if 'traffic_light' in str(type(reg)).lower() or 'TrafficLight' in str(type(reg)):
                    tls.add(reg.id)
        tl_count[gid] = len(tls)

    print(f"\n  {'Goal':<12}  TL elements near goal")
    for g in goals:
        gid = g['id']
        print(f"  {gid:<12}  {tl_count[gid]}")


if __name__ == "__main__":
    main()
