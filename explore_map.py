#!/usr/bin/env python3
"""
HD Map explorer for Shinjuku Autoware scenario.
Reads the lanelet2 OSM map and analyses route topology for our test goals.

Usage (must source ROS/Autoware first):
  source /opt/ros/humble/setup.bash
  python3 explore_map.py
"""

import math
import json
import lanelet2
from lanelet2.core import BasicPoint2d

MAP_FILE = "/home/df/Desktop/Kalpit-2026/Risk-Aware-Control/Shinjuku-Map/map/lanelet2_map.osm"
# Projector origin matches MapProcessor.py
ORIGIN = lanelet2.io.Origin(35.67, 139.65, 0)

# Spawn point in map frame (from captured_goals.json start_position)
SPAWN = (81384.53, 49921.95)

# Our test goals (id -> (x, y, distance_m))
GOALS = {
    "goal_003": (81641.37, 50492.94, 626.0),
    "goal_007": (81606.78, 50596.17, 709.9),
    "goal_011": (81497.79, 50544.91, 633.3),
    "goal_021": (81487.06, 50611.14, 696.9),
}

# Approximate obstacle placement: 150m travel from spawn along route + 30m ahead
# The actual point depends on the route, but let's find lanelets around 150-180m from spawn


def dist2d(ax, ay, bx, by):
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


def lanelet_midpoint(ll):
    cl = ll.centerline
    mid = len(cl) // 2
    return cl[mid].x, cl[mid].y


def lanelet_length(ll):
    """Approximate length of a lanelet via centerline arc."""
    total = 0.0
    pts = list(ll.centerline)
    for i in range(1, len(pts)):
        total += dist2d(pts[i].x, pts[i].y, pts[i-1].x, pts[i-1].y)
    return total


def ll_attr(ll, key, default=""):
    """Safe attribute access for lanelet2 AttributeMap."""
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


def find_adjacent_lanes(map_data, routing_graph, ll):
    """Return left/right adjacent lanelets if they exist."""
    left = routing_graph.left(ll)
    right = routing_graph.right(ll)
    adjacent_left = routing_graph.adjacentLeft(ll)
    adjacent_right = routing_graph.adjacentRight(ll)
    return {
        "left": left,
        "right": right,
        "adjacent_left": adjacent_left,
        "adjacent_right": adjacent_right,
    }


def lanelets_within_radius(map_data, cx, cy, radius, only_road=True):
    results = []
    for ll in map_data.laneletLayer:
        if only_road and ll_attr(ll, "subtype") != "road":
            continue
        mx, my = lanelet_midpoint(ll)
        if dist2d(cx, cy, mx, my) <= radius:
            results.append(ll)
    return results


def print_section(title):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def main():
    print("Loading lanelet2 map...")
    projector = lanelet2.projection.LocalCartesianProjector(ORIGIN)
    map_data, errs = lanelet2.io.loadRobust(MAP_FILE, projector)
    if errs:
        print(f"  Load warnings: {errs}")
    print(f"  Lanelets: {len(list(map_data.laneletLayer))}")
    print(f"  LineStrings: {len(list(map_data.lineStringLayer))}")

    # Build routing graph (required for left/right queries)
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle
    )
    routing_graph = lanelet2.routing.RoutingGraph(map_data, traffic_rules)

    # ------------------------------------------------------------------ #
    # 1. Spawn-point context
    # ------------------------------------------------------------------ #
    print_section("1. Spawn-point neighbourhood")
    sx, sy = SPAWN
    print(f"  Spawn: ({sx:.1f}, {sy:.1f})")
    spawn_lls = closest_lanelets(map_data, sx, sy, n=5)
    for d, ll in spawn_lls:
        mx, my = lanelet_midpoint(ll)
        length = lanelet_length(ll)
        subtype = ll_attr(ll, "subtype", "?")
        print(f"  LL id={ll.id:6d}  dist={d:6.1f}m  mid=({mx:.1f},{my:.1f})  "
              f"len={length:.1f}m  subtype={subtype}")

    # ------------------------------------------------------------------ #
    # 2. Goal-point context
    # ------------------------------------------------------------------ #
    print_section("2. Goal-point neighbourhood")
    for gname, (gx, gy, gdist) in GOALS.items():
        print(f"\n  {gname} @ ({gx:.1f},{gy:.1f})  route~{gdist:.0f}m")
        goal_lls = closest_lanelets(map_data, gx, gy, n=3)
        for d, ll in goal_lls:
            mx, my = lanelet_midpoint(ll)
            print(f"    LL id={ll.id:6d}  dist_to_goal={d:.1f}m  subtype={ll.attributes.get('subtype','?')}")

    # ------------------------------------------------------------------ #
    # 3. Obstacle placement zone (≈ 150-210m from spawn along heading)
    #    Spawn heading is roughly north (increasing y).  Rough estimate:
    #    At 150m travel, ego is near (81384, 50072). At 180m, (81384, 50102).
    #    Let's search a 60m radius around the midpoint.
    # ------------------------------------------------------------------ #
    print_section("3. Obstacle placement zone (150-200m from spawn)")
    # The route heads north from spawn and then curves.
    # A rough point 175m north of spawn:
    obs_cx, obs_cy = SPAWN[0], SPAWN[1] + 175.0
    print(f"  Approximate obstacle zone centre: ({obs_cx:.1f}, {obs_cy:.1f})")
    zone_lls = lanelets_within_radius(map_data, obs_cx, obs_cy, radius=80, only_road=True)
    print(f"  Road lanelets within 80m: {len(zone_lls)}")

    for ll in sorted(zone_lls, key=lambda l: dist2d(obs_cx, obs_cy, *lanelet_midpoint(l))):
        mx, my = lanelet_midpoint(ll)
        d = dist2d(obs_cx, obs_cy, mx, my)
        length = lanelet_length(ll)
        adj = find_adjacent_lanes(map_data, routing_graph, ll)
        left_id  = adj["left"].id  if adj["left"]  else None
        right_id = adj["right"].id if adj["right"] else None
        al_id    = adj["adjacent_left"].id  if adj["adjacent_left"]  else None
        ar_id    = adj["adjacent_right"].id if adj["adjacent_right"] else None

        can_avoid = any(v is not None for v in [left_id, right_id, al_id, ar_id])
        d_from_zone = dist2d(obs_cx, obs_cy, mx, my)
        print(f"  LL id={ll.id:6d}  dist={d_from_zone:5.1f}m  len={length:5.1f}m  "
              f"left={left_id}  right={right_id}  adj_L={al_id}  adj_R={ar_id}  "
              f"{'[CAN AVOID]' if can_avoid else '[NO ADJACENT]'}")

    # ------------------------------------------------------------------ #
    # 4. Route analysis: find all lanelets a routing from spawn→goal uses
    #    for each of our 4 goals, and check adjacency at each lanelet.
    # ------------------------------------------------------------------ #
    print_section("4. Route adjacency analysis for test goals")

    # Find the closest routable lanelet to spawn
    spawn_ll_candidates = closest_lanelets(map_data, sx, sy, n=10)
    spawn_ll = None
    for d, ll in spawn_ll_candidates:
        # Try to get routes from this lanelet
        try:
            test_route = routing_graph.getRoute(ll, ll, 0)
            spawn_ll = ll
            break
        except Exception:
            pass

    # Try each candidate lanelet as start
    for gname, (gx, gy, gdist) in GOALS.items():
        print(f"\n  Route to {gname}:")
        goal_lls = closest_lanelets(map_data, gx, gy, n=5)

        found_route = False
        for sd, sll in spawn_ll_candidates[:5]:
            for gd, gll in goal_lls[:3]:
                try:
                    route = routing_graph.getRoute(sll, gll, 0)
                    if route is None:
                        continue
                    llt = route.shortestPath()
                    if llt is None:
                        continue

                    print(f"    Route found: {len(llt)} lanelets  "
                          f"(spawn LL={sll.id}, goal LL={gll.id})")

                    # Walk the route and check adjacency + approx distance from spawn
                    cum_dist = 0.0
                    prev_x, prev_y = sx, sy
                    no_adj_segments = []
                    for i, rll in enumerate(llt):
                        mx, my = lanelet_midpoint(rll)
                        step = dist2d(prev_x, prev_y, mx, my)
                        cum_dist += step
                        prev_x, prev_y = mx, my

                        adj = find_adjacent_lanes(map_data, routing_graph, rll)
                        has_adj = any(v is not None for v in adj.values())

                        marker = ""
                        if 120 <= cum_dist <= 220:  # obstacle zone window
                            marker = " <<OBSTACLE ZONE>>"
                            if not has_adj:
                                no_adj_segments.append((i, rll.id, cum_dist))

                        if 100 <= cum_dist <= 250:  # print a window around obstacle zone
                            left_id  = adj["left"].id  if adj["left"]  else "—"
                            right_id = adj["right"].id if adj["right"] else "—"
                            print(f"      [{i:3d}] LL={rll.id:6d}  ~{cum_dist:5.0f}m  "
                                  f"left={left_id}  right={right_id}  "
                                  f"{'OK' if has_adj else 'NO ADJ'}{marker}")

                    if no_adj_segments:
                        print(f"    WARNING: {len(no_adj_segments)} segment(s) in obstacle zone have NO adjacent lanes")
                    found_route = True
                    break
                except Exception as e:
                    pass
            if found_route:
                break
        if not found_route:
            print(f"    Could not compute route (routing graph may need different start LL)")

    # ------------------------------------------------------------------ #
    # 5. All goals overview — distance, heading, lane context
    # ------------------------------------------------------------------ #
    print_section("5. All 25 goals — distance from spawn")
    goals_file = "/home/df/Desktop/Kalpit-2026/Risk-Aware-Control/experiments/configs/captured_goals.json"
    try:
        with open(goals_file) as f:
            gdata = json.load(f)
        for g in gdata["goals"]:
            gid = g["id"]
            px = g["goal"]["position"]["x"]
            py = g["goal"]["position"]["y"]
            edist = g.get("estimated_distance", "?")
            star = " ***" if gid in ("goal_003", "goal_007", "goal_011", "goal_021") else ""
            replaced = " [replaced]" if g.get("original_goal_replaced") else ""
            print(f"  {gid:10s}  pos=({px:.0f},{py:.0f})  est={edist}m{star}{replaced}")
    except Exception as e:
        print(f"  Could not load goals file: {e}")

    # ------------------------------------------------------------------ #
    # 6. Summary: what can Autoware do at the obstacle?
    # ------------------------------------------------------------------ #
    print_section("6. Autoware obstacle recovery — config summary")
    print("""
  From static_obstacle_avoidance.param.yaml:
    use_lane_type: "opposite_direction_lane"   → can use oncoming lane to avoid
    avoidance_for_ambiguous_vehicle.policy: "manual"  → WAITS for operator when
      object not clearly parked (stopped <3s or moving >1m/s)
    avoidance_for_parking_violation_vehicle.policy: "ignore"  → ignores parked-on-shoulder
    stop.max_distance: 20.0m  → stops up to 20m before obstacle

  From lane_change.param.yaml:
    regulation.traffic_light: true  → NO lane change near traffic lights
    regulation.intersection: true   → NO lane change in intersections
    stuck_detection.velocity: 0.5 m/s, stop_time: 3.0s  → triggers stuck detection

  Root cause of "getting stuck":
    1. Obstacle is placed in-lane (lateral_offset=0.0), classified as a stopped car.
    2. Avoidance module waits for object to be clearly parked (th_stopped_time=3s).
    3. "ambiguous vehicle" policy is "manual" → needs RTC approval button click.
    4. Without RTC approval, ego stops and STAYS stopped indefinitely.
    5. Lane change module may not activate if intersection/traffic-light regulation fires.

  Recovery options (see Section 7 below).
""")

    print_section("7. Recovery strategies")
    print("""
  Option A — RTC AUTO mode (recommended for experiments)
    Set all RTC modules to AUTO so avoidance/lane-change execute without approval.
    How: publish to /api/operation_mode/change_to_autonomous OR
         use Autoware API: OperationModeChangeRequest
    Risk: ego may make aggressive maneuvers in tight spaces.

  Option B — Publish RTC approve command programmatically
    Listen to /planning/scenario_planning/lane_driving/behavior_planning/
              behavior_path_planner/rtc_status
    Then publish ApproveRequest to the avoidance RTC topic.
    This explicitly approves the avoidance maneuver from our ROS2 node.

  Option C — Force a reroute (bypass the obstacle)
    When ego is stuck (velocity < 0.5 m/s for > 3s):
    - Cancel current route via /api/routing/clear_route
    - Re-issue SetRoutePoints with an intermediate waypoint that bypasses the obstacle lane
    - This works if there IS an adjacent lane (confirmed via lanelet analysis above)
    Limitation: the new route must be topologically reachable.

  Option D — Remove the obstacle after timeout
    Treat static obstacle as a sensor-limited scenario:
    - After a configurable timeout (e.g., 30s), remove the obstacle from perception
    - Autoware resumes on cleared path
    This is "recovery by perception clearing" — good for the RISE-on scenario
    because it tests whether RISE constraint relaxes as residuals drop.

  Option E — Freespace planner fallback
    Autoware has a freespace planner that activates when stuck.
    Enable it in the launch config; it plans a free-space path around the obstacle.
    May go off-route briefly, then re-join.

  RECOMMENDATION for RISE experiments:
    Use Option D (timeout removal) as the primary recovery:
    - It creates a clean before/after signal in residuals (spike → decay)
    - CVaR can be computed on the spike + recovery arc
    - Works regardless of map topology
    - Simple to implement: add a timer in perception_interceptor.py that removes
      the obstacle UUID after N seconds of ego being stopped.
    Combine with Option A (AUTO mode) so avoidance fires immediately when obstacle
    is removed and path is clear.
""")


if __name__ == "__main__":
    main()
