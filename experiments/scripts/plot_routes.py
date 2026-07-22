#!/usr/bin/env python3
"""
Plots each route's driven trajectory (all trials) superimposed on the HD map,
trimmed to a bounding box around that specific route.

Usage (must source ROS/Autoware, then the repo venv):
  source /opt/ros/humble/setup.bash
  source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
  source .venv/bin/activate
  python3 experiments/scripts/plot_routes.py
  python3 experiments/scripts/plot_routes.py --goal goal_003,goal_005
  python3 experiments/scripts/plot_routes.py --data-dir experiments/data/nom_v11 \
      --goals-file captured_goals.json
"""

import argparse
import glob
import json
import os

import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
WORKSPACE_DIR = os.path.dirname(REPO_DIR)

DEFAULT_MAP_FILE = os.path.join(WORKSPACE_DIR, 'Map', 'nishishinjuku_autoware_map', 'lanelet2_map.osm')
DEFAULT_GOALS_FILE = os.path.join(REPO_DIR, 'experiments', 'configs', 'captured_goals_original.json')
DEFAULT_DATA_DIR = os.path.join(REPO_DIR, 'experiments', 'data', 'nom_v11')
DEFAULT_OUTPUT_DIR = os.path.join(REPO_DIR, 'experiments', 'analysis', 'route_maps')

# Autoware's own map loader (autoware_map_projection_loader/load_info_from_lanelet2_map.cpp)
# projects lanelet2 maps with lanelet::projection::MGRSProjector when no
# map_projector_info.yaml is present (the case here) — NOT a generic UtmProjector
# with a guessed lat/lon origin. That projector derives the MGRS 100km-grid cell
# straight from the map's own lat/lon points, which is what actually lines up
# with the AWSIM/ROS2 map frame (verified: spawn point lands ~3.8m from the
# nearest road lanelet centerline, vs >1000m off with the guessed-origin
# UtmProjector previously used in explore_map.py).
SPAWN = (81384.53, 49921.95)


def make_projector():
    return MGRSProjector(lanelet2.io.Origin(0.0, 0.0))

GT_TOPIC = '/awsim/ground_truth/localization/kinematic_state'
FALLBACK_TOPIC = '/localization/kinematic_state'

STATUS_COLOR = {
    'goal_reached': '#2ca02c',
    'stuck': '#d62728',
}
DEFAULT_COLOR = '#ff7f0e'


def load_map(map_file):
    projector = make_projector()
    map_data, _errs = lanelet2.io.loadRobust(map_file, projector)
    return map_data


def ll_attr(ll, key, default=''):
    try:
        return ll.attributes[key]
    except Exception:
        return default


def lanelet_polygon_xy(ll):
    left = [(p.x, p.y) for p in ll.leftBound]
    right = [(p.x, p.y) for p in reversed(list(ll.rightBound))]
    return left + right


def bbox_hit(ll, xmin, xmax, ymin, ymax):
    for p in ll.centerline:
        if xmin <= p.x <= xmax and ymin <= p.y <= ymax:
            return True
    return False


def traffic_light_points(map_data, xmin, xmax, ymin, ymax):
    """Midpoint of each traffic-light-head linestring ('refers' role of
    traffic_light regulatory elements), deduped by regulatory element id."""
    seen_reg = set()
    pts = []
    for ll in map_data.laneletLayer:
        for reg in ll.regulatoryElements:
            if ll_attr(reg, 'subtype') != 'traffic_light' or reg.id in seen_reg:
                continue
            seen_reg.add(reg.id)
            try:
                refers = reg.parameters['refers']
            except Exception:
                continue
            for ls in refers:
                pts_xy = [(p.x, p.y) for p in ls]
                mx = sum(p[0] for p in pts_xy) / len(pts_xy)
                my = sum(p[1] for p in pts_xy) / len(pts_xy)
                if xmin <= mx <= xmax and ymin <= my <= ymax:
                    pts.append((mx, my))
    return pts


def read_trajectory(bag_dir):
    storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topics = {info.name for info in reader.get_all_topics_and_types()}
    topic = GT_TOPIC if GT_TOPIC in topics else FALLBACK_TOPIC

    pts = []
    while reader.has_next():
        t, data, _ts = reader.read_next()
        if t != topic:
            continue
        msg = deserialize_message(data, Odometry)
        pts.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
    return pts


def find_experiments(data_dir, goal_id):
    return sorted(glob.glob(os.path.join(data_dir, f'{goal_id}_*')))


def plot_goal(map_data, goal, data_dir, output_dir, margin):
    gid = goal['id']
    gx = goal['goal']['position']['x']
    gy = goal['goal']['position']['y']

    exp_dirs = find_experiments(data_dir, gid)
    if not exp_dirs:
        print(f'  {gid}: no experiment data in {data_dir}, skipping')
        return

    runs = []
    xs, ys = [SPAWN[0], gx], [SPAWN[1], gy]
    for exp_dir in exp_dirs:
        result_f = os.path.join(exp_dir, 'result.json')
        bag_dir = os.path.join(exp_dir, 'rosbag')
        if not (os.path.exists(result_f) and os.path.isdir(bag_dir)):
            continue
        result = json.load(open(result_f))
        status = result.get('status', 'unknown')
        traj = read_trajectory(bag_dir)
        if traj:
            xs.extend(p[0] for p in traj)
            ys.extend(p[1] for p in traj)
        runs.append({
            'run_id': os.path.basename(exp_dir),
            'status': status,
            'traj': traj,
        })

    if not runs:
        print(f'  {gid}: no readable trials, skipping')
        return

    xmin, xmax = min(xs) - margin, max(xs) + margin
    ymin, ymax = min(ys) - margin, max(ys) + margin

    fig, ax = plt.subplots(figsize=(10, 10))
    for ll in map_data.laneletLayer:
        if ll_attr(ll, 'subtype') != 'road':
            continue
        if not bbox_hit(ll, xmin, xmax, ymin, ymax):
            continue
        ax.add_patch(Polygon(lanelet_polygon_xy(ll), closed=True,
                              facecolor='#dddddd', edgecolor='#bbbbbb',
                              linewidth=0.3, zorder=1))

    tl_pts = traffic_light_points(map_data, xmin, xmax, ymin, ymax)
    if tl_pts:
        ax.scatter([p[0] for p in tl_pts], [p[1] for p in tl_pts],
                   marker='s', color='#f1c40f', edgecolor='#333333',
                   linewidth=0.5, s=40, zorder=2, label='traffic light')

    seen_status = set()
    for run in runs:
        traj = run['traj']
        if not traj:
            continue
        status = run['status']
        color = STATUS_COLOR.get(status, DEFAULT_COLOR)
        label = status if status not in seen_status else None
        seen_status.add(status)
        tx = [p[0] for p in traj]
        ty = [p[1] for p in traj]
        ax.plot(tx, ty, color=color, linewidth=1.8, alpha=0.85, zorder=3, label=label)
        if status != 'goal_reached':
            ax.plot(tx[-1], ty[-1], marker='x', color=color, markersize=10,
                    markeredgewidth=2.5, zorder=5)

    ax.plot(*SPAWN, marker='o', color='#1f77b4', markersize=9, zorder=4, label='start')
    ax.plot(gx, gy, marker='*', color='black', markersize=16, zorder=4, label='goal')

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal')
    captured_dist = goal.get('estimated_distance')
    dist_str = f'{captured_dist:.0f}m' if captured_dist is not None else 'unknown'
    ax.set_title(f'{gid}   route length: {dist_str}')
    ax.legend(loc='best', fontsize=7)
    ax.set_xlabel('map x (m)')
    ax.set_ylabel('map y (m)')

    out_path = os.path.join(output_dir, f'{gid}.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    statuses = ', '.join(f"{r['run_id']}={r['status']}" for r in runs)
    print(f'  {gid}: saved {out_path}  [{statuses}]')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', default=DEFAULT_DATA_DIR)
    parser.add_argument('--goals-file', default=DEFAULT_GOALS_FILE)
    parser.add_argument('--map-file', default=DEFAULT_MAP_FILE)
    parser.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    parser.add_argument('--goal', default=None,
                         help='Comma-separated goal IDs (default: all goals in --goals-file)')
    parser.add_argument('--margin', type=float, default=40.0,
                         help='Metres of padding around the route bounding box')
    args = parser.parse_args()

    goals_file = args.goals_file
    if not os.path.isabs(goals_file):
        goals_file = os.path.join(REPO_DIR, 'experiments', 'configs', goals_file)

    os.makedirs(args.output_dir, exist_ok=True)

    print(f'Loading lanelet2 map: {args.map_file}')
    map_data = load_map(args.map_file)

    goals_data = json.load(open(goals_file))
    goals = goals_data['goals']
    if args.goal:
        wanted = set(args.goal.split(','))
        goals = [g for g in goals if g['id'] in wanted]

    print(f'Plotting {len(goals)} goal(s) from {goals_file}')
    for goal in goals:
        plot_goal(map_data, goal, args.data_dir, args.output_dir, args.margin)


if __name__ == '__main__':
    main()
