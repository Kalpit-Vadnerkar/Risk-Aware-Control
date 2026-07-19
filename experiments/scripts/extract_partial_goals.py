#!/usr/bin/env python3
"""
Extract partial/intermediate goals from stuck experiments.

When an experiment gets stuck at a problematic map location, this script
analyzes the rosbag to find waypoints along the route BEFORE the stuck point.
These can be used as alternative shorter goals that avoid the problem area.

Usage:
  python3 extract_partial_goals.py <experiment_id>
  python3 extract_partial_goals.py goal_003_baseline_t1_20260203_194102
  python3 extract_partial_goals.py --all-stuck  # Process all stuck experiments

Output:
  - Creates partial goal entries with proper orientation
  - Saves to configs/partial_goals_<experiment_id>.json
  - Recommends the furthest safe position as replacement goal
"""

import os
import sys
import json
import math
import argparse
from datetime import datetime
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass

# Add lib to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

# ROS2 imports
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
EXPERIMENTS_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(EXPERIMENTS_DIR, 'data')
CONFIG_DIR = os.path.join(EXPERIMENTS_DIR, 'configs')

# Starting position (from baseline.json)
START_POSITION = {'x': 81384.60, 'y': 49922.00, 'z': 41.28}


@dataclass
class TrajectoryPoint:
    """Single point in vehicle trajectory."""
    time: float  # seconds from start
    x: float
    y: float
    z: float
    vx: float  # velocity components
    vy: float
    velocity: float  # magnitude
    heading: float  # radians


def read_trajectory_from_bag(bag_path: str) -> List[TrajectoryPoint]:
    """
    Read vehicle trajectory from rosbag using ground truth odometry.

    Uses proper ROS2 deserialization for reliable parsing.
    """
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic info
    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}

    # Prefer ground truth, fall back to localization
    odom_topic = None
    for topic in ['/awsim/ground_truth/localization/kinematic_state',
                  '/localization/kinematic_state']:
        if topic in topic_types:
            odom_topic = topic
            break

    if odom_topic is None:
        print("ERROR: No odometry topic found in rosbag")
        return []

    print(f"Using odometry topic: {odom_topic}")

    trajectory = []
    start_time_ns = None
    prev_point = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != odom_topic:
            continue

        if start_time_ns is None:
            start_time_ns = timestamp

        try:
            msg = deserialize_message(data, Odometry)
        except Exception as e:
            continue

        time_sec = (timestamp - start_time_ns) / 1e9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Get velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        velocity = math.sqrt(vx*vx + vy*vy)

        # Get heading from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        heading = 2.0 * math.atan2(qz, qw)

        point = TrajectoryPoint(
            time=time_sec, x=x, y=y, z=z,
            vx=vx, vy=vy, velocity=velocity, heading=heading
        )

        trajectory.append(point)
        prev_point = point

    return trajectory


def find_stuck_point(trajectory: List[TrajectoryPoint],
                     velocity_threshold: float = 0.5,
                     stuck_duration: float = 30.0,
                     movement_threshold: float = 1.0) -> Optional[int]:
    """
    Find the index where vehicle got TERMINALLY stuck (never recovered).

    This finds stop periods that extend to the end of the recording,
    indicating the vehicle never recovered from this stop.

    Returns index of the start of the terminal stuck period,
    or None if vehicle never got terminally stuck.
    """
    if not trajectory:
        return None

    # First, find when the vehicle started moving
    first_movement_idx = None
    for i, point in enumerate(trajectory):
        if point.velocity >= movement_threshold:
            first_movement_idx = i
            break

    if first_movement_idx is None:
        print("  Vehicle never started moving")
        return None

    print(f"  Vehicle started moving at t={trajectory[first_movement_idx].time:.1f}s")

    # Find stop periods
    sample_rate = len(trajectory) / trajectory[-1].time if trajectory[-1].time > 0 else 10
    min_stop_samples = int(5 * sample_rate)  # At least 5 seconds to count as a stop

    stop_periods = []
    consecutive_slow = 0
    stop_start_idx = None

    for i in range(first_movement_idx, len(trajectory)):
        point = trajectory[i]
        if point.velocity < velocity_threshold:
            if consecutive_slow == 0:
                stop_start_idx = i
            consecutive_slow += 1
        else:
            if consecutive_slow >= min_stop_samples:
                stop_periods.append({
                    'start_idx': stop_start_idx,
                    'end_idx': i,
                    'duration': trajectory[i].time - trajectory[stop_start_idx].time
                })
            consecutive_slow = 0

    # Check if ended while stopped (terminal stuck)
    if consecutive_slow >= min_stop_samples:
        terminal_duration = trajectory[-1].time - trajectory[stop_start_idx].time
        stop_periods.append({
            'start_idx': stop_start_idx,
            'end_idx': len(trajectory) - 1,
            'duration': terminal_duration,
            'terminal': True
        })

    if not stop_periods:
        return None

    print(f"  Found {len(stop_periods)} stop periods")

    # Look for terminal stuck period (one that extends to end)
    for sp in stop_periods:
        if sp.get('terminal') and sp['duration'] >= stuck_duration:
            idx = sp['start_idx']
            print(f"  Terminal stuck at t={trajectory[idx].time:.1f}s for {sp['duration']:.1f}s")
            return idx

    # If no terminal stuck, return None (vehicle completed or had recoverable stops)
    print("  No terminal stuck period found (vehicle recovered from all stops)")
    return None


def extract_waypoints(trajectory: List[TrajectoryPoint],
                      stuck_index: Optional[int] = None,
                      min_spacing: float = 50.0,
                      min_velocity: float = 1.0,
                      safety_margin: float = 30.0) -> List[TrajectoryPoint]:
    """
    Extract waypoints from trajectory that are safe to use as goals.

    Args:
        trajectory: Full trajectory
        stuck_index: Index where vehicle got stuck (waypoints stop before this)
        min_spacing: Minimum distance between waypoints (meters)
        min_velocity: Minimum velocity to consider a point "good" (m/s)
        safety_margin: Extra distance to stop before stuck point (meters)

    Returns:
        List of waypoints suitable for use as intermediate goals
    """
    if not trajectory:
        return []

    # Determine cutoff point
    if stuck_index is not None:
        # Stop before the stuck point with safety margin
        cutoff_point = trajectory[stuck_index]
        cutoff_dist_from_start = math.sqrt(
            (cutoff_point.x - START_POSITION['x'])**2 +
            (cutoff_point.y - START_POSITION['y'])**2
        )
        max_dist = cutoff_dist_from_start - safety_margin
    else:
        max_dist = float('inf')

    waypoints = []
    last_waypoint = None

    for point in trajectory:
        # Skip points where vehicle was too slow
        if point.velocity < min_velocity:
            continue

        # Calculate distance from start
        dist_from_start = math.sqrt(
            (point.x - START_POSITION['x'])**2 +
            (point.y - START_POSITION['y'])**2
        )

        # Skip if past the safe distance
        if dist_from_start > max_dist:
            continue

        # Check spacing from last waypoint
        if last_waypoint is not None:
            dist_from_last = math.sqrt(
                (point.x - last_waypoint.x)**2 +
                (point.y - last_waypoint.y)**2
            )
            if dist_from_last < min_spacing:
                continue

        waypoints.append(point)
        last_waypoint = point

    return waypoints


def create_goal_entry(waypoint: TrajectoryPoint,
                      original_goal_id: str,
                      waypoint_index: int) -> Dict:
    """Create a goal entry in captured_goals.json format."""

    # Convert heading to quaternion (only z and w needed for 2D)
    qz = math.sin(waypoint.heading / 2)
    qw = math.cos(waypoint.heading / 2)

    # Calculate distance from start
    dist_from_start = math.sqrt(
        (waypoint.x - START_POSITION['x'])**2 +
        (waypoint.y - START_POSITION['y'])**2
    )

    return {
        'id': f"{original_goal_id}_wp{waypoint_index:02d}",
        'goal': {
            'position': {
                'x': waypoint.x,
                'y': waypoint.y,
                'z': waypoint.z
            },
            'orientation': {
                'z': qz,
                'w': qw
            },
            'frame_id': 'map',
            'start_position': START_POSITION.copy(),
            'timestamp': datetime.now().isoformat()
        },
        'captured_at': datetime.now().isoformat(),
        'estimated_distance': round(dist_from_start, 1),
        'source': 'extracted_from_trajectory',
        'original_goal': original_goal_id,
        'trajectory_time': waypoint.time,
        'velocity_at_point': round(waypoint.velocity, 2)
    }


def process_experiment(experiment_id: str,
                       output_dir: str = None) -> Optional[List[Dict]]:
    """
    Process a single stuck experiment and extract intermediate goals.

    Returns list of goal entries or None on failure.
    """
    # Find rosbag path
    exp_dir = os.path.join(DATA_DIR, experiment_id)
    if not os.path.exists(exp_dir):
        # Try to find by prefix
        matches = [d for d in os.listdir(DATA_DIR) if d.startswith(experiment_id)]
        if matches:
            experiment_id = matches[0]
            exp_dir = os.path.join(DATA_DIR, experiment_id)
        else:
            print(f"ERROR: Experiment not found: {experiment_id}")
            return None

    rosbag_path = os.path.join(exp_dir, 'rosbag')
    if not os.path.exists(rosbag_path):
        print(f"ERROR: Rosbag not found: {rosbag_path}")
        return None

    print(f"\n{'='*60}")
    print(f"Processing: {experiment_id}")
    print(f"{'='*60}")

    # Read trajectory
    print("Reading trajectory from rosbag...")
    trajectory = read_trajectory_from_bag(rosbag_path)

    if not trajectory:
        print("ERROR: Could not read trajectory")
        return None

    print(f"Read {len(trajectory)} trajectory points over {trajectory[-1].time:.1f}s")

    # Find stuck point
    stuck_index = find_stuck_point(trajectory)
    if stuck_index is not None:
        stuck_point = trajectory[stuck_index]
        print(f"Vehicle got stuck at t={stuck_point.time:.1f}s, position=({stuck_point.x:.1f}, {stuck_point.y:.1f})")
    else:
        print("Vehicle did not appear to get stuck (using full trajectory)")

    # Extract waypoints
    waypoints = extract_waypoints(trajectory, stuck_index)

    if not waypoints:
        print("No suitable waypoints found")
        return None

    print(f"Extracted {len(waypoints)} waypoints:")

    # Extract original goal id (e.g., "goal_003" from "goal_003_baseline_t1_20260203_194102")
    parts = experiment_id.split('_')
    if len(parts) >= 2 and parts[0] == 'goal':
        original_goal_id = f"{parts[0]}_{parts[1]}"
    else:
        original_goal_id = experiment_id

    # Create goal entries
    goals = []
    for i, wp in enumerate(waypoints):
        dist = math.sqrt((wp.x - START_POSITION['x'])**2 + (wp.y - START_POSITION['y'])**2)
        print(f"  {i+1}. t={wp.time:.1f}s, pos=({wp.x:.1f}, {wp.y:.1f}), dist={dist:.0f}m, vel={wp.velocity:.1f}m/s")
        goal = create_goal_entry(wp, original_goal_id, i+1)
        goals.append(goal)

    # Save to file
    if output_dir is None:
        output_dir = CONFIG_DIR

    output_file = os.path.join(output_dir, f'partial_goals_{original_goal_id}.json')
    output_data = {
        'source_experiment': experiment_id,
        'extraction_time': datetime.now().isoformat(),
        'stuck_at_time': trajectory[stuck_index].time if stuck_index else None,
        'total_waypoints': len(goals),
        'goals': goals
    }

    with open(output_file, 'w') as f:
        json.dump(output_data, f, indent=2)

    print(f"\nSaved {len(goals)} waypoints to: {output_file}")

    # Recommend the furthest safe position
    if goals:
        furthest = max(goals, key=lambda g: g['estimated_distance'])
        print(f"\nRecommended replacement goal (furthest safe point):")
        print(f"  ID: {furthest['id']}")
        print(f"  Position: ({furthest['goal']['position']['x']:.1f}, {furthest['goal']['position']['y']:.1f})")
        print(f"  Distance: {furthest['estimated_distance']:.0f}m")

    return goals


def process_all_stuck():
    """Process all experiments marked as stuck in results."""

    # Find stuck experiments
    stuck_experiments = []

    for exp_dir in sorted(os.listdir(DATA_DIR)):
        result_file = os.path.join(DATA_DIR, exp_dir, 'result.json')
        if os.path.exists(result_file):
            with open(result_file) as f:
                result = json.load(f)
            if result.get('status') == 'stuck':
                stuck_experiments.append(exp_dir)

    if not stuck_experiments:
        print("No stuck experiments found")
        return

    print(f"Found {len(stuck_experiments)} stuck experiments:")
    for exp in stuck_experiments:
        print(f"  - {exp}")

    # Process each
    all_goals = []
    for exp in stuck_experiments:
        goals = process_experiment(exp)
        if goals:
            all_goals.extend(goals)

    # Save combined file
    if all_goals:
        output_file = os.path.join(CONFIG_DIR, 'partial_goals_all_stuck.json')
        output_data = {
            'extraction_time': datetime.now().isoformat(),
            'source_experiments': stuck_experiments,
            'total_goals': len(all_goals),
            'goals': all_goals
        }

        with open(output_file, 'w') as f:
            json.dump(output_data, f, indent=2)

        print(f"\n{'='*60}")
        print(f"Combined {len(all_goals)} waypoints from {len(stuck_experiments)} experiments")
        print(f"Saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Extract intermediate goals from stuck experiments')
    parser.add_argument('experiment_id', nargs='?', help='Experiment ID to process')
    parser.add_argument('--all-stuck', action='store_true', help='Process all stuck experiments')
    parser.add_argument('--output-dir', help='Output directory for goal files')

    args = parser.parse_args()

    if args.all_stuck:
        process_all_stuck()
    elif args.experiment_id:
        process_experiment(args.experiment_id, args.output_dir)
    else:
        parser.print_help()
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
