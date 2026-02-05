#!/usr/bin/env python3
"""
Extract partial goals from stuck experiments.

When an experiment gets stuck, this script analyzes the rosbag to find
the last known good position before the vehicle stopped. This position
can be used as a new shorter goal that avoids the problematic area.

Usage:
  python3 extract_partial_goals.py <experiment_id>
  python3 extract_partial_goals.py test2

This will:
  1. Load the rosbag from the experiment
  2. Find the position where the vehicle was moving well
  3. Create a new goal entry that can be added to captured_goals.json
"""

import sqlite3
import json
import os
import sys
import math
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, '..', 'data')
CONFIG_DIR = os.path.join(SCRIPT_DIR, '..', 'configs')


def analyze_rosbag(rosbag_path):
    """Analyze rosbag to find vehicle trajectory and identify good positions"""

    db_path = os.path.join(rosbag_path, 'rosbag_0.db3')
    if not os.path.exists(db_path):
        print(f"ERROR: Rosbag not found: {db_path}")
        return None

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Get topic IDs
    cursor.execute("SELECT id, name FROM topics")
    topics = {row[1]: row[0] for row in cursor.fetchall()}

    odom_topic_id = topics.get('/awsim/ground_truth/localization/kinematic_state')
    if not odom_topic_id:
        print("ERROR: Ground truth odometry topic not found in rosbag")
        return None

    # Get start time
    cursor.execute("SELECT MIN(timestamp) FROM messages")
    start_time = cursor.fetchone()[0]

    # Get all odometry messages
    cursor.execute("""
        SELECT timestamp, data FROM messages
        WHERE topic_id = ?
        ORDER BY timestamp
    """, (odom_topic_id,))

    messages = cursor.fetchall()
    conn.close()

    print(f"Analyzing {len(messages)} odometry messages...")

    # Extract positions and velocities
    trajectory = []
    for ts, data in messages:
        rel_time = (ts - start_time) / 1e9

        # Parse odometry data (simplified - just extract position)
        # The actual parsing would need proper CDR deserialization
        # For now, we'll use a simple approach based on data structure
        if len(data) >= 100:
            try:
                import struct
                # Odometry message has pose at a known offset after header
                # This is approximate - proper parsing would use ROS2 deserialization
                # Position is typically at offset 56 (after header + frame_id string)
                # Try to find the position floats

                # For AWSIM odometry, position is typically:
                # offset ~56-80 for x, y, z (double precision)
                # Let's try to find reasonable coordinates

                # Skip CDR header and look for position pattern
                # Position values should be in range ~81000-82000 for x, ~49000-51000 for y
                for offset in range(0, min(200, len(data)-24), 8):
                    try:
                        x = struct.unpack_from('d', data, offset)[0]
                        y = struct.unpack_from('d', data, offset+8)[0]
                        z = struct.unpack_from('d', data, offset+16)[0]

                        # Check if this looks like valid map coordinates
                        if 80000 < x < 83000 and 49000 < y < 52000 and 30 < z < 50:
                            trajectory.append({
                                'time': rel_time,
                                'x': x,
                                'y': y,
                                'z': z
                            })
                            break
                    except:
                        pass
            except Exception as e:
                pass

    if not trajectory:
        print("WARNING: Could not parse trajectory from rosbag")
        return None

    print(f"Extracted {len(trajectory)} position samples")
    return trajectory


def find_good_positions(trajectory, min_distance=50, velocity_threshold=2.0):
    """
    Find positions where the vehicle was moving well.

    Returns positions that are:
    - At least min_distance apart
    - Where the vehicle was moving (velocity > threshold)
    """

    if len(trajectory) < 10:
        return []

    good_positions = []

    # Calculate velocities
    for i in range(1, len(trajectory)):
        dt = trajectory[i]['time'] - trajectory[i-1]['time']
        if dt > 0:
            dx = trajectory[i]['x'] - trajectory[i-1]['x']
            dy = trajectory[i]['y'] - trajectory[i-1]['y']
            vel = math.sqrt(dx*dx + dy*dy) / dt
            trajectory[i]['velocity'] = vel
        else:
            trajectory[i]['velocity'] = 0

    # Find the point where vehicle first got stuck (velocity dropped)
    stuck_time = None
    stuck_index = None
    idle_count = 0

    for i in range(1, len(trajectory)):
        if trajectory[i].get('velocity', 0) < velocity_threshold:
            idle_count += 1
            if idle_count > 100:  # ~10 seconds of being stuck
                stuck_time = trajectory[i]['time']
                stuck_index = i - 100  # Go back to before stuck
                break
        else:
            idle_count = 0

    if stuck_index is None:
        stuck_index = len(trajectory) - 1

    print(f"Vehicle appears to have gotten stuck around t={stuck_time:.1f}s" if stuck_time else "")

    # Find good positions leading up to stuck point
    last_good_pos = None

    for i in range(0, stuck_index):
        pos = trajectory[i]
        vel = pos.get('velocity', 0)

        # Check if this is a good position (vehicle was moving)
        if vel >= velocity_threshold:
            # Check distance from last good position
            if last_good_pos is None:
                dist = float('inf')
            else:
                dist = math.sqrt(
                    (pos['x'] - last_good_pos['x'])**2 +
                    (pos['y'] - last_good_pos['y'])**2
                )

            if dist >= min_distance:
                good_positions.append({
                    'time': pos['time'],
                    'x': pos['x'],
                    'y': pos['y'],
                    'z': pos['z'],
                    'velocity': vel
                })
                last_good_pos = pos

    return good_positions


def create_goal_entry(position, base_goal_id, index):
    """Create a goal entry from a position"""

    # Use default orientation (facing direction of travel)
    # In practice, you'd want to calculate this from the trajectory
    return {
        'id': f"{base_goal_id}_partial_{index:02d}",
        'goal': {
            'position': {
                'x': position['x'],
                'y': position['y'],
                'z': position['z']
            },
            'orientation': {
                'z': 0.0,  # Would need to calculate from trajectory
                'w': 1.0
            },
            'frame_id': 'map',
            'start_position': {
                'x': 81384.60,
                'y': 49922.00,
                'z': 41.28
            },
            'timestamp': datetime.now().isoformat()
        },
        'captured_at': datetime.now().isoformat(),
        'source': 'extracted_from_stuck',
        'original_time': position['time']
    }


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 extract_partial_goals.py <experiment_id>")
        print("Example: python3 extract_partial_goals.py test2")
        return 1

    experiment_id = sys.argv[1]
    rosbag_path = os.path.join(DATA_DIR, experiment_id, 'rosbag')

    if not os.path.exists(rosbag_path):
        print(f"ERROR: Experiment data not found: {rosbag_path}")
        return 1

    print("=" * 60)
    print(f"EXTRACTING PARTIAL GOALS FROM: {experiment_id}")
    print("=" * 60)

    # Analyze trajectory
    trajectory = analyze_rosbag(rosbag_path)
    if not trajectory:
        return 1

    # Find good positions
    good_positions = find_good_positions(trajectory)

    if not good_positions:
        print("No good positions found in trajectory")
        return 1

    print(f"\nFound {len(good_positions)} candidate positions:")
    for i, pos in enumerate(good_positions):
        print(f"  {i+1}. t={pos['time']:.1f}s: ({pos['x']:.2f}, {pos['y']:.2f}) vel={pos['velocity']:.1f}m/s")

    # Create goal entries
    new_goals = []
    for i, pos in enumerate(good_positions):
        goal = create_goal_entry(pos, experiment_id, i+1)

        # Calculate estimated distance from start
        start_x, start_y = 81384.60, 49922.00
        dist = math.sqrt((pos['x'] - start_x)**2 + (pos['y'] - start_y)**2)
        goal['estimated_distance'] = round(dist, 1)

        new_goals.append(goal)

    # Save to a separate file
    output_file = os.path.join(CONFIG_DIR, f'partial_goals_{experiment_id}.json')
    output_data = {
        'source_experiment': experiment_id,
        'extraction_time': datetime.now().isoformat(),
        'total_goals': len(new_goals),
        'goals': new_goals
    }

    with open(output_file, 'w') as f:
        json.dump(output_data, f, indent=2)

    print(f"\nSaved {len(new_goals)} partial goals to: {output_file}")

    # Show the furthest position as the recommended new goal
    if new_goals:
        furthest = max(new_goals, key=lambda g: g['estimated_distance'])
        print(f"\nRecommended new goal (furthest safe position):")
        print(f"  ID: {furthest['id']}")
        print(f"  Position: ({furthest['goal']['position']['x']:.2f}, {furthest['goal']['position']['y']:.2f})")
        print(f"  Distance from start: {furthest['estimated_distance']:.0f}m")
        print(f"  Time in original run: {furthest['original_time']:.1f}s")

    return 0


if __name__ == '__main__':
    sys.exit(main())
