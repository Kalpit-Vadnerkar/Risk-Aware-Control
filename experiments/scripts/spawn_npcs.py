#!/usr/bin/env python3
"""
Spawn additional NPC vehicles via AWSIM's ROS2 entity controller.

AWSIM exposes entity spawning via the topic:
  awsim/entity_controller/spawn (entity_controller_msgs/SpawnEntity)

This script first checks if the entity controller topics exist.
If they do, it spawns NPC vehicles at specified positions.

NOTE: The entity_controller_msgs are custom AWSIM messages. If they are
not available as ROS2 packages, this script falls back to raw topic
publishing using geometry_msgs (which won't work but will report the issue).

Usage:
  python3 spawn_npcs.py [num_vehicles]
  Default: 10 vehicles

Must be run AFTER AWSIM is started.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import subprocess
import sys
import time
import math
import random


def check_entity_controller_topics():
    """Check if AWSIM entity controller topics exist."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=10
        )
        topics = result.stdout.strip().split('\n')
        entity_topics = [t for t in topics if 'entity_controller' in t]
        return entity_topics
    except Exception as e:
        print(f"Error checking topics: {e}")
        return []


def check_entity_msg_available():
    """Check if entity_controller_msgs is available."""
    try:
        result = subprocess.run(
            ['ros2', 'interface', 'show', 'entity_controller_msgs/msg/SpawnEntity'],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            return True, result.stdout.strip()
        return False, result.stderr.strip()
    except Exception as e:
        return False, str(e)


def spawn_via_ros2_cli(asset_key, unique_id, x, y, z, yaw_deg):
    """Try spawning via ros2 topic pub (requires message type to be available)."""
    # Convert yaw to quaternion
    yaw_rad = math.radians(yaw_deg)
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)

    msg_yaml = (
        f"{{asset_key: '{asset_key}', unique_id: '{unique_id}', "
        f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}}}}}"
    )

    cmd = [
        'ros2', 'topic', 'pub', '--once',
        'awsim/entity_controller/spawn',
        'entity_controller_msgs/msg/SpawnEntity',
        msg_yaml
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"  Spawned {unique_id} ({asset_key}) at ({x:.1f}, {y:.1f})")
            return True
        else:
            print(f"  Failed to spawn {unique_id}: {result.stderr.strip()}")
            return False
    except Exception as e:
        print(f"  Error spawning {unique_id}: {e}")
        return False


def main():
    num_vehicles = int(sys.argv[1]) if len(sys.argv) > 1 else 10

    print("=" * 50)
    print("AWSIM NPC Spawner")
    print("=" * 50)

    # Step 1: Check if entity controller topics exist
    print("\n[1/3] Checking entity controller topics...")
    entity_topics = check_entity_controller_topics()

    if entity_topics:
        print(f"  Found {len(entity_topics)} entity controller topics:")
        for t in entity_topics:
            print(f"    {t}")
    else:
        print("  No entity controller topics found.")
        print("  The AWSIM binary may not have the entity controller enabled.")
        print("  Traffic density is controlled by AWSIM's built-in settings.")
        print("\n  Options:")
        print("  1. Check if AWSIM has a UI traffic density slider")
        print("  2. Rebuild AWSIM from source with higher MaxVehicleCount")
        print("  3. Use scenario_simulator_v2 for complex NPC scenarios")
        sys.exit(1)

    # Step 2: Check if message type is available
    print("\n[2/3] Checking entity_controller_msgs availability...")
    msg_available, msg_info = check_entity_msg_available()

    if msg_available:
        print(f"  Message type available:")
        for line in msg_info.split('\n'):
            print(f"    {line}")
    else:
        print(f"  entity_controller_msgs NOT available in ROS2 environment.")
        print(f"  Detail: {msg_info}")
        print("\n  The entity controller topics exist but we can't publish to them")
        print("  without the message definitions. Options:")
        print("  1. Build entity_controller_msgs as a ROS2 package")
        print("  2. Use AWSIM's UI slider for traffic density")
        sys.exit(1)

    # Step 3: Spawn vehicles
    print(f"\n[3/3] Spawning {num_vehicles} NPC vehicles...")

    # Spawn positions along the Shinjuku route (offset from ego path)
    # Ego starts at approximately (81382, 49918) heading ~35 degrees
    vehicle_types = ["smallcar", "taxi", "van", "hatchback"]
    base_x, base_y, base_z = 81382.0, 49918.0, 41.5

    random.seed(42)  # Reproducible spawns
    spawned = 0

    for i in range(num_vehicles):
        asset_key = vehicle_types[i % len(vehicle_types)]
        unique_id = f"npc_{i:03d}"

        # Spread vehicles in a grid offset from the ego start
        offset_x = random.uniform(-50, 100)
        offset_y = random.uniform(-50, 100)
        yaw = random.uniform(0, 360)

        x = base_x + offset_x
        y = base_y + offset_y

        if spawn_via_ros2_cli(asset_key, unique_id, x, y, base_z, yaw):
            spawned += 1
            time.sleep(0.5)  # Don't flood

    print(f"\nSpawned {spawned}/{num_vehicles} vehicles.")


if __name__ == '__main__':
    main()
