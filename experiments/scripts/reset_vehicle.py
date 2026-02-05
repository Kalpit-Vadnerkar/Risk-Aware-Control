#!/usr/bin/env python3
"""
Reset the vehicle to the starting position between experiments.

This uses the same mechanism as RViz's "2D Pose Estimate" tool:
1. Publishes to /initialpose and /initialpose3d
2. AWSIM teleports the vehicle to that position
3. Autoware re-initializes localization

Usage:
  python3 reset_vehicle.py [--wait]

Options:
  --wait    Wait for localization to stabilize after reset
"""

import subprocess
import sys
import os
import json
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(SCRIPT_DIR, '..', 'configs')


def get_start_position():
    """Get the starting position from baseline config"""
    baseline_config = os.path.join(CONFIG_DIR, 'baseline.json')

    if os.path.exists(baseline_config):
        with open(baseline_config, 'r') as f:
            config = json.load(f)
            ego = config.get('egoConfiguration', {})
            pos = ego.get('egoPosition', {})
            angles = ego.get('egoEulerAngles', {})

            # Convert euler angles to quaternion
            import math
            yaw_rad = math.radians(angles.get('z', 33.5))
            qz = math.sin(yaw_rad / 2)
            qw = math.cos(yaw_rad / 2)

            return {
                'position': {
                    'x': pos.get('x', 81384.60),
                    'y': pos.get('y', 49922.00),
                    'z': pos.get('z', 41.28)
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': qz,
                    'w': qw
                },
                'frame_id': 'map'
            }

    # Default starting position
    import math
    yaw_rad = math.radians(33.5)
    return {
        'position': {'x': 81384.60, 'y': 49922.00, 'z': 41.28},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw_rad/2), 'w': math.cos(yaw_rad/2)},
        'frame_id': 'map'
    }


def clear_route():
    """Clear any existing route"""
    print("Clearing route...")
    try:
        subprocess.run(
            ['ros2', 'service', 'call', '/api/routing/clear_route',
             'autoware_adapi_v1_msgs/srv/ClearRoute', '{}'],
            capture_output=True, timeout=10
        )
        time.sleep(1)
        return True
    except Exception as e:
        print(f"  Warning: Could not clear route: {e}")
        return False


def reset_to_start(wait_for_localization=False):
    """Reset vehicle to starting position"""
    # Create temporary config for set_initial_pose.py
    pose = get_start_position()
    temp_config = os.path.join(CONFIG_DIR, 'reset_pose.json')

    with open(temp_config, 'w') as f:
        json.dump(pose, f, indent=2)

    print(f"Resetting to: ({pose['position']['x']:.2f}, {pose['position']['y']:.2f})")

    # Use set_initial_pose.py to do the actual reset
    set_pose_script = os.path.join(SCRIPT_DIR, 'set_initial_pose.py')

    cmd = ['python3', set_pose_script, '--config', temp_config]
    if wait_for_localization:
        cmd.append('--wait')

    result = subprocess.run(cmd)

    # Clean up temp config
    try:
        os.remove(temp_config)
    except:
        pass

    return result.returncode == 0


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Reset vehicle to starting position')
    parser.add_argument('--wait', action='store_true',
                        help='Wait for localization to stabilize after reset')
    args = parser.parse_args()

    print("=" * 50)
    print("VEHICLE RESET")
    print("=" * 50)

    # Step 1: Clear any existing route
    clear_route()

    # Step 2: Reset to start position
    # This teleports in AWSIM and re-initializes Autoware localization
    success = reset_to_start(wait_for_localization=args.wait)

    if success:
        print("\nReset complete!")
        print("Vehicle should now be at the starting position.")
        return 0
    else:
        print("\nReset may have had issues. Check Autoware state.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
