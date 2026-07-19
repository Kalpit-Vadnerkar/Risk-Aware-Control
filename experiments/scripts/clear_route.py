#!/usr/bin/env python3
"""
Clear the current route. Use this between setting goals in capture mode.
"""

import subprocess
import sys

def main():
    cmd = [
        'ros2', 'service', 'call',
        '/api/routing/clear_route',
        'autoware_adapi_v1_msgs/srv/ClearRoute'
    ]

    print("Clearing route...")
    result = subprocess.run(cmd, capture_output=True, text=True)

    if 'success=True' in result.stdout or 'success: true' in result.stdout.lower():
        print("Route cleared successfully.")
        return 0
    else:
        print(f"Result: {result.stdout}")
        if result.stderr:
            print(f"Error: {result.stderr}")
        return 1

if __name__ == '__main__':
    sys.exit(main())
