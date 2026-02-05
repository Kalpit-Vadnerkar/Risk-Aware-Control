#!/bin/bash
#
# Kill all AWSIM and Autoware processes
#
# Usage: ./cleanup.sh
#

echo "Killing all AWSIM and Autoware processes..."

# Kill AWSIM
pkill -9 -f "awsim_labs" 2>/dev/null && echo "  Killed AWSIM" || echo "  No AWSIM running"

# Kill Autoware components
pkill -9 -f "ros2.*launch.*autoware" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "autoware" 2>/dev/null || true

# Check if any remain
sleep 1
AWSIM_COUNT=$(pgrep -f "awsim_labs" | wc -l)
AUTOWARE_COUNT=$(pgrep -f "autoware" | wc -l)
CONTAINER_COUNT=$(pgrep -f "component_container" | wc -l)

if [ "$AWSIM_COUNT" -gt 0 ] || [ "$AUTOWARE_COUNT" -gt 0 ] || [ "$CONTAINER_COUNT" -gt 0 ]; then
    echo ""
    echo "WARNING: Some processes still running:"
    echo "  AWSIM: $AWSIM_COUNT"
    echo "  Autoware: $AUTOWARE_COUNT"
    echo "  Containers: $CONTAINER_COUNT"
    echo ""
    echo "Try running: sudo pkill -9 -f 'component_container|autoware|awsim'"
else
    echo ""
    echo "All processes killed successfully."
fi

# Reset ROS2 daemon
echo "Resetting ROS2 daemon..."
ros2 daemon stop 2>/dev/null || true
ros2 daemon start 2>/dev/null || true

echo "Cleanup complete."
