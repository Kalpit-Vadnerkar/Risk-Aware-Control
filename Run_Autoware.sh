#!/bin/bash

# Autoware Launch Script for AWSIM Labs
# This script launches Autoware configured for AWSIM Labs simulation
#
# Usage: ./Run_Autoware.sh
# Must be run after AWSIM is started.

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Autoware + AWSIM Labs Launcher${NC}"
echo -e "${GREEN}========================================${NC}"

# Kill any existing Autoware processes to ensure fresh start
echo -e "${BLUE}Cleaning up existing Autoware processes...${NC}"
pkill -9 -f "ros2.*launch.*autoware" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "autoware" 2>/dev/null || true
sleep 2

# Note: Don't reset ROS2 daemon here - AWSIM is already using it
# If you have issues, run ./cleanup.sh before starting AWSIM

# Directory setup (derive from script location, same pattern as Run_AWSIM.sh)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOWARE_DIR="$SCRIPT_DIR/autoware"
MAP_PATH="$SCRIPT_DIR/Shinjuku-Map/map"
VEHICLE_MODEL="awsim_labs_vehicle"
SENSOR_MODEL="awsim_labs_sensor_kit"

# Verify directories exist
if [ ! -d "$AUTOWARE_DIR" ]; then
    echo -e "${YELLOW}Error: Autoware directory not found at ${AUTOWARE_DIR}${NC}"
    exit 1
fi

if [ ! -d "$MAP_PATH" ]; then
    echo -e "${YELLOW}Error: Map directory not found at ${MAP_PATH}${NC}"
    exit 1
fi

# Navigate to Autoware directory
cd "$AUTOWARE_DIR" || exit 1

# Source ROS2 and Autoware setup
echo -e "${BLUE}Sourcing ROS2 Humble...${NC}"
source /opt/ros/humble/setup.bash

echo -e "${BLUE}Sourcing Autoware workspace...${NC}"
source "$AUTOWARE_DIR/install/setup.bash"

# Configure network settings for ROS2 (only needs to run once per boot)
if [ ! -e /tmp/cycloneDDS_configured ]; then
    echo -e "${BLUE}Configuring network settings...${NC}"
    sudo sysctl -w net.core.rmem_max=2147483647
    sudo sysctl -w net.ipv4.ipfrag_time=3
    sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
    sudo ip link set lo multicast on
    touch /tmp/cycloneDDS_configured
    echo -e "${GREEN}Network configured successfully${NC}"
fi

# Wait for AWSIM to be ready (check if topics are publishing)
echo -e "${BLUE}Waiting for AWSIM to start publishing topics...${NC}"
TIMEOUT=120
ELAPSED=0
while ! ros2 topic list | grep -q "/sensing/lidar/top/pointcloud_raw"; do
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo -e "${YELLOW}Warning: AWSIM topics not detected after ${TIMEOUT}s${NC}"
        echo -e "${YELLOW}Make sure AWSIM is running before launching Autoware${NC}"
        exit 1
    fi
    echo -e "${BLUE}  Waiting... ($ELAPSED/${TIMEOUT}s)${NC}"
    sleep 2
    ELAPSED=$((ELAPSED + 2))
done

echo -e "${GREEN}AWSIM topics detected!${NC}"
echo ""
echo -e "${GREEN}Launching Autoware...${NC}"
echo -e "${BLUE}  Vehicle Model: ${VEHICLE_MODEL}${NC}"
echo -e "${BLUE}  Sensor Model:  ${SENSOR_MODEL}${NC}"
echo -e "${BLUE}  Map Path:      ${MAP_PATH}${NC}"
echo ""

# Launch Autoware
exec ros2 launch autoware_launch e2e_simulator.launch.xml \
    vehicle_model:=${VEHICLE_MODEL} \
    sensor_model:=${SENSOR_MODEL} \
    map_path:=${MAP_PATH} \
    launch_vehicle_interface:=true

echo -e "${GREEN}Autoware shutdown complete${NC}"
