#!/bin/bash

# Autoware Launch Script for AWSIM Labs
# This script launches Autoware configured for AWSIM Labs simulation

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Autoware + AWSIM Labs Launcher${NC}"
echo -e "${GREEN}========================================${NC}"

# Configuration
AUTOWARE_DIR="/home/df/Desktop/Kalpit-2026/Risk-Aware-Control/autoware"
MAP_PATH="/home/df/Desktop/Kalpit-2026/Risk-Aware-Control/Shinjuku-Map/map"
VEHICLE_MODEL="awsim_labs_vehicle"
SENSOR_MODEL="awsim_labs_sensor_kit"

# Verify directories exist
if [ ! -d "$AUTOWARE_DIR" ]; then
    echo -e "${YELLOW}Error: Autoware directory not found at ${AUTOWARE_DIR}${NC}"
    exit 1
fi

if [ ! -d "$MAP_PATH" ]; then
    echo -e "${YELLOW}Error: Map directory not found at ${MAP_PATH}${NC}"
    echo "Please update MAP_PATH in this script"
    exit 1
fi

# Navigate to Autoware directory
cd "$AUTOWARE_DIR" || exit 1

# Source ROS2 and Autoware setup
echo -e "${BLUE}Sourcing ROS2 Humble...${NC}"
source /opt/ros/humble/setup.bash

echo -e "${BLUE}Sourcing Autoware workspace...${NC}"
source autoware/install/setup.bash

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
TIMEOUT=60
ELAPSED=0
while ! ros2 topic list | grep -q "/sensing/lidar/top/pointcloud_raw"; do
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo -e "${YELLOW}Warning: AWSIM topics not detected after ${TIMEOUT}s${NC}"
        echo -e "${YELLOW}Make sure AWSIM is running before launching Autoware${NC}"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
        break
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
ros2 launch autoware_launch e2e_simulator.launch.xml \
    vehicle_model:=${VEHICLE_MODEL} \
    sensor_model:=${SENSOR_MODEL} \
    map_path:=${MAP_PATH} \
    launch_vehicle_interface:=true

echo -e "${GREEN}Autoware shutdown complete${NC}"
