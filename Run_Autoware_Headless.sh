#!/bin/bash

# Autoware Launch Script for AWSIM Labs (HEADLESS MODE)
# This script launches Autoware WITHOUT RViz for automated experiments.
#
# Usage: ./Run_Autoware_Headless.sh
# Must be run after AWSIM is started.
#
# Environment variables:
#   SET_INITIAL_POSE=true  - Force initial pose setting (default: false, not needed if AWSIM spawns correctly)

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Autoware + AWSIM Labs (HEADLESS)${NC}"
echo -e "${GREEN}========================================${NC}"

# Kill any existing Autoware processes to ensure fresh start
echo -e "${BLUE}Cleaning up existing Autoware processes...${NC}"
pkill -9 -f "ros2.*launch.*autoware" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "autoware" 2>/dev/null || true
sleep 2

# Directory setup (derive from script location)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOWARE_DIR="$SCRIPT_DIR/autoware"
MAP_PATH="$SCRIPT_DIR/Shinjuku-Map/map"
VEHICLE_MODEL="awsim_labs_vehicle"
SENSOR_MODEL="awsim_labs_sensor_kit"

# Verify directories exist
if [ ! -d "$AUTOWARE_DIR" ]; then
    echo -e "${RED}Error: Autoware directory not found at ${AUTOWARE_DIR}${NC}"
    exit 1
fi

if [ ! -d "$MAP_PATH" ]; then
    echo -e "${RED}Error: Map directory not found at ${MAP_PATH}${NC}"
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
TIMEOUT=60
ELAPSED=0
while ! ros2 topic list 2>/dev/null | grep -q "/sensing/lidar/top/pointcloud_raw"; do
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo -e "${RED}Error: AWSIM topics not detected after ${TIMEOUT}s${NC}"
        echo -e "${RED}Make sure AWSIM is running before launching Autoware${NC}"
        exit 1
    fi
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    if [ $((ELAPSED % 10)) -eq 0 ]; then
        echo -e "${BLUE}  Waiting... ($ELAPSED/${TIMEOUT}s)${NC}"
    fi
done

echo -e "${GREEN}AWSIM topics detected!${NC}"
echo ""
echo -e "${GREEN}Launching Autoware (HEADLESS - no RViz)...${NC}"
echo -e "${BLUE}  Vehicle Model: ${VEHICLE_MODEL}${NC}"
echo -e "${BLUE}  Sensor Model:  ${SENSOR_MODEL}${NC}"
echo -e "${BLUE}  Map Path:      ${MAP_PATH}${NC}"
echo -e "${BLUE}  RViz:          DISABLED${NC}"
echo ""

# Launch Autoware without RViz
ros2 launch autoware_launch e2e_simulator.launch.xml \
    vehicle_model:=${VEHICLE_MODEL} \
    sensor_model:=${SENSOR_MODEL} \
    map_path:=${MAP_PATH} \
    launch_vehicle_interface:=true \
    rviz:=false &

AUTOWARE_PID=$!

# Wait for Autoware to reach WAITING_FOR_ROUTE state
# This is the only check that matters - it means localization is working
echo -e "${BLUE}Waiting for Autoware to initialize (WAITING_FOR_ROUTE state)...${NC}"
TIMEOUT=120
ELAPSED=0
STATE_NAMES=("UNKNOWN" "INITIALIZING" "WAITING_FOR_ROUTE" "PLANNING" "WAITING_FOR_ENGAGE" "DRIVING" "ARRIVED_GOAL")

while true; do
    AUTOWARE_STATE=$(ros2 topic echo /autoware/state --once 2>/dev/null | grep "state:" | head -1 | awk '{print $2}')

    if [ -n "$AUTOWARE_STATE" ] && [ "$AUTOWARE_STATE" -ge 2 ] 2>/dev/null; then
        STATE_NAME="${STATE_NAMES[$AUTOWARE_STATE]:-UNKNOWN}"
        echo -e "${GREEN}Autoware ready: $STATE_NAME${NC}"
        break
    fi

    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo -e "${YELLOW}Warning: Autoware did not reach WAITING_FOR_ROUTE in ${TIMEOUT}s${NC}"
        echo -e "${YELLOW}Localization may need manual initialization${NC}"
        break
    fi

    sleep 2
    ELAPSED=$((ELAPSED + 2))
    if [ $((ELAPSED % 10)) -eq 0 ]; then
        STATE_NAME="${STATE_NAMES[$AUTOWARE_STATE]:-INITIALIZING}"
        echo -e "${BLUE}  [$ELAPSED/${TIMEOUT}s] State: $STATE_NAME${NC}"
    fi
done

# Optional: Set initial pose if requested (usually not needed if AWSIM spawns correctly)
SET_INITIAL_POSE=${SET_INITIAL_POSE:-false}
INITIAL_POSE_CONFIG="$SCRIPT_DIR/experiments/configs/captured_initial_pose.json"

if [ "$SET_INITIAL_POSE" = "true" ] && [ -f "$INITIAL_POSE_CONFIG" ]; then
    echo -e "${BLUE}Setting initial pose from config (SET_INITIAL_POSE=true)...${NC}"
    python3 "$SCRIPT_DIR/experiments/scripts/set_initial_pose.py" --wait --timeout 30
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Initial pose set successfully${NC}"
    else
        echo -e "${YELLOW}Warning: Initial pose setting may have failed${NC}"
    fi
fi

echo ""
echo -e "${GREEN}Autoware is running in headless mode.${NC}"
echo -e "${BLUE}Monitor with: python3 experiments/scripts/diagnose_system.py --continuous${NC}"
echo ""

# Wait for Autoware process
wait $AUTOWARE_PID

echo -e "${GREEN}Autoware shutdown complete${NC}"
