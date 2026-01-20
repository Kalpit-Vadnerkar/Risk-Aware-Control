#!/bin/bash

# AWSIM Labs Launch Script
# This script launches AWSIM Labs simulator with optimized settings

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting AWSIM Labs Simulator...${NC}"

# Directory where AWSIM Labs binary is located
AWSIM_DIR="/home/df/Desktop/Kalpit-2026/awsim_labs_v1.6.1"
AWSIM_BINARY="${AWSIM_DIR}/awsim_labs.x86_64"

# Check if binary exists
if [ ! -f "$AWSIM_BINARY" ]; then
    echo -e "${YELLOW}Error: AWSIM binary not found at ${AWSIM_BINARY}${NC}"
    echo "Please update AWSIM_DIR in this script to point to your AWSIM installation"
    exit 1
fi

# Navigate to AWSIM directory
cd "$AWSIM_DIR" || exit 1

# Source ROS2 setup (required for topics to publish)
source /opt/ros/humble/setup.bash
source /home/df/Desktop/Kalpit-2026/autoware/install/setup.bash

echo -e "${GREEN}Launching AWSIM Labs...${NC}"

# Option 1: Launch with visible window (default)
# Uses NVIDIA GPU offload for better performance
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    ./awsim_labs.x86_64 \
    -screen-quality Fastest \
    -screen-width 1280 \
    -screen-height 720

# Option 2: Headless mode (uncomment to use)
# This runs without a GUI window using Xvfb
# Note: The simulation will auto-start in headless mode
# xvfb-run -a -s "-screen 0 1920x1080x24" \
#     ./awsim_labs.x86_64 \
#     -batchmode \
#     -nographics

echo -e "${GREEN}AWSIM Labs launched successfully!${NC}"
