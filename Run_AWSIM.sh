#!/bin/bash

# AWSIM Labs Launch Script
# Launches AWSIM with auto-map loading via --config
#
# Usage: ./Run_AWSIM.sh [config_path]
#   config_path: Optional path to AWSIM JSON config (default: experiments/configs/baseline.json)

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Directory setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AWSIM_DIR="$SCRIPT_DIR/awsim_labs_v1.6.1"
AWSIM_BINARY="${AWSIM_DIR}/awsim_labs.x86_64"
DEFAULT_CONFIG="$SCRIPT_DIR/experiments/configs/baseline.json"

# Config argument (use provided path or default)
CONFIG_PATH="${1:-$DEFAULT_CONFIG}"

# Check if binary exists
if [ ! -f "$AWSIM_BINARY" ]; then
    echo -e "${YELLOW}Error: AWSIM binary not found at ${AWSIM_BINARY}${NC}"
    exit 1
fi

# Check if config exists
if [ ! -f "$CONFIG_PATH" ]; then
    echo -e "${YELLOW}Error: Config not found at ${CONFIG_PATH}${NC}"
    exit 1
fi

# Navigate to AWSIM directory
cd "$AWSIM_DIR" || exit 1

# Source ROS2 setup (required for topics to publish)
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/autoware/install/setup.bash"

echo -e "${GREEN}Starting AWSIM Labs...${NC}"
echo "Config: $CONFIG_PATH"

# Launch with --config for automatic map loading (no manual GUI selection)
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    ./awsim_labs.x86_64 \
    --config "$CONFIG_PATH" \
    -screen-quality Fastest \
    -screen-width 600 \
    -screen-height 400

echo -e "${GREEN}AWSIM Labs exited.${NC}"
