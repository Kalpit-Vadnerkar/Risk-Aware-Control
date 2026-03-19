#!/bin/bash

# =============================================================================
# RISE Experiment Session Launcher
# =============================================================================
#
# Unified script that sources the Autoware environment, starts the perception
# interceptor, and runs experiments. AWSIM and Autoware must already be running.
#
# Launch order (3 terminals):
#   Terminal 1:  ./Run_AWSIM.sh
#   Terminal 2:  ./Run_Autoware_Headless.sh
#   Terminal 3:  ./run_experiment_session.sh [options]
#
# The interceptor bridges /perception/.../objects -> objects_filtered.
# Planning reads from objects_filtered (configured in launch XML).
#
# Usage:
#   # Baseline (passthrough, no faults)
#   ./run_experiment_session.sh --campaign nominal --goals "goal_007"
#
#   # Single scenario
#   ./run_experiment_session.sh --campaign static_obstacle \
#       --scenario static_obstacle.yaml --goals "goal_007"
#
#   # Parameter sweep
#   ./run_experiment_session.sh --sweep perception_delay_sweep.yaml \
#       --goals "goal_007,goal_011,goal_021" --trials 2
#
#   # Quick test run
#   ./run_experiment_session.sh --campaign test --goals "goal_007" --dry-run
#
# =============================================================================

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

# Directory setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOWARE_DIR="$SCRIPT_DIR/autoware"
EXPERIMENTS_DIR="$SCRIPT_DIR/experiments"

# ── Source ROS2 + Autoware ──────────────────────────────────────────────────

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   RISE Experiment Session${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "${BLUE}Sourcing ROS2 Humble...${NC}"
source /opt/ros/humble/setup.bash

echo -e "${BLUE}Sourcing Autoware workspace...${NC}"
source "$AUTOWARE_DIR/install/setup.bash"

# ── Verify prerequisites ───────────────────────────────────────────────────

echo -e "${BLUE}Checking prerequisites...${NC}"

# Check AWSIM is running (sensor topics present)
if ! ros2 topic list 2>/dev/null | grep -q "/sensing/lidar/top/pointcloud_raw"; then
    echo -e "${RED}ERROR: AWSIM does not appear to be running.${NC}"
    echo -e "${RED}Start AWSIM first: ./Run_AWSIM.sh${NC}"
    exit 1
fi
echo -e "  AWSIM: ${GREEN}OK${NC}"

# Check Autoware is running (autoware state topic present)
if ! ros2 topic list 2>/dev/null | grep -q "/autoware/state"; then
    echo -e "${RED}ERROR: Autoware does not appear to be running.${NC}"
    echo -e "${RED}Start Autoware first: ./Run_Autoware_Headless.sh${NC}"
    exit 1
fi
echo -e "  Autoware: ${GREEN}OK${NC}"

# Check perception output topic exists
if ! ros2 topic list 2>/dev/null | grep -q "/perception/object_recognition/objects$"; then
    echo -e "${YELLOW}WARNING: Perception objects topic not found yet.${NC}"
    echo -e "${YELLOW}Autoware may still be initializing. Continuing anyway...${NC}"
else
    echo -e "  Perception: ${GREEN}OK${NC}"
fi

echo ""

# ── Parse arguments ─────────────────────────────────────────────────────────

MODE="run"  # "run" or "sweep"
SWEEP_YAML=""
PASSTHROUGH_ARGS=()

for arg in "$@"; do
    case "$arg" in
        --sweep)
            MODE="sweep"
            ;;
        *)
            PASSTHROUGH_ARGS+=("$arg")
            ;;
    esac
done

# Handle --sweep mode: next positional arg after --sweep is the YAML file
if [ "$MODE" = "sweep" ]; then
    # Re-parse to extract sweep yaml
    SWEEP_ARGS=()
    FOUND_SWEEP=false
    for arg in "$@"; do
        if [ "$FOUND_SWEEP" = true ] && [ -z "$SWEEP_YAML" ]; then
            SWEEP_YAML="$arg"
            continue
        fi
        if [ "$arg" = "--sweep" ]; then
            FOUND_SWEEP=true
            continue
        fi
        SWEEP_ARGS+=("$arg")
    done

    if [ -z "$SWEEP_YAML" ]; then
        echo -e "${RED}ERROR: --sweep requires a scenario YAML file${NC}"
        echo "Usage: ./run_experiment_session.sh --sweep perception_delay_sweep.yaml [options]"
        exit 1
    fi

    echo -e "${GREEN}Mode: Parameter Sweep${NC}"
    echo -e "${BLUE}Scenario: ${SWEEP_YAML}${NC}"
    echo ""

    python3 "$EXPERIMENTS_DIR/scripts/run_scenario_sweep.py" \
        "$SWEEP_YAML" \
        "${SWEEP_ARGS[@]}"
else
    echo -e "${GREEN}Mode: Experiment Run${NC}"
    echo ""

    python3 "$EXPERIMENTS_DIR/scripts/run_experiments.py" \
        "${PASSTHROUGH_ARGS[@]}"
fi

echo ""
echo -e "${GREEN}Session complete.${NC}"
