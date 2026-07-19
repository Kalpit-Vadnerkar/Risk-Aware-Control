#!/bin/bash
# Run all four TL fault campaigns sequentially, restarting Autoware between each.
# Usage: ./experiments/scripts/run_all_tl.sh [--trials N] [--goals GOALS]
#
# Expects AWSIM already running.
# Restarts Autoware before each campaign to avoid planner state exhaustion.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKSPACE_DIR="$(dirname "$REPO_DIR")"
AUTOWARE_DIR="$WORKSPACE_DIR/autoware"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'

TRIALS=6
GOALS="goal_007,goal_011,goal_021"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --trials) TRIALS="$2"; shift 2 ;;
        --goals)  GOALS="$2";  shift 2 ;;
        *) echo -e "${RED}Unknown arg: $1${NC}"; exit 1 ;;
    esac
done

CAMPAIGNS=(tl_fault_s1 tl_fault_s2 tl_fault_s3 tl_fault_s4)

restart_autoware() {
    echo -e "${YELLOW}Stopping Autoware...${NC}"
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "autoware_launch"  2>/dev/null || true
    sleep 5
    pkill -9 -f "relay_node\|fault_injector\|PerceptionInterceptor\|rosbag2" 2>/dev/null || true
    sleep 2

    echo -e "${YELLOW}Starting Autoware headless...${NC}"
    source /opt/ros/humble/setup.bash
    source "$AUTOWARE_DIR/install/setup.bash"
    bash "$REPO_DIR/Run_Autoware_Headless.sh" &
    AUTOWARE_PID=$!
    echo "Autoware PID: $AUTOWARE_PID"

    echo -n "Waiting for Autoware to be ready..."
    for i in $(seq 1 90); do
        state=$(timeout 2 ros2 topic echo /autoware/state --once 2>/dev/null | grep "state:" | awk '{print $2}' || echo "0")
        if [[ "$state" =~ ^[2-9]$ ]]; then
            echo -e " ${GREEN}ready (state=$state)${NC}"
            return 0
        fi
        echo -n "."
        sleep 3
    done
    echo -e " ${RED}TIMEOUT${NC}"
    return 1
}

echo -e "${GREEN}===== TL Fault Campaign Runner =====${NC}"
echo "Campaigns: ${CAMPAIGNS[*]}"
echo "Trials: $TRIALS  Goals: $GOALS"
echo ""

source /opt/ros/humble/setup.bash
source "$AUTOWARE_DIR/install/setup.bash"

for CAMPAIGN in "${CAMPAIGNS[@]}"; do
    echo ""
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}  Campaign: $CAMPAIGN${NC}"
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    restart_autoware

    echo -e "${YELLOW}Running: $CAMPAIGN${NC}"
    cd "$REPO_DIR"
    bash collect.sh "$CAMPAIGN" --trials "$TRIALS" --goals "$GOALS" || true

    echo -e "${GREEN}$CAMPAIGN complete.${NC}"
    sleep 5
done

echo ""
echo -e "${GREEN}All TL fault campaigns complete.${NC}"
