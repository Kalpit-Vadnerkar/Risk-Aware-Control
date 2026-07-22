#!/bin/bash
# Run the nom_v11 nominal campaign across all goals in captured_goals_original.json,
# 3 goals at a time, restarting Autoware between each batch (matches the pattern in
# run_all_tl.sh — fresh planner state per batch, avoids the behavior_path_planner
# state-exhaustion issue documented in README.md item 5).
#
# Usage: ./experiments/scripts/run_nominal_batches.sh [--trials N] [--batch-size N] [--start-batch N]
# --start-batch resumes from batch N (1-indexed), skipping already-completed batches.
#
# Expects AWSIM already running.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKSPACE_DIR="$(dirname "$REPO_DIR")"
AUTOWARE_DIR="$WORKSPACE_DIR/autoware"
GOALS_FILE="$REPO_DIR/experiments/configs/captured_goals_original.json"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'

TRIALS=6
BATCH_SIZE=3
START_BATCH=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --trials)      TRIALS="$2";      shift 2 ;;
        --batch-size)  BATCH_SIZE="$2";  shift 2 ;;
        --start-batch) START_BATCH="$2"; shift 2 ;;
        *) echo -e "${RED}Unknown arg: $1${NC}"; exit 1 ;;
    esac
done

# Single restart attempt. Returns 1 on timeout instead of exiting — caller retries.
_restart_autoware_once() {
    echo -e "${YELLOW}Stopping Autoware...${NC}"
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "autoware_launch" 2>/dev/null || true
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
    for i in $(seq 1 150); do
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

# Retries up to 3 times, resetting the ROS2 daemon between attempts (same fix
# cleanup.sh uses) — repeated Autoware-only restarts (AWSIM stays up the whole
# campaign) can leave stale DDS discovery/node registrations that a daemon
# reset clears. Does NOT use `set -e` propagation for this — a single restart
# hiccup must not kill the whole multi-hour run (this killed a real run
# 2026-07-21: an unguarded timeout took down 44 remaining experiments).
restart_autoware() {
    for attempt in 1 2 3; do
        if _restart_autoware_once; then
            return 0
        fi
        echo -e "${RED}Restart attempt $attempt failed.${NC}"
        if [[ $attempt -lt 3 ]]; then
            echo -e "${YELLOW}Resetting ROS2 daemon and retrying...${NC}"
            pkill -9 -f "ros2 launch\|component_container\|autoware" 2>/dev/null || true
            sleep 3
            ros2 daemon stop 2>/dev/null || true
            ros2 daemon start 2>/dev/null || true
            sleep 3
        fi
    done
    echo -e "${RED}Autoware restart failed after 3 attempts.${NC}"
    return 1
}

# Build batches of goal IDs from captured_goals_original.json
mapfile -t ALL_GOALS < <(python3 -c "
import json
d = json.load(open('$GOALS_FILE'))
for g in d['goals']:
    print(g['id'])
")

echo -e "${GREEN}===== Nominal (nom_v11) Batch Runner =====${NC}"
echo "Goals file: $GOALS_FILE"
echo "Total goals: ${#ALL_GOALS[@]}   Batch size: $BATCH_SIZE   Trials/goal: $TRIALS"
echo ""

source /opt/ros/humble/setup.bash
source "$AUTOWARE_DIR/install/setup.bash"

BATCH_NUM=0
for ((i=0; i<${#ALL_GOALS[@]}; i+=BATCH_SIZE)); do
    BATCH_NUM=$((BATCH_NUM + 1))
    BATCH=("${ALL_GOALS[@]:i:BATCH_SIZE}")
    GOALS_CSV=$(IFS=,; echo "${BATCH[*]}")

    if (( BATCH_NUM < START_BATCH )); then
        echo "Skipping batch $BATCH_NUM ($GOALS_CSV) — already done."
        continue
    fi

    echo ""
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}  Batch $BATCH_NUM: $GOALS_CSV${NC}"
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    if ! restart_autoware; then
        echo -e "${RED}Skipping batch $BATCH_NUM ($GOALS_CSV) — Autoware would not come up. Resume later with --start-batch $BATCH_NUM.${NC}"
        continue
    fi

    echo -e "${YELLOW}Running: nom_v11 -- $GOALS_CSV${NC}"
    cd "$REPO_DIR"
    bash collect.sh nom_v11 --trials "$TRIALS" --goals "$GOALS_CSV" --goals-file "$GOALS_FILE" || true

    echo -e "${GREEN}Batch $BATCH_NUM complete.${NC}"
    sleep 5
done

echo ""
echo -e "${GREEN}All nominal batches complete.${NC}"
