#!/bin/bash
# =============================================================================
# Fault Campaign Runner
# =============================================================================
# Thin wrapper around collect.sh that runs the TL/IMU fault campaigns across
# a goal set in one command, instead of calling collect.sh 8 times by hand.
# AWSIM (Terminal 1) and Autoware (Terminal 2) must already be running.
#
# Usage:
#   ./run_fault_campaigns.sh [--goals GOALS] [--goals-file FILE] [--trials N]
#                            [--campaigns "c1 c2 ..."] [--arm A|B] [--confirm-each]
#                            [--no-restart] [--dry-run]
#
# Runs unattended by default (no "Press Enter" prompt between campaigns) since
# the whole point of this script is chaining several campaigns in one command.
# Pass --confirm-each to restore the per-campaign confirmation pause.
#
# Restarts Autoware between every campaign by default (added 2026-07-26,
# reusing run_nominal_batches.sh's proven restart_autoware — same 3-attempt
# retry + ROS2 daemon reset). The 2026-07-25 run deliberately skipped this to
# test whether a more powerful machine avoids README item 5's
# behavior_path_planner state-exhaustion (~18-36 experiments) without it — it
# didn't: all 4 IMU campaigns + part of tl_fault_s2 completed (~45
# experiments) before every remaining TL campaign started failing almost
# immediately. Pass --no-restart to go back to that (not recommended for an
# unattended/overnight run — a stall with no restart can burn hours before
# anyone notices, whereas run_experiments.py's 2026-07-26 auto-resume means
# a restart-covered failure just re-runs cleanly next time anyway).
#
# Auto-resume (run_experiments.py, 2026-07-26): --trials means "N trials
# total per goal" — a campaign that already has some trials on disk (e.g.
# from an interrupted run) only runs the deficit, per goal. Safe to just
# re-run this exact command to pick up where an interrupted run left off;
# no need to track which campaigns/goals/trials are already done by hand.
#
# --arm A|B (default A): passed straight through to collect.sh, which
# appends _armB to the campaign/condition names and stamps an 'arm' field in
# metadata.json/fault_log.jsonl when B. collect.sh will refuse to run if
# this doesn't match switch_diagnostic_arm.sh's marker for the diagnostic-gate
# config actually running — switch that (and restart Autoware) FIRST:
#   experiments/scripts/switch_diagnostic_arm.sh B   # then restart Autoware
#   ./run_fault_campaigns.sh --arm B
#
# Examples:
#   # Smoke test: goal_007 only, 1 trial, all 8 fault campaigns, Arm A
#   ./run_fault_campaigns.sh --goals goal_007 --trials 1
#
#   # Finalized run: goals 7/12/26 (default), 3 trials each, all 8 campaigns
#   ./run_fault_campaigns.sh
#
#   # Resume an interrupted run — identical command, auto-resume skips what's done
#   ./run_fault_campaigns.sh
#
#   # Same, but Arm B (stock diagnostic gate already switched + restarted)
#   ./run_fault_campaigns.sh --arm B
#
#   # Just the TL campaigns, 2 trials
#   ./run_fault_campaigns.sh --trials 2 \
#       --campaigns "tl_fault_s2 tl_fault_s3 tl_fault_s4"
# =============================================================================

set -eo pipefail

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; BLUE='\033[0;34m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
AUTOWARE_DIR="$WORKSPACE_DIR/autoware"

# ── Defaults (finalized goal set — see docs/research_notes/periodic_fault_strategy.md) ──
GOALS="goal_007,goal_012,goal_026"
GOALS_FILE="$SCRIPT_DIR/experiments/configs/captured_goals.json"
TRIALS=3
CAMPAIGNS="imu_fault_s3 imu_fault_scale imu_fault_stuck imu_fault_s1 tl_fault_s2 tl_fault_s3 tl_fault_s4 tl_fault_ramp"
DRY_RUN=""
YES="--yes"
FAULT_MIN_RUNWAY=""   # empty = collect.sh/fault_injector.py default (150m, GT-gated)
ARM="A"
DO_RESTART=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --goals)              GOALS="$2";             shift 2 ;;
        --goals-file)         GOALS_FILE="$2";         shift 2 ;;
        --trials)             TRIALS="$2";             shift 2 ;;
        --campaigns)          CAMPAIGNS="$2";          shift 2 ;;
        --fault-min-runway-m) FAULT_MIN_RUNWAY="$2";   shift 2 ;;
        --arm)                ARM="$2";                shift 2 ;;
        --confirm-each)  YES="";          shift ;;
        --no-restart)    DO_RESTART=0;    shift ;;
        --dry-run)       DRY_RUN="--dry-run"; shift ;;
        *) echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

cd "$SCRIPT_DIR"

# ── Autoware restart (copied from experiments/scripts/run_nominal_batches.sh —
#    proven 3-attempt retry with a ROS2 daemon reset between attempts; keep the
#    two in sync if either changes, not a shared lib to avoid a two-repo-deep
#    import for one function) ──────────────────────────────────────────────────
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
    bash "$SCRIPT_DIR/Run_Autoware_Headless.sh" &
    echo "Autoware PID: $!"

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

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} Fault campaign run${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "  Goals:      ${BLUE}${GOALS}${NC}"
echo -e "  Goals file: ${BLUE}${GOALS_FILE}${NC}"
echo -e "  Trials:     ${BLUE}${TRIALS}${NC}"
echo -e "  Campaigns:  ${BLUE}${CAMPAIGNS}${NC}"
echo -e "  Arm:        ${BLUE}${ARM}${NC}"
echo -e "  Restart between campaigns: ${BLUE}$([[ $DO_RESTART -eq 1 ]] && echo yes || echo no)${NC}"
echo ""

runway_arg=()
[[ -n "$FAULT_MIN_RUNWAY" ]] && runway_arg=(--fault-min-runway-m "$FAULT_MIN_RUNWAY")

FAILED_CAMPAIGNS=()
for c in $CAMPAIGNS; do
    echo -e "${GREEN}── ${c} (Arm ${ARM}) ──${NC}"
    if [[ $DO_RESTART -eq 1 ]]; then
        if ! restart_autoware; then
            echo -e "${RED}Skipping ${c} — Autoware would not come up. Re-run this script later; ${NC}"
            echo -e "${RED}auto-resume will skip whatever's already done and pick up here.${NC}"
            FAILED_CAMPAIGNS+=("$c")
            continue
        fi
    fi
    if ! ./collect.sh "$c" --goals "$GOALS" --goals-file "$GOALS_FILE" --trials "$TRIALS" --arm "$ARM" $YES $DRY_RUN "${runway_arg[@]}"; then
        echo -e "${RED}FAILED: ${c} — continuing with remaining campaigns${NC}"
        FAILED_CAMPAIGNS+=("$c")
    fi
    echo ""
done

if [[ ${#FAILED_CAMPAIGNS[@]} -gt 0 ]]; then
    echo -e "${RED}Completed with failures: ${FAILED_CAMPAIGNS[*]}${NC}"
    echo -e "${RED}Just re-run this same command — auto-resume skips completed trials.${NC}"
    exit 1
fi
echo -e "${GREEN}All campaigns complete.${NC}"
