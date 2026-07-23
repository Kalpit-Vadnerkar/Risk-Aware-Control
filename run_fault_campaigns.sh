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
#                            [--campaigns "c1 c2 ..."] [--confirm-each] [--dry-run]
#
# Runs unattended by default (no "Press Enter" prompt between campaigns) since
# the whole point of this script is chaining several campaigns in one command.
# Pass --confirm-each to restore the per-campaign confirmation pause.
#
# Examples:
#   # Smoke test: goal_007 only, 1 trial, all 8 fault campaigns
#   ./run_fault_campaigns.sh --goals goal_007 --trials 1
#
#   # Finalized run: goals 7/12/26 (default), 3 trials each, all 8 campaigns
#   ./run_fault_campaigns.sh
#
#   # Just the TL campaigns, 2 trials
#   ./run_fault_campaigns.sh --trials 2 \
#       --campaigns "tl_fault_s1 tl_fault_s2 tl_fault_s3 tl_fault_s4"
# =============================================================================

set -eo pipefail

GREEN='\033[0;32m'; BLUE='\033[0;34m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Defaults (finalized goal set — see docs/research_notes/periodic_fault_strategy.md) ──
GOALS="goal_007,goal_012,goal_026"
GOALS_FILE="$SCRIPT_DIR/experiments/configs/captured_goals.json"
TRIALS=3
CAMPAIGNS="tl_fault_s1 tl_fault_s2 tl_fault_s3 tl_fault_s4 imu_fault_s1 imu_fault_s2 imu_fault_s3 imu_fault_s4"
DRY_RUN=""
YES="--yes"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --goals)         GOALS="$2";      shift 2 ;;
        --goals-file)    GOALS_FILE="$2"; shift 2 ;;
        --trials)        TRIALS="$2";     shift 2 ;;
        --campaigns)     CAMPAIGNS="$2";  shift 2 ;;
        --confirm-each)  YES="";          shift ;;
        --dry-run)       DRY_RUN="--dry-run"; shift ;;
        *) echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

cd "$SCRIPT_DIR"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} Fault campaign run${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "  Goals:      ${BLUE}${GOALS}${NC}"
echo -e "  Goals file: ${BLUE}${GOALS_FILE}${NC}"
echo -e "  Trials:     ${BLUE}${TRIALS}${NC}"
echo -e "  Campaigns:  ${BLUE}${CAMPAIGNS}${NC}"
echo ""

for c in $CAMPAIGNS; do
    echo -e "${GREEN}── ${c} ──${NC}"
    ./collect.sh "$c" --goals "$GOALS" --goals-file "$GOALS_FILE" --trials "$TRIALS" $YES $DRY_RUN
    echo ""
done

echo -e "${GREEN}All campaigns complete.${NC}"
