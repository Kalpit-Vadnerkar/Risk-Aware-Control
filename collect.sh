#!/bin/bash
# =============================================================================
# Data Collection Script
# =============================================================================
# Collects experiment data for each campaign.
# AWSIM (Terminal 1) and Autoware (Terminal 2) must already be running.
#
# Usage:
#   ./collect.sh <campaign> [--trials N] [--goals GOALS] [--dry-run]
#
# Campaigns:
#   nom_v5        Nominal driving, 5 m/s velocity cap
#   nom_v7        Nominal driving, 7 m/s velocity cap
#   nom_v10       Nominal driving, 10 m/s velocity cap
#   obs_stuck     30m obstacle, Autoware stops (default policy)
#   obs_recovery  30m obstacle, Autoware swerves (avoidance=auto)
#   obs_noescape  30m obstacle in single-lane segment, no path forward
#
# Examples:
#   ./collect.sh nom_v7
#   ./collect.sh obs_recovery --trials 3
#   ./collect.sh obs_stuck --goals "goal_007,goal_011" --dry-run
# =============================================================================

set -eo pipefail

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m';  RED='\033[0;31m'; NC='\033[0m'

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOWARE_DIR="$SCRIPT_DIR/autoware"
EXPERIMENTS_DIR="$SCRIPT_DIR/experiments"
RUNNER="$EXPERIMENTS_DIR/scripts/run_experiments.py"
SCENARIOS_DIR="$EXPERIMENTS_DIR/configs/scenarios"
AVOIDANCE_CFG="$AUTOWARE_DIR/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml"

# ── Defaults ──────────────────────────────────────────────────────────────────
TRIALS=6
GOALS="goal_007,goal_011,goal_021"
DRY_RUN=""

# ── Parse arguments ───────────────────────────────────────────────────────────
if [[ $# -eq 0 ]]; then
    echo -e "${RED}Usage: ./collect.sh <campaign> [--trials N] [--goals GOALS] [--dry-run]${NC}"
    echo ""
    echo "Campaigns: nom_v5  nom_v7  nom_v10  obs_stuck  obs_recovery  obs_noescape"
    exit 1
fi

CAMPAIGN="$1"; shift

while [[ $# -gt 0 ]]; do
    case "$1" in
        --trials)   TRIALS="$2";  shift 2 ;;
        --goals)    GOALS="$2";   shift 2 ;;
        --dry-run)  DRY_RUN="--dry-run"; shift ;;
        *) echo -e "${RED}Unknown argument: $1${NC}"; exit 1 ;;
    esac
done

# ── Source environment ────────────────────────────────────────────────────────
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Data Collection — campaign: ${CAMPAIGN}${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "${BLUE}Sourcing ROS2 Humble...${NC}"
source /opt/ros/humble/setup.bash

echo -e "${BLUE}Sourcing Autoware install...${NC}"
source "$AUTOWARE_DIR/install/setup.bash"

# ── Prerequisites check ───────────────────────────────────────────────────────
echo -e "${BLUE}Checking prerequisites...${NC}"
if ! ros2 topic list 2>/dev/null | grep -q "/sensing/lidar/top/pointcloud_raw"; then
    echo -e "${RED}ERROR: AWSIM not running. Start it first: ./Run_AWSIM.sh${NC}"; exit 1
fi
echo -e "  AWSIM:     ${GREEN}OK${NC}"

AUTOWARE_UP=false
for i in 1 2 3; do
    if ros2 topic list 2>/dev/null | grep -qE "/autoware/state|/localization/kinematic_state"; then
        AUTOWARE_UP=true; break
    fi
    sleep 2
done
if [ "$AUTOWARE_UP" = false ]; then
    echo -e "${YELLOW}WARNING: Autoware topics not detected (may still be initializing).${NC}"
    echo -e "${YELLOW}         Continuing — the runner will wait for Autoware readiness.${NC}"
else
    echo -e "  Autoware:  ${GREEN}OK${NC}"
fi
echo ""

# ── Helper: run the Python experiment runner ──────────────────────────────────
run() {
    local campaign="$1"; local condition="$2"
    shift 2  # remaining args passed through (scenario, velocity-limit, etc.)
    python3 "$RUNNER" \
        --campaign "$campaign" \
        --condition "$condition" \
        --goals "$GOALS" \
        --trials "$TRIALS" \
        --stuck-timeout 200 \
        $DRY_RUN \
        "$@"
}

# ── Helper: verify/edit avoidance policy ─────────────────────────────────────
avoidance_set() {
    local target="$1"  # "auto" or "manual"
    if [[ ! -f "$AVOIDANCE_CFG" ]]; then
        echo -e "${RED}ERROR: Avoidance config not found:${NC}"; echo "  $AVOIDANCE_CFG"; exit 1
    fi
    local current
    current=$(grep -m1 '^ *policy:' "$AVOIDANCE_CFG" | awk '{print $2}' | tr -d '"')
    if [[ "$current" == "$target" ]]; then
        echo -e "  Avoidance policy already \"${target}\" — no change needed."
        return
    fi
    if [[ -z "$DRY_RUN" ]]; then
        cp "$AVOIDANCE_CFG" "${AVOIDANCE_CFG}.backup"
        sed -i "s/^          policy: \"manual\"/          policy: \"${target}\"/" "$AVOIDANCE_CFG"
        echo -e "  Avoidance policy set to ${GREEN}\"${target}\"${NC} (backup: *.backup)"
    else
        echo -e "  ${YELLOW}[DRY-RUN] Would set avoidance policy to \"${target}\"${NC}"
    fi
}

avoidance_verify() {
    local expected="$1"
    local current
    current=$(grep -m1 '^ *policy:' "$AVOIDANCE_CFG" | awk '{print $2}' | tr -d '"')
    if [[ "$current" != "$expected" ]]; then
        echo -e "${RED}ERROR: Avoidance policy is \"${current}\", expected \"${expected}\".${NC}"
        echo -e "       Run:  sed -i 's/policy: \"${current}\"/policy: \"${expected}\"/' \\"
        echo -e "             $AVOIDANCE_CFG"
        exit 1
    fi
    echo -e "  Avoidance policy: ${GREEN}\"${current}\"${NC} — correct"
}

# ── Dispatch campaign ─────────────────────────────────────────────────────────
case "$CAMPAIGN" in

    nom_v5)
        echo -e "${BLUE}Nominal driving — 5 m/s velocity cap${NC}"
        run nom_v5 nom_v5 --velocity-limit 5.0
        ;;

    nom_v7)
        echo -e "${BLUE}Nominal driving — 7 m/s velocity cap${NC}"
        run nom_v7 nom_v7 --velocity-limit 7.0
        ;;

    nom_v10)
        echo -e "${BLUE}Nominal driving — 10 m/s velocity cap${NC}"
        run nom_v10 nom_v10 --velocity-limit 10.0
        ;;

    obs_stuck)
        echo -e "${BLUE}Static obstacle — Autoware stops (avoidance=manual)${NC}"
        avoidance_verify "manual"
        run obs_stuck obs_stuck \
            --scenario "$SCENARIOS_DIR/obs_stuck.yaml"
        ;;

    obs_recovery)
        echo -e "${BLUE}Static obstacle — Autoware recovers (avoidance=auto)${NC}"
        echo -e "${YELLOW}NOTE: This will temporarily set avoidance policy to \"auto\".${NC}"
        echo -e "${YELLOW}      It will be restored to \"manual\" when the run completes.${NC}"
        echo ""
        avoidance_set "auto"
        # Trap ensures we restore even on Ctrl+C or error
        trap 'echo -e "\n${YELLOW}Restoring avoidance policy to manual...${NC}"; avoidance_set "manual"' EXIT
        run obs_recovery obs_recovery \
            --scenario "$SCENARIOS_DIR/obs_recovery.yaml"
        ;;

    obs_noescape)
        echo -e "${BLUE}Static obstacle — single-lane segment (no escape path)${NC}"
        avoidance_verify "manual"
        run obs_noescape obs_noescape \
            --scenario "$SCENARIOS_DIR/obs_noescape.yaml"
        ;;

    *)
        echo -e "${RED}Unknown campaign: ${CAMPAIGN}${NC}"
        echo "Valid campaigns: nom_v5  nom_v7  nom_v10  obs_stuck  obs_recovery  obs_noescape"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}Done. Data in: experiments/data/${CAMPAIGN}/${NC}"
