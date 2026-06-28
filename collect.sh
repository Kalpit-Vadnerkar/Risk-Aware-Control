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
#   nom_v5              Nominal driving, 5 m/s velocity cap
#   nom_v7              Nominal driving, 7 m/s velocity cap
#   nom_v11             Nominal driving, 11.11 m/s (Autoware map speed limit)
#   obs_stuck           30m obstacle, Autoware stops (avoidance=manual)
#   obs_recovery        30m obstacle, Autoware swerves (avoidance=auto)
#   obs_noescape        30m obstacle in single-lane (LL 241), no path forward
#   obs_singlelane      30m obstacle, no adjacent lane — Signal 1 validation
#   obs_tooclosetoreact 6m obstacle, multi-lane — Signal 2 (TTC) validation
#   tl_fault_s1         TL confidence degraded to 0.5 (fog/mild — reactive gating)
#   tl_fault_s2         TL confidence degraded to 0.1 (heavy occlusion — reactive gating)
#   tl_fault_s3         TL classification → UNKNOWN (reactive gating)
#   tl_fault_s4         TL full blackout during detection windows (reactive gating)
#   imu_fault_s1        IMU bias accel=0.1 m/s² gyro=0.05 rad/s, 30s on / 30s off
#   imu_fault_s2        IMU bias accel=0.5 m/s² gyro=0.10 rad/s, 30s on / 30s off
#   imu_fault_s3        IMU bias accel=1.0 m/s² gyro=0.30 rad/s, 20s on / 10s off
#   imu_fault_s4        IMU bias accel=2.0 m/s² gyro=0.50 rad/s, sustained
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
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
AUTOWARE_DIR="$WORKSPACE_DIR/autoware"
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
    echo "Campaigns: nom_v5  nom_v7  nom_v11  obs_stuck  obs_recovery  obs_noescape  obs_singlelane  obs_tooclosetoreact"
    echo "           tl_fault_s1..s4  imu_fault_s1..s4"
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
echo -e "  Autoware:  (readiness checked by the experiment runner)"
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
        trap 'echo -e "\n${YELLOW}Restoring velocity limit to default (11.11 m/s)...${NC}"; ros2 param set /planning/scenario_planning/external_velocity_limit_selector max_vel 11.11 2>/dev/null || true' EXIT
        run nom_v5 nom_v5 --velocity-limit 5.0
        ;;

    nom_v7)
        echo -e "${BLUE}Nominal driving — 7 m/s velocity cap${NC}"
        trap 'echo -e "\n${YELLOW}Restoring velocity limit to default (11.11 m/s)...${NC}"; ros2 param set /planning/scenario_planning/external_velocity_limit_selector max_vel 11.11 2>/dev/null || true' EXIT
        run nom_v7 nom_v7 --velocity-limit 7.0
        ;;

    nom_v11)
        echo -e "${BLUE}Nominal driving — 11.11 m/s (Autoware map speed limit, no external cap)${NC}"
        run nom_v11 nom_v11
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

    obs_singlelane)
        echo -e "${BLUE}Static obstacle — single-lane section, Signal 1 validation${NC}"
        avoidance_verify "manual"
        run obs_singlelane obs_singlelane \
            --scenario "$SCENARIOS_DIR/obs_singlelane.yaml"
        ;;

    obs_tooclosetoreact)
        echo -e "${BLUE}Static obstacle — 6m ahead, Signal 2 (TTC) validation${NC}"
        echo -e "${YELLOW}NOTE: Temporarily sets avoidance policy to \"auto\".${NC}"
        avoidance_set "auto"
        trap 'echo -e "\n${YELLOW}Restoring avoidance policy to manual...${NC}"; avoidance_set "manual"' EXIT
        run obs_tooclosetoreact obs_tooclosetoreact \
            --scenario "$SCENARIOS_DIR/obs_tooclosetoreact.yaml"
        ;;

    # ── Traffic light fault campaigns ─────────────────────────────────────────
    # Fault delay = 30s so localization converges before the fault activates.
    # Faults only applied during active TL detection windows (reactive gating).
    # Severity levels: Deng et al. (2023), ISO 21448 SOTIF.

    tl_fault_s1)
        echo -e "${BLUE}TL camera fault — S1: confidence degraded to 0.5 (mild, e.g. fog)${NC}"
        run tl_fault_s1 tl_fault_s1 \
            --tl-fault tl_confidence \
            --tl-params '{"confidence_scale":0.5}' \
            --fault-delay 30
        ;;

    tl_fault_s2)
        echo -e "${BLUE}TL camera fault — S2: confidence degraded to 0.1 (heavy occlusion)${NC}"
        run tl_fault_s2 tl_fault_s2 \
            --tl-fault tl_confidence \
            --tl-params '{"confidence_scale":0.1}' \
            --fault-delay 30
        ;;

    tl_fault_s3)
        echo -e "${BLUE}TL camera fault — S3: all elements forced to UNKNOWN (classification failure)${NC}"
        run tl_fault_s3 tl_fault_s3 \
            --tl-fault tl_unknown \
            --fault-delay 30
        ;;

    tl_fault_s4)
        echo -e "${BLUE}TL camera fault — S4: full blackout during detection windows${NC}"
        run tl_fault_s4 tl_fault_s4 \
            --tl-fault tl_blackout \
            --fault-delay 30
        ;;

    # ── IMU bias fault campaigns ───────────────────────────────────────────────
    # Periodic bias: ON for on_seconds then OFF for off_seconds, cycling.
    # S4 is sustained (off_seconds=0). Bias accumulates in EKF → x_var/y_var rises.
    # Magnitudes: Woodman (2007) UCAM-CL-TR-696; Abdel-Hafez et al. (2021).
    # Fault delay = 30s gives EKF time to converge on nominal trajectory first.

    imu_fault_s1)
        echo -e "${BLUE}IMU bias fault — S1: accel=0.1 m/s², gyro=0.05 rad/s, 30s on / 30s off${NC}"
        run imu_fault_s1 imu_fault_s1 \
            --imu-fault imu_bias \
            --imu-params '{"accel_bias_ms2":0.1,"gyro_bias_rads":0.05,"on_seconds":30,"off_seconds":30}' \
            --fault-delay 30
        ;;

    imu_fault_s2)
        echo -e "${BLUE}IMU bias fault — S2: accel=0.5 m/s², gyro=0.10 rad/s, 30s on / 30s off${NC}"
        run imu_fault_s2 imu_fault_s2 \
            --imu-fault imu_bias \
            --imu-params '{"accel_bias_ms2":0.5,"gyro_bias_rads":0.10,"on_seconds":30,"off_seconds":30}' \
            --fault-delay 30
        ;;

    imu_fault_s3)
        echo -e "${BLUE}IMU bias fault — S3: accel=1.0 m/s², gyro=0.30 rad/s, 20s on / 10s off${NC}"
        run imu_fault_s3 imu_fault_s3 \
            --imu-fault imu_bias \
            --imu-params '{"accel_bias_ms2":1.0,"gyro_bias_rads":0.30,"on_seconds":20,"off_seconds":10}' \
            --fault-delay 30
        ;;

    imu_fault_s4)
        echo -e "${BLUE}IMU bias fault — S4: accel=2.0 m/s², gyro=0.50 rad/s, sustained${NC}"
        run imu_fault_s4 imu_fault_s4 \
            --imu-fault imu_bias \
            --imu-params '{"accel_bias_ms2":2.0,"gyro_bias_rads":0.50,"on_seconds":9999,"off_seconds":0}' \
            --fault-delay 30
        ;;

    *)
        echo -e "${RED}Unknown campaign: ${CAMPAIGN}${NC}"
        echo "Valid campaigns: nom_v5  nom_v7  nom_v11  obs_stuck  obs_recovery  obs_noescape  obs_singlelane  obs_tooclosetoreact"
        echo "                 tl_fault_s1..s4  imu_fault_s1..s4"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}Done. Data in: experiments/data/${CAMPAIGN}/${NC}"
