#!/bin/bash
# =============================================================================
# RISE Scenario Sweep Runner
# =============================================================================
# Sources ROS2 + Autoware and runs a scenario sweep.
# AWSIM and Autoware must already be running.
#
# Usage:
#   ./run_sweep.sh <scenario_yaml> [--goals GOALS] [--trials N] [--stuck-timeout S] [--dry-run]
#
# Examples:
#   ./run_sweep.sh static_obstacle.yaml
#   ./run_sweep.sh static_obstacle.yaml --goals "goal_007,goal_011,goal_021" --trials 2
#   ./run_sweep.sh static_obstacle.yaml --dry-run
#   ./run_sweep.sh cut_in.yaml --goals "goal_007,goal_011,goal_021" --trials 2
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOWARE_DIR="$SCRIPT_DIR/autoware"
SCENARIOS_DIR="$SCRIPT_DIR/experiments/configs/scenarios"
SWEEP_SCRIPT="$SCRIPT_DIR/experiments/scripts/run_scenario_sweep.py"

# ── Source environment ────────────────────────────────────────────────────────

source /opt/ros/humble/setup.bash
source "$AUTOWARE_DIR/install/setup.bash"

# ── Argument handling ─────────────────────────────────────────────────────────

if [ $# -eq 0 ]; then
    echo "Usage: ./run_sweep.sh <scenario_yaml> [options]"
    echo ""
    echo "Available scenarios:"
    ls "$SCENARIOS_DIR"/*.yaml 2>/dev/null | xargs -I{} basename {}
    exit 1
fi

SCENARIO_ARG="$1"
shift

# Resolve YAML path: accept bare name, name.yaml, or full path
if [ -f "$SCENARIO_ARG" ]; then
    SCENARIO_YAML="$SCENARIO_ARG"
elif [ -f "$SCENARIOS_DIR/$SCENARIO_ARG" ]; then
    SCENARIO_YAML="$SCENARIOS_DIR/$SCENARIO_ARG"
elif [ -f "$SCENARIOS_DIR/${SCENARIO_ARG}.yaml" ]; then
    SCENARIO_YAML="$SCENARIOS_DIR/${SCENARIO_ARG}.yaml"
else
    echo "ERROR: Cannot find scenario: $SCENARIO_ARG"
    echo "Looked in: $SCENARIOS_DIR"
    exit 1
fi

echo "Scenario:  $SCENARIO_YAML"
echo "Args:      $@"
echo ""

# ── Run sweep ─────────────────────────────────────────────────────────────────

python3 "$SWEEP_SCRIPT" "$SCENARIO_YAML" "$@"
