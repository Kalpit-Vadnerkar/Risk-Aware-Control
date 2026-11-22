#!/bin/bash
# =============================================================================
# Data collection. AWSIM (Terminal 1) and Autoware (Terminal 2) must be running.
#
#   ./collect.sh --status [CAMPAIGN ...]        progress, no ROS needed
#   ./collect.sh CAMPAIGN [CAMPAIGN ...] [opts]  run the next missing trials
#   ./collect.sh all                            every campaign, in file order
#
# Campaigns: experiments/configs/campaigns.yaml. Each run does at most
# --max-trials (default 12) trials, then asks you to restart Autoware and run
# the same command again; it always continues with whatever is still missing.
#
# Options (passed to experiments/scripts/run_experiments.py):
#   --max-trials N    trials per Autoware restart (default 12)
#   --goals g1,g2     restrict to these goals
#   --trials N        override the campaign's per-goal target
#   --arm A|B         diagnostic gate Autoware runs under (default A); must
#                     match experiments/scripts/switch_diagnostic_arm.sh
#   --on-invalid stop|continue   (default stop)
#   --dry-run, --yes
# =============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOWARE_DIR="$(dirname "$SCRIPT_DIR")/autoware"
RUNNER="$SCRIPT_DIR/experiments/scripts/run_experiments.py"
RED='\033[0;31m'; GREEN='\033[0;32m'; NC='\033[0m'

if [[ $# -eq 0 ]]; then
    sed -n '3,21p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
fi

# --status needs no ROS
for a in "$@"; do
    if [[ "$a" == "--status" ]]; then exec python3 "$RUNNER" "$@"; fi
done

source /opt/ros/humble/setup.bash
source "$AUTOWARE_DIR/install/setup.bash"

# Capture the list once: with pipefail, `ros2 topic list | grep -q` fails
# when grep exits early on a match and ros2 gets SIGPIPE.
TOPICS="$(ros2 topic list 2>/dev/null || true)"
if ! grep -q "/sensing/lidar/top/pointcloud_raw" <<< "$TOPICS"; then
    echo -e "${RED}AWSIM is not running. Start it first: ./Run_AWSIM.sh${NC}"; exit 1
fi
if ! grep -qx "/autoware/state" <<< "$TOPICS"; then
    echo -e "${RED}Autoware is not running. Start it first: ./Run_Autoware_Headless.sh${NC}"; exit 1
fi

# The --arm label must match the diagnostic config actually loaded
ARM="A"
args=("$@")
for i in "${!args[@]}"; do
    [[ "${args[$i]}" == "--arm" ]] && ARM="${args[$((i+1))]}"
done
MARKER="$AUTOWARE_DIR/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/.active_arm"
if [[ -f "$MARKER" && "$(cat "$MARKER")" != "$ARM" ]]; then
    echo -e "${RED}--arm $ARM requested, but the diagnostic config on disk is Arm $(cat "$MARKER").${NC}"
    echo -e "${RED}Run experiments/scripts/switch_diagnostic_arm.sh $ARM and restart Autoware first.${NC}"
    exit 1
fi

echo -e "${GREEN}AWSIM and Autoware up; diagnostic arm ${ARM}.${NC}"
exec python3 "$RUNNER" "$@"
