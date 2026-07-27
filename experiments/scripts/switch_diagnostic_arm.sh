#!/bin/bash
# =============================================================================
# switch_diagnostic_arm.sh — toggle Autoware's MRM diagnostic gate between
# Arm A (safety features disabled, the science condition) and Arm B (stock/
# full gate, the ground-truth oracle for lead-time measurement).
#
# Background: README.md item 3 documents Arm A's diagnostic-gate changes
# (autoware-main.yaml / control.yaml / system.yaml under
# config/system/diagnostics/), made to stop routing resets, TF drops during
# teleports, and rosbag2_recorder double-registration from permanently
# wedging the vehicle via MRM deadlocks. Arm B restores Autoware's stock
# gate exactly (verified against this checkout's own git HEAD — see
# autoware-main.armB.yaml etc.), so Autoware's own MRM can serve as a
# ground-truth "did/when did the stack itself react" signal for a given
# fault, comparable against Arm A's fully-propagated fault trajectory.
#
# This script only copies config files — it does NOT restart Autoware.
# These are load-time params (diagnostic_graph_aggregator reads them at
# launch), so you MUST restart Autoware after switching for the change to
# take effect. Never launch Autoware from this tool — restart it yourself.
#
# Usage:
#   ./switch_diagnostic_arm.sh A     # science condition (default so far)
#   ./switch_diagnostic_arm.sh B     # stock gate, ground-truth oracle
#   ./switch_diagnostic_arm.sh       # print which arm is currently active
# =============================================================================

set -eo pipefail

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
WORKSPACE_DIR="$(dirname "$REPO_DIR")"
DIAG_DIR="$WORKSPACE_DIR/autoware/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics"
MARKER_FILE="$DIAG_DIR/.active_arm"
FILES=(autoware-main control system)

if [[ ! -d "$DIAG_DIR" ]]; then
    echo -e "${RED}ERROR: $DIAG_DIR not found — check WORKSPACE_DIR resolution.${NC}"
    exit 1
fi

current_arm() {
    if [[ -f "$MARKER_FILE" ]]; then
        cat "$MARKER_FILE"
    else
        echo "unknown (no marker — pre-dates this script; assume A, this repo's long-standing default)"
    fi
}

if [[ $# -eq 0 ]]; then
    echo -e "${GREEN}Currently active diagnostic arm: $(current_arm)${NC}"
    exit 0
fi

ARM="$1"
if [[ "$ARM" != "A" && "$ARM" != "B" ]]; then
    echo -e "${RED}Usage: $0 {A|B}${NC}"
    exit 1
fi

echo -e "${GREEN}Switching diagnostic gate to Arm ${ARM}...${NC}"
for f in "${FILES[@]}"; do
    src="$DIAG_DIR/${f}.arm${ARM}.yaml"
    dst="$DIAG_DIR/${f}.yaml"
    if [[ ! -f "$src" ]]; then
        echo -e "${RED}ERROR: $src missing${NC}"
        exit 1
    fi
    cp "$dst" "$dst.bak.$(date +%Y%m%d_%H%M%S)" 2>/dev/null || true
    cp "$src" "$dst"
    echo "  ${f}.yaml <- ${f}.arm${ARM}.yaml"
done
echo "$ARM" > "$MARKER_FILE"

echo -e "${YELLOW}Done. This is a load-time config — restart Autoware for it to take effect.${NC}"
echo -e "${YELLOW}Pass --arm ${ARM} to collect.sh for any campaign run under this configuration,${NC}"
echo -e "${YELLOW}so the data on disk stays correctly labeled.${NC}"
