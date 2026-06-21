#!/bin/bash
# =============================================================================
# Data Cleanup Script
# =============================================================================
# Removes experiment data that is no longer needed.
# Run with --dry-run first to review what will be deleted.
#
# Usage:
#   ./cleanup_data.sh            # execute deletion
#   ./cleanup_data.sh --dry-run  # preview only
# =============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 "$SCRIPT_DIR/experiments/scripts/cleanup_data.py" "$@"
