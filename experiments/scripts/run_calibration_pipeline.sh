#!/bin/bash
# Regenerates every Layer-1 (detect-the-unknown) result artifact from
# scratch: conformal calibration (leave-one-trial-out cross-conformal,
# coverage table + horizon-widening plot + reliability diagram) and the
# epistemic-disagreement check. Both scripts already scale automatically
# to however many trials exist in cfg.CAL_DIR (LOO-CV folds on
# len(pkl_files), not a hardcoded count) -- this script's only job is to
# chain them with one command and keep the "which checkpoints count as
# independent members" list in one obvious, editable place, so running it
# again after new nominal data or a new epistemic member is added
# regenerates everything without hunting through commands.
#
# Usage (repo venv sourced by this script itself, no ROS needed):
#   ./experiments/scripts/run_calibration_pipeline.sh
#   ./experiments/scripts/run_calibration_pipeline.sh --model path/to/other/checkpoint.pth
#
# Added 2026-08-21 per Kalpit's request to keep the pipeline robust as more
# data/checkpoints arrive -- see docs/research_notes/
# nll_calibration_arc_and_conformal_pivot_2026-08-20.md and
# open_world_safety_reframe_2026-08-20.md for what these results mean.

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source .venv/bin/activate

# The production point predictor conformal calibration is checked against.
# Override with --model if testing a different checkpoint.
POINT_PREDICTOR="st_gat/checkpoints/h30_30_pointpred_v1/mean_warmup.pth"

# Independently-trained point predictors for the epistemic-disagreement
# check -- edit this list as more members get trained (each needs its own
# --workers 0/2/4 --warmup-only run, see epistemic_disagreement_check.py's
# own docstring). Order doesn't matter; need >= 2.
EPISTEMIC_MEMBERS=(
  "st_gat/checkpoints/h30_30_epistemic_m1/mean_warmup.pth"
  "st_gat/checkpoints/h30_30_epistemic_m2/mean_warmup.pth"
  "st_gat/checkpoints/h30_30_pointpred_v1/mean_warmup.pth"
)

if [[ "${1:-}" == "--model" ]]; then
  POINT_PREDICTOR="$2"
fi

echo "=== Layer 1: conformal calibration (leave-one-trial-out) ==="
python3 experiments/scripts/conformal_horizon_calibration.py --model "$POINT_PREDICTOR"

echo
FOUND_MEMBERS=()
for f in "${EPISTEMIC_MEMBERS[@]}"; do
  if [[ -f "$f" ]]; then
    FOUND_MEMBERS+=("$f")
  else
    echo "WARNING: epistemic member checkpoint not found, skipping: $f" >&2
  fi
done
echo "=== Epistemic disagreement (${#FOUND_MEMBERS[@]}/${#EPISTEMIC_MEMBERS[@]} members found) ==="
if [[ ${#FOUND_MEMBERS[@]} -lt 2 ]]; then
  echo "SKIPPED: need >= 2 member checkpoints, found ${#FOUND_MEMBERS[@]}" >&2
else
  python3 experiments/scripts/epistemic_disagreement_check.py --models "${FOUND_MEMBERS[@]}"
fi

echo
echo "=== Done. Results in: ==="
echo "  experiments/analysis/conformal_horizon_calibration/"
echo "  experiments/analysis/epistemic_disagreement/"
