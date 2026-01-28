#!/bin/bash
#
# Run multiple experiments in sequence
#
# Usage: ./run_batch.sh <num_runs> [duration_per_run]
#
# Example: ./run_batch.sh 5 120
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

NUM_RUNS=${1:-5}
DURATION=${2:-120}
BATCH_ID="batch_$(date +%Y%m%d_%H%M%S)"

echo "=========================================="
echo "Starting batch: $BATCH_ID"
echo "Number of runs: $NUM_RUNS"
echo "Duration per run: ${DURATION}s"
echo "=========================================="

for i in $(seq 1 $NUM_RUNS); do
    EXPERIMENT_ID="${BATCH_ID}_run_$(printf '%03d' $i)"
    echo ""
    echo ">>> Starting run $i of $NUM_RUNS: $EXPERIMENT_ID"
    echo ""

    "$SCRIPT_DIR/run_experiment.sh" "$EXPERIMENT_ID" "$DURATION"

    echo ""
    echo ">>> Completed run $i of $NUM_RUNS"
    echo ">>> Waiting 10s before next run..."
    echo ""
    sleep 10
done

echo "=========================================="
echo "Batch $BATCH_ID complete!"
echo "Completed $NUM_RUNS experiments"
echo "=========================================="
