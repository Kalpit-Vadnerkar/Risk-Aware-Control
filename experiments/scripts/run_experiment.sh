#!/bin/bash
#
# Run a single automated experiment
#
# Usage: ./run_experiment.sh <experiment_id> [duration_seconds] [stuck_timeout]
#
# Example: ./run_experiment.sh baseline_001 120 30
#

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
AWSIM_DIR="$PROJECT_DIR/awsim_labs_v1.6.1"
CONFIG_DIR="$SCRIPT_DIR/../configs"
DATA_DIR="$SCRIPT_DIR/../data"

# Source ROS2 and Autoware
source /opt/ros/humble/setup.bash
if [ -f "$PROJECT_DIR/autoware/install/setup.bash" ]; then
    source "$PROJECT_DIR/autoware/install/setup.bash"
fi

# Arguments
EXPERIMENT_ID=${1:-"experiment_$(date +%Y%m%d_%H%M%S)"}
DURATION=${2:-120}
STUCK_TIMEOUT=${3:-30}  # seconds without movement before declaring stuck

# Load route config (must run capture_goal.py first)
ROUTE_CONFIG="$CONFIG_DIR/captured_route.json"
if [ ! -f "$ROUTE_CONFIG" ]; then
    echo "ERROR: Route config not found at $ROUTE_CONFIG"
    echo "Run capture_goal.py first to capture goal coordinates"
    exit 1
fi

# Extract goal coordinates from JSON
GOAL_X=$(python3 -c "import json; d=json.load(open('$ROUTE_CONFIG')); print(d['goal_pose']['position']['x'])")
GOAL_Y=$(python3 -c "import json; d=json.load(open('$ROUTE_CONFIG')); print(d['goal_pose']['position']['y'])")
GOAL_Z=$(python3 -c "import json; d=json.load(open('$ROUTE_CONFIG')); print(d['goal_pose']['position']['z'])")
GOAL_ORI_Z=$(python3 -c "import json; d=json.load(open('$ROUTE_CONFIG')); print(d['goal_pose']['orientation']['z'])")
GOAL_ORI_W=$(python3 -c "import json; d=json.load(open('$ROUTE_CONFIG')); print(d['goal_pose']['orientation']['w'])")

echo "=========================================="
echo "Running Experiment: $EXPERIMENT_ID"
echo "Duration: ${DURATION}s"
echo "Goal: ($GOAL_X, $GOAL_Y)"
echo "=========================================="

# Create data directory for this experiment
EXPERIMENT_DATA_DIR="$DATA_DIR/$EXPERIMENT_ID"
mkdir -p "$EXPERIMENT_DATA_DIR"

# Save experiment metadata
cat > "$EXPERIMENT_DATA_DIR/metadata.json" << EOF
{
  "experiment_id": "$EXPERIMENT_ID",
  "timestamp": "$(date -Iseconds)",
  "duration_planned": $DURATION,
  "goal": {
    "x": $GOAL_X,
    "y": $GOAL_Y,
    "z": $GOAL_Z
  },
  "config": "baseline.json"
}
EOF

echo "[1/6] Starting AWSIM..."
cd "$AWSIM_DIR"
./awsim_labs.x86_64 --config "$CONFIG_DIR/baseline.json" &
AWSIM_PID=$!
echo "AWSIM PID: $AWSIM_PID"

# Wait for AWSIM to initialize
echo "Waiting 30s for AWSIM to load..."
sleep 30

echo "[2/6] Starting Autoware..."
cd "$PROJECT_DIR"
./Run_Autoware.sh &
AUTOWARE_PID=$!
echo "Autoware PID: $AUTOWARE_PID"

# Wait for Autoware to be ready (monitors /autoware/state)
echo "Waiting for Autoware to reach WAITING_FOR_ROUTE state..."
set +e
python3 "$SCRIPT_DIR/wait_for_autoware.py" 120
AUTOWARE_READY=$?
set -e
if [ $AUTOWARE_READY -ne 0 ]; then
    echo "ERROR: Autoware did not become ready within timeout"
    kill $AWSIM_PID 2>/dev/null || true
    kill -INT $AUTOWARE_PID 2>/dev/null || true
    exit 1
fi

echo "[3/6] Starting rosbag recording..."
ros2 bag record -o "$EXPERIMENT_DATA_DIR/rosbag" \
    /localization/kinematic_state \
    /localization/pose_with_covariance \
    /awsim/ground_truth/localization/kinematic_state \
    /perception/object_recognition/objects \
    /control/command/control_cmd \
    /vehicle/status/velocity_status \
    /vehicle/status/steering_status \
    /planning/scenario_planning/trajectory \
    /diagnostics \
    /autoware/state &
ROSBAG_PID=$!
echo "Rosbag PID: $ROSBAG_PID"

sleep 2

echo "[4/6] Setting goal pose..."
ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped "{
    header: {frame_id: 'map'},
    pose: {
        position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z},
        orientation: {x: 0.0, y: 0.0, z: $GOAL_ORI_Z, w: $GOAL_ORI_W}
    }
}" --once

sleep 2

echo "[5/6] Engaging autonomous mode..."
ros2 service call /api/operation_mode/change_to_autonomous \
    autoware_adapi_v1_msgs/srv/ChangeOperationMode "{}" || echo "Service call failed, trying alternative..."

# Alternative: Some versions use different topic
ros2 topic pub /autoware/engage autoware_vehicle_msgs/msg/Engage "{engage: true}" --once 2>/dev/null || true

echo "[6/6] Running experiment (max ${DURATION}s, stuck timeout ${STUCK_TIMEOUT}s)..."
set +e  # Don't exit on watchdog non-zero (stuck detection returns 1)
python3 "$SCRIPT_DIR/experiment_watchdog.py" "$DURATION" "$GOAL_X" "$GOAL_Y" "$STUCK_TIMEOUT"
WATCHDOG_EXIT=$?
set -e

echo "Stopping experiment..."

# Copy watchdog result to experiment directory
if [ -f "$DATA_DIR/watchdog_result.json" ]; then
    cp "$DATA_DIR/watchdog_result.json" "$EXPERIMENT_DATA_DIR/watchdog_result.json"
fi

# Stop rosbag
kill $ROSBAG_PID 2>/dev/null || true
sleep 2

# Stop Autoware (sends SIGINT for clean shutdown)
kill -INT $AUTOWARE_PID 2>/dev/null || true
sleep 5

# Stop AWSIM
kill $AWSIM_PID 2>/dev/null || true

# Determine outcome
if [ $WATCHDOG_EXIT -eq 0 ]; then
    OUTCOME="completed"
elif [ $WATCHDOG_EXIT -eq 1 ]; then
    OUTCOME="stuck"
else
    OUTCOME="error"
fi

# Record completion
cat > "$EXPERIMENT_DATA_DIR/result.json" << EOF
{
  "experiment_id": "$EXPERIMENT_ID",
  "outcome": "$OUTCOME",
  "watchdog_exit_code": $WATCHDOG_EXIT,
  "timestamp": "$(date -Iseconds)"
}
EOF

echo "=========================================="
echo "Experiment $EXPERIMENT_ID: $OUTCOME"
echo "Data saved to: $EXPERIMENT_DATA_DIR"
echo "=========================================="
