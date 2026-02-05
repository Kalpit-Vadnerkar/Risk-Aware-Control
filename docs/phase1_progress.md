# Phase 1: Experiment Infrastructure Progress

**Started:** 2026-01-23
**Last Updated:** 2026-01-28
**Status:** In progress

---

## 1. What Was Done

### AWSIM Automation Investigation

Explored the AWSIM Labs v1.6.1 source code and found:
- **`--config` flag** supported for JSON configuration (see `Loader.cs`)
- Config controls: map selection, traffic, ego spawn position/orientation, time scale
- **Entity controller** (`Ros2EntityController.cs`) can spawn/despawn/update NPC entities at runtime
- **OpenSCENARIO** support via `scenario_simulator_v2` (for future complex scenarios)

### Scripts Created

All scripts are in `experiments/scripts/`:

| Script | Purpose |
|--------|---------|
| `capture_goal.py` | ROS2 node that listens for goal set in RViz, saves coordinates to `captured_route.json` |
| `run_experiment.sh` | Full automated experiment: AWSIM → Autoware → rosbag → goal → engage → wait → stop |
| `run_batch.sh` | Runs N experiments sequentially with configurable duration |
| `monitor_state.py` | Live display of position, velocity, goal distance, obstacle proximity |

### Goal Coordinates Captured

Successfully ran a manual session and captured goal coordinates via `capture_goal.py`:

```json
{
  "goal_pose": {
    "position": {"x": 81443.05, "y": 49955.75, "z": 0.0},
    "orientation": {"z": -0.9553, "w": 0.2957}
  },
  "initial_pose": {
    "position": {"x": 81380.73, "y": 49918.78, "z": 42.35}
  }
}
```

Route: approximately 4 minutes in the Shinjuku map.

---

## 2. Bugs Found and Fixed

### Bug 1: QoS Incompatibility (FIXED)

**Symptom:** `monitor_state.py` showed position `(0.0, 0.0)` — no ground truth data received.

**Error message:**
```
New publisher discovered on topic '/awsim/ground_truth/localization/kinematic_state',
offering incompatible QoS. No messages will be received from it.
Last incompatible policy: RELIABILITY
```

**Cause:** AWSIM publishes ground truth with `BEST_EFFORT` reliability. Our scripts subscribed with the default `RELIABLE` QoS, which is incompatible.

**Fix:** Added explicit `BEST_EFFORT` QoS profile to both `capture_goal.py` and `monitor_state.py`:
```python
best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**Lesson:** Always check QoS policies when subscribing to AWSIM topics. AWSIM uses `BEST_EFFORT` for sensor/ground truth topics.

### Bug 2: rclpy Double Shutdown (FIXED)

**Symptom:** Error on Ctrl+C:
```
rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called
```

**Cause:** `KeyboardInterrupt` triggers ROS2's internal shutdown. The `finally` block then calls `rclpy.shutdown()` again.

**Fix:** Guard shutdown with `if rclpy.ok():` check:
```python
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
```

### Bug 3: Missing Autoware Source (FIXED)

**Symptom:** Scripts couldn't find Autoware message types (e.g., `autoware_vehicle_msgs`).

**Cause:** Scripts only sourced ROS2 Humble, not the Autoware workspace overlay.

**Fix:** Added to `run_experiment.sh`:
```bash
if [ -f "$PROJECT_DIR/autoware/install/setup.bash" ]; then
    source "$PROJECT_DIR/autoware/install/setup.bash"
fi
```

**Note:** When running Python scripts manually, you must source both:
```bash
source /opt/ros/humble/setup.bash
source /path/to/autoware/install/setup.bash
```

---

## 3. Known Issue: AV Gets Stuck Mid-Route

### Symptom

During the manual test run, the AV started driving the route but got stuck partway through. The route was approximately 4 minutes long but the vehicle stopped and did not resume.

### Relevant Log Messages

```
MRM State changed: NORMAL -> MRM_OPERATING
EMERGENCY_STOP is operated
MRM_OPERATING -> MRM_SUCCEEDED
```

Additional warnings observed:
- **NDT scan matcher:** `pose_buffer_.size() < 2` — localization may have temporarily lost track
- **Interpolator warnings:** Suggest trajectory length mismatch or out-of-range queries
- **Raw vehicle cmd converter:** `Exceeding the acc range. Desired acc: -3.400000 < min acc on map: -2.201000` — requested deceleration exceeds allowed range

### Analysis

The MRM (Minimum Risk Maneuver) system triggered an emergency stop. This is Autoware's built-in safety mechanism. Possible causes:

1. **Localization failure:** NDT scan matcher lost pose estimation (low LiDAR feature area in Shinjuku map)
2. **Trajectory issue:** Planner produced a trajectory requiring deceleration beyond allowed limits
3. **Perception timeout:** Object detection may have timed out, triggering safety fallback

### Next Steps

- [ ] Investigate which diagnostic triggered the MRM (check `/diagnostics` topic)
- [ ] Try a shorter/simpler route to see if the issue is route-specific
- [ ] Check if NDT localization is stable throughout the route
- [ ] Consider increasing `min_acc_on_map` threshold if the map constraint is too restrictive

---

## 4. Experiment Pipeline Architecture

```
Manual (one-time):
  capture_goal.py → captured_route.json

Automated (per experiment):
  run_experiment.sh:
    1. Start AWSIM (--config baseline.json)     [wait 30s]
    2. Start Autoware (Run_Autoware.sh)          [wait 60s]
    3. Start rosbag recording                    [key topics]
    4. Publish goal pose                         [from captured_route.json]
    5. Engage autonomous mode                    [service call]
    6. Wait for duration                         [configurable]
    7. Stop all processes                        [clean shutdown]

Batch:
  run_batch.sh → calls run_experiment.sh N times

Monitoring (optional):
  monitor_state.py → live terminal display
```

### Rosbag Topics Recorded

| Topic | Content |
|-------|---------|
| `/localization/kinematic_state` | Estimated pose + velocity |
| `/localization/pose_with_covariance` | Pose with uncertainty |
| `/awsim/ground_truth/localization/kinematic_state` | Ground truth |
| `/perception/object_recognition/objects` | Detected objects |
| `/control/command/control_cmd` | Control commands sent |
| `/vehicle/status/velocity_status` | Actual velocity |
| `/vehicle/status/steering_status` | Actual steering |
| `/planning/scenario_planning/trajectory` | Planned trajectory |
| `/diagnostics` | System health |
| `/autoware/state` | Autoware state machine |

---

## 5. MRM Investigation (2026-01-28)

### What is MRM?

MRM (Minimum Risk Maneuver) is Autoware's built-in fail-safe system. It monitors system health via a diagnostic graph and triggers emergency behavior when autonomous mode becomes unavailable.

**State machine:** `NORMAL -> MRM_OPERATING -> MRM_SUCCEEDED (or MRM_FAILED)`

**Trigger chain:**
```
Component diagnostics (localization, perception, planning, control, etc.)
  -> diagnostic_graph_aggregator evaluates pass/fail
  -> Publishes OperationModeAvailability (autonomous: true/false)
  -> MRM handler checks every 100ms
  -> If autonomous=false: trigger MRM within 0.5s
```

**Behavior priority:** pull_over (disabled) -> comfortable_stop (disabled) -> emergency_stop (always on, -2.5 m/s^2)

### Key Config Files

| File | Purpose |
|------|---------|
| `autoware/.../config/system/mrm_handler/mrm_handler.param.yaml` | MRM timeouts, behaviors |
| `autoware/.../config/system/mrm_emergency_stop_operator/mrm_emergency_stop_operator.param.yaml` | Deceleration rates |
| `autoware/.../config/system/diagnostics/*.yaml` | Diagnostic graph definitions |

### Key Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `timeout_operation_mode_availability` | 0.5s | Heartbeat timeout before MRM triggers |
| `use_comfortable_stop` | false | Use gentle stop instead of emergency |
| `use_pull_over` | false | Pull to side of road |
| `target_acceleration` (e-stop) | -2.5 m/s^2 | Emergency deceleration rate |

### Diagnostic Script Created

`experiments/scripts/diagnose_mrm.py` - monitors `/diagnostics`, `/system/fail_safe/mrm_state`, `/system/operation_mode/availability`. Logs all state changes with full context (failing diagnostics, mode availability, vehicle state). Saves text + JSON reports to `experiments/data/`.

**Bug fix:** ROS2 Python returns `status.level` as `bytes`, not `int`. Added `_parse_level()` conversion.

### Diagnostic Run Results (2026-01-28)

Ran a full experiment with diagnostics monitoring. **No MRM triggered.** Vehicle completed the route successfully.

Persistent diagnostics during normal operation (not MRM-triggering):
- `localization: ekf_localizer` - "[ERROR] cov_ellipse_long_axis is large" (~10k occurrences)
- `distortion_corrector_node` - timestamp mismatch on all 3 LiDARs
- `control_validator: control_validation_latency` - latency larger than expected
- `ndt_scan_matcher: scan_matching_status` - interpolation warnings

These are background noise in the AWSIM+Autoware setup. They indicate the simulation environment isn't perfectly synchronized, but don't trigger MRM because the diagnostic graph has tolerance for these.

### MRM Implications for RISE

**MRM is useful for our research:**
- It's the existing "last resort" safety system
- RISE sits *above* MRM: tighten constraints preemptively so MRM never needs to fire
- MRM triggers = our system failed to prevent the violation
- MRM trigger rate is a key metric: RISE should reduce it vs baseline

**MRM is NOT a roadblock:**
- Cannot be fully disabled (good -- we don't want to)
- Can be configured (timeouts, behaviors) if needed
- The previous stuck incident was likely a transient localization failure, not a systematic MRM issue

---

## 6. Velocity Cap Fix (2026-01-28)

**Problem:** Vehicle capped at 15 km/h (4.17 m/s).

**Root cause:** Hardcoded in Autoware planning config:
```
autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/common.param.yaml
```

**Change made:** `max_vel: 4.17` -> `max_vel: 11.11` (40 km/h)

Both source and installed copies updated. The same file also contains acceleration/jerk limits for normal driving and hard limits:
- Normal: +/- 1.0 m/s^2 accel, +/- 1.0 m/s^3 jerk
- Limit: -2.5 to +1.0 m/s^2 accel, +/- 1.5 m/s^3 jerk

These may need tuning if the higher speed causes issues.

**max_vel is a static upper bound, not dynamically reconfigurable.** However, Autoware has a two-tier velocity system:
- `max_vel` (static config) = absolute ceiling, loaded at startup
- `/planning/scenario_planning/max_velocity` topic (dynamic) = runtime override, can only LOWER below max_vel

Every planning module uses `std::min(external_limit, max_vel)`. This is exactly what RISE needs: publish a CVaR-tightened velocity limit to the topic, and `max_vel` ensures it never exceeds the configured ceiling.

---

## 7. Traffic Density (2026-01-28)

**Problem:** Very few NPCs in the environment.

**Finding:** The AWSIM binary has TWO config loaders:

1. **Loader scene** (used by `--config` flag): `AWSIMConfiguration.cs`
   - Only supports: `mapConfiguration`, `simulationConfiguration` (useTraffic: true/false), `egoConfiguration`
   - **No traffic density control**

2. **AutowareSimulation scene** (uses `--json_path` flag): `AutowareSimulation.cs`
   - Supports: `MaxVehicleCount`, `RandomTrafficSeed`, `TimeScale`, `Ego`
   - This is a different scene, not used by our binary launch path

**Current binary** uses the Loader scene, which only has `useTraffic: true/false`. Traffic density is baked into the Unity scene at build time (default `maxVehicleCount = 100`, `targetVehicleCount = 10`).

**Options to increase traffic:**
1. **UI slider** - AWSIM may have a runtime traffic density slider (`UITrafficVehicleDensity.cs` exists in source). Check the AWSIM window during runtime.
2. **Rebuild from source** - Modify `TrafficManager` defaults in Unity and rebuild. Requires Unity Editor.
3. **ROS2 entity controller** - `Ros2EntityController.cs` can spawn entities at runtime via ROS2 services. More complex but doesn't require rebuild.
4. **Accept defaults** - The current traffic level may be sufficient for initial validation.

**Entity controller investigation:** AWSIM has `Ros2EntityController.cs` exposing three topics:
- `awsim/entity_controller/spawn` (SpawnEntity: asset_key, unique_id, pose)
- `awsim/entity_controller/update_pose` (UpdatePoseEntity)
- `awsim/entity_controller/despawn` (DespawnEntity)

Created `spawn_npcs.py` to test this. It checks if the topics exist and if `entity_controller_msgs` is available in the ROS2 environment. If the binary doesn't expose these topics, fallback to UI slider or source rebuild.

**Decision:** Test `spawn_npcs.py` at runtime. Check UI slider as backup.

---

## 8. Stuck Detection Watchdog (2026-01-28)

Created `experiments/scripts/experiment_watchdog.py` to replace passive `sleep $DURATION` in `run_experiment.sh`.

**Monitors:**
- Hard timeout (max duration)
- Velocity-based stuck detection (no movement for N seconds)
- Position-based stuck detection (backup for velocity)
- Goal reached detection (distance < 5m and velocity < 0.5 m/s)

**Exit codes:**
- 0 = goal reached or timeout (normal)
- 1 = stuck detected

**Integration:** `run_experiment.sh` updated to use watchdog, saves `result.json` with outcome (`completed`, `stuck`, `error`). Third argument `stuck_timeout` (default 30s).

---

## 9. AWSIM Auto-Load and Autoware Readiness (2026-01-28)

### AWSIM Auto-Map Loading

Updated `Run_AWSIM.sh` to pass `--config experiments/configs/baseline.json`. This auto-loads the Shinjuku map without manual GUI selection. Accepts optional config path argument.

### Autoware Readiness Detection

Created `experiments/scripts/wait_for_autoware.py` to replace hardcoded `sleep 60`.

Monitors `/autoware/state` topic (`autoware_system_msgs/msg/AutowareState`):
- `INITIALIZING (1)` - still starting
- `WAITING_FOR_ROUTE (2)` - ready for goal setting
- `PLANNING (3)` - planning route
- `WAITING_FOR_ENGAGE (4)` - ready for autonomous engagement
- `DRIVING (5)` - driving
- `ARRIVED_GOAL (6)` - at destination

Script exits with code 0 when state >= `WAITING_FOR_ROUTE`. Times out after 120s (configurable).

`run_experiment.sh` updated to use this instead of `sleep 60`.

---

## 10. Goal Setting Fix — ADAPI Service (2026-01-28)

### Bug 4: Wrong Goal-Setting API (FIXED)

**Symptom:** Automated pipeline declared vehicle stuck after 30s. Logs showed:
- `planning/routing/state ERROR` — no route set
- `no mrm operation available: operate emergency_stop`
- Planning stack never received a valid route

**Root cause:** `run_experiment.sh` published goals to `/planning/mission_planning/goal` (a raw ROS2 topic). This is **NOT the correct API**. Autoware's route planner expects goals via the **ADAPI service** `/api/routing/set_route_points`.

When the user set goals manually via RViz, it worked because RViz internally calls the ADAPI service. The automated script was bypassing this entirely.

**Investigation details:**
- `mission_planner.cpp` exposes goal setting via services (`SetLaneletRoute`, `SetWaypointRoute`), not via topic subscription
- `routing.cpp` (ADAPI layer) proxies `/api/routing/set_route_points` to the mission planner
- Route state is published on `/api/routing/state` with states: UNKNOWN(0), UNSET(1), SET(2), ARRIVED(3), CHANGING(4)
- The service only works when route state is UNSET; it validates state before accepting

**Fix:** Created `experiments/scripts/set_goal.py` which:
1. Subscribes to `/api/routing/state` to monitor route state
2. Waits for route state to be UNSET (ready for new route)
3. Waits for `/api/routing/set_route_points` service to be available
4. Calls the service with proper `SetRoutePoints.Request` (header, goal Pose, empty waypoints, `allow_goal_modification=true`)
5. Verifies the service response indicates success
6. Confirms route state transitions from UNSET to SET

Exit codes: 0=success, 1=service error, 2=route not accepted, 3=system error

**Pipeline changes:**
- `run_experiment.sh` now has 7 steps (was 6):
  1. Start AWSIM
  2. Start Autoware
  3. Start rosbag recording
  4. **Stabilization wait (10s)** — allows planning/perception nodes to fully initialize even after WAITING_FOR_ROUTE
  5. **Set goal via ADAPI service** — calls `set_goal.py` with 30s timeout
  6. Engage autonomous mode
  7. Run watchdog

The stabilization wait addresses a secondary issue: the planning stack may still be initializing its internal nodes even after `/autoware/state` reports WAITING_FOR_ROUTE.

### Headless vs GUI

**Question:** Should Autoware run without the interactive GUI (RViz)?

**Answer:** Keep RViz during the debugging and validation phase. The core issue was never the GUI — it was the wrong goal-setting API. Running headless would remove our primary visual debugging tool without solving the actual problem.

Autoware can run headless by passing `rviz:=false` to the launch command. This is useful for batch experiments on a headless server, but during Phase 1 we need visual confirmation that routes are being planned and the vehicle is driving correctly.

---

## 11. Scripts Summary (Current)

| Script | Purpose |
|--------|---------|
| `capture_goal.py` | Capture goal from RViz → `captured_route.json` |
| `run_experiment.sh` | Full automated experiment pipeline (7 steps) |
| `run_batch.sh` | Run N experiments sequentially |
| `monitor_state.py` | Live terminal state display |
| `diagnose_mrm.py` | MRM diagnostic investigation with reports |
| `experiment_watchdog.py` | Active stuck/completion detection |
| `wait_for_autoware.py` | Wait for Autoware readiness (replaces sleep) |
| `wait_for_routing_ready.py` | Wait for routing API ready (mission_planner) |
| `wait_for_driving.py` | Wait for DRIVING state after engagement |
| `set_goal.py` | Set goal via ADAPI service with route verification |
| `spawn_npcs.py` | Test NPC spawning via entity controller |

---

## 12. Updated Pipeline Architecture

```
Manual (one-time):
  capture_goal.py → captured_route.json

Automated (per experiment):
  run_experiment.sh:
    1. Start AWSIM (--config baseline.json)         [wait 60s]
    2. Start Autoware (Run_Autoware.sh)              [wait for WAITING_FOR_ROUTE]
    3. Start rosbag recording                        [key topics]
    4. Stabilization wait                            [10s for planning stack]
    5. Set goal via ADAPI service (set_goal.py)      [verify route accepted]
    6. Engage autonomous mode                        [service call + wait for DRIVING]
    7. Watchdog monitoring                           [stuck detection, goal reached]

Cleanup:
  trap EXIT/INT/TERM → soft kill → hard kill → pkill orphans

Batch:
  run_batch.sh → calls run_experiment.sh N times

Monitoring (optional):
  monitor_state.py → live terminal display
```

---

## 13. Routing API Readiness Issue (2026-01-29)

### Bug 5: Routing API Not Ready Despite WAITING_FOR_ROUTE State

**Symptom:** `set_goal.py` times out waiting for `/api/routing/state` topic, even though Autoware reports `WAITING_FOR_ROUTE` state.

**Key log messages:**
```
[mission_planner]: waiting odometry... Route API is not ready.
[ndt_scan_matcher]: pose_buffer_.size() < 2
Timeout: Could not call set_route_points service within 30.0s
```

**Root cause:** The Autoware state machine (`/autoware/state`) transitions to `WAITING_FOR_ROUTE` based on high-level readiness criteria. However, the **mission_planner** (which provides the routing ADAPI service) has its own readiness check: it must receive odometry from `/localization/kinematic_state` before it will:
1. Publish to `/api/routing/state`
2. Provide the `/api/routing/set_route_points` service

The 10-second fixed stabilization wait was insufficient because localization initialization time varies. The NDT scan matcher was still accumulating poses even after the EKF activated.

**Why NDT keeps failing:**
The `pose_buffer_.size() < 2` warning indicates NDT hasn't accumulated enough matched scans. This happens during:
1. Initial localization startup (normal)
2. When LiDAR data quality is poor
3. When the TF tree between sensors and map isn't stable yet

**Fix:** Created `wait_for_routing_ready.py` which explicitly waits for `/api/routing/state` to start publishing (up to 180s). This replaces the fixed 10s sleep.

**Pipeline changes:**
- Step 4 now calls `wait_for_routing_ready.py` instead of `sleep 10`
- Timeout increased from 30s to 180s to account for slow localization
- `set_goal.py` timeout increased to 60s

### New Script: wait_for_routing_ready.py

Monitors `/api/routing/state` topic and exits when:
- State is `UNSET` (1) — ready for new route
- State is `SET` (2) — already has a route

Exit codes: 0=ready, 1=timeout, 2=error

### Debugging Localization Issues

If routing API doesn't become ready, check localization:

```bash
# Check if localization is publishing
ros2 topic hz /localization/kinematic_state

# Check pose estimator status
ros2 topic echo /localization/pose_estimator/pose --once

# Check NDT status
ros2 topic echo /localization/pose_estimator/ndt_aligned_pose --once

# Check EKF status
ros2 topic echo /localization/pose_twist_fusion_filter/biased_pose_with_covariance --once
```

If `/localization/kinematic_state` is not publishing (0 Hz), localization has failed. Check:
1. LiDAR data: `ros2 topic hz /sensing/lidar/top/pointcloud_raw`
2. Initial pose: Was pose_initializer called? (should be automatic via AWSIM)
3. TF tree: `ros2 run tf2_tools view_frames` to visualize

---

## 14. Updated Pipeline Architecture

```
Automated (per experiment):
  run_experiment.sh:
    1. Start AWSIM (--config baseline.json)         [wait 60s]
    2. Start Autoware (Run_Autoware.sh)              [wait for WAITING_FOR_ROUTE]
    3. Start rosbag recording                        [key topics]
    4. Wait for routing API ready                    [wait_for_routing_ready.py, 180s timeout]
    5. Set goal via ADAPI service (set_goal.py)      [verify route accepted]
    6. Engage autonomous mode                        [service call + wait for DRIVING]
    7. Watchdog monitoring                           [stuck detection, goal reached]
```

---

## 15. Session-Based Experiments (2026-01-29)

### Problem: Full Automation Unreliable

The fully automated `run_experiment.sh` has timing issues:
- AWSIM initialization time is variable
- Fixed 60s wait may not be enough
- Pose initializer may not receive valid GNSS data
- Results in EKF never activating

**Key observation:** Manual startup always works because you visually confirm AWSIM is ready before starting Autoware.

### Solution: Session-Based Workflow

Created `run_experiment_session.sh` for experiments within an existing AWSIM+Autoware session.

**Workflow:**
```
# Terminal 1: Start AWSIM (once per session)
cd Risk-Aware-Control
./Run_AWSIM.sh

# Terminal 2: Start Autoware (once per session)
# Wait for AWSIM to fully load first!
./Run_Autoware.sh

# Wait for localization (visual confirmation in RViz)
# Vehicle should be visible on the map

# Terminal 3: Run experiments (can run multiple)
cd experiments/scripts
./run_experiment_session.sh baseline_001 120 30
./run_experiment_session.sh baseline_002 120 30
# ... or use batch:
./run_batch_session.sh 5 baseline 120 30 15
```

**Benefits:**
1. Reliable initialization (human confirms readiness)
2. Multiple experiments per session (no AWSIM/Autoware restart)
3. Faster iteration during development
4. Still automated: goal setting, engagement, recording, watchdog

**Limitation: Vehicle position after each experiment**

After clearing a route, the vehicle stays at its current position. For consistent starting positions:
- Option A: Restart AWSIM between experiments (slow but consistent)
- Option B: Use a route that returns near the start position
- Option C: Accept variable starting positions (may be fine for fault testing)

### New Scripts

| Script | Purpose |
|--------|---------|
| `run_experiment_session.sh` | Single experiment in existing session |
| `run_batch_session.sh` | Multiple experiments in existing session |

### Session Script Features

- **Prerequisite checks:** Validates Autoware is running and state is correct
- **Route clearing:** After each experiment, clears route for next run
- **No AWSIM/Autoware management:** Assumes they're already running
- **Same data collection:** Rosbag, metadata, result.json

---

## 16. Analysis of Experiment 120 Rosbag

Rosbag from failed experiment 120 shows:

| Topic | Messages | Rate | Status |
|-------|----------|------|--------|
| `/localization/kinematic_state` | 1121 | ~27 Hz | ✅ Working |
| `/awsim/ground_truth/localization/kinematic_state` | 2532 | ~60 Hz | ✅ Working |
| `/autoware/state` | 421 | ~10 Hz | ✅ Reached state=2 |
| `/planning/scenario_planning/trajectory` | 0 | 0 Hz | ❌ Never planned |
| `/control/command/control_cmd` | 6 | ~0.1 Hz | ❌ Minimal |

**Conclusion:** Localization WAS working in experiment 120 (1121 odometry messages). The failure was in the routing/planning layer, not localization. The mission_planner's "waiting odometry" message may have been a timing issue or QoS mismatch, not a fundamental localization failure.

---

## 17. Headless Mode & Initial Pose Management (2026-01-29)

### Problem: Localization Fails After Engagement

Even with `check_localization.py` reporting "READY", the vehicle fails to move after goal is set:
- NDT scan matcher reports `pose_buffer_.size() < 2`
- `/autoware/localization/state STALE`
- MRM triggers emergency stops repeatedly

**Root cause:** The NDT scan matcher needs continuous EKF pose input. When the system is stressed (goal set, engagement), timing issues cause the pose buffer to empty.

### Solution: Headless Mode with Captured Initial Pose

New workflow that:
1. Captures initial pose from RViz (one-time setup)
2. Runs Autoware headless (without RViz overhead)
3. Programmatically sets initial pose before each experiment
4. Uses comprehensive diagnostics for debugging

### New Scripts Created

| Script | Purpose |
|--------|---------|
| `capture_initial_pose.py` | Capture pose from RViz's "2D Pose Estimate" → `captured_initial_pose.json` |
| `set_initial_pose.py` | Programmatically publish initial pose (replaces RViz) |
| `diagnose_system.py` | Comprehensive system diagnostics for headless debugging |
| `check_localization.py` | Quick localization health check |

### New Launch Script

`Run_Autoware_Headless.sh` - Launches Autoware without RViz:
- Checks for captured initial pose config
- Sets initial pose programmatically
- Waits for localization to stabilize
- Provides diagnostic instructions

### Updated Experiment Script

`run_experiment_session.sh` now supports:
- `--set-pose` flag to set initial pose before experiment
- Checks MRM state and autonomous mode availability
- Better error messages with debugging hints
- Color-coded output

### Workflow: One-Time Setup

```bash
# 1. Start AWSIM
./Run_AWSIM.sh

# 2. Start Autoware WITH RViz
./Run_Autoware.sh

# 3. Wait for RViz to show vehicle on map

# 4. Run capture script
cd experiments/scripts
python3 capture_initial_pose.py

# 5. In RViz, use "2D Pose Estimate" to set vehicle position
#    The script will save it to captured_initial_pose.json

# 6. Stop Autoware (Ctrl+C)
```

### Workflow: Headless Experiments

```bash
# Terminal 1: Start AWSIM
./Run_AWSIM.sh

# Terminal 2: Start Autoware HEADLESS
./Run_Autoware_Headless.sh
# This automatically sets initial pose and waits for localization

# Terminal 3: Monitor (optional)
cd experiments/scripts
python3 diagnose_system.py --continuous

# Terminal 4: Run experiments
cd experiments/scripts
./run_experiment_session.sh baseline_001 120 30

# OR with explicit pose setting:
./run_experiment_session.sh baseline_001 120 30 --set-pose
```

### Diagnostic Output Example

`diagnose_system.py` provides:

```
======================================================================
System Diagnostics - 15:30:45 (elapsed: 12s)
======================================================================

[AUTOWARE STATE]
  State: OK (WAITING_FOR_ROUTE)
  Route: OK (UNSET)
  Operation Mode: STOP (autonomous_available: true)

[SAFETY SYSTEM]
  MRM State: OK (NORMAL / NONE)

[LOCALIZATION]
  Odometry: 48 Hz (1234 total)
  Position: (81382.7, 49918.8, 41.3)
  Velocity: 0.00 m/s
  NDT Score: OK (3.45 (threshold: 2.3))

[SENSING]
  LiDAR: OK (123456 points, 234 frames)
  AWSIM Ground Truth: OK (81382.7, 49918.8)

[RECOMMENDATIONS]
  System appears healthy. Ready to run experiments.
======================================================================
```

---

## 18. What Needs to Happen Next

1. **One-time setup:**
   - Capture initial pose using `capture_initial_pose.py`
   - Verify the pose is correct by running a test experiment with RViz

2. **Test headless workflow:**
   ```bash
   ./Run_AWSIM.sh                    # Terminal 1
   ./Run_Autoware_Headless.sh        # Terminal 2
   python3 diagnose_system.py -c     # Terminal 3 (monitor)
   ./run_experiment_session.sh test1 120  # Terminal 4
   ```

3. **If headless works:** Run 5 baseline experiments
4. **Build metrics extraction** — Parse rosbag data
5. **Then proceed to fault injection (Phase 1.3)**
