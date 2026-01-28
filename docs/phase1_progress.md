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

## 10. Scripts Summary (Current)

| Script | Purpose |
|--------|---------|
| `capture_goal.py` | Capture goal from RViz → `captured_route.json` |
| `run_experiment.sh` | Full automated experiment pipeline |
| `run_batch.sh` | Run N experiments sequentially |
| `monitor_state.py` | Live terminal state display |
| `diagnose_mrm.py` | MRM diagnostic investigation with reports |
| `experiment_watchdog.py` | Active stuck/completion detection |
| `wait_for_autoware.py` | Wait for Autoware readiness (replaces sleep) |
| `spawn_npcs.py` | Test NPC spawning via entity controller |

---

## 11. What Needs to Happen Next

1. **Test full pipeline** — Run AWSIM (auto-load) + Autoware (readiness) + watchdog
2. **Test NPC spawning** — Run `spawn_npcs.py` to check entity controller availability
3. **Run 5 baseline experiments** — Establish consistency at 40 km/h
4. **Build metrics extraction** — Parse rosbag data into structured metrics
5. **Then proceed to fault injection (Phase 1.3)**
