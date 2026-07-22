# Risk-Aware Control for Autonomous Vehicles

Extension of the Digital Twin framework from passive fault detection to active risk-aware
control using conformal prediction over ST-GAT residuals (RISE — Residual-Informed Safety
Envelopes).

**Principal Investigator:** Kalpit Vadnerkar  
**Institution:** Clemson University, Department of Electrical and Computer Engineering  
**Advisor:** Pierluigi Pisu, PhD

---

## Machine Specs (Workstation — RTX A6000)

| Component | Spec |
|-----------|------|
| GPU | NVIDIA RTX A6000 (49 GB) |
| CPU | Intel Core i9-10920X @ 3.50 GHz (24 threads) |
| RAM | 62 GB |
| OS | Ubuntu 22.04.5 LTS, kernel 6.8.0-134-generic |
| CUDA driver | 610.43.02 |
| ROS2 | Humble |

Previous workstation (P5000: 16GB, Xeon W-2135, 32GB RAM) specs kept in git history.

---

## Workspace Layout

This repo assumes Autoware and AWSIM are installed in the **parent directory**
(`/home/kvadner/Desktop/Dissertation/`), not inside this repo.

```
/home/kvadner/Desktop/Dissertation/        ← workspace root
├── autoware/                              ← Autoware Universe (pre-built, NOT in git)
│   ├── install/                           ← sourced by all scripts
│   └── src/
├── awsim_labs_v1.6.1/                     ← AWSIM Labs binary (NOT in git)
│   └── awsim_labs.x86_64
├── Map/
│   └── nishishinjuku_autoware_map/        ← HD map (NOT in git)
│       ├── lanelet2_map.osm
│       └── pointcloud_map.pcd
├── Graph-Scene-Representation-and-Prediction/  ← READ-ONLY T-ITS 2025 reference
└── Risk-Aware-Control/                    ← this repo
    ├── Run_AWSIM.sh
    ├── Run_Autoware.sh
    ├── Run_Autoware_Headless.sh
    ├── collect.sh                         ← primary data collection script
    ├── cleanup.sh
    ├── TODO.md                            ← task list (start here)
    ├── experiments/
    │   ├── configs/                       ← goal positions, scenario YAMLs
    │   ├── lib/                           ← PerceptionInterceptor, config, scenarios
    │   ├── scripts/                       ← run_experiments.py, sweep runner, etc.
    │   └── data/                          ← collected rosbags (gitignored)
    ├── st_gat/                            ← ST-GAT model and inference pipeline
    │   ├── model/                         ← architecture, loss, dataset
    │   ├── pipeline/                      ← bag reader, sequence builder
    │   ├── checkpoints/                   ← trained model weights
    │   └── results/residuals/             ← per-run residual CSVs
    └── docs/
        └── research_notes/               ← decision documentation
```

---

## Quick Start

```bash
# Terminal 1: Launch AWSIM
cd /home/kvadner/Desktop/Kalpit/Risk-Aware-Control
./Run_AWSIM.sh

# Terminal 2: Launch Autoware (headless for experiments)
./Run_Autoware_Headless.sh

# Terminal 3: Collect data
./collect.sh nom_v11
./collect.sh obs_recovery
./collect.sh obs_noescape
```

See `TODO.md` for current task status and `docs/research_notes/advisor_meeting_jun2026.md`
for the current research direction.

---

## Autoware Modifications

The Autoware install at `/home/kvadner/Desktop/Dissertation/autoware/` has been modified. The files live in `src/` and are symlinked into `install/` — edit the `src/` path.

> **If Autoware is ever rebuilt or reinstalled, reapply all changes below before running experiments.**

### 1. Velocity limit

| File | Change |
|------|--------|
| `config/planning/scenario_planning/common/common.param.yaml` | `max_vel: 11.11` (was 4.17 m/s) |

### 2. PerceptionInterceptor wiring

| File | Change |
|------|--------|
| `launch/components/tier4_planning_component.launch.xml` line 13 | `planning_input_objects_topic_name` → `objects_filtered` |

**Why:** Planning must read from `/perception/object_recognition/objects_filtered` (the PerceptionInterceptor output), not the raw perception topic.

### 3. MRM deadlock fixes — diagnostic gate for autonomous mode

This is the largest change. The default Autoware diagnostic chain triggers MRM (Minimum Risk Maneuver) on many conditions that are normal during AWSIM experiments — routing resets between trials, TF briefly dropping during teleports, rosbag2_recorder momentarily registering twice, EKF twist updates lagging through the Python IMU relay. Each of these caused MRM_SUCCEEDED deadlocks that permanently wedged the vehicle.

**All paths below are relative to** `config/system/diagnostics/` (inside `autoware_launch`).

**Note:** Always edit `autoware-main.yaml`, not `autoware-awsim.yaml`. Defining the same path in both causes a `PathConflict` crash in the diagnostic graph aggregator.

#### `autoware-main.yaml` — simplified `/autoware/modes/autonomous`

```yaml
- path: /autoware/modes/autonomous
  type: and
  list:
    - { type: link, link: /autoware/map }
    - { type: link, link: /autoware/control/autonomous }
    - { type: link, link: /autoware/vehicle }
    - { type: link, link: /autoware/system/autonomous }
    - { type: link, link: /adapi/mrm_request/delegate }
    # planning/perception removed: routing state errors during experiment resets
    #   block autonomous mode and cause unrecoverable MRM deadlocks.
    # localization removed: TF/pose_twist/accuracy all fail transiently during
    #   vehicle teleports between trials. AWSIM provides ground-truth localization.
    # control/autonomous and system/autonomous defined below.
```

#### `control.yaml` — add `/autoware/control/autonomous`

Append this unit to the file:

```yaml
- path: /autoware/control/autonomous
  type: and
  list:
    - { type: link, link: /autoware/control/node_alive_monitoring/command_gate }
    - { type: link, link: /autoware/control/emergency_braking }
  # topic_rate_check/control_command and trajectory_follower excluded:
  # control_cmd is not yet flowing at engage time (0-speed), triggering
  # immediate MRM_SUCCEEDED before the vehicle can start moving.
  # lane_departure and control_state excluded: both STALE at engage time.
```

#### `system.yaml` — add `/autoware/system/autonomous`

Append this unit to the file:

```yaml
- path: /autoware/system/autonomous
  type: and
  list:
    - { type: link, link: /autoware/system/topic_rate_check/emergency_control_command }
    - { type: link, link: /autoware/system/emergency_stop_operation }
  # duplicated_node_checker excluded: rosbag2_recorder briefly registers twice
  # between experiment trials, firing ERROR → MRM deadlock mid-session.
```

### 4. Experiment runner — post-stuck recovery timing

**File:** `experiments/scripts/run_experiments.py`

After a stuck trial (vehicle stopped for 200s timeout), the behavior_path_planner needs time to fully unwind its internal state before it can plan a new route. The original 3-second sleep was insufficient — extended to 20 seconds, and the route re-set and trajectory timeouts were increased:

| Parameter | Before | After |
|-----------|--------|-------|
| Sleep after `clear_route()` on trajectory timeout | 3 s | 20 s |
| `set_goal` timeout on re-set attempt | 30 s | 60 s |
| Trajectory wait after route re-set | 60 s | 90 s |

### 5. Autoware restart cadence

After approximately 18–36 experiments (one or two full campaigns), the behavior_path_planner accumulates internal state that prevents trajectory generation on subsequent route sets. **Restart Autoware between campaigns** to avoid this. `experiments/scripts/run_nominal_batches.sh` handles this automatically (batches goals 3 at a time with an Autoware restart between each).

### 6. Trajectory topic renamed upstream (found 2026-07-21, RTX A6000 machine)

This Autoware Universe checkout no longer publishes the final planned trajectory on
`/planning/scenario_planning/trajectory` (0 publishers) — it now publishes on
`/planning/trajectory` instead. Symptom: `set_goal_and_engage()` sets the route
successfully (Autoware reaches `WAITING_FOR_ENGAGE`) but `_wait_for_trajectory()` times
out forever waiting on the old topic name, with no MRM ever firing (nothing in the
diagnostic chain watches for trajectory presence). Fixed in `experiments/lib/ros_utils.py`,
`config.py`, `metrics.py`, and `perception_interceptor.py` to use `/planning/trajectory`.
If Autoware is ever pulled to a newer commit again, re-check this topic name first if
trials start failing with `engage_failed` despite a route being SET.

### 7. EKF twist-queue overflow → intermittent MRM blips during nominal driving (found 2026-07-21)

A successful nominal run still logged 73 brief (~0.1s) MRM_OPERATING blips with harsh
braking artifacts (max_jerk ~1000, 53 hard-brake events), despite reaching the goal.
Rosbag `/diagnostics` analysis traced it to `ekf_localizer`: repeated "twist topic is
delay" / "mahalanobis distance of twist topic is large" warnings, matching a live
"[EKF] Twist queue size (3) is exceeding max_queue_size (2)" log at boot — the Python
IMU relay delivers twist updates burstier than EKF's default queue (size 2) tolerates.
This cascades into `control_validator` deviation errors, which push
`operation_mode_availability` down long enough (mrm_handler's default 0.5s timeout) to
fire a brief MRM.

**Fix:** `config/localization/ekf_localizer.param.yaml` — `twist_smoothing_steps` and
`max_twist_queue_size` bumped 2 → 4. `config/system/mrm_handler/mrm_handler.param.yaml`
— `timeout_operation_mode_availability` bumped 0.5s → 1.0s as a second layer of
tolerance for brief transients (same philosophy as the item 3/4 fixes above).

**Validated 2026-07-21** (Autoware restarted to pick up the new params — these are
load-time, not hot-reloadable): the targeted `ekf_localizer` twist warnings dropped from
261 to 5 in a fresh `nom_v11` trial. Confirmed via `ros2 param get` that both nodes
picked up the new values after restart.

**Residual findings, deferred (not blocking data collection):**
- MRM trigger count dropped 73 → 31 despite the second trial running at a *higher*
  speed (11.11 vs 7 m/s, which should make transients worse, not better) — real
  improvement, not fully eliminated. All 31 still self-recovered (100% recovery rate,
  ~0.096s avg duration).
- A different, persistent issue is now the dominant diagnostic error:
  `duplicated_node_checker` fired 1710 times on
  `/perception/traffic_light_recognition/traffic_light/traffic_light_camera_info_relay`
  — a node double-registering, continuously rather than as a brief transient. Already
  excluded from the autonomous-mode gate (see item 3), so it isn't itself causing MRM,
  but is a real hygiene issue worth investigating separately.
- The likely actual remaining MRM cause is `control_validator` (`acceleration_error`,
  `max_distance_deviation`) — plausibly just the natural consequence of more aggressive
  tracking deviation at higher speed, not a quick config fix. Would need controller/
  velocity-smoother tuning work to reduce further, treated as future work rather than
  a blocker — data collection works fine as-is (goal reached, 100% MRM recovery).

---

## Data Collection Campaigns

Goals: **007, 011, 021** — verified feasible on this machine (live route test 2026-06-27).

**Direction change (2026-07-21):** all experiments now run at max possible velocity
(map limit, 11.11 m/s) — no more per-speed calibration set. `nom_v5`/`nom_v7`/`nom_v10`
are kept as working `collect.sh` commands (harmless to run) but are no longer part of
the data collection plan; `NOMINAL_DATASETS` in `st_gat/pipeline/config.py` only
includes `baseline_all` + `nom_v11`.

| Campaign | Command | What it tests |
|----------|---------|---------------|
| `nom_v11` | `./collect.sh nom_v11` | Nominal driving at max velocity (11.11 m/s) — the only calibration set needed |
| `obs_stuck` | `./collect.sh obs_stuck` | Obstacle — Autoware stops (avoidance=manual) |
| `obs_recovery` | `./collect.sh obs_recovery` | Obstacle — Autoware swerves (avoidance=auto) |
| `obs_noescape` | `./collect.sh obs_noescape` | 30m obstacle in single-lane (LL 241) — unsolvable baseline |
| `obs_singlelane` | `./collect.sh obs_singlelane` | 30m obstacle, no adjacent lane — Signal 1 validation |
| `obs_tooclosetoreact` | `./collect.sh obs_tooclosetoreact` | 5–8m obstacle, multi-lane — Signal 2 (TTC) validation |

Default: 6 trials × 3 goals. Override with `--trials N --goals GOALS`.

---

## Recorded Topics

| Topic | Purpose |
|-------|---------|
| `/localization/kinematic_state` | Vehicle state estimation |
| `/awsim/ground_truth/localization/kinematic_state` | Ground truth |
| `/tf` | Vehicle pose in map frame |
| `/perception/object_recognition/objects` | Detected objects (raw) |
| `/perception/object_recognition/objects_filtered` | Objects after interceptor |
| `/perception/traffic_light_recognition/traffic_signals` | Traffic light states |
| `/planning/trajectory` | Planned trajectory |
| `/planning/mission_planning/route` | Route geometry |
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | Behavior path |
| `/control/command/control_cmd` | Control commands |
| `/autoware/state` | System state machine |
| `/system/fail_safe/mrm_state` | MRM (safety) state |
| `/diagnostics` | System health |

---

## Related Work

- **IEEE T-ITS 2025 Paper:** "Digital Twins as Predictive Models for Real-Time Probabilistic
  Risk Assessment of Autonomous Vehicles"
- **Reference codebase:** `../Graph-Scene-Representation-and-Prediction/` (READ-ONLY — not a runtime
  dependency; `Point`, `GraphBuilder`, and a trimmed `MapProcessor` are copied into
  `st_gat/pipeline/Data_Curator/` and `st_gat/pipeline/State_Estimator/` — same
  package/class names as the original, just not imported live from that repo)

## Contact

- **Email:** kvadner@clemson.edu
- **Lab:** Center for Connected Multimodal Mobility (C2M2), Clemson University
