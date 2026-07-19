# Risk-Aware Control for Autonomous Vehicles

Extension of the Digital Twin framework from passive fault detection to active risk-aware
control using conformal prediction over ST-GAT residuals (RISE — Residual-Informed Safety
Envelopes).

**Principal Investigator:** Kalpit Vadnerkar  
**Institution:** Clemson University, Department of Electrical and Computer Engineering  
**Advisor:** Pierluigi Pisu, PhD

---

## Machine Specs (Workstation — P5000)

| Component | Spec |
|-----------|------|
| GPU | NVIDIA Quadro P5000 (16 GB, Pascal SM 6.1) |
| CPU | Intel Xeon W-2135 @ 3.70 GHz (12 cores) |
| RAM | 32 GB |
| OS | Ubuntu 22.04, kernel 6.8.0-124-generic |
| CUDA driver | 570.211.01 |
| ROS2 | Humble |

---

## Workspace Layout

This repo assumes Autoware and AWSIM are installed in the **parent directory**
(`/home/kvadner/Desktop/Kalpit/`), not inside this repo.

```
/home/kvadner/Desktop/Kalpit/              ← workspace root
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
./collect.sh nom_v7
./collect.sh obs_recovery
./collect.sh obs_noescape
```

See `TODO.md` for current task status and `docs/research_notes/advisor_meeting_jun2026.md`
for the current research direction.

---

## Autoware Modifications

The Autoware install at `/home/kvadner/Desktop/Kalpit/autoware/` has been modified. The files live in `src/` and are symlinked into `install/` — edit the `src/` path.

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

After approximately 18–36 experiments (one or two full campaigns), the behavior_path_planner accumulates internal state that prevents trajectory generation on subsequent route sets. **Restart Autoware between campaigns** to avoid this. The `run_remaining.sh` helper in `experiments/scripts/` handles this automatically.

---

## Data Collection Campaigns

Goals: **007, 011, 021** — verified feasible on this machine (live route test 2026-06-27).
All data collected fresh on the P5000 workstation.

| Campaign | Command | What it tests |
|----------|---------|---------------|
| `nom_v5` | `./collect.sh nom_v5` | Nominal driving at 5 m/s — v5 calibration set |
| `nom_v7` | `./collect.sh nom_v7` | Nominal driving at 7 m/s — v7 calibration set |
| `nom_v10` | `./collect.sh nom_v10` | Nominal driving at 10 m/s — v10 calibration set |
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
| `/planning/scenario_planning/trajectory` | Planned trajectory |
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
- **Reference codebase:** `../Graph-Scene-Representation-and-Prediction/` (READ-ONLY)

## Contact

- **Email:** kvadner@clemson.edu
- **Lab:** Center for Connected Multimodal Mobility (C2M2), Clemson University
