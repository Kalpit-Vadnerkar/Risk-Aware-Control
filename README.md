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

### 1. Velocity limit

| File | Parameter | Value | Reason |
|------|-----------|-------|--------|
| `src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/common.param.yaml` | `max_vel` | `11.11` m/s (40 km/h) | Enable higher-speed driving; default 4.17 m/s (15 km/h) is too slow |

```bash
# Apply: open file and change max_vel from 4.17 to 11.11
```

### 2. MRM wedge fix — remove planning/perception from autonomous mode gate

**File:** `src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/autoware-main.yaml`

**Why:** By default, any planning or routing state error (e.g. during route reset between experiments) blocks autonomous mode and triggers MRM. MRM then cannot self-recover because autonomous mode remains unavailable — the vehicle is permanently wedged. Removing planning and perception from the autonomous mode gate means only localization, control, vehicle, and system failures trigger MRM.

**Note:** The fix belongs in `autoware-main.yaml`, not `autoware-awsim.yaml`. Defining the same path in both causes a `PathConflict` crash in the diagnostic graph aggregator.

**How to apply:** In the `/autoware/modes/autonomous` unit, remove the two lines:
```yaml
      - { type: link, link: /autoware/planning }
      - { type: link, link: /autoware/perception }
```

### 3. PerceptionInterceptor wiring — planning reads from `objects_filtered`

**File:** `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component.launch.xml` line 13

**Why:** The PerceptionInterceptor node publishes injected/filtered objects to `/perception/object_recognition/objects_filtered`. Without this change, planning reads the raw perception topic and the interceptor has no effect on planner behavior.

**How to apply:**
```bash
# Change line 13:
# default="/perception/object_recognition/objects"
# to:
# default="/perception/object_recognition/objects_filtered"
```

If Autoware is ever rebuilt or reinstalled, reapply all three changes before running experiments.

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
