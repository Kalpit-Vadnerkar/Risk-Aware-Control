# Scenario Test Framework for RISE Validation

**Date:** 2026-02-16
**Status:** Implemented, pending verification runs

## Decision

Implemented a PerceptionInterceptor approach rather than a full ROS2 fault injection package. The interceptor is a single Python ROS2 node that sits between Autoware's perception output and planning input, which avoids any Autoware build changes beyond a one-line launch file default.

### Alternatives Considered

1. **Full ROS2 package in ros2_ws/** — Would require colcon build, CMakeLists, package.xml. Overkill for what is essentially message forwarding with optional modification.
2. **Topic remapping at launch time** — Would require modifying launch files per-experiment. Fragile and hard to switch between scenarios dynamically.
3. **AWSIM NPC scripting** — Only covers scenario-based tests (static obstacle, cut-in), not perception faults (delay, dropout, noise). Also couples test logic to the simulator.

### Rationale

The interceptor approach gives us all fault types through one mechanism, requires zero Autoware rebuilds, and composes naturally: baseline runs use passthrough mode, so all experiments go through the same pipeline ensuring fair comparison.

## Architecture

```
AWSIM (sensors) → Autoware Perception → /perception/.../objects (real)
                                              ↓
                                    PerceptionInterceptor
                                    (inject objects / apply faults)
                                              ↓
                              /perception/.../objects_filtered → Planning
```

### Launch File Change

`tier4_planning_component.launch.xml` line 11 changed:
```xml
<!-- Before -->
<arg name="planning_input_objects_topic_name" default="/perception/object_recognition/objects"/>

<!-- After (RISE) -->
<arg name="planning_input_objects_topic_name" default="/perception/object_recognition/objects_filtered"/>
```

This single parameter propagates to behavior_path_planner, motion_velocity_planner, and all downstream planning nodes.

**Implication:** The interceptor MUST be running for planning to receive any objects. In passthrough mode it just republishes without modification.

### Launch Order

```
Terminal 1: ./Run_AWSIM.sh                    # Simulator (sensor data)
Terminal 2: ./Run_Autoware_Headless.sh         # Autoware stack (perception, planning, control)
Terminal 3: ./run_experiment_session.sh ...     # Interceptor + experiment runner
```

Autoware must be running first because the interceptor subscribes to Autoware's `/perception/.../objects` output and `/localization/kinematic_state` for ego pose.

## Implemented Strategies

| Strategy | Description | Key Parameters |
|----------|-------------|----------------|
| `passthrough` | No modification (baseline) | — |
| `static_obstacle` | Stationary object ahead of ego | `distance`, `lateral_offset`, `obj_type` |
| `cut_in` | Object cuts from adjacent lane into ego lane | `distance`, `trigger_time`, `cut_in_duration`, `lateral_start/end` |
| `perception_delay` | Buffers messages, releases after delay | `delay_ms` |
| `perception_dropout` | Randomly removes objects per-message | `dropout_rate` (0.0–1.0) |
| `position_noise` | Gaussian noise on object x/y positions | `noise_std` (meters) |

## Files Created/Modified

### New Files
- `experiments/lib/scenarios.py` — ScenarioConfig, ObjectFactory, ObjectPlacer
- `experiments/lib/perception_interceptor.py` — ROS2 PerceptionInterceptor node
- `experiments/scripts/run_scenario_sweep.py` — Parameter sweep runner
- `experiments/configs/scenarios/*.yaml` — 5 scenario YAML definitions
- `run_experiment_session.sh` — Unified launch script

### Modified Files
- `autoware/.../tier4_planning_component.launch.xml` — Topic redirect (line 11)
- `experiments/lib/config.py` — Added scenario_type, scenario_params to ExperimentConfig
- `experiments/scripts/run_experiments.py` — Added --scenario flag, interceptor lifecycle

## Technical Details

### PredictedObject Message Construction
- Frame: `map` (MGRS coordinates)
- Each injected object gets a persistent UUID so Autoware's tracker treats it as the same object across frames
- Required fields: object_id, classification, shape (BOUNDING_BOX), kinematics (pose + twist), predicted_paths
- Existence probability: 1.0

### Object Placement
Position computed relative to ego using heading projection:
```
obs_x = ego_x + distance * cos(heading) - lateral_offset * sin(heading)
obs_y = ego_y + distance * sin(heading) + lateral_offset * cos(heading)
```

### Key Design Choices
- **deepcopy before modify:** ROS2 messages are mutable; modifying in-place corrupts the original for other subscribers
- **Persistent object IDs:** Autoware's multi-object tracker associates detections across frames by UUID. Random IDs per frame would create tracking instability.
- **Subprocess isolation:** Interceptor runs as a separate Python process, controlled via SIGINT. This avoids ROS2 context conflicts with the experiment runner's own rclpy instance.

## Data Organization

Data is organized by **campaign** — each campaign is a named subdirectory under
`experiments/data/`. Campaign name is set via `--campaign` flag (defaults to
scenario name for sweeps, `default` for run_experiments.py).

```
experiments/data/
├── nominal/                          # Baseline runs (25 goals, 2026-02-05)
│   ├── goal_001_baseline_t1_20260205_.../
│   │   ├── metadata.json             # Experiment config snapshot
│   │   ├── result.json               # Outcome (status, driving_time, mrm_count)
│   │   ├── metrics.json              # Computed metrics (safety, reliability, etc.)
│   │   └── rosbag/                   # Raw ROS2 bag recording
│   ├── goal_007_baseline_t1_.../
│   ├── ...
│   ├── batch_20260205_*.json         # Batch run summaries
│   └── summary.txt                   # Metrics summary table
├── perception_delay_sweep/           # Delay sweep campaign (future)
│   ├── goal_007_perception_delay_delay_ms50_t1_.../
│   └── ...
├── static_obstacle/                  # Obstacle injection campaign (future)
├── test_runs/                        # Ad-hoc test runs (not for analysis)
└── ...
```

Use `compute_metrics.py experiments/data/<campaign>` to generate per-campaign summaries.

## Planned Sweep Campaigns

| Scenario | Configurations | Goals | Trials | Total |
|----------|---------------|-------|--------|-------|
| Perception delay | 11 (0–1000ms) | 3 | 2 | 66 |
| Perception dropout | 6 (0–100%) | 3 | 2 | 36 |
| Position noise | 9 (0–4m σ) | 3 | 2 | 54 |
| Static obstacle | 6 (3 dist × 2 type) | 3 | 3 | 54 |
| Cut-in | 4 (2 dist × 2 speed) | 2 | 3 | 24 |
| **Total** | | | | **~234** |

Sweep goals: goal_007 (915m, 0.84 eff), goal_011 (780m, 0.87 eff), goal_021 (849m, 0.93 eff).
