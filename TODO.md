# Risk-Aware Control - Task List

**Last Updated:** 2026-02-05

---

## Development Philosophy

1. **Work in small chunks** - Pick specific tasks, not entire phases
2. **Validation-first** - Know what we're measuring before building
3. **Document decisions** - Rationale for committee defense
4. **Find failures first** - Can't fix what's not broken

---

## Phase 0: Environment Setup ✅ COMPLETE

- [x] AWSIM + Autoware verified working
- [x] Waypoint demo completed
- [x] ROS2 topics documented (788 topics)
- [x] Constraint parameters identified
- [x] Architecture understood

---

## Phase 1: Experiment Infrastructure ✅ COMPLETE

### 1.1 Automation ✅
- [x] AWSIM automation (`--config` JSON flag)
- [x] Experiment launcher scripts
- [x] Batch runner
- [x] Goal capture tool
- [x] Stuck detection watchdog
- [x] Session-based workflow
- [x] Vehicle reset script
- [x] MRM state tracking

### 1.2 Modular Experiment Framework ✅
- [x] `experiments/lib/` module structure
- [x] `config.py` - Configuration management
- [x] `ros_utils.py` - ROS2 utilities
- [x] `metrics.py` - Metrics computation
- [x] `run_experiments.py` - Main experiment runner
- [x] MRM diagnostic analysis script

### 1.3 Metrics Definition ✅
- [x] Safety metrics (collision proxy, min distance, TTC with lateral filtering)
- [x] Reliability metrics (goal success, driving time, velocity, path deviation)
- [x] Fail-operational metrics (MRM rate/km, E-stop ratio, recovery rate)
- [x] Comfort metrics (acceleration, jerk)
- [x] Normalized metrics (per km, event-based counting)

---

## Phase 1.5: Baseline Data Collection ✅ COMPLETE

### Results Summary
- **25 goals tested** at 100 km/h max velocity
- **100% success rate** (25/25) after goal_017 adjustment
- **Average MRM rate:** 199 MRM/km
- **Average E-stop ratio:** 91%
- **Path deviation:** 0.09-0.18m (good tracking)
- **Near-miss rate:** 0-2.4/km (properly filtered)

### Completed Tasks
- [x] Run baseline experiments on all goals
- [x] Fix goal_017 (shortened to avoid stuck point)
- [x] Fix TTC computation (lateral filtering, event-based counting)
- [x] Update max_vel to 100 km/h (27.78 m/s)
- [x] Add PathDev column to summary

---

## Phase 2: Fault Injection & Stress Testing 🔄 CURRENT

### 2.1 Fault Injection Infrastructure
| Status | Task |
|--------|------|
| ⏳ | Create `fault_injection` ROS2 package |
| ⏳ | Implement base fault injector class |
| ⏳ | Topic remapping launch configuration |
| ⏳ | Fault parameter configuration (YAML) |

### 2.2 Perception Fault Injectors (Primary)
| Status | Task | Target |
|--------|------|--------|
| ⏳ | Object detection dropout | Drop % of detected objects |
| ⏳ | Object detection delay | Buffer + delay messages |
| ⏳ | Object position noise | Gaussian noise on positions |
| ⏳ | False positive injection | Add phantom objects |

**Target Topics:**
- `/perception/object_recognition/objects`
- `/perception/object_recognition/tracking/objects`

### 2.3 Localization Fault Injectors
| Status | Task | Target |
|--------|------|--------|
| ⏳ | Position noise | Gaussian noise on pose |
| ⏳ | Position drift | Cumulative offset over time |
| ⏳ | Covariance spike | Increase uncertainty suddenly |
| ⏳ | Heading error | Offset yaw angle |

**Target Topics:**
- `/localization/kinematic_state`
- `/localization/pose_with_covariance`

### 2.4 Scenario-Based Challenges (NPC Behavior)
| Status | Task | Setup |
|--------|------|-------|
| ⏳ | Static obstacle in lane | Spawn stationary NPC |
| ⏳ | Lead vehicle sudden brake | NPC ahead brakes to zero |
| ⏳ | Cut-in scenario | NPC enters lane ahead |

### 2.5 Fault Sweep Experiments
| Status | Task | Goal |
|--------|------|------|
| ⏳ | Perception delay sweep (0-500ms) | Find MRM threshold |
| ⏳ | Dropout rate sweep (0-50%) | Find safety threshold |
| ⏳ | Position noise sweep (0-2m) | Find lane departure threshold |
| ⏳ | Document failure modes | Characterize system limits |

---

## Phase 3: Digital Twin Retraining

| Status | Task |
|--------|------|
| ⏳ | Collect training data (baseline + fault conditions) |
| ⏳ | Adapt ST-GAT for new state representation |
| ⏳ | Add MRM state to features |
| ⏳ | Train and validate model |
| ⏳ | Verify residual detection of faults |

---

## Phase 4: RISE Implementation

| Status | Task |
|--------|------|
| ⏳ | Implement CVaR computation from residuals |
| ⏳ | Design constraint tightening mapping (CVaR → velocity limit) |
| ⏳ | Implement preemptive tightening (trend detection) |
| ⏳ | Integration with Autoware (velocity_smoother limits) |
| ⏳ | Parameter tuning |

---

## Phase 5: Validation

| Status | Task |
|--------|------|
| ⏳ | Run baseline vs RISE under fault conditions |
| ⏳ | Statistical analysis (MRM reduction, safety maintenance) |
| ⏳ | Generate publication figures |
| ⏳ | Write results section |

---

## Documentation

| File | Purpose |
|------|---------|
| `docs/theoretical_framework.md` | Core RISE formulation |
| `docs/research_notes/metrics_framework.md` | Metrics definitions and rationale |
| `docs/research_notes/fault_injection_strategy.md` | Fault types and implementation plan |
| `docs/research_notes/experiment_findings_and_mrm_analysis.md` | Experiment results, MRM analysis |

---

## Key Configuration

**Autoware:**
- `max_vel`: 27.78 m/s (100 km/h)
- File: `autoware/.../common.param.yaml`

**AWSIM:**
- Spawn position: (81384.60, 49922.00, 41.28)
- File: `experiments/configs/baseline.json`

**Experiments:**
- 25 goals covering ~17 km total distance
- Recording: 19 topics (see `experiments/lib/config.py`)

---

## Repository Structure

```
Risk-Aware-Control/
├── autoware/                 # Pre-built Autoware (modified config)
├── awsim_labs_v1.6.1/        # AWSIM binary
├── Shinjuku-Map/             # HD map
├── Run_AWSIM.sh              # Launch AWSIM
├── Run_Autoware.sh           # Launch Autoware
│
├── docs/
│   ├── theoretical_framework.md
│   └── research_notes/
│       ├── metrics_framework.md
│       ├── fault_injection_strategy.md
│       └── experiment_findings_and_mrm_analysis.md
│
├── experiments/
│   ├── lib/                  # Modular experiment library
│   │   ├── config.py         # Configuration + recording topics
│   │   ├── ros_utils.py      # ROS2 utilities
│   │   └── metrics.py        # Metrics computation
│   ├── scripts/              # Experiment automation
│   ├── configs/              # AWSIM + goal configs
│   └── data/                 # Experiment output
│       └── summary.txt       # Latest metrics summary
│
└── ros2_ws/                  # Custom ROS2 packages (to be created)
    └── src/
        └── fault_injection/  # Fault injection package
```
