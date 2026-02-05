# Risk-Aware Control - Task List

**Last Updated:** 2026-02-04

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

## Phase 1: Experiment Infrastructure ✅ MOSTLY COMPLETE

### 1.1 Automation ✅
| Status | Task |
|--------|------|
| ✅ | AWSIM automation (`--config` JSON flag) |
| ✅ | Experiment launcher scripts |
| ✅ | Batch runner |
| ✅ | Goal capture tool |
| ✅ | Stuck detection watchdog |
| ✅ | Session-based workflow |
| ✅ | Vehicle reset script |
| ✅ | MRM state tracking |

### 1.2 Modular Experiment Framework ✅
| Status | Task |
|--------|------|
| ✅ | Create `experiments/lib/` module structure |
| ✅ | `config.py` - Configuration management |
| ✅ | `ros_utils.py` - ROS2 utilities |
| ✅ | `metrics.py` - Metrics computation |
| ✅ | `run_experiments.py` - Main experiment runner |
| ✅ | MRM diagnostic analysis script |

### 1.3 Metrics Definition ✅
| Status | Task |
|--------|------|
| ✅ | Safety metrics (collision proxy, min distance, TTC) |
| ✅ | Reliability metrics (goal success, driving time, velocity) |
| ✅ | Fail-operational metrics (MRM triggers, recovery rate) |
| ✅ | Comfort metrics (acceleration, jerk) |
| ✅ | Collision detection via planning_evaluator topic |

### 1.4 Data Pipeline Planning ✅
| Status | Task |
|--------|------|
| ✅ | Document T-ITS paper data pipeline |
| ✅ | Define recording topics (13 topics) |
| ✅ | Plan extraction → sequence → training flow |

---

## Phase 1.5: Data Collection 🔄 CURRENT

### Immediate Tasks
| Status | Task |
|--------|------|
| ✅ | Run experiments on all captured goals (baseline) - 25 goals, 15 success, 10 failed |
| ✅ | Analyze MRM triggers from rosbag data - MRM self-recovers in ~0.1s |
| ✅ | Validate metrics computation - Fixed bytes bug, goal_reached detection |
| ⏳ | Add `/planning_evaluator/metrics` to recording |
| ⏳ | Recapture goals avoiding stuck cluster (Y=50545-50600) |

### Data Pipeline Implementation
| Status | Task |
|--------|------|
| ⏳ | Implement rosbag → JSON extraction |
| ⏳ | Implement timestamp synchronization (10Hz) |
| ⏳ | Implement feature extraction |
| ⏳ | Implement sequence generation |
| ⏳ | Test with ST-GAT training |

---

## Phase 2: Stress Testing (Find Failures)

### Fault Injection
| Status | Task |
|--------|------|
| ⏳ | Create `fault_injection` ROS2 package |
| ⏳ | Implement localization noise injector |
| ⏳ | Implement perception dropout |
| ⏳ | Implement control latency |
| ⏳ | Test fault propagation to MRM |

### Failure Analysis
| Status | Task |
|--------|------|
| ⏳ | Run fault sweep experiments |
| ⏳ | Identify failure thresholds |
| ⏳ | Document failure modes |
| ⏳ | Select "goldilocks" scenarios for RISE validation |

---

## Phase 3: Digital Twin Retraining

| Status | Task |
|--------|------|
| ⏳ | Collect training data (new Autoware/AWSIM) |
| ⏳ | Adapt ST-GAT for new state representation |
| ⏳ | Add MRM state to features |
| ⏳ | Add trajectory deviation to features |
| ⏳ | Train and validate model |

---

## Phase 4: RISE Implementation

| Status | Task |
|--------|------|
| ⏳ | Implement CVaR computation from residuals |
| ⏳ | Implement constraint tightening |
| ⏳ | Implement preemptive intervention |
| ⏳ | Integration with Autoware (velocity limits) |

---

## Phase 5: Validation

| Status | Task |
|--------|------|
| ⏳ | Run baseline vs RISE comparison |
| ⏳ | Statistical analysis |
| ⏳ | Generate figures |

---

## Documentation

| File | Purpose |
|------|---------|
| `docs/theoretical_framework.md` | Core RISE formulation |
| `docs/research_notes/metrics_framework.md` | Metrics definitions |
| `docs/research_notes/data_pipeline_plan.md` | Data collection plan |
| `docs/research_notes/experiment_findings_and_mrm_analysis.md` | Experiment results, MRM analysis, historical fixes |
| `README.md` | Project overview and config changes |

**Note:** Historical fix documentation (localization_initialization_fix.md, awsim_routing_state_fix.md) consolidated into experiment_findings_and_mrm_analysis.md

---

## Key Configuration Changes

**Autoware:**
- `max_vel`: 4.17 → 11.11 m/s (15 → 40 km/h)
- File: `autoware/.../common.param.yaml`

**AWSIM:**
- Spawn position: (81384.60, 49922.00, 41.28)
- File: `experiments/configs/baseline.json`

---

## Repository Structure

```
Risk-Aware-Control/
├── autoware/                 # Pre-built Autoware
├── awsim_labs_v1.6.1/        # AWSIM binary
├── Shinjuku-Map/             # HD map
├── Run_AWSIM.sh              # Launch AWSIM
├── Run_Autoware.sh           # Launch Autoware
├── README.md                 # Project overview
├── TODO.md                   # This file
│
├── docs/
│   ├── theoretical_framework.md
│   └── research_notes/
│       ├── metrics_framework.md
│       ├── data_pipeline_plan.md
│       └── experiment_findings_and_mrm_analysis.md
│
├── experiments/
│   ├── lib/                  # Modular experiment library
│   │   ├── __init__.py
│   │   ├── config.py         # Configuration management
│   │   ├── ros_utils.py      # ROS2 utilities
│   │   └── metrics.py        # Metrics computation
│   ├── scripts/
│   │   ├── run_experiments.py          # Main experiment runner
│   │   ├── analyze_mrm_diagnostics.py  # MRM diagnostic analysis
│   │   ├── capture_goals_session.py    # Goal capture tool
│   │   ├── reset_vehicle.py            # Vehicle reset
│   │   ├── set_goal.py                 # Goal setting
│   │   └── diagnose_system.py          # System diagnostics
│   ├── configs/
│   │   ├── baseline.json         # AWSIM config
│   │   └── captured_goals.json   # Goal coordinates
│   └── data/                     # Experiment output
│
└── ros2_ws/                  # (future) Custom ROS2 packages
```
