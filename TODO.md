# Risk-Aware Control - Task List

**Last Updated:** 2026-01-28

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

**Summary:** See `docs/phase0_summary.md`

---

## Phase 1: Experiment Infrastructure 🔄 CURRENT

**Progress notes:** See `docs/phase1_progress.md`

### 1.1 Automation
| Status | Task |
|--------|------|
| ✅ | Investigate AWSIM automation options (supports `--config` JSON flag) |
| ✅ | Create experiment launcher script (`experiments/scripts/run_experiment.sh`) |
| ✅ | Create batch runner (`experiments/scripts/run_batch.sh`) |
| ✅ | Create goal capture tool (`experiments/scripts/capture_goal.py`) |
| ✅ | Create live monitor (`experiments/scripts/monitor_state.py`) |
| ✅ | Capture goal coordinates from manual RViz session |
| ✅ | Fix QoS incompatibility (BEST_EFFORT for AWSIM ground truth) |
| ✅ | Fix rclpy shutdown error (guard with `rclpy.ok()`) |
| ✅ | Fix autoware source in run scripts |
| ✅ | Investigate MRM system (diagnostic script, source code analysis) |
| ✅ | Fix velocity cap (4.17 -> 11.11 m/s = 15 -> 40 km/h) |
| ✅ | Add stuck detection watchdog (`experiment_watchdog.py`) |
| ✅ | Fix diagnostic script byte comparison bug |
| ⏳ | Investigate AWSIM traffic density options (check UI slider) |
| ⏳ | End-to-end test of updated automation pipeline |
| ⏳ | Test with 5 baseline runs at 40 km/h |

### 1.2 Metrics Definition
| Status | Task |
|--------|------|
| ⏳ | Build rosbag → metrics extraction pipeline |
| ⏳ | Define collision detection method |
| ⏳ | Define lane departure detection |
| ⏳ | Define mission success criteria |
| ⏳ | Implement TTC computation |

### 1.3 Fault Injection
| Status | Task |
|--------|------|
| ⏳ | Create `fault_injection` ROS2 package |
| ⏳ | Implement GNSS noise injector |
| ⏳ | Implement IMU bias injector |
| ⏳ | Test fault propagation |

---

## Phase 2: Stress Testing (Find Failures)

- [ ] Run fault sweep experiments
- [ ] Identify failure thresholds
- [ ] Document failure modes
- [ ] Select "goldilocks" scenarios for RISE validation

---

## Phase 3: RISE Implementation

- [ ] Implement uncertainty propagation module
- [ ] Implement tube computation
- [ ] Implement constraint adjustment
- [ ] Integration with Autoware

---

## Phase 4: Validation

- [ ] Run baseline vs RISE comparison
- [ ] Statistical analysis
- [ ] Generate figures

---

## Documentation

| File | Purpose |
|------|---------|
| `docs/theoretical_framework.md` | Core RISE formulation |
| `docs/design_decisions.md` | Key decisions with rationale |
| `docs/validation_strategy.md` | Experiment methodology |
| `docs/phase0_summary.md` | Environment exploration results |
| `docs/phase1_progress.md` | Phase 1 progress, issues, and pickup notes |

---

## Repository Structure

```
Risk-Aware-Control/
├── autoware/                 # Autoware source
├── awsim_labs_v1.6.1/        # AWSIM binary
├── awsim_labs_source/        # AWSIM source (for reference)
├── Shinjuku-Map/             # HD map
├── Run_AWSIM.sh              # Launch AWSIM
├── Run_Autoware.sh           # Launch Autoware
├── TODO.md                   # This file
│
├── docs/
│   ├── theoretical_framework.md
│   ├── design_decisions.md
│   ├── validation_strategy.md
│   └── phase0_summary.md
│
├── experiments/
│   ├── scripts/
│   │   ├── run_experiment.sh    # Single automated experiment
│   │   ├── run_batch.sh         # Batch runner
│   │   ├── capture_goal.py      # Capture goal from RViz
│   │   ├── monitor_state.py     # Live state monitor
│   │   ├── diagnose_mrm.py      # MRM diagnostic investigation
│   │   └── experiment_watchdog.py # Stuck/completion detection
│   ├── configs/
│   │   ├── baseline.json        # AWSIM startup config
│   │   └── captured_route.json  # Captured goal coordinates
│   └── data/                    # Experiment output (rosbags, metadata)
│
└── ros2_ws/                  # (to be built)
    └── src/
        └── fault_injection/
```
