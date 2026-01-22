# Risk-Aware Control - Task List

**Last Updated:** 2026-01-21
**Status Legend:** ✅ Complete | 🔄 In Progress | ⏳ Pending | ❌ Blocked | 🔍 Needs Research

---

## Development Philosophy

1. **Work in small chunks** - Pick specific tasks, not entire phases
2. **Validation-first** - Know what we're measuring before building solutions
3. **Document everything** - Decisions, rationale, alternatives considered
4. **Find the failure points first** - Can't fix what's not broken

---

## Phase 0: Environment Setup & Exploration

**Goal:** Verify AWSIM + Autoware work reliably, understand what's available

| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Verify AWSIM launches | Test `./Run_AWSIM.sh` |
| ⏳ | Verify Autoware launches | Test `./Run_Autoware.sh` |
| ⏳ | Complete one manual drive | Drive through Shinjuku map successfully |
| ⏳ | Document available ROS2 topics | List all topics published by Autoware |
| ⏳ | Document AWSIM fault injection API | What faults can AWSIM inject natively? |
| ⏳ | Document Autoware parameters | Which params control safety margins? |
| 🔍 | Identify constraint parameters | **What constraints can we manipulate?** |

### Key Questions to Answer:
- [ ] What fault injection does AWSIM support out of the box?
- [ ] What ROS2 topics carry localization covariance?
- [ ] What ROS2 topics carry perception confidence?
- [ ] Which Autoware parameters control obstacle distance margins?
- [ ] Which parameters control velocity limits?
- [ ] Which parameters control lane keeping tolerance?

---

## Phase 1: Experiment Infrastructure

**Goal:** Create a reliable way to run repeatable experiments

### 1.1 Manual Experiment Process
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Document manual experiment steps | How to run one experiment by hand |
| ⏳ | Identify key metrics to record | What do we measure? |
| ⏳ | Create simple data recording method | rosbag? CSV? What's easiest? |
| ⏳ | Run 3 manual baseline experiments | Verify process works |

### 1.2 Metrics Definition
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Define "collision" detection | How do we know a collision occurred? |
| ⏳ | Define "lane departure" detection | How do we know ego left the lane? |
| ⏳ | Define "mission success" criteria | What counts as successful completion? |
| ⏳ | Research TTC computation | How to compute time-to-collision? |
| 🔍 | Determine feasible metrics | What can we actually measure reliably? |

### 1.3 Simple Experiment Script
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Create experiment launcher script | Start AWSIM + Autoware + recording |
| ⏳ | Create experiment stopper script | Clean shutdown + save data |
| ⏳ | Test with 5 baseline runs | Verify automation works |

---

## Phase 2: Stress Testing (Find Failure Points)

**Goal:** Discover where Autoware fails so we know what to fix

### 2.1 Native Fault Testing
| Status | Task | Description |
|--------|------|-------------|
| 🔍 | Investigate AWSIM fault injection | What's available natively? |
| ⏳ | Test with sensor noise (if available) | Does Autoware handle it? |
| ⏳ | Test with sensor dropout (if available) | Does Autoware handle it? |
| ⏳ | Document which faults cause failures | Find the "interesting region" |

### 2.2 Parameter Manipulation Testing
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Test with reduced safety margins | Lower obstacle distance threshold |
| ⏳ | Test with increased velocity limits | Push Autoware faster |
| ⏳ | Document parameter-failure relationships | What causes problems? |

### 2.3 Scenario Complexity
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Test with dense traffic (if possible) | More objects = more challenge |
| ⏳ | Test with complex route | Turns, intersections |
| ⏳ | Document scenario-failure relationships | What scenarios are hard? |

### 2.4 Failure Analysis
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Catalog all observed failures | What went wrong and when? |
| ⏳ | Classify failures by type | Collision, lane departure, stuck, etc. |
| ⏳ | Identify "goldilocks" scenarios | Not too easy, not impossible |
| ⏳ | Document in research notes | `failure_analysis.md` |

---

## Phase 3: Design Risk Assessment (AFTER Phase 2)

**Goal:** Design the CVaR system based on empirical findings

### 3.1 Constraint Identification
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | List controllable constraints | Based on Phase 0 exploration |
| ⏳ | Map constraints to failure modes | Which constraint prevents which failure? |
| ⏳ | Prioritize constraints | Which are most impactful? |
| ⏳ | Document in research notes | `constraint_selection.md` |

### 3.2 Metric Selection
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Select input features for ST-GAT | What do we predict? |
| ⏳ | Select residual types to use | Raw, KL, CUSUM - which ones? |
| ⏳ | Select CVaR parameters | Alpha level, window size |
| ⏳ | Document rationale | `metric_selection.md` |

### 3.3 Tightening Strategy Design
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Design γ(CVaR) mapping | How does CVaR translate to margin? |
| ⏳ | Set constraint bounds | Min/max for each constraint |
| ⏳ | Design preemptive trigger | When to tighten early? |
| ⏳ | Document design decisions | `tightening_design.md` |

---

## Phase 4: Implementation (AFTER Phase 3)

**Goal:** Build the system based on validated design

### 4.1 CVaR Estimator
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Implement CVaR computation | Based on selected parameters |
| ⏳ | Implement rolling window | For real-time estimation |
| ⏳ | Implement trend detection | For preemptive tightening |
| ⏳ | Unit tests | Verify correctness |

### 4.2 Safety Envelope
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Implement constraint tightener | Based on selected constraints |
| ⏳ | Implement Autoware interface | Parameter updates |
| ⏳ | Integration test | Verify updates take effect |

### 4.3 ST-GAT Port (if needed)
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Port model architecture | From reference repo |
| ⏳ | Port data pipeline | Adapt for ROS2 Humble |
| ⏳ | Verify inference | Load weights, run prediction |

---

## Phase 5: Validation (AFTER Phase 4)

**Goal:** Prove the system works

### 5.1 Comparative Experiments
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Run baseline (no RISE) | On failure scenarios from Phase 2 |
| ⏳ | Run with RISE | Same scenarios |
| ⏳ | Compute comparison metrics | Did we improve? |

### 5.2 Statistical Analysis
| Status | Task | Description |
|--------|------|-------------|
| ⏳ | Significance testing | Is improvement real? |
| ⏳ | Effect size computation | How big is the improvement? |
| ⏳ | Generate figures | For dissertation |

---

## Documentation Tasks (Ongoing)

| Status | Task | Description |
|--------|------|-------------|
| ✅ | Create theoretical framework | `docs/theoretical_framework.md` |
| ✅ | Document weight modulation rejection | `docs/research_notes/weight_modulation_rejected.md` |
| ✅ | Document intrinsic delay analysis | `docs/research_notes/intrinsic_delay_analysis.md` |
| ✅ | Document fault severity taxonomy | `docs/research_notes/fault_severity_taxonomy.md` |
| ✅ | Document validation strategy | `docs/research_notes/validation_strategy.md` |
| ⏳ | Document AWSIM capabilities | After Phase 0 exploration |
| ⏳ | Document Autoware parameters | After Phase 0 exploration |
| ⏳ | Document failure analysis | After Phase 2 |
| ⏳ | Document constraint selection | After Phase 3 |
| ⏳ | Document metric selection | After Phase 3 |
| ⏳ | Document tightening design | After Phase 3 |

---

## Current Files

```
Risk-Aware-Control/
├── autoware/                 # Pre-built Autoware
├── awsim_labs_v1.6.1/        # AWSIM binary
├── Shinjuku-Map/             # HD map
├── Run_AWSIM.sh              # Launch script
├── Run_Autoware.sh           # Launch script
├── TODO.md                   # This file
│
├── docs/
│   ├── theoretical_framework.md
│   └── research_notes/
│       ├── weight_modulation_rejected.md
│       ├── intrinsic_delay_analysis.md
│       ├── fault_severity_taxonomy.md
│       └── validation_strategy.md
│
├── experiments/
│   ├── scenarios/            # (empty - to be created)
│   └── configs/
│       ├── baseline.yaml     # (template)
│       └── fault_sweep.yaml  # (template)
│
├── ros2_ws/                  # (empty - to be built)
└── st_gat/                   # (empty - to be ported)
```

---

## Next Steps Suggestions

**Recommended starting point:** Phase 0 tasks

1. Verify AWSIM + Autoware launch successfully
2. Document available ROS2 topics
3. Investigate AWSIM fault injection capabilities
4. Identify which Autoware parameters control safety margins

This gives us the foundation to design experiments properly.
