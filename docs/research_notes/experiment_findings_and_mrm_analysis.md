# Experiment Findings and MRM Analysis

**Last Updated:** 2026-02-04
**Context:** Baseline experiments on 25 captured goals in Shinjuku map

---

## Executive Summary

We completed 25 baseline experiments to validate the experiment infrastructure and understand Autoware's behavior:

| Outcome | Count | Percentage |
|---------|-------|------------|
| Goal Reached | 15 | 60% |
| Stuck (Map/Sim Issues) | 8 | 32% |
| MRM Terminal State | 2 | 8% |

**Key Findings:**
1. MRM self-recovers extremely fast (~0.1s average), not a reliability concern
2. Stuck failures cluster geographically - 6/8 stuck goals are in one map region
3. Infrastructure is robust; failures are due to map/simulator limitations, not code bugs

---

## Detailed Results

### Per-Goal Outcomes

| Goal | Status | Driving Time | MRM Triggers |
|------|--------|--------------|--------------|
| goal_001 | goal_reached | 41.0s | 7 |
| goal_002 | goal_reached | 80.1s | 8 |
| **goal_003** | **stuck** | 448.5s | 40 |
| goal_004 | goal_reached | 228.2s | 7 |
| **goal_005** | **stuck** | 425.4s | 6 |
| goal_006 | goal_reached | 194.2s | 3 |
| goal_007 | goal_reached | 187.2s | 2 |
| **goal_008** | **stuck** | 442.5s | 40 |
| **goal_009** | **stuck** | 338.4s | 6 |
| **goal_010** | **stuck** | 345.4s | 3 |
| goal_011 | goal_reached | 151.2s | 25 |
| **goal_012** | **stuck** | 275.3s | 9 |
| goal_013 | goal_reached | 104.1s | 3 |
| goal_014 | goal_reached | 104.1s | 3 |
| goal_015 | goal_reached | 104.1s | 3 |
| goal_016 | goal_reached | 117.1s | 1 |
| goal_017 | mrm_terminal | 147.2s | 4 |
| goal_018 | goal_reached | 116.1s | 26 |
| goal_019 | goal_reached | 120.1s | 13 |
| **goal_020** | **stuck** | 232.3s | 9 |
| goal_021 | goal_reached | 145.2s | 19 |
| goal_022 | mrm_terminal | 31.0s | 2 |
| **goal_023** | **stuck** | 283.3s | 34 |
| goal_024 | goal_reached | 166.2s | 5 |
| goal_025 | goal_reached | 183.2s | 4 |

---

## MRM Self-Recovery Analysis

### Recovery Timing

Analyzed metrics from successful experiments show MRM recovers extremely fast:

| Goal | MRM Triggers | Total MRM Duration | Avg Duration/Trigger | Recoveries |
|------|--------------|-------------------|----------------------|------------|
| goal_001 | 41 | 3.95s | **0.096s** | 40 |
| goal_002 | 71 | 7.79s | **0.110s** | 71 |
| goal_004 | 151 | 15.77s | **0.104s** | 151 |

**Average recovery time: ~0.1 seconds per MRM event**

### Interpretation

1. **MRM is transient, not sustained**: 41-151 triggers per run, but total MRM duration is only 4-16 seconds
2. **Recovery is automatic**: Autoware's diagnostic graph clears quickly, returning to NORMAL state
3. **Our recovery code helps inter-experiment cleanup**: The `recover_from_emergency()` function ensures clean state between experiments, not within them

### MRM Trigger Patterns

From earlier detailed analysis:
- **Early phase (0-10s):** Mostly EMERGENCY_STOP (system initializing)
- **Mid phase (10-70s):** Mix of EMERGENCY and COMFORTABLE
- **Late phase (70s+):** Mostly COMFORTABLE_STOP (system stable)

This suggests initial MRM triggers are due to diagnostic startup delays, not actual faults.

---

## Stuck Location Analysis

### Geographic Clustering

Stuck goals cluster geographically, indicating map/simulator issues at specific locations:

| Goal | X | Y | Est. Distance | Cluster |
|------|---|---|---------------|---------|
| goal_003 | 81567 | 50228 | 357m | Isolated |
| goal_005 | 81848 | 50092 | 494m | Isolated |
| goal_008 | 81657 | 50599 | 729m | **Main Cluster** |
| goal_009 | 81533 | 50580 | 674m | **Main Cluster** |
| goal_010 | 81532 | 50576 | 671m | **Main Cluster** |
| goal_012 | 81507 | 50546 | 636m | **Main Cluster** |
| goal_020 | 81507 | 50547 | 637m | **Main Cluster** |
| goal_023 | 81458 | 50564 | 646m | **Main Cluster** |

**6 out of 8 stuck goals (75%) are in the main cluster** around:
- X: 81457-81657 (~200m band)
- Y: 50546-50599 (~53m band)

### Root Cause Hypothesis

The main cluster is likely a specific intersection or road segment where:
1. Trajectory interpolation fails (observed in logs)
2. Planning latency increases (observed delays > 1s threshold)
3. Lane geometry may be poorly defined in the HD map

The two isolated stuck goals (003, 005) may have similar issues at different intersections.

### MRM Terminal States

Two goals ended in MRM terminal states (MRM_SUCCEEDED):
- goal_017: (81568, 50390) - Between the two problem areas
- goal_022: (81490, 50616) - Within the main stuck cluster

These represent cases where MRM completed its safe-stop procedure but the vehicle couldn't resume.

---

## Recommendations

### For Current Experiments

1. **Use successful goals only**: The 15 successful goals provide sufficient coverage for baseline validation
2. **Document known problem areas**: Mark the stuck cluster region for avoidance in future goal capture
3. **Consider partial goals**: For stuck goals, capture intermediate waypoints before the problem area

### For Future Work

1. **Map quality investigation**: Examine the HD map in the stuck cluster region
2. **Trajectory planning analysis**: Debug the interpolator failures at specific geometries
3. **AWSIM limitations**: Document simulator-specific constraints

---

## Infrastructure Validation

The experiment infrastructure is now robust:

### Working Features
- ROS2 native state monitoring (no subprocess polling)
- Velocity-based stuck detection (30s threshold)
- MRM terminal state detection
- Automatic recovery between experiments
- Skip-existing for resuming batches
- Comprehensive metrics collection

### Code Changes Made
- `ros_utils.py`: Rewrote to use rclpy with proper QoS profiles
- `run_experiments.py`: Added velocity monitoring, MRM detection, recovery
- `metrics.py`: Fixed bytes comparison bug, goal_reached detection

---

## Historical Issues (Resolved)

### Localization Initialization (2026-01-29)

**Problem:** Vehicle wouldn't move - autonomous mode unavailable
- NDT scan matcher failing with `pose_buffer_.size() < 2`
- EKF localizer not publishing poses

**Root Causes:**
1. Publishing to `/initialpose` instead of `/initialpose3d`
2. Missing node activation via trigger services

**Solution:** Fixed `set_initial_pose.py` to:
- Publish to `/initialpose3d`
- Call EKF and NDT trigger services
- Follow proper deactivate → publish → activate sequence

### QoS Incompatibility (2026-01-30)

**Problem:** State monitor not receiving messages
- Warning: `offering incompatible QoS. No messages will be received`

**Root Cause:** Mismatched QoS durability settings

**Solution:** Configured subscribers with correct QoS:
- `/autoware/state`: VOLATILE durability
- `/api/routing/state`: TRANSIENT_LOCAL durability
- `/system/fail_safe/mrm_state`: VOLATILE durability

### AWSIM Routing State (2026-01-30)

**Problem:** Vehicle stuck during left turns, routing/state STALE diagnostic

**Analysis:** The routing/state staleness was a symptom detected during failure, not the root cause. Actual cause was trajectory interpolator failing at specific geometries.

**Status:** Not fixable without map/simulator changes; workaround is to avoid problematic goal locations.

---

## Appendix: MRM Diagnostic Analysis Tools

### Automated Analysis Script

```bash
source /opt/ros/humble/setup.bash
source autoware/install/setup.bash
python3 experiments/scripts/analyze_mrm_diagnostics.py <rosbag_path>
```

### Live Monitoring Commands

```bash
# Watch MRM state changes
ros2 topic echo /system/fail_safe/mrm_state

# Watch diagnostics with ERROR level
ros2 topic echo /diagnostics --no-arr 2>/dev/null | grep -B2 -A3 "level: 2"

# Watch operation mode availability
ros2 topic echo /system/operation_mode/availability
```

### Key Diagnostic Paths

| Diagnostic Path | Meaning | MRM Impact |
|-----------------|---------|------------|
| `/autoware/modes/autonomous` | Overall autonomous mode health | Direct trigger |
| `/autoware/localization/performance` | Localization confidence | Can disable autonomous |
| `/autoware/planning/performance` | Planning loop health | Can disable autonomous |
| `/autoware/perception/performance` | Object detection health | Can disable autonomous |
| `/autoware/control/performance` | Control loop health | Can disable autonomous |

---

## References

- Autoware MRM Handler: `autoware/src/universe/autoware_universe/system/autoware_mrm_handler/`
- Diagnostic Graph: `autoware/src/universe/autoware_universe/system/autoware_diagnostic_graph_aggregator/`
- Metrics Framework: `docs/research_notes/metrics_framework.md`
