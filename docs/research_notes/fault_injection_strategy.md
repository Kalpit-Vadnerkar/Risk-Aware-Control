# Fault Injection Strategy for RISE Validation

**Created:** 2026-02-05
**Purpose:** Define fault scenarios and implementation approach for testing fail-operational capabilities

---

## Research Goal

Demonstrate that RISE (Residual-Informed Safety Envelopes) can:
1. Detect degraded conditions BEFORE they cause MRM triggers
2. Preemptively tighten constraints to prevent constraint violations
3. Reduce emergency stops while maintaining safety

---

## AWSIM Capabilities Summary

### What's Available (Binary Level)
| Capability | Level | Notes |
|------------|-------|-------|
| NPC spawning | High | Via RViz, position + velocity controllable |
| Traffic control | High | Enable/disable, time scale |
| LiDAR noise | Medium | Gaussian distance/angular noise parameters |
| Camera distortion | Medium | Plumb Bob model coefficients |
| Smoke/particles | Medium | Can occlude sensors |

### What's NOT Available (Requires Source Modification)
| Capability | Alternative |
|------------|-------------|
| Weather (rain, fog) | Not implemented in v1.6.1 |
| Sensor dropout | ROS2 fault injection layer |
| Sensor latency | ROS2 fault injection layer |
| NPC swerving/complex behaviors | Manual control or OpenSCENARIO |
| GPS spoofing | ROS2 topic manipulation |

### OpenSCENARIO (scenario_simulator_v2)
- Prototype support exists
- Requires rebuilding Autoware with scenario_simulator_v2
- Complex setup, stability not guaranteed
- **Recommendation:** Defer unless simpler approaches fail

---

## Fault Injection Architecture

### Option A: ROS2 Fault Injection Layer (Recommended)

```
┌─────────────────────────────────────────────────────────────┐
│                        AWSIM                                 │
│  (Clean sensor data, NPC traffic)                           │
└──────────────────────┬──────────────────────────────────────┘
                       │ Original Topics
                       ▼
┌─────────────────────────────────────────────────────────────┐
│              Fault Injection Node (ROS2)                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │ Perception   │  │ Localization │  │   Sensor     │       │
│  │   Faults     │  │    Faults    │  │   Faults     │       │
│  │ - Dropout    │  │ - Noise      │  │ - Latency    │       │
│  │ - Delay      │  │ - Drift      │  │ - Dropout    │       │
│  │ - False +/-  │  │ - Jump       │  │ - Noise      │       │
│  └──────────────┘  └──────────────┘  └──────────────┘       │
└──────────────────────┬──────────────────────────────────────┘
                       │ Faulted Topics (remapped)
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                      Autoware                                │
│  (Receives faulted data, must handle gracefully)            │
└─────────────────────────────────────────────────────────────┘
```

**Pros:**
- Fast iteration, no AWSIM rebuild
- Precise control over fault parameters
- Easy to enable/disable specific faults
- Reproducible experiments

**Cons:**
- Added latency (~1-5ms per topic)
- Not physically realistic (faults at data level, not sensor level)

**Latency Mitigation:**
- Use efficient message passing (zero-copy if possible)
- Run fault injection on same machine
- Measure and document injection latency
- For real-time critical topics, consider C++ implementation

### Option B: AWSIM Source Modification

Modify Unity project to inject faults at sensor level.

**Pros:**
- Physically realistic
- No ROS2 latency

**Cons:**
- Requires Unity expertise
- Longer iteration time (rebuild for each change)
- Complex to parameterize

**Recommendation:** Use Option A for initial experiments, consider Option B for final validation if needed.

---

## Fault Categories and Scenarios

### Category 1: Perception Faults (Primary Focus)

These directly affect the ST-GAT predictions since the model uses perception data.

| Fault | Implementation | Severity Levels |
|-------|----------------|-----------------|
| **Object Detection Dropout** | Don't republish some objects | 10%, 30%, 50% drop rate |
| **Object Detection Delay** | Buffer and delay messages | 100ms, 300ms, 500ms |
| **False Positive Objects** | Inject phantom objects | 1, 3, 5 objects |
| **Object Position Noise** | Add Gaussian noise to positions | σ = 0.5m, 1m, 2m |
| **Classification Error** | Change object types | 5%, 10%, 20% error rate |

**Target Topics:**
- `/perception/object_recognition/objects` (PredictedObjects)
- `/perception/object_recognition/tracking/objects` (TrackedObjects)

### Category 2: Localization Faults

| Fault | Implementation | Severity Levels |
|-------|----------------|-----------------|
| **Position Drift** | Add cumulative offset | 0.1m/s, 0.5m/s, 1m/s drift |
| **Position Jump** | Sudden position shift | 1m, 3m, 5m jump |
| **Covariance Spike** | Increase uncertainty | 2x, 5x, 10x covariance |
| **Heading Error** | Add heading offset | 1°, 5°, 10° |

**Target Topics:**
- `/localization/kinematic_state`
- `/localization/pose_with_covariance`

### Category 3: Sensor Faults

| Fault | Implementation | Severity Levels |
|-------|----------------|-----------------|
| **LiDAR Dropout** | Stop publishing | 100ms, 500ms, 1s blackout |
| **Camera Blackout** | Publish black image | 100ms, 500ms, 1s |
| **Partial Occlusion** | Mask regions of sensor data | 10%, 30%, 50% |

**Target Topics:**
- `/sensing/lidar/concatenated/pointcloud`
- `/sensing/camera/*`

### Category 4: Scenario-Based Challenges (NPC Behavior)

These test the vehicle's ability to handle dynamic situations.

| Scenario | Setup | Expected Behavior |
|----------|-------|-------------------|
| **Static Obstacle** | Spawn stationary NPC in lane | Lane change or stop |
| **Cut-in** | NPC enters lane ahead | Decelerate |
| **Sudden Brake** | Lead NPC brakes hard | Emergency response |
| **Pedestrian Crossing** | Pedestrian enters path | Stop |

**Implementation:**
- Use RViz NPC Spawner for manual placement
- Script ROS2 pose publishers for reproducible scenarios

---

## Prioritized Experiment Plan

### Phase 2A: Perception Fault Experiments (Week 1-2)

**Objective:** Find thresholds where baseline Autoware triggers MRM

1. **Object Detection Delay**
   - Baseline: Measure normal processing latency
   - Sweep: 0ms → 500ms in 50ms steps
   - Metric: MRM trigger rate, near-miss rate

2. **Object Detection Dropout**
   - Sweep: 0% → 50% in 10% steps
   - Metric: Min clearance, collision proxy count

3. **Object Position Noise**
   - Sweep: σ = 0 → 2m in 0.25m steps
   - Metric: Path deviation, MRM rate

### Phase 2B: Localization Fault Experiments (Week 2-3)

**Objective:** Test localization degradation handling

1. **Position Noise**
   - Sweep: σ = 0 → 1m
   - Metric: Path deviation, lane departure

2. **Covariance Spike**
   - Trigger temporary uncertainty spikes
   - Metric: Does Autoware react appropriately?

### Phase 2C: Scenario Challenges (Week 3-4)

**Objective:** Test dynamic obstacle handling

1. **Static Obstacle in Lane**
   - Place NPC at various distances ahead
   - Measure: Does vehicle stop? Change lanes? Time to react?

2. **Lead Vehicle Sudden Stop**
   - NPC traveling ahead brakes to zero
   - Measure: Collision avoidance success, min clearance

---

## Implementation Plan

### Step 1: Create Fault Injection ROS2 Package

```
ros2_ws/
└── src/
    └── fault_injection/
        ├── package.xml
        ├── CMakeLists.txt
        ├── fault_injection/
        │   ├── __init__.py
        │   ├── perception_faults.py
        │   ├── localization_faults.py
        │   └── sensor_faults.py
        ├── launch/
        │   └── fault_injection.launch.py
        └── config/
            └── fault_params.yaml
```

### Step 2: Implement Core Fault Injectors

```python
# Example: Perception Dropout Fault
class PerceptionDropoutFault(Node):
    def __init__(self):
        super().__init__('perception_dropout_fault')
        self.drop_rate = self.declare_parameter('drop_rate', 0.0).value

        self.sub = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            PredictedObjects,
            '/perception/object_recognition/objects_faulted',
            10
        )

    def callback(self, msg):
        # Randomly drop objects based on drop_rate
        if random.random() > self.drop_rate:
            self.pub.publish(msg)
```

### Step 3: Create Topic Remapping Launch File

Remap Autoware to receive faulted topics instead of original.

### Step 4: Integrate with Experiment Framework

Add fault parameters to experiment configuration.

---

## Metrics for Fault Experiments

### Primary Metrics (Same as Baseline)
- MRM rate (per km)
- E-stop ratio
- Near-miss rate
- Min clearance (P5)
- Path deviation

### Fault-Specific Metrics
- **Detection Latency Tolerance:** Max delay before MRM
- **Dropout Tolerance:** Max drop rate before safety degradation
- **Noise Tolerance:** Max position noise before lane departure

### Research Questions Answered

| Question | Experiment | Expected Finding |
|----------|------------|------------------|
| At what perception delay does MRM trigger? | Delay sweep | ~200-300ms threshold |
| Can RISE detect delay before MRM? | RISE vs baseline with delay | RISE tightens constraints at ~100ms |
| Does constraint tightening reduce MRM rate? | RISE vs baseline | RISE: fewer MRMs, same safety |

---

## Timeline

| Week | Focus | Deliverable |
|------|-------|-------------|
| 1 | Fault injection package skeleton | Working ROS2 package |
| 2 | Perception fault implementation | Dropout, delay, noise injectors |
| 3 | Fault sweep experiments | Threshold data for perception faults |
| 4 | Localization faults + scenarios | Complete fault characterization |

---

## Decision: Why ROS2 Layer Over AWSIM Modification

1. **Fast iteration:** Change parameters without rebuild
2. **Reproducibility:** Exact same fault can be replayed
3. **Decoupled:** Fault injection independent of simulator version
4. **Measurable:** Can log exactly what was injected
5. **Realistic enough:** For research purposes, data-level faults are sufficient

The goal is to show RISE works, not to build a production fault injection system.

---

## Open Questions

1. **Latency impact:** Is 1-5ms injection latency acceptable?
   - Need to measure actual latency
   - Compare to Autoware's normal processing latency (~10-50ms)

2. **Topic remapping complexity:** How to cleanly remap topics?
   - Option: Launch fault injection with Autoware
   - Option: Use ROS2 namespace remapping

3. **NPC scripting:** How to create reproducible NPC behaviors?
   - Simple: Use ROS2 to publish NPC pose commands
   - Complex: Need OpenSCENARIO

