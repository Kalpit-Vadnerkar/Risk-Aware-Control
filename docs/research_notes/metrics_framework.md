# Metrics Framework for RISE Validation

**Purpose:** Define metrics to evaluate AV safety, reliability, and fail-operational capability across three conditions:
1. **Baseline (Normal)** - No faults, standard driving
2. **Fault Injection** - Sensor noise, delays, failures
3. **RISE Active** - Our system running with constraint tightening

## Metric Categories

### 1. Safety Metrics

These measure how well the system avoids dangerous situations.

| Metric | Formula/Method | Unit | Good Direction |
|--------|---------------|------|----------------|
| **Collision Count** | Count of collisions per run | count | ↓ Lower is better |
| **Minimum TTC** | min(TTC) across run, TTC = distance/closing_velocity | seconds | ↑ Higher is better |
| **Near-Miss Count** | Count of TTC < threshold (e.g., 2s) | count | ↓ Lower is better |
| **Lane Departure Rate** | Time outside lane / Total driving time | % | ↓ Lower is better |
| **Lateral Safety Margin** | min(distance to lane boundary) | meters | ↑ Higher is better |
| **Object Proximity Violations** | Time within unsafe distance of objects | seconds | ↓ Lower is better |

**Collision Detection:**
AWSIM does not publish a dedicated collision topic. Options:
1. **Monitor `/perception/object_recognition/objects`** - Check if any object has distance < vehicle_radius
2. **Post-process ground truth** - Compare ego position with object positions from bag
3. **AWSIM logs** - Check Unity console for collision callbacks (manual inspection)

Recommended: Compute **Minimum Object Distance** from perception data:
```python
min_distance = min(object.distance for object in detected_objects)
collision_proxy = 1 if min_distance < COLLISION_THRESHOLD else 0
```

### 2. Reliability Metrics

These measure consistent, predictable behavior.

| Metric | Formula/Method | Unit | Good Direction |
|--------|---------------|------|----------------|
| **Goal Success Rate** | Successful arrivals / Total attempts | % | ↑ Higher is better |
| **Trajectory Tracking Error (Lateral)** | RMS of lateral deviation from planned path | meters | ↓ Lower is better |
| **Trajectory Tracking Error (Longitudinal)** | RMS of longitudinal deviation from planned path | meters | ↓ Lower is better |
| **Velocity Tracking Error** | RMS of (actual_vel - planned_vel) | m/s | ↓ Lower is better |
| **Stuck Rate** | Experiments ended as stuck / Total experiments | % | ↓ Lower is better |
| **Completion Time Variance** | σ of completion times across same route | seconds | ↓ Lower is better |

### 3. Fail-Operational Metrics

These measure graceful degradation under stress.

| Metric | Formula/Method | Unit | Good Direction |
|--------|---------------|------|----------------|
| **MRM Trigger Rate** | MRM triggers / Driving time | triggers/min | Context-dependent* |
| **MRM Duration** | Total time in MRM_OPERATING state | seconds | ↓ Lower is better |
| **Recovery Rate** | Successful MRM recoveries / Total MRM triggers | % | ↑ Higher is better |
| **Time-to-MRM** | Time from fault injection to MRM trigger | seconds | Context-dependent* |
| **EMERGENCY vs COMFORTABLE Ratio** | EMERGENCY_STOP / (EMERGENCY + COMFORTABLE) | ratio | ↓ Lower is better |
| **Degraded Mode Driving** | Time driving with degraded diagnostics | seconds | Informational |

*MRM interpretation:
- **Too many MRM triggers** = System overly conservative, poor usability
- **Too few under faults** = System not detecting real risks
- **RISE goal**: Reduce unnecessary MRMs while maintaining safety

### 4. Comfort Metrics (Secondary)

| Metric | Formula/Method | Unit | Good Direction |
|--------|---------------|------|----------------|
| **Maximum Jerk** | max(|d(accel)/dt|) | m/s³ | ↓ Lower is better |
| **Maximum Lateral Acceleration** | max(|a_lateral|) | m/s² | ↓ Lower is better |
| **Deceleration Events** | Count of decel > threshold | count | ↓ Lower is better |

## Measurement Protocol

### Baseline (Normal Conditions)
```
For each goal in validated_goals:
    For trial in range(N_TRIALS):
        Run experiment with no modifications
        Record all metrics
Compute: mean, std, min, max for each metric
```

### Fault Injection Conditions

| Fault Type | Implementation | Severity Levels |
|------------|----------------|-----------------|
| **Localization Noise** | Add Gaussian noise to pose | σ = [0.1, 0.5, 1.0] m |
| **Localization Delay** | Buffer and delay pose messages | delay = [100, 500, 1000] ms |
| **Perception Dropout** | Randomly drop object detections | rate = [10%, 30%, 50%] |
| **Sensor Failure** | Stop publishing specific sensor | LiDAR, Camera, IMU |
| **Control Latency** | Delay control commands | delay = [50, 100, 200] ms |

### RISE Active
```
For each goal in validated_goals:
    For each fault_condition in fault_matrix:
        For trial in range(N_TRIALS):
            Run with RISE enabled
            Record all metrics + RISE-specific metrics
```

### RISE-Specific Metrics

| Metric | Description | Unit |
|--------|-------------|------|
| **CVaR Value** | Computed tail-risk metric | unitless |
| **Constraint Tightening Factor** | How much constraints were reduced | % |
| **Preemptive Intervention Count** | Times RISE tightened before violation | count |
| **Prediction Residual** | Digital Twin prediction error | varies |
| **RISE Latency** | Time from data to constraint update | ms |

## Data Requirements

### Per-Experiment Output

```json
{
  "experiment_id": "goal_001_fault_loc_noise_0.5_trial_3",
  "condition": {
    "type": "localization_noise",
    "parameters": {"sigma": 0.5}
  },
  "rise_enabled": true,

  "safety_metrics": {
    "collision_count": 0,
    "min_ttc": 4.2,
    "near_miss_count": 1,
    "lane_departure_rate": 0.02,
    "min_lateral_margin": 0.8,
    "min_object_distance": 2.1
  },

  "reliability_metrics": {
    "goal_reached": true,
    "lateral_rmse": 0.15,
    "longitudinal_rmse": 0.23,
    "velocity_rmse": 0.8,
    "completion_time": 85.3
  },

  "fail_operational_metrics": {
    "mrm_trigger_count": 3,
    "mrm_total_duration": 5.2,
    "emergency_stop_count": 1,
    "comfortable_stop_count": 2,
    "recovery_rate": 1.0
  },

  "rise_metrics": {
    "mean_cvar": 0.45,
    "max_cvar": 0.82,
    "tightening_events": 12,
    "preemptive_interventions": 2,
    "mean_residual": 0.08
  }
}
```

## Analysis Plan

### Comparative Analysis

For each metric M:
```
Δ_fault = M(fault) - M(baseline)           # Impact of fault
Δ_rise = M(fault+RISE) - M(fault)          # Impact of RISE
Improvement = -Δ_rise / Δ_fault            # % recovery toward baseline
```

### Statistical Tests

1. **Paired t-test**: Compare same routes with/without RISE
2. **ANOVA**: Compare across fault severity levels
3. **Correlation**: CVaR vs actual violations

### Key Hypotheses

**H1:** RISE reduces collision proxy metrics under fault conditions
- Metric: min_object_distance, near_miss_count
- Test: Wilcoxon signed-rank test (non-parametric)

**H2:** RISE reduces unnecessary MRM triggers
- Metric: mrm_trigger_count, mrm_duration
- Condition: Only for non-critical situations

**H3:** CVaR correlates with actual constraint violations
- Metric: Pearson correlation of CVaR vs violation_count
- Expected: r > 0.7

**H4:** RISE provides preemptive warning
- Metric: Time between CVaR spike and violation
- Expected: CVaR elevates before violation in >80% of cases

## Implementation Checklist

- [ ] Add collision proxy computation to watchdog
- [ ] Add trajectory deviation computation (requires planned path)
- [ ] Implement TTC computation from object data
- [ ] Create fault injection framework
- [ ] Implement RISE metrics logging
- [ ] Create analysis/visualization scripts

## Next Steps

1. Finalize collision detection approach (recommend proximity-based)
2. Add `/planning/mission_planning/route` to recording for path deviation
3. Implement fault injection nodes
4. Design DT retraining pipeline for new Autoware/AWSIM version
