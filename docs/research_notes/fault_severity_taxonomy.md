# Fault Severity Taxonomy

**Date:** 2026-01-21
**Purpose:** Define and justify fault severity levels for validation experiments

## Severity Level Definitions

### Level 0: None (Baseline)
- No fault injection
- Nominal sensor operation
- Used to establish baseline metrics

### Level 1: Mild (10-25% degradation)
- Minor perturbations that typical sensor fusion should handle
- **Expected Outcome:** System handles gracefully, no intervention needed
- **Our System Value:** Minimal - validates that we don't over-react

### Level 2: Moderate (25-50% degradation)
- Noticeable degradation affecting some perception/localization accuracy
- **Expected Outcome:** Degraded performance but mission completion
- **Our System Value:** Marginal improvement in safety metrics

### Level 3: Severe (50-75% degradation)
- Significant degradation that may challenge the control system
- **Expected Outcome:** Baseline may have safety violations
- **Our System Value:** **Primary target - collision prevention**

### Level 4: Critical (75%+ degradation)
- Near-complete sensor failure
- **Expected Outcome:** Baseline will fail
- **Our System Value:** Reduce severity of inevitable incidents

## Fault-Specific Configurations

### IMU Bias

| Severity | Accel Bias (m/s²) | Gyro Bias (rad/s) | Rationale |
|----------|-------------------|-------------------|-----------|
| Mild | [0.1, 0.05, 0.02] | [0.01, 0.01, 0.005] | ~10% of typical accel |
| Moderate | [0.3, 0.15, 0.05] | [0.03, 0.03, 0.015] | ~30% of typical accel |
| Severe | [0.6, 0.3, 0.1] | [0.06, 0.06, 0.03] | ~60% of typical accel |
| Critical | [1.0, 0.5, 0.2] | [0.1, 0.1, 0.05] | Exceeds typical range |

**Reference:** Typical automotive IMU noise ~0.1 m/s², bias ~0.01 m/s²

### IMU Noise (Standard Deviation)

| Severity | Accel Noise (m/s²) | Gyro Noise (rad/s) |
|----------|--------------------|--------------------|
| Mild | 0.1 | 0.01 |
| Moderate | 0.3 | 0.03 |
| Severe | 0.6 | 0.06 |
| Critical | 1.0 | 0.1 |

### LiDAR Point Dropout

| Severity | Dropout Rate | Description |
|----------|--------------|-------------|
| Mild | 15% | Light fog/rain |
| Moderate | 35% | Moderate precipitation |
| Severe | 60% | Heavy precipitation |
| Critical | 85% | Near-total occlusion |

**Reference:** Studies show 30-50% dropout can significantly impact object detection

### Camera Confidence Reduction

| Severity | Confidence Reduction | Interpretation |
|----------|---------------------|----------------|
| Mild | 15% | Lighting variation |
| Moderate | 35% | Partial glare/dirt |
| Severe | 60% | Significant occlusion |
| Critical | 85% | Near-complete blindness |

### Localization Drift

| Severity | Drift Rate (m/s) | Noise Std (m) |
|----------|------------------|---------------|
| Mild | 0.02 | 0.05 |
| Moderate | 0.05 | 0.15 |
| Severe | 0.10 | 0.30 |
| Critical | 0.20 | 0.50 |

**Reference:** Lane width ~3.5m; 0.3m drift is ~10% of lane width

## Expected Autoware Response

Based on preliminary testing and literature:

### IMU Faults
- Mild: EKF fusion compensates adequately
- Moderate: Noticeable yaw drift, recoverable
- Severe: Significant trajectory deviation
- Critical: Loss of orientation, emergency stop likely

### LiDAR Faults
- Mild: Object detection slightly degraded
- Moderate: Increased false negatives
- Severe: Major detection failures
- Critical: Effective blindness, collision risk

### Localization Faults
- Mild: Increased tracking error
- Moderate: Occasional lane boundary violations
- Severe: Frequent lane departures
- Critical: Complete loss of localization

## Justification for Severity Ranges

The severity levels are designed to:

1. **Cover the interesting region:** Focus on where baseline fails but intervention helps
2. **Align with real-world scenarios:** Values based on published sensor degradation studies
3. **Enable statistical analysis:** Multiple severity levels allow trend analysis
4. **Support honest evaluation:** Including mild faults shows we don't claim value where there is none

## Experimental Priorities

**High Priority (Essential for contribution):**
- Severe IMU bias/noise
- Severe LiDAR dropout
- Severe localization drift

**Medium Priority (Support claims):**
- Moderate faults (show transition region)
- Critical faults (show graceful degradation)

**Lower Priority (Completeness):**
- Mild faults (verify no over-reaction)
- Camera faults (if time permits)

## Metrics for Each Severity Level

For each (fault_type, severity) combination, measure:

1. **Safety Metrics:**
   - Collision rate (collisions / trials)
   - Lane departure rate
   - Minimum TTC observed
   - Emergency stop rate

2. **Performance Metrics:**
   - Lateral tracking error (mean, max, std)
   - Velocity tracking error
   - Mission completion rate

3. **Detection Metrics:**
   - Time to first residual spike
   - Time to CVaR threshold breach
   - Preemptive trigger rate (trend-based)

## References

- IMU specifications: [Typical automotive IMU datasheets]
- LiDAR degradation: "Impact of Weather on LiDAR Performance" [Various studies]
- Localization accuracy: Autoware documentation, academic benchmarks
