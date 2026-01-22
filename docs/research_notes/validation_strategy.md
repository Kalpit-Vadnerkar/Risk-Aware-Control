# Validation Strategy for Risk-Aware Control

**Date:** 2026-01-21
**Purpose:** Define validation methodology before implementation

## Validation Philosophy

**Validation-First Approach:** Establish validation framework BEFORE building the model. Know what success looks like first.

## Phase 1: Baseline Metrics (Ideal Conditions)

**Goal:** Measure Autoware's nominal performance to establish baselines

### Experiments
- 10 runs on standard AWSIM Shinjuku route
- No fault injection
- Record all metrics at 10Hz

### Metrics to Establish
1. **Safety (baseline)**
   - TTC distribution: mean, std, min
   - Obstacle distance: mean, std, min
   - Lane keeping: % time in lane

2. **Performance (baseline)**
   - Lateral tracking error: mean, std, max
   - Velocity tracking error: mean, std
   - Mission completion rate

3. **Comfort (baseline)**
   - Jerk: mean, max
   - Lateral acceleration: mean, max

### Success Criteria
- Baseline collision rate = 0
- Baseline lane departure rate < 1%
- Mission completion rate = 100%

## Phase 2: Fault Injection Experiments

**Goal:** Find where Autoware breaks

### Systematic Fault Sweep
For each sensor (IMU, LiDAR, Camera, Localization):
- For each severity (1, 2, 3, 4):
  - Run 5 trials
  - Inject fault at t=15s for 45s duration
  - Total experiment: 90s

### Metrics to Capture
- Collision (Y/N)
- Lane departure (Y/N, count)
- Emergency stop (Y/N)
- All metrics from Phase 1

### Analysis
1. **Failure Threshold:** At what severity does Autoware start failing?
2. **Critical Sensors:** Which sensor faults are most dangerous?
3. **Interesting Region:** Where does our system have value?

## Phase 3: CVaR System Validation

**Goal:** Validate that RISE improves safety for severe faults

### Comparison Experiments
For each severe fault configuration:
- **Baseline:** No RISE (fixed margins)
- **RISE-Linear:** Linear margin function
- **RISE-Exponential:** Exponential margin function
- **RISE-Preemptive:** With trend-based preemption

### Metrics
1. **Safety Improvement**
   - Δ Collision rate
   - Δ Lane departure rate
   - Δ Minimum TTC

2. **Performance Cost**
   - Δ Mission completion time
   - Δ Average velocity
   - Δ Tracking error

3. **Calibration**
   - CVaR prediction accuracy
   - Empirical coverage rate

## Phase 4: Adversarial NPC Scenarios (Optional)

**Goal:** Test dynamic scenarios beyond sensor faults

### Scenarios
1. **Cut-in:** NPC vehicle sudden lane change
2. **Hard brake:** Lead vehicle emergency stop
3. **Occlusion:** Pedestrian from behind parked car

### Metrics
- Collision avoidance rate
- Minimum TTC during scenario
- Emergency intervention rate

## Statistical Analysis Plan

### Sample Size Justification
For collision rate comparison:
- Expected baseline rate: 20% (severe faults)
- Expected RISE rate: 5%
- α = 0.05, power = 0.80
- Required n ≈ 25 trials per condition

### Statistical Tests
1. **Collision rates:** Chi-squared test or Fisher's exact
2. **Continuous metrics:** Two-sample t-test or Wilcoxon
3. **Time series:** Paired comparisons at matched time points

### Effect Size Reporting
Report Cohen's d or odds ratio for all comparisons.

## Success Criteria

### Primary Success Criteria
1. For severe faults: RISE collision rate < Baseline collision rate (p < 0.05)
2. CVaR calibration error < 10% (empirical vs. theoretical)
3. Preemptive detection rate > 50% (CVaR trend triggers before magnitude)

### Secondary Success Criteria
1. Performance degradation < 20% (mission time, tracking)
2. No increase in emergency stops (demonstrates controlled response)
3. Generalization across fault types

## Data Collection Checklist

### Per-Trial Data
- [ ] ego_state.csv (position, velocity, orientation, covariance)
- [ ] safety_metrics.csv (TTC, obstacle distance, lane status)
- [ ] performance_metrics.csv (tracking errors)
- [ ] comfort_metrics.csv (jerk, acceleration)
- [ ] uncertainty_metrics.csv (localization quality, perception confidence)
- [ ] fault_log.json (injection timing, parameters)
- [ ] metadata.json (experiment config, trial info)

### Per-Condition Summary
- [ ] Mean and std of all metrics
- [ ] Collision/lane departure/emergency counts
- [ ] Timing analysis (detection latency)

### Aggregate Analysis
- [ ] Comparison tables (baseline vs. RISE)
- [ ] Statistical test results
- [ ] Visualization plots

## Risk Mitigation

### If baseline never fails:
- Increase fault severity
- Add combined fault scenarios
- Consider scenario-based testing (NPCs)

### If RISE doesn't help:
- Verify CVaR computation is correct
- Check margin function scaling
- Investigate detection latency

### If RISE helps too much (suspiciously):
- Verify baseline implementation
- Check for data leakage in fault injection
- Ensure apples-to-apples comparison

## Timeline

| Phase | Duration | Deliverable |
|-------|----------|-------------|
| 1. Baseline | 1 week | Baseline metrics report |
| 2. Fault Sweep | 2 weeks | Failure thresholds, severity taxonomy |
| 3. RISE Validation | 2-3 weeks | Comparison results |
| 4. NPC Scenarios | 1-2 weeks | (Optional) Dynamic scenario results |
| Analysis | 1 week | Final statistical analysis |

## References

- Power analysis: G*Power software
- Statistical methods: [Standard hypothesis testing references]
- Calibration assessment: [Kuleshov et al., 2018]
