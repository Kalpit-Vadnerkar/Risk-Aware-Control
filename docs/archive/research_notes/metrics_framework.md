# Metrics Framework for RISE Validation

**Last Updated:** 2026-02-05
**Purpose:** Define metrics to evaluate AV safety, reliability, and fail-operational capability

---

## Issues with Initial Metrics (2026-02-05)

### Problem: Lateral Error Computation Was Wrong

Our initial "lateral_rmse" values were nonsensical (e.g., 47m for some runs, 0.1m for others).

**Root Cause:** We computed distance to closest trajectory point, but:
1. The trajectory is in vehicle frame (future path), not a reference line
2. We should use Autoware's built-in `control_performance_analysis` module

**Solution:** Use `/control/control_performance/error` topic which publishes:
- `lateral_error` - proper cross-track error
- `longitudinal_error` - along-track error
- `heading_error` - orientation error

### Problem: Metrics Not Normalized

MRM count of 239 vs 67 doesn't mean much if routes are different lengths.

**Solution:** Normalize by distance traveled:
- `mrm_rate = mrm_count / distance_km`
- `near_miss_rate = near_miss_count / distance_km`

### Problem: Ratio > 1 Is Confusing

Time ratio of 2.87 meaning "took 3x longer" is counterintuitive.

**Solution:** Use efficiency = expected/actual (so 1.0 = optimal, <1.0 = slower)

---

## Revised Metric Categories

### Tier 1: Safety Metrics (Primary)

These directly measure what we're trying to improve with risk-aware control.

| Metric | Formula | Unit | Source | Good Direction |
|--------|---------|------|--------|----------------|
| **Collision Count** | Objects within collision_threshold | count | Object tracking | ↓ 0 |
| **Near Miss Rate** | TTC < 2s events / distance_km | events/km | TTC computation | ↓ Lower |
| **Critical TTC Rate** | TTC < 1s events / distance_km | events/km | TTC computation | ↓ Lower |
| **Min Clearance (P5)** | 5th percentile of object distances | meters | Object tracking | ↑ Higher |
| **Lane Departure Count** | From lane_departure_checker diagnostic | count | /diagnostics | ↓ 0 |

**Why P5 instead of absolute minimum:**
Absolute minimum can be noisy (single frame artifacts). P5 (5th percentile) gives a more robust measure of "typical worst case."

### Tier 2: MRM/Intervention Metrics (Key for Research)

These measure safety system interventions - directly relevant to our goal of preemptive constraint tightening.

| Metric | Formula | Unit | Interpretation |
|--------|---------|------|----------------|
| **MRM Rate** | mrm_count / distance_km | events/km | Overall intervention frequency |
| **Emergency Stop Rate** | emergency_stops / distance_km | events/km | Severe interventions |
| **Comfortable Stop Rate** | comfortable_stops / distance_km | events/km | Mild interventions |
| **MRM Time %** | mrm_duration / driving_time × 100 | % | Time in degraded mode |
| **Recovery Rate** | recoveries / mrm_count | ratio | System resilience |
| **Avg MRM Duration** | mrm_duration / mrm_count | seconds | Recovery speed |
| **E-Stop Ratio** | emergency / (emergency + comfortable) | ratio | Severity distribution |

**Interpretation Guide:**
- High MRM rate + high recovery rate = conservative but functional
- High MRM rate + low recovery rate = system struggling (like goal_017)
- Low MRM rate = either very stable OR not detecting real risks

### Tier 3: Reliability Metrics

Mission completion and efficiency.

| Metric | Formula | Unit | Interpretation |
|--------|---------|------|----------------|
| **Mission Success Rate** | successful / total | % | Basic reliability |
| **Time Efficiency** | expected_time / actual_time | ratio | 1.0 = optimal, <1.0 = slower |
| **Stuck Rate** | stuck_runs / total | % | Deadlock frequency |

### Tier 4: Control Quality Metrics

From Autoware's `control_performance_analysis` module.

| Metric | Source | Unit | Notes |
|--------|--------|------|-------|
| **Lateral Error (RMS)** | `/control/control_performance/error` | meters | Cross-track error |
| **Heading Error (RMS)** | `/control/control_performance/error` | radians | Orientation tracking |
| **Longitudinal Error (RMS)** | `/control/control_performance/error` | meters | Speed/position tracking |

**Recording Requirement:** Add this topic to RECORDING_TOPICS in config.py

### Tier 5: Comfort Metrics (Secondary)

| Metric | Formula | Unit |
|--------|---------|------|
| **Hard Brake Rate** | hard_brakes / distance_km | events/km |
| **Max Jerk** | max(d²v/dt²) | m/s³ |
| **Max Lateral Accel** | max(v² × curvature) | m/s² |

---

## Baseline Results Analysis (2026-02-05)

### Summary Statistics

From 25 experiments with updated goals:
- **Success Rate:** 24/25 (96%)
- **Total Distance:** 16.8 km
- **Average MRM Rate:** ~210 MRM/km (very high!)
- **Average MRM Duration:** 116ms (fast recovery)
- **MRM Time:** ~15% of driving time

### Goal 017 Failure Analysis

```
goal_reached: false
distance: 552m
mrm_count: 239 → 433 MRM/km (extremely high)
emergency_stops: 224 (94% of MRMs)
comfortable_stops: 15 (6%)
recovery_rate: 93.3% (some didn't recover)
time_ratio: 2.87 (3x slower than expected)
```

**Failure Mode:** Very high MRM frequency with predominantly emergency stops. Eventually some MRMs don't recover, leading to stuck state.

**Research Implication:** This is exactly the scenario risk-aware control should help - detecting escalating risk BEFORE the cascade of MRMs.

### Runs with Anomalous Lateral Error

Goals 004, 009, 010, 011, 016, 021, 024 showed "lateral_error" of 30-50m (impossible values).

**Root Cause:** Our computation was wrong. These runs likely had:
1. Trajectory in different coordinate frame
2. Long look-ahead trajectories with vehicle far from start
3. Trajectory resets/jumps during MRM

**Fix:** Use Autoware's built-in control_performance_analysis topic.

---

## Topics to Record

Update `experiments/lib/config.py`:

```python
RECORDING_TOPICS = [
    # ... existing topics ...

    # Control performance (for proper tracking error)
    '/control/control_performance/error',

    # Lane departure checker diagnostics
    '/control/lane_departure_checker/debug/processing_time_ms_diag',
]
```

---

## Revised Summary Table Format

```
Goal       Status   Dist(km)  Effic  NearMiss/km  MRM/km  MRM%  E-Stop%  MinClr(P5)
goal_001   SUCCESS    0.153   0.57        0.0      157    2.0%    92%       5.6m
goal_002   SUCCESS    0.346   0.61        2.9      197    3.2%    88%       3.1m
goal_017   FAILED     0.552   0.35        0.0      433   10.3%    94%       3.9m
...

Legend:
  Dist(km)   - Distance traveled in kilometers
  Effic      - Time efficiency (expected/actual, 1.0 = optimal)
  NearMiss/km- TTC < 2s events per kilometer
  MRM/km     - MRM triggers per kilometer
  MRM%       - Percentage of driving time in MRM state
  E-Stop%    - Emergency stops as % of all MRMs
  MinClr(P5) - 5th percentile minimum clearance (meters)
```

---

## Key Metrics for Research Story

### Baseline Characterization
1. **MRM Rate (per km)** - How often does the system trigger safety stops?
2. **Recovery Rate** - When MRM triggers, does it recover automatically?
3. **E-Stop vs Comfortable Ratio** - Are interventions severe or mild?

### Risk-Aware Control Effectiveness
1. **Near Miss Rate reduction** - Do we have fewer close calls?
2. **MRM Rate reduction** - Do preemptive constraints reduce reactive stops?
3. **E-Stop Ratio reduction** - Are remaining interventions less severe?

### The Story We Want to Tell

> "Baseline Autoware triggers MRM 200+ times per km, with 90%+ being emergency stops.
> Our risk-aware control system, using CVaR computed from ST-GAT residuals,
> reduces MRM rate by X% and shifts interventions from emergency to comfortable stops,
> while maintaining safety margins."

---

## Implementation Checklist

- [x] MRM counting (emergency vs comfortable)
- [x] TTC computation
- [x] Object distance tracking
- [ ] **Fix lateral error - use control_performance_analysis**
- [ ] **Normalize all counts by distance (per km)**
- [ ] **Add lane departure detection from diagnostics**
- [ ] **Use percentile-based clearance (P5)**
- [ ] Add control_performance/error to recording topics
- [ ] Re-run experiments with updated recording

---

## Comparison with Prior Work (T-ITS Paper)

The T-ITS paper used these metrics for fault detection:
- Detection accuracy (93.7%)
- Processing time (1.13ms)
- Fault detection latency

For RISE, we need metrics that show:
1. **Safety improvement** (not just detection)
2. **Operational impact** (does it help complete missions?)
3. **Intervention quality** (preemptive vs reactive)

The MRM rate and near-miss rate are the key metrics that connect detection to action.
