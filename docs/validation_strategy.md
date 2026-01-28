# Validation Strategy

**Last Updated:** 2026-01-23

---

## 1. Validation Phases

### Phase 1: Baseline (No Faults)
- 10 runs on Shinjuku route
- Establish nominal metrics
- Expected: 0 collisions, 100% mission completion

### Phase 2: Fault Injection
- Systematic sweep: IMU, LiDAR, localization
- Severity levels: Mild (10-25%), Moderate (25-50%), Severe (50-75%), Critical (75%+)
- 5 trials per condition
- Find where Autoware fails

### Phase 3: RISE Comparison
- Run with and without RISE on failure scenarios
- Compare collision rate, lane departures, mission time

---

## 2. Fault Severity Reference

| Severity | IMU Bias | LiDAR Dropout | Localization Drift |
|----------|----------|---------------|-------------------|
| Mild | 0.1 m/s² | 15% | 0.02 m/s |
| Moderate | 0.3 m/s² | 35% | 0.05 m/s |
| **Severe** | 0.6 m/s² | 60% | 0.10 m/s |
| Critical | 1.0 m/s² | 85% | 0.20 m/s |

**Primary target:** Severe faults

---

## 3. Metrics

### Safety
- Collision rate (collisions / trials)
- Lane departure rate
- Minimum TTC observed
- Emergency stop rate

### Performance
- Mission completion time
- Tracking error (lateral, longitudinal)
- Average velocity

### Detection
- Time to residual spike
- Tube width at fault onset

---

## 4. Success Criteria

| Criterion | Threshold |
|-----------|-----------|
| Collision rate reduction (severe faults) | p < 0.05 vs baseline |
| Performance degradation | < 20% increase in mission time |
| Coverage calibration | Within 10% of k_σ prediction |

---

## 5. Sample Size

For collision rate comparison (20% baseline → 5% RISE):
- α = 0.05, power = 0.80
- Required: ~25 trials per condition
