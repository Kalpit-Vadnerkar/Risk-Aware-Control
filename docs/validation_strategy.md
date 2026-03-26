# Validation Strategy

**Last Updated:** 2026-03-20

---

## 1. Validation Phases

### Phase 1: Nominal Baseline ✅ COMPLETE (Mar 2026)

Run routes without any fault injection or scenario injection. Establish the floor for
safety and reliability metrics.

**Results:** 14/15 goal runs succeed (93%). MRM rate 124 ± 40 /km (transient, < 0.15s
duration). Zero near-misses. Mean velocity 4.46 m/s. See:
`docs/research_notes/nominal_baseline_analysis_mar2026.md`

### Phase 2: Scenario Baseline + Fault Characterization (Current)

For each risk scenario (static obstacle, cut-in), run:
1. **Scenario only** — characterize Autoware's natural handling, find distances where
   Autoware is stressed but not blocked
2. **Fault + scenario** — apply perception faults on top; find combinations that produce
   near-misses (min clearance < 2m) or MRM events

**Scenarios under test:**

| Scenario | Parameters | Goal |
|----------|-----------|------|
| Static obstacle | 20m, 30m in-lane | Find blocking threshold |
| Cut-in | 40m ahead, 2s / 4s duration | Find tracking failure conditions |
| Fault overlays | Dropout 30%, position noise 0.5m | Degrade Autoware handling |

**See:** `docs/research_notes/rise_experiment_plan_mar2026.md` for full scenario rationale.
**See:** `docs/research_notes/scenario_framework.md` for PerceptionInterceptor implementation.

### Phase 3: RISE Comparison

Re-run Phase 2 fault+scenario combinations with RISE active. RISE monitors ST-GAT
residuals in real time and adjusts constraints when the residual signal indicates elevated
risk.

**Comparison metrics:**
- min_object_distance and collision proxy count (fault+scenario vs. fault+scenario+RISE)
- MRM_SUCCEEDED rate
- Route completion time (graceful degradation check — mission should still complete)

---

## 2. What Constitutes a Useful Test Scenario

For a scenario to be a valid RISE test case, it must:

1. Create a **real threat** (object in path at a distance requiring response)
2. Produce **observable behavioral deviation** (braking, velocity change)
3. Generate **measurable residuals** in the ST-GAT signal
4. Have a **fault condition that degrades response** (fault makes it worse)
5. Have a **constraint RISE can tighten** (earlier deceleration prevents violation)

The Feb 2026 perception dropout sweep failed criterion 1 (NPCs yield to ego; no real
collision threat created). Static obstacles at 20–30m with fault overlays are the
correct fault scenario design.

---

## 3. Metrics

### Safety (Primary)
- `min_object_distance` / collision proxy count (distance < 2m to injected object)
- `min_closing_time` (longitudinal distance / ego speed for static objects)
- MRM rate and duration

### Reliability
- Route completion rate
- Mission time (relative to nominal — should not increase excessively)
- Mean velocity

### RISE-Specific
- Time from residual spike to constraint tightening (detection latency)
- Velocity at closest-approach event (did RISE slow vehicle early enough?)
- False positive tightening rate on nominal runs

---

## 4. Success Criteria

| Criterion | Threshold |
|-----------|-----------|
| Collision proxy reduction (fault+scenario) | Statistically significant vs. no-RISE |
| Route completion rate with RISE | ≥ baseline (no degradation in mission success) |
| False positive rate on nominal runs | < 5% of timesteps with unnecessary tightening |

---

## 5. Fault Types in Scope

All faults are injected at the **perception layer** via PerceptionInterceptor.
Sensor-level faults (raw LiDAR, IMU) are out of scope — we test degraded perception
signals, not raw sensor hardware failures.

| Fault | Implementation | Severity Range |
|-------|----------------|----------------|
| Object detection dropout | Drop N% of objects randomly | 10%, 30%, 50% |
| Position noise | Add Gaussian noise to object positions | σ = 0.25m, 0.5m, 1m |
| Object delay | (planned) Buffer objects by Nms | 100ms, 200ms |

These faults are realistic proxies for degraded object detection conditions (weather,
occlusion, sensor noise) without requiring AWSIM source modification.
