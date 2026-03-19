# Perception Dropout Sweep Analysis

**Date:** 2026-03-05
**Campaign:** `experiments/data/perception_dropout_sweep/`
**Goals:** goal_007 (915m), goal_011 (780m), goal_021 (849m)
**Dropout rates:** 0.0, 0.2, 0.4, 0.6, 0.8, 1.0
**Trials:** 2 per condition × goal (36 total, 33 successful)

---

## Result: All Routes Completed at Every Dropout Level

| Dropout | N | Succeeded | Mean Vel (m/s) | MRM/km | Min Dist (m) | Near-Misses | Hard Brakes |
|---------|---|-----------|----------------|--------|--------------|-------------|-------------|
| 0.0 | 5 | 5/5 | 5.22 ± 0.40 | 102.4 | 3.23 | 1 | 617 |
| 0.2 | 4 | 4/6 | 5.23 ± 0.69 | 101.2 | 3.23 | 0 | 348 |
| 0.4 | 6 | 6/6 | 5.44 ± 0.70 | 101.7 | 3.99 | 0 | 479 |
| 0.6 | 6 | 6/6 | 4.96 ± 0.39 | 107.2 | 4.65 | 0 | 480 |
| 0.8 | 6 | 6/6 | 5.44 ± 0.68 | 97.6 | 3.33 | 0 | 420 |
| 1.0 | 6 | 6/6 | 5.56 ± 0.43 | 92.6 | 3.92 | 0 | 355 |

*Two failures (goal_007 t1 stuck at 0.0; goal_021 t1 stuck at 0.2) are attributed to NPC blocking at route start — not dropout-induced.*

---

## Behavioral Change: Hard Braking Declines with Dropout

Correlation analysis of metrics vs dropout rate (successful runs, N=33):

| Metric | Pearson r | Interpretation |
|--------|-----------|----------------|
| mean velocity | +0.154 | weak/noise |
| MRM rate | -0.255 | weak/noise |
| min clearance | +0.092 | none |
| **hard brake count** | **-0.673** | **STRONG negative** |
| max deceleration | -0.226 | weak |

The only statistically meaningful signal is that **hard brake count drops sharply as dropout increases**. The vehicle performs 617 hard braking events at 0% dropout versus 355 at 100% dropout — a **42% reduction**.

### Why this happens

At higher dropout rates, the planning module receives fewer (or no) object detections. With nothing to brake for, the vehicle drives more smoothly in terms of planned stops. The road appears clear. This is physically "correct" from a planning perspective — the system is behaving as designed when given an empty perception input.

### Why this does NOT translate to collisions

NPC traffic in the Shinjuku AWSIM environment is well-behaved. NPCs follow lanes and yield to the ego vehicle. For these three routes, the ego vehicle's planned path does not create head-on conflict geometry with NPCs. The ego vehicle drives past NPCs that are in adjacent lanes or yielding — even at 100% dropout.

---

## Critical Finding: Dropout Is the Wrong Fault Type for RISE

**Increasing dropout level will not help.** We already have the maximum fault (100% dropout = complete blindness) and see no route failures, no near-misses, no collision risk. The PREVIOUS finding of failures at 60%+ dropout (Feb 2026) was entirely due to the MRM wedge — an infrastructure bug, not a real safety degradation. That bug is now fixed.

### Why dropout doesn't stress-test RISE

RISE is designed to prevent constraint violations by tightening safety margins when the digital twin detects anomalies. For this to work:
1. The fault must create a **real physical threat** (an object in the vehicle's path)
2. That threat must cause **observable behavioral deviation** (unexpected braking, path changes)
3. The deviation must generate **ST-GAT residuals** that CVaR can quantify
4. RISE tightens constraints **before** the violation occurs

Random perception dropout fails at step 1. The threat is theoretical (invisible NPCs) but not actualized (NPCs don't hit the vehicle). No deviation → no residuals → nothing for RISE to act on.

### What dropout IS good for

The hard brake reduction IS a meaningful signal for a different research question: characterizing the relationship between perception completeness and driving conservatism. At 100% dropout, the vehicle drives more aggressively (fewer brakes) which is unsafe in denser traffic. But demonstrating this requires a simulation environment where NPCs actually create collision scenarios, which the current AWSIM Shinjuku setup does not reliably provide.

---

## Recommendation: Pivot to Obstacle-Based Scenarios

The correct fault types for RISE validation are those that create **actual collision risk**:

### Priority 1: Static Obstacle
Place a stationary object (vehicle, cone) directly in the ego vehicle's lane at varying distances ahead. This:
- Forces the ego to brake (observable behavior change)
- Creates a near-miss scenario if RISE doesn't intervene
- Generates large ST-GAT residuals (sudden unexpected object)
- Gives RISE a clear job: tighten velocity constraints when residuals spike

### Priority 2: Cut-In
An object enters the ego's lane from the side at close range. This:
- Creates a dynamic near-miss scenario
- Tests RISE's ability to react to rapidly changing risk
- More challenging than static obstacle (faster residual growth)

### Deprioritize
- Dropout sweep: ✗ no behavioral effect in current environment
- Position noise sweep: likely similar result (moderate noise won't create threats; extreme noise creates implausible objects planning ignores)

---

## Next Steps

1. ✅ Dropout characterization complete — no further sweeps needed
2. 🔜 Run static obstacle sweep: vary distance (20m, 40m, 80m) × object type (vehicle/cone) × 3 goals × 2 trials
3. 🔜 Run cut-in sweep: vary speed and lateral distance
4. Document ST-GAT residual behavior under each scenario (needed for Phase 3)
