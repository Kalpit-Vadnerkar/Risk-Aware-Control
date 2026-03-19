# Perception Fault Sweep Results Report

**Date:** February 19, 2026
**Author:** Kalpit Vadnerkar
**Advisor:** Dr. Pierluigi Pisu

## Executive Summary

We completed two overnight fault injection sweeps using our PerceptionInterceptor framework: **perception dropout** (36 experiments) and **position noise** (54 experiments). Both sweeps revealed meaningful degradation curves AND exposed a critical infrastructure problem — an **MRM recovery wedge** — that invalidates a portion of the data and must be fixed before further sweeps.

**Key findings:**
1. Autoware tolerates perception dropout up to 40% and position noise up to 0.75m std before failing
2. The failure mode is NOT graceful degradation — it is a sudden system wedge after an unrecoverable MRM event
3. Baseline MRM rates vary significantly across sessions (28–160 MRM/km), indicating high sensitivity to NPC traffic
4. The interceptor passthrough is validated — route distances are identical to non-interceptor runs

---

## 1. Experimental Setup

### Architecture
A PerceptionInterceptor ROS2 node sits between Autoware's perception pipeline and planning input:

```
AWSIM → Autoware Perception → /perception/.../objects
                                      ↓
                            PerceptionInterceptor
                            (inject faults / passthrough)
                                      ↓
                        /perception/.../objects_filtered → Planning
```

All experiments, including baselines, route through the interceptor in passthrough mode to ensure fair comparison.

### Goals Used
| Goal ID | Route Distance | Notes |
|---------|---------------|-------|
| goal_007 | 915 m | Primary benchmark |
| goal_011 | 780 m | Different road topology |
| goal_021 | 849 m | High reliability baseline |

### Trials
- 2 trials per (condition × goal) combination
- Vehicle reset between experiments

---

## 2. Perception Dropout Sweep

**Fault type:** Randomly drops each detected object with probability `dropout_rate`
**Sweep values:** 0.0, 0.2, 0.4, 0.6, 0.8, 1.0
**Total experiments:** 36 (3 goals × 6 conditions × 2 trials)

### Results Summary

| Dropout Rate | Success Rate | Avg MRM Rate (/km) | Avg Completion Time (s) | Status |
|-------------|-------------|---------------------|------------------------|--------|
| 0.0 | 6/6 (100%) | 129.5 | 170 | Valid |
| 0.2 | 6/6 (100%) | 152.1 | 171 | Valid |
| 0.4 | 5/6 (83%) | 136.8* | 164* | Valid (1 MRM failure) |
| 0.6 | 0/6 (0%) | — | — | **INVALID (wedged)** |
| 0.8 | 0/6 (0%) | — | — | **INVALID (wedged)** |
| 1.0 | 0/6 (0%) | — | — | **INVALID (wedged)** |

*Averages for successful runs only

### What Happened
At dropout_rate=0.4 (goal_021, trial 2), an MRM event triggered and reached `MRM_SUCCEEDED` state. The vehicle stopped safely but the system could not be recovered — the vehicle reset script failed to bring Autoware back to `DRIVING` state. **All 18 subsequent experiments** (dropout 0.6, 0.8, 1.0) failed immediately with `goal_setting_failed` and `driving_time: 0.0`. This means we have **no valid data** for dropout rates above 0.4.

### Valid Findings (dropout 0.0–0.4)
- Autoware is **remarkably tolerant** of up to 40% random object dropout
- MRM rates do not increase significantly with dropout rate (they are dominated by other factors)
- Route completion times are stable across 0.0–0.4 dropout range
- E-stop ratios remain consistently high (~0.95), suggesting most MRM events are emergency stops regardless of dropout level

---

## 3. Position Noise Sweep

**Fault type:** Adds Gaussian noise (σ = `noise_std` meters) to each object's x/y position
**Sweep values:** 0.0, 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 4.0
**Total experiments:** 54 (3 goals × 9 conditions × 2 trials)

### Results Summary

| Noise σ (m) | Success Rate | Avg MRM Rate (/km) | Avg E-stop Ratio | Avg Hard Brakes | Status |
|-------------|-------------|---------------------|-------------------|-----------------|--------|
| 0.0 | 6/6 (100%) | 83.5 | 0.19 | 100 | Valid |
| 0.1 | 5/6 (83%) | 90.7 | 0.18 | 63 | Valid |
| 0.25 | 6/6 (100%) | 100.1 | 0.24 | 86 | Valid |
| 0.5 | 5/6 (83%) | 93.5 | 0.25 | 82 | Valid |
| 0.75 | 6/6 (100%) | 92.9 | 0.26 | 78 | Valid |
| 1.0 | 1/6 (17%) | 96.2† | 0.17† | 69† | Partially valid |
| 1.5 | 0/6 (0%) | — | — | — | **INVALID (wedged)** |
| 2.0 | 0/6 (0%) | — | — | — | **INVALID (wedged)** |
| 4.0 | 0/6 (0%) | — | — | — | **INVALID (wedged)** |

†Based on single successful run (goal_007 t1 only)

### Key Observations

**Recovery was intermittent.** Unlike the dropout sweep where a single wedge poisoned everything after it, the noise sweep showed **partial recovery**:
- noise_std=0.1: goal_021 t2 had MRM_SUCCEEDED → but noise_std=0.25 experiments **recovered and succeeded**
- noise_std=0.5: goal_011 t2 had MRM_SUCCEEDED (COMFORTABLE_STOP) → noise_std=0.75 **still recovered**
- noise_std=1.0: goal_007 t2 had MRM_SUCCEEDED (EMERGENCY_STOP) → noise_std=1.5 onward **permanently wedged**

This tells us that recovery from MRM_SUCCEEDED is **non-deterministic** — sometimes the vehicle reset works, sometimes it doesn't. The permanent wedge at noise_std ≥ 1.5 was likely caused by the noise_std=1.0 t2 MRM.

**Autoware is surprisingly resilient to position noise.** Up through noise_std=0.75m (where bounding boxes could be displaced by ~2.25m at 3σ), all 6/6 experiments succeeded. The planning module appears to use sufficient safety margins that moderate position errors don't cause failures.

**MRM rates don't increase linearly with noise.** The MRM rate stays in the 83–100/km range across all noise levels, suggesting that the MRM triggers are dominated by NPC traffic interactions rather than perception noise.

---

## 4. Baseline Comparison: Interceptor Transparency

Comparing goal_007 across three baseline conditions:

| Condition | Date | Route (m) | Reached | MRM Count | MRM Rate (/km) | Mean Vel (m/s) |
|-----------|------|-----------|---------|-----------|-----------------|----------------|
| Nominal (no interceptor) | Feb 5 | 914.95 | Yes | 147 | 160.7 | 5.49 |
| Interceptor passthrough | Feb 17 | 915.04 | Yes | 26 | 28.4 | 5.05 |
| Noise sweep σ=0.0 t1 | Feb 19 | 915.12 | Yes | 91 | 99.4 | 4.90 |
| Noise sweep σ=0.0 t2 | Feb 19 | 915.19 | Yes | 77 | 84.1 | 5.20 |
| Dropout sweep rate=0.0 t1 | Feb 18 | 915.13 | Yes | 131 | 143.2 | 4.51 |
| Dropout sweep rate=0.0 t2 | Feb 18 | 914.96 | Yes | 115 | 125.7 | 5.36 |

### Observations

1. **Route distance is identical** (914.9–915.2m) across all conditions, confirming the interceptor does not alter the vehicle's path planning.

2. **MRM rates vary enormously** (28–161/km) across sessions with identical fault conditions (passthrough/0.0). This 6x variation is driven by **NPC traffic** — the AWSIM environment has randomized traffic that independently triggers MRM events.

3. **This variance is a significant confounding factor.** The MRM rate variation across baseline sessions is *larger* than the variation caused by our fault injection (dropout 0.0 vs 0.4, or noise 0.0 vs 0.75). This means:
   - We cannot use MRM rate as a reliable discriminator between mild fault conditions
   - We need either (a) many more trials per condition, or (b) a way to control/normalize for NPC traffic
   - Safety-critical metrics (min clearance, near-miss count) may be more reliable discriminators

---

## 5. Critical Problem: MRM Recovery Wedge

### The Problem
After an MRM event reaches `MRM_SUCCEEDED` state, Autoware sometimes cannot be recovered to accept new route goals. The `reset_vehicle.py` script attempts:
1. Cancel current route
2. Wait for state transition
3. Set new goal

When recovery fails, the system is stuck in a state where new goals are rejected (`goal_setting_failed`). All subsequent experiments in the sweep fail with `driving_time: 0.0`.

### Impact on Data
| Sweep | Total Experiments | Valid | Invalid (wedged) | Data Loss |
|-------|-------------------|-------|-------------------|-----------|
| Dropout | 36 | 17 | 19 | **53%** |
| Position Noise | 54 | 29 | 25 | **46%** |
| **Total** | **90** | **46** | **44** | **49%** |

We lost nearly **half** of all overnight experiment time (~5 hours) to this issue.

### Root Cause Analysis
The MRM (Minimum Risk Maneuver) system has three states:
- `MRM_OPERATING` — vehicle is executing emergency stop
- `MRM_SUCCEEDED` — vehicle has stopped safely
- `NORMAL` — vehicle is operating normally

The transition `MRM_SUCCEEDED → NORMAL` requires clearing the MRM trigger condition and publishing appropriate state transitions. Our vehicle reset script does not reliably achieve this, particularly after `EMERGENCY_STOP` behaviors (vs. `COMFORTABLE_STOP` which seems more recoverable based on the noise sweep data).

### Proposed Solutions

**Short-term (for next sweep):**
1. **Detect-and-restart:** After each experiment, check if the system is in a wedged state (consecutive `goal_setting_failed`). If so, restart Autoware entirely (kill and relaunch the stack) before continuing.
2. **Skip-and-flag:** When `goal_setting_failed` occurs, skip the experiment, mark it as `infra_failure` (distinct from `fault_failure`), and attempt a full restart before the next experiment.

**Medium-term:**
3. **Deeper MRM recovery:** Investigate Autoware's `/system/mrm/` topics and `/autoware/state` transitions to find the correct recovery sequence. May need to publish to `/system/mrm/mrm_operation/cancel` or similar.
4. **AWSIM-side reset:** Instead of just resetting Autoware state, also teleport the vehicle back to the start position in AWSIM, which may help clear stale sensor data.

**Long-term:**
5. **Full stack restart per experiment:** Launch/kill the entire Autoware stack for each experiment. Slower (~2 min overhead per run) but guarantees clean state.

---

## 6. Preliminary Degradation Curves

Based on valid data only:

### Perception Dropout — Goal Completion vs Dropout Rate
```
Success Rate (%)
100 |  ████  ████  ████
 80 |                    ████
 60 |
 40 |
 20 |
  0 |                          ?     ?     ?
    +------+------+------+------+------+------
    0.0   0.2   0.4   0.6   0.8   1.0
                dropout_rate

(? = no valid data due to wedge)
```

**Interpretation:** Autoware tolerates up to 40% dropout without significant impact. The true failure threshold is somewhere between 0.4 and 0.6 — we need to re-run with wedge recovery to determine this.

### Position Noise — Goal Completion vs Noise Std
```
Success Rate (%)
100 |  ████        ████        ████
 80 |        ████        ████
 60 |
 40 |
 20 |                                ████
  0 |                                      ?     ?     ?
    +------+------+------+------+------+------+------+------+------
    0.0   0.1   0.25  0.5   0.75  1.0   1.5   2.0   4.0
                noise_std (meters)

(? = no valid data due to wedge)
```

**Interpretation:** Autoware tolerates position noise up to σ=0.75m (objects displaced by up to ~2.25m at 3σ). Failure begins at σ=1.0m. We need data at 1.0–1.5m range to find the exact threshold.

---

## 7. Implications for RISE Framework

### What We Learned for Risk Assessment Design

1. **Failure modes are abrupt, not gradual.** Autoware doesn't slowly degrade — it either completes the route or hits an unrecoverable MRM. This supports the use of **threshold-based** risk assessment (CVaR exceeding critical value → immediate constraint tightening) rather than proportional adjustment.

2. **MRM rate is too noisy to be a useful risk indicator.** The 6x baseline variation means we cannot reliably distinguish "normal operation with MRM events" from "fault-induced degradation" using MRM count alone. RISE should use **perception-level residuals** (the ST-GAT output) rather than system-level state.

3. **Safety margins exist but have limits.** Autoware's planning module has enough built-in margin to handle ±0.75m position errors and 40% object dropout. RISE's constraint tightening should activate *before* these thresholds are reached, providing early warning.

4. **The failure threshold defines our validation target.** RISE must demonstrate that it can detect and react to degradation *before* the system reaches the cliff-edge at dropout=0.4–0.6 or noise σ=0.75–1.0m.

### Next Steps

1. **Fix the MRM wedge** (highest priority) — implement detect-and-restart to prevent data loss
2. **Re-run dropout sweep** at 0.4–0.7 with finer granularity (0.05 increments) to find exact threshold
3. **Re-run noise sweep** at 0.75–1.5m with finer granularity (0.1m increments)
4. **Run remaining sweeps:** perception delay (11 conditions), static obstacle, cut-in scenarios
5. **Collect full nominal baseline** (25 goals × 2 trials) for ST-GAT training data
6. **Address NPC traffic confound** — either increase trials (≥5 per condition) or investigate AWSIM traffic seed control

---

## Appendix A: Detailed Metrics by Condition

### A.1 Position Noise — Goal 007 (915m route)

| σ (m) | Trial | Reached | Comp Time (s) | Mean Vel (m/s) | MRM Count | MRM Rate | E-stop Ratio | Min Dist (m) | Hard Brakes |
|-------|-------|---------|---------------|----------------|-----------|----------|--------------|-------------|-------------|
| 0.0 | t1 | Yes | 191 | 4.90 | 91 | 99.4 | 0.154 | 2.93 | 132 |
| 0.0 | t2 | Yes | 175 | 5.20 | 77 | 84.1 | 0.169 | 2.87 | 122 |
| 0.1 | t1 | Yes | 201 | 4.54 | 94 | 102.7 | 0.128 | 2.75 | 80 |
| 0.1 | t2 | Yes | 163 | 5.59 | 86 | 94.0 | 0.256 | 6.28 | 69 |
| 0.25 | t1 | Yes | 241 | 3.78 | 128 | 139.8 | 0.234 | 2.80 | 150 |
| 0.25 | t2 | Yes | 213 | 4.30 | 102 | 111.4 | 0.186 | 3.03 | 111 |
| 0.5 | t1 | Yes | 195 | 4.66 | 91 | 99.4 | 0.198 | 2.83 | 116 |
| 0.5 | t2 | Yes | 161 | 5.66 | 76 | 83.1 | 0.224 | 2.65 | 77 |
| 0.75 | t1 | Yes | 190 | 4.76 | 96 | 104.9 | 0.313 | 3.80 | 98 |
| 0.75 | t2 | Yes | 161 | 5.68 | 76 | 83.1 | 0.303 | 5.43 | 97 |
| 1.0 | t1 | Yes | 186 | 4.90 | 88 | 96.2 | 0.170 | 3.82 | 69 |
| 1.0 | t2 | **No** | — | — | — | — | — | — | — |

### A.2 Perception Dropout — All Goals Aggregated (successful runs)

| Rate | Runs | Avg Comp Time (s) | Avg Mean Vel (m/s) | Avg MRM Rate (/km) | Avg E-stop Ratio | Avg Min Dist (m) |
|------|------|-------------------|---------------------|---------------------|-------------------|-------------------|
| 0.0 | 6/6 | 169.6 | 5.06 | 129.5 | 0.962 | 3.82 |
| 0.2 | 6/6 | 171.2 | 4.91 | 152.1 | 0.957 | 3.56 |
| 0.4 | 5/6 | 163.7 | 5.27 | 133.1 | 0.973 | 2.95 |

### A.3 Baseline Comparison (goal_007 only, all sessions)

| Session | Date | MRM Count | MRM Rate (/km) | E-stop Ratio | Mean Vel (m/s) | Hard Brakes |
|---------|------|-----------|-----------------|--------------|----------------|-------------|
| Nominal (pre-interceptor) | Feb 5 | 147 | 160.7 | 0.789 | 5.49 | 52 |
| Interceptor passthrough | Feb 17 | 26 | 28.4 | 0.808 | 5.05 | 181 |
| Dropout sweep σ=0.0 | Feb 18 | 123 avg | 134.4 | 0.965 | 4.94 | 102 avg |
| Noise sweep σ=0.0 | Feb 19 | 84 avg | 91.8 | 0.162 | 5.05 | 127 avg |

---

## Appendix B: Raw Data Locations

```
experiments/data/
├── nominal/                        # 25 baseline goals (Feb 5 + Feb 17)
├── perception_dropout_sweep/       # 36 experiments (Feb 18)
│   └── sweep_*_summary.txt        # Human-readable summary
├── position_noise_sweep/           # 54 experiments (Feb 19)
│   └── sweep_*_summary.txt        # Human-readable summary
└── test_runs/                      # Ad-hoc validation runs
```

Each experiment directory contains:
- `metadata.json` — experiment configuration, scenario params, campaign
- `result.json` — status, goal_reached, driving_time, MRM count
- `metrics.json` — full metrics (safety, reliability, fail-operational, comfort)
- `autoware.log` — Autoware console output (when available)
