# Q1 Analysis: ST-GAT Residuals vs Constraint Violation
**Date**: 2026-06-24  
**Status**: COMPLETE — key findings documented

## Context

RQ1: *Can CVaR computed from digital twin prediction residuals provide valid probabilistic bounds on constraint violation?*

Model trained: ST-GAT RISE edition, 486,712 parameters, 10 features (8 original + 2 EKF uncertainty).
Training: 130 epochs (early stopping), best val NLL = -513.7 at epoch 110.
Inference run on: 18 obs_recovery runs + 18 obs_noescape runs.

---

## Key Findings

### Finding 1: Max combined NLL separates avoidance from stuck (p=0.034)

Per-run MAX of combined NLL (position + velocity + steering):

| Dataset | Max NLL mean ± std |
|---|---|
| obs_recovery (avoidance) | -2.82 ± 6.50 |
| obs_noescape (stuck) | -6.65 ± 1.85 |

Mann-Whitney U one-sided (recovery > noescape): **p=0.034** — statistically significant.

Interpretation: Avoidance maneuvers (lane changes) in obs_recovery produce large NLL spikes because the model was trained on nominal lane-following. The model is *surprised* by lateral departures. Stuck scenarios don't involve anomalous maneuvers.

### Finding 2: Aggregate CVaR does NOT separate the classes (p=0.94)

CVaR_95 computed across the entire run:

| Dataset | CVaR_95 mean |
|---|---|
| obs_recovery | -7.17 |
| obs_noescape | -8.71 |

Mann-Whitney p=0.94 — no significant difference.

Cause: Long obs_recovery runs (1155-1622 windows) contain extensive nominal driving before and after the obstacle event. The aggregate CVaR is dominated by nominal-phase windows.

### Finding 3: Anomaly rate ~6× higher in obs_recovery than obs_noescape

Using 3-sigma pooled threshold (τ = -6.37):

| Dataset | Anomalous windows | Total windows | Rate |
|---|---|---|---|
| obs_recovery | 517 | 16,363 | **3.16%** |
| obs_noescape | 27 | 4,872 | **0.55%** |

All 27 anomalous obs_noescape windows occur in the last 10% of their respective runs (obstacle encounter phase). No proactive early warning.

### Finding 4: Last-50-window analysis

| Dataset | Last 50 windows mean NLL | p95 |
|---|---|---|
| obs_recovery | -12.58 | -11.08 |
| obs_noescape | -11.47 | -7.74 |

obs_noescape shows slight elevation (mean shifts +1 unit, p95 shifts +3.3 units) in the final 50 windows before getting stuck. This is minimal early warning: only 5 seconds before the vehicle is already stuck.

---

## Mechanistic Interpretation

The ST-GAT model was trained on **nominal lane-following** trajectories at speeds 5-10 m/s. The model learned to predict smooth, forward motion.

**obs_recovery (avoidance)**: Vehicle performs a *lane change* or sharp lateral maneuver. This is highly anomalous relative to training distribution → large residual spike (NLL often positive: +5 to +10). The spike occurs *during* the maneuver, not before.

**obs_noescape (stuck)**: Vehicle continues in its lane, decelerating toward an obstacle. The trajectory looks like *nominal deceleration* (e.g., approaching a red light). The model predicts deceleration correctly → near-zero residual. Constraint violation (stuck) is indistinguishable from nominal stopping by trajectory alone.

---

## Implications for RISE Framework

### What works:
1. **Avoidance detection**: The residual signal reliably detects anomalous maneuvers (lane changes, emergency braking). This validates the residual as a signal of non-nominal operation.
2. **Post-hoc risk validation**: After a constraint violation episode, the residual time-series shows elevated NLL, confirming the episode was anomalous.

### Critical limitation:
**The residual cannot proactively detect unsolvable situations** (obs_noescape). The model sees "vehicle decelerating behind obstacle" as nominally plausible (it was trained on deceleration patterns at signals). Constraint violation in obs_noescape results from *scene topology* (no escape route), not from *trajectory anomaly*.

### Required additional signal:
To close this gap, RISE needs a *scene-level risk component* beyond trajectory residuals:
1. **Object distance trend**: Δ(object_distance) over recent windows — rapid decrease signals obstacle approach
2. **Graph topology**: Whether the current road graph has adjacent lanes available (from HD map)
3. **Conformal prediction on residual history**: A rising residual TREND (not just peak) may precede stuck events by a few windows

---

## Revised Research Direction for RISE

The original framing: *"CVaR from residuals provides valid probabilistic bounds on constraint violation"*

Refined framing after Q1: **"ST-GAT residuals provide reliable detection of anomalous vehicle maneuvers (reactive). For proactive detection of unsolvable situations, residual TRENDS combined with scene-level features are needed."**

Concretely for the dissertation:

**Contribution 1** (validated): ST-GAT residuals spike during non-nominal maneuvers and are statistically distinguishable from stuck scenarios by their peak value (p=0.034).

**Contribution 2** (to be validated): Windowed CVaR on the residual time-series can predict upcoming constraint violations with lead time measured in seconds — requires isolating the *approach phase* signal.

---

## Next Steps

1. **Approach-phase isolation**: Compare residuals in the window BEFORE the obstacle event in both datasets (need obstacle encounter timestamps from metrics.json or object_distance profile).
2. **Object-distance residual**: Add object_distance NLL component to inference script — may be more diagnostic for obstacle approach.
3. **Conformal calibration**: Use the 9-run calibration set to build prediction intervals; check coverage on test set.
4. **Dissertation framing**: Position this as "the residual signal is a necessary but not sufficient condition for proactive risk management — scene topology is also required."
