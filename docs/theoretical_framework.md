# RISE: Residual-Informed Safety Envelopes — Theoretical Framework

**Last Updated:** 2026-03-20

> **Status:** Conceptual framework is stable. The specific residual-to-constraint
> mapping method is still under development. This document captures the research
> framing; specific formulas will be finalized once experimental data is available.

---

## 1. Core Concept

RISE uses ST-GAT digital twin prediction residuals to dynamically adjust safety
constraints. When predictions diverge from observations, something unexpected is
happening — safety margins should increase.

**The relationship:**
```
residual signal → anomaly score → constraint adjustment
```

The anomaly score transforms raw residuals into a monotonic risk signal. The constraint
adjustment translates that score into a tighter velocity limit or safety distance.
Both mappings are under active development (see Section 3).

---

## 2. Why Residuals Are the Right Signal

The ST-GAT model predicts expected vehicle behavior in nominal conditions. Residuals
(prediction error) have two useful properties:

1. **Interpretability:** A spike in position residual means "the vehicle moved more than
   expected." A spike in velocity residual means "the vehicle is accelerating/decelerating
   more than expected." The signal is physically meaningful.

2. **Early warning:** Residuals rise as the vehicle enters an anomalous regime, before
   a constraint is actually violated. This is what enables preemptive constraint tightening.

The prior T-ITS 2025 paper used these residuals for **passive fault detection** (classify
driving state). RISE uses them for **active control feedback** (adjust constraints
continuously based on residual magnitude).

---

## 3. Mapping Residuals to Constraints

### 3.1 Residual Types (from T-ITS 2025)

| Type | Formula | Captures |
|------|---------|---------|
| Raw | `Φ_raw = μ - u` | Direct prediction error |
| KL-divergence | `Φ_kld = 0.5[ln(2πσ²) + (u-μ)²/σ²]` | Error normalized by predicted uncertainty |
| CUSUM | `Φ_cusum = max(C⁺, C⁻) / d` | Cumulative drift from baseline |

### 3.2 Anomaly Score (Method TBD)

The anomaly score summarizes the residual stream into a scalar risk signal. Candidate
approaches under consideration:

- **CVaR-based:** Compute CVaR_α over a sliding window of residuals; focuses on the
  dangerous tail of the distribution
- **Normalized magnitude:** Ratio of current residual to nominal baseline
- **Trend-based:** Combine current level with rate of change (preemptive component)

The key property any method must satisfy: the score should be **monotonically increasing**
as the vehicle's behavior diverges further from nominal, and **calibrated** so that a
given score corresponds to a meaningful risk level.

### 3.3 Constraint Adjustment (Method TBD)

The anomaly score must be mapped to a constraint value (velocity limit, safety distance).
The mapping must satisfy:

- At nominal score → constraint ≈ nominal (no unnecessary tightening)
- At high score → constraint tightens toward a minimum (not binary stop)
- Smooth / continuous (no abrupt velocity jumps)
- Reversible (when score returns to nominal, constraint relaxes)

Candidate mapping approaches: monotonic functions (sigmoid, piecewise linear, power law),
analytically derived from uncertainty propagation (tube-MPC style), or empirically
calibrated from Phase 2 experiment data.

### 3.4 Which Constraints to Adjust

| Constraint | RISE Action | Rationale |
|------------|------------|---------|
| Velocity limit | Reduce proportional to anomaly score | Buys reaction time for any threat |
| Safety distance | Increase | Direct collision margin |
| Lateral margin | Context-dependent | Only tighten if lateral residuals are elevated |

---

## 4. Fail-Operational Behavior

A key design principle: RISE should enable **graceful degradation**, not binary
stop-or-go. The vehicle continues its mission at reduced capability, rather than
triggering an emergency stop. This is the key distinction from MRM.

| Condition | Residuals | Constraints | Vehicle Behavior |
|-----------|-----------|-------------|-----------------|
| Normal | Low | ≈ Nominal | Full mission, normal speed |
| Elevated risk | Moderate | Moderately tightened | Mission continues, slower |
| High risk | High | Significantly tightened | Mission continues, very slow |
| MRM threshold | (out of RISE scope) | MRM takes over | Emergency stop |

RISE operates in the space between "nominal" and "MRM trigger." It should reduce the
frequency of MRM events by addressing conditions preemptively.

---

## 5. Framework Evolution (History)

Understanding why earlier formulations were set aside helps motivate the current direction.

**v1: CVaR threshold mapping** — Compute CVaR_α over residual window; apply margin
function γ(CVaR). The limitation: how to set the parameters of the margin function
without principled derivation. CVaR remains a strong candidate for the anomaly score;
the question is how to calibrate the mapping.

**v2: Learned sensitivity matrix** — Learn a matrix A mapping residual vector to
constraints. Rejected: requires RL-like training data that we don't have and may not
generalize.

**v3: Uncertainty propagation (covariance inflation)** — Derive constraint tightening
analytically by inflating the predicted covariance proportional to residual magnitude,
then computing a safety tube width. This is principled and requires no training data.
Under active evaluation: the challenge is ensuring the tube width has a meaningful
physical interpretation in the Autoware constraint context.

The final method will be chosen based on: (1) availability of a principled derivation,
(2) calibration tractability, (3) performance on Phase 2 experimental data.

---

## 6. Formal Guarantee (Goal)

The ideal outcome is a statement of the form:

> "With probability ≥ 1-δ, if the RISE constraint is active and the anomaly score is
> below threshold τ, the constraint will not be violated within the planning horizon."

This requires relating the residual distribution (what we can measure) to constraint
violation probability (what we care about). This is the primary theoretical challenge
and is the subject of ongoing work.
