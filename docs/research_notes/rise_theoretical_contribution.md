# RISE Theoretical Contribution — Key Insight

**Date:** 2026-03-05
**Last Updated:** 2026-03-20
**Status:** Working draft — conceptual framing is stable; specific formulas are
candidates under evaluation, not final decisions

---

## The Core Problem

Standard AV safety systems (MRM, emergency stop) are binary: nominal operation → emergency
stop. There is no middle ground that lets the vehicle continue its mission at reduced
capability while managing elevated risk. MRM "bricks the car" — which is safe but not
useful.

RISE is motivated by the question: **can the system continuously adapt its operational
envelope based on real-time risk, rather than switching between normal and emergency?**

---

## Three Distinct Concepts and Why All Three Are Needed

### 1. Uncertainty (What the Digital Twin Provides)

The ST-GAT digital twin predicts expected vehicle behavior. Residuals measure the gap
between prediction and reality:
- Small residuals → vehicle behaving as expected → low uncertainty
- Large residuals → something unexpected is happening → high uncertainty

**Uncertainty alone is not enough.** The vehicle might have large residuals because it
is at an unusual intersection (novel but benign) or because it is about to run into an
obstacle (novel and dangerous). Uncertainty doesn't tell us how consequential the deviation is.

### 2. Risk (What We Need to Compute)

Risk is uncertainty + consequence. The same residual magnitude means different things
depending on the situation:
- Large residual at high speed, obstacle 20m ahead → HIGH risk
- Large residual at low speed in open road → MODERATE risk
- Large residual during a normal lane change → LOW risk

**Risk = f(residual magnitude, geometric context, current constraint proximity)**

CVaR is the right tool for capturing tail risk from the residual distribution:
- CVaR_α = expected value of residuals in the worst α fraction
- This focuses the risk estimate on the dangerous tail, not the average case
- High CVaR → the system is in a regime where bad outcomes are probable

But CVaR from residuals alone is still uncertainty. The geometric context (TTC, clearance,
distance to constraint boundary) must be incorporated to compute actual risk.

### 3. Mission Completion (The Optimization Objective)

RISE must not just minimize risk — it must **optimize the tradeoff** between risk and
mission completion. A constraint that always says "stop" achieves zero risk but zero utility.

**The objective:** find the tightest constraint that keeps CVaR below a threshold while
preserving the ability to complete the route.

This is different from MRM in a fundamental way:
- MRM: IF risk_detected THEN stop (binary, mission-terminating)
- RISE: GIVEN current_risk, FIND max_velocity SUCH THAT risk stays acceptable AND
  mission progress continues

---

## Proposed Mapping: Residuals → Risk → Constraint

> **Note:** The steps below describe the conceptual pipeline. Specific formulas
> (e.g., CVaR, sigmoid) are candidates under evaluation, not finalized methods.

### Step 1: Residual Stream → Anomaly Score

At each timestep, maintain a sliding window of residuals (raw, KL-div, CUSUM) and
compute an anomaly score summarizing the tail behavior of the distribution.

**Leading candidate — CVaR-based:**
```
anomaly(t) = CVaR_α(t) = E[Φ | Φ ≥ VaR_α]    where Φ is the residual distribution
```

This focuses the score on the worst-case tail, not the average residual. Other candidates
(normalized magnitude, exponential moving average over threshold exceedances) are also
under consideration.

Higher anomaly score → system is in a high-deviation regime.

### Step 2: Anomaly Score + Geometric Context → Risk Score

Combine the statistical anomaly with the geometric context:

```
R(t) = anomaly(t) × g(TTC, clearance, speed)
```

Where g(·) amplifies the anomaly signal when the vehicle is close to a constraint boundary
(low TTC, low clearance) and attenuates it when the vehicle is in open space.

This ensures that high uncertainty in a geometrically safe situation does not trigger
unnecessary constraint tightening.

### Step 3: Risk Score → Velocity Constraint

Map R(t) to a velocity limit. The mapping must be:
- Monotonically decreasing with R(t) (higher risk → lower v_max)
- Smooth (no abrupt velocity jumps)
- Bounded (v_max never goes below a minimum, not a binary stop)
- Reversible (relaxes back toward nominal when risk decreases)

**Candidate form (sigmoid-based):**
```
v_max(t) = v_nominal × (1 - k · sigmoid(R(t) - R_threshold))
```
This is one candidate; the exact functional form will be selected based on Phase 2
experimental calibration data.

The parameters (k, threshold, v_min) cannot be set a priori — they are calibrated
so that the constraint tightening is neither over-conservative (unnecessary slowing)
nor under-conservative (too late to prevent the violation).

### Step 4: Preemptive Tightening via Trend Detection

RISE should tighten constraints **before** the risk score peaks, not after. If R(t)
is rising, begin tightening early — the first derivative acts as an early warning.

**Candidate form:**
```
v_max(t) = v_nominal × (1 - k1 · f(R(t)) - k2 · max(0, dR/dt))
```

This addresses detection latency — by the time the score is high, the vehicle may
already be at the violation boundary. This is a candidate formulation; alternatives
(e.g., adaptive thresholds, predictive filtering) will be evaluated against data.

---

## Why This Is Novel (Anticipated Gap)

Based on what has been published:

1. **Most ODD-adaptation work** changes the ODD at a mission-planning level (refuse route
   segments with certain properties) — not at the real-time constraint level during execution.

2. **Tail-risk metrics (CVaR etc.) in AV literature** are used for offline risk analysis
   or for planning with *external* obstacle trajectory distributions — not for real-time
   constraint adaptation based on the *ego vehicle's own* prediction residuals from a
   running digital twin.

3. **Digital twin + residuals** approaches (including our prior T-ITS 2025 paper) have been
   used for fault detection (passive observer) — not as an input to an active control
   constraint.

4. **The combination** — digital twin residuals + residual anomaly score + real-time
   velocity constraint adaptation + mission-preserving graceful degradation — appears to
   be unoccupied in the literature.

The specific gap: **using ego-system prediction residual distributions to compute a
tail-risk anomaly score and continuously modulate operational constraints during execution,
while preserving mission completion, in a formally-motivated probabilistic framework.**

---

## Open Questions for Formalization

1. **What anomaly scoring method?** CVaR over sliding window is the leading candidate.
   Normalized magnitude and trend-based methods are alternatives. Choice depends on
   responsiveness, noise sensitivity, and calibrability to Phase 2 data.

2. **How to incorporate geometric risk?** TTC from perception, or compute TTC from the
   predicted trajectory in the digital twin? Context weighting g(·) amplifies the anomaly
   signal near constraint boundaries.

3. **What is the right window size?** Short window → responsive but noisy. Long window →
   smooth but lagging. Adaptive window based on variance of the residual stream?

4. **How to calibrate the mapping parameters?** Grid search on Phase 2 data, or derive
   analytically from a constraint violation probability bound?

5. **Formal guarantee:** Can we state "with probability 1-δ, the constraint will not be
   violated if RISE is active and the anomaly score is below threshold τ"? This requires
   a concentration inequality relating the residual anomaly score to actual constraint
   violation probability.
