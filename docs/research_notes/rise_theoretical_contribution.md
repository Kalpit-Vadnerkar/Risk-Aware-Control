# RISE Theoretical Contribution — Key Insight

**Date:** 2026-03-05
**Status:** Working draft — to be formalized

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

### Step 1: Residual Stream → CVaR

At each timestep, maintain a sliding window of residuals (raw, KL-div, CUSUM).
Compute CVaR_α over the window:

```
CVaR_α(t) = E[Φ | Φ ≥ VaR_α]    where Φ is the residual distribution
```

Higher CVaR → system is in a high-deviation regime.

### Step 2: CVaR + Geometric Risk → Risk Score

Combine the statistical tail risk with the geometric context:

```
R(t) = CVaR_α(t) × g(TTC, clearance, speed)
```

Where g(·) amplifies the CVaR signal when the vehicle is close to a constraint boundary
(low TTC, low clearance) and attenuates it when the vehicle is in open space.

This ensures that high uncertainty in a safe situation does not trigger unnecessary
constraint tightening.

### Step 3: Risk Score → Velocity Constraint

Map R(t) to a velocity limit:

```
v_max(t) = v_nominal × (1 - k · sigmoid(R(t) - R_threshold))
```

- When R(t) < threshold: v_max = v_nominal (no tightening)
- When R(t) >> threshold: v_max → v_min (maximum tightening, approaching but not
  identical to emergency stop)
- The sigmoid ensures smooth transitions (no abrupt velocity changes)

The parameters (k, threshold, v_min) are tuned on Phase 2 experimental data to
minimize collision proxy count while maximizing route completion rate.

### Step 4: Preemptive Tightening via Trend Detection

A key addition: RISE should tighten constraints **before** CVaR peaks, not after.
If the CVaR trend (first derivative) is rising, begin tightening:

```
v_max(t) = v_nominal × (1 - k1 · sigmoid(R(t)) - k2 · max(0, dR/dt))
```

This addresses detection latency — by the time CVaR is high, the vehicle may already
be in a dangerous situation. The trend component acts as an early warning.

---

## Why This Is Novel (Anticipated Gap)

Based on what has been published:

1. **Most ODD-adaptation work** changes the ODD at a mission-planning level (refuse route
   segments with certain properties) — not at the real-time constraint level during execution.

2. **CVaR in AV literature** is typically used for offline risk analysis or for planning
   in stochastic environments — not for real-time constraint adaptation based on a running
   digital twin.

3. **Digital twin + residuals** approaches (including our prior T-ITS 2025 paper) have been
   used for fault detection (passive observer) — not as an input to an active control
   constraint.

4. **The combination** — digital twin residuals + CVaR + real-time velocity constraint
   adaptation + mission-preserving optimization — appears to be unoccupied in the literature.

The specific gap: **using prediction residual distributions to compute tail risk and
continuously modulate operational constraints during execution, while preserving mission
completion, in a formally-motivated probabilistic framework.**

---

## Open Questions for Formalization

1. **How to incorporate geometric risk into the CVaR signal?** TTC from perception? Or
   compute TTC from the predicted trajectory in the digital twin?

2. **What is the right window size for CVaR computation?** Short window → responsive but
   noisy. Long window → smooth but lagging. Adaptive window?

3. **How to tune the mapping parameters?** Grid search on Phase 2 data, or derive
   analytically from a constraint violation probability bound?

4. **Formal guarantee:** Can we state "with probability 1-δ, the constraint will not be
   violated if RISE is active and CVaR < threshold"? This requires a concentration
   inequality relating residual CVaR to actual constraint violation probability.
