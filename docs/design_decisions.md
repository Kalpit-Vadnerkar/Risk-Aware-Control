# Design Decisions

**Last Updated:** 2026-01-23

This document records key design decisions with rationale for committee defense and future reference.

---

## 1. Constraint Tightening vs Weight Modulation

**Decision:** Use constraint tightening, not MPC weight modulation.

**Rationale:**
- Constraint tightening provides formal probabilistic guarantees
- Interpretable in physical units (meters, m/s)
- Weight modulation has no clear relationship to violation probability
- Stronger research contribution (novel theoretical framework)

**For defense:** "Weight modulation provides no formal guarantees on constraint satisfaction. Our approach derives probabilistic bounds from uncertainty propagation."

---

## 2. Analytical vs Learned Constraint Mapping

**Decision:** Derive constraint tightening analytically from uncertainty propagation.

**Alternatives considered:**
- Learned sensitivity matrix A (v2) - rejected, requires RL-like training
- Manual tuning - rejected, unprincipled
- CVaR threshold-based - rejected, still requires manual threshold setting

**Rationale:**
- Tube-based MPC provides principled constraint tightening from uncertainty
- No training data needed
- k_σ parameter has statistical interpretation (confidence level)
- Same math used in robust control literature

---

## 3. Handling Detection Latency

**Problem:** Can't detect faults until they affect behavior.

**Solution:** Multi-pronged approach:
1. **Trend detection:** React to rising residuals before threshold breach
2. **Severity targeting:** Focus on severe faults where quick detection occurs
3. **Continuous adaptation:** Frame as "maintaining margins proportional to reliability" not "detect and react"

**For defense:** "Severe faults cause large residual spikes quickly. In experiments, severe IMU faults are detected within 1 second. The key metric is TTC minus detection latency."

---

## 4. Fail-Operational Design

**Problem:** Original design only tightened constraints (fail-safe), but sometimes loosening is needed for emergency maneuvers.

**Solution:** Uncertainty propagation naturally handles both:
- High uncertainty → wide tube → tighten
- Low uncertainty → narrow tube → can relax toward nominal
- Different residual patterns → different constraint responses

**Key insight:** For external threats (sudden obstacle), longitudinal constraints tighten but lateral constraints respond only to lateral uncertainty.

---

## 5. State Representation Enhancement

**Context:** T-ITS paper hardcoded unit variance for some features.

**Decision:** Extract actual covariance from ROS2 messages:
- Position/velocity: From `/localization/*_with_covariance`
- Object confidence: From `existence_probability`
- Traffic light: From `confidence` field

**Benefit:** Residuals now reflect true sensor reliability, not assumed constants.

---

## 6. Fault Severity Focus

**Decision:** Target severe faults (50-75% degradation), not mild ones.

**Rationale:**
- Mild faults: Autoware handles adequately
- Moderate faults: Marginal benefit
- Severe faults: **Primary contribution** - collision prevention
- Critical faults: Reduce incident severity

**For defense:** "We're honest about our operating envelope. This system is not designed for mild faults - existing safety systems handle those. Our contribution is for severe faults where baseline approaches fail."
