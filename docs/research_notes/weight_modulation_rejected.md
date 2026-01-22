# Research Decision: Constraint Tightening vs. Weight Modulation

**Date:** 2026-01-21
**Decision:** Use constraint tightening as primary control method
**Status:** Documented for committee defense

## Background

Two approaches were considered for implementing risk-aware control:

1. **Constraint Tightening:** Modify constraint bounds based on CVaR
2. **Weight Modulation:** Modify MPC objective weights based on CVaR

## Analysis

### Constraint Tightening

**Pros:**
- Provides formal probabilistic guarantees (Theorem 2 in theoretical framework)
- Interpretable: "Stay X meters further from obstacles when uncertain"
- Validation is straightforward: measure actual margin vs. required margin
- Stronger publication contribution (novel theoretical framework)
- Can derive P(violation) bounds from CVaR properties

**Cons:**
- May become infeasible if margins are too large
- Requires integration with Autoware's constraint system
- More complex implementation

### Weight Modulation

**Pros:**
- Always feasible (just re-prioritizes objectives)
- Simpler to implement (change parameters)
- Familiar in MPC literature

**Cons:**
- No formal safety guarantees
- "Care less about tracking" is harder to interpret
- Hard to verify correctness
- Less novel (common engineering approach)
- Relationship between weights and violation probability unclear

## Decision Rationale

1. **Research Contribution:** Constraint tightening enables a novel theoretical contribution with probabilistic guarantees. Weight modulation is a well-known engineering practice.

2. **Safety Guarantees:** The primary goal is improved safety. Constraint tightening provides explicit bounds on violation probability. Weight modulation provides no such guarantees.

3. **Committee Defense:** "We tighten the obstacle distance constraint by γ(CVaR) meters, ensuring P(collision) < 0.05" is defensible. "We reduce w_tracking when CVaR is high" lacks formal justification.

4. **Validation:** Constraint tightening allows straightforward validation: plot actual margin vs. required margin. Weight modulation has no clear validation metric.

## Mitigation for Feasibility Concern

Implement with bounded tightening:
```python
margin = min(gamma(CVaR), max_allowed_margin)
```

If infeasibility still occurs, gracefully degrade to emergency stop mode.

## For Committee Defense

If asked "Why not weight modulation?":

> "Weight modulation provides no formal guarantees on constraint satisfaction. Our constraint tightening approach allows us to derive probabilistic bounds on safety violations using CVaR properties. Additionally, constraint margins are directly interpretable in physical units (meters, seconds), making validation and debugging more tractable."

## Related Work

- Robust MPC with constraint tightening: [Kouvaritakis & Cannon, 2016]
- Risk-aware constraint modification: [Majumdar & Pavone, 2020]
- Weight modulation in adaptive MPC: [Standard practice, no formal guarantees]
