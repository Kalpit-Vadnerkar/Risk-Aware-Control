# RISE: Residual-Informed Safety Envelope
## Theoretical Framework

**Last Updated:** 2026-01-23

---

## 1. Core Concept

RISE uses digital twin prediction residuals to dynamically adjust safety constraints. When predictions diverge from observations, uncertainty is high, so safety margins increase.

**The key equation:**
```
constraint_margin = nominal_margin + f(uncertainty)
```

Where uncertainty is derived from prediction residuals.

---

## 2. Framework Evolution

### v1: CVaR-Based Margin Function (Initial)
**Approach:** Compute CVaR from residuals, use margin function γ(CVaR).
```
γ_tightened = γ_base + k × CVaR^β
```
**Limitation:** How to set k, β? Manual tuning is unprincipled.

### v2: Learned Sensitivity Matrix (Rejected)
**Approach:** Learn matrix A mapping risk vector to constraints.
```
γ = clip(γ_base + A·ρ, γ_min, γ_max)
```
**Limitation:** Where does A come from? Would require RL-like training, infeasible for our setup.

### v3: Uncertainty Propagation (Current)
**Approach:** Derive constraint tightening analytically from uncertainty propagation.
```
Σ_effective = Σ_predicted × (1 + κ(Φ))    # Inflate covariance based on residual
tube_width = k_σ × √(Σ_propagated)         # Compute safety tube
margin = nominal + tube_width              # Tighten by tube width
```
**Advantage:** Principled derivation, no learning required, probabilistic guarantees via k_σ.

---

## 3. Current Formulation (v3)

### 3.1 Inputs

| Input | Source | Description |
|-------|--------|-------------|
| Observations z_t | Sensors | Current state with covariance R_t |
| Predictions (μ_t, Σ_t) | ST-GAT | Expected state with uncertainty |
| Trajectory τ | Autoware | Planned path with velocities |

### 3.2 Residual Computation

**Normalized residual:**
```
Φ = |z - μ| / √(Σ + R)
```

Accounts for both prediction and observation uncertainty.

### 3.3 Covariance Inflation

When residuals are high, predictions are unreliable:
```
Σ_effective = Σ_predicted × (1 + α(Φ/Φ_nominal)^β)
```

| Parameter | Meaning | Typical Value |
|-----------|---------|---------------|
| α | Max inflation | 1.0 - 3.0 |
| β | Sensitivity | 1.0 (linear) |
| Φ_nominal | Baseline residual | Measured from normal operation |

### 3.4 Tube Computation

Propagate uncertainty along trajectory, compute tube width:
```
tube_width = k_σ × √(diag(Σ_propagated))
```

Where k_σ determines confidence level:
- k_σ = 2.0 → ~95% coverage
- k_σ = 3.0 → ~99.7% coverage

### 3.5 Constraint Adjustment

| Constraint | Formula |
|------------|---------|
| Safe distance | d = d_nominal + tube_width_position |
| Velocity limit | v = v_nominal × (1 - tube_ratio) |
| Lateral margin | m = m_nominal + tube_width_lateral |

---

## 4. Fail-Operational Behavior

The framework is naturally bidirectional:

| Condition | Residuals | Tube | Constraints |
|-----------|-----------|------|-------------|
| Normal | Low | Narrow | ≈ Nominal |
| Internal fault | High | Wide | Tightened |
| Low uncertainty | Very low | Very narrow | Can relax toward nominal |

For external threats (sudden obstacle): object residual spikes → longitudinal margin increases. If no in-lane avoidance feasible, lateral margin doesn't additionally tighten, enabling avoidance maneuvers.

---

## 5. What Makes This Novel

1. **Residual → Uncertainty → Constraint loop:** Closes the loop from passive digital twin to active control
2. **Analytical derivation:** No learned mappings, principled from uncertainty propagation
3. **Probabilistic guarantees:** k_σ provides coverage bounds
4. **Fail-operational:** Same framework handles both tightening and relaxation
