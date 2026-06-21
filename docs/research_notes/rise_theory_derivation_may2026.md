# RISE Theory Derivation — Formal Framework

**Date:** 2026-05-01
**Status:** Finalized direction — this is the core theoretical contribution
**Supersedes:** `rise_experiment_plan_mar2026.md` (experiment structure changed)

---

## Decision: Theory-First with Minimal Experiments

**What changed:** Prior plan called for a large scenario matrix (5 scenario types,
4 distances, 2 fault types, 100+ runs). This was an infrastructure project, not a
research project. Pivoted to a lean, theory-first approach.

**New structure:**
1. Derive the formal constraint mapping (conformal prediction, no parametric assumptions)
2. Validate the anomaly signal on existing data (~100 runs already collected)
3. Demonstrate RISE on ONE well-chosen scenario (~10 additional runs)

**Rationale:** One tight result with a formal guarantee is more publishable and
defensible than a wide matrix of inconclusive runs. The theoretical gap
(ego-system residuals → continuous constraint) is the contribution; the scenario
runs demonstrate it.

---

## The Full Derivation (for Chapter 4)

### 4.1 Residual Construction

ST-GAT outputs a predicted distribution over the next state:
- Continuous features: p̂ᵢ(u|xᵗ) = N(μᵢ,ₜ, σ²ᵢ,ₜ)
- Observed state: uᵢ,ₜ

NLL (negative log-likelihood) residual per feature:
```
φᵢ,ₜ = 0.5 × [ln(2πσ²ᵢ,ₜ) + (uᵢ,ₜ - μᵢ,ₜ)² / σ²ᵢ,ₜ]
```

This is the Φ_kld residual from Vadnerkar et al. 2025, per feature.
It penalizes both wrong mean predictions AND overconfident uncertainty estimates.

**Aggregation:** Use feature importance weights wᵢ from T-ITS 2025 paper:
```
Φₜ = Σᵢ wᵢ × φᵢ,ₜ
```

Traffic light (0.297), velocity (approx.), position (approx.) — exact weights
from the prior paper's feature importance analysis.

### 4.2 CVaR Anomaly Score

Over a sliding window W = {t-w+1, ..., t} (w=20 timesteps, 2s at 10Hz):

```
Aₜ = CVaR_α(Wₜ) = (1/|Wₜ⁺|) × Σ_{Φ ∈ Wₜ⁺} Φ
```

where Wₜ⁺ = {Φ ∈ Wₜ : Φ ≥ VaR_α(Wₜ)}, α = 0.95.

**Why CVaR:** Captures the dangerous tail of the residual window, not the average.
A single hard brake spike in a 2s window should register; averaging would smooth it out.

### 4.3 Baseline Normalization

From nominal calibration data D_cal (nominal driving runs):
```
A₀ = CVaR_0.95({Aₜ : t ∈ D_cal})
Ã_t = Aₜ / A₀
```

- Ã_t ≈ 1: nominal behavior
- Ã_t > 1: anomalous (worse than 95th percentile of normal driving)
- Ã_t >> 1: strong anomaly (obstacle encounter, fault condition)

### 4.4 Conformal Prediction Bound (The Formal Guarantee)

**Goal:** Bound the H-step-ahead position error eₜ₊ₕ = ||uₜ₊ₕ - μₜ₊ₕ||₂

**Split conformal procedure:**
1. Split D_cal → D_train (50%) + D_conf (50%)
2. Fit ê_H(A): monotone regression of eₜ₊ₕ on Aₜ, using D_train
3. Compute conformity scores on D_conf:
   sᵢ = eₜ₊ₕ,ᵢ - ê_H(Aₜ,ᵢ)
4. Compute the (1-δ) quantile of {sᵢ}:
   q̂_{1-δ} = quantile_{1-δ}({sᵢ})
5. Define: r(Aₜ) = ê_H(Aₜ) + q̂_{1-δ}

**Theorem (distribution-free coverage):**
For any test point drawn exchangeably from the same distribution as D_cal:
```
P( eₜ₊ₕ ≤ r(Aₜ) ) ≥ 1 - δ
```

**Key property:** This holds WITHOUT assuming Gaussian residuals. The only
assumption is exchangeability — calibration and test data come from the same
operating conditions. This is the critical advantage over GP-based or
parametric approaches, which all require distributional assumptions that
empirical GNN residuals cannot satisfy.

### 4.5 Velocity Constraint Derivation (Kinematic Argument)

At time t:
- dₜ = nearest obstacle distance (from perception)
- d_min = minimum required clearance (2m)
- H = planning horizon (3s)
- r(Aₜ) = conformal position error bound at current anomaly level

Worst-case vehicle position at t+H: travels forward v×H, actual position
deviates from prediction by at most r(Aₜ) (with probability ≥ 1-δ).

Safety requirement: (dₜ - v×H) - r(Aₜ) ≥ d_min

Solving for v:
```
v_max(t) = max(v_min, (dₜ - d_min - r(Aₜ)) / H)
```

**Fallback (no obstacle detected):**
```
v_max(t) = v_nominal × max(κ_min, 1 - κ × max(0, Ã_t - 1))
```
This is explicitly heuristic (less formal); used only when perception returns
no nearby objects. κ and κ_min are calibrated from scenario data.

### 4.6 Formal Safety Theorem

**Theorem (RISE Safety Guarantee):**
Let D_cal be N calibration windows from nominal driving, δ ∈ (0,1).
Define v_max(t) as above with r(Aₜ) as the (1-δ)-conformal bound on eₜ₊ₕ.

For any test window drawn exchangeably from D_cal's distribution:
```
P[ clearance at t+H ≥ d_min ] ≥ 1 - δ
```

**Proof sketch:**
1. By conformal prediction: P(eₜ₊ₕ ≤ r(Aₜ)) ≥ 1-δ
2. Under v_max(t): vehicle displacement ≤ v_max × H = dₜ - d_min - r(Aₜ)
3. Clearance ≥ dₜ - v_max×H - eₜ₊ₕ ≥ d_min + r(Aₜ) - eₜ₊ₕ ≥ d_min w.p. ≥ 1-δ ∎

---

## Design Parameter Choices

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| α (CVaR level) | 0.95 | Consistent with T-ITS 2025 paper |
| w (window) | 20 timesteps | 2s at 10Hz; captures maneuver duration |
| δ (coverage slack) | 0.05 | 95% coverage guarantee |
| H (horizon) | 3s | Autoware planning horizon |
| d_min | 2m | Collision proxy threshold from experiments |
| v_min | 1 m/s | Don't command a full stop via RISE; MRM handles that |

---

## Honest Limitations (for thesis Section 4.7)

1. **Exchangeability assumption:** Calibration data must cover the test distribution.
   RISE is calibrated for Shinjuku map + Autoware + AWSIM. Out-of-distribution
   deployments require recalibration.

2. **No formal guarantee for fallback mode:** The precautionary reduction when
   no obstacle is detected is heuristic. Explicitly scoped as conservative behavior,
   not a formal claim.

3. **H-step error prediction:** The isotonic regression ê_H(A) is a point estimate;
   we rely on conformity scores to correct for its error. If the regression is very
   wrong, the conformity scores absorb it but the bound r(Aₜ) may become loose.

4. **Single scenario validation:** Core experimental demonstration uses one scenario
   type. Generalization to other scenarios (cut-in, pedestrian, etc.) is cited as
   future work.

---

## Novelty Position (confirmed against 18 verified papers)

RISE is the only approach that simultaneously:
1. Uses a learned digital twin (GNN) for ego-health residuals
2. Derives a principled, distribution-free mapping to a constraint value
3. Applies that mapping continuously to a velocity constraint in a production AV stack
4. Provides bidirectional operation (tighten and relax)
5. Offers a formal bound without parametric distribution assumptions
6. Is validated end-to-end in Autoware + AWSIM

See `lit_review_constraint_risk_aware_control.md` for full gap analysis.
