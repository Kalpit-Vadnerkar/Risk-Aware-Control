# Theoretical Framework: Residual-Informed Safety Envelopes

## 1. Introduction

This document formalizes the theoretical framework for Residual-Informed Safety Envelopes (RISE), a novel approach to risk-aware control in autonomous vehicles. The core contribution is using CVaR computed from digital twin prediction residuals to dynamically adjust control constraints.

## 2. Problem Formulation

### 2.1 Digital Twin Prediction

The ST-GAT digital twin predicts the vehicle's nominal behavior:

$$\hat{Y}^{t+H} = f_{DT}(S^t) = f_{DT}(G, X^t)$$

Where:
- $S^t = (G, X^t)$: Joint state (HD map graph $G$, temporal history $X^t$)
- $H$: Prediction horizon (30 timesteps)
- Output: Distributions $(\mu, \sigma^2)$ for continuous features, $p$ for discrete

### 2.2 Residual Computation

At each timestep, we compute residuals between predictions and observations:

**Raw Residual:**
$$\Phi_{raw}^t = \mu^t - u^t$$

**KL Divergence (NLL-based):**
$$\Phi_{KLD}^t = \frac{1}{2}\left[\ln(2\pi\sigma^2) + \frac{(u - \mu)^2}{\sigma^2}\right]$$

**CUSUM:**
$$z_t = \frac{\mu - u}{\sigma}$$
$$C^+_t = \max(0, C^+_{t-1} + z_t - \frac{\delta}{2})$$
$$C^-_t = \max(0, C^-_{t-1} - z_t - \frac{\delta}{2})$$
$$\Phi_{CUSUM}^t = \frac{\max(C^+, C^-)}{d}$$

### 2.3 CVaR Computation

Given a window of residuals $R = \{r_1, ..., r_n\}$:

**Value-at-Risk:**
$$VaR_\alpha = \inf\{x : P(R \leq x) \geq \alpha\}$$

**Conditional Value-at-Risk:**
$$CVaR_\alpha = E[R | R \geq VaR_\alpha]$$

Interpretation: CVaR_α represents the expected residual magnitude in the worst $(1-\alpha)\%$ of cases.

## 3. Safety Envelope Formulation

### 3.1 Constraint Tightening

The core mechanism is constraint tightening based on CVaR:

**Original constraint:**
$$h(x) \geq 0$$

**Tightened constraint:**
$$h(x) \geq \gamma(CVaR)$$

Where $\gamma: [0, \infty) \rightarrow [0, \infty)$ is a monotonically increasing margin function.

### 3.2 Margin Function Design

We consider several margin functions:

**Linear:**
$$\gamma_{linear}(c) = k \cdot c$$

**Exponential:**
$$\gamma_{exp}(c) = k(e^{\beta c} - 1)$$

**Sigmoid (bounded):**
$$\gamma_{sigmoid}(c) = \frac{\gamma_{max}}{1 + e^{-\lambda(c - c_0)}}$$

### 3.3 Preemptive Tightening

To address detection latency, we also react to CVaR trends:

$$\gamma_{eff} = \begin{cases}
\gamma(CVaR) \cdot \kappa & \text{if } \frac{dCVaR}{dt} > \theta \\
\gamma(CVaR) & \text{otherwise}
\end{cases}$$

Where $\kappa > 1$ is the preemptive multiplier and $\theta$ is the trend threshold.

## 4. Probabilistic Guarantees

### 4.1 Coverage Validity

**Theorem 1 (Coverage):** For a calibrated CVaR estimator with confidence level $\alpha$, the probability that the actual residual exceeds the safety margin is bounded by:

$$P(\Phi > \gamma(CVaR_\alpha)) \leq 1 - \alpha$$

*Proof sketch:* By definition of CVaR as a coherent risk measure, it provides an upper bound on expected tail loss.

### 4.2 Constraint Satisfaction

**Theorem 2 (Constraint Satisfaction):** If the margin function $\gamma$ satisfies:
$$\gamma(CVaR_\alpha) \geq \mathbb{E}[\Phi | \Phi \geq VaR_\alpha]$$

Then the tightened constraint provides probabilistic safety guarantees at level $\alpha$.

### 4.3 Calibration Requirement

For valid guarantees, the CVaR estimator must be calibrated:

$$\frac{1}{T}\sum_{t=1}^{T} \mathbb{1}[\Phi^t > VaR_\alpha] \approx 1 - \alpha$$

## 5. Application to Autoware Constraints

### 5.1 Distance Constraint

**Original:** $d_{obstacle} \geq d_{safe}$

**Tightened:** $d_{obstacle} \geq d_{safe} + \gamma(CVaR)$

### 5.2 Lane Keeping

**Original:** $|e_{lateral}| \leq e_{max}$

**Tightened:** $|e_{lateral}| \leq e_{max} - \gamma(CVaR)$

### 5.3 Velocity Limit

**Original:** $v \leq v_{limit}$

**Tightened:** $v \leq v_{limit} - \gamma(CVaR)$

## 6. Validation Methodology

### 6.1 CVaR Calibration Test

Run N scenarios, measure empirical coverage:

$$\hat{p}_{violation} = \frac{\#\{i : \Phi_i > VaR_\alpha\}}{N}$$

Accept if $|\hat{p}_{violation} - (1-\alpha)| < \epsilon$ for tolerance $\epsilon$.

### 6.2 Safety Improvement Test

Compare baseline (fixed margins) vs. RISE:
- Collision rate
- Lane departure rate
- Minimum TTC distribution

### 6.3 Performance Trade-off Test

Measure impact on nominal performance:
- Mission completion time
- Tracking error
- Comfort metrics

## 7. Limitations and Assumptions

1. **Stationarity:** CVaR estimation assumes approximately stationary residual distribution over the window
2. **Detection Latency:** Cannot detect faults before they affect vehicle behavior
3. **Model Quality:** Effectiveness depends on ST-GAT prediction quality
4. **Constraint Feasibility:** Aggressive tightening may cause infeasibility

## 8. References

1. Rockafellar, R.T., Uryasev, S. (2002). Conditional value-at-risk for general loss distributions.
2. Vadnerkar, K., et al. (2025). Digital Twins as Predictive Models for Real-Time Probabilistic Risk Assessment of Autonomous Vehicles. IEEE T-ITS.
3. Majumdar, A., Pavone, M. (2020). How should a robot assess risk? Towards an axiomatic theory of risk in robotics.
