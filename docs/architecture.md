# Pipeline architecture

Current design, 2026-10-08. Shareable version with diagram:
https://claude.ai/code/artifact/1cc568ab-f9b5-4945-afe4-9cf1c9f55a65 — this file
is the repo copy agents read. What is measured so far lives in
`docs/findings.md`; what to collect next in `docs/experiment_plan.md`.

## The question

Not "which fault is this", but: how far off-nominal is the vehicle right
now, and what does that imply about whether this behavior is unsafe? Every
anomaly is described by three quantities, and all three change the safety
envelope:

| Quantity | Question | Source |
|---|---|---|
| When | Since when can the state / forecast not be trusted? | Behavioral residuals of the learned predictor (any channel) |
| How much | How large is the error (heading, position, speed)? | Cross-source consistency residuals, physical units |
| Which way | Which direction is the true state off? | Sign of the same consistency residuals |

Behavioral residuals compare a forecast with one, possibly corrupted,
source: they show something is abnormal, not by how much (measured:
`findings.md` §3.4). Size and direction need two independent measurements of
the same quantity.

## Layer 1 — multi-source predictor and calibrated scores

One ST-GAT, trained on nominal driving only, predicts 3 s at 10 Hz from the
last 3 s (`st_gat/`, schema selected by `STGAT_GRAPH_FRAME`, see CLAUDE.md).

- Inputs: ego history in the ego's own frame; map graph centred on the
  current position (150 m, past-only), metres + lane direction; route ahead
  (20 points, 3 m apart); tracked objects; perceived traffic-light state;
  redundant ego-state sources (ego_v4: vehicle-reported yaw rate and wheel
  speed, EKF gate Mahalanobis, LiDAR-to-map match probability).
- Outputs: behavior heads (position, velocity, steering, acceleration,
  traffic-light channels) + consistency heads (ego_v6: EKF − vehicle yaw rate,
  EKF − wheel speed; pose yaw rate − vehicle yaw rate; pose along-track and
  lateral speed − vehicle speed, in the estimate's heading frame).
- Scores: every head's residual is a conformal nonconformity score,
  calibrated per horizon step, leave-one-trial-out on nominal trials. With 30
  overlapping forecasts in flight, every instant has ~435 observed (lag, step)
  residual points.
  - When: onset statistic over all channels' recent points.
  - How much / which way: the consistency heads' calibrated, signed residuals.

## Layer 2 — consequence

**2a, nominal envelope.** K = 200 whole-trajectory counterfactuals: forecast
+ one recorded error trajectory each (rotated into the current heading), drawn
only from similar calibration windows (embedding k-NN, or Mondrian zone
group). Tightened by a lane-keeping prior — valid only while the state is
trusted (planned; removes the nominal open-loop tail). No physics trim: it
judges counterfactuals against the measured state, which a fault corrupts.

**2b, evidence-driven uncertainty.** The pose error δ = (ψ, x, y) is
estimated from the consistency gaps (equations in the next section) and each
counterfactual is rigidly rotated by ψ and shifted by (x, y). Mean = which
way, spread = how much (calibrated band), start = when. An
unmitigated fault drives P(violation) up continuously; slowing down halts the
position growth, so mitigation shows as P levelling off. While distrusted the
2a tightening is relaxed. No redundant source (e.g. traffic-light state):
Layer 1 still gives when, the envelope cannot be sized — a flagged, unsized
anomaly, stated as a limit.

**2c, safe sets.** P(violation) = share of counterfactuals that violate, per
type. Acceptance test for every safe set: realized nominal futures must
(almost) never violate.

| Type | Safe set | Status |
|---|---|---|
| Road departure | Footprint (half-width 0.948 m) inside union of road lanelets (signed distance field) | Built |
| Object conflict | No overlap with tracked objects' constant-velocity footprints | Built |
| Red-light running | Do not cross the governing stop line while the true light is red | Planned |
| Speed | Below lanelet limit + margin | Planned |

Alarm thresholds: per variant, firing on 1% of nominal windows (matched
false-alarm rate). Lead time: onset of the alarm episode still active at the
real violation (gaps ≤ 1 s merged).

## Formal model and consistency checks

Every mechanism that moves the envelope has to follow from the equations
below. A change that cannot be stated as one of them is a patch.

**Pose error.** True pose (p, θ); the vehicle's own estimate (p̂, θ̂). The
error, expressed in the estimate's heading frame R(θ̂):

```math
\psi = \hat\theta - \theta, \qquad
\begin{bmatrix} x \\ y \end{bmatrix} = R(\hat\theta)^\top (\hat p - p)
```

**Measurements.** The estimate's own rates (differenced pose: yaw rate
r̂, velocity (â, l̂) along and across its heading) and the vehicle's own
report (yaw rate r_v, wheel speed v; lateral speed v_l ≈ 0). The pose
estimate includes every LiDAR scan-match correction, so its rates do too.
Consistency gaps (the ego_v6 heads' targets):

```math
g_\psi = \hat r - r_v, \qquad g_a = \hat a - v, \qquad g_l = \hat l - v_l
```

**Error dynamics** (exact kinematics, no tuning; the true velocity is
(v, v_l) in the true heading frame, hence R(−ψ) in the estimate's frame):

```math
\dot\psi = g_\psi - b_\psi + n_\psi, \qquad
\dot x = g_a + v(1-\cos\psi) + \hat r\, y, \qquad
\dot y = g_l + v \sin\psi - \hat r\, x
```

b_ψ and n are the redundant sensor's bias and noise. Their effect on an
integral over τ seconds is what the nominal drift bands b(τ) measure
(heading 1.1° at 3 s, 1.8° at 10 s; lateral 0.17 m / 0.43 m). The
r̂·x, r̂·y terms come from the rotating frame and are dropped in the code
(second order while the errors are small).

**Absolute reference.** LiDAR-to-map matching is the one source that
observes δ directly: when it is healthy, δ ≈ 0 within the nominal band. Its
health h(t) (EKF gate Mahalanobis distance, scan-match probability, both
judged against nominal quantiles) sets how much it can be trusted.

**One estimator for all three behaviours.** A filter on δ: predict with the
dynamics above (process noise Q from the drift bands), update with the
pseudo-measurement δ = 0 whose noise R(h) is small when the reference is
healthy and infinite while Layer 1 distrusts it:

```math
\hat\delta_{k+1}^- = \hat\delta_k + f(\hat\delta_k, g_k, v_k)\,\Delta t, \quad
P^-_{k+1} = F P_k F^\top + Q, \quad
K = P^- (P^- + R(h_k))^{-1}, \quad
\hat\delta_{k+1} = (I - K)\,\hat\delta_{k+1}^-
```

Implemented as `pose_error_filter` in `layer2_envelope.py` (default since
2026-10-09; health weight graded between the nominal median and 1% tail of
the EKF gate distance and scan-match probability, independent of Layer 1
distrust). It replaced three hand-made mechanisms: heading integrated from
onset, a 3 s sliding window for position, and a binary release after 2 s of
healthy LiDAR. The assumption it rests on held: wherever the true error is
large, LiDAR health is never nominal. Input is the consistency heads'
residual (the head's prediction acts as the bias b); the r̂ terms are still
dropped (tested, no gain). Known limit: a stopped vehicle whose heading
estimate spins (head out of distribution).

**Envelope and risk.** Counterfactual k at window t: forecast μ_t plus a
recorded error trajectory ε_k from the scene neighbours (2a), then moved
by a draw of δ (2b):

```math
\tau_k = R(\psi_k)\,(\mu_t + \varepsilon_k) + (x_k, y_k), \quad \delta_k \sim \mathcal N(\hat\delta_t, P_t), \qquad
P_t(\text{violation}) = \frac{1}{K}\sum_k \mathbf 1[\tau_k \notin \mathcal S_t]
```

**Invariants (the flow check).** Every change is checked against these:

| # | Invariant | Check | Now |
|---|---|---|---|
| I1 | Every score is calibrated leave-one-trial-out on nominal trials | conformal coverage per horizon step | 0.90 at 0.90 |
| I2 | Null preservation: inside the nominal band, 2b does not move the envelope | estimate outside the band on nominal windows | 0.00% |
| I3 | δ follows the error dynamics above, nothing tuned | derivation, as here | filter in place; R0 and the health references set from nominal data |
| I4 | Onset and release use the same evidence | one source of truth for distrust | the estimate is set by LiDAR health alone; Layer 1 decides when the envelope moves |
| I5 | Every safe set passes the realized-nominal test | `--mode check-safe-set` | 0–0.1% |
| I6 | δ̂ tracks AWSIM truth | `heading_tracking.png`, `correction_offline.png` | heading 1.0–2.6°, lateral 0.35–1.2 m left at departure |

## Layer 3 — response hook

Consumes P(violation) per type, its level and its growth rate; graded
responses (slow down, increase gaps, hand over, stop). A working response
shows up in the signal (lower speed flattens 2b's position-error growth).
Scope (signal definition only vs. closed-loop control) is an open decision.

## Code map

| Piece | Where |
|---|---|
| Extraction, schemas | `st_gat/pipeline/` (`config.py` GRAPH_FRAME, `bag_reader.py`, `sequence_builder.py`) |
| Model, training | `st_gat/model/`, `python3 -m st_gat.train --warmup-only --zone-weighted-sampling --selection kinematic` |
| Model gate / ablation table | `experiments/scripts/model_safety_benchmark.py` |
| Layer 1 calibration | `conformal_horizon_calibration.py`, `conformal_mondrian_calibration.py`, `conformal_scene_conditioning.py`, `audit_minority_scenarios.py` |
| Residual triangle | `experiments/scripts/residual_triangle_check.py` |
| Layer 2 (method) | `experiments/scripts/layer2_envelope.py`; helpers and safe-set check in `layer2_consequence_estimation.py`; safe sets in `experiments/lib/margin.py` |
| Highlight figures | `experiments/scripts/plot_highlights.py` |
