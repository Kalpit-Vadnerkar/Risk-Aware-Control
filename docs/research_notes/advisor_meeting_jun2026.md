# Advisor Meeting — June 2026

**Date:** 2026-06-25
**Attendees:** Kalpit Vadnerkar, Prof. Pierluigi Pisu
**Status:** Decisions recorded here supersede all prior direction documents

---

## Context Going In

Presented preliminary results from Q1 analysis (obs_recovery vs. obs_noescape, 18 runs each).
Key finding: max combined NLL separates the two conditions (p=0.034), but full-run CVaR does not.
Also presented the three-signal classifier framework and the advisor presentation slides.

---

## Decisions and Direction Changes

### 1. Traffic Light as Explicit Differentiator Signal

**Question raised:** Could including traffic light existence, detection confidence, and
state (red/yellow/green) provide a better signal to distinguish planned stops from
obstacle-induced stops?

**Answer:** Yes — this directly addresses the identifiability problem. The current model
has a single binary traffic light feature. A richer representation (TL present, TL
detected, TL state) would allow the model to predict "vehicle will stop because the
light is red" as nominal behavior, reducing false positives from planned stops.

**Action:** Plan richer traffic light feature extraction in the data pipeline. This
requires model retraining with additional input channels. Not immediate — design first,
then collect data, then retrain.

---

### 2. Record Avoidance Decision Timestamp

**Question raised:** At what moment does Autoware (with auto avoidance policy) actually
decide to perform avoidance — i.e., when does the planned trajectory change?

**Why it matters:** Currently we observe the NLL spike AFTER the physical lane change
begins (reactive). The planning decision happens earlier. Recording the planning
decision timestamp reveals the lead time between framework detection and Autoware's
internal decision, which is the key metric for whether the framework can be proactive.

**Implication:** New data collection is required. Current rosbags likely do not record
the behavior_path_planner output topic at sufficient resolution. New runs on the new
machine should subscribe to:
- `/planning/scenario_planning/lane_driving/behavior_path_planner/output/path`
- or `/planning/behavior_path_planner/output/path` (version-dependent)
Detect lateral path changes (planned trajectory shifts laterally) and record timestamp.

---

### 3. Confidence Values, Not Just a Threshold

**Feedback:** A threshold-based binary trigger is insufficient. The framework must
express how reliable its signal is — a confidence value.

**Why:** Different fault conditions degrade the signal differently. A camera fault that
corrupts traffic light state recognition may cause many false positives. An IMU fault
may corrupt the ego state features that the ST-GAT relies on. Knowing the fault type
and magnitude should map to a confidence value: P(classification correct | fault).

**How:** Collect behavior data under camera and IMU faults (see section 4). Compute
classification accuracy under each fault condition. The resulting table of
(fault_type, magnitude) → accuracy is the confidence function. This mirrors the
T-ITS 2025 paper's approach to computing fault detection confidence.

---

### 4. Fault Scope: Camera and IMU Only

**Decision:** Focus fault experiments on two sensor types:

**Camera:**
- Affects traffic light recognition pipeline (not object detection, which is primarily
  LiDAR-based in Autoware)
- Faults to simulate: traffic light state dropout (camera cannot see light), wrong
  state (camera reports wrong color)
- Implementation: intercept `/perception/traffic_light_recognition/traffic_signals`
  topic and inject faults (drop messages, substitute wrong state)

**IMU:**
- Affects localization/state estimation that produces ego kinematics (position,
  velocity, heading) fed to the ST-GAT as features
- Faults to simulate: additive Gaussian noise on angular velocity and linear acceleration
- Implementation: intercept `/sensing/imu/imu_data` and add noise before it reaches
  the localization stack
- Note: IMU noise propagates through the EKF → corrupts x_var, y_var features
  and the ego state time series → degrades ST-GAT predictions

**Out of scope (deferred):** LiDAR dropout (affects object perception but NPC vehicles
yield to ego in AWSIM, so no real collision threat), GPS spoofing (too complex to
simulate realistically), camera-for-object-detection (AWSIM uses LiDAR primarily).

---

### 5. Nominal Training Data Policy

**Question raised:** Should nominal training data be collected with the "auto" avoidance
policy enabled?

**Decision:** No. Nominal training data should be collected in passthrough mode (no
obstacles, policy irrelevant). If avoidance maneuvers appear in training data, the model
learns them as "nominal" and the residual signal for avoidance detection becomes
weaker.

**Confidence values** come from fault experiments, not from nominal training data. They
are a separate characterization step after the classifier is trained.

---

### 6. CUSUM as Primary Residual Signal

**Feedback:** Choosing combined NLL or max NLL as the single decision signal loses
information. A richer signal is needed for interpretability.

**CUSUM rationale:**
- Accumulates deviations from nominal over consecutive windows (trend-sensitive)
- Does not require a single large spike to trigger — sustained small deviations
  accumulate to a clear crossing
- Direct controls analogy: CUSUM control charts are a standard tool in process
  monitoring that Dr. Pisu will recognize
- Already in the prior paper's residual type list; ready to implement
- More interpretable to the committee than "maximum of a combined NLL"

**Action:** Implement CUSUM residual in st_gat/infer.py alongside existing NLL;
compare discriminability of CUSUM vs. max NLL on existing obs_recovery/obs_noescape data.

---

### 7. New Machine Available

A more powerful PC is available for running experiments. Benefits:
- More consistent timing (fewer dropped frames, less system noise)
- Can run more parallel trials
- Faster data collection

**Action items for new machine setup:**
1. Clone repo (`git clone https://github.com/Kalpit-Vadnerkar/Risk-Aware-Control.git`)
2. Install AWSIM Labs v1.6.1 + Autoware (pre-built)
3. Verify `Run_AWSIM.sh` and `Run_Autoware.sh` paths match new system layout
4. Test one experiment run with passthrough mode before collecting data
5. Re-collect obs_recovery + obs_noescape with avoidance decision timestamps recorded

---

## Updated Research Scope

### Phase 1 — Signal Validation (No Faults)
Validate that each of the three signals correctly identifies scenario type.

| Condition | Status |
|---|---|
| obs_recovery (30m obstacle, avoidance=auto) | 18 runs complete |
| obs_noescape (30m obstacle, avoidance=manual) | 18 runs complete |
| obs_singlelane (30m obstacle, no adjacent lane) | 0 — collect on new machine |
| obs_tooclosetoreact (5–8m obstacle, multi-lane) | 0 — collect on new machine |

⚠ **Route diversity needed:** Current 3 routes (goal_007/011/021) are all similar
road segments in Shinjuku. Need to pick routes that include:
- A guaranteed single-lane section (for obs_singlelane)
- Different traffic light configurations
- At least one longer corridor for obs_tooclosetoreact

### Phase 2 — Fault Robustness (Confidence Values)
Show how signal reliability changes under sensor degradation.

| Condition | Fault | Magnitude | Purpose |
|---|---|---|---|
| nominal + camera_low | TL state dropout | 20% | Signal baseline under mild camera fault |
| nominal + camera_high | TL state dropout | 60% | Signal under severe camera fault |
| obs_recovery + camera_low | TL state dropout | 20% | Avoidance detection under mild camera fault |
| obs_recovery + camera_high | TL state dropout | 60% | Avoidance detection under severe camera fault |
| nominal + imu_low | IMU noise | σ=0.05 rad/s | Signal baseline under mild IMU noise |
| nominal + imu_high | IMU noise | σ=0.25 rad/s | Signal under severe IMU noise |
| obs_recovery + imu_low | IMU noise | σ=0.05 rad/s | Avoidance detection under mild IMU noise |
| obs_recovery + imu_high | IMU noise | σ=0.25 rad/s | Avoidance detection under severe IMU noise |

---

## Open Questions for Next Meeting

1. What is the expected lead time (planning decision → physical maneuver)? If it is less
   than 1 second, the framework's proactive value is limited.
2. Should the confidence function be a lookup table (discrete magnitudes) or a parametric
   curve fitted across magnitudes?
3. Does the committee expect a formal stability proof for the closed-loop system, or is
   empirical validation with conformal coverage sufficient?
