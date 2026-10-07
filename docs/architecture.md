# Pipeline architecture

Current design, 2026-10-07. Shareable version with diagram:
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
  traffic-light channels) + consistency heads (ego_v4: EKF − vehicle yaw rate,
  EKF − wheel speed).
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

**2b, evidence-driven uncertainty.** From the onset time t0, integrate the
calibrated consistency gap e(s) (and its conformal band) into a pose-error
distribution; rigidly rotate/shift each counterfactual:

    dpsi(t) = integral_{t0}^{t} e(s) ds,   dy(t) = integral_{t0}^{t} v(s) sin(dpsi(s)) ds

Mean = which way, spread = how much (calibrated band), start = when. An
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
| Layer 2 | `experiments/scripts/layer2_consequence_estimation.py`, safe sets in `experiments/lib/margin.py` |
