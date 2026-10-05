# Layer 2b v2, and why it sent us back to fix the model (2026-10-04)

Supersedes the "remaining work" framing of
`layer2b_wiring_plan_2026-10-04.md` (written earlier the same day): wiring
the existing v1 mechanism to fault data turned up three problems the plan
didn't anticipate, and the last of them was a Layer 1 model problem, not a
Layer 2 one.

## 1. The v1 margin was not a safety set

Acceptance test (new, `layer2_consequence_estimation.py --mode
check-safe-set`): the REALIZED future of a nominal calibration window is,
by definition, safe and physically feasible, so any margin definition must
(almost) never flag it.

| definition | realized nominal futures flagged |
|---|---|
| v1 `margin.py` (nearest edge of any of 5 nearest lanelets < 1.0m, worst-case head-on objects) | **48.6%** (6,915 windows) |
| v2 road: ego footprint vs. drivable area (union of road lanelets, SDF) | 0.0% |
| v2 objects: oriented multi-disc footprints, constant-velocity objects | 0.1% |
| v2 kinematic-feasibility trim (1.0s segments) | 0.0% |

v1's lane term counts the shared line between adjacent lanes and every
crossing lanelet inside an intersection as "a boundary"; its object term
assumes every object and ego close head-on. The 2026-08-21 nominal run's
mean P(violation) = 0.84 measured that definition, not risk. The wiring
plan's regression check ("match the 2026-08-21 numbers") would have locked
it in. Feasibility at 0.3s segments trimmed 2.8% of real trajectories:
pose is forward-filled onto the 10Hz objects clock, so a one-frame timing
slip reads as ~9 m/s^2 phantom acceleration — 1.0s segments fix it.

## 2. Ground truth for real failures already exists

Every bag records `/awsim/ground_truth/localization/kinematic_state`
(fault_log's `gt_pose_never_received` only means the injector missed it at
startup). Three IMU trials have a real ground-truth road departure while
Autoware's estimated pose never leaves the lane:

| trial | fault on | real vehicle leaves road | estimate leaves road |
|---|---|---|---|
| imu_fault_scale/goal_007 | ~51s | ~60s (off-road 64% of trial, up to 7m) | never |
| imu_fault_stuck/goal_007 | ~53s | 60.0s (63%) | never |
| imu_fault_s3/goal_007 | 50s / 78s | 88.6s | never |

Localization error starts growing ~5s after onset; the departure follows
~2-3s later. Lead time can be measured against a real departure, not just
fault onset. Implication: any Layer 2 computed purely in Autoware's
estimated frame is blind to these; only Layer 1's anomaly signal can see
them.

## 3. The wiring plan had no Layer 1 -> Layer 2 channel

A tube built by bootstrapping NOMINAL residuals around the point forecast
cannot widen under a fault. v2 adds a causal coupling: the prediction made
1s ago has had its first second observed; per kinematic series, mean
|residual| / step-wise 90% conformal quantile, max over series, divided by
that statistic's own nominal q90, clipped to [1, 10]. s = 1 on >= 90% of
nominal windows by construction. Static (s = 1) is kept as the control.
Also fixed: v1 bootstrapped residuals in the map frame, so an along-track
speed error from a north-heading window became a lateral error on an
east-heading one — residuals are now moved in the ego heading frame.

## 4. The blocker: the point forecast itself left the road

With the safe set fixed, the nominal P(violation) floor was still ~0.32
(pooled) / 0.13 (embedding pool), making 1%-false-alarm thresholds ~0.99
and matched-FAR lead times meaningless. Cause: **the ST-GAT forecast itself
left the drivable area in 18.6% of held-out nominal windows**. Kalpit's call:
fixing Layer 1 issues inside Layer 2 is patchwork; fix the model, and make
"good enough for safety" a measured gate.

`experiments/scripts/model_safety_benchmark.py` (new; also the ablation
table, `experiments/analysis/model_safety_benchmark/benchmark_table.md`)
diagnosed it:
- 88% of off-road forecasts were on straight roads; 35% of straight-road
  forecasts predicted a >10 deg turn that never happened; 22% of real turns
  were predicted in the wrong direction.
- Swapping each window's map graph for a random other location's raised
  median 3s error only 1.64 -> 1.86m; zeroing the route flag: 1.64 -> 1.67m.
  The model had a road-network prior but no way to place itself on it.

Root cause (bbox_v1 graph schema): node x/y normalized to [0,1] inside each
graph's own bounding box (x and y separately — destroys metric scale and
angles), ego history in metres-from-now in a different frame, and
`graph_bounds` (the only link between them) never fed to the model. Also a
future leak: the graph was centred on the midpoint of the window's first
frame and its last FUTURE frame.

## 5. ego_v2 graph schema (stages A-C)

`STGAT_GRAPH_FRAME=ego_v2` (config.py; default stays `bbox_v1` so the
canonical stack is untouched; ego_v2 data/checkpoints/models live under
`h30_30_ego/`):
- **A** nodes in real metres, converted per window to the ego frame
  (relative to `position_ref`, rotated by `heading_ref` = localized yaw,
  fixed 50m scale); the model rotates its position history into the same
  frame and rotates `position_mean` back, so every downstream consumer is
  unchanged.
- **B** graph centred on the CURRENT position (past-only), same 150m
  radius — still far beyond the ~45m reachable in 3s, so forecasts always
  have map to land on (Kalpit's requirement: don't starve the forecast of
  map ahead while removing the leak).
- **C** per-node lane direction (centerline tangent).

Retrained with the canonical recipe (`--warmup-only
--zone-weighted-sampling`); val loss bottomed at epoch ~11 and rose after
(overfitting; early stopping kept epoch 11).

| metric | bbox_v1 canonical | ego_v2 A+B+C | gate |
|---|---|---|---|
| FDE p50 | 1.60m | 0.73m | |
| lateral error 3s p50 / p90 | 0.98 / 4.04m | 0.15 / 0.85m | <= 0.5 PASS |
| off-road forecasts | 18.6% | 3.6% (tl_zones 15%) | <= 2% FAIL |
| phantom turns | 34.9% | 0.9% | <= 5% PASS |
| wrong-direction turns | 21.7% | 0% | <= 5% PASS |
| under-turned turns | 25.6% | 63% | |
| map reliance (FDE delta, random graph) | +0.19m | +0.53m | |
| route reliance (FDE delta, zero path flag) | +0.02m | +0.04m | |

Layer 2 nominal floor with ego_v2 (stride 25): mean P(violation) 0.32 ->
0.09 (pooled), 0.125 -> 0.042 (embedding); embedding windows with P > 0.1:
28% -> 8.5%. The p99 is still ~0.95, so matched-FAR thresholds are still
too high. The remaining tail is intersections: the model now knows where
the road is but not which branch the route takes, and hedges straight.

## 6. Next: stage D (explicit route-ahead input) + loader speed-up

Next ~60m of planned-route centerline as a short ego-frame polyline with
its own encoder, instead of a per-node flag pooled across ~1000 nodes. The
AV always knows its own mission route, so this is a legitimate input.
Before that: vectorize graph-tensor building (per-sample Python node loop +
networkx dense adjacency; ~9.5 min/epoch now).

## Results so far with the bbox_v1 model (5 trials, for the record)

At matched 1% nominal FAR almost everything missed (thresholds ~0.99; the
adaptive variants' threshold was 1.0, so they could never fire).
Qualitatively, in imu_fault_stuck both P(violation) and Layer 1's s(t)
jump ~58s, ~2s before the real departure at 60.0s, and stay quiet before
the fault and on the no-departure controls (imu_fault_s1,
tl_fault_fixed_070). Plots: `experiments/analysis/layer2_consequence_estimation/`.
