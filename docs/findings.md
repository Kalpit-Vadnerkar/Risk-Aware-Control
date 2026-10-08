# Findings — what we know (as of 2026-10-07)

One place for current conclusions and their numbers. Design: `architecture.md`.
Next experiments: `experiment_plan.md`. Dated narratives behind these lines
are in `docs/archive/` (historical, not maintained — read only to trace a
decision). When a finding changes, edit it here; don't add a new dated note.

## 1. Data, simulation, ground truth

- **Ground truth in every bag.** AWSIM true pose
  (`/awsim/ground_truth/localization/kinematic_state`), unfaulted and faulted
  traffic-light topics, `/diagnostics` (EKF gate Mahalanobis, NDT match
  probability, Autoware's gyro-bias and pose-instability monitors), vehicle
  speed / yaw rate / tire angle. Raw unfaulted IMU is recorded too but counts
  only as ground truth (a real car wouldn't have it).
- **Stale route (fixed for ego schemas, 2026-10-05).** In 38/45 nominal bags
  and ~half the fault bags the first route message is the previous goal's;
  the real one arrives ~1.2 s later and repeats unchanged. bbox_v1 data (the
  canonical model) still uses the stale route — path flags and the TL
  expectation input were wrong for those trials.
- **Frame timing.** Pose is forward-filled onto the ~10 Hz object clock; a
  one-frame slip is ~1.1 m at 11 m/s. Anything differentiating positions
  needs ≥ 1 s windows (0.3 s windows read ~9 m/s² of phantom acceleration).
- **Held-out means "the deployed model never trained on it",** anchored to
  the model's training date, not today's CAL_DIR membership (bug found and
  fixed 2026-10-04 in the TL severity analysis).
- **Operating envelope of all data so far:** one speed (11.11 m/s map limit),
  one traffic density (10 NPCs), Arm A (Autoware safety gate relaxed) only.
- **Arm A config:** the MRM wedge (emergency state never clearing after a
  stop) is avoided by dropping perception/planning from the autonomous-mode
  requirement (`autoware-awsim.yaml` override). Arm B (stock gate, the
  intended oracle) has never been run with fault injection.
- **Collection hazards:** every 2026-10-03/04 failure was at goal_026 (not
  root-caused; may just be batch order); AWSIM can freeze in Park while
  Autoware reports healthy (restart AWSIM, not only Autoware).
- **Pipeline fixes still in force** (2026-08): frame-gap contiguity gate,
  stopped-period filter removed (it deleted fault signatures), graph node cap
  1024, extraction 47x faster, training ~19x faster (cached graph tensors).

## 2. The predictor (ST-GAT)

Benchmark table (held-out CAL, every model):
`experiments/analysis/model_safety_benchmark/benchmark_table.md`.

| Model | 3 s error p50 | 3 s lateral p50 / p90 | Off-road forecasts | Wrong-direction turns | Map reliance |
|---|---|---|---|---|---|
| bbox_v1 (canonical) | 1.60 m | 0.98 / 4.04 m | 18.6% | 21.7% | +0.19 m |
| ego_v3 | 0.63 m | 0.13 / 0.71 m | 3.3% | 0% | +0.54 m |
| ego_v4 (sign-bug inputs), kinematic selection — first to pass the gate | 0.58 m | 0.11 m (p50) | 1.7% | 0% | — |

- **Why bbox_v1 failed:** graph nodes normalized to each graph's own box,
  no link to the ego's position, graph centred on a future point — the model
  couldn't place itself on the map (random-location graph barely changed it).
- **What fixed it:** ego-frame map, past-only graph centre, lane direction,
  route fix, route-ahead polyline, normalized checkpoint selection (the old
  criterion was dominated by the traffic-light heads, ~20x position's scale,
  which overfit within a few epochs).
- **ego_v4 (2026-10-07):** + redundant ego-state inputs and a consistency
  head (EKF − vehicle yaw rate / speed). **First version had a sign bug:**
  AWSIM's `heading_rate` has the opposite sign to ROS (corr −0.998 with the
  pose-derived yaw rate), so the "gap" was EKF + true yaw rate ≈ 2x the yaw
  rate in every turn; the head's 0.97 correlation was it predicting that.
  Its benchmark numbers (below) are still valid for forecasting; its
  consistency evidence is not. Fixed in `bag_reader.py`; retrained model
  pending. Selection must ignore the traffic-light heads
  (`--selection kinematic`): with them in, the kept epoch had under-trained
  position (under-turn 93%); without, 20%. Single seed each.
- **Remaining forecast error** is footprint clipping by ~0.3 m at 13 tight
  traffic-light-zone spots (real clearance 0.33–0.75 m); single-seed runs
  vary ±0.5–1 pp off-road.
- **Distributional heads don't calibrate.** Jointly trained Student-t / NLL
  heads never converged (coverage max-gap 0.066–0.376 vs 0.004–0.011 for
  conformal); the NLL machinery is paused, not deleted.
- **Epistemic disagreement** between independently trained predictors
  widens with horizon only for position (partly acceleration).

## 3. Layer 1 — calibration and detection

1. **Leave-one-trial-out cross-conformal** (windows overlap 59/60 frames, so
   a per-window split leaks). Coverage 0.896–0.900 for every feature and
   horizon (ego_v3, 17 CAL trials).
2. **Pooled coverage hides per-scenario gaps** — always run
   `audit_minority_scenarios.py`. ego_v3 steering: 0.80 in turns, 0.99 on
   open road. **Mondrian (zone-grouped) closes it:** every group x feature
   0.885–0.914. Embedding k-NN (k = 150) adapts the right way (steering in
   turns 0.68 → 0.92) but under-covers overall (0.857) — revisit k.
3. **Width vs the canonical model** (same 17 trials): position −47% at 3 s
   (5.25 → 2.79 m), steering −23%, longitudinal velocity +27%, acceleration
   +14% (canonical is favoured: it trained on some of these trials).
4. **Fault visibility** (active vs before-first-fault exceedance, 8 original
   campaigns): ego_v3 position 28x (IMU) / 75x (TL); canonical 7x / 4x. The
   August "13.66x acceleration" headline used a "clean" baseline that counted
   post-fault stuck periods.
5. **Behavioral residuals give WHEN, not how much / which way.** Recent
   residuals turn significant 2.1–4.1 s before real departures (within
   ~0.5–1 s of a physics check), but underestimate the pose error ~10x
   (heading +8° vs true −60°) and their sign relation to the truth flips
   between trials. The estimated trajectory cannot reveal its own error.
6. **Redundant sources carry the size.** Before the imu_fault_stuck
   departure (60.0 s): EKF gate Mahalanobis 7.0 (nominal ~0.5–1), NDT match
   2.3 (nominal ~6). Estimated vs physical (bicycle-model) heading change
   correlate 0.999 nominally. (A "0.35 rad/s yaw-rate gap" reported earlier
   was contaminated by the heading_rate sign bug, §2.)
7. **Sign/units of every redundant source must be checked against the
   pose** before use: AWSIM `heading_rate` is sign-flipped vs ROS.

## 4. Layer 2 — consequence

- **Safe sets must pass the realized-nominal test.** The first lane margin
  flagged 48.6% of real nominal driving (its 0.84 nominal P(violation) was the
  definition, not risk). Drivable-area SDF: 0%; object footprints: 0.1%.
- **Bootstrap in the ego heading frame** (map-frame residuals turned
  along-track errors into lateral ones).
- **Open-loop tail.** Normal driving at tight spots gives P ≈ 1 because
  counterfactuals ignore lane-keeping; it sets every 1%-false-alarm threshold
  (~0.82–0.93). Fix planned: lane-keeping prior under trust.
- **Coupling Layer 1 into the envelope** (4 real IMU departures, 1% nominal
  false-alarm rate):

| Coupling | Uses | Departures caught | Lead |
|---|---|---|---|
| Static (none) | — | 0/4 | — |
| Multiply motion residuals by anomaly | behavioral | 0/4 | — (trimmed as infeasible; 10x0 = 0 when stopped) |
| Residual-triggered trust region (omega 0.03–0.15 rad/s) | behavioral | 0/4 | — (expansion raises nominal and fault P together) |
| Dead-reckoning pose correction (baseline) | redundancy (hand-built) | 4/4 | 1.8–3.0 s |
| Layer 1 alone (no map) | behavioral | 3/4 | 0.2–3.0 s |

- No alarms of note on traffic-light trials (in-fault ≤ 0.07). One IMU
  trial without departure alarms 55% of its fault windows — its real
  footprint came within ~5 cm of the edge.
- **Physics trim removed** (default off since 2026-10-07): it never trimmed
  nominal futures and only broke the residual-scaling coupling.

## 5. Faults and experiments

- **Families:** IMU (bias 0.03 / 0.08 rad/s, scale 1.8, stuck) and
  traffic-light perception (oscillate, unknown, blackout, confidence ramp,
  fixed confidence 0.3/0.5/0.7). "Camera fault" here can only mean
  traffic-light perception (object detection is LiDAR-only).
- **Consequences so far:** 4 real road departures in 38 fault trials, all
  IMU, goal_007/012. IMU consequences keep growing past 120 s (EKF
  corruption doesn't self-heal).
- **Every traffic-light fault mode is fail-safe by Autoware's design**
  (unknown / blackout / low confidence → stop): 0/30 consequences. A red
  reported as green has never been injected.
- **TL severity sweep** (fixed severities, matched nominal controls,
  2026-10-03/04): collected and analysed, results in
  `experiments/analysis/tl_severity_sweep/`; goal_026 vs ramp-pilot check
  still open. Severity sweeps use fixed levels: the ramp confounds severity
  with time-in-zone.
- **Injection hazards:** a wrong topic name once made a whole TL campaign a
  silent no-op — verify with `compare_fault_vs_nominal.py`; zone files must be
  regenerated when goals change.

## 6. Superseded (don't revive without a new reason)

| Idea | Why dropped |
|---|---|
| Active control "RISE" as the contribution | Control/handling, not safety evidence; Layer 3 scope still open |
| Closed-set fault classification, "belief divergence" | Needs fault data per class, says nothing about novel failures |
| SPRT evidence signal | Saturated on nominal by the TL-discrepancy branch |
| Jointly trained NLL / Student-t heads | Never calibrated; conformal replaced them |
| bbox_v1 graph, first-route-message rule | §2 |
| Static lane margin (v1 Layer 2) | Failed the realized-nominal test |
| Residual-scaling and trust-region couplings, physics trim | §4 |
