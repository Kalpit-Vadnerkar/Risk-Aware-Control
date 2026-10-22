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
- **Coupling Layer 1 into the envelope** (4 real IMU departures, alarms at a
  1% nominal false-alarm rate; earlier couplings, ego_v3):

| Coupling | Uses | Departures caught | Lead |
|---|---|---|---|
| Static (none) | — | 0/4 | — |
| Multiply motion residuals by anomaly | behavioral | 0/4 | — (trimmed as infeasible; 10x0 = 0 when stopped) |
| Residual-triggered trust region (omega 0.03–0.15 rad/s) | behavioral | 0/4 | — (expansion raises nominal and fault P together) |
| Dead-reckoning pose correction (baseline) | redundancy (hand-built) | 4/4 | 1.8–3.0 s |
| Layer 1 alone (no map) | behavioral | 3/4 | 0.2–3.0 s |

- **The architecture method** (`layer2_envelope.py`, ego_v6, 2026-10-08):
  per-channel conformal p-values combined into a bounded score; capped
  CUSUM onset calibrated to <= 6 nominal distrust episodes per hour (4.2/h
  achieved, 0.3% of driving); 2 s recovery test; heading error integrated
  from onset with the pose-consistency channel, along/lateral over a sliding
  3 s window; carried error released once LiDAR-map matching is healthy
  for 2 s. Results, lead time (alarm start after fault onset):

| Departure | Layer 1 alone | 2a+2b evidence | dead-reckoning |
|---|---|---|---|
| s3/goal_007 | 10.0 s (f+28.7) | 2.0 s (f+36.7) | 1.6 s (f+37.1) |
| scale/goal_007 | 6.2 s (f+3.3) | 2.0 s (f+7.5) | 2.4 s (f+7.1) |
| stuck/goal_007 | 4.2 s (f+3.2) | 3.0 s (f+4.4) | 3.0 s (f+4.4) |
| stuck/goal_012 | 3.6 s (f+22.8) | 3.4 s (f+23.0) | 3.4 s (f+23.0) |

  Harmless IMU faults (s1/007, s1/012, s3/012, scale/012), share of fault
  time in alarm: Layer 1 0.98/0.98/0.99/0.21 (the anomaly is real),
  evidence 0.18/0.00/0.18/0.00, dead-reckoning 0.05/0.00/0.59/0.00.
  Heading error tracks AWSIM truth closely (stuck: +115 deg vs +115 true).
  dP (evidence - nominal) misses scale/goal_007 because P is already high
  at that tight spot before the fault.
- **Pose-error estimator from the formal model** (2026-10-09, default in
  `layer2_envelope.py`, `--estimator legacy` for the above). One filter on
  (psi, x, y): predict with the consistency gaps, update with delta = 0 from
  LiDAR-to-map matching weighted by its health. Assumption checked first:
  in every window with true error > 5 deg or > 1 m, LiDAR health is never
  nominal (0/6 faults); small-error windows are healthy 66–97%.
  - vs the legacy mechanisms, against AWSIM truth: nominal envelope moved
    0.00% of windows (was 0.30%); harmless s1 max lateral error 1.45 ->
    0.44 m, heading 3.4 -> 2.7 deg; departures heading 1.0–2.6 deg left,
    lateral 0.35–1.2 m left (legacy 0.17–1.1 m). The lateral floor is
    insensitive to how strictly LiDAR is trusted: a measurement limit.
  - Integrating the raw gap (minus nominal bias) instead of the head's
    residual, and the rotating-frame terms: tried, worse overall (nominal
    moved 2.9%, departures worse); fixes only s3/goal_012.
  - Known limit: s3/goal_012, vehicle stopped while its heading estimate
    spins past 180 deg; the consistency head is out of distribution
    (lateral error up to 5.8 m).
  - Full run: departures unchanged (4/4, same leads); harmless in alarm
    s1/007 0.18 -> 0.04, s1/012 0, scale/012 0, s3/012 0.18 -> 0.65 (the
    nominal envelope alone reads 0.63 there: stopped at a tight spot;
    legacy's lower value came from its drift moving the envelope away).
- **Error dynamics checked at the derivative level** (2026-10-22,
  `pose_error_derivative_check.py`, raw bags vs AWSIM truth, 8 IMU trials).
  The formal model is exact: driven by AWSIM's true twist, predicted vs true
  error rate differs by ~0.01 m/s. The divergence comes from the gap
  INPUTS, two systematic biases:
  - Lever arm: AWSIM's velocity report is the rigidbody (centre-of-mass)
    velocity (`Vehicle.cs` `m_rigidbody.velocity`), the pose is base_link:
    v_l,report = v_l,base_link + 1.18 m x r on every trial (residual 0.10 ->
    0.003 m/s RMS). Uncorrected: ~1.8 m of phantom lateral error per 90 deg
    of turning.
  - Window lag: the pipeline gaps compare a 1 s (0.5 s yaw) backward pose
    difference with the instantaneous report, i.e. ~half-window x
    acceleration (0.4 m/s along-track while launching from a stop).
  Open loop from the fault onset, no LiDAR updates, 30 s, max error vs truth:
  current pipeline gaps up to 5.1 deg / 4.2 m along / 4.5 m lateral; per-step
  gaps (one-sample pose increment minus the report averaged over that sample,
  lever-corrected, increments telescope so jitter cancels) <= 0.8 deg, along
  <= 0.6 m, lateral <= 0.6 m, except scale/goal_007 (3.3 m along, 1.4 m
  lateral at 28 deg heading error).
  Through the filter (`step_gap_filter_check.py`, same filter, LiDAR health
  and R0; each input with its own nominal drift band; step gaps minus their
  nominal mean), error left after correction, heading / lateral:

  | Trial | Autoware | head residuals (current) | corrected step gaps |
  |---|---|---|---|
  | s3/007 at departure | 23.2° / 2.73 m | 1.0° / 0.96 m | 0.0° / 0.35 m |
  | scale/007 at departure | 22.7° / 3.46 m | 1.3° / 1.19 m | 0.1° / 0.23 m |
  | stuck/007 at departure | 94.8° / 5.80 m | 1.1° / 0.84 m | 0.8° / 0.43 m |
  | stuck/012 at departure | 26.2° / 0.82 m | 2.5° / 0.35 m | 0.4° / 0.20 m |
  | s3/012 max (stopped, heading spins) | 179.9° / 1.88 m | 10.7° / 5.79 m | 0.5° / 2.94 m |
  | scale/012 max (harmless) | 24.5° / 1.95 m | 4.3° / 1.23 m | 0.4° / 0.25 m |
  | s1/007 max (harmless) | 2.2° / 0.16 m | 2.7° / 0.44 m | 1.0° / 0.36 m |
  | s1/012 max (harmless) | 2.2° / 0.08 m | 2.1° / 0.28 m | 1.6° / 0.14 m |

  Nominal (17 cal trials) |estimate − true error| p99: head 0.53° / 0.11 m,
  step 0.34° / 0.07 m, Autoware's own error 0.32° / 0.065 m. Nominal drift
  band at 3 s: heading 1.09° → 0.11°, lateral 0.17 → 0.08 m. I2 against the
  current band: 0.0% both; against step's own (10x tighter, 95%) band 6.4% —
  the "envelope moves" gate needs re-deriving for the new input (e.g. a
  nominal p99.9 of the estimate: 0.27° / 0.11 m). Small-fault caveat
  unchanged: on s1/007 correcting still raises lateral error (0.16 → 0.36 m).
  Full pipeline (`layer2_envelope.py --gap-source step` vs `head`, ego_v6,
  all 38 fault trials, 2026-10-22; outputs `layer2_envelope_{head,step}/`):
  nominal thresholds identical (1% FAR: 0.825), distrust 4.2 episodes/h
  both, envelope moved on 0.30% (head) / 0.51% (step, gate = nominal
  p99.9 of the estimate: 0.29 deg / 0.11 m) of nominal windows. Departures
  4/4 both; evidence lead s3/007 2.0 -> 2.2 s, scale/007 2.0 -> 2.2,
  stuck/007 3.0 -> 2.8, stuck/012 3.4 -> 3.4. Harmless IMU faults, share of
  fault time in alarm: s3/012 0.65 -> 0.24 (nominal envelope alone 0.63:
  the corrected envelope sits on the true pose, away from the edge), s1/007
  0.04 -> 0.06, s1/012 and scale/012 0 both. Traffic-light trials unchanged
  (<= 0.12). Lead time is set by Layer 1 onset and the geometry, not by the
  size estimate; the gain is accuracy (correction) and fewer false alarms.
- **Step filter: rotating-frame terms and midpoint step** (2026-10-22).
  Re-scored with RMS over the fault window (onset to departure, else fault
  end) instead of one instant at departure, which hid e.g. s3/007 lateral
  rising 0.35 -> ~1 m a second later. Diagnosis (offline, same gaps):
  - r_hat = the estimate's own per-step yaw rate restores the r_hat*x,
    r_hat*y terms ("no gain" before only because the gaps were biased):
    s3/012 lateral RMS 1.48 -> 0.35 m, scale/007 at departure 0.23 -> 0.07 m;
    nominal unchanged.
  - Midpoint step: stuck/007 open-loop lateral 0.40 -> 0.20 m (Euler error
    when the heading error grows ~13 deg/s); neutral elsewhere.
  - Open loop from onset (no LiDAR update): heading 0.1–0.4 deg RMS on
    every trial, lateral drifts ~1.4 m over a 190 s fault: the LiDAR update
    is needed for long faults, and a slow lateral gap drift remains
    (candidate: estimate it as a filter state).
  - The update's assumption "LiDAR healthy => delta = 0" is wrong on small
    faults (s1: 2 deg true while healthy); no update while Layer 1
    distrusts fixes s1 heading (0.5 -> 0.1 deg) but raises lateral. Proper
    fix needs the NDT pose (EKF - NDT measures delta): now in
    `RECORDING_TOPICS` (`/localization/pose_estimator/pose_with_covariance`,
    + transform_probability, NVTL); absent from all existing bags.
  RMS over the fault window, error left (heading deg / lateral m),
  `step_gap_filter_check.py` (step = r_hat + midpoint, now the step default):

  | Trial | Autoware | head residuals | step gaps |
  |---|---|---|---|
  | s3/007 (departs) | 5.9 / 0.49 | 0.8 / 0.42 | 0.2 / 0.16 |
  | scale/007 (departs) | 9.4 / 1.07 | 1.4 / 0.65 | 0.1 / 0.04 |
  | stuck/007 (departs) | 29.1 / 1.67 | 0.7 / 0.20 | 0.4 / 0.07 |
  | stuck/012 (departs) | 3.2 / 0.09 | 0.5 / 0.08 | 0.1 / 0.04 |
  | s3/012 (harmless) | 82.3 / 0.91 | 4.2 / 1.99 | 0.3 / 0.35 |
  | scale/012 (harmless) | 5.7 / 0.30 | 1.1 / 0.34 | 0.2 / 0.03 |
  | s1/007 (harmless) | 1.3 / 0.06 | 1.1 / 0.19 | 0.5 / 0.15 |
  | s1/012 (harmless) | 1.2 / 0.03 | 1.1 / 0.06 | 0.7 / 0.04 |

  At departure (s3/007, scale/007, stuck/007, stuck/012): lateral 0.29 /
  0.12 / 0.24 / 0.14 m (head 0.96 / 1.19 / 0.84 / 0.35). Lateral is still
  worse than no correction on s1 (small faults, see the update assumption).
  Full pipeline rerun with the step filter (`--gap-source step`, 38 fault
  trials, outputs `layer2_envelope_step/`): nominal threshold 0.825, 4.2
  distrust episodes/h, envelope moved 0.51% (gate 0.29 deg / 0.10 m), all
  unchanged. Departures 4/4, evidence lead s3/007 2.2 s, scale/007 2.2,
  stuck/007 2.8, stuck/012 3.4 (unchanged). Harmless, share of fault time in
  alarm: s3/012 0.24 -> 0.00 (head 0.65; the lateral estimate now follows
  truth through the spin), s1/007 0.06, s1/012 0.00, scale/012 0.00.
  Traffic-light trials unchanged (<= 0.12). Timeseries panel 2 now shows
  the step gaps (what 2b integrates) in step mode.
- **Offline correction** (open loop, estimator above): subtracting the
  estimate at the moment of departure leaves heading 1.0–2.6 deg (Autoware
  23–95 deg) and lateral 0.4–1.2 m (0.8–5.9 m). On harmless s1 it raises
  lateral error 0.1 -> 0.4 m: correct only when Layer 1 distrusts.
  Figure: `experiments/analysis/h30_30_ego6/highlights/correction_offline.png`.
- **Alarm floor** (internal benchmark): at the 1%-of-windows threshold the
  nominal envelope fires ~35 episodes/h at tight spots (P_evidence =
  P_nominal on 99.96% of nominal windows). Alarming only while Layer 1
  distrusts: 0 episodes in 0.96 h, 4/4 departures (rule chosen after
  seeing them). `plot_highlights.py`.
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
