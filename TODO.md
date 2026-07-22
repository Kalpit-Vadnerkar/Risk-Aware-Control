# Risk-Aware Control — Task List

**Last Updated:** 2026-07-22 — direction pivot: calibration/conformal-prediction work
(old Phase 3) is deferred. Immediate priority is an exploratory study of how the
ST-GAT digital twin reacts to IMU and Camera faults, building on the published prior
work (see below). See `docs/research_notes/fault_literature_review.md` for the
literature-grounding pass started this session.

---

## Research Direction (revised 2026-07-22)

**Overarching dissertation goal:** show that this approach can make AVs safer in a
meaningful, measurable way — not just publish a novel algorithm. Everything below is
in service of that; conformal prediction and any other specific mechanism is a
*candidate*, not a commitment.

**Prior published work (context, not this repo):** Vadnerkar & Pisu, "Digital Twins as
Predictive Models for Real-Time Probabilistic Risk Assessment of Autonomous Vehicles,"
IEEE T-ITS, April 2026 (PDF in project root). Trained an ST-GAT purely on nominal
Autoware+AWSIM driving data (same Nishishinjuku map/vehicle setup as this repo),
detected Camera/IMU/LiDAR faults via prediction-observation residuals (Raw, KL
divergence, CUSUM) + PCA + Random Forest, 93.7% accuracy. Traffic Light Status Flag
was the single most discriminative feature (29.7% importance); CUSUM consistently beat
Raw/KL. This repo (`Risk-Aware-Control`) is the follow-on: move from passive fault
*detection* to active risk-aware *control* — using the residual signal to relax a
velocity constraint so the AV can make progress instead of defaulting to a full stop,
pushing toward the Pareto frontier of (safety, progress) rather than the trivially
safe, zero-progress corner.

**Immediate focus (as of 2026-07-22):** before any control/calibration work, run an
*exploratory* study of how the ST-GAT digital twin reacts to IMU and Camera faults —
essentially, does the residual signal actually behave the way the prior paper's
passive-detection result implies it should, now inside this repo's own pipeline and
framing. LiDAR is out of scope for this round. See "Phase 1.5" below.

**Distributions over distribution-free:** the prior paper's ST-GAT already outputs
full predictive distributions (Gaussian mean+variance for continuous features,
Bernoulli probability for discrete ones) via deep ensembles — that's where the
uncertainty and interpretability signal actually lives. The dissertation direction is
to **embrace those distributions directly**, not collapse them into a single
distribution-free conformal bound. Conformal prediction (old Phase 3 below) remains on
the table as *one* possible way to turn a residual into a calibrated guarantee, but
it's no longer the assumed mechanism — alternatives that use the full predicted
distribution (e.g. likelihood-based risk scores, calibrated probabilistic bounds that
still exploit the Gaussian/Bernoulli structure rather than discarding it) should be
considered on equal footing before committing.

**What this is NOT:** Re-implementing Autoware's planner. Autoware already has
binary stop/avoidance behavior. Our signal is continuous and probabilistic — not a
fixed policy, and not required to be distribution-free.

---

## Phase 0: Data Collection ← CURRENT

### 0.1 Nominal Campaign

Nominal data is the calibration foundation for conformal prediction. Direction changed
2026-07-21: all experiments now run at max possible velocity (map limit, 11.11 m/s) —
`nom_v5`/`nom_v7` are no longer needed (dropped from `NOMINAL_DATASETS`, no more
per-speed calibration). `nom_v5`/`nom_v7` rosbags already collected on the P5000 are
kept for reference but are not part of the current plan; the `collect.sh nom_v5/nom_v7`
commands still work if ever needed again.

| Campaign | Status | Command |
|----------|--------|---------|
| `nom_v11` | 🔄 In progress | `./collect.sh nom_v11` |

### 0.2 Obstacle Campaigns

Three obstacle conditions bracket the (solvable, unsolvable, recovery) space:

| Campaign | Status | What it shows |
|----------|--------|---------------|
| `obs_stuck`       | ⏳ Need | Autoware stops — baseline conservative behavior |
| `obs_recovery`    | ⏳ Need | Autoware swerves (policy=auto) — actual recovery |
| `obs_noescape`    | ⏳ Need | Single-lane (LL 241), no path — stopping IS optimal |
| `obs_singlelane`  | ⏳ Need | 30m obstacle, no adjacent lane — Signal 1 validation |
| `obs_tooclosetoreact` | ⏳ Need | 5–8m obstacle, multi-lane — Signal 2 (TTC) validation |

Commands:
```bash
./collect.sh obs_stuck
./collect.sh obs_recovery        # sets policy to "auto", restores on exit
./collect.sh obs_noescape        # verify obstacle lands in LL 241 (single-lane)
./collect.sh obs_singlelane
./collect.sh obs_tooclosetoreact
```

**Before running obs_noescape / obs_singlelane:** Verify obstacle placement lands in
LL 241 by checking rosbag `/tf` ego position when obstacle appears. `min_travel_before_placement: 60.0` + 30m ahead ≈ 90–100m arc — confirm empirically on first run.

### 0.3 Fault Campaigns (IMU + Camera) — feeds Phase 1.5

Needed before Phase 1.5 can run. Infrastructure already exists in
`experiments/lib/fault_injector.py` and `collect.sh` (built from earlier
research-direction work, before the obstacle-scenario pivot — the standalone runner
scripts were deleted in the 2026-07-22 repo cleanup as stale, but the campaign
definitions inside `collect.sh` itself still work):

| Campaign | Status | What it is |
|----------|--------|------------|
| `imu_fault_s1`..`s4` | Exists, not yet run this direction | Gyro bias 0.05→0.60 rad/s, see `fault_injector.py` header |
| `tl_fault_s1`..`s4`  | Exists, not yet run this direction | TL confidence/oscillate/unknown/blackout, see `fault_injector.py` header |

**Open question (see `docs/research_notes/fault_literature_review.md`):** the existing
"camera fault" campaigns operate on the traffic-light *classification output*, not raw
camera pixels — closer to the T-ITS paper's own dominant fault-detection feature (TL
Status Flag, 29.7% importance) than it first looks, but not a literal camera-sensor
fault. Decide whether that's sufficient for this study or whether a raw-image-level
fault (soiling/occlusion mask on the camera topic, closer to the paper's Gaussian
pixel-noise approach) is worth adding first.

---

## Phase 1: ST-GAT Training

Port and train the model from the T-ITS paper. Reference implementation is
READ-ONLY at `../Graph-Scene-Representation-and-Prediction/` — code we need from it
(`Point`, `GraphBuilder`, `MapProcessor`) is copied into `st_gat/pipeline/Data_Curator/`
and `st_gat/pipeline/State_Estimator/` — same package/class names as the original
(it's Kalpit's own prior codebase), just not imported live from that repo.

Work goes in `st_gat/` within this repo.

### 1.1 Data Extraction

- [ ] Write `st_gat/extract.py`: reads rosbag files from `experiments/data/baseline_all/`
      and `nom_v11/`, extracts per-timestep features:
      position(2), velocity(2), steering(1), accel(1), obj_distance(1), traffic_light(1)
- [ ] Output: `st_gat/data/nominal_features.pkl` — one row per timestep, labeled by
      run_id
- [ ] Verify feature distributions look sane (no NaNs, velocity capped at expected values)

### 1.2 Model Training

- [ ] Write `st_gat/config.py`: hyperparams matching T-ITS paper
- [ ] Write `st_gat/train.py`: train ST-GAT on nominal data (baseline_all + nom_v11),
      80/20 train/val split
- [ ] Train and save model checkpoint to `st_gat/checkpoints/`
- [ ] Verify residuals are low on held-out nominal runs (raw residual < 1σ in majority
      of timesteps)

**Success criterion:** On held-out nominal data, mean raw residual < 0.5 m/s for
velocity features, < 1.0m for position features.

### 1.3 Residual Computation

- [ ] Write `st_gat/residuals.py`: run trained model over all experiment rosbags,
      compute per-timestep Raw, KL, and CUSUM residuals
- [ ] Output: one residual CSV per run in `st_gat/residuals/`
- [ ] Plot residual traces for:
      - A clean nominal run (should be flat near zero)
      - An obs_stuck run (should spike when obstacle appears)
      - An obs_recovery run (should spike, then return to nominal)
      - An obs_noescape run (should spike and stay elevated — no escape)

---

## Phase 1.5: Exploratory Fault-Reaction Study ← NEXT (added 2026-07-22)

**Goal:** before committing to any control mechanism, actually look at how the ST-GAT
digital twin's residuals (and its full predicted distributions — mean/variance,
Bernoulli probabilities) react to IMU and Camera faults inside this repo's own
pipeline. This is deliberately exploratory, not hypothesis-confirming — the point is
to build intuition and figure out what the distributional signal looks like before
deciding how to formalize it into a risk/control mechanism (conformal or otherwise).

- [ ] Read `docs/research_notes/fault_literature_review.md`, resolve its open
      questions (camera fault definition, accel_bias_ms2 no-op) before running fault
      campaigns
- [ ] Run `imu_fault_s1`..`s4` and `tl_fault_s1`..`s4` campaigns (0.3 above)
- [ ] Run trained ST-GAT (Phase 1.2) over nominal + fault-condition rosbags, compute
      full predictive distributions and residuals (Raw/KL/CUSUM, matching the T-ITS
      paper's three types) per timestep
- [ ] Plot: predicted mean ± variance vs. observed value, through a fault window
      (onset → sustained → recovery) for both IMU and Camera/TL conditions — does the
      predicted variance itself widen under fault, independent of the residual?
- [ ] Compare against the T-ITS paper's finding (CUSUM > KL > Raw, TL Status Flag
      dominant) — does that hold in this repo's pipeline, or does the shift from
      obstacle-scenario framing change which features/residuals matter?
- [ ] Write up findings in `docs/research_notes/` — this is the evidence base the
      eventual risk/control mechanism gets designed against, not a mechanism itself

---

## Phase 2: Signal Validation

**Goal:** Confirm the residual signal empirically differentiates solvable from
unsolvable scenarios before building any control logic. Written against the obstacle-
scenario framing (Phase 0.2) — revisit after Phase 1.5, which may suggest the
distributional signal itself (not just a scalar residual) is what should carry forward
into this validation step.

### 2.1 Solvable vs Unsolvable Separation

- [ ] Compare residual traces: obs_recovery vs obs_noescape
- [ ] Key question: does the residual pattern (shape, peak, or duration) differ
      between the two? If yes, it's a viable signal. If no, the hypothesis fails.
- [ ] Compute a simple scalar: AUC of residual during obstacle window (t_appear to
      t_escape or timeout) — does it separate the two populations?

**This is the empirical test that validates or invalidates the core hypothesis.**
If residuals don't separate solvable from unsolvable, redesign before proceeding.

### 2.2 Residual Baseline (single speed — dropped multi-speed comparison 2026-07-21)

- [ ] Compute the 95th-percentile residual during nominal (max-velocity) driving →
      this is A₀, the "what's normal at this speed"
- [ ] This baseline anchors conformal calibration in Phase 3
- ~~Per-velocity-level A₀(v) comparison~~ — moot now that only one velocity condition
  is used; not needed for the single-speed Pareto claim.

---

## Phase 3: Conformal Prediction Calibration — ⏸ DEFERRED (2026-07-22)

Deferred behind Phase 1.5 — calibration runs are on hold until the exploratory
fault-reaction study is done. Also no longer the assumed mechanism: conformal
prediction's distribution-free framing is one candidate, not a commitment (see
"Research Direction" above — the goal is to embrace the ST-GAT's native predictive
distributions, not necessarily collapse them into a distribution-free bound). Keep
this phase's content as one option to evaluate against alternatives, not as the plan.

**Goal (as originally scoped):** Get a distribution-free guarantee: at the operating
(max) velocity, P[prediction error ≤ r(A)] ≥ 1-δ, where r(A) is the conformal bound at
anomaly score A.

### 3.1 Calibration (single speed — dropped per-speed loop 2026-07-21)

- [ ] Split nominal runs into D_train (fit residual→error mapping) / D_conf (calibrate)
- [ ] Fit isotonic regression: anomaly score Aₜ → expected position error at t+H
- [ ] Compute conformity scores on D_conf; find q̂_{1-δ} quantile
- [ ] Verify empirical coverage ≥ 0.94 at δ=0.05

**Success criterion:** Coverage check passes at the operating velocity.

### 3.2 Coverage Visualization

- [ ] Plot: residual score vs. actual prediction error, with conformal bound overlaid
- [ ] Show coverage fraction across the calibration set
- [ ] This becomes Figure X in the thesis

---

## Phase 4: Constraint Mapping and Validation

**Goal:** Show the framework can select a velocity constraint that enables forward
progress while satisfying the conformal bound.

### 4.1 Constraint Selection Logic

- [ ] Given residual A at time t and obstacle distance d:
      v_max(t) = max(v_min, (d - d_min - r(A)) / H)
- [ ] Implement as `st_gat/constraint.py`
- [ ] Test on obs_recovery rosbag data (offline): does the computed v_max allow the
      vehicle to reach the obstacle swerve zone?

### 4.2 ROS2 Constraint Publisher (if offline validation passes)

- [ ] Write a ROS2 node that subscribes to residual stream, publishes v_max to
      `/planning/scenario_planning/max_velocity`
- [ ] Run live on obs_stuck scenario: does constraint relaxation let Autoware proceed?
- [ ] Compare: obs_stuck baseline (stops) vs. framework-active (swerves or proceeds)

---

## Key Design Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Velocity | 11.11 m/s (max/map-limit) | Single operating condition — all experiments now run at max velocity, no per-speed calibration |
| Obstacle distance | 30m | obs_* scenarios |
| Conformal δ | 0.05 | Standard 95% coverage |
| Planning horizon H | 3s | Kinematic stopping distance |
| Min clearance d_min | 2m | Physical buffer |
| ST-GAT features | pos(2), vel(2), steer(1), accel(1), obj_dist(1), tl(1) | T-ITS 2025 |
| Goals | 007, 011, 021 | Verified live on P5000 (2026-06-27), all routes feasible |

---

## Explicitly Out of Scope

- Multiple velocity levels / per-speed calibration (nom_v5, nom_v7) — dropped 2026-07-21;
  all experiments now run at max velocity (11.11 m/s) only
- Perception faults (dropout30, position noise) — not needed for core Pareto claim
- Obstacle removal / disappearing obstacle — artificial, doesn't reflect real recovery
- Multi-fault type comparison matrix
- LiDAR faults — deferred for the Phase 1.5 exploratory study (2026-07-22); IMU and
  Camera only this round, LiDAR can follow once those are understood
- Scenarios other than static obstacle (cut-in, pedestrian, slow lead vehicle)
- Distance sweep (20m, 50m, 100m) — 30m is the one scenario
- Weight modulation of ST-GAT (constraint tightening is the intervention)
- RISE name and framing

These are valid future work for the thesis "Limitations and Future Work" section.
