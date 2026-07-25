# Risk-Aware Control — Task List

**Last Updated:** 2026-07-24 — major reframe. The dissertation's core contribution
is now **belief-divergence fault detection under a map-grounded prior**, reported
with **calibrated confidence** and measured **lead time** — not active risk-aware
control. This sharpens, rather than discards, the 2026-07-22 pivot to an
exploratory IMU/Camera fault-reaction study; that study now feeds directly into
the mechanism experiments below. See "Research Direction" for the full claim and
"Priority 0" for the two experiments that establish or kill it — those are
blocking and outrank everything else in this file.

---

## Research Direction (revised 2026-07-24 — supersedes the 2026-07-22 text below it)

**The claim to defend:** This digital-twin framework detects when the autonomy
stack's perceptual belief has diverged from a map-grounded, independently derived
expectation of the world; it reports that divergence with calibrated confidence, a
bounded number of seconds of lead time before the divergence degrades vehicle
safety.

**The mechanism — not a restatement of the T-ITS paper, an explanation of it.**
The digital twin maintains an independent expectation of what the perception layer
should be reporting, from three sources: HD map (Lanelet2) annotations that
categorically assert what infrastructure exists at a given lanelet, learned
nominal temporal patterns, and vehicle dynamics context. A fault is detected when
the autonomy stack's perceptual belief diverges from that independent expectation.
The strong form is **negative-evidence detection**: the map licenses a hard
categorical expectation ("a signalized intersection exists at this node"), and the
fault signature is perception failing to report what must be there.

This is already the dominant detection pathway in the published T-ITS results —
not future work. Camera fault is injected as pixel-level Gaussian noise (σ = 10)
degrading camera-based detection; Traffic Light Status Flag is a camera-derived
discrete feature already in the model's state vector; it's the highest-importance
feature at 29.7%, and its CUSUM residual is the single most discriminative
feature-residual combination at 22%.

**Why negative evidence explains, not just reports, the published results.**
Negative evidence requires a hard prior. Traffic lights have one (the map
annotation). Object detections don't — objects legitimately appear and disappear,
so absence of a detection isn't evidence of a fault. This asymmetry *predicts* two
published observations, it doesn't just describe them after the fact:
traffic-light features compress to very few PCA components while
object-detection-flag features need more; LiDAR fault — which manifests mainly
through the object detection flag — has the worst recall (0.850), confused into
Camera (14 cases) and IMU (13 cases). Negative evidence works where the map
licenses a hard expectation and degrades where it doesn't.

**Three load-bearing pieces of the claim, each mapping to a result:**
1. **Belief divergence / negative evidence** — established as a *mechanism*, not
   an observed correlation. Requires Priority 0 below (Experiments A & B) — this
   is not yet proven, it's the current highest-risk open question in the
   dissertation.
2. **Calibrated confidence** — the uncertainty-quantification / conformal-
   prediction contribution. Headline result is a calibration curve, not an
   accuracy number: when the monitor reports 70% confidence it should be right
   ~70% of the time. This is what converts a classifier into a safety monitor
   that can be trusted — a monitor that is confidently wrong is worse than
   useless.
3. **Lead time** — the prediction-horizon study (history and prediction window
   lengths), reframed from a hyperparameter sweep into a safety result: how much
   advance warning does the framework give before the fault compromises safety?

**Framing shift, applied throughout this file from here on:** *detection*
("fault: yes/no" — already demonstrated and banked from the T-ITS paper) versus
*verification* (a calibrated statement about confidence, including knowing when
the monitor doesn't know). Uncertainty quantification and interpretability are
core to the contribution, not a side quest bolted onto a classifier.

**Scoping corollary — what this deliberately excludes.** Scenario-based avoidance
demonstrations (staged cut-in, unexpected stopped vehicle — the `obs_*` campaigns
in Phase 0.2/Phase 2 below) are contributions to control and handling, not to
safety verification. They are de-emphasized to illustrative/context status.
**No new staged-avoidance experiments.** By the same logic, the old
"Pareto frontier of (safety, progress)" framing and the RISE active-control
mechanism (old Phase 3/4 below, and the original content of
`docs/theoretical_framework.md`) are **not the thing being defended** — kept as
future-work content, not deleted, but not blocking. Rationale: the claim above is
entirely about detecting and reporting divergence with calibrated confidence and
lead time; closing the loop into active control is a downstream application of
that signal, not evidence for the verification claim itself.

**Overarching dissertation goal (unchanged in spirit, sharpened in mechanism):**
show this approach can make AVs safer in a measurable way — specifically, by being
a trustworthy *early-warning / verification layer* that knows what it knows, not
by being a novel controller.

**Prior published work (context, not this repo):** Vadnerkar & Pisu, "Digital
Twins as Predictive Models for Real-Time Probabilistic Risk Assessment of
Autonomous Vehicles," IEEE T-ITS vol. 27 no. 4, April 2026 (PDF in project root).
Trained an ST-GAT purely on nominal Autoware+AWSIM driving data (same
Nishishinjuku map/vehicle setup as this repo), detected Camera/IMU/LiDAR faults
via prediction-observation residuals (Raw, KL divergence, CUSUM) + PCA + Random
Forest, 93.7% accuracy. Traffic Light Status Flag was the single most
discriminative feature (29.7% importance); its CUSUM residual the top
feature-residual combination (22%); CUSUM consistently beat Raw/KL.

**Distributions over distribution-free:** the ST-GAT already outputs full
predictive distributions (Gaussian mean+variance for continuous features,
Bernoulli probability for discrete ones) via deep ensembles — that's where the
calibrated-confidence contribution lives. The direction is to **embrace those
distributions directly**, not collapse them into a single distribution-free
conformal bound. Conformal prediction remains one candidate mechanism for
calibration (not the only one — likelihood-based risk scores and other
distribution-aware calibrated bounds are on the table too), but calibration
*itself* is no longer deferred (see Priority 2 below) — it is a core, non-optional
contribution, not a candidate that might get dropped.

**What this is NOT:** Re-implementing Autoware's planner. Not a fixed control
policy. As of this reframe, not primarily a control-extension dissertation —
active control is illustrative future work, not the claim under defense.

---

## Priority 0 — BLOCKING: Mechanism Experiments (added 2026-07-24)

These two experiments either establish or kill the central mechanistic claim
(negative-evidence detection via a map-grounded prior). Nothing downstream —
the calibration framing, the lead-time framing, the defense narrative — is
credible until these run. Outranks every phase below, including in-progress data
collection.

### P0.1 — Experiment A: Per-fault-class feature importance (no new sim runs)

Published Fig. 9 and Fig. 10 (T-ITS paper) aggregate feature importance across
*all* fault conditions. Recompute importance sliced by fault class (Camera / IMU /
LiDAR separately).

**Sharp, falsifiable prediction:** traffic-light-CUSUM should be selectively
dominant for Camera faults and substantially weaker for IMU faults. If it's
uniformly dominant across all three fault types, it's a clean-signal artifact — a
binary flag just produces a higher-SNR residual than noisy continuous kinematics,
independent of any map-grounded mechanism — and the negative-evidence story is
wrong.

**Data:** already in hand — this is the published T-ITS model/results, not new
data collection in this repo.

**Gap (flagged, not yet resolved):** the trained model, per-condition residuals,
and feature-importance computation for the T-ITS paper live in
`../Graph-Scene-Representation-and-Prediction/` (the read-only reference repo) or
wherever the paper's original analysis artifacts were kept — not currently in
`Risk-Aware-Control`. Needs a scoping decision: recompute directly against those
artifacts, or port the saved per-condition residual/importance data into this
repo's analysis tooling. **Status: not started.**

### P0.2 — Experiment B: HD map ablation on detection performance

The published ablation (Table II) measures trajectory prediction quality
(minADE/minFDE/MR) only — it does **not** test whether the HD map is load-bearing
for *detection*. The paper lists three sources feeding traffic-light prediction:
intersection timing patterns, deceleration profiles, and HD map annotations —
only the third is the map prior being claimed as the mechanism.

**Procedure:** retrain with map annotations for traffic signals stripped from the
graph node features. Measure the change in detection accuracy on Camera faults
specifically (Camera fault is the one that manifests through the TL channel — see
mechanism above).

- If Camera-fault detection **collapses** → the map prior is load-bearing,
  negative evidence is established as the mechanism.
- If detection **barely moves** → the model was reading deceleration profiles
  (or timing patterns) instead of the map, and the mechanistic claim must be
  revised or abandoned.

**This single experiment either establishes or kills the central claim.** The
single highest-priority item in the whole project — higher than data collection,
higher than calibration work, higher than anything else in this file.

**Status: not started.** Depends on: (1) resolving where the ablation runs (same
scoping question as P0.1 — original T-ITS pipeline vs. this repo's ST-GAT, which
is itself mid-retrain per the 2026-07-23 feature-vector change below); (2) a
graph-node-feature strip-out of map-derived TL annotations, which doesn't exist
yet in either codebase.

---

## Two-Arm Fault Injection Design (added 2026-07-24)

All fault-injection experiments (Priority 0 supporting work, the sweep, the
horizon study) should run in both arms going forward. This is not redundant with
Autoware's own safety layer — it's what makes lead time measurable at all.

- **Arm A — Autoware built-in safety features DISABLED (the science condition).**
  Lets the injected fault propagate so the full healthy → degraded → dangerous
  trajectory is observable. With the built-in safety layer active, Autoware
  amputates the failure before it can be studied. Answers: can the framework
  detect and characterize the fault, and how early? This is approximately what
  the current MRM diagnostic gate (README.md item 3: perception/planning/
  localization stripped from the autonomous-mode gate) already gives us.
- **Arm B — Autoware built-in safety features ENABLED (the ground-truth oracle).**
  Autoware's reactive MRM trigger firing is treated as a labeled ground-truth
  moment of "this was objectively unsafe." Not a baseline competitor — the
  validation signal. Answers: did the framework raise calibrated uncertainty *N*
  seconds before Autoware's hard trigger fired? Lead time measured against Arm B
  is the primary evidence for the claim.

**Natural fit with negative evidence:** with safety features disabled, a camera
occlusion that suppresses traffic-light detection produces a concrete,
unambiguous safety violation (proceeding through a signalized intersection on
red) — a hard ground-truth event with a well-defined timestamp, making lead time
cleanly measurable.

**Gap (flagged, not resolved — also noted in CLAUDE.md):** there is currently only
one MRM gate configuration in this repo (Arm A — safety features stripped down to
avoid deadlocking experiment resets). There is no Arm B config that restores the
full/stock diagnostic gate so Autoware's own MRM trigger can serve as ground
truth. Building and validating that second configuration (without reintroducing
the MRM deadlocks that item 3/4/7 in README.md fixed) is real engineering work,
not just documentation — needed before any lead-time-vs-Arm-B result can be
claimed.

---

## Phase 0: Data Collection ← CURRENT (infrastructure work, feeds Priority 0/1.5)

### 0.1 Nominal Campaign

Nominal data is the calibration foundation for conformal prediction / UQ. Direction
changed 2026-07-21: all experiments now run at max possible velocity (map limit,
11.11 m/s) — `nom_v5`/`nom_v7` are no longer needed (dropped from
`NOMINAL_DATASETS`, no more per-speed calibration). `nom_v5`/`nom_v7` rosbags
already collected on the P5000 are kept for reference but are not part of the
current plan; the `collect.sh nom_v5/nom_v7` commands still work if ever needed
again.

| Campaign | Status | Command |
|----------|--------|---------|
| `nom_v11` | 🔄 In progress | `./collect.sh nom_v11` |

### 0.2 Obstacle Campaigns — ⏸ DEPRIORITIZED (2026-07-24, control/handling only)

**Reframed 2026-07-24:** these are scenario-based avoidance demonstrations — per
the scoping corollary above, they're a control/handling contribution, not
evidence for the safety-verification claim. Not deleted (still useful as
illustrative context, and the infrastructure is real and working), but they are
**not** required for the defense and should not consume priority ahead of
Phase 0/Priority 0 fault work. Do not add new staged-avoidance scenarios.

| Campaign | Status | What it shows |
|----------|--------|---------------|
| `obs_stuck`       | ⏳ Not blocking | Autoware stops — baseline conservative behavior |
| `obs_recovery`    | ⏳ Not blocking | Autoware swerves (policy=auto) — actual recovery |
| `obs_noescape`    | ⏳ Not blocking | Single-lane (LL 241), no path — stopping IS optimal |
| `obs_singlelane`  | ⏳ Not blocking | 30m obstacle, no adjacent lane |
| `obs_tooclosetoreact` | ⏳ Not blocking | 5–8m obstacle, multi-lane |

Commands (unchanged, kept working, just not on the critical path):
```bash
./collect.sh obs_stuck
./collect.sh obs_recovery        # sets policy to "auto", restores on exit
./collect.sh obs_noescape        # verify obstacle lands in LL 241 (single-lane)
./collect.sh obs_singlelane
./collect.sh obs_tooclosetoreact
```

### 0.3 Fault Campaigns (IMU + Camera) — feeds Priority 0 / Phase 1.5

This is now core-path work, not exploratory-only — it's the substrate for the
mechanism experiments above. Infrastructure already exists in
`experiments/lib/fault_injector.py` and `collect.sh`.

| Campaign | Status | What it is |
|----------|--------|------------|
| `imu_fault_s1`..`s4` | Redesigned 2026-07-23, rerun pending | Gyro bias, periodic on/off, bounded accumulated heading error ≤1.2 rad — see below |
| `tl_fault_s1`..`s4`  | Redesigned 2026-07-22, validated 2026-07-23 | TL confidence/oscillate/unknown/blackout, zone-triggered periodic (repeats at every TL intersection on the route) |

**Camera scope confirmed (2026-07-22):** verified directly against this project's
Autoware install — `perception_mode` defaults to `lidar` (object detection is
LiDAR-only) and the `awsim_labs_sensor_kit` has exactly one camera, feeding
traffic-light recognition only. So "camera fault" in this repo can only mean a
TL-detection fault — the existing `tl_*` fault modes are the correct and only
injection point, not a stand-in for a literal camera-sensor fault. This maps
cleanly onto the mechanism claim: TL detection IS the camera-derived channel the
negative-evidence story is about. A genuine image-level occlusion mask (closer to
the T-ITS paper's Gaussian pixel-noise approach) remains a deferred stretch goal —
see `docs/research_notes/periodic_fault_strategy.md`.

**LiDAR is still out of scope for this repo's own data collection.** Priority 0's
Experiment A uses the *published* paper's LiDAR fault data (already in hand, see
gap note above), not new LiDAR sim runs here. Whether this repo eventually needs
its own LiDAR fault campaign (for the supporting sweep/generalization work below)
is an open scoping question, not yet decided.

**TL fault redesign (2026-07-22):** `fault_injector.py`'s TL state machine now
loops (`waiting_zone → fault_active → recovering → waiting_zone → ...`), re-arming
at every TL zone for the rest of the trial, each cycle bounded by a per-zone
duration cap (`--fault-duration`, 15s, derived from replaying goal_007/012/026's
actual TL-zone dwell times) or early exit from the zone, followed by an 8s
(`--tl-recovery-gap`) nominal gap.

**Goals finalized (2026-07-22):** fault campaigns run on **goal_007, goal_012,
goal_026** — measured as the routes with the most real TL-zone entries per trial
(3, 3, 2 respectively; see `periodic_fault_strategy.md` §4).

**Critical bug found and fixed (2026-07-22): fault campaigns had zero effect.**
goal_007 smoke-test trials for `tl_fault_s1..s4` (all 4) confirmed dead —
`msg_count_tl: 0` in every campaign's `fault_log.jsonl`, and
`compare_fault_vs_nominal.py` showed zero behavioral difference from nominal.
Root cause was Autoware topic wiring, not injection logic — see README.md item 8
for the full before/after. Fixed: `fault_injector.py`'s TL topic names, two
Autoware launch XML defaults (`behavior_planning.launch.xml`,
`gyro_odometer.launch.xml`), and `config.py`'s `RECORDING_TOPICS`. **All
`tl_fault_s1..s4`/`imu_fault_s1..s2` goal_007 data collected before this fix is
invalid (the fault never reached the vehicle) and was rerun.**

**Rerun confirmed the fix works — plus a real IMU severity problem (2026-07-23).**
Reran goal_007 (`tl_fault_s1..s4`, `imu_fault_s1..s2` — `s3/s4` not run, stopped
after `s2`'s outcome below). TL: message-level fault clearly confirmed across all
4 tiers (`tl_confidence` z-scores 4.29–8.61 vs ~1.5–2.1 nominal, 0% detection rate
under blackout, 100% UNKNOWN under S3) — but no tier produced a detectable
velocity/steering response on this route/trial; open question for more
goals/trials, not a wiring problem. IMU: **S2 (old: gyro=0.15 rad/s, 30s on)
caused a hard-brake + permanent stuck within the first fault cycle**
(`status: stuck`, 1 MRM trigger, EKF-vs-ground-truth divergence jumped 0→14.5m
and plateaued since the vehicle stopped moving) — only a 3x bias increase from S1
(safe, measurable effect) crossed from mild to catastrophic, implying a stability
cliff well before the old S2's ~4.5 rad accumulated heading error (integrates and
does not reset when the bias turns off, unlike TL faults). **Redesigned all 4 IMU
tiers in `collect.sh`** to bound accumulated heading error ≤1.2 rad (S1=0.03/20s,
S2=0.05/20s, S3=0.08/15s, S4=0.12/10s, all with a uniform 30s recovery gap) — a
hypothesis pending validation on the next run, not a guarantee, given the
cliff-edge (not smooth) sensitivity observed. `run_fault_campaigns.sh`
results/plots for this rerun are in `experiments/analysis/fault_comparison/`.

**ST-GAT feature vector changed 2026-07-23 — retrain required.** Auditing the
fault-vs-nominal analysis against `st_gat/pipeline/` (per Kalpit's "make sure the
signal exists in the pipeline, otherwise the anomaly is unobservable" principle)
found two real gaps, now fixed:
1. `config.py`'s `TOPICS['traffic_lights']` read the real, unmodified
   `traffic_signals` topic — same class of bug as `fault_injector.py`'s original
   wiring, meaning ST-GAT would have been structurally blind to every TL fault
   regardless of severity. Fixed to read `traffic_signals_faulted`, with a
   fallback to the unmodified topic for bags collected before the fix (all of
   `nom_v11`, the actual training set).
2. `traffic_light_state` was color-only (ignored `confidence` entirely) — a
   confidence-degradation fault like `tl_confidence` would leave color untouched
   and be completely invisible. Also, UNKNOWN color, complete blackout, and
   genuinely no TL nearby all collapsed to the same 0.0 value.

Fixed by folding confidence into `traffic_light_state` (color × confidence), and
adding a new explicit `traffic_light_discrepancy` feature (map expects a TL here
but perception found nothing usable) — **this feature is the literal negative-
evidence signal the mechanism claim is about**, not just a bugfix; it should be
treated as load-bearing for Priority 0 work, not incidental. **Feature count
changed 13→14 — this invalidates `st_gat/checkpoints/best_model.pth` and
`st_gat/models/st_gat_rise.pth` (different input dimensionality). Retrain before
trusting either checkpoint again**, and before running Experiment B's ablation
(which strips a different subset of node features but depends on this vector
being current).

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
- [ ] Plot residual traces for a clean nominal run and representative fault runs

---

## Phase 1.5: Exploratory Fault-Reaction Study (feeds Priority 0)

**Goal:** before formalizing calibration/lead-time work, look at how the ST-GAT
digital twin's residuals (and full predicted distributions — mean/variance,
Bernoulli probabilities) react to IMU and Camera faults inside this repo's own
pipeline. Still exploratory, but now explicitly in service of Priority 0: this is
where the intuition for "is TL-CUSUM selectively dominant for Camera faults"
(Experiment A's prediction) gets a first, cheap look before the formal recompute.

- [ ] Read `docs/research_notes/fault_literature_review.md`, resolve its open
      questions (camera fault definition, accel_bias_ms2 no-op) before running fault
      campaigns
- [ ] Run `imu_fault_s1`..`s4` and `tl_fault_s1`..`s4` campaigns (0.3 above)
- [ ] Run trained ST-GAT (Phase 1.2) over nominal + fault-condition rosbags, compute
      full predictive distributions and residuals (Raw/KL/CUSUM) per timestep
- [ ] Plot: predicted mean ± variance vs. observed value, through a fault window
      (onset → sustained → recovery) for both IMU and Camera/TL conditions — does the
      predicted variance itself widen under fault, independent of the residual?
- [ ] Compare against the T-ITS paper's finding (CUSUM > KL > Raw, TL Status Flag
      dominant) split by fault type where possible — an early, informal read on the
      Experiment A prediction (TL-CUSUM should be selectively strong for Camera,
      weak for IMU) using this repo's own pipeline
- [ ] Write up findings in `docs/research_notes/` — this is the evidence base
      Priority 0 and the supporting experiments below get designed against

---

## Priority 1 — Supporting Experiments (build after Priority 0 confirms the mechanism)

### P1.1 Systematic fault injection sweep

Structured across fault type and fault magnitude (the T-ITS paper's limitations
section already flags magnitude sensitivity across 10%–75% degradation as
unaddressed). Frame as **characterizing the detectable boundary**, not improving
accuracy. Key question: at what fault magnitude does the residual signal
disappear into nominal noise, and does the model know when it's near that edge?
Run in both arms (two-arm design above).

### P1.2 Calibration / conformal prediction ← was Phase 3, now un-deferred

**No longer deferred as of 2026-07-24** — calibrated confidence is one of the
three load-bearing pieces of the claim, not a candidate mechanism that might get
dropped. The distribution-free conformal framing from the original Phase 3 is
kept as one option; likelihood-based / distribution-aware calibrated bounds
(exploiting the ST-GAT's native Gaussian/Bernoulli outputs) are evaluated on
equal footing (see "Distributions over distribution-free" above).

- [ ] Split nominal runs into D_train (fit residual→error mapping) / D_conf
      (calibrate)
- [ ] Fit a calibration mapping (isotonic regression is the baseline candidate;
      evaluate distribution-aware alternatives too): anomaly score → expected
      error / confidence level
- [ ] Compute conformity scores (or the equivalent for the chosen method) on
      D_conf; find q̂_{1-δ}
- [ ] **Headline result: a calibration curve** — for each reported confidence
      level, does the empirical accuracy match it across the fault sweep? — not
      a single coverage-check pass/fail number
- [ ] Report where calibration holds and where it breaks down (which fault types,
      which magnitudes, which arm)

**Success criterion:** calibration curve stays close to the diagonal (reported
confidence ≈ empirical accuracy) across the fault sweep; empirical coverage
≥ 0.94 at δ=0.05 as a secondary check, not the headline number.

### P1.3 Horizon study — lead time ← was a hyperparameter sweep, now a safety result

- [ ] Vary history and prediction window lengths
- [ ] **Report as lead time in seconds relative to the ground-truth safety
      event** (Arm B's Autoware MRM trigger timestamp), not as accuracy deltas
- [ ] This is the primary evidence for the "lead time" pillar of the claim —
      depends on the Arm B gap being resolved first (see two-arm design section)

### P1.4 Interpretability

- [ ] Does residual attribution correctly localize the injected fault within the
      stack (right feature, right channel)?
- [ ] Doubles as a check on whether the monitor is detecting the right thing for
      the right reason — a second, independent line of evidence for the
      mechanism claim, alongside Priority 0

### P1.5 Generalization framing

- [ ] If Experiment B (Priority 0) confirms the map prior is load-bearing,
      framework performance should depend on map annotation density and quality
- [ ] Converts the published limitation (validated only on Nishishinjuku) from an
      apology into a principled, testable statement about where the method
      applies — not yet run, blocked on Priority 0 confirming the mechanism first
      (this experiment presumes the mechanism it's explaining)

---

## Deprioritized / Future Work: Active Control (RISE) — was Phase 2/3/4

**Reframed 2026-07-24.** Everything below was written for the old "active
risk-aware control" framing (relax a velocity constraint under uncertainty,
Pareto frontier of safety vs. progress). Per the scoping corollary above, this is
control/handling work, not safety-verification work, and is **not required for
the defense**. Kept as a real, coherent future-work chapter — the infrastructure
exists and the ideas aren't wrong, they're just not what's being defended.

### Signal Validation (was Phase 2) — written against `obs_*` scenarios

- Compare residual traces: `obs_recovery` vs `obs_noescape` — does the residual
  pattern differ between solvable and unsolvable obstacle scenarios?
- This was "the empirical test that validates or invalidates the core
  hypothesis" under the old framing. Under the new framing it validates a
  *control* hypothesis, not the safety-verification claim — do not treat a
  negative result here as evidence against the dissertation's actual claim.

### Conformal Calibration for Control (was Phase 3, pre-2026-07-24 framing)

The original Phase 3 goal — "get a distribution-free guarantee: at the operating
velocity, P[prediction error ≤ r(A)] ≥ 1-δ" — is superseded by P1.2 above, which
generalizes the same machinery to serve the calibration claim rather than a
control-constraint derivation specifically. Not duplicated content, just
redirected.

### Constraint Mapping and Validation (was Phase 4)

- Given residual A at time t and obstacle distance d: v_max(t) = max(v_min,
  (d - d_min - r(A)) / H); implement as `st_gat/constraint.py`; test offline on
  `obs_recovery`; if it passes, a ROS2 constraint publisher node live on
  `obs_stuck`.
- Untouched, not started, not blocking. Valid future-work chapter once the
  verification claim (Priority 0–1 above) is established — a calibrated,
  lead-time-aware detector is a *precondition* for a principled controller, not
  a competing deliverable.

---

## Key Design Parameters

| Parameter | Value | Source | Status |
|-----------|-------|--------|--------|
| Velocity | 11.11 m/s (max/map-limit) | Single operating condition | Active |
| Conformal δ | 0.05 | Standard 95% coverage | Active — now calibration-curve headline, coverage is secondary check |
| ST-GAT features | 14 features incl. `traffic_light_discrepancy` | Changed 2026-07-23 | Active — retrain required before any P0/P1 work trusts a checkpoint |
| Fault goals | 007, 012, 026 | Most TL-zone entries per trial | Active |
| Nominal/obstacle goals | 007, 011, 021 | Verified live (2026-06-27) | Active for nominal; obstacle goals now control-track only |
| Obstacle distance | 30m | `obs_*` scenarios | Deprioritized — control track only |
| Planning horizon H | 3s | Kinematic stopping distance | Deprioritized — control-constraint parameter, not a verification metric |
| Min clearance d_min | 2m | Physical buffer | Deprioritized — control track only |

---

## Explicitly Out of Scope

- **Scenario-based avoidance demonstrations** (staged cut-in, unexpected stopped
  vehicle, and the existing `obs_*` static-obstacle campaigns) — reclassified
  2026-07-24 as control/handling contributions, not safety-verification evidence.
  Deliberately scoped out of the claim being defended; kept as illustrative
  future work, not deleted. **No new staged-avoidance experiments.**
- **RISE name and framing / active constraint relaxation** — same reasoning;
  demoted from "the extension this repo is building" to "future work," not
  required for the defense claim.
- Multiple velocity levels / per-speed calibration (nom_v5, nom_v7) — dropped
  2026-07-21; all experiments now run at max velocity (11.11 m/s) only.
- LiDAR fault data collection *in this repo* — Experiment A uses the published
  paper's existing LiDAR data; whether this repo needs its own LiDAR campaign for
  the supporting sweep/generalization work is still an open question, not a
  closed one.
- Weight modulation of ST-GAT (constraint tightening is the control-track
  intervention, itself now non-blocking).
- Distance sweep (20m, 50m, 100m) for obstacle scenarios — control-track only.

These are valid future-work items for the dissertation's "Limitations and Future
Work" chapter, several now explicitly upgraded from "dropped" to "a coherent
control-extension chapter, just not this one."
