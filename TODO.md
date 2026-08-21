# Risk-Aware Control — Task List

**Last Updated:** 2026-08-20 (open-world reframe — read
`docs/research_notes/open_world_safety_reframe_2026-08-20.md` FIRST, it
supersedes the "belief divergence" claim language used throughout this
file). The calibration-pivot status block immediately below is still
factually accurate and worth reading — it's the mechanics of Layer 1 in
the new reframe's terms — but its own framing language (written a few
hours earlier the same day) predates that reframe. The "Research Direction"
section further down (last-revised 2026-07-24) and "Priority 0" are both
**retired** by the reframe — kept verbatim further down as history, not
deleted, not current.

---

## Status block (2026-08-20 — calibration pivot, read this first)

**Calibration is now working — via a different mechanism than originally
planned.** The 2026-08-06 reframe's step (1) ("ground the belief-divergence
mechanism's own credibility with calibration/trust plots") now holds up:
pooled coverage 89.7-90.2% (target 90%) at every horizon step for all 7
series (position, velocity split per-axis, steering, acceleration, both TL
heads), via **leave-one-trial-out cross-conformal calibration wrapped
around a plain point predictor** (`experiments/scripts/
conformal_horizon_calibration.py`) — not via the jointly-trained Student-t/
NLL heads that the 2026-08-06 through 2026-08-20 arc (summarized below,
kept for history) tried and never fully converged. Real caveat: only 7
nominal trials exist, and per-trial coverage varies much more than the
pooled number (e.g. 77-95% for acceleration) — the aggregate claim is
solid, a claim about one single new drive is not yet, without more nominal
data. Full arc + pivot rationale:
`docs/research_notes/nll_calibration_arc_and_conformal_pivot_2026-08-20.md`
— **read that before resuming any calibration work**, so effort isn't
spent re-diagnosing an already-understood, already-paused problem.

**What this changes, concretely:**
- Step (1) of the explicit order below is now satisfied. Step (2) (SPRT
  signal behavior — still not quiet under nominal driving, see below) is
  the next open item, followed by (3) Autoware planning/control interfaces.
- The primary calibration artifact to trust going forward is
  `conformal_horizon_calibration.py`'s output
  (`experiments/analysis/conformal_horizon_calibration/`), not
  `plot_calibration_diagrams.py`'s `horizon_widening.png` (still real,
  still useful if the NLL arc is ever resumed, just not the current
  headline result).
- Two real architecture bugs found during the NLL arc are KEPT regardless
  of the pivot (not reverted): per-head horizon-step embeddings
  (`model.py`, fixes a genuine gradient-contention bug) and the dof-collapse
  regularizer (`loss.py`'s `DOF_REG_WEIGHT`).
- **Open, not yet answered**: does conformal calibration (validated on
  held-out *nominal* data, under an exchangeability assumption) hold up
  under actual fault/distribution-shift conditions? An epistemic-
  disagreement test (independently-trained point-predictor members, does
  their disagreement widen over the horizon) is the current parallel
  thread investigating model robustness under shift — see the research
  note's closing section.

**Old status (2026-08-06 through 2026-08-19, kept for history — the arc the
note above supersedes):**

**Direction reframe.** Stop optimizing toward "detect and classify a
specific fault type" — collapsing a fault into a named category only
licenses whatever canned contingency response was pre-planned for that
category, which is explicitly not the goal. Instead: provide a continuous,
digestible signal to the modules that actually own the vehicle's
decision-making (planning/control), so it can keep operating sensibly under
degraded/uncertain conditions without needing to know which specific fault
is occurring. Motto: **"operational under degradation."** This does not
discard the belief-divergence/calibrated-confidence/lead-time claim below —
it changes what "done" looks like: a well-behaved continuous confidence
trace consumed by planning, not a tuned binary alarm chasing a detection
rate inside a fixed window.

**Explicit order, do not skip ahead:** (1) ground the belief-divergence
mechanism's own credibility (calibration/trust plots) before anything else;
(2) check the SPRT/sequential-evidence signal behaves as an interpretable
continuous trace (not "does it cross a threshold"); (3) only then look at
Autoware planning/control interfaces (MRM's staged escalation, velocity/
safety-envelope constraints) as candidate consumers of the signal.

**Where (1) and (2) currently stand — they do NOT hold up cleanly.** Full
writeup: `docs/research_notes/trust_and_signal_behavior_2026-08-06.md`.
Summary:
- All 6 Gaussian-headed features (position, velocity, steering,
  acceleration, traffic_light_color, traffic_light_confidence) are
  **leptokurtic, not Gaussian** — excess kurtosis 4.8–115.8, Anderson-
  Darling statistic 94–742 against a critical value of 0.79 (normality
  decisively rejected) — even though `check_calibration.py`'s std(z)
  summary (0.85–1.13) looks fine. That single number can't distinguish
  "right variance, right shape" from "right variance, wrong shape."
- **Predicted uncertainty doesn't widen with prediction horizon** for 5 of
  6 Gaussian heads (only `position` does) — by 3s ahead, actual RMSE is
  2–4x the model's own claimed std for velocity/steering/acceleration/both
  TL heads. Any confidence claim beyond 1-step-ahead is currently
  unfounded for those features.
- **The SPRT `p_fault_*` signal is not quiet under nominal driving** — it
  saturates to >0.99 roughly every 9 seconds during ordinary nominal
  trials, traced to the `traffic_light_discrepancy` Bernoulli branch alone
  (96% of nominal time above p=0.99, from a ~9.3%-per-10Hz-step nominal
  base rate too high for a memoryless sequential accumulator to treat as
  rare evidence).

**First fix attempt done (same day) — mixed result, still blocking.**
Gaussian heads → Student-t (mean/scale/dof) + the flat single-shot output
heads → a per-horizon-step conditioned decoder (`_StepConditionedHead`),
retrained (early-stopped epoch 55, val raw error 0.0158, slightly better
than the 0.0168 pre-fix baseline). Full writeup incl. before/after
coverage-curve table: §3 of the same research note. Result: calibration
measurably improved for `position`/`steering`, ~unchanged for `velocity`,
measurably **worse** for `acceleration`/both TL heads; horizon widening
**still doesn't work for 5 of 6 heads** despite the architecture now
having the capacity for it. Leading hypothesis (not yet tested): the
`Trainer`'s checkpoint-selection/early-stopping criterion
(`position_l2_raw + 0.8·velocity_l1_raw`) is blind to calibration for
every feature and blind to accuracy *and* calibration for
acceleration/TL heads specifically — training stops the moment
position/velocity point-accuracy plateaus, with no signal rewarding
correct scale/dof convergence for the other heads before that happens.

**(2026-08-20: the paragraph above — "make the training/checkpoint-selection
criterion calibration-aware, then retrain again" — is superseded by the
conformal pivot at the top of this status block. Calibration is no longer
blocked on the NLL training criterion converging.)**

**Current priority: step (2)** — the `traffic_light_discrepancy` branch's
SPRT-saturation root cause (base rate too high for a memoryless sequential
accumulator to treat as rare evidence) is still a real, unaddressed
problem, now that step (1) is satisfied via the conformal pivot. Revisit
`experiments/scripts/plot_sprt_signal_behavior.py` (pure pandas over
`st_gat/results/h30_30/traces/*.csv`, no ROS needed) with that in mind.

Tooling: `experiments/scripts/conformal_horizon_calibration.py` (repo venv
only, no ROS) is the primary calibration check now — see CLAUDE.md's
"Trusting the model itself" section. `plot_calibration_diagrams.py`
(needs ROS+model) is the older, paused NLL-head diagnostic, still real but
not the first thing to run.

---

## Research Direction — RETIRED 2026-08-20, kept as history (was: revised 2026-07-24, supersedes the 2026-07-22 text below it)

**Superseded by `docs/research_notes/open_world_safety_reframe_2026-08-20.md`
— the "belief divergence / negative evidence" claim below is retired, not
current.** Some mechanics described in this section still apply unchanged
under the new reframe (the two-arm fault-injection design just below this
section; the scoping corollary excluding staged-avoidance/RISE work, itself
now an open question again per the reframe's Layer 3 — see `CLAUDE.md`) —
read the new note to know which parts of this section still hold.

**The claim to defend (OLD, retired):** This digital-twin framework detects when the autonomy
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

**See `docs/theoretical_framework.md` (rewritten 2026-08-01)** for the full
thesis statement, epistemic stance, horizon-as-variable design principles, the
divergence trace schema, and the fatal-moment anchor decision — this section
gives the research-direction summary, that doc is the authoritative statement of
the claim itself; don't duplicate its text here.

---

## Priority 0 — RETIRED 2026-08-20 (was: BLOCKING Mechanism Experiments, added 2026-07-24)

**Retired by the open-world reframe — see
`docs/research_notes/open_world_safety_reframe_2026-08-20.md` §1 for the
full reasoning.** Both experiments below were closed-set fault
classification (fixed bin set, requires fault data, says nothing about a
novel failure mode) — exactly what the reframe moves away from. Kept
verbatim below as history, not current work. Disposition:
- **P0.1 dropped entirely.** No residual value under the new framing.
- **P0.2 not reused as-is, but its underlying question survives**: whether
  map-derived graph context is load-bearing for the model's behavior is now
  a prerequisite for Layer 2 (consequence estimation) rather than a
  fault-classification ablation — re-run only with a continuous,
  scene-sensitivity metric, not the old classification-accuracy-delta one.
  Not yet run either way.
- Decoupled from `../Graph-Scene-Representation-and-Prediction/` as a
  dependency for new work — stays cited reference material only.

<details>
<summary>Original text (2026-07-24, historical — do not treat as current)</summary>

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

</details>

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

**Gap (2026-07-25 — Arm B config now built, NOT yet validated live):** Arm B
(stock/full diagnostic gate, restoring Autoware's default
`/autoware/modes/autonomous` linkage exactly) exists now — see CLAUDE.md and
`experiments/scripts/switch_diagnostic_arm.sh`. Whether it can actually run a
fault campaign without reintroducing the MRM deadlocks that README.md items
3/4/7 fixed is untested as of this writing — that validation, not the config
itself, is what's still open before any lead-time-vs-Arm-B result can be
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

**Superseded the standalone `extract.py` idea below — implemented instead as
`st_gat/pipeline/{bag_reader,sequence_builder,run_pipeline}.py`** (14-feature
vector, graph-per-window, train/cal split by goal). Code exists; actually running
it against the collected fault campaigns has not happened yet.

- [x] Entity-collapsing TL feature bug (§1.10/§1.12 in
      `docs/stgat_pipeline_plan.md`) — fixed 2026-08-01: `bag_reader.py` now
      scopes `traffic_light_state`/`traffic_light_discrepancy` to the one
      `group_id` governing the vehicle's current lanelet (via
      `experiments/configs/tl_zones.json`, same selection `fault_injector.py`
      itself uses), not pooled across every currently-tracked group. Falls back
      to the old pooled behavior when a goal has no zones file entry.
- [ ] Decide the cross-topic time-sync strategy explicitly in writing (Stage 0 in
      `stgat_pipeline_plan.md`) — currently forward-fill onto the `objects` topic
      as master clock with a 300ms staleness cutoff; not yet re-litigated against
      the plan doc's "proper interpolation vs. nearest-neighbor snap" question.
- [ ] Actually run `python3 -m st_gat.pipeline.run_pipeline` against the
      collected nominal + fault campaigns and verify feature distributions look
      sane (no NaNs, velocity capped at expected values) — not yet done.

### 1.2 Model Training

**`st_gat/pipeline/config.py`, `st_gat/model/{model,loss,dataset,trainer}.py`,
`st_gat/train.py` already exist** (STGAT architecture: graph encoder + BatchNorm
+ transformer + LSTM + distributional output heads, ~1.2M params). Not yet
trained on real data — `st_gat/data/` doesn't exist yet, and the existing
`st_gat/checkpoints/best_model.pth`/`st_gat/models/st_gat_rise.pth` predate the
2026-07-23 14-feature change (config.py already flags them stale) and this
session's new output heads below — **delete/ignore them, retrain from scratch.**

- [x] `traffic_light_state`/`traffic_light_discrepancy` output heads added
      2026-08-01 (`model.py`/`loss.py`) — previously only `traffic_light_detected`
      (a map fact, not actually uncertain) had a head, so no residual/NLL was
      computable on the actual perception-vs-map negative-evidence signal this
      dissertation's mechanism depends on. See `stgat_pipeline_plan.md` §1.12.
- [x] Horizon-tagged cache/checkpoint paths added 2026-08-01
      (`cfg.HORIZON_TAG`) — a horizon sweep (P1.3 below) needs to hold multiple
      `INPUT_SEQ_LEN`/`OUTPUT_SEQ_LEN` combinations' extracted data and
      checkpoints side by side without collision.
- [ ] Train and save model checkpoint on real extracted data — not yet done.
- [ ] Verify residuals are low on held-out nominal runs (raw residual < 1σ in
      majority of timesteps).

**Success criterion:** On held-out nominal data, mean raw residual < 0.5 m/s for
velocity features, < 1.0m for position features.

### 1.3 Residual Computation

**`st_gat/infer.py` retired 2026-08-01** — it was stale (referenced feature keys
that no longer match `cfg.FEATURE_SIZES`, would `KeyError` on model forward),
targeted the deprioritized `obs_recovery`/`obs_noescape` scenario campaigns
(scoped out, not even present in `experiments/data/`), and only logged one
CVaR95 number per run instead of a per-timestep trace. Replaced by
`st_gat/residuals.py`, built around the divergence trace schema in
`docs/theoretical_framework.md` §5:

- [x] `st_gat/residuals.py` written 2026-08-01: per-timestep trace (not
      per-window summary) over `cfg.NOMINAL_DATASETS` + `cfg.FAULT_DATASETS`
      (the actual campaigns on disk: `imu_fault_s1/s3/scale/stuck`,
      `tl_fault_s2/s3/s4/ramp`) — predicted mean/var + actual + raw residual per
      output feature (including the two new TL heads), map-expectation channel
      kept separate from perception-report channel, fault-onset-relative time
      (joined from `fault_log.jsonl`), and fatal-moment markers joined from
      `metrics.json` (`static_collision.permanent_stop_time_s`) and the
      first-`mrm_active` frame.
- [ ] Actually run it once a model is trained (1.2) and inspect real traces —
      not yet done.
- [ ] Plot residual traces for a clean nominal run and representative fault runs.

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

- [x] Prerequisite done 2026-08-01: cache/checkpoint paths are now horizon-tagged
      (`cfg.HORIZON_TAG`, see 1.2 above) so re-extracting/retraining at a
      different `INPUT_SEQ_LEN`/`OUTPUT_SEQ_LEN` no longer overwrites another
      horizon's data — sweeping is now mechanically possible, not just planned.
- [ ] Vary history and prediction window lengths
- [ ] **Report as lead time in seconds relative to the ground-truth safety
      event** (Arm B's Autoware MRM trigger timestamp, and/or the fatal-moment
      anchor in `docs/theoretical_framework.md` §6), not as accuracy deltas
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

### P1.6 Architecture ablation study (added 2026-08-02)

Several architecture decisions in `st_gat/model/model.py` were made by
judgment call, not measurement — worth grounding in an actual ablation once
the core mechanism (Priority 0) is established, rather than defending them
by intuition alone in the eventual write-up. See
`docs/research_notes/model_improvement_notes_2026.md` for the fuller
write-up (calibration mix, route/goal-signal dilution, ensemble/VAR_FLOOR/
object-pooling follow-ups) this list is a summary of.

- [ ] **Graph cadence**: `graph_ctx` is currently pooled once per window (all
      30 input timesteps share the same vector — see
      `docs/theoretical_framework.md` §4) vs. rebuilding/re-pooling it more
      finely. Measure whether finer cadence actually improves position/
      velocity tracking or fault-reaction lead time, rather than assuming
      either way.
- [ ] **Graph node count** (`MAX_GRAPH_NODES = 150`): untested whether this is
      enough, too many, or arbitrary relative to what the GCN actually uses —
      sweep it.
- [ ] **`d_model`/`d_graph`/`hidden_size`/`num_layers`/`nhead`** (all currently
      128/128/128/2/4, set once and never revisited): a real capacity sweep,
      not just "it trained and the loss went down."
- [ ] **Object set encoder** (added 2026-08-02): `MAX_TRACKED_OBJECTS=8`,
      `d_obj=32`, mean-pool vs. attention-pool — all first-pass choices, not
      measured against alternatives.
- [ ] **Route/goal signal utilization** (found 2026-08-02): the graph's
      `path_node` flag (real Autoware planned route, not a placeholder) is
      mean-pooled away with every other node before it reaches the temporal
      stream, diluting exactly the directional "which way does my route go"
      signal that should matter most right before a turn. Candidate: an
      explicit route-heading/next-turn scalar feature, or attention-weighted
      (not uniform mean) pooling — see the model improvement notes doc for
      the full reasoning. Worth checking whether this explains any of the
      remaining longer-horizon (3s) position error specifically.
- [ ] Report as: does the mechanism/calibration/lead-time result hold up
      across these choices (robustness), and does any of them materially
      change it (a real finding, not just noise)?

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
