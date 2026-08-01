# Theoretical Framework

**Last Updated:** 2026-08-01 — rewritten around Kalpit's sharper thesis framing
(calibrated confidence + lead time + severity as the contribution; detection is a
solved given). Supersedes the 2026-07-24 version. The prior version's mechanism
content (belief divergence, negative evidence, two-arm design) is not discarded —
it's folded in below as the mechanistic explanation underneath this thesis, not a
competing framing.

> **Status:** the mechanism (Section 3) is stated as a claim, not yet a proven
> result — Priority 0 in `TODO.md` (Experiments A & B) is what proves or kills it.
> Section 5 (calibration) is the next blocking piece of work — Stage 4 in
> `docs/stgat_pipeline_plan.md`, not yet started. The original RISE/active-control
> content is preserved in Section 11 as future-work, not deleted.

---

## 1. One-Sentence Thesis

This dissertation builds a reusable fault-injection and analysis apparatus that
measures, with calibrated confidence, how early a map-grounded digital twin
detects perception faults before they become fatal — characterized across fault
type and severity. Detection is treated as a solved given (established in the
T-ITS 2025 paper); the contribution is calibration, lead time, and the horizon
tradeoff that governs them.

---

## 2. What Changed From the Prior Paper

The 2025 paper showed fault detection is possible (93.7% classification accuracy)
using severe, unparameterized faults ("camera fault," "IMU fault"). That result is
now the **foundation**, not the claim. Re-running detection as the headline would
be incremental. The new work is defined by three things the prior paper physically
could not produce:

1. **Calibrated confidence, not accuracy.** The signal is a probability under a
   characterized model of healthy behavior, expressed in calibrated units (e.g.
   conformal p-values against a nominal calibration set), not raw residual
   magnitude or a static accuracy number.
2. **Lead time as the safety metric.** The headline result is how early, before a
   fault becomes fatal, a calibrated alarm crosses threshold — reframing detection
   from a classification result into a safety result.
3. **Severity as a studied axis.** Faults now have types and magnitudes (IMU,
   traffic-light/camera). This enables dose-response curves — detection
   confidence and lead time as monotonic functions of fault severity — that the
   prior categorical setup could not draw.

---

## 3. Epistemic Stance and Mechanism: Belief Divergence Under a Map-Grounded Prior

**This is introspection / self-assessment, not interpretability.** The goal is not
to open the perception black box or explain its internals. The goal is a reliable
external signal about the stack's competence, anchored against something outside
the model: a map-licensed ground-truth expectation. This anchoring is what makes
the self-assessment non-circular — the model is not grading its own homework.

The digital twin maintains an independent expectation of what the perception layer
should be reporting, derived from three sources:

1. **HD map (Lanelet2) annotations** — categorical assertions about what
   infrastructure exists at a given lanelet (e.g., "a signalized intersection is
   here").
2. **Learned nominal temporal patterns** — intersection timing, deceleration
   profiles, and other behavior the ST-GAT learns from nominal driving data.
3. **Vehicle dynamics context** — kinematic state that constrains what the
   vehicle should plausibly be observing next.

A fault is detected when the autonomy stack's perceptual belief diverges from that
independent expectation.

**The strong form is negative-evidence detection:** the map licenses a hard
categorical expectation ("a signalized intersection exists at this node"), and the
fault signature is perception *failing to report what must be there*. This is not
future work — it is already the dominant detection pathway in the published T-ITS
results:

- Camera fault is injected as pixel-level Gaussian noise (σ = 10) that degrades
  camera-based detection.
- Traffic light status is a camera-derived discrete feature, already present in
  the model's input/output state vector.
- Traffic Light Status Flag is the highest-importance feature at 29.7%, and its
  CUSUM residual is the single most discriminative feature-residual combination
  at 22%.

### 3.1 Negative Evidence: Explaining the Asymmetry

Negative evidence requires a hard prior. Traffic lights have one — the map
annotation categorically asserts "an intersection with a signal exists here."
Object detections do not — objects legitimately appear and disappear, so absence
of a detection is not, by itself, evidence of a fault.

This asymmetry **predicts**, rather than merely accommodates after the fact, two
observations already present in the published results:

- Traffic light features compress to very few PCA components; object detection
  flag features require more (a hard-prior signal is lower-dimensional/cleaner
  than a soft-prior one).
- LiDAR fault — which manifests primarily through the object detection flag —
  has the worst recall (0.850), with confusion into Camera (14 cases) and IMU
  (13 cases).

**Negative evidence works where the map licenses a hard expectation, and degrades
where it does not.** This is an explanatory account of the confusion matrix, not
just a description of it — and it is directly testable (Experiment A, `TODO.md`
Priority 0).

### 3.2 Why This Framing Matters

It elevates the contribution from fault detection (a signal deviated) to detecting
**epistemic failure of the autonomy stack** (the vehicle believes something false
about the world that the map and nominal behavior contradict). That is the
foundation for a safety-verification claim rather than a machine-learning accuracy
claim.

---

## 4. Horizon Is a Variable to Study, Not a Constant to Fix

The prediction horizon bounds the achievable lead time (you cannot warn earlier
than you can foresee) but does not determine it — realized lead time depends on
how fast the calibrated divergence signal accumulates past threshold. There is a
genuine tradeoff: longer horizons offer more theoretical lead time but noisier
predictions and weaker per-step divergence signal; shorter horizons are crisp but
cap earliness. This tradeoff curve (lead time vs. horizon length) is a **finding**,
with a likely sweet spot. Sweep it; do not pick it a priori.

**Design principles that keep the door open (do not violate these):**

- **Horizon parameterized end to end** — dataset builder, model output layer, and
  evaluation all take horizon as a config value, never a hardcoded constant.
- **Log full divergence traces, not summary statistics** — one row per timestep
  per trial. Lead time, thresholds, and calibration method are all applied
  offline against these traces, so curves can be redrawn without recollecting
  rosbags.

**Mantra: horizon parameterized, full traces logged, threshold applied offline.**

**Where this repo stands (2026-08-01):** `st_gat/pipeline/config.py`'s
`INPUT_SEQ_LEN`/`OUTPUT_SEQ_LEN` are config values already, but the sequence cache
and checkpoint paths were not horizon-tagged — fixed this session (`HORIZON_TAG`,
see `stgat_pipeline_plan.md`) so multiple horizons' extracted data/checkpoints can
coexist ahead of the actual sweep (Priority 1 / P1.3 in `TODO.md`).

---

## 5. Divergence Trace Schema

Per timestep, per trial:

- **Timestamp** — both sim time and time-relative-to-fault-onset (lead time is
  meaningless without fault-injection time).
- **Predicted state, observed state, and the model's own predicted uncertainty**
  for that step — kept as separate components, not collapsed, so raw residual and
  calibrated divergence can both be computed offline and calibration methods
  swapped later.
- **Map-licensed expectation vs. perception report** — a distinct channel from the
  continuous residual (the negative-evidence signal), analyzed on its own.
- **Ground-truth safety-outcome markers** — when the vehicle actually went
  off-road / collided. This is the "fatal moment" anchor; lead time is the gap
  between threshold-crossing and this.

**What this repo already has for this (2026-08-01):** fault-onset timestamps are
already logged per trial in `fault_log.jsonl`; the "map expectation" channel
(`traffic_light_detected`, a route/map fact) and the "perception report" channel
(`traffic_light_state`/`traffic_light_discrepancy`) already exist as separate
features in `st_gat/pipeline/sequence_builder.py` — but until this session, only
the map-fact channel had a model output head, so no residual existed yet on the
perception-report channel (see `stgat_pipeline_plan.md` §1.12, fixed). Fatal-moment
markers are addressed in Section 6 below. `st_gat/residuals.py` (replacing the
stale `infer.py`) is what actually assembles this schema into per-timestep CSVs —
see `stgat_pipeline_plan.md` Stage 5.

**What's still missing:** the calibration layer itself (Section 8, Stage 4) that
turns a raw residual column into a calibrated confidence value.

---

## 6. The Fatal-Moment Anchor

Off-road and collision are clean anchors, but some faults degrade safety without
a discrete catastrophic event. The fatal-moment definition — event-based vs.
threshold-based — propagates into every lead-time number reported, so it's worth
deciding early rather than backfilling it.

**This repo already computes both kinds of candidate anchor:**

- **Event-based (recommended primary anchor):** `metrics.json`'s
  `static_collision.permanent_stop_time_s` / `likely_static_collision` heuristic
  (Arm A) — a real "the vehicle stopped moving and never recovered" event,
  validated against multiple fault trials in the 2026-07-25 fault-sweep session
  (see `docs/research_notes/` history). Not literal ground-truth collision
  (AWSIM's own collision topic isn't wired to the ego prefab), but a defensible,
  already-implemented proxy.
- **Threshold-based (secondary / sensitivity check):** EKF-vs-ground-truth
  divergence crossing a distance bound (e.g. a lane-width), already computed by
  `experiments/scripts/compare_fault_vs_nominal.py`'s `ekf_gt_divergence`.

**Recommendation:** use the event-based heuristic as the primary fatal-moment
anchor for headline lead-time numbers, and report the threshold-based measure as a
sensitivity check, not a second independent claim. This is a modeling choice, not
a settled fact — worth one explicit sentence in the eventual write-up rather than
leaving it implicit.

---

## 7. The Claim to Defend

> This digital-twin framework detects when the autonomy stack's perceptual belief
> has diverged from a map-grounded, independently derived expectation of the
> world; it reports that divergence with calibrated confidence, a bounded number
> of seconds of lead time before the divergence degrades vehicle safety.

Three load-bearing pieces, each mapping to a section above:

1. **Belief divergence / negative evidence** (Section 3) — established as a
   mechanism, not an observed correlation. Requires Experiments A & B (`TODO.md`
   Priority 0).
2. **Calibrated confidence** (Section 8) — the uncertainty-quantification /
   conformal-prediction contribution. The headline result is a calibration curve,
   not an accuracy number: when the monitor reports 70% confidence it should be
   right ~70% of the time. This is what converts a classifier into a safety
   monitor that can be trusted — a monitor that is confidently wrong is worse
   than useless.
3. **Lead time** (Sections 4, 6) — the prediction-horizon study, reframed from a
   hyperparameter sweep into a safety result, measured against the fatal-moment
   anchor and against Autoware's own reactive MRM trigger (Arm B, Section 9) as
   ground truth.

**Framing shift applied throughout the dissertation:** *detection* ("fault:
yes/no," already demonstrated and banked from the T-ITS paper) versus
*verification* (a calibrated statement about confidence, including knowing when
the monitor does not know). Uncertainty quantification and interpretability are
core to the contribution, not a side quest.

**Scoping corollary:** scenario-based avoidance demonstrations (cut-in, unexpected
stopped vehicle, the `obs_*` campaigns) are contributions to control and handling,
not to safety verification. They are de-emphasized to illustrative context; no new
staged-avoidance experiments are planned. See Section 11 and `TODO.md` for what
this deliberately excludes and why.

**Mental model:** the ST-GAT pipeline is not the deliverable — it is the
instrument. The deliverable is the characterized experimental regime: the fault
library (types × magnitudes), closed-loop collection, trace logging, and an
offline analysis layer that turns traces into calibrated curves. Get the
apparatus right and the papers fall out of it: run a campaign → get a trace →
apply calibration offline → draw a curve → produce a figure.

---

## 8. Calibration — Methods Menu (Reference, Not Yet Built)

**Status: Stage 4 in `docs/stgat_pipeline_plan.md` — not started.** Listed here
for reference, not all required:

- **Conformal prediction** — distribution-free, converts a divergence score into
  a rigorous statement against a nominal calibration set ("beyond the 95th
  percentile of healthy behavior"), yields coverage guarantees.
- **Heteroscedastic / predicted variance** — model predicts its own per-situation
  error bars, so the same residual means different things in different contexts
  (context-aware threshold rather than one global cutoff).
- **Sequential / accumulated evidence** — treat detection as evidence accumulated
  over time rather than per-frame, which is what makes lead-time claims honest.

**Severity axis note:** the severity axis (Section 2, item 3) is load-bearing for
the eventual dose-response curves, so fault-magnitude parameterization needs to be
explicit and documented per fault type (IMU bias in what units, traffic-light
corruption at what rate, etc.) — see `docs/fault_scenario_table.md` for the
current per-campaign parameterization; keep it current as new severity tiers are
added.

---

## 9. Two-Arm Fault Injection Design

All fault-injection experiments run in both arms.

- **Arm A — Autoware built-in safety features DISABLED (the science
  condition).** Purpose: let the injected fault propagate so the full trajectory
  from healthy to degraded to dangerous is observable. With the built-in safety
  layer active, Autoware amputates the failure before it can be studied. Answers:
  can the framework detect and characterize the fault, and how early?
- **Arm B — Autoware built-in safety features ENABLED (the ground-truth
  oracle).** Autoware's reactive safety trigger firing is treated as a labeled
  ground-truth moment of "this was objectively unsafe." Not a baseline
  competitor — the validation signal. Answers: did the framework raise calibrated
  uncertainty *N* seconds before Autoware's hard trigger fired?

This resolves a redundancy concern head-on: Autoware's safety features are
reactive and rule-based — they answer "should I stop now?" This framework is
predictive and epistemic — it answers "has the stack's belief about the world
diverged from what the map and nominal behavior imply, and can that be seen
coming?" It is not a replacement and not a standalone alternative — it is an
early-warning layer, and Autoware's own reactive trigger is the yardstick that
demonstrates earliness. **Lead time measured against Arm B is the primary
evidence for the claim.**

**Natural fit with the negative-evidence mechanism:** with safety features
disabled, a camera occlusion that suppresses traffic light detection produces a
concrete, unambiguous safety violation (proceeding through a signalized
intersection on a red) — a hard ground-truth event with a well-defined timestamp,
making lead time cleanly measurable.

**Implementation status:** the repo has both MRM diagnostic-gate configurations
(Arm A and Arm B), toggleable via `experiments/scripts/switch_diagnostic_arm.sh`.
Arm B hasn't been run against a real fault campaign yet, so whether it
reintroduces the deadlocks Arm A was built to avoid is still open. See `TODO.md`'s
"Two-Arm Fault Injection Design" section and `docs/design_decisions.md` item 5.

---

## 10. Relationship to the T-ITS Paper

The dissertation is not a restatement of the published paper — it explains *why*
the paper's numbers came out the way they did, and builds evidence the paper
doesn't have:

| T-ITS paper (published, banked) | Dissertation (this framework) |
|---|---|
| 93.7% fault detection accuracy | *Why* — negative evidence via map-grounded prior, tested by ablation (Experiment B) |
| TL Status Flag 29.7% importance, TL×CUSUM 22% top combo | *Why* — hard prior vs. soft prior asymmetry, tested per-fault-class (Experiment A) |
| LiDAR worst recall (0.850), confused with Camera/IMU | *Why* — object-detection flag lacks a hard map prior (Section 3.1) |
| Descriptive: "perception + dynamics give strongest signal" | Mechanistic: belief divergence against an independent, map-grounded expectation |
| Point accuracy metric | Calibration curve — confidence that matches empirical correctness |
| No lead-time result | Lead time in seconds, measured against Arm B's MRM trigger and the fatal-moment anchor (Section 6) |
| Table II ablation: trajectory quality only (minADE/minFDE/MR) | Table II + detection-performance ablation (Experiment B) |
| Categorical faults only | Severity-parameterized faults → dose-response curves (Section 2, item 3) |
| Limitation: single map (Nishishinjuku), unaddressed | Reframed as a testable generalization claim tied to map annotation density (Section 4/P1.5 in TODO.md) |

---

## 11. Future Work: Active Control ("RISE") — Preserved, Not Under Defense

**Status: not the dissertation's core contribution.** The content below framed the
residual signal as feeding an active controller that relaxes velocity constraints
under uncertainty. Per the scoping corollary in Section 7, this is a
control/handling contribution, kept as a coherent future-work chapter, not
deleted, but not required for or evidence toward the safety-verification claim
above.

### 11.1 Original Core Concept

RISE uses ST-GAT digital twin prediction residuals to dynamically adjust safety
constraints. When predictions diverge from observations, something unexpected is
happening — safety margins should increase.

```
residual signal → anomaly score → constraint adjustment
```

The prior T-ITS 2025 paper used these residuals for **passive fault detection**
(classify driving state). RISE proposed using them for **active control feedback**
(adjust constraints continuously based on residual magnitude). The current
dissertation direction instead uses them for **calibrated verification** (report a
trustworthy, quantified belief-divergence signal with lead time) — control remains
a plausible downstream consumer of that signal, not the thing being defended.

### 11.2 Residual Types (from T-ITS 2025 — still in active use for the current framing)

| Type | Formula | Captures |
|------|---------|---------|
| Raw | `Φ_raw = μ - u` | Direct prediction error |
| KL-divergence | `Φ_kld = 0.5[ln(2πσ²) + (u-μ)²/σ²]` | Error normalized by predicted uncertainty |
| CUSUM | `Φ_cusum = max(C⁺, C⁻) / d` | Cumulative drift from baseline |

These residual types remain load-bearing under the new framing too — they're the
raw material calibration (Section 8) is built on. Only the *purpose* they're put
to (control-constraint input vs. calibrated confidence output) has changed.

### 11.3 Anomaly Score → Constraint Adjustment (control-track, deprioritized)

Candidate anomaly-score approaches (CVaR-based, normalized magnitude,
trend-based) and candidate constraint-mapping approaches (monotonic functions,
uncertainty propagation / tube-MPC style, empirically calibrated mapping) remain
valid ideas for a future control-extension chapter. Not under active development.

### 11.4 Fail-Operational Behavior (control-track framing)

| Condition | Residuals | Constraints | Vehicle Behavior |
|-----------|-----------|-------------|-----------------|
| Normal | Low | ≈ Nominal | Full mission, normal speed |
| Elevated risk | Moderate | Moderately tightened | Mission continues, slower |
| High risk | High | Significantly tightened | Mission continues, very slow |
| MRM threshold | (out of RISE scope) | MRM takes over | Emergency stop |

### 11.5 Framework Evolution (history, unchanged)

**v1: CVaR threshold mapping** — Compute CVaR_α over residual window; apply
margin function γ(CVaR). Limitation: no principled way to set the margin
function's parameters.

**v2: Learned sensitivity matrix** — Learn a matrix A mapping residual vector to
constraints. Rejected: requires RL-like training data not available.

**v3: Uncertainty propagation (covariance inflation)** — Inflate predicted
covariance proportional to residual magnitude, compute a safety tube width.
Principled, no training data needed; open question was giving the tube width a
meaningful physical interpretation in the Autoware constraint context.

### 11.6 Original Formal Guarantee Goal

> "With probability ≥ 1-δ, if the RISE constraint is active and the anomaly score
> is below threshold τ, the constraint will not be violated within the planning
> horizon."

Superseded by the calibration-curve framing in Section 8 — the guarantee under
the new claim is about calibrated confidence in the *detection* signal, not about
constraint-violation probability under an active controller.
