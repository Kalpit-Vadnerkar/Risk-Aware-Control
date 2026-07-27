# Theoretical Framework

**Last Updated:** 2026-07-24 — replaces the RISE-centric version of this document.

> **Status:** the mechanism (Section 1) is stated as a claim, not yet a proven
> result — Priority 0 in `TODO.md` (Experiments A & B) is what proves or kills it.
> The calibration and lead-time pieces (Sections 3–4) depend on that mechanism
> holding up. The original content of this document (residual → anomaly score →
> constraint adjustment, "RISE") is preserved in Section 6 as future-work, not
> deleted — it is no longer the dissertation's primary theoretical framework.

---

## 1. Core Mechanism: Belief Divergence Under a Map-Grounded Prior

The framework detects faults through **belief divergence under a map-grounded
prior**, not through generic anomaly detection on residual signals.

The digital twin maintains an independent expectation of what the perception
layer should be reporting, derived from three sources:

1. **HD map (Lanelet2) annotations** — categorical assertions about what
   infrastructure exists at a given lanelet (e.g., "a signalized intersection is
   here").
2. **Learned nominal temporal patterns** — intersection timing, deceleration
   profiles, and other behavior the ST-GAT learns from nominal driving data.
3. **Vehicle dynamics context** — kinematic state that constrains what the
   vehicle should plausibly be observing next.

A fault is detected when the autonomy stack's perceptual belief diverges from
that independent expectation.

**The strong form is negative-evidence detection:** the map licenses a hard
categorical expectation ("a signalized intersection exists at this node"), and
the fault signature is perception *failing to report what must be there*. This is
not future work — it is already the dominant detection pathway in the published
T-ITS results:

- Camera fault is injected as pixel-level Gaussian noise (σ = 10) that degrades
  camera-based detection.
- Traffic light status is a camera-derived discrete feature, already present in
  the model's input/output state vector.
- Traffic Light Status Flag is the highest-importance feature at 29.7%, and its
  CUSUM residual is the single most discriminative feature-residual combination
  at 22%.

This replaces the paper's purely descriptive framing ("perception features
combined with vehicle dynamics provide the strongest fault detection signals")
with a mechanistic claim: those features win *because* they're where a hard,
map-derived prior exists to diverge from.

### Why this framing matters

It elevates the contribution from fault detection (a signal deviated) to
detecting **epistemic failure of the autonomy stack** (the vehicle believes
something false about the world that the map and nominal behavior contradict).
That is the foundation for a safety-verification claim rather than a
machine-learning accuracy claim.

---

## 2. Negative Evidence: Explaining the Asymmetry

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

**Negative evidence works where the map licenses a hard expectation, and
degrades where it does not.** This is an explanatory account of the confusion
matrix, not just a description of it — and it is directly testable (Experiment A,
`TODO.md` Priority 0).

---

## 3. The Claim to Defend

> This digital-twin framework detects when the autonomy stack's perceptual belief
> has diverged from a map-grounded, independently derived expectation of the
> world; it reports that divergence with calibrated confidence, a bounded number
> of seconds before the divergence degrades vehicle safety.

Three load-bearing pieces, each mapping to a result:

1. **Belief divergence / negative evidence** (Sections 1–2 above) — established
   as a mechanism, not an observed correlation. Requires Experiments A & B
   (`TODO.md` Priority 0).
2. **Calibrated confidence** — the uncertainty-quantification / conformal-
   prediction contribution. The headline result is a calibration curve, not an
   accuracy number: when the monitor reports 70% confidence it should be right
   ~70% of the time. This is what converts a classifier into a safety monitor
   that can be trusted — a monitor that is confidently wrong is worse than
   useless.
3. **Lead time** — the prediction-horizon study (history and prediction window
   lengths) reframed from a hyperparameter sweep into a safety result: how much
   advance warning does the framework provide before the fault compromises
   safety, measured against Autoware's own reactive MRM trigger (Arm B, Section
   4) as ground truth.

**Framing shift applied throughout the dissertation:** *detection* ("fault:
yes/no," already demonstrated and banked from the T-ITS paper) versus
*verification* (a calibrated statement about confidence, including knowing when
the monitor does not know). Uncertainty quantification and interpretability are
core to the contribution, not a side quest.

**Scoping corollary:** scenario-based avoidance demonstrations (cut-in,
unexpected stopped vehicle, the `obs_*` campaigns) are contributions to control
and handling, not to safety verification. They are de-emphasized to illustrative
context; no new staged-avoidance experiments are planned. See Section 6 and
`TODO.md` for what this deliberately excludes and why.

---

## 4. Two-Arm Fault Injection Design

All fault-injection experiments run in both arms.

- **Arm A — Autoware built-in safety features DISABLED (the science
  condition).** Purpose: let the injected fault propagate so the full trajectory
  from healthy to degraded to dangerous is observable. With the built-in safety
  layer active, Autoware amputates the failure before it can be studied.
  Answers: can the framework detect and characterize the fault, and how early?
- **Arm B — Autoware built-in safety features ENABLED (the ground-truth
  oracle).** Autoware's reactive safety trigger firing is treated as a labeled
  ground-truth moment of "this was objectively unsafe." Not a baseline
  competitor — the validation signal. Answers: did the framework raise
  calibrated uncertainty *N* seconds before Autoware's hard trigger fired?

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
intersection on a red) — a hard ground-truth event with a well-defined
timestamp, making lead time cleanly measurable.

**Implementation gap (2026-07-25 — Arm B config built, not yet validated live):**
the repo now has both MRM diagnostic-gate configurations — Arm A (README.md item
3 — perception/planning/localization stripped out to prevent deadlocks during
experiment resets) and Arm B (stock/full gate, restoring Autoware's default
linkage exactly), toggleable via `experiments/scripts/switch_diagnostic_arm.sh`.
Arm B hasn't been run against a real fault campaign yet, so whether it
reintroduces the deadlocks Arm A was built to avoid is still open. See
`TODO.md`'s "Two-Arm Fault Injection Design" section and
`docs/design_decisions.md` item 5.

---

## 5. Relationship to the T-ITS Paper

The dissertation is not a restatement of the published paper — it explains *why*
the paper's numbers came out the way they did, and builds evidence the paper
doesn't have:

| T-ITS paper (published, banked) | Dissertation (this framework) |
|---|---|
| 93.7% fault detection accuracy | *Why* — negative evidence via map-grounded prior, tested by ablation (Experiment B) |
| TL Status Flag 29.7% importance, TL×CUSUM 22% top combo | *Why* — hard prior vs. soft prior asymmetry, tested per-fault-class (Experiment A) |
| LiDAR worst recall (0.850), confused with Camera/IMU | *Why* — object-detection flag lacks a hard map prior (Section 2) |
| Descriptive: "perception + dynamics give strongest signal" | Mechanistic: belief divergence against an independent, map-grounded expectation |
| Point accuracy metric | Calibration curve — confidence that matches empirical correctness |
| No lead-time result | Lead time in seconds, measured against Arm B's MRM trigger |
| Table II ablation: trajectory quality only (minADE/minFDE/MR) | Table II + detection-performance ablation (Experiment B) |
| Limitation: single map (Nishishinjuku), unaddressed | Reframed as a testable generalization claim tied to map annotation density (Section 4/P1.5 in TODO.md) |

---

## 6. Future Work: Active Control ("RISE") — original content of this document, preserved

**Status as of 2026-07-24: not the dissertation's core contribution.** The
content below (originally the entirety of this document) framed the residual
signal as feeding an active controller that relaxes velocity constraints under
uncertainty. Per the scoping corollary in Section 3, this is a control/handling
contribution, kept as a coherent future-work chapter, not deleted, but not
required for or evidence toward the safety-verification claim above.

### 6.1 Original Core Concept

RISE uses ST-GAT digital twin prediction residuals to dynamically adjust safety
constraints. When predictions diverge from observations, something unexpected is
happening — safety margins should increase.

```
residual signal → anomaly score → constraint adjustment
```

The prior T-ITS 2025 paper used these residuals for **passive fault detection**
(classify driving state). RISE proposed using them for **active control
feedback** (adjust constraints continuously based on residual magnitude). The
current dissertation direction instead uses them for **calibrated verification**
(report a trustworthy, quantified belief-divergence signal with lead time) —
control remains a plausible downstream consumer of that signal, not the thing
being defended.

### 6.2 Residual Types (from T-ITS 2025 — still in active use for the current framing)

| Type | Formula | Captures |
|------|---------|---------|
| Raw | `Φ_raw = μ - u` | Direct prediction error |
| KL-divergence | `Φ_kld = 0.5[ln(2πσ²) + (u-μ)²/σ²]` | Error normalized by predicted uncertainty |
| CUSUM | `Φ_cusum = max(C⁺, C⁻) / d` | Cumulative drift from baseline |

These residual types remain load-bearing under the new framing too — they're the
raw material calibration (Section 3, piece 2) is built on. Only the *purpose*
they're put to (control-constraint input vs. calibrated confidence output) has
changed.

### 6.3 Anomaly Score → Constraint Adjustment (control-track, deprioritized)

Candidate anomaly-score approaches (CVaR-based, normalized magnitude,
trend-based) and candidate constraint-mapping approaches (monotonic functions,
uncertainty propagation / tube-MPC style, empirically calibrated mapping) remain
valid ideas for a future control-extension chapter. Not under active development.

### 6.4 Fail-Operational Behavior (control-track framing)

| Condition | Residuals | Constraints | Vehicle Behavior |
|-----------|-----------|-------------|-----------------|
| Normal | Low | ≈ Nominal | Full mission, normal speed |
| Elevated risk | Moderate | Moderately tightened | Mission continues, slower |
| High risk | High | Significantly tightened | Mission continues, very slow |
| MRM threshold | (out of RISE scope) | MRM takes over | Emergency stop |

### 6.5 Framework Evolution (history, unchanged)

**v1: CVaR threshold mapping** — Compute CVaR_α over residual window; apply
margin function γ(CVaR). Limitation: no principled way to set the margin
function's parameters.

**v2: Learned sensitivity matrix** — Learn a matrix A mapping residual vector to
constraints. Rejected: requires RL-like training data not available.

**v3: Uncertainty propagation (covariance inflation)** — Inflate predicted
covariance proportional to residual magnitude, compute a safety tube width.
Principled, no training data needed; open question was giving the tube width a
meaningful physical interpretation in the Autoware constraint context.

### 6.6 Original Formal Guarantee Goal

> "With probability ≥ 1-δ, if the RISE constraint is active and the anomaly score
> is below threshold τ, the constraint will not be violated within the planning
> horizon."

Superseded by the calibration-curve framing in Section 3 (piece 2) — the
guarantee under the new claim is about calibrated confidence in the *detection*
signal, not about constraint-violation probability under an active controller.
