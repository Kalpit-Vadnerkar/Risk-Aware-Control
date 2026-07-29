# ST-GAT Pipeline Plan

**Last Updated:** 2026-07-28 — added §1.10 and Stage 0 notes on entity-collapsing
features, compute budget, and topic time-sync, from a real bug found live in
this repo's own fault-comparison analysis code (not just the reference repo).

**Originally drafted:** 2026-07-26 — first draft, written before any implementation.
Not started yet: fault-campaign data collection (see `TODO.md`) is still being
finalized. This is a planning document to be discussed and revised, not a
committed design.

> **Status:** no code exists for this yet. Everything below is a plan to
> critique and modify before implementation begins — see `TODO.md` for
> current phase status.

---

## 0. Guiding constraint

Every stage below has a way to optimize for "looks accurate" that actively
works against "tells us something trustworthy about safety." The published
T-ITS paper's 93.7% accuracy number is a **detection** result; this
dissertation's job is **verification** — calibrated confidence and lead time,
not a bigger accuracy number. Where a design choice could pull those two goals
in different directions, that tension is called out explicitly rather than
defaulted past.

This document leans heavily on lessons pulled directly from reading
`../Graph-Scene-Representation-and-Prediction` (the published paper's own
repo, READ-ONLY reference — see `CLAUDE.md`), not generic ML advice. Several
concrete, previously-undocumented bugs were found there in the course of
writing this plan (2026-07-26) — they're recorded below because they're
exactly the kind of mistake this pipeline needs to avoid repeating.

---

## 1. Confirmed problems in the reference repo (read, not assumed)

**1.1 — The map projector bug, confirmed and worse than "wrong numbers."**
`State_Estimator/MapProcessor.py:61`:
```python
projector = lanelet2.projection.LocalCartesianProjector(lanelet2.io.Origin(35.67, 139.65, 0))
```
A generic projector with a hand-guessed lat/lon origin — the same class of bug
found and fixed in this repo for `fault_injector.py`/`plot_routes.py`/
`explore_map.py` (off by 1000+ meters from the real AWSIM/Autoware frame). The
consequence isn't just "positions are shifted": any "is ego near a real
traffic light" computation done through this map is checking proximity in the
*wrong frame*. The published paper's headline result is that **Traffic Light
Status Flag is the single most discriminative feature (29.7% importance)** —
a feature whose meaning depends entirely on correct map-grounding. This is a
real question mark over that specific result, not a cosmetic issue.

**Rule for this pipeline: `MGRSProjector(lanelet2.io.Origin(0.0, 0.0))`
everywhere a map is loaded, no exceptions.** Already the established pattern
in `plot_routes.py`/`explore_map.py`/`fault_injector.py`. Worth a grep-based
check (pre-commit or CI) for `UtmProjector`/`LocalCartesianProjector` showing
up anywhere new, since this is exactly the kind of bug that doesn't crash and
doesn't look wrong.

**1.2 — `CUSUMResidual.set_delta()` is a confirmed no-op.**
```python
def set_delta(self, value):
    self.k - value          # computed, never assigned or returned
```
`ResidualGenerator.compute_residuals()` calls this before every CUSUM
computation, intending to apply a per-feature sensitivity
(`DELTA_VALUES[feature]`). It never took effect — every feature used the same
default `k=0.5`. CUSUM was the paper's best-performing residual type; it got
there *without* the per-feature tuning the code thought it was applying.

**Lesson:** any stateful setter/mutator needs a unit test asserting the state
actually changed. This bug would show up as a one-line assertion failure and
instead sat invisible through a full publication.

**1.3 — Stateful residual detectors risk cross-trial contamination.**
`ResidualGenerator.__init__` builds `CUSUMResidual()` (and Shewart/SPRT) once;
`self.Cp`/`self.Cn` persist across every call to `compute_residuals()`. Unless
the caller constructs a fresh `ResidualGenerator` per trial, a trial's CUSUM
statistic starts carrying drift from wherever the *previous* trial in the loop
left off. Not confirmed as actually manifesting in the published results
(didn't trace the exact orchestration loop) — but the design doesn't defend
against the mistake, which is the actual lesson.

**Rule for this pipeline:** any stateful detector is constructed fresh per
trial, with a test that literally checks two consecutive trials produce
identical scores at t=0 regardless of what happened in the prior trial.

**1.4 — The uncertainty-quantification loss has a disabled safety valve.**
`Prediction_Model/LossFunctions.py`:
```python
#variance = torch.clamp(pred[var_key], min=self.min_var)
```
Commented out, with `epsilon=0`. The Gaussian NLL divides by
`(variance + epsilon)` — with no floor, a variance that underflows toward zero
(which `softplus` can produce) risks NaN losses, and even short of NaN,
near-zero predicted variance on an "easy" example is a well-known
heteroscedastic-regression failure mode: the model learns to be falsely,
catastrophically confident instead of genuinely well-calibrated. This matters
most to this dissertation specifically, since calibrated confidence is the
actual deliverable.

**Fix:** a real variance floor (not commented out), non-zero epsilon, or
predict log-variance instead of variance-via-softplus if instability persists
(numerically nicer, can't go non-positive).

**1.5 — Task losses summed with equal weight across incompatible scales.**
Position (meters), steering (radians), object_distance ([0,1]) all get
`reg_weights[key] = 1.0` by default — whichever feature has the largest raw
squared error dominates the gradient. Worth normalizing targets or handling
per-task weighting deliberately, not left at a uniform default.

**1.6 — The graph representation is static per training sequence, not per
timestep.** One graph per sequence, mean-pooled over all nodes into a single
vector, then repeated identically across every timestep in the input window
(`TrajectoryDataset.py`: one `sequence['graph']` per sample; `DLModels.py`:
`.unsqueeze(1).repeat(1, self.input_seq_len, 1)`). For a "negative evidence"
framing where the map-grounded prior is supposed to reflect *where the vehicle
currently is*, freezing that for the whole window blurs exactly the signal
that matters most as the vehicle actually moves through an intersection. This
is a deliberate decision to re-litigate (§2 below), not an inherited default.

**1.7 — Normalization/scaling factors are constructor defaults, not saved
artifacts.** `TrajectoryDataset.__init__(position_scaling_factor=10, ...)` —
hardcoded, not derived from training-set statistics, not versioned alongside
the model. Classic train/inference skew bug: if the value used at training
time and the value used at inference time ever diverge (different code path,
forgotten kwarg), predictions go silently wrong with no error.

**Rule for this pipeline:** normalization stats computed once (from the
training split only), saved as a versioned artifact next to the model
checkpoint, never recomputed or defaulted at inference.

**1.8 — Sample-level train/test split for the fault classifier, not
trial-level.** `Risk_Assessment/FaultDetector.py`:
```python
X_train, X_test, y_train, y_test = train_test_split(X, y_multi, test_size=0.2,
                                                      random_state=47, stratify=y_multi)
```
No grouping by trial. If `X` is built from many timesteps/residual-windows
drawn from continuous trials, adjacent samples from the *same* trial are
highly autocorrelated and can land on both sides of the split — leakage that
inflates reported accuracy with no obvious symptom. The `StratifiedKFold`
cross-validation right below it has the identical issue.

**Rule for this pipeline: `GroupShuffleSplit`/`GroupKFold` keyed by trial ID,
always, no exceptions** — from the first version of any training/evaluation
script, not retrofitted later.

**1.9 — What the reference repo got right, worth reusing.** The deep-ensemble
uncertainty combination (`Risk_Assessment/DataLoader.py:
_calculate_epistemic_uncertainty`) correctly implements the standard
decomposition — combined variance = mean(individual variances + means²) −
(combined mean)², i.e. aleatoric + epistemic via the law of total variance,
with a sane `1e-6` variance floor on the *output* (contrast with 1.4's
disabled floor on the *training* loss). Reuse the formula, not the
surrounding plumbing (ensemble loading via bare `os.listdir` with no
extension filter is a minor robustness gap).

**1.10 — Hand-engineered feature collapse hid a real, confirmed-present fault
signature (found live in this repo's own analysis code, 2026-07-28, not the
reference repo).** `compare_fault_vs_nominal.py`'s original TL feature
computation pooled elements across *every* currently-tracked
`traffic_light_group` in a message (2-6 at once, whatever's in the planning
lookahead) into one mean-confidence / max-confidence-color scalar. A TL fault
only ever mutates the ONE `group_id` the vehicle's route is actually governed
by right now — for `tl_fault_s4` (total blackout of that one light), the
corrupted group's empty/zero-confidence elements were simply outvoted by
whichever *other*, unfaulted group in the same message had higher confidence.
Confirmed at the raw signal level that the fault was real and total (in-fault
detection rate for the targeted group: 0.06%) — the pooled feature nonetheless
showed 0/6 features discriminable. Fixed by scoping extraction to the one
`group_id` nearest the vehicle's current position (same selection
`fault_injector.py` itself uses to target a fault), replicated per-message,
not computed once and reused.

**Lesson, generalized beyond TL:** a feature-engineering step that collapses
across entities (multiple tracked objects, multiple traffic-light groups,
multiple lanelets in view) before the model or detector ever sees the data can
make a real, present fault signature statistically invisible — not because the
fault has no signature, but because the collapse discarded *which entity* the
signal belonged to before anything downstream had a chance to use it. This is
now a standing risk to check for explicitly in Stage 0 below, not just a
TL-specific bug that got fixed once.

---

## 2. Pipeline plan, stage by stage

### Stage 0 — State/feature definition (map-grounded, before any model code)
- Every map query uses `MGRSProjector` (§1.1). Consider a lint-level check.
- Decide explicitly, in writing, what "TL Status Flag" (and any other
  map-derived categorical feature) *means* and *when it's allowed to be
  positive* — this feature is the whole negative-evidence mechanism, so its
  correctness deserves its own validation step (plot it against ground-truth
  map positions the way `plot_routes.py` already does for traffic lights)
  before it reaches the model.
- Decide graph *cadence* explicitly (§1.6): one graph per sequence (matches
  the reference repo, cheaper) vs. one graph per timestep (more faithful to
  "belief divergence right now," more expensive). Given the core claim is
  about *moment-of-divergence* detection, lean toward per-timestep or at
  least per-short-subwindow — re-litigate the reference repo's choice, don't
  inherit it by default.
- **No entity-collapsing features (§1.10).** State per relevant entity (the
  TL group(s) actually governing the vehicle's current lanelet, tracked
  objects individually, not a scene-wide min/mean) rather than pre-reducing
  across entities into one scalar before the model sees it — a graph model
  exists specifically so the *architecture* can learn what to attend to
  across entities; don't let a preprocessing step make that decision first
  and silently. This doesn't mean "feed raw topic bytes" — the map-grounded
  prior is itself structural (which `group_id` governs this lanelet, HD map
  connectivity), and the graph's node/edge structure should carry that, not
  discard it. The distinction that matters: don't collapse *which entity* a
  signal belongs to; do keep the structural/categorical context (map graph,
  entity identity) the negative-evidence mechanism depends on.
- **Compute budget is a real constraint, not an afterthought.** The lead-time
  claim (§ Stage 6, `docs/theoretical_framework.md`) is only meaningful if the
  state representation can be computed fast enough to matter for an online
  detector, not just a post-hoc analysis script — a feature step that's fine
  for offline discriminability ranking (this repo's current
  `compare_fault_vs_nominal.py`) may be too slow (e.g. per-message map
  queries against the full lanelet graph) for anything resembling real-time.
  Worth timing the Stage 0 feature computation itself once it exists, against
  whatever cadence the model needs, before assuming it scales.
- **Time-sync across topics, explicitly, before windowing.** Input topics
  publish at different, independent rates (GT/IMU near sensor rate,
  perception/planning slower, TL recognition slower still) and aren't
  guaranteed in-phase with each other. Decide and document the
  resampling/alignment strategy (nearest-neighbor snap onto a fixed grid, as
  `compare_fault_vs_nominal.py`'s `resample()` currently does for offline
  analysis, vs. proper interpolation) rather than letting whatever a naive
  per-topic loop produces define window boundaries by accident — a jittery
  or inconsistently-phased alignment corrupts exactly the temporal structure
  the "ST" in ST-GAT, and the time-horizon/lead-time claim built on it,
  depend on.

### Stage 1 — Dataset construction
- Split at the **trial level** before any windowing happens — whole driving
  sessions (including fault trials) assigned wholesale to train/val/test,
  never individual overlapping windows shuffled post-hoc. Applies to
  nominal-vs-nominal splits too, not just the fault-detector stage.
- Compute normalization statistics once, from the training split only (using
  val/test statistics anywhere is its own leakage), save as a versioned
  artifact alongside the model checkpoint (§1.7).
- Be memory-conscious about sequence loading — the reference repo's
  `pickle.load` + `list.extend` pattern loads the entire dataset into RAM at
  Dataset-construction time. Given this machine's OOM history, prefer
  lazy/per-file loading if the eventual dataset (nominal + all fault
  campaigns × goals × trials) gets large, rather than assuming it always fits.

### Stage 2 — Model architecture
- Keep the distributional heads (mean + variance per continuous feature,
  sigmoid for categorical) — sound, and exactly what "embrace distributions"
  requires.
- Batch the graph convolution properly (PyTorch Geometric or an explicit
  batched sparse op) instead of the reference repo's per-sample Python loop —
  pure performance, but matters at dataset scale.
- Implement the graph pooling/cadence decision from Stage 0 explicitly in the
  architecture, don't let it fall out of a copy-pasted `forward()`.

### Stage 3 — Training
- Real variance floor, non-zero epsilon, or log-variance parameterization —
  pick one, stress-test it against a fault trial's corrupted data (feed it
  early, before the ensemble even exists, as an adversarial-ish smoke test).
- Deliberate per-task loss weighting or target normalization, not silent
  equal-weighting (§1.5).
- Deep ensemble: same recipe as the reference repo (independent seeds, same
  data) — that part's fine, keep it (§1.9).

### Stage 4 — Calibration (the actual novel contribution)
- Conformal prediction as a **post-hoc calibration layer** on top of the
  trained ensemble's predictive distributions, not baked into the training
  loss. Given 1.4's instability, keeping calibration separate, simpler, and
  more auditable is a feature, not a limitation — conformal guarantees are
  about the calibration step, not the underlying model, so they don't need
  the model's internal uncertainty to already be perfect.
- Calibration set must be its own trial-level split, disjoint from both
  training and whatever lead-time numbers eventually get reported — a
  three-way split (train/calibrate/evaluate), not two.

### Stage 5 — Residual/detection layer
- Reimplement CUSUM (and whichever of Raw/KL are kept)
  **stateless-by-construction between trials** — a fresh instance per trial,
  with the corresponding unit test (§1.3).
- Fix the actual bug: make per-feature delta/sensitivity really apply, or
  deliberately remove the dead parameter rather than keep a broken knob that
  looks live (§1.2).
- Any classifier fit on residual features (the Random Forest step):
  `GroupKFold`/`GroupShuffleSplit` keyed by trial ID, always (§1.8).

### Stage 6 — Evaluation, reframed around the actual claim
- Report calibration curves (are the confidence intervals actually right X%
  of the time), not just accuracy.
- Report lead time against the ground-truth events this repo already has
  infrastructure for (Arm A's `static_collision`/permanent-stop heuristic, or
  an EKF-divergence-crosses-a-lane-width threshold) — connects directly to
  the Arm A fault data once collection is finalized.
- Keep an explicit "what would fool this" section — e.g., does the detector
  fire on the same benign `control_validator` noise that caused trouble under
  Arm B, even though the model never saw Arm B data? A real robustness
  question worth designing a test for, not assuming away.

---

## 3. Suggested build order

Start with **Stage 0 + Stage 1** (state definition and dataset construction)
once fault-campaign data collection is finalized — that's where the reference
repo's worst, least-visible bugs lived (§1.1, §1.7, §1.8). Get that foundation
reviewed before any model code exists, rather than discovering a frame or
leakage bug after training runs are already underway.
