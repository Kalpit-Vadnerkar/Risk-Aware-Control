# Layer 2a→2b wiring: a real flashy result on real fault data (2026-10-04)

## Context

The 2026-09-03 advisor reframe (see `CLAUDE.md`'s decision log and
`docs/research_notes/layer2_apriori_aposteriori_split_2026-09-03.md`) split
Layer 2 into **2a (a priori)** — shape the calibrated interval using scene
context before rollout — and **2b (a posteriori)** — geometrically trim the
rolled-forward tube against real map/object constraints. 2a was already
built (it's just the existing Mondrian/embedding conditional calibration,
relabeled). 2b was "not yet started" per every status doc to date.

Investigating today found that's an overstatement: **2b's hard engineering
already exists**, built and run once on nominal-only data 2026-08-21, then
explicitly paused — `experiments/lib/margin.py` has real, map-grounded
primitives (`lane_boundary_distance()` via actual `lanelet2.geometry`,
`object_clearance()` via a conservative worst-case bound) and
`experiments/scripts/layer2_consequence_estimation.py` already does the
full bootstrap-residuals → trim-against-margin → P(violation) mechanism.
It was paused specifically because real fault data didn't exist yet
("fault-data work, deferred until Kalpit's back at the lab" — the script's
own docstring). That's no longer true: IMU faults plus the 2026-10-03/04
TL severity sweep's 18 fixed-severity trials exist and are validated (see
`docs/research_notes/tl_severity_sweep_lab_plan_2026-08-26.md`'s updated
verification section and `TODO.md` §1, now checked off).

**What this plan is not**: a from-scratch reachable-set/object-prediction
build. **What it is**: wiring 2a's conditioning into the existing bootstrap
pool, pointing the existing P(violation) mechanism at real fault trials for
the first time, and producing two concrete artifacts that make the
calibration→consequence pipeline visible and defensible — the thing
needed to show the advisor's "isn't this redundant" critique is actually
resolved, with a real result instead of an architecture diagram.

**Target artifacts** (both from ONE real fault trial — a good IMU candidate
per the decision log's own numbers: `imu_fault_s3`, which already showed a
real lead-time result, 6-9s → 12.6s, in an earlier analysis; pick the
specific goal/trial empirically in step 1 of execution, whichever shows
the clearest divergence):
1. **Envelope-cascade overlay**, one dramatic frame, on the real map: raw
   pooled conformal band → Mondrian-conditioned band → embedding-conditioned
   band → final lane/object-trimmed set. Shows the pruning doing real work,
   not just being asserted.
2. **P(violation) over time** for that trial, fault-onset marked, showing
   the signal rise before the vehicle visibly misbehaves — the lead-time
   claim, operationalized on real fault data for the first time (v1 only
   ever validated the "stays near zero on nominal" floor).

## Design

### 1. Fault-trial support (new capability, not yet in the v1 script)

`layer2_consequence_estimation.py` currently only reads pre-extracted
`.pkl` sequences via `TrajectoryDataset(cfg.CAL_DIR)` — fault campaigns are
never extracted that way (deliberately guarded, `run_pipeline.py`'s
`FAULT_DATASETS` check). Don't change the extraction pipeline. Instead
reuse the exact pattern `tl_severity_sweep_analysis.py` already established
for exactly this problem: `from inspect_fault_predictions import
process_trial`, which builds the SAME sequence dicts
(`SequenceBuilder.build()`, identical shape to the `.pkl` contents)
directly from a fault trial's raw rosbag, plus `t_rel`/`fault_windows` for
free.

Add a function alongside `_collect_position_means_and_raw()` that runs
model inference over a `process_trial()` result's `sequences` list
in-memory (adapt `tl_severity_sweep_analysis.py`'s
`residuals_for_sequences()` as the template — same per-sequence forward
pass, swap residual-distance for `position_mean`/`position_actual` to match
what this script's bootstrap step needs).

### 2. A priori (2a) conditioning for the bootstrap pool — build and compare both

Currently `pool_idx` is drawn uniformly from the WHOLE calibration pool
(`trial_id != trial_id[i]`, no scene-conditioning at all) — this is the
redundancy the advisor flagged: Layer 2 doing nothing a priori. Per
Kalpit's call (2026-10-04): build both conditioning strategies and compare
honestly (same spirit as Layer 1's own Mondrian-vs-embedding writeup, see
`CLAUDE.md`'s decision log entry "Both discrete (Mondrian) and continuous
(embedding k-NN) conditional calibration are kept"):

- **Mondrian**: reuse `assign_groups()` from
  `conformal_mondrian_calibration.py` unchanged. Compute each calibration
  window's group once (cache it), compute each analyzed fault/nominal
  window's own group the same way (`inspect_fault_predictions.py` already
  does exactly this for its own Mondrian lookup — same pattern, different
  use: here the group restricts which residuals can be DRAWN, not which
  quantile is looked up). Bootstrap pool for a window in group G =
  calibration windows in group G (fall back to the global pool below
  `MIN_FOLD_GROUP_N`, same threshold already used in
  `conformal_mondrian_calibration.py`).
- **Embedding k-NN**: reuse `model.encode_scene(past, graph)` +
  `conformal_embedding_calibration.py`'s k-NN pattern unchanged. Bootstrap
  pool for a window = its k nearest calibration windows by `h_last`
  distance (same `--k-neighbors` default, 150).
- **Unconditioned (current v1 behavior)** is kept as the explicit control/
  baseline — this is what makes the comparison honest and is also what
  artifact 1's "raw pooled band" panel shows.

Three `P(violation)` series result per window (pooled / Mondrian /
embedding) — report numbers for all three, don't silently pick a winner.

### 3. A posteriori (2b) trim — unchanged, already correct

`trajectory_margin_series()` / `margin_lib.margin()` need no changes —
they already operate on any real `(x, y)` trajectory + object list,
agnostic to which conditioning scheme produced the counterfactual. Reuse
as-is.

### 4. Artifact 1 — envelope-cascade overlay

New plotting function, built on `experiments/lib/plotting.py`'s existing
map-loading/rendering helpers (reuse, don't reimplement — this is what
`plot_fault_impact.py`/`plot_layer1_trust_examples.py` already do for
map-grounded trust plots). At the chosen frame, draw:
- the raw pooled-conformal position band (reuse
  `conformal_horizon_calibration.py`'s pooled quantile around the point
  prediction — same quantity Layer 1 already reports),
- the Mondrian-conditioned band for that window's group,
- the embedding-conditioned band for that window,
- the K bootstrap counterfactual endpoints, colored by whether
  `trajectory_margin_series()` flagged them as margin-violating — visually
  showing the trim happening, not just a clipped polygon.

### 5. Artifact 2 — P(violation) over time, fault-onset marked

Adapt the existing `n-example-trials` time-series plotting code (already
in the v1 script, ~lines 241-259) for a single featured fault trial: all
three P(violation) series (pooled/Mondrian/embedding) on one axis, a
vertical line at `fault_windows[0]['start']` (from `process_trial()`,
already computed), and if available, a second vertical line at the
trial's real MRM-trigger time (reuse `analyze_mrm_diagnostics.py`'s
existing MRM-trigger extraction) to show lead time against a real
ground-truth marker, not just fault onset.

### 6. CLI additions

New flags on the existing script (don't fork a new file — this is an
extension of the same v1 mechanism, update its docstring to "v2" and keep
a short changelog note): `--fault-campaign`, `--fault-goal`,
`--fault-trial` (selects the one featured trial for artifacts 1+2),
`--condition {pooled,mondrian,embedding,all}` (default `all` — produces
all three series). The existing nominal-only validation path (current
`main()` body, `cfg.CAL_DIR` sweep) stays unchanged and keeps running as
the regression/sanity check ("P(violation) stays near zero under
nominal") — this is additive, not a replacement.

## Files touched (when this is implemented)

- `experiments/scripts/layer2_consequence_estimation.py` — the bulk of the
  work: fault-trial support, three-way conditioning, both new plots, new
  CLI flags. Update its docstring (v1 → v2, keep the v1 history note).
- No changes needed to `experiments/lib/margin.py`,
  `conformal_mondrian_calibration.py`, `conformal_embedding_calibration.py`,
  `inspect_fault_predictions.py` — all reused as library imports exactly as
  they already are used elsewhere in this repo.

## Explicitly out of scope (separable future work, don't bundle in)

- Upgrading `object_clearance()` to use Autoware's own
  `PredictedObjects.kinematics.predicted_paths` (already recorded in every
  rosbag, already a tracked `TODO.md` §3 item) — the existing conservative
  worst-case bound is good enough for this first result; swapping it in
  is a clean, separate follow-up once this lands.
- Any new AWSIM/Autoware data collection.
- A full reliability diagram for P(violation) (`TODO.md`'s target-result-
  shape item 1) — needs real violation ground truth across MANY fault
  trials, not just one featured one; this plan produces the single
  worked example that makes that larger effort worth scoping next.

## Verification (when this is implemented)

- Regression: re-run the unchanged nominal-only path
  (`python3 experiments/scripts/layer2_consequence_estimation.py`, no new
  flags) and confirm `layer2_report.json`'s
  `mean_p_violation`/`max_p_violation` are consistent with the original
  2026-08-21 numbers — real regression check since shared code paths are
  being touched.
- New fault-trial run (`--fault-campaign imu_fault_s3 --fault-goal
  <picked goal> --fault-trial <picked trial>`): confirm P(violation) is
  near-zero before fault onset and visibly rises after it, for at least
  the pooled series (the Mondrian/embedding series may differ in
  magnitude — that's the honest comparison this plan wants).
- Sanity-check the Mondrian/embedding pools are actually DIFFERENT from
  the pooled baseline for at least one real window (print pool sizes and
  a quantile-width comparison) — catches a silent no-op conditioning bug
  before trusting the "cascade" artifact shows anything real.
- Visual check of artifact 1: confirm the raw pooled band visibly differs
  from the trimmed/conditioned bands at the chosen frame (if it doesn't,
  the frame was picked wrong — try a different one rather than force the
  figure).
