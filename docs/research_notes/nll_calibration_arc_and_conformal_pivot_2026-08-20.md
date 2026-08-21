# The single-network NLL calibration arc, and the pivot to conformal calibration (2026-08-20)

**Read this first if you're about to resume work on calibrated uncertainty.**
This note closes out a multi-week arc (2026-08-06 through 2026-08-20) of
trying to get the ST-GAT model's own Student-t heads (mean+scale+dof per
feature, trained via NLL) correctly calibrated and horizon-widening. That
arc is **paused, not abandoned** — everything built is real, working code,
documented below so it can be picked back up with full context rather than
re-diagnosed from scratch. The **current primary path** to the calibration
deliverable is different: a plain point predictor + post-hoc split conformal
calibration (§4). See `TODO.md`'s status block and `CLAUDE.md` for how this
changes day-to-day priorities.

---

## 1. The original problem

`docs/research_notes/trust_and_signal_behavior_2026-08-06.md` found the
model's predicted uncertainty didn't widen with prediction horizon for 5 of
6 features (only `position` did), despite actual error growing 2-4x over
the 3s window for every feature. Three single-network fix attempts followed
(Student-t heads, two-phase training, `VAR_REG_WEIGHT`) — see
`calibration_training_literature_2026-08-07.md` and `CLAUDE.md`'s prior
"Direction reframe" section for that history. None fully worked; a deep
ensemble was designed and built (`st_gat/model/ensemble.py`, commit
`3bf698d`) but paused before training (see the 2026-08-19 pause) because it
was never verified to target the *original* widening problem specifically.

## 2. Root causes found, 2026-08-19/20 (the productive part of this arc)

Two real, confirmed, data-grounded bugs were found and fixed — both still
live in the code, both real contributions regardless of the pivot below:

**2.1 Shared horizon-step embedding, dominated by position's gradient.**
`_StepConditionedHead`'s per-step embedding table (`model.py`) used to be
ONE `nn.Embedding` shared across all 7 output heads. A gradient probe
against the trained checkpoint (backprop each head's raw NLL separately,
compare gradient norms/directions on the shared table) found position owned
40-44% of the gradient magnitude, with **negative cosine similarity
(-0.32 to -0.68) against 4 of the other 5 heads** — a genuine resource
conflict, not just an imbalance. **Fixed**: each head now gets its own
embedding table (`STGAT.horizon_embed` is now an `nn.ModuleDict`, ~2026-08-19).
Confirmed real, if partial, improvement — `velocity`/`traffic_light_color`
started showing net horizon-widening for the first time.

**2.2 Scale (not just dof) drifts below the true validation-residual
spread over training — and the reason `VAR_REG_WEIGHT` can't see it.**
A second regularizer (`DOF_REG_WEIGHT`, added 2026-08-20) confirmed and
fixed a genuine dof-collapse-toward-`DOF_FLOOR` pathology for
`position`/`velocity`/`steering`/`acceleration` (deliberately NOT applied to
`traffic_light_color`/`traffic_light_confidence` — see §3). But fixing dof
did **not** stop validation `total_nll` from degrading monotonically through
training. Diagnostic logging added the same day (`{key}_scale_mean` vs.
`{key}_empirical_scale_mean`, `loss.py`) found the real mechanism: predicted
`scale` for `velocity`/`steering` crosses from overdispersed (ratio ~1.2-1.5x
the batch's own empirical residual) to **underdispersed** (~0.7-0.9x) by
epoch 20; `acceleration` starts underdispersed and gets worse (0.63x→0.41x).
**Why `VAR_REG_WEIGHT` never caught this**: it computes its "don't shrink
below this" target from the empirical residual of whichever batch it's
looking at — but it only ever contributes gradient during **training**
batches (validation batches run under `torch.no_grad()`). If training
residuals shrink faster than validation residuals as the model overfits (an
ordinary generalization gap), the regularizer stays fully satisfied relative
to training data while scale quietly drifts below what validation data
actually needs. This is a genuinely different, more specific diagnosis than
"variance collapse" in general — it's a **wrong-reference-point** bug, not
an under-strength one (confirmed: 5x-ing `VAR_REG_WEIGHT`, 0.1→0.5, changed
almost nothing — see the attempt log). **Not yet fixed** — flagged as a
concrete, well-understood target if this thread is resumed (§5).

## 3. Two heads were never going to be fixed by any of the above

`calibration_training_literature_2026-08-07.md` §1 found
`traffic_light_color` (~4 discrete values, 80% of residuals in one
histogram bin) and, to a lesser extent, `traffic_light_confidence`
(bimodal-continuous) are **not continuous unimodal data** — no Student-t,
however heavy-tailed, can represent a point-mass-plus-jump shape correctly.
This is a representation mismatch, not a training-dynamics problem, and was
never actually fixed by any regularizer or training-schedule change tried
(both heads stayed the worst-performing throughout the whole arc, and got
measurably *worse* under the original Gaussian→Student-t redesign).

## 4. The pivot (2026-08-20): conformal calibration on a plain point predictor

An independent architecture review (fresh agent, asked to critique the
whole joint multi-task approach against the goal rather than extend the
incremental-fix list — see the session's own record for the full report)
made the case directly: the recurring failure (§2.2, and dof-collapse
before it) is a general instance of a well-documented pathology
(Wong-Toi et al., UAI 2023) in jointly training a network to predict a
value AND self-report its own confidence in one objective — not a bug
fixable by one more regularizer. It proposed **conformal calibration**
wrapped around a plain point predictor as a structurally different path
that sidesteps the entire pathology: no jointly-optimized variance head, no
distribution-family assumption (directly solves §3's problem too, for
free), and horizon-widening comes from the empirical residual distribution
by construction rather than requiring the network to learn to represent it.

**Implemented, reviewed, and corrected same day**:
`experiments/scripts/conformal_horizon_calibration.py`. The point predictor
is `st_gat/train.py --warmup-only` — Phase 1's mean-only training (already
existed, was always a good point predictor throughout every attempt this
arc — point accuracy never regressed even while NLL calibration fought
hard), now run to real convergence via early stopping instead of the
previously-arbitrary fixed 25 epochs (`Trainer.train()`, 2026-08-20 — no
more Phase 2 waiting on it, so no reason to cap it early).

Went through two iterations before landing on the current, trustworthy
version — both real, worth knowing about if this is ever touched again:

1. **First version** (standard split conformal, one fixed 50/50 trial
   split): looked like a clean win (89-91% coverage at every horizon step,
   every feature) but a fresh-agent review of the implementation itself
   caught a real leakage bug — the fit/test split partitioned individual
   overlapping windows randomly rather than by trial. `CAL_DIR` is only 7
   trials, and `sequence_builder.py` builds stride-1 windows (up to 59/60
   frames of overlap between neighbors), so most "test" windows were
   near-duplicates of some "fit" window from the same trial. Re-running
   with a proper trial-level split (mirroring the train/cal split's own
   goal-level discipline one level up, `run_pipeline.py`'s
   `_train_cal_split()`) exposed real instability the leaky number had
   hidden: with only 3 fit-trials/4 test-trials, several features drifted
   well off target (`velocity_lateral` as low as 79.5% coverage vs. 90%
   target) — a genuine data-scarcity problem (all 39 raw `nom_v11` trials
   are already allocated, 32 to `TRAIN_DIR` and 7 to `CAL_DIR` — no free
   pool to move into calibration without retraining on less data or
   collecting more).
2. **Current version**: leave-one-trial-out cross-conformal — fit each of
   the 7 folds' quantile on the other 6 trials, check coverage on the held-
   out trial, pool every fold's out-of-fold coverage indicators into one
   number per horizon step (standard cross-conformal/CV+ aggregate). Uses
   the full dataset's worth of test evidence while every check point stays
   genuinely held out. Same review also caught that pooling `velocity`'s
   x/y into one L2 magnitude was a de facto longitudinal-only bound
   (`VELOCITY_X_RANGE`/`VELOCITY_Y_RANGE` differ 30x) — split into
   `velocity_longitudinal`/`velocity_lateral`, each calibrated separately.

**Final result**: pooled coverage 89.7-90.2% (target 90%) at **every**
horizon step for **all 7 series** (position, velocity split into 2 axes,
steering, acceleration, both traffic-light heads) — tighter and far more
trustworthy than the original leaky number. `conformal_vs_actual.png` shows
correct widening (tracking non-monotonic dip-then-rise shapes distinctly
per axis for velocity) for every series — see
`experiments/analysis/conformal_horizon_calibration/`. **Real, honestly-
reported caveat**: per-fold coverage (holding out one trial at a time)
swings much more widely than the pooled number suggests — e.g. acceleration
ranges 77-95% across the 7 individual folds at t=0, steering 80-94%,
position 81-96% — because 7 trials is a small sample at the level that
actually matters for exchangeability (per-trial, not per-window). The
*aggregate* calibration claim is well-supported; a claim about any single
new drive's realized coverage would not be, without more nominal trials.
See `conformal_report.json`'s `per_fold_coverage_by_step` for the full
breakdown, not just the pooled headline number.

**Known, deliberate trade-off**: standard split conformal gives one
interval width per horizon step, constant across samples at that step — not
per-sample-adaptive the way a correctly-trained heteroscedastic head would
be. That adaptivity was the real motivation for the Student-t approach and
is a legitimate thing to want back eventually; **Conformalized Quantile
Regression** (conformalize a quantile-regression network instead of a plain
point predictor) is the standard way to get it without reintroducing this
arc's pathology — not implemented, flagged as real future work, not a
correctness gap in what's shipped now.

**Traffic-light categoricity — confirmed non-issue.** The same review
checked whether `traffic_light_color`'s near-categorical structure (~4
discrete values, `calibration_training_literature_2026-08-07.md` §1) needs
a classification-style conformal treatment (e.g. Adaptive Prediction Sets)
instead of a symmetric regression interval. Verdict: no — measured sharpness
for both TL heads is not worse than the continuous features, and a
classification treatment would also cut against the 2026-08-06 reframe's
own move away from discrete-state framing. Not built, deliberately.

## 5. If this arc is ever resumed

Everything in §2 is real, working code, not reverted:
- Per-head horizon embeddings (`model.py`) — keep regardless; this is a
  genuine architecture fix independent of the conformal pivot.
- `DOF_REG_WEIGHT`/`DOF_REG_TARGET`/`DOF_REG_KEYS` (`loss.py`) — confirmed
  effective at what it targets (dof-collapse for the 4 continuous heads).
- `{key}_scale_mean`/`{key}_empirical_scale_mean`/`{key}_implied_std_mean`
  diagnostic logging (`loss.py`) — kept, cheap, no gradient, useful for any
  future NLL-training work.
- `--freeze-trunk-phase2` — real, partial stabilizing effect (bounds the
  NLL degradation instead of letting it diverge unboundedly) but doesn't
  fully fix it alone, and costs point accuracy (the trunk can't adapt).
  Independent review's read: may be worth making permanent rather than a
  toggle, since it plausibly reduces the same shared-resource-contention
  space as §2.1's fix, just at whole-trunk scale. Untested combined with
  the §2.2 fix below.
- **The concrete next fix for §2.2, not yet tried**: widen
  `VAR_REG_WEIGHT`'s target to account for the expected train/val
  generalization gap (a margin, or computed from a held-out mini-batch
  rather than the training batch itself) — this is a specific, well-
  evidenced target, not a guess.
- Full attempt-by-attempt table (Kendall weighting, `VAR_REG_WEIGHT` at two
  strengths, trunk-freeze, dof-reg, scale-tracking): see the project's own
  session memory system (`project_calibration_attempt_log_2026-08-20`) if
  working with Claude Code, or ask Kalpit for the equivalent notes — not
  duplicated here to avoid two copies drifting apart.

**Decision criterion for resuming**: only worth it if conformal calibration
(§4) turns out to be insufficient for some part of the dissertation's claim
— e.g., if per-sample-adaptive sharpness turns out to matter more than
expected, or if conformal's exchangeability assumption breaks down badly
enough under actual fault/distribution-shift conditions (§6 below is the
robustness-under-shift question this note flagged as still open; it's
answered now, and the answer isn't fully reassuring).

## 6. Epistemic-disagreement check (2026-08-20) — real, but mixed

The paused deep-ensemble design's own recommended next step (single
Gaussian member, check aleatoric widening — see
`project_ensemble_reconsideration_2026-08-19`) would have re-derived
already-known information (self-reported scale is flat regardless of
Gaussian vs. Student-t). The actually-untested mechanism, per the
independent architecture review's point 5, is EPISTEMIC uncertainty —
disagreement between independently-trained members' point predictions —
which is what ensembling is supposed to add on top of whatever a single
member's aleatoric scale does or doesn't do, and is the more direct probe
of whether the model generalizes differently on out-of-distribution
(fault) data, i.e. calibration robustness under the shift conformal
calibration doesn't itself address.

**Method**: 2 independently-trained (different seed/data order, no
NLL/scale/dof anywhere) point predictors (`st_gat/train.py --warmup-only`),
cross-member std of their mean predictions per feature per horizon step,
compared against actual RMSE the same way as every other horizon-widening
plot in this note. `experiments/scripts/epistemic_disagreement_check.py`,
`experiments/analysis/epistemic_disagreement/`.

**Result: real for 2 of 7 series, flat for the other 5.** `position`
epistemic std grows 6.1x from t=0 to t=3s (actual RMSE grows 9.75x — real,
same-direction growth, just under-scaled in absolute terms).
`acceleration` grows 1.52x (actual 2.33x) — partial signal. But
`velocity_longitudinal` (epistemic 1.13x vs. actual 2.2x),
`velocity_lateral`, `steering`, `traffic_light_color`, and
`traffic_light_confidence` all show essentially FLAT cross-member
disagreement despite real (if more modest) actual error growth for most of
them. See `epistemic_vs_actual.png`.

**This is a genuinely interesting echo of the original problem, through a
completely different mechanism.** The single-network NLL arc (§1-§2) found
only `position` reliably widened; 2-member epistemic disagreement, with
zero shared machinery (different training objective, different models,
different metric entirely), finds essentially the same pattern —
`position` (and partially `acceleration`) is the one place where growing
uncertainty shows up easily, everything else doesn't. This is consistent
with §1's later finding (this session, gradient/marginal-distribution
analysis) that `position`'s target has a genuine, unconditional statistical
advantage — its marginal variance grows 29x over the horizon from pure
kinematic integration, something no other feature's target does — so ANY
reasonable uncertainty-estimation mechanism (self-reported NLL scale, or
cross-model disagreement) has an easy time picking it up for `position`
specifically, and a genuinely harder time for features whose difficulty
growth is a subtler, conditional property of the specific scenario.

**Caveat**: only M=2 members — a small sample for a "spread" statistic
(though averaged over ~11,684 windows, the reported MEAN epistemic std
should be reasonably stable even if individual-sample estimates are noisy).
The original ensemble design's M=5 would reduce that noise, but is unlikely
to change the qualitative pattern found here (which features show real
cross-model disagreement) — more members means a better-estimated
magnitude, not a different mechanism.

**What this means for the dissertation claim**: conformal calibration (§4)
is validated and solid for held-out *nominal* data. This result is a real,
if incomplete, signal that a single point predictor's behavior may not be
uniformly informative about *out-of-distribution* (fault) uncertainty
either — for 5 of 7 series, two independently-trained models mostly agree
regardless of horizon, which doesn't by itself prove they'd also agree
under a genuine fault, but doesn't provide reassuring evidence they
wouldn't overconfidently agree either. Not yet a blocking problem — no
fault-condition data has been run through either the conformal or the
epistemic pipeline yet, only nominal — but a concrete, well-scoped next
question if the calibration-under-shift claim needs stronger support than
"conformal works on nominal data" alone.
