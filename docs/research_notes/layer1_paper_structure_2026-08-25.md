# Layer 1 paper: structure, comparisons, and narrative arc (2026-08-25)

Scoped per Kalpit's explicit direction (2026-08-25): split the dissertation
into two papers rather than one. This note maps out Paper 1 — the
calibration/UQ contribution, self-contained, not depending on Layer 2 —
so that once the TL severity sweep and additional nominal data land, the
paper's shape is already decided and slotting results in is mechanical,
not a re-design.

Independent grounding for the split and for Paper 1's standalone
sufficiency: see project memory's 2026-08-25 novelty-verdict entry (a
fresh, non-session-colored agent's independent assessment) — the verdict
was that Paper 1's content, once the sweep and more data land, clears the
bar for a real contribution on its own, closest prior work being Reuter
et al. (ITSC 2026, arXiv:2605.19655), which differs on every axis that
matters here.

## Working title

*"Pooled Coverage Hides Real Failures: Scenario-Grounded Conformal
Calibration for Autonomous-Vehicle Trajectory Prediction, Validated
Against Injected Faults"*

(Working. The independent agent's proposed one-sentence framing is a good
candidate for the abstract's closing line — see the 2026-08-25 memory
entry.)

## Narrative arc

The paper tells one continuous story, each section motivated by a failure
found in the previous one — not a list of independent techniques:

1. Naive uncertainty (a jointly-trained distributional head) looks
   reasonable but doesn't actually calibrate → abandon it for something
   with a real guarantee.
2. A real guarantee (split-conformal) works *on average* — but "on
   average" turns out to hide a real, load-bearing failure once you look
   at WHERE the errors happen, not just how many.
3. That hidden failure is fixable, but only if you fix it at the right
   layer (training, not just calibration).
4. Once the model itself is better, the calibration question shifts: how
   should the calibrated interval vary by scenario? Two answers (discrete
   grouping vs. continuous embedding similarity) — neither is free, and
   the trade-offs are real and worth stating precisely, not oversold.
5. Does any of this matter for an actual fault? Yes — checked against
   real injected sensor/perception faults in a live simulator, not
   synthetic held-out perturbation — and the feature that carries the
   clearest evidence isn't the one you'd guess.
6. Generalize the single-fault-case result into a real dose-response
   claim (the severity sweep) — a stronger, more defensible closing
   result than one anecdote per fault type.

## Section-by-section, with the concrete comparison in each

### 1. Introduction / motivation
No comparison — framing only. State the actual claim precisely and
narrowly (per the independent agent's phrasing, adapted): pooled
conformal coverage for AV trajectory prediction can look valid while
hiding real per-scenario failure; those failures are both findable
(systematic, geometry-grounded auditing) and fixable (training-level,
not just calibration-level); and the resulting calibrated signal
distinguishes real injected faults, with the diagnostic feature depending
on fault type in a way that isn't obvious in advance.

### 2. Related work
Position against: general conformal prediction for trajectory
forecasting (CUQDS, arXiv:2406.12100, and similar — pooled/online
calibration, no scenario-conditional treatment); group/local conditional
conformal prediction generally (Mondrian conformal regressors, Boström &
Johansson PMLR 2020; Gibbs & Cherian, arXiv:2305.12616; kNN/local
conformal methods); and — most important — **Reuter et al., ITSC 2026
(arXiv:2605.19655)**, the closest prior work, which found the same
pooled-vs-per-group coverage gap on a real AV but with a single binary
curvature grouping, calibration-only fix, scalar output, no fault
injection, and no discrete-vs-continuous comparison. State the
differences explicitly, don't bury them.

### 3. Method — the point predictor + conformal calibration
**Comparison C1 — distributional head vs. conformal-calibrated point
predictor.** Already run: `experiments/scripts/compare_layer1_approaches.py`.
Point accuracy is mixed (the abandoned Student-t/NLL approach is actually
*better* on 4/6 features) — say so plainly, don't hide it. The decisive
difference is calibration quality: coverage-curve max gap 0.066–0.376 for
the distributional head (KS test rejects calibration for every feature)
vs. 0.004–0.011 for conformal. This is the paper's first "the obvious
thing doesn't work, here's why" beat.

### 4. Finding: pooled coverage is deceptive
**Comparison C2 — pooled vs. per-scenario-audited coverage.** Already
run: `experiments/scripts/audit_minority_scenarios.py`, grounded in the
same real zone geometry `fault_injector.py` gates faults on (not an
invented taxonomy). Headline number: pooled steering coverage reads ~90%,
but splits into 94.2% on calm driving and 59.0%/75.6% (pre/near-
intersection) depending on category — a real, hidden failure a single
aggregate number cannot surface. This is the paper's central "why does
this matter" result — motivates everything after it.

### 5. Fix: a training-level intervention, not just recalibration
**Comparison C3 — pre-fix (v1) vs. post-fix (v2) model.** Already run:
zone-weighted retraining (`st_gat/train.py --zone-weighted-sampling`,
data-driven inverse-frequency reweighting, no hand-tuned constants) closes
the gap: tl_zones steering coverage 75.6% → 87.3%, turn-window position
residual on hard cases down 47%. Frame this explicitly as a methodological
point, not just an engineering fix: a coverage gap found via calibration
auditing can and should be closed at the model level when it reflects a
real capability gap, not papered over by widening the interval (which
would cost detection sensitivity exactly where it's needed most — tie
this to the fault-injection motivation early, don't wait until §7).

### 6. Refinement: conditional calibration, two approaches compared honestly
**Comparison C4 — vanilla (pooled) vs. Mondrian (discrete group-
conditional) vs. embedding (continuous kNN-in-h_last) conditional
conformal calibration.** Already run:
`conformal_mondrian_calibration.py` / `conformal_embedding_calibration.py`
/ `conformal_scene_conditioning.py`. Report the SIZE-WEIGHTED net effect,
not just per-group width (the honest finding: Mondrian is NOT uniformly
tighter — steering and TL-confidence are net WIDER under Mondrian once
weighted by group prevalence, a real statistical cost of smaller
per-group fit sets, not a free efficiency win). Report the embedding
approach's distinguishing property: it improves automatically as the
underlying model improves (demonstrated directly — v1→v2 retrain improved
both the vanilla baseline AND the scene-conditioned result with zero
recalibration-scheme changes), and can tighten a feature where global
calibration was already fine, not just widen where needed — something a
fixed discrete partition structurally cannot do. Also report its honest
cost: noisier per-step quantiles from smaller effective sample sizes
(visible directly in the trust-example plots — a sudden, large widening
partway through the horizon for an otherwise calm window). This section
is the most methodologically rich in the paper if reviewers care about
conformal prediction specifically — don't undersell it.

### 7. Validation against real faults
**Comparison C5 — position-only vs. multi-feature residual response to
injected faults.** Already run: `experiments/scripts/
inspect_fault_predictions.py`, all 8 fault campaigns, all 7 series.
Headline finding: position is NOT the most diagnostic feature for either
fault family — acceleration shows a 13.66x active/clean exceed-rate ratio
for IMU faults (vs. position's 1.66x); velocity_lateral beats the
directly-spoofed TL-color/confidence features for TL faults. This is very
likely a genuinely new empirical result (the independent novelty check
could not locate prior work asking this specific question) — frame it as
such.

**Comparison C6 — the severity sweep (pending this session's analysis;
see the accompanying `tl_severity_sweep` results once run).** Generalizes
C5's single-fault-case anecdote into a real dose-response curve: mean
residual / exceed-rate vs. fault severity (reconstructed confidence_scale
from `tl_confidence_ramp`), matched against a same-goal nominal reference
to control the intersection-difficulty confound directly, rather than
comparing against a generic pooled nominal baseline. This is the paper's
closing empirical result — a controlled, continuous, confound-aware
validation, not a single-condition anecdote. (Status as of this note: a
first pass may already be extractable from data already on disk — see
`tl_severity_sweep_analysis.py` — pending whether the clean/held-out
subset alone gives a strong enough result, or whether new matched trials
at goal_007/012-equivalent held-out conditions are needed to strengthen
it.)

### 8. Discussion / limitations
State plainly, per the independent agent's own flagged caveat: 7 nominal
trials is small for claims resting on finite-sample coverage guarantees
and a k=150-neighbor local method. This is not optional — say explicitly
what would and wouldn't change with more data, and scope the paper's
claims to what's actually defensible at the current data scale. Forward-
pointer to Layer 2 (one paragraph, not a subsection) — calibrated
uncertainty is a necessary input to consequence estimation, which is a
separate paper's job.

## What's already done vs. still needed, at a glance

| Comparison | Status |
|---|---|
| C1 distributional head vs. conformal | Done |
| C2 pooled vs. per-scenario audit | Done |
| C3 pre-fix vs. post-fix (v1 vs v2) | Done |
| C4 vanilla vs. Mondrian vs. embedding | Done, needs the size-weighted table written up cleanly across all 7 features (currently partial coverage across scripts) |
| C5 position-only vs. multi-feature fault diagnosticity | Done |
| C6 severity sweep (dose-response) | In progress — first pass against existing data underway |
| More nominal calibration data | Not started — flagged as load-bearing by three independent sources now (this project's own prior notes, the reframe note, and the independent novelty-check agent) |

## Open scoping question, not yet decided

Whether NPC/traffic-density belongs in Paper 1 or Paper 2 (or neither, for
now). Current lean: it fits Layer 2's "consequence, not just detection"
framing much better than Layer 1's calibration story (NPC density
modulates true physical consequence, not calibration quality) — likely a
Paper 2 direction if pursued at all, not Paper 1. Not committed either way
yet.
