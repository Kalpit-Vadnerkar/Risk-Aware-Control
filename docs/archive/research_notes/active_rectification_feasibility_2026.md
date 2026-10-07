# From probing to rectifying: feasibility of using the model to correct faults, not just detect them (2026-08-02)

**Status: exploratory, not scoped into the dissertation's critical path.** Written
because Kalpit asked for a literature-grounded feasibility read on a real scope
question — could the digital twin's own output be fed back through the fault
interceptor to *correct* a detected divergence, not just flag it — with the
explicit instruction that this is research-approach-and-feasibility only, no
code. `CLAUDE.md` currently scopes active control (RISE) out of the
dissertation's defended claim; this note is Kalpit's input to deciding whether
to reopen that scope, not a decision.

## 1. What this would concretely mean in this repo

The plumbing already exists in a form that's directly reusable. `fault_injector.py`
already sits between real perception topics and what Autoware's planner
actually subscribes to (`/perception/traffic_light_recognition/traffic_signals_faulted`,
the pattern documented in `CLAUDE.md`) — it currently *corrupts* the message on
the way through. A rectifier is the same interception point run in the other
direction: when the model's belief-divergence alarm (the conformal threshold
from `conformal_lead_time.py`) fires on a specific channel, the node stops
passing the raw (possibly faulted) value through and substitutes the model's
own map-grounded expectation for that one channel instead.

That "same architecture, opposite direction" framing is the strongest
feasibility point in favor of this — it is not a new subsystem, it's a second
mode of a subsystem that's already built, tested, and well understood in this
codebase. It is *not*, however, a small change to the model or the claim being
defended: right now the model's output only has to be right enough to produce
a calibrated alarm (bounded cost if wrong: a missed or false alarm). Under
rectification, the model's point estimate becomes something Autoware's planner
directly consumes and acts on — a materially higher bar.

## 2. Where this sits in the literature

- **Runtime assurance / Simplex architecture** (Sha, "Using Simplicity to
  Control Complexity," IEEE Software 2001; the broader runtime-assurance
  literature it spawned, e.g. NASA/AFRL work on RTA for UAS). The classic
  pattern is a certified, simple monitor that can switch control authority to a
  certified, simple recovery controller when an advanced (uncertified)
  component misbehaves. The proposal here **deviates from classical Simplex in
  the one place that matters most**: Simplex's recovery controller is
  deliberately *not* the fallible system being monitored — it's a separately
  certified fallback. Substituting the same learned model's own output as the
  correction collapses that separation. Worth being explicit about this
  difference rather than borrowing the Simplex name without the property that
  makes it trustworthy.
- **SOTIF (ISO 21448)** formalizes exactly the distinction this proposal would
  cross: hazard *identification* (what the current dissertation claim is
  about) is a substantially easier certification target than hazard
  *mitigation by a learned component*. Citing this standard is useful mainly to
  frame the feasibility risk precisely for a committee, not because compliance
  is in scope.
- **Conformal prediction feeding a control decision, not just an alarm**:
  Lindemann et al. (e.g. "Safe Planning in Dynamic Environments using Conformal
  Prediction," 2023) is the closest existing precedent for turning a
  calibrated uncertainty bound into an actual control/planning decision rather
  than a passive flag — directly relevant since Stage 4's conformal layer is
  already built and calibrated; this is the natural next citation if this
  direction is pursued.
- **Learned world models imputing corrupted/missing sensor channels**: Wayve's
  GAIA-1 and Hu et al.'s MILE are the visible examples of a learned model
  standing in for part of the perceived world in driving specifically. The
  relevant lesson from that literature is that these models are evaluated by
  how well their *imagined* rollout matches reality under distribution shift —
  which is precisely the open question here (see §4).
- **Classical FDIR (fault detection, isolation, and reconfiguration)** in
  aerospace/controls is the non-learned ancestor of this whole idea — multi-
  decade literature on using a redundant/model-based estimate to replace a
  failed sensor channel. Useful for framing this as "FDIR with a learned
  observer" rather than an unprecedented idea, which is both more honest and a
  stronger citation base than treating it as novel.

## 3. The central feasibility risk

The model would be asked to substitute its own value **exactly when its own
reliability is least established**: faults are rare and out-of-distribution
relative to the mostly-nominal training data (the same reason calibration is
hard, per `model_improvement_notes_2026.md` §1). A forecasting model that is
honestly uncertain under a fault (which is the current, defensible claim) is a
very different reliability bar than a corrective model that must be *right*
under a fault. If the substituted value is wrong, the outcome could plausibly
be worse than leaving the known-faulty raw value in place, because a wrong
correction is confidently wrong in a way the raw fault (which Autoware's own
downstream logic may have some independent handling for) might not be.

## 4. What would have to change in the model and learning strategy

- **Different objective.** The current model is trained as a sequence
  forecaster (Gaussian NLL / BCE per feature, `st_gat/model/loss.py`) — it
  predicts what *should* happen next given history. Rectification needs a
  conditional-imputation objective instead: given the map-grounded prior and
  pre-fault context, generate the value a *specific corrupted channel* should
  have reported. Related to, but not the same task as, forecasting — closer to
  masked-imputation/denoising objectives than to the existing loss.
- **Training data this repo may already have a shortcut to.** The two-arm
  design (Arm A safety-disabled / Arm B safety-enabled) plus this project's own
  practice of running the *same goal* under both nominal and fault conditions
  means paired (faulted-channel, true-nominal-value) examples may already be
  obtainable by matching a fault trial to its same-goal nominal trial — without
  collecting new data. That's the same "apples-to-apples same-goal" principle
  already used for the discriminability and calibration analyses, reapplied
  here. Worth flagging as a real synergy, not a new data-collection burden.
- **Different evaluation.** Calibration/NLL on a forecast doesn't answer "does
  substituting this value actually help." That requires closed-loop
  evaluation — inject a fault, apply the candidate correction, measure the
  resulting trip outcome vs. no correction — which is a genuinely bigger
  simulation burden than the current open-loop residual-trace analysis and
  would need new AWSIM/Autoware runs eventually, even though a first feasibility
  pass would not (see §5).
- **Real-time constraint not yet measured.** `st_gat.residuals` currently runs
  offline over complete trials. Rectification needs the forward pass plus the
  conformal check running online, inside Autoware's control loop timing
  budget. The model is small enough (128-dim hidden state, 2 layers per
  `st_gat/model/model.py`) that this is plausible, but inference latency
  against the control loop's actual rate has never been measured — a concrete,
  cheap thing to check before investing further, not yet done.

## 5. A cheap first feasibility check, given no new experiments right now

Everything above can get a first real signal **without** any new AWSIM/Autoware
runs or closed-loop integration: an offline counterfactual replay. For each
already-collected fault trial, take its matched same-goal nominal trial as the
"what should have been reported" ground truth, and ask whether the model's own
prediction at each post-alarm timestep would have been a *closer* match to that
nominal trial's true value than the corrupted value actually was. This reuses
data and infrastructure that already exist (the residual traces, the per-goal
matching already built for the discriminability/calibration analyses) and would
give a real accuracy number for "how good would the correction have been" before
committing to any of the harder engineering in §4.

## 6. Where this leaves the scope question

This is a materially different research bet than the currently scoped
calibration + lead time + severity contribution: a new training objective, a
new evaluation harness, and a harder certification argument, likely amounting
to its own sub-study rather than a quick add-on. It is also a more
mechanistically coherent extension of this dissertation's actual claim than
the original RISE idea (relaxing kinematic constraints for progress) —
rectification reuses the *same* map-grounded expectation the whole thesis is
built on, just adds a second consumer for it (substitution, not only alarm),
rather than introducing an unrelated planning-relaxation mechanism. Recommend
treating it as a stretch-goal/future-work section for now, with §5's offline
replay check as the one concrete, low-cost thing worth trying before deciding
whether to commit further.
