# Model redesign: literature review + architecture (2026-08-07)

Prompted by Kalpit's question two sessions ago ("what if we started from an
uncertainty-aware architecture instead of bolting uncertainty onto a
trajectory-prediction one") and the explicit instruction this session: do
the literature review properly, download high-quality papers, then design
and build a new model from scratch for this specific problem — not generic
uncertainty quantification, but what THIS dissertation's mechanism needs.

All papers below are downloaded to `docs/papers/` (gitignored, as already
established for this repo). Selection bar: NeurIPS/ICML/ICST-tier venues,
foundational or directly-on-topic, not marginal preprints.

---

## 1. What does the model actually need to do?

Not "quantify uncertainty" in the abstract — specifically, per
`docs/theoretical_framework.md`/`CLAUDE.md`'s claim: detect when the
autonomy stack's perceptual belief diverges from a map-grounded
expectation, and report that divergence with **calibrated confidence**.
Distilling the concrete requirements:

1. **The uncertainty must stay trustworthy specifically under the
   nominal→fault transition** — that transition IS a distribution shift
   (the model trains only on nominal data; a fault is, by construction,
   out-of-distribution relative to that training set). This is not an
   incidental property to hope for — it is close to the entire point.
2. **Calibration, not just point accuracy** — a confidence number that
   means what it claims, checked via coverage/PIT, not just "the mean
   prediction is close."
3. **Per-feature, interpretable divergence** — the existing per-feature
   NLL/residual trace design (not a single collapsed anomaly score) is a
   real requirement from the mechanism itself (traffic-light negative
   evidence vs. motion-feature divergence need to stay distinguishable),
   independent of whatever produces the uncertainty.
4. **Horizon-aware behavior** — predicted uncertainty should grow
   sensibly across the 3s prediction window, which the single-network
   approach never reliably achieved despite three attempts.
5. **Works with graph-structured HD map context** — the model conditions
   on a lanelet2 graph; whatever uncertainty mechanism is chosen has to
   compose with a GNN encoder, not just a flat feedforward net.
6. **Real compute constraints** — a single training run already costs
   hours on this project's hardware; whatever is chosen has to be a
   deliberate, justified compute trade, not an unconstrained ideal.

## 2. What the literature says, specifically for requirement 1

This is the load-bearing question, and there's a direct, high-quality
empirical answer, not just theoretical appeal:

**Ovadia, Fertig, Ren, Nado, Sculley, Nowozin, Dillon, Lakshminarayanan &
Snoek, "Can You Trust Your Model's Uncertainty? Evaluating Predictive
Uncertainty Under Dataset Shift," NeurIPS 2019**
(`docs/papers/2019_ovadia_can_you_trust_model_uncertainty.pdf`) — a
large-scale benchmark (Google Brain/DeepMind) of essentially every major UQ
method (MC-dropout, single-network heteroscedastic heads, temperature
scaling, variational BNNs, deep ensembles) across multiple data modalities,
specifically measuring calibration **as a function of dataset shift
severity** — exactly this project's actual operating scenario (nominal
training, fault-condition evaluation). Their headline finding: post-hoc
calibration methods and most single-network approaches degrade sharply
under shift; methods that **marginalize over multiple independently-trained
models** (i.e. deep ensembles) hold up "surprisingly" well by comparison,
across every modality tested. This is the single most directly relevant
data point available: it's not "ensembles are a nice idea," it's "this
specific failure mode (calibration collapsing exactly where we need it
most) is a *documented, benchmarked* property of the architecture we've
been using, and a *documented, benchmarked* strength of the alternative."

**Fort, Hu & Lakshminarayanan, "Deep Ensembles: A Loss Landscape
Perspective," 2019** (`docs/papers/2019_fort_deep_ensembles_loss_landscape.pdf`)
— explains the mechanism, not just the empirical result: popular
approximate-Bayesian methods (MC-dropout included) stay within a single
loss-landscape mode, while independently-initialized ensemble members
explore genuinely different modes ("the diversity-accuracy plane").
Directly relevant to this session's own debugging history: the
single-network Student-t collapse (Wong-Toi et al., already in
`docs/papers/`, cited in `loss.py`) is fundamentally a single-mode
pathology — one network's likelihood parameters being driven to overfit
training-specific noise. An ensemble doesn't have "a" likelihood surface to
game this way; disagreement across independently-trained modes is a
structurally different signal source, immune to that specific collapse.

**Grewal, Tonella & Stocco, "Predicting Safety Misbehaviours in Autonomous
Driving Systems using Uncertainty Quantification," ICST 2024**
(`docs/papers/2024_grewal_safety_misbehaviours_uncertainty_quantification.pdf`)
— the closest published analogue to this dissertation's actual use case:
UQ for anticipating safety misbehaviour in AV simulation testing. Directly
compares MC-Dropout and Deep Ensembles empirically; found deep ensembles
detected most misbehaviours with fewer false alarms, while remaining
computationally feasible for real-time use. This is a real answer to
requirement 6 (compute) from within the AV-safety literature specifically,
not extrapolated from an unrelated domain.

**Kendall & Gal, "What Uncertainties Do We Need in Bayesian Deep Learning
for Computer Vision," NeurIPS 2017**
(`docs/papers/2017_kendall_gal_what_uncertainties_bayesian_dl.pdf`) —
foundational aleatoric/epistemic decomposition (same first two authors as
the multi-task weighting paper already cited in `loss.py`). Aleatoric =
irreducible data noise (what the current single-network Gaussian/Student-t
heads represent); epistemic = model uncertainty, reducible with more/
different data, present only when the model hasn't seen something like
this before. **This project's current architecture has zero epistemic
signal** — every uncertainty number produced so far is purely aleatoric,
which is a real structural gap given the actual target (faults are, almost
by definition, epistemic events: situations the model wasn't trained on).

**Lakshminarayanan, Pritzel & Blundell, "Simple and Scalable Predictive
Uncertainty Estimation using Deep Ensembles," NeurIPS 2017**
(`docs/papers/2017_lakshminarayanan_deep_ensembles.pdf`, already in
`docs/papers/` from an earlier session) — the foundational recipe: each
ensemble member is itself a proper-scoring-rule-trained probabilistic
predictor (Gaussian NLL in their original formulation), combined via
straightforward moment-matching (law of total variance: aleatoric = mean
of each member's own predicted variance, epistemic = variance of the
members' means). Gives a clean, well-established way to report aleatoric
and epistemic uncertainty as *separate* numbers, not just a combined one.

### Alternative considered and not selected as the primary mechanism: evidential deep learning

**Amini, Schwarting, Soleimany & Rus, "Deep Evidential Regression,"
NeurIPS 2020** (`docs/papers/2019_amini_deep_evidential_regression.pdf`)
and the broader **Ulmer, Hardmeier & Frellsen, "Prior and Posterior
Networks: A Survey on Evidential Deep Learning Methods for Uncertainty
Estimation," 2021** (`docs/papers/2021_ulmer_evidential_deep_learning_survey.pdf`)
— a genuinely elegant single-network alternative: predict parameters of a
*higher-order* distribution over (mean, variance) itself (Normal-Inverse-
Gamma), with a loss term that explicitly regularizes against unsupported
confidence claims. This is architecturally closer to what this project
already built by hand today (the `VAR_REG_WEIGHT` regularizer is a hand-
rolled, cruder version of what evidential priors do natively). **Not
selected as the primary redesign** because: (a) it's still fundamentally a
single-network mechanism, so Ovadia et al.'s specific finding about
marginalized/ensemble methods generalizing better under exactly this
project's shift scenario doesn't transfer to it directly; (b) more recent
literature (cited within the survey) has found evidential regression's own
calibration claims don't always hold up as robustly as originally reported,
especially under real (not synthetic) distribution shift. Kept as a
documented, real fallback/extension — worth revisiting specifically for
`traffic_light_color`'s near-categorical structure (see
`calibration_training_literature_2026-08-07.md` §1's point-mass finding),
where evidential regression's native multimodality handling may be a better
fit than either a single Gaussian or an ensemble of unimodal Gaussians. Not
in scope for this redesign's first version.

### GNN-specific consideration

**"Uncertainty Quantification on Graph Learning: A Survey," 2024**
(`docs/papers/2024_uncertainty_quantification_graph_learning_survey.pdf`)
— confirms ensembling composes cleanly with GNN encoders (no GNN-specific
obstacle to the ensemble approach), and separately flags a cheaper
ensemble variant ("shallow ensembles" / direct propagation) as a future
compute-saving option if full independent-model ensembling proves too
expensive here — noted as a fallback, not adopted now.

---

## 3. Architecture decision

**Deep ensemble of the existing (validated) ST-GAT backbone, with Gaussian
(not Student-t) per-member heads, combined via aleatoric/epistemic
decomposition.**

Important framing: this is NOT throwing away today's backbone. The graph
encoder, temporal attention, LSTM, and per-horizon-step-conditioned decoder
(`_StepConditionedHead`) were never shown to be the problem — every
failure this session traced specifically to how a SINGLE network's
distributional likelihood parameters get optimized (joint mean+scale+dof
collapse, Kendall-weight runaway). "Building a new model from scratch"
here means restructuring *how uncertainty is produced* — from one
network's self-reported likelihood to ensemble disagreement — not
discarding validated architecture components without cause.

Concrete design:
- **M=5 independently-initialized STGAT instances** (matching
  Lakshminarayanan et al.'s and Grewal et al.'s own practice — diminishing
  returns reported past ~5 in both the original ensembles literature and
  the AV-specific study).
- **Gaussian, not Student-t, per-member heads** — deliberate reversion.
  Two reasons: (1) Lakshminarayanan's combination formula (law of total
  variance) is exact/simple for Gaussian mixtures, not as clean for a
  mixture of Student-t's; (2) a real, testable hypothesis (not yet
  confirmed): some of the leptokurtosis found in the single-network
  Gaussian model may itself have been epistemic uncertainty masquerading
  as aleatoric heavy tails — situations the single model hadn't seen
  well, showing up as occasional large residuals. If true, an ensemble
  that structurally separates epistemic (via disagreement) from aleatoric
  (via each member's own Gaussian spread) may need less aleatoric tail
  flexibility than a single network did. Falsifiable directly once trained:
  check the SAME excess-kurtosis/Anderson-Darling diagnostic (already
  built, `plot_calibration_diagrams.py`) against the ensemble's aleatoric
  component alone.
- **Per member: keep the two-phase training (mean warmup, then NLL
  fine-tune) and the `VAR_REG_WEIGHT` regularizer** — both are real,
  validated fixes to real problems (Stirn et al.'s undertrained-mean
  pathology; Wong-Toi et al.'s variance-collapse pathology), independent
  of the ensemble question, and cheap to keep.
- **Drop the Kendall-et-al. learned task weighting per member** — revert
  to fixed, hand-set task weights (the pre-2026-08-07 `DEFAULT_WEIGHTS`).
  This was the specific mechanism that ran away; ensemble diversity is
  supposed to come from independent initialization/data order, not from a
  per-member adaptive weighting scheme, and removing it eliminates the
  exact failure mode found today rather than requiring it to be perfectly
  tuned first.
- **Combination**: per-timestep, per-feature —
  `mean = (1/M) * sum(mean_i)`,
  `aleatoric_var = (1/M) * sum(var_i)`,
  `epistemic_var = (1/M) * sum((mean_i - mean)^2)`,
  `total_var = aleatoric_var + epistemic_var` (law of total variance) —
  all four reported as separate trace columns, not just the total. The
  aleatoric/epistemic split is itself a new, directly interpretable signal
  for the dissertation's mechanism: a fault that's a genuinely novel
  situation should show epistemic_var spiking; a fault that just adds
  noise to an otherwise-familiar situation should show aleatoric_var
  responding instead. That distinction has no analogue in the current
  single-network design at all.

**Compute honesty**: 5x the per-model training cost of what's already been
run today (each of today's runs took ~2-3.5 hours). This is a real,
substantial commitment (potentially 10-15+ hours of GPU time for the full
ensemble, though members train independently and could run concurrently
given the GPU has been sitting under 1.5GB/49GB used all session). Not
started without flagging this explicitly.

---

## Status

Literature review and design complete. Implementation in progress
(`st_gat/model/ensemble.py` — see that module's own docstring for
current state). Full ensemble training not yet started; will not be
started without explicit confirmation given the compute cost above.
