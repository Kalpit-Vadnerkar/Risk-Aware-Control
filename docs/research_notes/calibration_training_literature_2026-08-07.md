# Calibration-aware training: literature grounding (2026-08-07)

Follow-up to `trust_and_signal_behavior_2026-08-06.md` §3's mixed Student-t
retrain result. Kalpit raised three questions after that result; answering
them properly required both a quick empirical check and an actual
literature search (not just informal reasoning) — this note is that
search, the papers it's grounded in (downloaded to `docs/papers/`, not
just cited from memory), and the design it points to for the next retrain.

---

## 1. Do different features need different distributions? (checked empirically, yes — and it's bigger than "Gaussian vs. Student-t")

Pulled raw residuals (`actual - predicted mean`, pooled across all 39
`nom_v11` trials, `st_gat/results/h30_30/traces/`) for four features and
looked at the actual shape, not just a summary statistic:

| feature | actual value distribution | residual histogram shape |
|---|---|---|
| `traffic_light_color` | **4 discrete values only**: 0.0 (37%), 0.33 (30%), 0.67 (0.6%), 1.0 (32%) — this is categorical data, not a continuous quantity | 80% of all residuals fall in ONE histogram bin (near zero), rest scattered in a long thin tail |
| `traffic_light_confidence` | heavily bimodal at the extremes (0.0 and ~1.0), long tail of near-1.0 values in between | 72% of residuals in one bin, same point-mass-plus-tail shape |
| `steering` | 23% of samples land at exactly the scaled-center value (0.5, i.e. "no steering") | 78% of residuals in one bin — same shape, driven by nominal driving being mostly straight-line punctuated by real turns |
| `position_x` | smoothly spread across dozens of distinct values | smoothly spread histogram, two moderate humps, no single dominant bin |

**This is the real finding, and it changes the diagnosis.** `position` is
the one feature that calibrated well under both Gaussian and Student-t
because it's genuinely close to continuously distributed. `traffic_light_color`,
`traffic_light_confidence`, and `steering` are not "heavy-tailed continuous"
data — they're closer to **point-mass-plus-occasional-jump** data (a
near-deterministic "nothing changed" value the vast majority of the time,
punctuated by real discrete events). No single continuous unimodal density
— Gaussian OR Student-t — can represent that well: matching kurtosis with a
wider tail (what the Student-t redesign did) can make the *tail* behavior
more honest without fixing the fact that the *bulk* of the distribution is
a near-delta-function the model is trying to spread a smooth density over.
This is a much more specific, and more promising, lead than "wrong tail
weight" for explaining why `traffic_light_color`/`traffic_light_confidence`
got *worse* under the Student-t redesign, not better — a more flexible
tail doesn't help if the mismatch is in the bulk shape, not the tails.

**Literature grounding:**
- Bishop, "Mixture Density Networks," Aston NCRG technical report NCRG/94/004,
  1994 (`docs/papers/1994_bishop_mixture_density_networks.pdf`) — the
  foundational result that a neural network can output the *parameters of
  a mixture* (weights + per-component mean/variance) instead of a single
  distribution's parameters, letting the predicted density be genuinely
  multimodal. A 2-3 component mixture (one tight component near "no
  change," one wider component for real events) is a natural fit for
  exactly the point-mass-plus-jump shape found above.
- Kong, Bai, Lee, Chen, Allyn, Stuart, Pinsky, Mills, Gomes, "Deep Hurdle
  Networks for Zero-Inflated Multi-Target Regression," IJCAI 2020
  (`docs/papers/2020_kong_deep_hurdle_zero_inflated_regression.pdf`) — a
  more specialized, directly-on-point architecture family for exactly this
  "excess mass at one structural value" problem: a two-part model, a
  classifier/gate for "does this differ from the structural value at all"
  coupled with a continuous distribution for the magnitude when it does.
  `traffic_light_color`'s near-4-valued categorical structure in
  particular might be more honestly modeled as closer to a classification
  head than a continuous regression head at all.

**Not recommending an immediate architecture change here** — this is a
real, literature-supported hypothesis worth testing, but it's a bigger
change than the training-dynamics fix below, and touching two different
things (training criterion AND distribution family) in the same retrain
would make it hard to attribute the next result to either cause. Flagged
as the next thing to try if the training-dynamics fix (below) doesn't
close the gap for `traffic_light_color`/`traffic_light_confidence`
specifically.

---

## 2. Decoupling the LR schedule/stopping criterion from point-accuracy: literature says this is a known, named problem

Searched specifically for what's already understood about NLL-based
training dynamics in heteroscedastic regression, not just "early stopping
in general." Two papers turned out to describe close to exactly the
symptom found in §3 of the prior note:

- Stirn, Wessels, Schertzer, Pereira, Sanjana, Knowles, "Faithful
  Heteroscedastic Regression with Neural Networks," AISTATS 2023
  (`docs/papers/2023_stirn_faithful_heteroscedastic_regression.pdf`).
  Directly states the mechanism: optimizing mean and variance jointly via
  NLL gradients "can yield suboptimal mean AND uncalibrated variance
  estimates" — mean accuracy from a jointly-trained heteroscedastic model
  can be *worse* than an equivalent mean-only model, because the NLL loss
  implicitly down-weights gradient signal for samples the model has
  assigned high predicted variance to (the same beta-NLL pathology
  `loss.py` already cites Seitzer et al. 2022 for, but this paper is about
  the complementary problem: even with beta-NLL-style reweighting, JOINT
  optimization from a cold start is still fragile). **Their fix — cited
  directly, not just alluded to — is staged/alternating training: fit the
  mean first via plain squared error, THEN alternate between NLL steps for
  mean and for variance.** This is a concrete, literature-backed answer to
  what "decoupling" should actually mean in practice, not just "look at
  NLL for early stopping instead."
- Wong-Toi, Boyd, Fortuin, Mandt, "Understanding Pathologies of Deep
  Heteroskedastic Regression," UAI 2023
  (`docs/papers/2023_wongtoi_pathologies_deep_heteroskedastic_regression.pdf`).
  Describes the SAME class of model (heteroscedastic mean+variance neural
  nets) collapsing to one of two degenerate regimes: fitting the mean
  perfectly while the variance head shrinks to near-zero (overconfident),
  or the variance head absorbing all the signal while the mean collapses
  toward a constant (underfit mean, "explaining away" real error as noise
  instead of reducing it). This is a genuinely useful frame for
  `st_gat`'s current result: `position`'s mean is real and precise, its
  distributional head can widen appropriately — the well-behaved regime.
  `acceleration`/TL heads may be closer to the collapsed-variance regime
  (small scale, but wrong dof/shape) given they got WORSE, not
  better — worth checking directly next session (are the low-KS-p-value
  features actually near `DOF_FLOOR`, suggesting a degenerate rather than
  a genuinely-converged heavy-tailed fit?).
- Lakshminarayanan, Pritzel, Blundell, "Simple and Scalable Predictive
  Uncertainty Estimation using Deep Ensembles," NeurIPS 2017
  (`docs/papers/2017_lakshminarayanan_deep_ensembles.pdf`) — the
  foundational statement that a **proper scoring rule** (NLL is one) is
  what should drive training/selection for calibrated uncertainty, not
  point-accuracy alone. Cited here as the standard justification for why
  "track NLL, not just raw point error" is the right general direction
  Kalpit's question 2 is pointing at — this is established practice in the
  UQ literature, not a novel idea being improvised for this project.

**Concrete plan this points to for the next retrain, staged rather than
guessed:**
1. **Phase 1 — mean-only warmup.** Train for some number of epochs (needs
   picking, e.g. until raw point-error plateaus, roughly where the current
   run's epoch ~54 already is) using only the point-accuracy loss
   (`position_l2_raw`-style, not NLL) for the mean heads, ignoring
   scale/dof entirely (freeze or exclude those parameters from the loss).
   Directly implements Stirn et al.'s recommended fix, and reuses a
   metric (`v_raw`) `trainer.py` already computes every epoch.
2. **Phase 2 — full NLL fine-tuning.** Switch to the Student-t NLL over
   ALL parameters (mean+scale+dof), reset or extend the LR schedule so
   this phase gets real optimization budget of its own instead of
   inheriting whatever LR decay phase 1 left it in, and track validation
   NLL (or a PIT/coverage summary) as the selection criterion for THIS
   phase specifically — safe to use now because phase 1 already solved
   the "NLL prefers an undertrained mean" pathology `v_raw`-only selection
   was built to avoid.
3. **Report both phases' epoch counts and metrics separately** in the
   training log/checkpoint metadata — needed to actually verify the staged
   design worked as intended, not just assumed to.

---

## 3. Do different features competing for gradient/training budget already get accounted for? No — this is a real gap, and it's the multi-task learning literature's exact problem

Checked `st_gat/model/loss.py`'s `DEFAULT_WEIGHTS`: fixed, hand-set
constants (`position: 1.0, velocity: 0.8, steering: 0.5, acceleration: 0.5,
traffic_light_color: 0.2, traffic_light_confidence: 0.2,
traffic_light_discrepancy: 0.2`), never adjusted based on how each
feature's training is actually progressing. This is precisely the problem
the multi-task learning literature calls **loss/gradient imbalance** —
different tasks (here, different features) have different intrinsic
difficulty, noise scale, and gradient magnitude, and a fixed-weight sum
lets whichever task produces the largest/easiest-to-reduce gradients
dominate training, starving the others of real optimization pressure
regardless of how "important" the hand-picked weight says they are.
Notably, the three features that got WORSE in the Student-t retrain
(`acceleration`, both TL heads) are exactly the three with the LOWEST
fixed weights (0.5, 0.2, 0.2) — consistent with, though not proof of, this
being a contributing factor alongside the checkpoint-selection issue above.

**Literature grounding, two standard approaches, either genuinely
applicable here:**
- Kendall, Gal, Cipolla, "Multi-Task Learning Using Uncertainty to Weigh
  Losses for Scene Geometry and Semantics," CVPR 2018
  (`docs/papers/2018_kendall_multitask_uncertainty_weighting.pdf`) — each
  task gets one learned scalar (homoscedastic task uncertainty), and the
  combined loss becomes `sum_task( L_task / (2*sigma_task^2) + log(sigma_task) )`
  instead of a hand-picked fixed weight. Directly implementable as one
  small addition to `CombinedLoss` (one learned parameter per feature,
  `nn.Parameter`), no architecture change needed elsewhere. Notably, this
  is a *different* uncertainty than the per-timestep aleatoric scale/dof
  the model already predicts — this would be a second, coarser "how hard
  is this whole task" uncertainty, at the loss-weighting level, not the
  prediction level.
- Chen, Badrinarayanan, Lee, Rabinovich, "GradNorm: Gradient Normalization
  for Adaptive Loss Balancing in Deep Multitask Networks," ICML 2018
  (`docs/papers/2018_chen_gradnorm_multitask_balancing.pdf`) — directly
  monitors and rebalances each task's gradient MAGNITUDE at a shared
  layer (naturally the LSTM's final hidden state, `h_last`, in this
  architecture) rather than inferring a weight from a learned uncertainty
  parameter. More implementation overhead (needs gradient-norm bookkeeping
  per step) than Kendall et al.'s approach, but more directly targets the
  actual mechanism (gradient magnitude, not just loss scale) — a
  reasonable second thing to try if Kendall-style weighting alone doesn't
  close the gap for the under-performing heads.

**Recommendation: try Kendall et al.'s uncertainty weighting first** — it
is the lower-overhead change, replaces the currently-arbitrary
`DEFAULT_WEIGHTS` constants with something learned and literature-backed,
and combines cleanly with the staged training plan from §2 (task weights
would only need to be learned in Phase 2, where the full NLL objective is
active).

---

## Combined recommendation for the next retrain (not yet implemented)

Three independent, literature-grounded changes, all addressing different
parts of the same mixed result:
1. Staged training (mean-only warmup, then full NLL fine-tuning) — Stirn
   et al. 2023, targets the checkpoint-selection blindness directly.
2. Learned per-feature task-uncertainty weighting (Kendall et al. 2018)
   replacing the fixed `DEFAULT_WEIGHTS` — targets the gradient-imbalance
   question.
3. NOT changing the distribution family again yet — the point-mass finding
   in §1 is real and worth acting on, but deliberately sequenced after 1/2
   so a still-bad result for `traffic_light_color` specifically can be
   attributed to "wrong family" rather than confounded with a training-
   dynamics fix that might independently help.

This is a bigger set of changes than the last retrain and costs real GPU
time (~3.5h last time) to test — a design decision worth Kalpit's explicit
go-ahead before implementing, not assumed.
