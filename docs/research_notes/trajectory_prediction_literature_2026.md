# Trajectory Prediction Literature Review (2026-08-02)

**Status:** First pass, done in direct response to two concrete problems found
this session (position accuracy 30x worse than a trivial baseline; every
Gaussian-headed feature underconfident) — prioritized by what can ground or
improve `st_gat/model/`'s actual architecture and training recipe, not general
survey background. Not exhaustive. Follows the same convention as
`fault_literature_review.md`: anchors first, then what's actionable now vs.
later.

**Why this matters now:** the ST-GAT architecture in this repo is a lightly
modified port of the T-ITS 2025 paper's own model, which was itself built
without much reference to the broader trajectory-prediction literature (see
`docs/stgat_pipeline_plan.md`'s catalog of bugs found by actually reading the
reference repo). Two bugs found and fixed this session — the position feature's
per-window-relative scale, and variance systematically wider than warranted —
both turn out to be well-known, named failure modes in the field, each with a
specific published fix. Grounding this project's remaining architecture
decisions in that literature (rather than re-discovering the same failure
modes one at a time) is the point of this doc.

---

## 1. Position/coordinate representation — directly grounds this session's fix

**Already applied:** `sequence_builder.py`'s `_scale_position_relative` now
encodes position as real-metre displacement from a fixed reference frame (the
window's own "current" timestep), not an absolute position rescaled by that
window's own, model-invisible bounding box.

This is a special case of a distinction the field has a name for —
**agent-centric vs. scene-centric (vs. query-centric) coordinate
normalization**:

- **Scene-centric**: one shared coordinate frame for the whole scene (e.g.
  centered on the ego vehicle). Cheaper, but "this shared normalization method
  can diminish the accuracy of predictions for non-central agents, as the
  distribution of shared scene information is uneven for non-centric agents"
  — the general version of the bug found here (a shared/arbitrary frame that
  isn't given to the model loses information the model needs).
- **Agent-centric**: normalize relative to *each* focal agent's own current
  state (position + heading). "Enhances training stability and demonstrates
  excellent prediction accuracy" — this is the family our fix now belongs to
  (relative-to-current-frame, fixed scale). The known drawback is
  recomputation cost per agent, irrelevant here (single-ego, not multi-agent).
- **Query-centric** (newer, e.g. Zhou et al., "Query-Centric Trajectory
  Prediction," CVPR 2023): encodes scene elements in local per-element
  frames and models interactions via relative position, avoiding
  agent-centric's per-agent recompute cost. Worth knowing about for the
  object-set encoder (already per-timestep, per-object — see §4) if it ever
  needs to scale beyond a handful of tracked objects.
- Zhang et al., **"Narrowing the Coordinate-frame Gap in Behavior Prediction
  Models"** (arXiv:2206.03970) — studies exactly this scene-centric vs.
  agent-centric accuracy/cost tradeoff and proposes distillation to get
  agent-centric accuracy at scene-centric cost. Not needed here (single ego,
  no cost problem) but useful framing for the tradeoff itself.

**Not yet applied, worth considering:** none of these papers normalize
*heading* the way we haven't — our fix removes the bbox-scale ambiguity but
still expresses displacement in the fixed map frame (x, y), not rotated into
the ego's own heading frame (forward/lateral). Agent-centric encodings
typically also rotate into the agent's heading, so "forward" is always the
same axis regardless of the vehicle's absolute orientation on the map — this
would let the model learn one canonical "how do I move forward" pattern
instead of one per heading. Given velocity is already in body frame
(longitudinal/lateral) while position is map-frame, there's an existing
frame mismatch between the two that a heading-rotated position would resolve.
**Flagged as a candidate follow-up, not implemented this session** — it's a
second, smaller version of the same class of fix, worth doing once the
current fix's actual effect is measured rather than stacking two changes
before evaluating either.

---

## 2. Heteroscedastic regression / variance calibration — directly grounds this session's other fix

**Already applied:** `loss.py` now uses **beta-NLL** (Seitzer, Tavakoli, Antic
& Martius, **"On the Pitfalls of Heteroscedastic Uncertainty Estimation with
Probabilistic Neural Networks,"** ICLR 2022, arXiv:2203.09168,
https://github.com/martius-lab/beta-nll). Their finding: "minimizing the
log-likelihood objective parameterized by mean and variance can lead to
compromised mean fits and overconfident variance estimates due to gradient
dependence on predictive variance" — i.e. plain NLL's gradient is implicitly
scaled by 1/variance, so a model facing a hard example can cut its loss by
inflating variance instead of improving the mean prediction. This is the
literal mechanism behind this session's finding (position error 30x a trivial
baseline, every feature underconfident). Their fix: reweight each sample's NLL
by its own variance (stop-gradient) raised to a tunable β; β=0 is plain NLL,
β=1 approaches plain-MSE-like mean gradients.

**Alternatives noted, not applied (candidates if beta-NLL alone isn't
enough):**
- Stirn, Wessels, Schertzer, Pereira, Sanjana, Fusi, **"Faithful
  Heteroscedastic Regression with Neural Networks"** (AISTATS 2023,
  https://proceedings.mlr.press/v206/stirn23a/stirn23a.pdf) — proposes
  separating mean and variance into two networks (or stop-gradient the mean
  path into the variance head) so the variance head can't distort the mean
  fit at all, a more aggressive decoupling than beta-NLL's reweighting.
- Immer et al., **"Effective Bayesian Heteroscedastic Regression with Deep
  Neural Networks"** (NeurIPS 2023) — uses the natural (canonical)
  parametrization of the Gaussian rather than mean/variance directly, reported
  more stable than direct mean/variance parametrization in some regimes.
- The general pathologies these all target — "optimization difficulties,
  representation collapse, and variance overfitting" — are the same family
  `loss.py`'s existing `VAR_REG_WEIGHT` term (a hand-rolled 1/variance
  penalty against *collapse*) was trying to guard against from the opposite
  direction. Worth checking, once beta-NLL's effect is measured, whether
  `VAR_REG_WEIGHT` is still pulling in a useful direction or now
  double-counting against beta-NLL's own rebalancing — flagged for the
  architecture ablation study (`TODO.md` P1.6), not decided here.

---

## 3. Conformal prediction for calibrated fault/anomaly detection — directly relevant to Stage 4 (the actual novel contribution)

This dissertation's calibration stage (`docs/theoretical_framework.md` §8,
`docs/stgat_pipeline_plan.md` Stage 4, not yet built) is exactly the target of
active 2024-2025 work combining conformal prediction with trajectory/residual
monitoring:

- **CUQDS — "Conformal Uncertainty Quantification Under Distribution Shift for
  Trajectory Prediction"** (2025) — a Gaussian-process module plus a conformal
  control module that recalibrates an *existing* trajectory model's
  uncertainty online, under distribution shift. This maps almost exactly onto
  this project's own setup: calibrate on nominal data, then the fault trials
  are themselves a distribution shift the calibration needs to remain honest
  under (or explicitly flag as no-longer-covered) — directly relevant framing
  for Stage 4, not just a generic conformal-prediction citation.
- General 2024-2025 conformal-anomaly-detection framing (multiple sources,
  cyber-physical-systems and AV safety monitoring venues): conformal
  prediction turns a raw residual/nonconformity score into a calibrated
  statement ("beyond the Nth percentile of healthy behavior") with a coverage
  guarantee, and can reduce false alarms in fault detection specifically by
  giving a principled threshold instead of a hand-picked one — this is the
  literature-level version of exactly what `docs/theoretical_framework.md`
  already argues for the calibration piece; worth citing as external
  validation that the approach is active, not idiosyncratic to this project.
- Also worth a closer read when Stage 4 actually starts: "Multi-Modal
  Conformal Prediction Regions by Optimizing Convex Shape Templates" (2024)
  and "AdaptNC: Adaptive Nonconformity Scores for Uncertainty-Aware
  Autonomous Systems in Dynamic Environments" (2026) — both about *shaping*
  the nonconformity score/region rather than using a generic one, relevant
  once this project has to decide what its own nonconformity score looks like
  (CUSUM? raw residual? something distribution-aware given the "embrace
  distributions" framing in `docs/theoretical_framework.md` §3?).

## 4. Graph/attention architecture landscape — context for the object-set encoder and future work

Current SOTA trajectory-prediction architectures split roughly into two
families, both relevant to how `st_gat/model/model.py` is built:

- **Explicit map-graph methods** — VectorNet (subgraphs over agent
  trajectories + map polylines, cross-attention between them), LaneGCN (a
  graph of lane segments with predecessor/successor/left/right edges,
  propagated before fusing with agent features). This is the same family
  `GraphEncoder`'s GCN-over-lanelet-nodes belongs to.
- **Transformer/attention-heavy methods** — HiVT, Wayformer, MTR, all using
  multi-head attention over both spatial and temporal dimensions;
  leaderboard minFDE order on Argoverse 1 (lower is better): QCNet 1.69,
  Wayformer 1.74, HiVT 1.84, LaneGCN 2.06. `model.py`'s own
  `attn_block` (a single `TransformerEncoderLayer` over the temporal axis)
  is a small-scale instance of this family already.
- This project's `ObjectSetEncoder` (added this session, masked mean-pool
  over per-object embeddings) is a minimal DeepSets-style version of the same
  idea VectorNet/LaneGCN apply to map polylines — treat entities as a set,
  let attention/pooling decide relevance instead of a hand-picked heuristic.
  If tracked-object counts ever grow large enough that mean-pooling loses too
  much (unlikely at AWSIM's typical object counts, but worth having in mind
  for the ablation study), an actual attention-pooling layer (weighted, not
  uniform mean) is the natural next step, following HiVT/Wayformer's general
  pattern rather than inventing a new mechanism.
- General survey anchor for citing the field's current shape:
  Abdel Madjid et al., **"Trajectory Prediction for Autonomous Driving:
  Progress, Limitations, and Future Directions"** (arXiv:2503.03262, 2025) —
  broad taxonomy of input/output modalities and prediction paradigms, useful
  as a single citation for "the field looks like this" framing in the
  dissertation rather than citing each architecture paper individually.

## 5. Limited training data — relevant given the overfitting signature found this session

The current best-val checkpoint peaks around epoch 20-23 then overfits
(val loss worsens while train loss keeps improving) with only 32 training
trials. Two literature-level angles, neither implemented this session:

- Azevedo et al. (2022, cited via 2024-2025 surveys of the same problem) —
  augment/diversify limited trajectory data using the HD map itself: generate
  synthetic-but-plausible trajectories along real lane graph structure with
  synthetic speeds. Directly applicable here (`Map/nishishinjuku_autoware_map`
  + this project's own lanelet2 tooling already exists) as a cheaper
  alternative to collecting many more real AWSIM trials, IF the overfitting
  turns out to be a data-volume problem rather than something the
  position/beta-NLL fixes already resolve — worth checking after the current
  fixes are evaluated, not before.
- Emerging (2025) LLM/foundation-model-based synthetic trajectory generation
  (e.g. "Trajectory-LLM," ICLR 2025) — much heavier machinery, not a fit for
  this project's scale; noted for completeness, not recommended.

---

## Next steps (not literature, just the obvious follow-through)

1. Finish training with the position fix + beta-NLL loss together, re-run the
   same accuracy/calibration diagnostics used to find these bugs, and report
   the actual before/after numbers rather than assuming the fixes worked.
2. If position accuracy or calibration is still off after that, the
   heading-rotated position (§1) and the alternative heteroscedastic losses
   (§2) are the next things to try, in that order — both are small, contained
   changes, not full redesigns.
3. Revisit §5 (data augmentation) only if the above two don't close the gap
   and the residual problem looks like genuine data scarcity, not an
   architecture/loss issue.
