# Open-world safety reframe: from closed-set fault classification to consequence estimation (2026-08-20)

**Read this first if you're touching the dissertation's core claim, `TODO.md`'s
Research Direction, or `docs/theoretical_framework.md`.** This supersedes
the 2026-07-24 "belief divergence / negative evidence" framing and its
Priority 0 mechanism experiments (kept below as history, not deleted).
Prompted by Kalpit, same day as the conformal-calibration pivot
(`nll_calibration_arc_and_conformal_pivot_2026-08-20.md`) — the two are
tightly related: Layer 1 below *is* that pivot's deliverable.

---

## 1. Why the old framing is being retired

The 2026-07-24 claim ("belief divergence under a map-grounded prior,"
Priority 0's per-fault-class feature-importance recompute and HD-map
ablation) is fundamentally **closed-set fault classification**: it defines
a fixed bin set (Camera/IMU/LiDAR fault classes, inherited from the
published T-ITS paper) and asks whether the model sorts correctly into
those bins. Two structural problems, not incidental ones:
- It requires *fault data* to validate at all — bins need labeled examples.
- It cannot say anything useful about a failure mode that isn't in the bin
  set. The whole point of the 2026-08-06 "operational under degradation"
  reframe (stop optimizing toward "which known fault is this") already
  argued against this shape of question; Priority 0 never actually got
  updated to match that reframe, it just sat there un-executed. This note
  finishes that job.

**Disposition, agreed 2026-08-20:**
- **P0.1** (per-fault-class feature-importance recompute) — dropped
  entirely. Purely closed-set, no residual value under the new framing.
- **P0.2** (HD map ablation on detection accuracy) — NOT simply dropped.
  Its underlying question — is the map-derived graph context actually
  load-bearing for how the model behaves near structurally relevant scene
  elements (intersections, occlusion), or is it reading something else
  entirely — is a real prerequisite for Layer 2 below (§3): if the map
  context doesn't actually shape the model's anomaly behavior, there's
  nothing map-grounded to propagate through the digital twin, and the
  claim in §2 weakens to a generic trajectory-uncertainty roll-forward.
  Reframed, not reused as-is: the experiment should measure whether
  removing map context degrades the anomaly signal's scene-sensitivity
  (a continuous property), not classification-accuracy delta on a fixed
  fault-class label (the old, closed-set metric). Not yet run.
- Decoupling from `Graph-Scene-Representation-and-Prediction` (the T-ITS
  reference repo) as a dependency: agreed. It stays read-only reference
  material, cited for context, not something new work needs to reproduce
  against.

## 2. The new question

**Old**: "which known fault is this?" (classification into a fixed bin
set, needs fault data to validate).
**New**: "how far off-nominal am I right now, in what way, and what does
that imply about whether I'm about to do something unsafe?" (continuous,
needs no fault data to *detect* — see Layer 1).

The residual signal itself doesn't change — the ST-GAT's per-feature
prediction residuals are still the substrate. What changes is the question
asked of them: from "which bin does this residual sort into" to "how
surprising is this residual, and so what."

## 3. The actual gap in the literature, and the claimed contribution

Two bodies of work exist and don't talk to each other:
1. **UQ / anomaly detection**: monitors model uncertainty, raises a flag,
   hands control back or degrades. Treats uncertainty as an alarm — a
   magnitude to threshold, not a quantity to reason about further.
2. **Risk-aware planning**: computes collision risk and plans against it,
   but *assumes its own prediction models are correct*. Treats risk as a
   known input, not something whose own uncertainty needs propagating.

**Claimed contribution**: a principled translation from "my model is
uncertain in this specific way, right now" to "therefore this specific
driving behavior is unsafe, by this much." The digital twin is the bridge
— it can propagate uncertainty *forward into outcomes* (roll a
counterfactual future forward and see what it implies) rather than just
reporting a magnitude.

**Nuance flagged in review (2026-08-20, worth keeping precise for the lit
review)**: don't overclaim "nobody has a principled way to do this" as a
bare statement — some risk-aware planning work does already roll a
predicted distribution forward into a violation probability. What's
actually novel here is more specific: propagating a distribution-free,
*calibrated*, *per-feature-decomposed*, *map-grounded* uncertainty signal
— not a generic parametric (e.g. Gaussian) trajectory blob assumed correct
by construction. The novelty is in the quality and structure of what gets
propagated and how it's justified, not in the bare act of propagation.

**REFRAMED 2026-08-24, per an independent literature review (§9) and
Kalpit's explicit direction**: the paragraph above is now understood to be
still too close to "extend the UQ side." Waymo's own published safety
research already does forward-propagated, map-grounded, decomposed
consequence estimation — via forward REACHABILITY sets (Johnson/Victor/
Engström's "Field of Safe Motion," 2026), not a learned residual signal.
That work is not calibrated in the conformal sense (it assumes correct
state estimation, doesn't derive its safety margin from a model's own
learned, statistically-guaranteed uncertainty). The precise, defensible
novelty claim is now: **a synthesis of two traditions that don't currently
talk to each other** — data-driven, distribution-free CALIBRATION
(this project's Layer 1) combined with REACHABILITY-STYLE consequence
estimation (Layer 2, pivoting toward reachability sets rather than a
static lane-boundary margin — see §9) — not an extension of either
tradition alone. State it this way in any future writing; the older
"propagate a calibrated signal forward" framing above undersells what's
actually different once Field of Safe Motion is accounted for.

## 4. Three-layer architecture

1. **Layer 1 — Detect the unknown.** A continuously-valued anomaly
   measure with statistical guarantees, not a classifier. Residuals become
   nonconformity scores; conformal calibration gives distribution-free
   coverage without needing fault data at all. **Already built** —
   this IS `experiments/scripts/conformal_horizon_calibration.py`
   (leave-one-trial-out cross-conformal, validated 2026-08-20, see the
   pivot note). The reframe here is retroactive but exact: today's
   calibration pivot already produces Layer 1's deliverable.
2. **Layer 2 — Consequence estimation. ← THE THESIS.** A weird residual on
   an empty straight road is noise; the same residual approaching an
   occluded intersection is a crash. Map anomaly to actual risk by rolling
   counterfactual futures forward through the digital twin. **Not yet
   built** — this is the core remaining work.
3. **Layer 3 — Graceful response.** Not limp-mode or ODD binning —
   continuous modulation of behavior proportional to quantified ignorance.
   **Scoping question, not yet resolved**: does this pull the previously
   descoped RISE/active-control work (`CLAUDE.md`'s "Repo name is a legacy
   artifact" section — explicitly marked out-of-scope, future-work-only as
   of the 2026-07-24 reframe) back into the core claim, or does Layer 3
   stay at "define the shape the signal should take" without rebuilding an
   actual controller? Needs an explicit call before scoping Layer 3 work.

## 5. Conditioning the conformal quantile

Design question: what should the per-window conformal quantile be
conditioned on? Framing (agreed): **this is a sample-selection rule, not a
finding** — plumbing for borrowing statistical strength across similar
calibration windows, not a research contribution in itself. Don't chase a
novel similarity metric.

- Vanilla conformal (today's LOO-CV result) conditions on nothing beyond
  horizon step — one global width per step. Valid, but wide everywhere;
  the honest baseline to compare any conditioning scheme against, not a
  discarded first attempt.
- Hand-labeled regimes ("this is an intersection") — rejected. Doesn't
  scale, and the interesting situations are precisely the ones nobody
  thought to label ahead of time.
- **Proposed and preferred**: condition on the model's own learned scene
  embedding. The ST-GAT already computes one — `GraphEncoder`'s
  attention-pooled graph context (or `h_last` after the LSTM) encodes
  "many agents, converging trajectories, occlusion" without anyone typing
  the word "intersection." Scenario context is already latent in the
  architecture; use it rather than re-deriving it.

**Resolved 2026-08-20: this is a different axis from the horizon-step
embedding already in the model, not competing with it or replaceable by
it.** `model.py`'s per-head `horizon_embed` (fixed 2026-08-19, see
`project_horizon_embedding_fix_2026-08-19`) encodes *which future
timestep* — a lookup table indexed 0..29, carrying no scene content at
all. The scene embedding needed for conditioning here is a *different*
representation (graph context / `h_last`) carrying *what kind of
situation* this is. Condition on **both, layered**: keep per-horizon-step
quantiles (already built and validated) as one axis, and add scene-
embedding similarity as a second axis within each step — e.g., for a given
horizon step, take the ~200 calibration windows whose scene embedding is
nearest the current one (k-NN, or a small number of learned clusters), fit
the quantile on that neighborhood instead of the full calibration set.
Not yet implemented.

## 6. Experimental design needs, going forward

- **Richer "bad event" logging.** `experiments/lib/metrics.py`'s current
  `static_collision`/stuck heuristics are too rare and unreliable to build
  a real evaluation around (already noted in this repo's own history —
  see `project_risk_aware_control`'s 2026-07-25 entry on `static_collision`
  being a heuristic, not ground truth). Need continuously-loggable safety
  *margin* violations (time-to-collision thresholds, minimum following
  distance, lateral lane-boundary margin, etc.), not binary collision/stuck
  outcomes, to build Curve 1/3 and the fault-type table below.
- **More nominal data.** Independently re-derived from two directions now:
  this reframe's own experimental-design needs, and the conformal-
  calibration pivot's finding that `CAL_DIR`'s 7 trials give real,
  honestly-reported per-trial coverage instability (see the pivot note's
  §4). Same underlying gap, worth prioritizing once P0's old blocking
  status is fully retired.
- **What makes a risk number matter to an industrial safety team** (two
  properties, both currently absent from any existing signal in this
  repo):
  1. **Decomposition** — not just a magnitude but *where it came from*
     ("this risk is dominated by epistemic uncertainty about X"). The
     number triggers a response; the decomposition directs it (slow down,
     change viewpoint, buy information). This is a natural fit for the
     ST-GAT's already-existing per-feature residual/NLL trace design — see
     `docs/theoretical_framework.md`'s divergence-trace schema — don't
     rebuild that machinery, extend it.
  2. **Risk as currency, not alarm** — a planner-optimizable quantity
     (e.g. "braking harder cuts violation probability from 1e-3 to 1e-5 at
     comfort cost C" is a tradeoff a planner can act on), not a binary
     flag. This is the actual bridge to Layer 3.

## 7. Target result shape

Four artifacts, roughly in order of how buildable they are today:

1. **Curve 2 — reliability diagram (calibration of the risk estimate)**.
   Closest to already-buildable: log predicted P(violation within horizon
   H) at every timestep, check ground truth H seconds later, bin by
   predicted probability, plot predicted vs. observed. This is the same
   bookkeeping as today's LOO-CV coverage check, generalized from
   per-feature interval coverage to a violation-probability formulation.
   Needs: horizon H, a violation definition (§6), enough episodes per bin.
2. **Curve 1 — lead time vs. false alarm rate (the headline)**. A single
   lead-time number is meaningless (any anomaly signal has a sensitivity
   knob trading lead time against false-alarm rate) — report the curve,
   compare methods at matched false-alarm rates. **Why decomposition wins
   here, stated precisely because it's the clearest justification yet for
   this architecture's per-feature design**: a monolithic residual must
   clear one global threshold, pinning it to one point on that tradeoff
   curve. Per-subsystem anomaly tracking exploits *shape* — a structured,
   persistent signature (e.g. an IMU bias producing coherent drift in one
   direction) can be flagged far more sensitively than incoherent noise,
   without eating false alarms elsewhere. **Lead time is bought with
   structure, not magnitude.** Needs: fault-condition data, margin-
   violation logging (§6), Layer 2 built.
3. **Table — fault type × margin violated × lead time.** Same
   dependencies as Curve 1.
4. **Curve 3 — detection threshold vs. fault magnitude.** Sweep fault
   magnitude downward; large faults are easy, the discriminating regime is
   small ones. Target shape: "conditional conformal detects bias faults
   40% smaller than baseline at matched false-alarm rate." Explicitly
   flagged by Kalpit as needing more experiment infrastructure than the
   others — treat as a stretch goal, not a near-term commitment.

## 8. How this relates to what's already built

- Layer 1 (§4) = the conformal-calibration pivot, already done and
  validated. No new work needed there beyond the conditioning extension
  (§5) and running it against fault-condition data (§6/§7's dependency).
- The epistemic-disagreement check
  (`nll_calibration_arc_and_conformal_pivot_2026-08-20.md` §6) is a
  Layer-1-adjacent robustness question (does a point predictor's behavior
  stay trustworthy under distribution shift), not yet Layer 2.
- Layer 2 is genuinely new work — nothing in this repo does counterfactual
  rollout-through-the-digital-twin yet.
- Layer 3's controller question is explicitly unscoped pending Kalpit's
  call (§4).

## 9. Layer 1 wasn't actually done; Layer 2 pivots to reachability (2026-08-24)

Three things happened the same day that materially change §4 and §8 above.

**(a) Layer 1's "fully validated" claim (§8, and the 2026-08-21 memory
entry) was premature.** Concrete trust-visualization plots
(`experiments/scripts/plot_layer1_trust_examples.py`) surfaced by eye that
the position head barely anticipates turns from route/map context —
quantified in `experiments/scripts/diagnose_turn_learning.py`: predicted
heading-change magnitude is only ~48% of actual when a turn hasn't started
yet in the observed past, vs. ~65% once already mid-turn. Kalpit's
response, correct and important: **this cannot just be papered over by
widening the calibrated interval on turns (scene-conditioning, §5) — the
model's actual weakness needs fixing, because fault injection is
specifically gated on turn zones and TL/intersection zones
(`experiments/lib/fault_injector.py`: all 4 IMU fault types gate on
`turn_zones`/`bias_leadin_zones`, all 5 TL fault types gate near
`tl_zones` — there is no OTHER fault-injection locus in this project).**
A wider-but-honest interval at exactly the place faults are injected means
lower detection sensitivity exactly where it matters most for the
dissertation's practical claim — calibration validity and detection power
are in tension here, not the same thing. Scene-conditioning
(`experiments/scripts/conformal_scene_conditioning.py`) stays valuable as
a safety net for genuinely irreducible scene-dependent variance, but is no
longer treated as sufficient on its own — model improvement (reweighting/
oversampling minority scenes, see below) is now the primary track, with
scene-conditioning re-evaluated afterward on whatever residual gap
remains once the model itself is better.

**(b) Systematic minority-scenario audit, grounded in the project's own
fault-targeting geometry, not an ad hoc metric** (Kalpit's point: "we
would have never caught [the turn gap] if it weren't for the plots... we
might be missing other minority scenes too"). `experiments/scripts/
audit_minority_scenarios.py` reuses the exact zone files fault_injector.py
already gates on (`experiments/configs/turn_zones.json`'s turn_zones/
bias_leadin_zones/lane_change_zones/curved_road_zones, and
`experiments/configs/tl_zones.json`'s tl_zones — all real geometry
computed from driven trajectories, not guessed), labels every calibration
window by proximity, and checks coverage against the ALREADY-DEPLOYED
global quantile per category. Found a SECOND gap the turn diagnostic alone
missed: **`tl_zones` (intersections) has the worst steering coverage of
any category, 75.6%** — worse even than turn_zones (80.3%) or
bias_leadin_zones (81.6%) — and both turn_zones/tl_zones are exactly where
fault injection is targeted. Also surprising: `open_road` (44.4% of the
data, NOT near any special zone) under-covers on position (85.3%) and
longitudinal velocity (86.2%) despite being the "easy" majority case —
not yet investigated further, a real open item. `curved_road_zones`
(not fault-targeted) calibrates fine. `lane_change_zones` has essentially
zero matching windows in the current 7-trial dataset — not a calibration
gap, a DATA gap (this scenario type is barely represented in the map/route
set at all) — worth flagging for future data collection alongside more
nominal trials generally (§6). **This audit script should be re-run any
time the model or calibration set changes — it's the repeatable version
of what an ad hoc plot review caught once by luck.**

**(c) Layer 2's preferred direction is now forward reachability, not a
static lane-boundary margin.** Per the independent Waymo-literature review
(2026-08-24, summarized in project memory — full report not duplicated
here) and Kalpit's explicit confirmation: replace the paused v1 design's
`lane_boundary_distance` + naive constant-velocity object extrapolation
(`experiments/lib/margin.py`, still has its known adjacent-lane-boundary
bug, still unfixed) with a reachability-style margin in the shape of
Johnson/Victor/Engström's "Field of Safe Motion" (2026) — bounded reachable
sets for nearby tracked objects (not a single constant-velocity point
estimate) pruned by lane-containment (`lanelet2.geometry.inside` on the
ego's actual current lanelet, not nearest-5), rather than a bare geometric
distance. This is a genuine redesign of margin.py's approach, not a patch
to the existing bug — do not just fix the nearest-lanelet bug in the old
design and call Layer 2 unblocked; the whole margin computation shape is
changing. Not yet implemented as of this note.

**(d) A dedicated, robust literature review is now an explicit task**,
not assumed already done by the one 9-paper Waymo-only review. That review
was scoped to Waymo's own published safety research specifically (a
reasonable first pass, and it's what surfaced the reachability pivot in
(c)) — it is NOT a substitute for a broader search across the reachability-
analysis, conformal-prediction-for-planning, and AV-safety-verification
literatures more generally, needed to make sure the "calibration ×
reachability" synthesis claim (§3's reframe above) actually holds up and
isn't already done elsewhere under different terminology. Scope this
before writing up Layer 2's contribution as novel, not after.
