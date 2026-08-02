# ST-GAT Model — Known Limitations & Candidate Improvements (2026-08-02)

**Status:** the model now meets this project's stated accuracy targets and is
reasonably calibrated (see `docs/research_notes/trajectory_prediction_literature_2026.md`
for the fixes that got it there). This doc is the punch list of what's still
imperfect or unexplored, written down so it doesn't have to be rediscovered —
none of it is currently blocking, all of it is a candidate for later.

## 1. Calibration is now a mix of slightly under- and over-confident, not uniformly one direction

Final z-score std (want 1.0; `>1` = predicted band too *narrow* = **overconfident**;
`<1` = too *wide* = **underconfident**), 1-step-ahead on held-out cal data:

| feature | std(z) | direction |
|---|---|---|
| position | 0.885 | mildly underconfident |
| velocity | 1.174 | mildly **overconfident** |
| steering | 1.101 | mildly **overconfident** |
| traffic_light_state | 1.332 | **overconfident** (the largest miscalibration remaining) |

Position calibration also drifts with horizon — std(z) 0.885 at 0.1s ahead,
1.177 at 3.0s ahead (see the horizon table this note is based on, computed
2026-08-02) — crossing from underconfident to overconfident as horizon grows.
Not alarming (both ends are within a reasonable band, and accuracy itself
degrades gracefully with horizon too), but worth knowing before treating
"calibrated" as a solved, static property rather than something that shifts
by feature and by horizon.

**Why this matters for the dissertation specifically:** overconfidence is the
more dangerous failure mode for a safety-verification monitor (a system that's
confidently wrong is worse than one that's honestly unsure — this is stated
explicitly in `docs/theoretical_framework.md` §7). traffic_light_state being
the most overconfident feature is a real thing to watch once Stage 4
(conformal calibration) is built — conformal prediction's coverage guarantee
should catch and correct this kind of miscalibration automatically, but it's
worth confirming that empirically rather than assuming the calibration layer
fixes everything by construction.

**Candidate follow-up, not done:** per-feature `BETA_NLL` (currently one
global value, 0.5, for every Gaussian-headed feature) — traffic_light_state's
worse overconfidence might respond to a different beta than position's. Cheap
to try, not yet tried.

## 2. Route/goal information reaches the model, but gets diluted

Checked directly (not assumed): the model **does** receive real route
information — `GraphBuilder._create_lanelet_nodes` sets a binary `path_node`
flag per graph node from the actual Autoware planned route
(`/planning/mission_planning/route`, parsed in `sequence_builder.py`'s
`extract_route_from_bag`). It does **not** receive any goal-position,
distance-to-goal, or route-progress feature — no code path computes one.

For a 3-second-horizon trajectory model, absolute goal position (often
hundreds of metres away) is probably not that informative on its own — but
the **local route direction/curvature** (which the `path_node` flags already
encode) plausibly is, especially right before a turn. The problem is how it's
used: `GraphEncoder.forward()` mean-pools **all** ~150 graph nodes into one
vector per window (`h.mean(dim=0)`), discarding *which* specific nodes are
on-route vs. not — and that single pooled vector is then broadcast identically
across all 30 input timesteps (`graph_ctx.unsqueeze(1)` in `model.py`). So the
architecture has real route data available, but its own pooling step blurs
away exactly the directional signal ("which way does my route go from here")
that would most help — the same mean-pooling design already flagged as a
cadence question in `docs/theoretical_framework.md` §4 turns out to also cost
route-direction fidelity, not just moment-of-divergence freshness.

**Candidate follow-ups, not done (from cheapest to more involved):**
- An explicit scalar feature — e.g. route heading a few seconds ahead, or
  distance/bearing to the next turn — computed once per timestep from the
  route polyline, sidestepping the pooling problem entirely without touching
  the graph architecture.
- Attention-weighted pooling instead of uniform mean-pooling in
  `GraphEncoder` (weight nodes near ego / on-path higher) — closer to how
  VectorNet/LaneGCN-style architectures handle this (see the literature
  review doc), a real architecture change, not a quick add.
- Per-timestep graph re-pooling (revisiting the cadence question directly)
  would also incidentally fix this, since a freshly-pooled context could
  track "which on-route node is nearest right now" instead of one static
  window-level average.

Given the model already meets its accuracy targets, this is a plausible lever
for the **longer-horizon** end specifically (where turns matter most, and
where position calibration is already the shakiest per §1) — not urgent, but
a good first thing to try if longer-horizon accuracy needs to improve later.

## 3. Deep ensemble (epistemic uncertainty) not yet built

Already tracked in `TODO.md` P1.6, repeated here for completeness: the current
model is a single network, so its predicted variance is aleatoric-only
(irreducible per-situation noise), not the aleatoric+epistemic decomposition
`docs/stgat_pipeline_plan.md` §1.9 notes the reference repo got right and this
project intends to reuse. Matters more for Stage 4 (calibration) than for the
exploratory fault-reaction study, since epistemic uncertainty is specifically
about "how much would this prediction change with different training data" —
relevant to trusting calibration under distribution shift (fault trials),
less relevant to just checking whether residuals discriminate faults at all.

## 4. VAR_FLOOR is one global constant; got lucky it only needed lowering once

`VAR_FLOOR` (now `1e-8`, was `1e-4`) applies uniformly to every Gaussian-headed
feature regardless of that feature's own natural error scale — this session's
bug was that position's real error scale shrank enough (after the
representation fix) to fall below the old floor, pinning its variance. The
current fix (lower the one global constant) worked because no other feature's
natural variance was close to `1e-8` either direction — but this is a
coincidence of today's numbers, not a structural guarantee. If any feature's
error scale changes substantially in the future (e.g. after the route/goal
fix in §2, or after training on more/different data), the same failure mode
could reappear for a different feature. **Candidate follow-up:** a per-feature
floor (e.g. a dict in `config.py`, each tied to that feature's own expected
error scale) would be more robust than hoping one global constant stays
non-binding for everyone — not done now since the current single value works
and there's no evidence yet it needs to be split.

## 5. Object-set encoder: mean-pool only, unweighted

`ObjectSetEncoder` (added this session) mean-pools per-object embeddings
uniformly regardless of relevance (a stationary parked car far from the route
counts the same as a pedestrian about to cross). Deliberately kept simple for
a first pass (see `TODO.md` P1.6) — attention-pooling is the natural next
step if object context turns out to matter for anything (unlikely to be
high-priority given objects don't carry a hard map-grounded prior per
`docs/theoretical_framework.md` §3.1, but noted for completeness).
