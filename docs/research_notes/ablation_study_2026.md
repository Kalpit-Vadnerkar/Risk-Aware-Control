# Ablation / pipeline-improvement study (started 2026-08-05)

**Purpose:** track every change made to the pipeline or model against measured
before/after numbers, not intuition — the thing `TODO.md`'s P1.6 checklist
asks for but doesn't itself track results against. Each entry below is a real
change with a real measurement, not a hypothesis alone. Candidates not yet
tried are listed at the end, cross-referenced to `TODO.md` P1.6.

## Summary table

| # | Change | Level | Status | Key before → after | Verdict |
|---|---|---|---|---|---|
| 1 | Frame-gap contiguity gate (`sequence_builder.py`) | data pipeline | **done, retrained** | velocity std(z) 1.174→**0.978**; steering 1.101→0.927; tl_state 1.332→1.266 | Real, measured improvement — see §1 |
| 2 | Route-aware graph node selection (`GraphBuilder.build_graph`) | data pipeline | **found, not yet implemented** | path_node fraction currently 5.6% mean (median 3.3%, min 0%) of 150-node budget | High-confidence candidate — see §2 |
| 3 | Attention-weighted graph/object pooling (vs. uniform mean) | model architecture | proposed, not implemented | n/a | See §3 — likely secondary to #2 |
| 4 | TL-discrepancy calibration (temperature scaling / conformal classification) | analysis layer | in progress | 0 detections before AND after retrain (§1 confirms retrain didn't touch this) | Structural fix needed regardless of model quality |
| 5 | Graph cadence (re-pool per-timestep vs. once per window) | model architecture | not started (`TODO.md` P1.6) | — | — |
| 6 | Capacity sweep (`d_model`/`hidden_size`/`num_layers`/`nhead`) | model architecture | not started (`TODO.md` P1.6) | — | — |
| 7 | `MAX_GRAPH_NODES=150` sweep | model architecture | not started (`TODO.md` P1.6) | — | — |

---

## 1. Frame-gap contiguity gate (done, measured)

**Finding:** `bag_reader.py`'s per-topic staleness check correctly drops
individual stale frames, but the resulting list has holes; `sequence_builder.py`
sliced `frames[i:i+total]` assuming array-adjacency implied ~0.1s real-time
adjacency. Confirmed directly: gaps up to 31.4s between array-adjacent frames
in an ordinary nominal trial (e.g. a `control_cmd` topic outage), silently
splicing unrelated moments into one training window.

**Fix:** reject any window whose internal frame-to-frame gap exceeds 0.2s
(`SequenceBuilder._MAX_FRAME_GAP_SEC`). Measured directly: **237 of 2012
windows (11.8%) in one trial** were corrupted this way before the fix.

**Before/after (z-score calibration on held-out calibration split,
`experiments/scripts/check_calibration.py`, 1-step-ahead):**

| feature | std(z) before | std(z) after | direction |
|---|---|---|---|
| position | 0.885 | 0.837 | slightly more underconfident |
| velocity | 1.174 (overconfident) | **0.978** | corrected — nearly exact |
| steering | 1.101 (overconfident) | 0.927 | corrected, now mildly underconfident |
| traffic_light_state | 1.332 (overconfident) | 1.266 | improved, still worst |

**Downstream effect on fault-reaction results** (`conformal_lead_time.py`,
`--statistic velocity_nll`):
- `imu_fault_stuck`: 0/2 detections before → **1/2, 21.1s lead time** after (a genuinely new detection, not just a better number on an existing one)
- `imu_fault_s3`: 1/2 detections both before and after, lead time grew 6-9s → **12.6s**
- `imu_fault_s1` (negative control, meant to be harmless): 1/2 → **0/2** — a *decrease* in detections that is the *correct* direction, matching the discriminability rerun showing velocity_nll_delta going from positive to near-zero/negative for this campaign specifically

**Verdict:** confirmed real improvement, not noise — the direction of every change is what the frame-gap hypothesis predicted, including the encouraging *decrease* in false-positive-shaped behavior on the negative control.

---

## 2. Route-aware graph node selection (found, not yet implemented)

**Finding, in two parts:**

1. `st_gat/pipeline/State_Estimator/GraphBuilder.py`'s `build_graph()` iterates
   candidate lanelets sorted **purely by distance** from the window's center
   point (`_get_sorted_lanelets`), filling the `MAX_GRAPH_NODES=150` budget
   greedily nearest-first, with **no check for whether a lanelet is on the
   route** (`path_node`). `clip_graph()`'s later truncation step is the same
   — distance-only.
2. Measured directly on 200 real training sequences: mean **5.6%** of a
   graph's nodes are `path_node=1` (median 3.3%, min **0%** — some windows
   have zero route representation in the graph at all).

This means route information is diluted twice: the graph construction itself
rarely includes many route nodes in the first place (root cause), and the
model's `GraphEncoder.forward()` then mean-pools all ~150 nodes uniformly
(`h.mean(dim=0)`, compounding factor — see §3). In a dense intersection area
(Nishishinjuku has several multi-lane crossings), the route is one thread
among many nearby lanelets — a purely-distance-greedy selection has no reason
to prefer it.

**Proposed fix:** two-pass fill in `build_graph()` — add all `path_node`
lanelets within the window first (a 3-second window's local route segment is
almost certainly far under 150 nodes), then fill remaining budget with
nearest off-route lanelets for context, same as today. Cheap: a data-pipeline
change (like §1), not a model-architecture change — testable with
re-extraction + retrain, no new model code.

**Not yet measured — this is the next candidate to test**, likely alongside
the object-pooling change in §3 so both fixes get one retrain cycle instead
of two.

---

## 3. Attention-weighted pooling (graph nodes + object set)

Same underlying architectural pattern shows up in two places:

- `GraphEncoder.forward()`: `h.mean(dim=0)` — uniform mean over all graph
  nodes, no matter how sparse the important ones are (compounds §2's
  selection problem).
- `ObjectSetEncoder.forward()`: masked mean-pool over up to 8 tracked
  objects — a stationary parked car far from the route counts the same as a
  pedestrian about to cross (already flagged in
  `model_improvement_notes_2026.md` §5, now understood as the same pattern
  as the graph case, not an isolated concern).

**Recommendation:** fix §2 (graph node selection) first and measure — if
route nodes are reliably present in the graph, a uniform mean over a
mostly-route-relevant node set is a much smaller problem than a uniform mean
over a mostly-irrelevant one. Attention-pooling is still likely worth doing
for the object set (K=8, no selection-level fix available there the way §2
fixes the graph case), but treat it as a secondary change, not the first
lever to pull.

---

## 4. TL-discrepancy calibration (in progress)

`traffic_light_discrepancy` is a Bernoulli/sigmoid head (BCE loss), not a
Gaussian mean/variance head — no notion of a confidence interval the way
position/velocity have one. Confirmed (§1's retrain) that this is a
*structural* mismatch, not a training-noise problem: the frame-gap fix
measurably improved every Gaussian-headed feature but left
`traffic_light_discrepancy_residual`'s conformal detection at 0/8 fault
trials, unchanged, before and after. Literature grounding downloaded
(`docs/papers/2017_guo_temperature_scaling_calibration.pdf`,
`2016_sadinle_conformal_classification_sets.pdf`,
`2021_angelopoulos_bates_conformal_prediction_intro.pdf`,
`2002_zadrozny_elkan_isotonic_calibration.pdf`) — implementation next.

---

## Candidates not yet started (see `TODO.md` P1.6 for the original list)

- Graph cadence (re-pool per-timestep vs. once per window) — a third,
  independent dilution axis on top of §2/§3: even a perfectly-selected,
  attention-pooled graph context is currently broadcast identically across
  all 30 input timesteps (`seq = seq + graph_ctx.unsqueeze(1)` in
  `STGAT.forward()`), so the model can't distinguish "the route turns in 1s"
  from "the route turned 2s ago" within one window.
- Capacity sweep (`d_model`/`hidden_size`/`num_layers`/`nhead`, all 128/128/2/4,
  never tuned).
- `MAX_GRAPH_NODES=150` sweep — is 150 enough, too many, or arbitrary.
