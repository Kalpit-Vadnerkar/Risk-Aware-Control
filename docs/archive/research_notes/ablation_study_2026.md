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
| 2 | Route-aware graph node selection (`GraphBuilder.build_graph`) | data pipeline | **done, retrained** | path_node fraction 5.6%→**22.0%** (and stable, not just higher) | Real, measured fix — see §2 |
| 3 | Attention-weighted graph/object pooling (vs. uniform mean) | model architecture | **done, retrained (bundled with #2)** | `imu_fault_s3` lead time 12.6s→**62.0s** (same detection rate); calibration MIXED (position/accel improved, velocity/steering/tl_state worse) | Real fault-detection win, real calibration cost — see §3 |
| 4 | TL-discrepancy temperature scaling (Guo et al. 2017) | analysis layer | **done** | ECE 0.0292→**0.0085** (3.4x tighter); TL fault detection **unchanged, still 0/12** | Fixed the calibration metric, not the detection problem — see §4 |
| 5 | Graph cadence (re-pool per-timestep vs. once per window) | model architecture | **TBD** | — | Independent of #2/#3 — even a perfectly-selected, attention-pooled graph context is still one static vector broadcast across all 30 input timesteps; see §"Candidates not yet started" |
| 6 | Capacity sweep (`d_model`/`hidden_size`/`num_layers`/`nhead`) | model architecture | **TBD** | — | — |
| 7 | `MAX_GRAPH_NODES=150` sweep | model architecture | **done — raised to 1024** | uncapped candidate nodes at radius=150m measured 700-990 per window; 150-node cap was discarding ~80% of them | Not a sweep — direct measurement showed 150 was still badly binding even after the radius gate (§10); raised to comfortably cover the observed range, per Kalpit's explicit choice to accept the GCN compute cost rather than coarsen node spacing or shrink the radius — see §10 |
| 8 | Stopped-period frame filter removed (`bag_reader.py`) | data pipeline | **done** | — (no retrain needed to observe the fix, it's a data-inclusion change) | Filter unconditionally deleted any >3s continuous near-zero-speed segment from EVERY bag, fault trials included — see §5 |
| 9 | TL feature redesign: color/confidence split, `traffic_light_detected` retired | data pipeline + model architecture | **done, not yet retrained** | — | Fixes two separate signal-mangling bugs in the old 3-feature TL scheme — see §6 |
| 10 | Real traffic-light graph nodes + radius-gated candidate selection (tied to horizon) | data pipeline | **done, not yet retrained** | on-route candidates 18-19/19 lanelets (100%) at radius=∞ → mixed on/off-route within a 150m radius | Fixes both the "old 150-node cap crowded out off-route context" AND "route-aware fix overshot into an all-route graph" failures — see §7 |
| 11 | Topology-aware graph connectivity (lanelet2 routing relations, not reactive bridging) | data pipeline | **done, not yet retrained** | see §8 for the graph-quality argument, §9 for the runtime number this also fixed | Replaces the brute-force nearest-pair connectivity repair, which fired on 100% of graph builds pre-fix |
| 12 | Extraction runtime fix (per-frame memoization, KD-tree spatial indexing, deepcopy-on-rebuild-only) | data pipeline | **done, measured** | one representative trial's `SequenceBuilder.build()`: **106.4s → 2.26s (47x)** | See §9 — root-caused via profiling, not guessed |

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

**Fix implemented 2026-08-05:** `_get_sorted_lanelets` now sorts on-route
lanelets first (still nearest-first within each group) via
`sorted(lanelets, key=lambda x: (not x[3], x[2]))`. `clip_graph()` was left
alone — it's a no-op in the current config since `MIN_GRAPH_NODES ==
MAX_GRAPH_NODES == 150`, so the fill loop's own cap always short-circuits
it; flagged in a comment in case the two constants are ever decoupled.

**Measured:** re-extracted and checked the same 200-sequence sample from the
same file as the original finding — path_node fraction went from a highly
variable 0–18.7% (mean 5.6%) to a **stable ~22.0%** across all 200 windows.
Bundled with §3's attention-pooling change for one retrain cycle (see §3 for
the downstream fault-detection and calibration effect — the two changes were
trained together, not measured in isolation).

---

## 3. Attention-weighted pooling (graph nodes + object set)

Same underlying architectural pattern showed up in two places:

- `GraphEncoder.forward()`: `h.mean(dim=0)` — uniform mean over all graph
  nodes, no matter how sparse the important ones are (compounded §2's
  selection problem).
- `ObjectSetEncoder.forward()`: masked mean-pool over up to 8 tracked
  objects — a stationary parked car far from the route counted the same as a
  pedestrian about to cross (already flagged in
  `model_improvement_notes_2026.md` §5, understood as the same pattern as
  the graph case, not an isolated concern).

**Fix implemented 2026-08-05, bundled with §2 in one retrain:** both encoders
now compute a learned per-node/per-object attention score (`nn.Linear(d, 1)`),
masked to `-inf` on padding before softmax for the object case, and take a
weighted sum instead of a plain mean. Smoke-tested against synthetic tensors
first (no NaN, correct shapes, all-padding case still gives a clean zero
vector) before the full retrain.

**Baseline preserved** at `st_gat/models/h30_30_baseline_meanpool/` and
`st_gat/checkpoints/h30_30_baseline_meanpool/` for future ablation
comparison, per Kalpit's standing preference (2026-08-05: "next time we
should keep the trained models for ablation studies") — not deleted this
round.

**Measured (§2+§3 combined, since trained together):**

Retraining converged faster (early stop epoch 48 vs. 80) to a *slightly
worse* raw validation error (0.0218 vs. 0.0207, ~5% relative) — flagged
honestly rather than glossed over; raw position/velocity tracking error was
never the actual target metric, just the checkpoint-selection proxy.

z-score calibration, mean-pool baseline → route-aware + attention:

| feature | before | after | direction |
|---|---|---|---|
| position | 0.837 | 0.928 | improved |
| velocity | **0.978** (near-perfect) | 0.833 | **worse** — now underconfident |
| steering | 0.927 | 0.835 | **worse** — now underconfident |
| acceleration | 1.178 | 1.114 | improved |
| traffic_light_state | 1.266 | **1.444** | **worse** — most overconfident yet |

Mixed, not a clean win on calibration. Plausible explanation (not confirmed):
new attention parameters need more training than early stopping allowed —
epoch 48 is much earlier than the previous run's 80.

Fault-reaction effect (`conformal_lead_time.py --statistic velocity_nll`,
mean-pool baseline → route-aware + attention, same 2 valid trials/campaign
after the goal_026 exclusions):

| campaign | detection rate | lead time |
|---|---|---|
| `imu_fault_s1` (negative control) | 0/2 → 0/2 | — (correctly stayed at zero) |
| `imu_fault_s3` | 1/2 → 1/2 | 12.6s → **62.0s** |
| `imu_fault_scale` | 0/2 → 0/2 | — |
| `imu_fault_stuck` | 1/2 → 1/2 | 21.1s → 21.4s (~unchanged) |

**Verdict:** a real, substantial win for `imu_fault_s3`'s lead time (5x
larger), no change in which trials get detected at all, and a real
calibration cost on 3 of 5 Gaussian features. Net positive for the stated
goal (IMU fault detection) but not free — worth revisiting once graph
cadence (still TBD) is also addressed, since the mixed calibration result
could partly be a symptom of the same static-broadcast problem rather than
the attention mechanism itself.

---

## 4. TL-discrepancy calibration (done — fixed the metric, not the detection problem)

`traffic_light_discrepancy` is a Bernoulli/sigmoid head (BCE loss), not a
Gaussian mean/variance head — no notion of a confidence interval the way
position/velocity have one. Confirmed (§1's retrain) that this is a
*structural* mismatch, not a training-noise problem: the frame-gap fix
measurably improved every Gaussian-headed feature but left
`traffic_light_discrepancy_residual`'s conformal detection at 0/8 fault
trials, unchanged, before and after.

**Implemented:** temperature scaling (Guo, Pleiss, Sun & Weinberger, ICML
2017 — `docs/papers/2017_guo_temperature_scaling_calibration.pdf`). Model now
also returns the pre-sigmoid logit (`traffic_light_discrepancy_logit`, no
effect on training/loss). `experiments/scripts/calibrate_tl_discrepancy.py`
grid-searches a single scalar T minimizing BCE on the held-out calibration
split; `st_gat/residuals.py` uses the fitted T to add a
`traffic_light_discrepancy_calibrated_residual` column alongside the
original.

**Measured:** T=0.80 (mild sharpening — the model was mildly
underconfident overall). **ECE (10-bin) 0.0292 → 0.0085**, a real ~3.4x
tighter probability calibration; reliability bins visibly track the
diagonal much more closely after scaling.

**But: zero effect on fault detection.** Ran `conformal_lead_time.py
--statistic traffic_light_discrepancy_calibrated_residual` against the
retrained (route-aware + attention) model and got numbers **identical to
three decimal places** to the uncalibrated version — TL campaigns stayed at
0/12 detections either way. Explanation: most predicted probabilities sit
in the [0, 0.1) reliability bin (5759/8074 examples), where a T=0.80 rescale
barely moves the actual value — temperature scaling corrects calibration in
aggregate but doesn't change which specific examples cross a threshold when
the underlying signal magnitude for genuine TL faults is this small to
begin with. Calibration and detection strength are genuinely separate axes;
fixing one doesn't fix the other. `traffic_light_discrepancy` conformal
classification (Sadinle et al. 2016) is still worth trying since it changes
*how* the threshold itself is built, not just the probability feeding it —
but the honest expectation now is that it may not move detection either if
the signal itself is this weak for these campaigns.

---

## 5. Stopped-period filter removed (done)

`bag_reader.py`'s `_filter_stopped_periods` (inherited verbatim from the old
reference repo's `MessageCleaner.process_velocity_data()`) deleted any
continuous >3s near-zero-speed segment from every bag's frame list,
unconditionally — nominal AND fault trials alike, no exemption. Audited
2026-08-05: this is a real problem for exactly the campaigns that matter
most. `imu_fault_stuck` is a campaign built around inducing this behavior;
any MRM emergency-stop escalation produces the same pattern. The filter
could silently delete the fault signature itself before the model or
residual pipeline ever saw it. It also corrupted `mrm_first_trigger_rel_s`
(a ground-truth safety marker `st_gat/residuals.py` joins into every trace),
which scans the post-filter frame list for the first MRM-active frame — if
MRM triggered during a deleted stop, that marker shifted to a later,
spurious trigger or went missing entirely.

**Decision (Kalpit, 2026-08-05):** remove outright, not adjust the
threshold or exempt fault campaigns selectively. The filter predates
`traffic_light_state`/color/confidence existing as features and was
designed to keep long idle/parked segments out of nominal training data —
the nominal dataset doesn't actually contain idle/parked segments, so it
had no remaining upside once that original motivation stopped applying.

## 6. TL feature redesign: color/confidence split, `traffic_light_detected` retired (done, not yet retrained)

Three separate problems found in the old 3-feature TL scheme
(`traffic_light_detected`, `traffic_light_state`, `traffic_light_discrepancy`),
all traced to code inherited from the reference repo and never re-examined:

1. **`traffic_light_state` collapsed color and confidence into one scalar**
   (`color_value * confidence`). The fault-injection design has a dedicated
   `tl_confidence` fault (`fault_injector.py`: multiplies each reported
   element's confidence by a scale factor, color untouched) — but a
   collapsed scalar can't distinguish "confidence dropped, color unchanged"
   from "an undamaged reading of a more-permissive color at higher
   confidence"; the two faults produced indistinguishable downstream signal.
   Split into two independent Gaussian-headed features, `traffic_light_color`
   and `traffic_light_confidence`, both describing the SAME reported element
   (not maxed independently across elements, which would reintroduce the
   same kind of manufactured mixing).
2. **`traffic_light_detected` was a manufactured, redundant proxy.** It
   flagged whichever EXISTING lane-centerline node happened to fall within
   10m of a TL linestring, then had its own Bernoulli output head predicting
   it — a deterministic geometric fact given ego position + map, with no
   real uncertainty, riding on a node whose position was really about lane
   geometry, not the light. Retired as an output feature entirely (head and
   loss term removed) once real TL graph nodes (§7) made the map-adjacency
   pattern structurally visible to the GCN directly.
3. As a direct consequence, `traffic_light_discrepancy`'s ground-truth label
   (`map expects a TL here AND perception reported nothing`) is now computed
   from a new `TrafficLightExpectationChecker` (real-world-coordinate
   proximity to an actual regulatory-element position, reusing
   `compute_tl_zones.py`'s exact position computation) instead of from the
   retired `traffic_light_detected` proxy.

Model/loss/dataset/residuals/check_calibration all updated in lockstep
(`head_tl_color`/`head_tl_confidence` replace `head_traffic`/`head_tl_state`;
`DEFAULT_WEIGHTS`/`_GAUSSIAN_FEATURES`/`_SCALAR_KEYS` updated accordingly).
`traffic_light_detected`'s own temperature-scaling question (raised
2026-08-05: empirically measured ECE 0.0422 uncalibrated, comparable to
`traffic_light_discrepancy`'s pre-fix 0.0292) is now moot — there's no head
left to calibrate.

## 7. Real traffic-light graph nodes + radius-gated candidate selection (done, not yet retrained)

Two changes, designed together because they interact:

**Real TL nodes** (`GraphBuilder._add_traffic_light_nodes`, replacing
`update_traffic_lights`): one dedicated node per distinct traffic-light
regulatory element governing a lanelet actually included in the graph,
positioned at the light's real physical location (average of its
linestring points — identical computation to `compute_tl_zones.py`'s
`route_tl_zones`, so "the light a TL fault mutates" and "the light with a
graph node" are always the same real-world entity). Connected to its
nearest existing lane node. Replaces flagging an existing lane-centerline
node as a proxy for TL presence.

**Radius-gated candidate selection** (`GraphBuilder._get_sorted_lanelets`,
`config.py`'s `GRAPH_RADIUS_M`): candidate lanelets are filtered to a radius
around the window centre BEFORE route-first sorting is applied, instead of
route preference having no distance bound at all. Measured directly why
this was needed: one real trial's graph filled its entire 150-node budget
from just 19 lanelets, 18 of them on-route — including one lanelet 656m
from ego — while only 1 off-route lanelet made it in, out of 884 total road
lanelets in the map. A route lanelet 600m away is irrelevant to a 3-second
prediction window; the route-aware fix (§2) correctly solved the original
"route nodes crowded out by nearby off-route ones" problem but overshot
into the opposite failure mode once combined with an unbounded-distance
sort. Radius is derived from the horizon
(`(INPUT_SEQ_LEN+OUTPUT_SEQ_LEN)*0.1s * 15 m/s + 60m margin` = 150m at the
current h30_30 horizon), per Kalpit's request to tie it to the prediction
horizon rather than hand-pick a constant — a future horizon sweep gets a
correctly-scaled radius automatically.

**Bug caught by direct testing, fixed same day:** TL nodes are added AFTER
the lane-node fill loop. On a graph where the lane fill alone already
consumed the full 150-node cap (observed directly: 150 lane nodes + 7 TL
nodes = 157 total), the TL nodes' ids (150-156) landed past
`dataset.py`'s fixed-size `_MAX_GRAPH_NODES=150` padding tensor and were
silently dropped before ever reaching the model — the entire point of this
change would have been invisible to training. Fixed by counting distinct TL
regulatory elements among ALL candidate lanelets before the fill loop and
reserving that many node slots (`lane_node_budget = max_nodes - n_reserved_tl`),
guaranteeing TL nodes always fit. Re-verified after the fix: 138 lane nodes
+ 7 TL nodes = 145 ≤ 150, all present, `node_mask` correctly reports 145 real
nodes.

**`MAX_GRAPH_NODES=150` (table row #7): checked directly, was still binding,
raised to 1024.** Kalpit's question when this session started: "is 150
optimal?" Assumed at first (when the radius gate above was added) that it
would rarely bind now that the radius does the real scoping — checked that
assumption directly instead of trusting it, and it was wrong: at radius=150m
(5 sample windows across 3 trials), uncapped candidate node counts measured
**700-990**, meaning the 150 cap was silently discarding ~80% of the
locally-relevant road network the radius fix was specifically built to
include — a majority of it, not an edge case. Raised to 1024 (comfortable
headroom above the observed max) per Kalpit's explicit choice, after being
shown the tradeoff: `GraphEncoder`'s GCN layers do a dense `adj @ h @ weight`
matmul per graph in an unbatched per-sample loop, cost scaling ~N² — going
from 150 to ~900 real nodes is a real training-compute cost, not just a
config bump. The alternatives (coarsen `MIN_DIST_BETWEEN_NODES` to shrink
node count, or shrink `GRAPH_RADIUS_M` below what the horizon formula calls
for) were both on the table; Kalpit chose to accept the compute cost and
keep the radius/spacing as horizon-derived and resolution-faithful,
respectively, rather than compromise either for cheaper training. If
training throughput becomes a real bottleneck later, a sparse-adjacency
GCN implementation (the current dense `(N,N)` tensor is extremely wasteful
for what's structurally a sparse road-network graph) is the right fix to
revisit before re-shrinking the node budget.

Because graphs are no longer guaranteed to fill exactly `MAX_GRAPH_NODES`
real nodes, `GraphEncoder`'s attention pooling (§3) needed a padding mask —
added, mirroring `ObjectSetEncoder`'s existing masked-softmax pattern
(`dataset.py`'s `_build_graph_tensors` now also returns `node_mask`).

## 8. Topology-aware graph connectivity (done, not yet retrained)

`GraphBuilder.build_graph()` only ever added edges within a single
lanelet's own centerline chain — nothing connected different lanelets to
each other except a reactive repair step
(`_ensure_graph_connectivity`/`_connect_closest_components`) that bridged
whatever two nodes were globally nearest across disconnected fragments, with
**no distance cap at all**: `connection_threshold` was accepted as a
constructor argument and stored, but never actually read anywhere in the
class. Fixed two ways:

1. **Real lanelet2 routing relations** (`_connect_lanelet_topology`,
   new): successor/predecessor edges connect a lanelet's last node to its
   following lanelet's first node; adjacent-left/right edges connect
   parallel lanelets' first-to-first and last-to-last nodes — using the
   `lanelet2.routing.RoutingGraph` already needed for
   `LaneletAdjacencyChecker`, now built once in `SequenceBuilder.__init__`
   and shared between both instead of each constructing its own copy.
2. **`connection_threshold` now actually enforced** as a distance cap on
   whatever reactive bridging remains (logged, not silently applied, when
   exceeded) — genuinely rare after (1), not the primary mechanism anymore.

Confirmed empirically that (1) does most of the real work: on one sampled
window, 95 radius-filtered candidate lanelets (only 7 on-route — the radius
fix from §7 working as intended) produced 4 connected components from
topology edges alone, the largest holding 108 of ~150 nodes — the remaining
small fragments are dangling appendages whose only path back to the main
cluster exits the radius window, a structural consequence of any bounded
neighborhood query, not a bug.

## 9. Extraction runtime fix (done, measured)

Kalpit's question: does `st_gat.residuals`'s ~1-hour total runtime make
sense for the actual computation needed? Answer, from direct profiling
(one representative trial, `tl_fault_s3/goal_007`, ~1600-1900 frames): **no
— `SequenceBuilder.build()` took 106.4s** for a workload that should cost a
few seconds, root-caused to concrete, fixable causes rather than "this is
just what it costs":

| Cause | Share of 106.4s | Fix |
|---|---|---|
| `_connect_closest_components`'s brute-force O(n₁·n₂) search across every pair of graph fragments, on every rebuild | 60% (63.6s) | Real lanelet2 routing-relation edges first (§8) so this rarely fires; when it does, a KD-tree nearest-fragment query replaces the all-pairs search (§8) |
| `LaneletAdjacencyChecker.query()`'s linear scan over 979 lanelet centroids, called once per (window, frame) pair — 90,360 calls for 1,616 unique frames (56x redundant, from `STRIDE=1`) | 23% (24.6s) | `scipy.spatial.cKDTree`, built once; per-frame results memoized once per unique frame instead of once per overlapping window (enabled by `traffic_light_detected`'s retirement removing the last per-window-graph dependency from per-frame feature computation) |
| `copy.deepcopy(G_cached)` on every window instead of every graph rebuild | 6% (6.3s) | Deepcopy once per rebuild, reuse the same object reference for every window sharing that rebuild period (safe — nothing downstream mutates a sequence's graph in place) |

**Measured after all three fixes, same trial: 106.4s → 2.26s (47x).** Applies
identically to `run_pipeline.py`'s training-extraction path, not just
`residuals.py` — both call the same `SequenceBuilder.build()`.

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
- Sparse-adjacency GCN — `GraphEncoder`'s dense `(N,N)` matmul is wasteful
  for a structurally sparse road-network graph, now more relevant with
  `MAX_GRAPH_NODES=1024` (§7/§10) than it was at 150.
- **State-representation fidelity (`MAX_GRAPH_NODES`) as its own studied
  variable** (Kalpit, 2026-08-05) — not just a one-time "raise it and move
  on" fix. Once detection/lead-time/calibration results exist against the
  1024-node graph, worth sweeping node budget (e.g. 150 / 300 / 600 / 1024)
  against those same metrics to see whether more state fidelity actually
  buys anything past some point, or whether the compute cost from §7/§10 is
  paying for representational capacity the model doesn't end up using — a
  real ablation, not just the current single before/after data point (which
  only measured the extraction-side node-count truncation, not any
  downstream effect on detection/calibration). Deliberately deferred: this
  needs a working, trained baseline to compare against first.
