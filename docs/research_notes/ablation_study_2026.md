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
| 7 | `MAX_GRAPH_NODES=150` sweep | model architecture | **TBD** | — | — |

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
