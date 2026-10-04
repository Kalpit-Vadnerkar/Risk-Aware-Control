# Risk-Aware Control — Task List

**Last updated:** 2026-10-04. For the current architecture, claims, decision
log, and gotchas, read `CLAUDE.md` first — this file is only the actionable
task list. Retired framings (closed-set fault classification, "belief
divergence," the original active-control/RISE plan) are not reproduced here
— see `CLAUDE.md`'s decision log and `docs/research_notes/
open_world_safety_reframe_2026-08-20.md` if you need that history; git
history has every prior version of this file if more detail is ever needed.

---

## 1. TL severity sweep — DONE 2026-10-03/04

Ran to completion: 6 new nominal trials (goal_007/012/026, now 4/goal) +
18 fixed-severity fault trials (3 severities × 3 goals × 2 trials), all
`goal_reached`/`fault_validation.valid`, confidence_scale confirmed
0.7/0.5/0.3 for fixed_030/050/070 in each trial's own `fault_log.jsonl`.
goal_007/012 promoted to `CAL_DIR` via `manage_goal_split.py` (see
`CLAUDE.md`'s directory-conventions entry). `tl_severity_sweep_analysis.py`
run successfully — dose-response result saved to
`experiments/analysis/tl_severity_sweep/`.

**Real bug found and fixed in the analysis itself, not just the data**:
the script's "clean/held-out" check was `run_name in CAL_DIR_TRIALS`
(today's `CAL_DIR` membership) — but the deployed model
(`st_gat_rise.pth`, 2026-08-25) trained on goal_007/012's *original*
2026-07-22 trials back when they were still in `TRAIN_DIR`; promoting them
to `CAL_DIR` today doesn't retroactively un-train the model on them. Fixed
to the real rule (`_trial_is_clean()`): clean = goal_026 (never trained on,
ever) OR collected after the model's training cutoff. Before the fix,
`clean_only` and `all_including_contaminated` were byte-identical — a
red flag that got caught, not missed.

**Not done** (lower priority, revisit if time allows — not blocking
Layer 2 work): the TODO item "goal_026's new curve should agree
directionally with the 2026-08-25 ramp-based pilot's goal_026 result" —
the current analysis pools all 3 goals together, so this specific
goal_026-only comparison against the old pilot hasn't been isolated and
run separately.

**Operational notes from this session, for the next lab run**:
- AWSIM itself can get stuck (vehicle stays in Park, Autoware reports
  itself fully healthy — `DRIVING`/`AUTONOMOUS`/MRM `NORMAL`, NDT/LiDAR
  fine, velocity pinned at 0) in a way that restarting Autoware alone does
  NOT fix — restart AWSIM too (`./Run_AWSIM.sh`) when this happens, not
  just `Run_Autoware_Headless.sh`. `experiments/scripts/diagnose_system.py
  --continuous` is the fast way to tell "Autoware's own view of itself"
  from "is the vehicle actually moving."
- Added a hard cap (`--max-batch`, default 12) on `collect.sh`/
  `run_fault_campaigns.sh` so a batch can't silently chain past the known
  behavior_path_planner state-exhaustion threshold (README item 5,
  ~18-36 experiments) unattended — restart Autoware manually between
  batches instead of relying on auto-restart.
- `count_existing_trials()` (auto-resume) counts ANY trial with a
  `result.json`, regardless of outcome — a leftover `stuck`/`timeout`/
  `engage_failed` trial directory silently blocks that slot from ever
  being retried. Delete broken trial directories before re-running, don't
  just leave them.
- Across this whole sweep, every failure (6 total: `engage_failed`,
  `timeout` ×3, `stuck` ×2) was at `goal_026`specifically — goal_007/012
  never failed once. Not yet root-caused; two live hypotheses (a real
  goal_026 map/routing issue, vs. goal_026 simply always being listed last
  in `--goals` so it absorbs whatever degrades with trial count regardless
  of which goal it is) — worth a real test (run goal_026 first in a batch
  sometime) before trusting goal_026-specific results at face value.

## 2. More nominal calibration data (ongoing, not just this session)

Partially addressed 2026-10-03/04: goal_007/012 each went from 2→4 trials
and are now in `CAL_DIR` (16 trials there now, up from 7) — still worth
collecting more nominal trials generally beyond that, at other goals.

## 3. Layer 2 — a priori (2a) + a posteriori (2b) reachability-set optimization

**CURRENT PRIORITY, 2026-10-04**: concrete wiring plan written —
`docs/research_notes/layer2b_wiring_plan_2026-10-04.md`. Read that first,
this section is now just the compressed pointer to it.

**2a (a priori) — done, just relabeled.** Mondrian
(`conformal_mondrian_calibration.py`) and embedding k-NN
(`conformal_embedding_calibration.py`) conditional calibration already
shape the interval using zone/scene context before it's emitted — see
slide 12 / `CLAUDE.md` decision log.

**2b (a posteriori) — found to already mostly exist, 2026-10-04.** The
"not yet started" status on this item (every doc up to and including
2026-09-03) was an overstatement: `experiments/lib/margin.py`
(real `lanelet2.geometry` lane-boundary distance + conservative
object-clearance bound) and `experiments/scripts/
layer2_consequence_estimation.py` (bootstrap-residuals →
trim-against-margin → P(violation), already produces time-series traces)
were built and run once on nominal-only data 2026-08-21, then explicitly
paused awaiting real fault data — which now exists (TL severity sweep,
item 1 above, plus existing IMU campaigns). The wiring plan's actual scope
is narrower than a from-scratch build:
- [ ] Point `layer2_consequence_estimation.py` at a real fault trial for
      the first time (reuse `inspect_fault_predictions.py`'s
      `process_trial()` — same pattern `tl_severity_sweep_analysis.py`
      already uses for raw-bag fault processing, no extraction-pipeline
      changes needed).
- [ ] Wire 2a's conditioning (Mondrian AND embedding, build+compare both
      per Kalpit's call) into the bootstrap pool selection — currently
      draws uniformly from the whole calibration pool, which is the actual
      redundancy the advisor flagged.
- [ ] Produce the two target artifacts: an envelope-cascade overlay (raw
      pooled → Mondrian → embedding → lane/object-trimmed, one dramatic
      frame, real map) and a P(violation)-over-time trace with fault onset
      marked, for one real fault trial (candidate: `imu_fault_s3`, already
      has a real recorded lead-time result — pick the specific goal/trial
      empirically).
- [ ] Regression-check the existing nominal-only validation path still
      gives consistent numbers (shared code is being touched).

**Deferred, explicitly out of scope for the above** (see the wiring plan's
own "explicitly out of scope" section for why): consuming Autoware's own
`PredictedObjects.kinematics.predicted_paths` for a real (not worst-case
conservative) object reachable set; a full reliability diagram for
P(violation) (needs many fault trials, not one featured example); any new
data collection.

## 4. Broader literature review (before writing up Layer 2 as novel)

The existing lit-review pass was scoped to Waymo's own published safety
research only (reasonable first pass, is what surfaced the reachability
pivot) — not a substitute for a broader search across reachability
analysis, conformal-prediction-for-planning, and AV-safety-verification
generally. Needed to confirm the "calibration × reachability" synthesis
claim isn't already done elsewhere under different terminology. Scope this
before writing up Layer 2 as a contribution, not after.

## 5. Layer 3 scoping decision (open, needs Kalpit's call)

Does "graceful response" pull the previously-descoped active-control
(RISE) work back into the core claim, or does Layer 3 stay at "define the
shape the signal should take" without rebuilding an actual controller?
Not yet decided — see `CLAUDE.md` decision log.

## 6. Arm B live validation

Arm B (stock/full MRM diagnostic gate, the ground-truth oracle for
lead-time measurement) is built (`experiments/scripts/
switch_diagnostic_arm.sh B`) but has never actually run a fault campaign —
untested whether it reintroduces the MRM deadlocks (routing resets, TF
drops during teleports) that Arm A was built to route around. Needed before
any lead-time-vs-Arm-B result can be claimed.

## 7. Architecture ablation backlog (P1.6 — lower priority, revisit once 1-4 land)

Several `st_gat/model/model.py` architecture choices were set by judgment
call, not measurement. Tracked with results so far in
`docs/research_notes/ablation_study_2026.md` — check that file before
re-running any of these, several already have a done/measured verdict:

- [x] Frame-gap contiguity gate, route-aware graph node selection,
      attention-weighted pooling, TL-discrepancy temperature scaling,
      `MAX_GRAPH_NODES` (raised 150→1024, direct measurement showed 150 was
      still discarding ~80% of in-radius nodes) — all done, see the
      ablation study doc for numbers.
- [ ] Graph cadence: `graph_ctx` is pooled once per window (static across
      all 30 input timesteps) — measure whether finer re-pooling cadence
      improves tracking or fault-reaction lead time.
- [ ] Capacity sweep: `d_model`/`d_graph`/`hidden_size`/`num_layers`/
      `nhead` (currently 128/128/128/2/4, never revisited).
- [ ] Object-set encoder: mean-pool vs. attention-pool for tracked objects.
- [ ] Sparse-adjacency GCN — more relevant now than at the old 150-node cap.
- [ ] `MAX_GRAPH_NODES` itself as a studied variable (150/300/600/1024
      against detection/calibration/lead-time results, not just the
      extraction-side node-count fix already measured).

## Explicitly out of scope

- **New staged-avoidance/obstacle-scenario experiments** (`obs_*`
  campaigns) — control/handling contribution, not safety-verification
  evidence. Existing infrastructure/data kept as illustrative context.
- **New LiDAR fault data collection in this repo** — out of scope; the
  published T-ITS paper's own LiDAR fault data is the reference if ever
  needed for a comparison.
- **Multiple velocity levels / per-speed calibration** — all experiments
  run at max map-limit velocity (11.11 m/s) only.
- **Rewriting or re-litigating the closed-set/"belief divergence" framing**
  — retired, see `CLAUDE.md` decision log; don't resurrect Priority-0-style
  mechanism experiments without a new, explicit reason.

## Key parameters (current)

| Parameter | Value | Notes |
|---|---|---|
| Velocity | 11.11 m/s (map limit) | Single operating condition, all experiments |
| NPC density | 10 (AWSIM GUI slider) | Single condition, all data collected so far (confirmed 2026-10-04) — config-driven override now exists (see `CLAUDE.md`'s AWSIM gotcha) but not yet used; plan is to keep it at 10 for now and collect a higher-density round later, once Layer 2b exists to evaluate it |
| Conformal target coverage | 90% (δ=0.10) | Headline is the reliability diagram, coverage is the secondary check |
| ST-GAT features | 14 | Includes `traffic_light_discrepancy`; retrain required after any further feature-vector change |
| Fault goals | goal_007, goal_012, goal_026 | Most TL-zone entries per trial |
| Canonical model | `st_gat/models/h30_30/st_gat_rise.pth` | v2 (zone-weighted retrain), gated via `promote_model.py` |
| Nominal calibration trials | 17 (goal-split `CAL_DIR`, up from 7 as of 2026-10-03) | goal_007/012 added this session; still worth more at other goals, see `CLAUDE.md` limitations |
