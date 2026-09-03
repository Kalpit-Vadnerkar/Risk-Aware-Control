# Risk-Aware Control — Task List

**Last updated:** 2026-09-03. For the current architecture, claims, decision
log, and gotchas, read `CLAUDE.md` first — this file is only the actionable
task list. Retired framings (closed-set fault classification, "belief
divergence," the original active-control/RISE plan) are not reproduced here
— see `CLAUDE.md`'s decision log and `docs/research_notes/
open_world_safety_reframe_2026-08-20.md` if you need that history; git
history has every prior version of this file if more detail is ever needed.

---

## 1. Next lab session — TL severity sweep (ready to run)

A concrete, exact-commands runbook already exists:
`docs/research_notes/tl_severity_sweep_lab_plan_2026-08-26.md`. **Not yet
run** — verified 2026-09-03: no `tl_fault_fixed_*` data on disk, no new
nominal trials at goal_007/012 since 2026-07-22. This is Paper 1's last
missing result (comparison C6 in `docs/research_notes/
layer1_paper_structure_2026-08-25.md`).

Requires AWSIM + Autoware running (see `README.md`'s startup section) —
Kalpit runs this manually.

- [ ] `./collect.sh nom_v11 --goals goal_007,goal_012,goal_026 --trials 2` —
      6 new nominal trials (also expands the load-bearing-thin 7-trial
      calibration set at exactly the two goals that are currently
      under-represented in `CAL_DIR`).
- [ ] `./run_fault_campaigns.sh --goals goal_007,goal_012,goal_026 --trials 2
      --campaigns "tl_fault_fixed_030 tl_fault_fixed_050 tl_fault_fixed_070"`
      — 18 new fault trials at 3 fixed severities.
- [ ] **Verify the new goal_007/012 nominal trials actually land in
      `CAL_DIR`, not `TRAIN_DIR`** — the per-goal 80/20 split does not
      guarantee this. Check
      `ls st_gat/data/h30_30/sequences/calibration/` against the new run
      dirs; if a new trial isn't there, manually move its symlink (see the
      lab plan's verification section for the exact procedure — this is a
      deliberate one-off override, not a pipeline change).
- [ ] Small code change needed before analysis: extend
      `tl_severity_sweep_analysis.py` to read severity directly from the
      new fixed-severity trials' `fault_log.jsonl`
      (`params.confidence_scale`) instead of only reconstructing it from
      `tl_fault_ramp`'s decay formula (a new branch, not a rewrite).
- [ ] Run `python3 -m st_gat.pipeline.run_pipeline --datasets nom_v11` then
      `python3 experiments/scripts/tl_severity_sweep_analysis.py`.
- [ ] Sanity check: goal_026's new curve should agree directionally with
      the 2026-08-25 ramp-based pilot's goal_026 result.

## 2. More nominal calibration data (ongoing, not just this session)

Only 7 trials currently in `CAL_DIR` — flagged load-bearing-weak by three
independent sources (see `CLAUDE.md` limitations). The severity-sweep
session above is a partial step (goal_007/012 specifically); still worth
collecting more nominal trials generally beyond that.

## 3. Layer 2 — rebuild around forward reachability

Not yet started in its intended form. Replace the paused static-margin v1
prototype (`experiments/lib/margin.py`,
`experiments/scripts/layer2_consequence_estimation.py`) with a
reachability-style margin (bounded reachable sets for tracked objects,
pruned by lane-containment) — see `CLAUDE.md`'s decision log and
`docs/research_notes/open_world_safety_reframe_2026-08-20.md` §9(c) for the
full design rationale. Depends on richer margin-violation logging
(continuous time-to-collision / lane-boundary-margin traces, not just
binary collision/stuck heuristics — `experiments/lib/metrics.py`'s current
`static_collision`/stuck logic is too rare/unreliable to build an
evaluation around).

- [ ] Design the reachable-set computation against real tracked-object data
      and `lanelet2.geometry.inside` lane-containment (not nearest-5
      lanelet heuristics).
- [ ] Add continuous margin-violation logging.
- [ ] Decompose along aleatoric (Layer 1 residual bootstrap) vs. epistemic
      (cross-member disagreement) contribution — the v1 prototype's
      position-only margin made a per-feature decomposition vacuous; this
      axis is more meaningful given the reachability margin's structure
      too, revisit whether per-feature decomposition becomes meaningful
      once the margin depends on more than position.
- [ ] Build the reliability diagram for P(violation within H) — needs real
      violation ground truth, which barely exists in nominal-only data by
      construction; likely needs fault-condition data run through it.

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
| Conformal target coverage | 90% (δ=0.10) | Headline is the reliability diagram, coverage is the secondary check |
| ST-GAT features | 14 | Includes `traffic_light_discrepancy`; retrain required after any further feature-vector change |
| Fault goals | goal_007, goal_012, goal_026 | Most TL-zone entries per trial |
| Canonical model | `st_gat/models/h30_30/st_gat_rise.pth` | v2 (zone-weighted retrain), gated via `promote_model.py` |
| Nominal calibration trials | 7 (goal-split `CAL_DIR`) | Load-bearing-thin, see `CLAUDE.md` limitations |
