# CLAUDE.md

Guidance for Claude Code (or any agent) working in this repo.

## What this is

Kalpit Vadnerkar's PhD dissertation project (Clemson ECE, advisor Pierluigi
Pisu). Runs on Autoware + AWSIM in a simulated Nishishinjuku (Tokyo) map.
Builds on Vadnerkar & Pisu, "Digital Twins as Predictive Models for
Real-Time Probabilistic Risk Assessment of Autonomous Vehicles," IEEE
T-ITS vol. 27 no. 4, April 2026 (PDF in project root) — that paper trained
an ST-GAT purely on nominal driving data and detected Camera/IMU/LiDAR
faults via prediction residuals + PCA + Random Forest (93.7% accuracy,
Traffic Light Status Flag the single most discriminative feature). This
repo is new work building on that result, decoupled from its reference
implementation (`../Graph-Scene-Representation-and-Prediction/`, read-only,
cited context only, not a dependency).

**Repo name is a legacy artifact.** Originally scoped as "RISE" (Residual-
Informed Safety Envelopes — active control, relaxing velocity constraints
under uncertainty). That framing is gone; see the decision log.

## Current claim and architecture

The question this project answers is **not** "which known fault is this"
(closed-set classification — needs fault data, a fixed bin set, and says
nothing about a failure mode outside that set) but **"how far off-nominal
am I right now, in what way, and what does that imply about whether I'm
about to do something unsafe."** Three layers:

- **Layer 1 — detect the unknown.** A continuous, distribution-free anomaly
  measure: ST-GAT prediction residuals as conformal nonconformity scores.
  No fault data required to validate. **Built and validated on nominal
  data**; checked against real injected faults with a real, useful (if
  partial) result — see "Current state" below.
- **Layer 2 — consequence estimation (THE thesis).** Map anomaly to actual
  risk by rolling counterfactual futures forward through the digital twin —
  same residual signal, asking "so what happens" instead of "which bin."
  **Not yet built** in its current intended form (a reachability-based
  design) — a v1 prototype using a simpler static margin exists but is
  paused/superseded, see decision log.
- **Layer 3 — graceful response.** Continuous behavior modulation
  proportional to quantified ignorance, not limp-mode/ODD binning. Whether
  this pulls the descoped active-control (RISE) work back into scope is an
  **open question**, not resolved.

**Claim to defend:** a principled translation from "my model is uncertain
in this specific way, right now" to "therefore this specific driving
behavior is unsafe, by this much" — reported with calibrated confidence (a
reliability diagram, not just a coverage number), **decomposed** by
subsystem/uncertainty-source (not just a magnitude), giving measurable lead
time before a safety-margin violation, evaluated at matched false-alarm
rates. Novelty is specifically the synthesis of two traditions that don't
currently talk to each other: distribution-free statistical **calibration**
(Layer 1) and **reachability-style consequence estimation** (Layer 2, in
the shape of Johnson/Victor/Engström's "Field of Safe Motion," 2026) —
not an extension of either alone. A broader literature search to confirm
this synthesis claim actually holds up is still pending (see TODO.md).

**The project is being written up as two papers**, not one: Paper 1 is
Layer 1 (calibration/UQ) standalone; Paper 2 is Layer 2 building on it.
See `docs/research_notes/layer1_paper_structure_2026-08-25.md` for Paper
1's section-by-section structure and which comparisons are already run.

## Current state: built/validated vs. planned

**Built and validated:**
- Data collection + two-arm fault-injection infrastructure (TL + IMU
  faults, zone-gated); Arm A (safety disabled) exercised extensively, Arm B
  (safety enabled, ground-truth oracle) exists but is untested live.
- ST-GAT point-predictor training pipeline (14-feature vector), promoted
  canonical checkpoint at `st_gat/models/h30_30/st_gat_rise.pth` (currently
  the "v2," zone-weighted retrain — gated through `promote_model.py`, see
  below).
- Layer 1: leave-one-trial-out cross-conformal calibration
  (`experiments/scripts/conformal_horizon_calibration.py`), a systematic
  per-scenario coverage audit, a training-level fix for the scenario gap it
  found, and two conditional-calibration variants (Mondrian and embedding
  k-NN) — see decision log for what each found.
- Layer 1 checked against real injected faults across all 8 fault campaigns
  and all 7 feature series (`experiments/scripts/inspect_fault_predictions.py`).
- `experiments/scripts/promote_model.py` — gates every model swap (refuses
  a regression against the worst-performing scenario category, not just
  the average; auto-regenerates the canonical calibration report on
  promotion).

**Planned / pending, in rough priority order:**
1. TL fault severity sweep — a concrete, ready-to-run lab-session plan
   exists (`docs/research_notes/tl_severity_sweep_lab_plan_2026-08-26.md`)
   but has **not been run yet** (verified: no `tl_fault_fixed_*` data on
   disk, no new nominal trials at goal_007/012 since 2026-07-22). This is
   Paper 1's last missing result (comparison C6).
2. More nominal calibration data generally (only 7 trials currently in
   `CAL_DIR` — see limitations below).
3. Layer 2, rebuilt around forward reachability sets rather than the
   paused static-margin v1 prototype.
4. A broader (non-Waymo-only) literature review to confirm the
   calibration-x-reachability novelty claim before writing it up.
5. Layer 3 scoping decision (does active control come back into the core
   claim, or stay at "define the signal's shape").
6. Architecture ablation backlog (P1.6 in TODO.md) — several model
   hyperparameters were set by judgment call, not measured.

## Decision log

Each entry: what was decided and the load-bearing *why*. Full narrative
arcs are in `docs/research_notes/` (pointed to per entry) — don't re-derive
these from scratch, and don't re-litigate a settled one without reading its
note first.

- **Closed-set fault classification → open-world anomaly detection
  (2026-08-06, sharpened 2026-08-20).** Stopped optimizing toward
  "detect and classify a specific fault type" — collapsing a fault into a
  named bin only licenses a pre-planned canned response and requires fault
  data to validate at all; it also says nothing about a novel failure mode.
  Replaced with the continuous, distribution-free "how far off-nominal, so
  what" framing above. The old "belief-divergence / negative-evidence"
  claim and its Priority-0 mechanism experiments (per-fault-class feature
  importance, HD-map ablation) were retired; the HD-map question survives
  in a different, continuous form as a Layer-2 prerequisite. See
  `docs/research_notes/open_world_safety_reframe_2026-08-20.md`.
- **Jointly-trained distributional (Student-t/NLL) heads → conformal
  calibration on a plain point predictor (2026-08-20).** A multi-week arc
  trying to get the model's own mean+scale+dof heads calibrated fixed two
  real bugs (a shared-gradient-contention bug in the horizon-step
  embedding; a dof-collapse pathology) but never converged — this is a
  known pathology in jointly training a network to predict a value *and*
  self-report its own confidence (Wong-Toi et al., UAI 2023), not a bug
  fixable by one more regularizer. Conformal calibration wrapped around a
  point predictor sidesteps it entirely: no distribution-family assumption
  (also fixes traffic-light heads' non-Gaussian, quasi-categorical shape
  for free), horizon-widening comes from the empirical residual
  distribution by construction. Decisively confirmed by a direct
  side-by-side: coverage max-gap 0.066–0.376 (NLL heads, KS-rejects
  calibration for every feature) vs. 0.004–0.011 (conformal), for all 7
  series. NLL machinery is not deleted, just paused — see
  `docs/research_notes/nll_calibration_arc_and_conformal_pivot_2026-08-20.md`
  §5 for the concrete next step if it's ever resumed.
- **Leave-one-trial-out cross-conformal, not a fixed split.** Only 7
  nominal trials exist in the calibration pool; a first attempt at a fixed
  50/50 split leaked (windows are stride-1 overlapping, so "test" windows
  were near-duplicates of "fit" windows from the same trial) and, once
  fixed to a proper trial-level split, showed real per-trial instability a
  naive split's number hid. LOO-CV pools every trial's out-of-fold coverage
  into one number while keeping every check point genuinely held out — see
  the pivot note above, §4.
- **Pooled coverage is not sufficient evidence of calibration — must audit
  per-scenario, using the same real zone geometry fault injection uses
  (2026-08-24).** Pooled steering coverage read ~90%, but a geometry-
  grounded audit (`experiments/scripts/audit_minority_scenarios.py`, using
  the actual turn/TL zone files `fault_injector.py` gates on) found it was
  only 59–76% specifically on turns and near intersections — exactly where
  faults are injected. A pooled/aggregate number can hide a systematic gap
  via cancellation. Re-run this audit any time the model or calibration set
  changes.
- **Fix the scenario gap at the training level (zone-weighted resampling),
  not by widening the calibrated interval at inference (2026-08-25).**
  Widening on turns/intersections to stay valid would cost detection
  sensitivity exactly where faults are injected — calibration validity and
  detection power are in tension, not the same objective. Data-driven
  inverse-frequency resampling (no hand-tuned boost constants,
  `scenario_zones.compute_train_sample_weights`) closed the gap
  (tl_zones steering coverage 75.6% → 87.3%) and is now the canonical (v2)
  model, gated through `promote_model.py`.
- **Both discrete (Mondrian) and continuous (embedding k-NN) conditional
  calibration are kept — neither dominates (2026-08-25).** Mondrian
  (Vovk 2012, 4 real zone-derived groups) hits ~90% per group but is NOT
  uniformly tighter once weighted by group prevalence — steering and
  TL-confidence are net *wider* on average, the real statistical cost of
  fitting a quantile on a fraction of the data. The continuous approach
  (k-NN in the model's own learned scene embedding `h_last`) improves
  automatically as the underlying model improves with zero recalibration
  changes, and can tighten a feature where global calibration was already
  fine (something a fixed discrete partition can't do) — but is noisier at
  small k and non-monotonic in k. Report both honestly in the paper, don't
  present one as a strict win.
- **Layer 2 pivots from a static lane-boundary margin to forward-
  reachability-style consequence estimation (2026-08-24).** An independent
  literature review found Waymo's own published safety research ("Field of
  Safe Motion," Johnson/Victor/Engström 2026) already does forward-
  propagated, map-grounded, decomposed consequence estimation via
  reachable sets — but without this project's calibrated, data-driven
  uncertainty. A static lane-margin design (`experiments/lib/margin.py`,
  `experiments/scripts/layer2_consequence_estimation.py`) was built and run
  once (2026-08-21, nominal-only, pre-dating this pivot) but is now a
  paused v1 prototype, not the design to build on — see decision log entry
  above on the two-paper split and `open_world_safety_reframe_2026-08-20.md`
  §9(c).
- **Multi-feature fault check, not position-only (2026-08-25).** Checking
  all 7 series against real fault campaigns found position is NOT the most
  diagnostic feature for either fault family: acceleration's active/clean
  exceed-rate ratio is 13.66x for IMU faults (vs. position's 1.66x);
  velocity_lateral/longitudinal beat the directly-spoofed TL-color/
  confidence features for TL faults. A Layer-2 margin built only on
  predicted position (the original plan) would miss the sharpest available
  fault evidence — decomposition by feature is a real, useful signal here,
  not "structurally vacuous" as earlier judged against the old design.
- **Two-arm fault-injection design kept from the earlier framing
  (2026-07-24).** Arm A (Autoware's built-in safety features disabled) lets
  an injected fault propagate to a full healthy→degraded→dangerous
  trajectory; Arm B (safety features enabled, stock diagnostic gate) is the
  ground-truth oracle — Autoware's own MRM trigger is treated as a labeled
  "this was objectively unsafe" moment, and lead time is measured against
  it. This mechanic is unaffected by the open-world reframe.
- **Active control (RISE) descoped, then reopened as an unresolved
  question.** Scoped out entirely 2026-07-24 (control/handling
  contribution, not safety-verification evidence — no new staged-avoidance
  work). The 2026-08-20 reframe's Layer 3 ("graceful response") reopens
  whether some form of it belongs back in the core claim; not yet decided.
- **TL severity sweep uses discrete fixed severities (0.3/0.5/0.7), not the
  existing ramp (2026-08-26).** Confirmed directly: `tl_fault_ramp`
  confounds severity with elapsed-time-in-zone/physical proximity to the
  intersection, so a dose-response curve built on it can't cleanly
  attribute effect to severity alone. Fixed-severity trials plus matched
  same-goal nominal controls isolate severity from that confound. See
  `docs/research_notes/tl_severity_sweep_lab_plan_2026-08-26.md`.

## Known gotchas (hit and fixed this repo's history — don't reintroduce)

- **Lanelet2 map projection:** always use
  `autoware_lanelet2_extension_python.projection.MGRSProjector(lanelet2.io.Origin(0.0, 0.0))`
  to load `Map/nishishinjuku_autoware_map/lanelet2_map.osm`. A generic
  projector with a guessed lat/lon origin does **not** line up with the
  AWSIM/ROS2 map frame (off by 1000s–10000s of meters) — this is what
  Autoware's own `autoware_map_projection_loader` uses internally.
  `experiments/scripts/plot_routes.py`/`explore_map.py`/`margin.py` are
  reference-correct examples.
- **Background ROS processes:** `$!` after `cmd &` is unreliable in this
  harness. Always verify with `pgrep -af <script_name>` before trusting a
  PID to kill, and confirm it's actually gone before starting a
  replacement — two uncoordinated listeners on the same ROS topics
  produces garbled output that looks like a sensor bug but isn't.
- **`autoware_pose_instability_detector_node`** can crash (`cannot store a
  negative time point in rclcpp::Time`) and not auto-restart for the rest
  of a session. Doesn't corrupt collected data, just silently drops that
  one monitoring signal — worth a `ros2 node list | grep
  pose_instability_detector` check before trusting a long session.
- Params under `config/**/*.param.yaml` are **load-time, not
  hot-reloadable** — Autoware needs a restart after editing; verify with
  `ros2 param get` that the new value actually loaded.
- **MRM diagnostic gate is toggleable between Arm A and Arm B.**
  `experiments/scripts/switch_diagnostic_arm.sh {A,B}` swaps config files
  under `autoware/src/launcher/autoware_launch/.../config/system/
  diagnostics/` (each arm's variant kept alongside as `.armA.yaml`/
  `.armB.yaml`); it only copies files, it does **not** restart Autoware —
  these are load-time params, restart yourself for a switch to take
  effect. `collect.sh`/`run_fault_campaigns.sh` take `--arm A|B` (default
  A) and refuse to run if that doesn't match the switch script's
  `.active_arm` marker. Arm B is untested against this repo's actual
  fault-injection workflow — it's exactly the config that caused the
  original MRM deadlocks Arm A was built to route around, watch for that
  if Arm B collection ever runs.
- **Before any new TL or IMU fault campaign**, regenerate the zone files
  the fault injector gates on whenever the goal set/map/route changes:
  `experiments/scripts/compute_turn_zones.py` (turn/bias-lead-in zones) and
  `experiments/scripts/compute_tl_zones.py` (TL zones, keyed by real
  `traffic_light_group_id` so a fault only targets the light the vehicle is
  actually governed by). If `tl_zones.json` is missing/stale for a goal, TL
  faults silently fall back to mutating every group instead of the scoped
  one. `experiments/scripts/plot_fault_plan.py --campaign <name>` is a fast
  visual sanity check of the gating zones before a full run. Both zone
  files carry a `reachable` field — informational only, **not a filter**:
  `fault_injector.py` loads every zone regardless (the runway/arming state
  machine already handles unreachable ones); only plotting/reporting should
  filter by it, or runway-clear detection silently breaks.
- **Before trusting newly-collected data**, `result.json`'s `status` field
  isn't enough (`goal_reached`/`stuck` can both hide real problems). Run in
  order: `analyse_experiments.py --campaign <name>` (fast JSON-only sanity
  check), `analyze_mrm_diagnostics.py --batch <dir>` (needs ROS; whole-trial
  diagnostic audit, catches silent localization stalls that never trigger
  MRM), `plot_routes.py` (needs ROS; visual check). For fault campaigns
  specifically also run `compare_fault_vs_nominal.py` (confirms the fault
  actually changed the targeted signal, ranks which ST-GAT features respond
  most) and the matching `verify_imu_zone_arming.py`/`verify_tl_zone_arming.py`
  (independently re-checks zone-arming/group-id targeting from
  `fault_log.jsonl` alone, no ROS needed; both skip pre-zone-based-arming
  data automatically). A trial with `mrm_trigger_count: 0` is not
  automatically clean — that only reflects Autoware's own MRM state
  machine, not whether the vehicle actually moved.
- **A units/scaling bug class has bitten this project more than once** —
  watch for it: (1) the uncertainty-disc radius in
  `plot_layer1_trust_examples.py` was once passed straight from a
  *normalized* conformal quantile without multiplying by
  `POSITION_DISPLACEMENT_RANGE_M`, making circles ~100x too small against
  real map coordinates (fixed); (2) pooling PIT (probability-integral-
  transform) values across multiple feature dimensions must **flatten**,
  not average — averaging two independent Uniform(0,1) values is not
  itself Uniform(0,1), so an averaged pooled-PIT plot silently misrepresents
  calibration. Any new plot/metric that combines a per-feature or
  per-dimension statistic into one number should be checked against this
  class of bug before trusting it.
- **Concurrent multi-process model training on this machine requires
  `--workers 0`.** Fork-based PyTorch DataLoader workers combined with this
  project's graph-heavy per-sequence dataset multiply host RAM sharply
  enough that 5 concurrent training processes at `--workers 2` were killed
  by `earlyoom` within ~20 seconds, before any checkpoint saved. Relevant
  any time more than one training run is launched at once.
- **"Camera fault" in this repo can only mean a traffic-light-detection
  fault.** Verified directly against this Autoware install: object
  detection is LiDAR-only (`perception_mode: lidar`), and the sensor kit
  has exactly one camera, feeding traffic-light recognition only. Don't
  build a new "camera fault" that assumes a general camera-based object
  detector exists — it doesn't.
- **Fault-injection topic wiring can silently no-op.** A prior TL fault
  campaign ran with zero effect (`msg_count_tl: 0` in every trial's
  `fault_log.jsonl`) because `fault_injector.py` was publishing to a topic
  name Autoware's launch files didn't actually subscribe to downstream —
  the injection code ran fine, logged normally, and simply never reached
  the vehicle. If a fault type or topic wiring is ever changed, verify with
  `compare_fault_vs_nominal.py` (or a direct topic echo) that the fault
  measurably changed the targeted signal, don't trust `fault_log.jsonl`
  alone.
- **A feature-vector or architecture change invalidates existing
  checkpoints silently unless you check.** The 13→14 feature change
  (adding `traffic_light_discrepancy`) and the per-head horizon-embedding
  fix both changed the model's input/state-dict shape — an old checkpoint
  loads (or half-loads) without necessarily erroring loudly. `config.py`
  flags stale checkpoints, but confirm dimensionality matches before
  trusting a loaded model's output.

## Environment — sourcing order matters

```bash
source /opt/ros/humble/setup.bash
source /home/kvadner/Desktop/Dissertation/autoware/install/setup.bash
source .venv/bin/activate   # only needed for lanelet2/rosbag2_py/torch work
```
ROS + Autoware **before** the venv (the venv is `--system-site-packages`, built so
rclpy/rosbag2_py/lanelet2 stay visible — activating it first breaks that resolution).

**Never launch AWSIM or Autoware via the Bash tool.** They're long-running,
GUI-adjacent Unity/ROS processes that need one-time interactive `sudo sysctl` setup
with no TTY available here. Confirm scripts parse (`bash -n`) and hand off to the user
to actually run `Run_AWSIM.sh` / `Run_Autoware_Headless.sh` themselves.

## Directory conventions

- `experiments/scripts/` — every one-off/manual Python and shell tool lives here (not
  repo root — keep it that way when adding new scripts).
- `experiments/lib/` — shared library code imported by multiple scripts, not
  runnable on its own: `fault_injector.py`, `metrics.py`, `plotting.py`
  (map loading/rendering, outcome styling, zone-drawing, divergence-trace
  panels — shared by the plot_*.py scripts, reuse it rather than
  copy-pasting a helper), `margin.py` (Layer-2 safety-margin primitives,
  currently the paused static-lane-margin design), `scenario_zones.py`
  (zone-based sample weighting for training).
- `experiments/analysis/` — output artifacts from analysis scripts. Not raw
  experiment data.
- `experiments/data/<campaign>/<goal_id>/t<N>_<timestamp>/` — one dir per
  trial (`result.json`, `metadata.json`, `metrics.json`, `fault_log.jsonl`,
  `rosbag/`); campaign-level batch summaries live in `<campaign>/_meta/`.
- `experiments/configs/captured_goals.json` — the **operative** goal set
  (26 goals, edited in place; `captured_goals_original.json` is a frozen
  historical snapshot, never edited). `capture_goals_session.py` always
  **overwrites** `captured_goals.json` with only that session's captures —
  back the file up first, then hand-merge new entries by ID.
- `st_gat/` — the model. `st_gat/pipeline/` (extraction: `bag_reader.py`,
  `sequence_builder.py`, `run_pipeline.py`), `st_gat/model/` (architecture,
  loss, trainer), `st_gat/train.py` (`--warmup-only` = the plain point
  predictor Layer 1 wraps; `--zone-weighted-sampling` = the v2 training
  fix), `st_gat/residuals.py` (per-timestep residual trace generation),
  `st_gat/checkpoints/<run>/` (training checkpoints) vs.
  `st_gat/models/<tag>/st_gat_rise.pth` (deployed/canonical, gated by
  `promote_model.py`).

## Trusting the model itself (before trusting any Layer 1 result)

`./experiments/scripts/run_calibration_pipeline.sh` runs the full Layer-1
result set in one command (repo venv only, no ROS needed).

**Primary check**: `experiments/scripts/conformal_horizon_calibration.py`
— leave-one-trial-out cross-conformal per feature-series per horizon step,
wrapped around the plain point predictor. Reports pooled + per-fold-spread
coverage, `conformal_vs_actual.png`, and `reliability_diagram.png`.
**Read the pooled number carefully** — see decision log's per-scenario-audit
entry; also run `audit_minority_scenarios.py` before trusting a pooled
number in isolation. Conditional-calibration variants:
`conformal_mondrian_calibration.py` (discrete, zone-grouped),
`conformal_embedding_calibration.py`/`conformal_scene_conditioning.py`
(continuous, k-NN in the model's `h_last` scene embedding) — see decision
log for the honest trade-offs between them.

The paused NLL/Student-t diagnostics (`plot_calibration_diagrams.py`,
`plot_sprt_signal_behavior.py`) are still real, working tools if that arc
or the old SPRT sequential-evidence signal is ever resumed, but are not
part of the current critical path — see
`docs/research_notes/nll_calibration_arc_and_conformal_pivot_2026-08-20.md`.

## Real, current limitations (qualify any claim made right now)

- **Only 7 trials in the nominal calibration set.** The pooled/aggregate
  coverage claim is solid; a claim about any single new drive's realized
  coverage is not, without more nominal data. This is flagged as
  load-bearing-weak by three independent sources (this project's own
  notes, the reframe note, and an independent novelty-check agent) — not
  optional polish, a real scope limit on what's currently defensible.
- **`lane_change_zones` has essentially zero matching windows** in the
  current dataset — a data gap, not a calibration gap.
- **`open_road` (44% of the data) under-covers unexpectedly** (85–86% on
  position/velocity) despite being the "easy" majority case — not yet
  investigated.
- **TL-fault detection is confounded with intersection difficulty**: TL
  faults are gated to fire only at real intersections, already the
  hardest nominal scenario, so elevated residual near a TL fault can't yet
  be cleanly attributed to "the fault" vs. "the intersection." The severity
  sweep's matched same-goal nominal controls are the fix, not yet run.
- **IMU-fault consequence keeps growing past 120s post-fault** (compounding
  EKF state-estimation corruption that doesn't self-heal) — Layer 2's
  intended ~3s rollout horizon is plausibly well-matched to TL faults but
  structurally too short to see a slow-compounding IMU fault coming.
- **Epistemic disagreement (independently-trained point predictors) only
  widens with horizon for 2 of 7 series** (position, partially
  acceleration) — not yet reassuring evidence that the model's behavior
  stays trustworthy under real out-of-distribution/fault shift, though not
  yet a blocking problem either (no fault data has been run through this
  check yet, only nominal).
- **`margin.py`'s static lane-boundary distance has a known, unfixed
  nearest-lanelet-boundary edge case** near lane-change boundaries — moot
  if Layer 2 fully moves to the reachability design, but be aware if
  anything still imports it.

## Don't duplicate context that's already written down

- `TODO.md` — current task list, read fresh every session.
- `README.md` — environment setup, Autoware source patches to reapply after
  any reinstall/rebuild, data collection campaign commands.
- `docs/theoretical_framework.md` — the two-arm fault-injection design and
  divergence-trace schema (still applies); its "belief divergence" claim
  language is superseded by the open-world reframe above.
- `docs/research_notes/` — dated findings docs, one per topic/session. Check
  the relevant one for the *why* behind a non-obvious decision before
  re-deriving it; this file only carries the compressed conclusion.

## Working style notes

- This repo moves fast — re-read `TODO.md` at the start of a session rather
  than trusting memory of a prior conversation.
- Kalpit runs AWSIM/Autoware experiment collection manually and reports
  back; don't assume a background agent can drive that part.
