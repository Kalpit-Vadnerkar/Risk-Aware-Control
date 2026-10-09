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
under uncertainty). That framing is gone; see `docs/findings.md` §6.

## Where things stand — read these first

The current state lives in three short documents; this file only carries
operational guidance.

- `docs/architecture.md` — the pipeline design (Layer 1 multi-source
  predictor + conformal scores giving when / how much / which way; Layer 2
  nominal envelope, evidence-driven growth, safe sets; Layer 3 hook).
  Shareable version with diagram:
  https://claude.ai/code/artifact/1cc568ab-f9b5-4945-afe4-9cf1c9f55a65
- `docs/findings.md` — everything measured so far, with numbers, plus a
  table of superseded ideas not to revive.
- `docs/experiment_plan.md` — what to collect next and open decisions.
- `TODO.md` — the task list.
- `docs/archive/` — historical dated notes; not maintained, often
  contradicted by the current design. Read only to trace why something was
  decided.

When a finding changes, edit `docs/findings.md` (and `architecture.md` if
the design moved) instead of writing a new dated note.

**Paper split:** Paper 1 = Layer 1 (calibration / UQ) standalone, Paper 2 =
Layer 2 on top of it (`docs/archive/research_notes/layer1_paper_structure_2026-08-25.md`).

**Status in one line each (2026-10-09):**
- Canonical model is still bbox_v1 (`st_gat/models/h30_30/`); Layer 2 work
  uses ego_v6 (`h30_30_ego6/`, multi-source + planar pose consistency), not
  promoted. Run with `STGAT_GRAPH_FRAME=ego_v6`.
- Layer 2 method = `experiments/scripts/layer2_envelope.py`: catches 4/4
  real IMU departures (2.0-3.4 s lead), harmless IMU faults in alarm 0-18%
  of fault time (`findings.md` §4). 2b uses the formal-model pose-error filter. Next: closed-loop comparison (Autoware / +L1 / +L2 / +L3 correction), see `presentation/STORY.md`.
- Work happens on branch `layer2b-ego-frame` (not merged, not pushed).

## Standing rules learned the hard way

- **Calibrate leave-one-trial-out**, never a per-window split (windows
  overlap 59/60 frames).
- **Never trust a pooled coverage number alone** — run
  `audit_minority_scenarios.py`; pooled 0.90 has hidden 0.59–0.80 in turns.
- **"Held out" = the deployed model never trained on it**, anchored to its
  training date, not today's `CAL_DIR` membership.
- **Every safe set must pass the realized-nominal test** (`layer2 --mode
  check-safe-set`): real nominal futures must almost never violate it.
- **Report every fault campaign** in result tables; never silently drop one.
- **Behavioral residuals tell when, not how much** — don't size an envelope
  from them alone.
- **No physics trim** in Layer 2 (`--trim` exists for comparison only).
- **Train with `--selection kinematic`** (normalized over position, velocity,
  steering, acceleration, consistency); any criterion that includes the
  traffic-light heads lets their early overfitting pick the epoch.
- **Lead time = onset of the alarm episode still active at the violation**,
  thresholds at a matched nominal false-alarm rate.

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

- **AWSIM NPC density is now config-driven, not a Unity-rebuild-required
  hardcoded value** — a prior assessment (memory, corrected 2026-09-03)
  overstated this. `Assets/AWSIM/Scripts/Loader/{AWSIMConfiguration,
  SimulationManager,SimConfiguration}.cs` in `../Awsin-Source-code/
  AWSIM-Labs-1.6.1/` (vendored AWSIM source, sibling to this repo, not
  git-tracked) were patched to read a `targetVehicleCount` field from the
  same `--config` JSON `Run_AWSIM.sh` already uses for the map
  (`experiments/configs/baseline.json` now has it, default `-1` = no
  override). **Still requires one Unity Editor rebuild** (2022.3.62f1,
  pinned in the source's `ProjectVersion.txt`) to compile the patch into
  `awsim_labs_v1.6.1/`'s Linux player binary — no Editor is installed on
  this machine yet, and building one is an interactive/GUI step for
  Kalpit, same as never launching AWSIM itself via Bash. See
  `docs/archive/research_notes/layer2_apriori_aposteriori_split_2026-09-03.md`.
- **AWSIM itself can get stuck in a way restarting Autoware alone does
  NOT fix** (found 2026-10-03): vehicle stays frozen in Park, velocity
  pinned at 0, while Autoware reports itself completely healthy
  (`DRIVING`/`AUTONOMOUS`/MRM `NORMAL`, NDT/LiDAR all fine) —
  `experiments/scripts/diagnose_system.py --continuous` is the fast way to
  tell "Autoware's own view of itself" from "is the vehicle actually
  moving." Fix is restarting AWSIM (`./Run_AWSIM.sh`) too, not just
  `Run_Autoware_Headless.sh` — they're separate processes and a Terminal-2
  restart never touches Terminal-1.
- **`count_existing_trials()` (auto-resume, `run_experiments.py`) counts
  ANY trial directory with a `result.json`, regardless of outcome** — this
  is deliberate (a legitimate `mrm_succeeded` fault outcome shouldn't be
  endlessly re-run), but it means a leftover `stuck`/`timeout`/
  `engage_failed` trial silently blocks that slot from ever being retried.
  Delete broken trial directories before re-running a campaign, don't just
  leave them — auto-resume can't tell the difference between "this trial
  legitimately finished" and "this trial crashed."
- **Every live-session failure across the 2026-10-03/04 TL severity sweep
  was at `goal_026` specifically** (6 total: `engage_failed`, `timeout`
  ×3, `stuck` ×2) — goal_007/012 never failed once. Not yet root-caused.
  Two live hypotheses, not mutually exclusive: a real goal_026 map/routing
  issue, or goal_026 simply always being listed last in
  `--goals goal_007,goal_012,goal_026` so it absorbs whatever degrades
  with trial count regardless of which goal it is. Worth testing (run
  goal_026 first in a batch) before trusting goal_026-specific results at
  face value.

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
  **Campaign naming convention + a glossary of existing (legacy) names**:
  `docs/data_naming_convention.md` (added 2026-10-03) — read before naming
  a new campaign; nothing existing has been renamed.
- `experiments/configs/captured_goals.json` — the **operative** goal set
  (26 goals, edited in place; `captured_goals_original.json` is a frozen
  historical snapshot, never edited). `capture_goals_session.py` always
  **overwrites** `captured_goals.json` with only that session's captures —
  back the file up first, then hand-merge new entries by ID.
- **`STGAT_GRAPH_FRAME` env var** (`bbox_v1` default = canonical model;
  `ego_v2` = new schema): selects graph schema AND the data/checkpoint/model
  paths (`h30_30` vs `h30_30_ego`). Set it for every extraction/training/
  analysis command touching an ego_v2 model — mixing a model with the other
  schema's data fails loudly (node feature dim / schema hash). `train.py`
  must be run as `python3 -m st_gat.train`.
- `experiments/configs/goal_split_manifest.json` — the **explicit,
  git-tracked** train/calibration split assignment (added 2026-10-03,
  replaces a recomputed-every-run shuffle that had no record of a goal's
  prior assignment and could silently drift — see
  `st_gat/pipeline/goal_split.py`'s docstring). Never edit this file by
  hand — use `experiments/scripts/manage_goal_split.py` (`seed` /
  `generate --dataset X` / `set --dataset X --goal Y --split {train,cal}` /
  `verify`). `run_pipeline.py`'s `assemble_splits()` reads this and
  reconciles `TRAIN_DIR`/`CAL_DIR` to match it exactly (adds missing
  symlinks, removes stale ones) — it now fails loudly instead of silently
  defaulting if an extracted goal has no manifest entry yet.
- `st_gat/` — the model. `st_gat/pipeline/` (extraction: `bag_reader.py`,
  `sequence_builder.py`, `run_pipeline.py`), `st_gat/model/` (architecture,
  loss, trainer), `st_gat/train.py` (`--warmup-only` = the plain point
  predictor Layer 1 wraps; `--zone-weighted-sampling` = the v2 training
  fix), `st_gat/residuals.py` (per-timestep residual trace generation),
  `st_gat/checkpoints/<run>/` (training checkpoints) vs.
  `st_gat/models/<tag>/st_gat_rise.pth` (deployed/canonical, gated by
  `promote_model.py`).

## Trusting a model before trusting its results

1. `experiments/scripts/model_safety_benchmark.py --tag <name>` — the
   "good enough for Layer 2" gate and the ablation table (one row per model).
2. Layer 1: `conformal_horizon_calibration.py`, then
   `audit_minority_scenarios.py`, `conformal_mondrian_calibration.py`,
   `conformal_scene_conditioning.py`; against faults
   `inspect_fault_predictions.py --campaigns <all fault campaigns>`.
3. Layer 2: `layer2_consequence_estimation.py --mode all`.
Run non-canonical models with the matching `STGAT_GRAPH_FRAME`; their outputs
go to `experiments/analysis/<DATA_TAG>/` (`cfg.ANALYSIS_ROOT`), never over
the canonical results.

## Working style notes

- This repo moves fast — re-read `TODO.md` at the start of a session rather
  than trusting memory of a prior conversation.
- Kalpit runs AWSIM/Autoware experiment collection manually and reports
  back; don't assume a background agent can drive that part.
