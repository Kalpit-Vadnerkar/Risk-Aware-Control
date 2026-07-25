# CLAUDE.md

Guidance for Claude Code (or any agent) working in this repo.

## What this is

Kalpit Vadnerkar's PhD dissertation project (Clemson ECE, advisor Pierluigi Pisu).

**Core contribution (reframed 2026-07-24 — see TODO.md "Research Direction" and
`docs/theoretical_framework.md` for the full argument):** the digital twin detects
faults through **belief divergence under a map-grounded prior**, not generic
residual anomaly detection. It maintains an independent expectation of what the
perception layer should be reporting — from HD map (Lanelet2) categorical
assertions, learned nominal temporal patterns, and vehicle dynamics context — and
flags a fault when the autonomy stack's perceptual belief diverges from that
expectation. The strong form is **negative-evidence detection**: the map licenses a
hard categorical expectation ("a signalized intersection exists here"), and the
fault signature is perception failing to report what must be there.

**Claim to defend:** this framework detects when the autonomy stack's perceptual
belief has diverged from a map-grounded, independently derived expectation of the
world; it reports that divergence with **calibrated confidence**, a bounded number
of seconds of **lead time** before the divergence degrades vehicle safety. Three
load-bearing pieces: belief divergence/negative evidence (mechanism — not yet
proven, see Priority 0 in TODO.md), calibrated confidence (conformal prediction /
UQ — a calibration curve, not an accuracy number), lead time (prediction-horizon
study reframed as a safety result, measured against Autoware's own MRM trigger as
ground truth). Detection ("fault: yes/no") is already banked from the published
paper; the dissertation's job is *verification*.

Extends the published prior work — `Digital_Twins_as_Predictive_Models_for_Real-Time_Probabilistic_Risk_Assessment_of_Autonomous_Vehicles.pdf`
in the project root (IEEE T-ITS vol. 27 no. 4, April 2026, 93.7% fault detection
accuracy across Camera/IMU/LiDAR, Traffic Light Status Flag 29.7% feature
importance, TL Status Flag × CUSUM 22% top feature-residual combination) — not by
restating it but by explaining *why* those numbers came out that way (negative
evidence needs a hard prior; traffic lights have one via the map, object
detections don't) and building the calibration/lead-time evidence the paper
doesn't have. Runs on Autoware + AWSIM in a simulated Nishishinjuku (Tokyo) map.

**Repo name is a legacy artifact.** This repo was originally scoped as an
extension from passive detection to *active risk-aware control* ("RISE" —
Residual-Informed Safety Envelopes; relaxing a velocity constraint under
uncertainty so the AV makes progress instead of stopping). **That control work is
not the core dissertation contribution as of this reframe.** Scenario-based
avoidance demonstrations (staged obstacles, cut-ins — the `obs_*` campaigns below)
are a contribution to control/handling, not to safety verification, and are
deliberately scoped out of the claim being defended (see TODO.md's "Scoping
corollary"). RISE content stays in `docs/` as clearly-marked future-work, not
deleted — don't treat it as the thing under defense.

**Don't duplicate context that's already written down and moves fast:**
- `TODO.md` — current research direction, phase status, what's next, and the two
  **blocking** experiments (per-fault-class importance recompute; HD map detection
  ablation) that either establish or kill the mechanistic claim. **Read this
  fresh every session** — direction changes are common.
- `README.md` — environment setup, the Autoware source patches that must be reapplied
  after any Autoware reinstall/rebuild, data collection campaign commands.
- `docs/theoretical_framework.md` — the belief-divergence mechanism, the claim,
  and the two-arm fault-injection design (Arm A: Autoware safety disabled, the
  science condition; Arm B: Autoware safety enabled, the ground-truth oracle for
  lead-time measurement).
- `docs/research_notes/` — dated findings docs (MRM analysis, EKF fixes, fault
  literature review, etc.) — check these for the *why* behind non-obvious decisions
  before re-deriving them. These predate the 2026-07-24 reframe and are kept as
  historical record, not rewritten to match it.

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

## Known gotchas (hit and fixed this repo's history — don't reintroduce)

- **Lanelet2 map projection:** always use
  `autoware_lanelet2_extension_python.projection.MGRSProjector(lanelet2.io.Origin(0.0, 0.0))`
  to load `Map/nishishinjuku_autoware_map/lanelet2_map.osm`. A generic
  `lanelet2.projection.UtmProjector`/`LocalCartesianProjector` with a guessed lat/lon
  origin does **not** line up with the AWSIM/ROS2 map frame (verified off by
  1000s–10000s of meters) — this is what Autoware's own `autoware_map_projection_loader`
  uses internally, not a convention we invented. `experiments/scripts/plot_routes.py`
  and `explore_map.py` are the reference-correct examples.
- **Background ROS processes:** `$!` after `cmd &` is unreliable in this harness (the
  command gets wrapped, so `$!` can capture the wrong PID). Always verify with
  `pgrep -af <script_name>` before trusting a PID to kill, and confirm it's actually
  gone after killing before starting a replacement — two uncoordinated listeners on
  the same ROS topics produces garbled, hard-to-debug output that looks like a sensor
  bug but isn't.
- **`autoware_pose_instability_detector_node`** has been observed to crash
  (`cannot store a negative time point in rclcpp::Time`) and not auto-restart for the
  rest of an Autoware session. Doesn't appear to corrupt collected data (verified via
  trajectory-teleport + diagnostics checks — see `analyze_mrm_diagnostics.py --batch`),
  just silently drops that one monitoring signal. Worth a `ros2 node list | grep
  pose_instability_detector` check before trusting a long session.
- Params under `config/**/*.param.yaml` are **load-time, not hot-reloadable** —
  Autoware needs a restart after editing, verify with `ros2 param get` that the new
  value actually loaded.
- **MRM diagnostic gate is currently one configuration, not two.** README.md item 3
  strips perception/planning/localization out of Autoware's autonomous-mode gate so
  injected faults propagate instead of being amputated by MRM — that's Arm A
  (safety disabled, the science condition) territory. There is no Arm B config
  (stock/full diagnostic gate, so Autoware's own MRM trigger can serve as the
  ground-truth "unsafe" timestamp for lead-time measurement) yet. See TODO.md's
  two-arm fault injection section — this is a real gap, not just a documentation
  one, before lead-time-vs-Arm-B experiments can run.

## Directory conventions

- `experiments/scripts/` — every one-off/manual Python and shell tool lives here (not
  repo root — keep it that way when adding new scripts).
- `experiments/analysis/` — output artifacts from analysis scripts (route maps,
  feasibility reports). Not raw experiment data.
- `experiments/data/<campaign>/<goal_id>/t<N>_<timestamp>/` — one dir per trial:
  `result.json`, `metadata.json`, `metrics.json`, `fault_log.jsonl`, `rosbag/`.
  Nested by goal (revised 2026-07-22, was flat with the campaign name repeated
  in every trial dirname — `goal_XXX_<campaign>_t<N>_<timestamp>/`). Campaign-level
  files (batch summaries, the running fault log) live in `<campaign>/_meta/`,
  not loose at the campaign root — so `experiments/data/<campaign>/` only ever
  contains goal subdirectories plus `_meta/`.
- `experiments/configs/captured_goals.json` — the **operative** goal set (currently
  26 goals; edited in place when a goal gets replaced/added — see its neighbor
  `captured_goals_original.json`, a frozen historical snapshot, kept for comparison,
  never edited). `experiments/scripts/capture_goals_session.py` always **overwrites**
  `captured_goals.json` with only that session's captures (IDs restart at `goal_001`)
  — back the file up before running it, then hand-merge the new entries in by ID.

## Before trusting newly-collected experiment data

Don't just look at `result.json`'s `status` field — `goal_reached`/`stuck` can both
hide real problems (or hide nothing at all). Run, in order:
1. `experiments/scripts/analyse_experiments.py --campaign <name>` — fast, no ROS
   needed, JSON-only sanity checks (drift/zero-movement, implausible velocity, UUID
   tracking gaps).
2. `experiments/scripts/analyze_mrm_diagnostics.py --batch experiments/data/<campaign>`
   — needs ROS sourced; whole-trial ERROR/WARN diagnostic audit + ground-truth
   teleport/zero-movement check per trial, catches things that never triggered MRM
   (like a localization/NDT convergence failure that just silently stalls the vehicle).
3. `experiments/scripts/plot_routes.py --goal <ids> --goals-file captured_goals.json`
   — needs ROS sourced; visual sanity check, trims the map to the route, colors by
   outcome, shows traffic light locations.

For fault campaigns specifically (`tl_fault_s1..s4`, `imu_fault_s1..s4`), also run
`experiments/scripts/compare_fault_vs_nominal.py --campaign <name> --goal <goal_id>`
— needs ROS sourced; verifies the fault actually changed the signal it targets
(TL confidence/color, or EKF-vs-ground-truth divergence for IMU) rather than just
trusting that `fault_log.jsonl` logged a cycle, and ranks candidate ST-GAT state
features by how strongly each responds to the fault. See
`docs/research_notes/periodic_fault_strategy.md` for the fault-injection design
this checks against.

A trial with `mrm_trigger_count: 0` is not automatically clean — that count only
reflects Autoware's own MRM state machine, not whether the vehicle actually moved.

## Working style notes

- This repo moves fast (research direction, phase status) — re-read `TODO.md` at the
  start of a session rather than trusting memory of a prior conversation.
- Kalpit runs AWSIM/Autoware experiment collection manually and reports back; don't
  assume a background agent can drive that part.
