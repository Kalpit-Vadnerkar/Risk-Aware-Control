# CLAUDE.md

Guidance for Claude Code (or any agent) working in this repo.

## What this is

Kalpit Vadnerkar's PhD dissertation project (Clemson ECE, advisor Pierluigi Pisu).

**Read `open_world_safety_reframe_2026-08-20.md` first — it supersedes the
"belief divergence" claim language in this section and below, though the
step/phase mechanics described here (calibration first, then the SPRT
signal, then planning interfaces) still map onto that reframe's Layer 1/2/3
structure: step (1) below = Layer 1 (now done, via conformal), step (2) =
still open, step (3) ≈ Layer 3 (now an open scoping question, not a settled
future-work exclusion).**

**Direction reframe (2026-08-06 — supersedes the framing
below where they conflict).** The project should NOT converge on "detect and
classify a specific fault type" — collapsing a fault into a named category
just licenses a pre-planned canned response for that category, which is
explicitly not the goal. Instead: provide a meaningful, continuous,
digestible signal to the modules that actually own the vehicle's
decision-making (planning/control), so the vehicle can keep operating
sensibly under degraded/uncertain conditions without needing to know which
specific fault is occurring. Motto: **"operational under degradation."**
Concretely this means: (1) ground the belief-divergence mechanism's own
credibility with calibration/trust plots before anything else, (2) check
that the SPRT/sequential-evidence signal (`p_fault_motion`/`p_fault_tl`/
`p_fault_combined` in `st_gat/residuals.py`) behaves as an interpretable
continuous trace, not "does it cross a threshold," and only then (3) look
at Autoware planning/control interfaces.

**Calibration status (2026-08-20 — supersedes everything below in this
section; read
`docs/research_notes/nll_calibration_arc_and_conformal_pivot_2026-08-20.md`
for the full arc).** Step (1) above now HOLDS UP: after a multi-week arc
trying to get the model's own jointly-trained Student-t heads
(mean+scale+dof per feature via NLL) correctly calibrated — which surfaced
real, fixable bugs (a shared-gradient-contention bug in the horizon-step
embedding, fixed; a dof-collapse pathology, fixed; a scale/validation-
residual-drift pathology, diagnosed but not yet fixed) but never converged
end-to-end — the project **pivoted to conformal calibration** on top of a
plain point predictor (`st_gat/train.py --warmup-only`, no variance/dof
head involved in training at all). Validated same day via leave-one-trial-
out cross-conformal (only 7 nominal trials exist — a naive fixed fit/test
split was tried first and found unreliable, see the research note):
pooled empirical coverage 89.7-90.2% (target 90%) at every horizon step for
all 7 series (position, velocity split per-axis, steering, acceleration,
both TL heads), correct widening for every one, including the two
traffic-light heads the distributional approach never calibrated. Real
caveat, not hidden: per-trial coverage swings much wider (e.g. 77-95% for
acceleration across the 7 individual held-out trials) — the *aggregate*
claim is well-supported, a claim about any single new drive is not, given
only 7 trials exist. **This is now Layer 1 of the 2026-08-20 open-world
reframe** (`open_world_safety_reframe_2026-08-20.md`) — detect the unknown,
no fault data required. The
Student-t/NLL machinery is not deleted (still real, working code) but is
paused — see the research note's §5 for the concrete, well-evidenced next
step if it's ever resumed, and don't re-diagnose the collapse pathology
from scratch without reading it first.

**Core contribution (reframed 2026-08-20 — see
`docs/research_notes/open_world_safety_reframe_2026-08-20.md` for the full
argument, supersedes the 2026-07-24 framing below where they conflict):**
open-world safety verification, not closed-set fault classification. The
question is not "which known fault is this" (requires fault data, a fixed
bin set, says nothing about a failure mode outside that set) but "how far
off-nominal am I right now, in what way, and what does that imply about
whether I'm about to do something unsafe." Three-layer architecture:
- **Layer 1 — detect the unknown**: a continuous, distribution-free anomaly
  measure (ST-GAT residuals as conformal nonconformity scores), no fault
  data required. **Already built** — this is the conformal-calibration
  pivot (`nll_calibration_arc_and_conformal_pivot_2026-08-20.md`).
- **Layer 2 — consequence estimation (THE thesis)**: map anomaly to actual
  risk by rolling counterfactual futures forward through the digital twin
  — the same residual, asking "so what" instead of "which bin." Not yet
  built; the core remaining work.
- **Layer 3 — graceful response**: continuous behavior modulation
  proportional to quantified ignorance, not limp-mode/ODD binning. Scoping
  vs. the descoped RISE work below is an open question, not yet resolved.

**Claim to defend:** a principled translation from "my model is uncertain
in this specific way, right now" to "therefore this specific driving
behavior is unsafe, by this much" — reported with calibrated confidence (a
reliability diagram, not just a coverage number) and **decomposition**
(which subsystem/uncertainty-source it's attributable to, not just a
magnitude), giving measurable lead time before a safety-margin violation,
evaluated at matched false-alarm rates. The 2026-07-24 "belief divergence /
negative evidence" framing and its Priority 0 mechanism experiments
(per-fault-class importance recompute, HD map ablation) are retired — see
the reframe note §1 for the disposition of each (one dropped outright, one
folded into Layer 2 validation with a different, continuous metric).
Decoupled from the published T-ITS paper's own reference repo as a
dependency — it stays cited context, not something new work reproduces
against.

Runs on Autoware + AWSIM in a simulated Nishishinjuku (Tokyo) map.

**Repo name is a legacy artifact.** This repo was originally scoped as an
extension from passive detection to *active risk-aware control* ("RISE" —
Residual-Informed Safety Envelopes; relaxing a velocity constraint under
uncertainty so the AV makes progress instead of stopping). Under the
2026-08-06 reframe that control work was explicitly out of scope, future-
work-only. **As of the 2026-08-20 reframe's Layer 3 ("graceful response"),
this is an open scoping question again, not a settled exclusion** — see
`open_world_safety_reframe_2026-08-20.md` §4. Don't assume RISE content is
still purely illustrative without checking whether Layer 3 has been scoped
one way or the other since this was written.

**Don't duplicate context that's already written down and moves fast:**
- `TODO.md` — current research direction, phase status, what's next. **Read
  this fresh every session** — direction changes are common. Priority 0's
  old "blocking" mechanism experiments (per-fault-class importance,
  HD-map-ablation-as-classification-metric) are retired as of 2026-08-20 —
  see the reframe note for the one piece that was kept, reframed.
- `README.md` — environment setup, the Autoware source patches that must be reapplied
  after any Autoware reinstall/rebuild, data collection campaign commands.
- `docs/theoretical_framework.md` — the belief-divergence mechanism and the
  two-arm fault-injection design (Arm A: Autoware safety disabled, the
  science condition; Arm B: Autoware safety enabled, the ground-truth oracle
  for lead-time measurement) — the two-arm design still applies under the
  2026-08-20 reframe; the belief-divergence *claim* language there is
  superseded, not the fault-injection mechanics.
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
- **MRM diagnostic gate is now toggleable between Arm A and Arm B (2026-07-25).**
  README.md item 3 strips perception/planning/localization out of Autoware's
  autonomous-mode gate so injected faults propagate instead of being amputated by
  MRM — that's Arm A (safety disabled, the science condition). Arm B (stock/full
  gate, restores Autoware's default `/autoware/modes/autonomous` linkage exactly,
  verified against this checkout's own git HEAD) now exists too, as the
  ground-truth oracle for MRM lead-time measurement. Switch with
  `experiments/scripts/switch_diagnostic_arm.sh {A,B}` — this only copies config
  files (`autoware-main.yaml`/`control.yaml`/`system.yaml` under
  `autoware/src/launcher/autoware_launch/.../config/system/diagnostics/`, each
  arm's variant kept alongside as `.armA.yaml`/`.armB.yaml`), it does **not**
  restart Autoware — these are load-time params, you must restart yourself for a
  switch to take effect. `collect.sh`/`run_fault_campaigns.sh` take `--arm A|B`
  (default A) and will refuse to run if that doesn't match the switch script's
  `.active_arm` marker, to stop mislabeled data before it happens. Arm B is
  untested against this repo's actual fault-injection workflow as of this
  writing — it's exactly the config that caused the original MRM deadlocks
  (routing resets, TF drops during teleports) that Arm A was built to route
  around, so watch for that when Arm B collection actually runs.

## Directory conventions

- `experiments/scripts/` — every one-off/manual Python and shell tool lives here (not
  repo root — keep it that way when adding new scripts).
- `experiments/lib/` — shared library code imported by multiple scripts (not runnable
  on its own): `fault_injector.py`, `metrics.py`, and (added 2026-08-01)
  `plotting.py` — map loading/rendering, outcome styling, zone-drawing, and
  divergence-trace panel primitives shared by `plot_routes.py`/`plot_fault_plan.py`/
  `plot_fault_impact.py` and meant to be reused by future model-prediction
  distribution plots too. Add new cross-script plotting helpers there, not as a
  copy-pasted function in whichever script needed it first.
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

## Before running a new TL or IMU fault campaign

`fault_injector.py` gates both fault types on precomputed, route-derived zone
files, not live thresholds — regenerate these whenever `nom_v11`, the goal
set, or the map changes, and before the very first campaign run:
- `experiments/scripts/compute_turn_zones.py` — IMU turn / bias-lead-in zones
  (`experiments/configs/turn_zones.json`), from each goal's actual planned
  route (`/planning/mission_planning/route`) geometry.
- `experiments/scripts/compute_tl_zones.py` — TL zones with each one's real
  `traffic_light_group_id` (`experiments/configs/tl_zones.json`), from the
  same route's lanelet sequence. This is what lets a TL fault target only
  the one light the vehicle's current lanelet is actually governed by
  (`_tl_fault_group_id`) instead of every group in the perception message —
  if this file is missing or stale for a goal, TL faults silently fall back
  to mutating everything, which still works but isn't scoped.

`experiments/scripts/plot_fault_plan.py --campaign <name>` renders a given
campaign's actual gating zones/radii over the route map — a fast visual
sanity check before committing to a full collection run.

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

For IMU fault campaigns specifically (`imu_fault_s1`, `imu_fault_s3`, `imu_fault_scale`,
`imu_fault_stuck`), also run `experiments/scripts/verify_imu_zone_arming.py --campaign
<name>` — no ROS needed, reads `fault_log.jsonl` only; independently re-checks (from
`turn_zones.json`, not from trusting `fault_injector.py`'s own gating) that every
`imu_bias_on` on-cycle actually started inside its intended zone (turn zone for
scale/stuck, bias lead-in zone for bias). Skips pre-2026-07-26 data, which predates
zone-based arming and doesn't log `position`/`zone_kind`.

For TL fault campaigns specifically (`tl_fault_s2..s4`, `tl_fault_ramp`), also run
`experiments/scripts/verify_tl_zone_arming.py --campaign <name>` — no ROS needed, reads
`fault_log.jsonl` only; independently re-checks (from `tl_zones.json`) that every
`tl_fault_start` event's `group_id` matches a real, reachable zone for that goal — the
group-id scoping added 2026-07-27 that lets a TL fault target only the one light the
vehicle is reacting to (see `fault_injector.py`'s `_tl_fault_group_id`). Skips
pre-2026-07-27 data, which predates group-id scoping and doesn't log `group_id`.

Both `turn_zones.json` and `tl_zones.json` carry a `reachable` field per zone (false
for the one zone every goal's runway-clear point structurally consumes — see
`compute_turn_zones.py`'s module docstring) — this is informational, NOT a filter:
`fault_injector.py` loads the complete list either way, since the runway/arming state
machine already makes an unreachable zone unable to arm on its own. Don't re-filter
by `reachable` when loading these files for gating; only the plotting/reporting layer
should use it, to avoid quietly breaking runway-clear detection again (see
`docs/fault_scenario_table.md`'s "how zones are computed" section for the incident
this note is guarding against).

A trial with `mrm_trigger_count: 0` is not automatically clean — that count only
reflects Autoware's own MRM state machine, not whether the vehicle actually moved.

## Trusting the model itself (before trusting any detection/SPRT result)

Added 2026-08-06, per the reframe above. **Run
`./experiments/scripts/run_calibration_pipeline.sh` for the full Layer-1
result set in one command** (repo venv only, no ROS needed — reads
precomputed `.pkl` sequences, not live bags) — chains the two scripts
below and lists the independently-trained epistemic members in one place
to edit as more get trained. Scales automatically as `CAL_DIR` grows (LOO-
CV folds on however many trial files exist, not a hardcoded count).

**Primary calibration check as of 2026-08-20 (Layer 1 of the open-world
reframe)**: `experiments/scripts/conformal_horizon_calibration.py` —
leave-one-trial-out cross-conformal per feature-series per horizon step
(NOT a fixed split — `CAL_DIR` only has 7 trials, a naive fixed split gave
an unreliable estimate, see the research note), wrapped around a plain
point predictor's residuals (no distributional head). Reports pooled +
per-fold-spread coverage, `conformal_vs_actual.png` (interval width vs.
actual RMSE), and `reliability_diagram.png` (nominal vs. empirical
coverage swept across 50%-98%, pooled across horizon — the headline
calibration plot; current result: max gap 0.004-0.011 across every series
and every coverage level). **Read this pooled number carefully, don't
over-trust it on its own** — it pools every window, fold, and horizon step
into one number per alpha, which is heavy averaging; the coverage table's
per-fold spread (e.g. acceleration ranged 71-100% across the 7 individual
held-out trials at one alpha) shows real per-trial instability that the
pooled/aggregate number can hide via cancellation. The near-perfect
diagonal is a genuine average-case result, not evidence every trial or
region of driving is well-calibrated on its own — more nominal trials
(pending future data collection) is the actual test of that. See
`docs/research_notes/nll_calibration_arc_and_conformal_pivot_2026-08-20.md`
for why this replaced the Student-t/NLL approach, and
`open_world_safety_reframe_2026-08-20.md` for how it fits the current claim.

**Concrete trust visualization (2026-08-21)**:
`experiments/scripts/plot_layer1_trust_examples.py` — the aggregate plots
above prove calibration statistically; this shows it concretely, for a
handful of real example windows: predicted mean ± the calibrated conformal
band vs. actual value, per scalar feature (`window*_scalar_bands.png`), and
a map-view of predicted vs. actual trajectory with the calibrated position
uncertainty drawn as discs along the path (`window*_position_map.png`).
Reuses `experiments/lib/plotting.py`'s existing map/band-plotting
primitives. Output: `experiments/analysis/layer1_trust_examples/`.
**Fixed bug (2026-08-24)**: the uncertainty-disc radius was passed straight
from `conformal_report.json`'s NORMALIZED quantile without multiplying by
`POSITION_DISPLACEMENT_RANGE_M`, making circles ~100x too small to see
against real map coordinates — now scaled correctly.
Example windows are no longer random: `experiments/scripts/
select_trust_example_windows.py` scores every calibration window (turn
severity, TL color transition, braking, acceleration, calm baseline) and
picks one fixed canonical example per category, saved to `experiments/
analysis/trust_example_windows.json`, so the same 5 scenarios are reused
every time the pipeline regenerates (pass `--random` to
`plot_layer1_trust_examples.py` for the old random-sample behavior).

**Turn-anticipation gap found (2026-08-24)** — real, quantified, NOT yet
fixed: `experiments/scripts/diagnose_turn_learning.py` shows the position
head captures real future turns only partially (predicted heading change
~60% of actual magnitude on average), and the gap is much worse
specifically when the turn hasn't started yet in the observed 3s history —
predicted turn magnitude is only ~48% of actual for windows where the turn
is genuinely upcoming (not yet visible in the past), vs. ~65% for windows
where the ego is already mid-turn. r(actual future turn, predicted turn) =
0.62 overall, but r(past turn already observed, predicted turn) = 0.53 is
nearly as strong — meaning much of what looks like "turn prediction" is
actually the model extrapolating a turn already underway (kinematic
continuation), not genuinely anticipating one from route/map context via
the graph encoder, despite `GraphBuilder` including route-ahead lanelets
(`path_node=1`) within `GRAPH_RADIUS_M` (~150m, horizon-derived) of the
window's start. Practical risk this raises for Layer 1's trustworthiness,
flagged by Kalpit directly: if ordinary turns systematically produce
larger residuals than calm driving, the pooled/global conformal quantile
either (a) is wider than necessary during calm driving to stay valid
during turns, or worse, (b) a real anomaly occurring near/during a turn
gets partially masked because the "turn residual" and the "fault residual"
are conflated into one undifferentiated magnitude — exactly the scenario
the already-agreed-but-unbuilt scene-embedding conditioning of the
conformal quantile (`open_world_safety_reframe_2026-08-20.md` §5) would
directly address, by giving turn-like scenes their own quantile instead of
sharing one global per-step width with calm ones. Root cause not yet
isolated further (candidates: rare-event MSE-attenuation from turns being
only ~10% of windows, vs. weak/diluted graph-context signal reaching the
position head at all — GraphEncoder's attention-pooling behavior on
route-ahead nodes hasn't been directly inspected).

**Scene-embedding-conditioned conformal quantile — first implementation
and validation (2026-08-24)**, Kalpit's chosen fix for the above:
`experiments/scripts/conformal_scene_conditioning.py`. `model.py`'s
`STGAT.forward()` was refactored (non-breaking) to factor its trunk
computation (steps 1-6, everything upstream of the per-head decoders) into
a new `encode_scene(x, graph) -> h_last` method, so this script gets the
model's own learned "what kind of situation is this" vector without
duplicating trunk logic. Method: for each test window, find its k nearest
neighbors by Euclidean distance in `h_last`-space among the FIT set only
(same leave-one-trial-out discipline, no leakage), fit the conformal
quantile on that neighborhood's residuals instead of the whole fit set.

Confirms the exact mechanism Kalpit predicted, decisively, for steering:
vanilla (global) per-step coverage pools to ~90% overall, but splits into
94.2% on calm windows and only **59.0%** on real-turn windows — the pooled
average hides a badly broken turn-specific guarantee. Scene-conditioning
(k=150 neighbors) raises turn-window coverage to 90.1% (k=400: 79.0%,
non-monotonic in k — see caveat below) while correctly widening the
interval ~2x specifically for turn-like scenes (steering half-width
0.024→0.046-0.057 on turns, i.e. it detects the harder scenario and
reacts) and tightening it for the calm majority (0.024→0.015-0.016).
Position, whose vanilla per-step coverage was already uniform across
turn/calm (94.7%/90.1%, no gap to fix), got mildly WORSE under
scene-conditioning (~85% both) — the fix is real and targeted, not a
blanket improvement, and doesn't help where there was nothing to fix.

Not yet a finished/production result — real caveats, don't overclaim: (1)
coverage doesn't hit the 90% target cleanly and isn't monotonic in
k-neighbors (k=150 outperformed k=400 for steering-on-turns in one
comparison) — likely a mix of small-sample quantile noise and h_last not
being a perfectly turn-specific similarity metric (it encodes the WHOLE
scene, so neighbor sets also vary on other axes, visible as high width
variance even among turn_deg≈0 windows in the scatter plot). (2) Only
checked for `position` and `steering` series so far (the two most
turn-relevant), not the full 7-series set. (3) A "whole-horizon joint
coverage" metric (all 30 steps simultaneously in-band) was also computed
alongside the validated per-step metric — reads much lower (~55-77%) by
construction (a strictly harder bar) — the two are NOT the same number,
keep them separate when discussing this result to avoid contradicting the
already-published ~90% figure. Candidate next steps, not yet tried:
normalize/whiten `h_last` before the distance metric, try cosine
similarity, or adapt k per-window instead of a fixed constant.

Companion check, same day: `experiments/scripts/epistemic_disagreement_check.py`
— does disagreement between independently-trained point predictors widen
with horizon (a robustness-under-shift proxy the conformal check alone
doesn't cover, since that's validated on nominal data only). Mixed result:
real for `position`/`acceleration`, flat for the other 5 series — see the
pivot note's §6.

The two scripts below are the now-paused NLL/Student-t-head diagnostics —
still real, working tools (useful if that arc is ever resumed per the
research note's §5), just not the first thing to run anymore:
- `experiments/scripts/plot_calibration_diagrams.py` (needs ROS+model) —
  coverage curves, PIT histograms, PIT-uniformity (Kolmogorov-Smirnov)
  stats, horizon-widening, TL-discrepancy reliability diagram, on the held-
  out calibration split. Rewritten 2026-08-06 around the **probability
  integral transform (PIT)**, not a raw z-score — the model's heads predict
  a per-sample degrees-of-freedom (Student-t, not Gaussian, since
  2026-08-06), so there's no single fixed reference distribution to check a
  z-score against; PIT generalizes correctly regardless of family. If you
  add a multi-dim feature's PIT to a pooled array, **flatten across dims,
  don't average** — averaging two independent Uniform(0,1) values is not
  itself Uniform(0,1) (a real bug this script had once, see the research
  note below for the exact effect it had).
- `experiments/scripts/plot_sprt_signal_behavior.py` (no ROS needed, pure
  pandas over `st_gat/results/h30_30/traces/*.csv`) — does `p_fault_*` stay
  near its floor (0.5, not 0.0 — see the script's own docstring) under
  nominal noise, and does it change shape at fault onset. Currently: no,
  it saturates to >0.99 roughly every 9s under nominal driving alone.

## Working style notes

- This repo moves fast (research direction, phase status) — re-read `TODO.md` at the
  start of a session rather than trusting memory of a prior conversation.
- Kalpit runs AWSIM/Autoware experiment collection manually and reports back; don't
  assume a background agent can drive that part.
