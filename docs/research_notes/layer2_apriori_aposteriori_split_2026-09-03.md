# Layer 2 a priori / a posteriori split (2026-09-03)

**Context:** presented the 2026-08-26 deck to advisor. Verdict: on track
with Layer 1 (calibration/uncertainty), but Layer 2 as currently framed —
bootstrap-resample the calibrated interval into counterfactual futures,
then build a SEPARATE reachability margin (lane-containment + object
reachable sets) to check them against — is partly redundant. Swept across
the horizon, Layer 1's calibrated interval per feature per horizon step
already IS a raw reachable tube in feature space. Building a second,
independently-constructed "reachability set" on top of it re-derives the
same object rather than reasoning about it. Advisor's framing: Layer 2
needs a priori and a posteriori reachability-set optimization — trim/shape
the envelope using HD-map and scene cues, not construct a new one from
scratch.

## Resolution — two stages, both operating on Layer 1's envelope, not replacing it

- **Layer 2a (a priori — before the interval is emitted).** Shape the
  calibrated band itself using scene/zone context, so it's already tighter
  where the map/scene licenses tightness. **Not new work** — a relabeling
  of what's already built (slide 12): Mondrian (discrete zone-grouped,
  `experiments/scripts/conformal_mondrian_calibration.py`) and embedding
  k-NN (continuous scene-similarity,
  `experiments/scripts/conformal_embedding_calibration.py`/
  `conformal_scene_conditioning.py`) conditional calibration. Both
  condition the interval on map/scene context *before* the band exists —
  that's precisely "a priori."
- **Layer 2b (a posteriori — after the interval is rolled forward).**
  Bootstrap-resample the (a-priori-conditioned) calibrated residuals into
  counterfactual futures, then geometrically TRIM that tube against real
  physical constraints only available at prediction time: lane-containment
  (`lanelet2.geometry.inside`, not nearest-5-lanelet heuristics), tracked-
  object reachable sets, kinematic feasibility. This is the actual
  "reachability" work — now explicitly framed as pruning an existing
  envelope rather than constructing an independent one, which resolves the
  redundancy critique. **Not yet built** — same engineering scope as
  before this reframe (TODO.md §3's design is otherwise unchanged), just
  now explicitly named as the 2b half.

Checked against the safety margin → calibrated P(violation), same as
before this reframe.

## Two data-availability findings surfaced while resolving this, both relevant to 2b's design

1. **Autoware's own object motion prediction is already recorded in every
   trial's rosbag, but not consumed by the ST-GAT pipeline.**
   `RECORDING_TOPICS` (`experiments/lib/config.py`) records both
   `/perception/object_recognition/objects` (`PredictedObjects` —
   Autoware's own per-object `PredictedPath`s, multi-modal, with
   confidence) and `/perception/object_recognition/tracking/objects`
   (`TrackedObjects` — current state only). But
   `st_gat/pipeline/bag_reader.py`/`sequence_builder.py` only read
   `tracking/objects` — the ST-GAT model's object-set input is current
   relative position/speed/class only, no predicted future path. This is
   correct for the model itself (objects are input-only context, not
   something the model predicts a future for — deliberate design, see
   `config.py`'s note on the permutation-invariant object-set encoder) —
   but it means **Layer 2b's "tracked-object reachable sets" should
   consume Autoware's own already-recorded
   `PredictedObjects.kinematics.predicted_paths` directly at rollout
   time**, not require training the ST-GAT model to predict object
   futures itself. No new data collection needed — the topic is already
   in every bag on disk (also already consumed elsewhere in the repo, for
   fault injection and metrics — `experiments/lib/perception_interceptor.py`,
   `experiments/lib/metrics.py`); this only needs a new read path in
   whatever script builds Layer 2b's rollout.
2. **Route info reaches the model but is diluted by pooling — a known,
   already-tracked item, not new.** `GraphBuilder` sets a real `path_node`
   flag per graph node from the actual Autoware route
   (`/planning/mission_planning/route`), and the 2026-08-05 route-aware
   node-selection + attention-pooling fix (`docs/research_notes/
   ablation_study_2026.md` §2/§3) measurably increased on-route node
   representation (5.6%→22.0%) and gave attention weight preference to
   them. But the pooled graph context vector is still one static vector
   broadcast identically across all 30 input timesteps
   (`model_improvement_notes_2026.md` §2; tracked as TODO.md §7's "graph
   cadence" item, status TBD). Not a Layer 2b blocker directly — 2b grounds
   against `lanelet2.geometry` directly, not the model's internal route
   representation — but worth closing before over-claiming the model
   "reasons about its route" in the Paper 1/2 writeup.

## NPC/traffic-density — raised again in this conversation, engineering blocker was overstated

See `project_open_world_safety_reframe_2026-08-20.md`'s memory for the
original flag (which claimed "needs a full Unity rebuild, not a config
toggle" — **corrected below, that was overstated**).

**Corrected finding (2026-09-03):** checked the actual AWSIM Labs source
(`Awsin-Source-code/AWSIM-Labs-1.6.1/`, vendored locally). NPC density is
NOT scene-baked — `TrafficManager.targetVehicleCount` is already a live,
runtime-adjustable value with a full UI control shipped in the binary
(`Assets/AWSIM/Scripts/UI/TrafficControlManager.cs` +
`UITrafficVehicleDensity.cs`, a slider wired to
`TrafficControlManager.TrafficManagerUpdate()` → `TrafficManager.
targetVehicleCount = ...; TrafficManager.RestartTraffic()`). It just
wasn't wired into the `--config` JSON path the way `mapConfiguration`/
`useTraffic`/`timeScale` already are. Patched this session, same pattern
as the existing config fields:
- `Assets/AWSIM/Scripts/Loader/AWSIMConfiguration.cs` — added
  `targetVehicleCount` (default `-1` = "leave scene default alone") to
  `SimulationConfiguration`.
- `Assets/AWSIM/Scripts/Loader/SimulationManager.cs` — validates it in
  `LoadConfig()` (must be `-1` or `>= 0`).
- `Assets/AWSIM/Scripts/Loader/SimConfiguration.cs` — applies it in
  `Configure()`, right after the existing `useTraffic` `SetActive` loop
  (must run after `TrafficManager.gameObject` is active, or
  `TrafficManagerUpdate()` just logs a warning and no-ops). Preserves the
  scene's configured seed explicitly — `TrafficControlManager.SeedInput`
  defaults to 0, not the `TrafficManager`'s actual seed, so an
  unconditional `TrafficManagerUpdate()` would otherwise silently reset
  the random seed as a side effect.
- `Risk-Aware-Control/experiments/configs/baseline.json` — added
  `"targetVehicleCount": -1` (no behavior change from before this patch;
  set it to a real value to override density).

**What's still actually needed, and it's real:** this source patch has to
be compiled into a new Linux player build to take effect — no Unity
Editor is installed on this machine (`Awsin-Source-code/` is source-only,
not a git repo either, so patches here aren't yet version-controlled).
One-time setup: install Unity Hub + Editor `2022.3.62f1` (pinned in
`ProjectSettings/ProjectVersion.txt`), open the project, Build Settings →
Linux Build, output replacing (or versioned alongside)
`awsim_labs_v1.6.1/`. Kalpit needs to do this — it's an interactive/GUI
step, same category as "never launch AWSIM/Autoware via Bash" in
`CLAUDE.md`'s environment section.

**Still not recommended for the near-term data collection plan** (the TL
severity sweep, TODO.md §1) even once the rebuild is done: its intended
purpose — fault severity × NPC density → P(margin-violation) — can't
actually be evaluated until Layer 2b's violation-checking pipeline
exists. Collecting it before then would produce data that can't yet be
used for what it's for. Revisit once 2b is built, as a deliberate,
batched Layer 2 experiment, not an add-on to the severity sweep — that
sequencing recommendation is unchanged, only the "it's a big engineering
lift" reason for delaying it was wrong.
