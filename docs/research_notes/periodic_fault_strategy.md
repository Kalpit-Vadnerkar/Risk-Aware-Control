# Periodic, Location-Aware Fault Injection Strategy — Camera (TL) & IMU

**Status:** Draft strategy for review, 2026-07-22. Grounds the redesign in
`docs/research_notes/fault_literature_review.md` and the 11 papers now in
`docs/papers/` (gitignored). Not yet implemented — `experiments/lib/fault_injector.py`
still runs the one-shot design described below until this is approved.

---

## 0. Scope check: camera → traffic light is the only path

Verified directly against this project's Autoware install (not assumed):
`autoware.launch.xml:48` sets `perception_mode` default to `"lidar"` — object
detection runs LiDAR-only. The `awsim_labs_sensor_kit` (`sensor_kit.xacro`) defines
exactly one camera, a monocular unit feeding
`/perception/traffic_light_recognition/*`. So a "camera fault" in this simulated
vehicle can only ever mean a traffic-light-detection fault — there is no
camera-based object detector in this pipeline to occlude instead. This confirms
`fault_injector.py`'s existing `tl_*` fault modes (operating on the TL
classification array, `TrafficLightGroupArray`) are the correct and only injection
point, not a workaround.

---

## 1. Camera/TL fault — from one-shot to zone-triggered periodic

**Current behavior** (`fault_injector.py`, `_tl_phase` state machine):
`waiting_delay → waiting_zone → fault_active (fixed duration) → done`. The fault
fires exactly **once** per trial, at the first TL detection zone entered after
`--fault-delay`, then the node passes through raw signals for the rest of the
route — permanently. On a route with multiple intersections, only the first one
is ever faulted.

**Why that's not enough for reaction/recovery analysis:** a single (fault_start,
fault_end) pair per trial gives one reaction-time sample and one recovery-time
sample per trial. To plot "how long does ST-GAT take to react / recover" with any
statistical weight, we want **repeated fault→recovery cycles within a single
trial**, one per TL intersection the route actually passes — which is also exactly
the "be strategic with vehicle location" requirement: a TL fault is only
meaningful while a real traffic light is in detection range, and the existing
detection-window gating (`_in_detection_zone`, ratio of non-empty messages in the
last ~3s) already *is* the location signal — no hardcoded coordinates needed.

**Proposed redesign:** turn `_tl_phase` into a loop instead of a one-shot:

```
waiting_delay → waiting_zone → fault_active → recovering → waiting_zone → ...
                                  (≤ per-zone         (nominal gap,
                                   duration cap)        route continues)
```

- Enter `fault_active` on zone entry (unchanged).
- Exit `fault_active` on **whichever comes first**: the per-zone duration cap
  (existing `--fault-duration`, now a cap rather than a fixed window) or the
  vehicle leaving the detection zone (`tl_window_exited`) — a fault that outlasts
  the intersection isn't meaningful, and this bounds fault exposure at long
  dwells (e.g. stopped at a red light).
- Enter `recovering` for a short nominal gap (e.g. 5–10s) so the raw/CUSUM
  residual has a clean nominal segment to return to before the next cycle —
  otherwise back-to-back intersections could blur recovery-time measurement.
- Return to `waiting_zone` to arm for the **next** TL zone, repeating for every
  intersection on the route (optionally capped at `--max-tl-cycles`, default
  unbounded — a route's own intersection count already bounds it).
- Every cycle already gets `tl_fault_start`/`tl_fault_end` log lines (existing
  logging, unchanged) — this alone converts each trial from 1 sample to N
  samples of (reaction time, recovery time), where N = intersections encountered.

**Severity tiers:** keep the existing S1–S4 (`tl_confidence` 0.5,
`tl_oscillate` 5s period, `tl_unknown`, `tl_blackout`) — already well-motivated
(TL Status Flag was the single most discriminative feature, 29.7%, in the T-ITS
paper's fault classifier). The papers newly reviewed (WoodScape/SoilingNet,
MTF50-vs-droplet-volume, Occluded nuScenes) argue for a genuine *image-level*
occlusion mask as a more literal "camera fault," but building that means
modifying AWSIM's camera render path, not just the TL classification topic —
recommend treating that as a deferred stretch goal (already flagged as open
question #1 in the lit review), not a blocker for this round.

---

### 1.2 What the fault modes actually do, and why (added 2026-07-22)

`TrafficLightElement` is a discrete classification message (color enum, shape,
confidence float) — there's no covariance field on this topic, so "noise on the
covariance" isn't applicable here (that concept fits *continuous* signals, which
is exactly where the IMU fault below uses magnitude/bias injection instead).
None of the four TL modes remove-or-add noise uniformly; each targets a
different point on the real-world degradation spectrum surveyed in the papers:

| Mode | Mechanism | What it's modeling |
|------|-----------|---------------------|
| `tl_confidence` | Scales confidence, keeps the true color | Continuous soft degradation (mild fog/glare/early soiling) — same *shape* as the MTF50-vs-droplet-volume curve (continuous quality loss), though the current `confidence_scale=0.5` is a round number, not read off that curve |
| `tl_oscillate` | Forces WRONG color (GREEN) periodically | Misclassification — general TL-detection literature identifies glare/motion blur/lighting as the dominant real-world cause of wrong-color reads (as opposed to total occlusion). Originally justified in-code purely for CUSUM detectability; the literature independently supports it as realistic |
| `tl_unknown` | Zero confidence, no color | Classifier gives up — a harder failure than misclassification, consistent with heavier soiling/occlusion (WoodScape) |
| `tl_blackout` | No message at all | Total loss — matches the BEVFusion weather-occlusion paper's finding that camera-only detection fails sharply (cliff-edge), not gradually, once occlusion is severe enough |

Net: not noise-based, but a 4-point discrete taxonomy approximating what's
fundamentally a continuous real-world degradation process. If tighter grounding
is wanted later, the clearest next step is reading `confidence_scale` tiers off
the MTF50 droplet-volume curve instead of a flat 0.5 — flagged as a possible
refinement, not required to start running trials.

## 2. IMU fault — already periodic, location-awareness doesn't apply

`fault_injector.py`'s `imu_bias` mode already cycles on/off across the whole
trial (`on_seconds`/`off_seconds`, S1–S4 tiers) independent of position — and
that's correct as-is: gyro bias corrupts EKF twist uncertainty everywhere along
the route, not just near a landmark, so there's no analogous "detection zone" to
gate on. **No structural redesign needed here** — the existing design already
gives repeated on/off cycles per trial, i.e. already periodic in the sense the
TL redesign above is trying to reach.

Two open items from the lit review, still unresolved, worth flagging rather than
silently deferring again:
1. `accel_bias_ms2` remains a documented no-op (`gyro_odometer` doesn't consume
   linear acceleration) — general IMU fault taxonomy treats accelerometer bias as
   standard alongside gyro bias. Fixing this would need a different injection
   point than the current relay.
2. Only bias/drift is modeled; scale-factor error (the third standard IMU fault
   category per the aerospace VGG16 paper) isn't represented. Possible S5 tier,
   not urgent.

---

### 1.1 Recovery-sample validity (added 2026-07-22, review feedback)

A fault cycle can end two ways — `reason: "duration_cap"` (cap hit while still
inside the zone) or `reason: "zone_exited"` (vehicle left the zone before the cap).
**Only `duration_cap`-ended cycles are valid recovery-time samples.** If the fault
disarms because the vehicle already left the TL zone, there is nothing left for
ST-GAT to observe recovering *from* — the perception input reverts to "no TL"
either way, fault or not, so measuring recovery time from that `tl_fault_end`
would be measuring the zone exit, not the fault clearing. `zone_exited` cycles
still give a valid **reaction**-time sample (measured from `tl_fault_start`,
unaffected by how the cycle ends) — just not a recovery-time one.

Even a `duration_cap` ending doesn't guarantee much observation margin (the
vehicle could exit the zone one message later). The robust rule for analysis:
cross-reference each `tl_fault_end` against the next `tl_window_exited` log line
and only trust the recovery sample if `tl_window_exited.wall_time -
tl_fault_end.wall_time` exceeds some minimum (e.g. 3s) — both events are already
logged with wall/sim time in `fault_log.jsonl`, so this is a post-hoc filter, not
a code change.

## 3. Bridging to ST-GAT reaction/recovery analysis (once Phase 1 exists)

`fault_log.jsonl` already timestamps every transition
(`tl_fault_start`/`tl_fault_end`/`imu_bias_on`/`imu_bias_off`) in both wall time
and sim time — this is the alignment key for a future residual-vs-time plot:

- **reaction_time** = first sim_time after `*_fault_start`/`imu_bias_on` where the
  ST-GAT residual (raw/KL/CUSUM) crosses its detection threshold.
- **recovery_time** = first sim_time after `*_fault_end`/`imu_bias_off` where the
  residual falls back under threshold.

With the zone-triggered TL redesign, a single `nom_v11`-style route yields
multiple (reaction, recovery) pairs per trial for camera faults, matching what
the IMU on/off cycle already gives for free.

---

## 4. Which goals to run fault campaigns on

Measured directly, not estimated: replayed each collected `nom_v11` trial's
actual `/perception/traffic_light_recognition/traffic_signals` recording through
the exact same detection-window logic `fault_injector.py` uses (30-msg ring
buffer, enter ≥30% non-empty, exit <15%), counting real zone entries — i.e. how
many periodic TL-fault cycles the redesign above would actually fire on that
route. (An earlier attempt using a straight-line start→goal bounding box badly
overestimated TL exposure — up to 289 of the map's 383 total TL points — because
the bbox spans a large diagonal swath of this fairly compact, dense map
regardless of the actual road-following path; replaying the real driven
trajectory's TL topic avoids that.)

| Goal | Zone entries/trial | Notes |
|------|--------------------|-------|
| `goal_012` | 3, 3 | Tied for most in the whole 26-goal set |
| `goal_026` | 3, 3 | Tied for most in the whole 26-goal set |
| `goal_007` | 2, 2 | Mid-tier — tied with `goal_003/006/008/009/010`, not distinctly better than those |
| (all others) | 1–2 | Most goals see only 1 zone entry per trial |

### 4.1 Duration cap, derived from real dwell times (added 2026-07-22)

Finalized goals: **7, 12, 26**. Replayed the same three goals' real TL-topic
recordings again, this time measuring actual zone *dwell* durations
(entry→exit, not just entry count) to pick `--fault-duration` (the per-zone cap):

| Goal | Per-zone dwells observed (both trials, seconds) |
|------|--------------------------------------------------|
| `goal_007` | 73.4, 55.1 / 76.7, 56.3 |
| `goal_012` | 130.5, **20.9**, 108.4 / 93.2, **21.1**, 108.3 |
| `goal_026` | 136.4, 27.0, 41.8 / 221.7, 28.6, 38.1 |

Across all 16 closed zone-dwells: min 20.9s, p25 38.1s, median 73.4s, max 221.7s.
The binding constraint is the shortest zone (`goal_012`'s second intersection,
~21s reproducibly across both trials) — the cap must clear it with margin for
the fault to disarm via `duration_cap` (meaningful, still-in-zone recovery,
§1.1) rather than `zone_exited`. **Set `--fault-duration 15`** (down from the
old 45s default, which exceeded that 21s zone entirely — those cycles would
*always* end via `zone_exited`, i.e. never produce a valid recovery sample):
15s clears the 21s floor with ~6s margin, still gives `tl_oscillate` 3 full
5s periods, and every longer zone gets proportionally more recovery-while-
visible margin for free. Updated in `collect.sh`'s four `tl_fault_s1..s4`
cases and the design-comment block above them. This value is scoped to goals
7/12/26 specifically — re-derive it (same replay method, `docs/papers` n/a
here, just the recorded rosbags) if fault campaigns are ever extended to other
goals with different dwell characteristics.

**Goals 12 and 26 are the objectively best picks** — both tie for the highest
TL-zone-entry count of any goal collected so far, each giving 3 independent
fault/recovery cycles per trial. **Goal 7 is fine but not special** — it ties
with five other goals at 2 entries/trial; if a third goal is wanted specifically
*because* it's the best available (rather than for route-length/geometry
diversity or another reason), `goal_003`, `goal_006`, `goal_008`, `goal_009`, or
`goal_010` would be equally good substitutes at the same count. Kept goal_007 in
the plan since the difference (2 vs. slightly-more-of-the-same-tier) is minor and
Kalpit may have other reasons (route geometry, prior familiarity) for that pick.

---

## 5. Run commands (finalized 2026-07-22)

`run_fault_campaigns.sh` (repo root) wraps the 8-campaign for-loop into one
command — AWSIM + Autoware must already be running (same prerequisite check
`collect.sh` already has).

```bash
# Smoke test first: goal_007 only, 1 trial, all 8 fault campaigns
./run_fault_campaigns.sh --goals goal_007 --trials 1

# Once that looks right, the finalized run (goals 7/12/26, 3 trials, default):
./run_fault_campaigns.sh
```

Defaults baked into the script: goals `goal_007,goal_012,goal_026` against
`captured_goals.json`, 3 trials/goal, all 8 campaigns
(`tl_fault_s1..s4`, `imu_fault_s1..s4`). Override any of it with
`--goals`/`--goals-file`/`--trials`/`--campaigns "..."`/`--dry-run`; see the
script's own header for examples (e.g. running just the TL campaigns).
8 campaigns × 3 goals × 3 trials = 72 experiments for the full run; the smoke
test is 8 campaigns × 1 goal × 1 trial = 8 experiments. No automated Autoware
restart between campaigns (unlike `run_nominal_batches.sh`) — restart manually
between campaigns if runs start drifting.

### 5.1 Injector performance (added 2026-07-22)

Checked actual load before optimizing blindly: `/perception/traffic_light_recognition/traffic_signals`
runs ~20Hz, `/sensing/imu/imu_data` ~30Hz (measured from an existing goal_007
rosbag) — not extreme, but non-trivial over a 15–45s fault window. There isn't
much to *precompute* in the classical sense: which TL zone is active depends on
live perception, which is trial-to-trial non-deterministic, so the real-time
detection-window gating can't be replaced by an offline schedule without losing
correctness (the same reason the zone-triggered design uses live gating instead
of hardcoded coordinates in the first place — §1). What *was* real, fixable
per-callback overhead, now removed:

- **Dropped `copy.deepcopy(msg)`** in the three TL fault branches that mutate
  fields (`tl_confidence`, `tl_oscillate`, `tl_unknown`) — rclpy hands each
  callback a freshly-deserialized message no one else holds a reference to, so
  mutating it in place before publishing is safe and skips a full nested-object
  copy on every message while a fault is active (~20Hz × up to 15s/cycle).
- **Log file handle kept open** for the node's lifetime instead of
  open+write+close on every logged event (rare events, but still needless
  syscall pairs); now a single open in `__init__`, buffered write + flush per
  event, close on shutdown.
- **Incremental detection-ring count**: the non-empty-message count in the
  30-message sliding window is now updated by ±1 on push/evict instead of
  `sum(ring)` recomputed from scratch every single TL callback.

Verified the incremental ring-count change is exactly equivalent to the old
`sum()`-based version via a standalone randomized replay before committing to
it. IMU relay (`_on_imu`) was already minimal (two float adds, no copy) — no
change needed there.

---

## Papers downloaded (`docs/papers/`, gitignored)

| File | Contributes |
|------|-------------|
| `2024_matos_sensor_failures_survey.pdf` | Camera/IMU/LiDAR failure taxonomy (108-pub survey) |
| `2019_woodscape_lens_soiling_gan.pdf` | Real soiling patterns — image-level occlusion reference |
| `2025_droplet_contamination_mtf50.pdf` | Quantified MTF50-vs-droplet-volume severity curve |
| `2025_occluded_nuscenes.pdf` | Severity-tiered synthetic occlusion template |
| `2025_bevfusion_weather_occlusion.pdf` | Camera vs. LiDAR robustness under occlusion |
| `2022_vgg16_aerospace_imu_faults.pdf` | IMU bias/drift/stuck-at fault taxonomy + classifier analog |
| `2025_issta_multisensor_fault_tolerance.pdf` | Apollo-validated fault-severity framing |
| `2025_injecting_hallucinations.pdf` | Component-agnostic (vs. siloed) fault-injection argument |
| `2018_jha_avfi_fault_injection.pdf` | AV fault-injection methodology precedent (2-page DSN-W paper) |
| `2024_carla_autoware_bridge.pdf` | CARLA↔Autoware bridge (methodology comparison, short paper) |
| `2025_sensor_failure_sim_carla_autoware_thesis.pdf` | Full IMU/LiDAR/GNSS fault-injection framework on Autoware+CARLA — directly comparable to this project's own AWSIM+Autoware setup |

**Not obtained** (blocked by publisher bot-protection, no open-access mirror found):
Zhang et al. 2023 ISPRS adverse-weather survey (Elsevier, paywalled); "Performance
Degradation... Natural Visual Contamination" Computers 15(4):254, 2024 (MDPI,
blocked even via Wayback Machine — archived snapshot is a bot-check page, not the
PDF). Both are Discussion/Introduction-level citations, not needed for
severity-tier parameter values — lower priority to chase further.
