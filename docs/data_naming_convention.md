# Data collection naming convention

Added 2026-10-03, alongside the train/cal split-assignment hardening (see
`st_gat/pipeline/goal_split.py`'s docstring and
`experiments/scripts/manage_goal_split.py`). This file does two things:
states the convention for **new** campaigns going forward, and gives a
glossary for the **existing** (legacy, ad hoc) ones — nothing on disk is
renamed by this file; it only documents what's already there and sets a
rule for what gets collected next.

Confirmed directly (so this convention doesn't accidentally break
anything): campaign directory names are mostly opaque strings to the
code — `collect.sh` dispatches via an exact-match `case` statement (any
new campaign needs its own arm there regardless of naming scheme), and
severity/other parameter values are read from each trial's own
`fault_log.jsonl`, never parsed from the directory name. The one real
constraint is **fault-kind classification** (`tl` vs. `imu`), which used to
be guessed from the name string in several scripts — fixed 2026-10-03 to
read it directly from `fault_log.jsonl`'s own `tl_fault`/`imu_fault` fields
instead (`experiments/lib/fault_log.py::campaign_fault_kind`), so a new
fault-family name imposes no naming constraint on this axis either.

## Convention for new campaigns

Form: `<imu_fault|tl_fault>_<profile>[_<param>]`.

- `<profile>` is a short, descriptive word — follow the existing
  `fixed`/`ramp`/`scale`/`stuck` examples, not the older bare scenario
  numbers (`s1`/`s2`/`s3`/`s4`) below, which are exactly why this glossary
  has to exist in the first place.
- `<param>`, if the profile has one tunable literal value (e.g. a fixed
  severity), is a zero-padded mnemonic — e.g. `tl_fault_fixed_030` for
  `confidence_scale: 0.30`.
- **Rule: the name is a human mnemonic only, never the parsed source of
  truth.** Any literal value implied by the name (a severity, a density,
  any future axis) must also be logged as a real field in that trial's
  `fault_log.jsonl`/`metadata.json`. This is already how the fixed-severity
  campaigns correctly work (`confidence_scale` in `fault_log.jsonl`,
  read directly by `tl_severity_sweep_analysis.py` — not reconstructed from
  the directory name). The next new axis should follow the same pattern
  from the start rather than reinventing it under lab-session time
  pressure — NPC/traffic density (config-driven as of the 2026-09-03
  session, see `CLAUDE.md`'s gotchas) is the concrete example to watch
  for: if it's ever collected, its configured value belongs in
  `metadata.json`, and the campaign/dataset name carries at most a
  mnemonic.

## Glossary — existing (legacy) campaign names

Full detail, including zone-gating geometry and expected outcome per row,
is in `docs/fault_scenario_table.md` — this is just the one-line decoder.

| Directory name | Real fault | One-line meaning |
|---|---|---|
| `nom_v11` | none | Current nominal baseline, 26 goals, 1-2 trials/goal, collected 2026-07-21/22 at single (max map-limit) velocity. `baseline_all` (an earlier, now-superseded nominal collection) still appears in `NOMINAL_DATASETS`/the split manifest for backward compatibility but has no raw data left on disk. |
| `imu_fault_s1` | `imu_bias`, gyro +0.03 rad/s | Negative control — bias small enough to be absorbed by EKF noise rejection, expected to show no effect even through a turn. |
| `imu_fault_s3` | `imu_bias`, gyro +0.08 rad/s | Bias large enough to integrate against real turn yaw rate into a visible EKF-vs-GT heading divergence. |
| `imu_fault_scale` | `imu_scale_factor`, gyro ×1.8 | Scale-factor error, proportional to true yaw rate — by construction, zero effect on straights, turn-triggered. |
| `imu_fault_stuck` | `imu_stuck_at`, gyro frozen at activation value | Frozen gyro stops tracking yaw rate changes from the moment of freezing — harmless if frozen during a straight, divergent mid-turn. |
| `tl_fault_s2` | `tl_oscillate`, 5s period | Oscillating reported TL color — confuses the stop/go decision specifically near a real signal. |
| `tl_fault_s3` | `tl_unknown` | Forces an UNKNOWN classification — over-cautious (unwarranted stop) behavior at a real signal. |
| `tl_fault_s4` | `tl_blackout` | No signal reported at all — forces the planner's no-information fallback at a decision point. |
| `tl_fault_ramp` | `tl_confidence_ramp`, confidence decays 0.1/s | Continuous confidence decay rather than a switch — tests threshold-crossing behavior. **Known confound** (2026-08-26): conflates severity with elapsed-time-in-zone/physical proximity to the intersection — this is exactly why the next campaigns use fixed severities instead. |
| `tl_fault_fixed_030`/`_050`/`_070` | `tl_confidence`, fixed at 0.30/0.50/0.70 | Planned (TODO.md §1), not yet collected as of 2026-10-03 — fixed-severity replacement for `tl_fault_ramp`'s confound, isolates severity from time-in-zone. |
