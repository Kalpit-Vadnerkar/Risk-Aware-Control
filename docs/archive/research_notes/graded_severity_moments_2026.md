# Multiple severity moments, not one "fatal moment" (2026-08-04)

**Prompted by:** Kalpit's push that a single binary fatal-moment anchor is
the wrong shape for this claim — asked whether published work supports
staging severity into multiple moments, and whether Autoware's own source
already has something to reuse. Short answer to both: yes.

## What's already staged inside Autoware itself (strongest, most direct source)

Found while investigating the fatal-moment redesign (2026-08-04):

- **`autoware_mrm_handler`** (`autoware/src/universe/autoware_universe/system/
  autoware_mrm_handler/src/mrm_handler/mrm_handler_core.cpp`,
  `getCurrentMrmBehavior()`) already escalates through staged responses, not
  a single trigger: `COMFORTABLE_STOP` → `PULL_OVER` → `EMERGENCY_STOP`,
  chosen by what's still available given current system health. This is a
  graded severity ladder that already exists in the system under study, not
  something to invent.
- **`autoware_control_validator`** (`config/control_validator.param.yaml`)
  stages its own checks two ways: `yaw_deviation_warn: 0.5` rad vs.
  `yaw_deviation_error: 1.0` rad — a WARN level and a separate, more severe
  ERROR level for the same underlying measurement, not one cutoff.
- Both are already recorded in every collected bag's `/diagnostics` topic
  (confirmed directly, see `compute_fatal_moments.py`'s module docstring) —
  usable without any Arm B collection, though (also confirmed) they check
  self-consistency, not ground-truth correctness, so they're a severity-
  staging *pattern* to borrow, not numbers to reuse directly for a
  localization/perception fault's ground-truth deviation.

## External grounding for the same idea

- **ISO 21448 (SOTIF)** and **ISO 26262**: both formalize hazard analysis as
  graded (severity × exposure × controllability for ASIL; SOTIF separately
  stages "hazardous behavior" from "harm"), never a single binary
  safe/unsafe cut. Standard engineering practice for exactly this reason —
  citable as the established norm, not a novel choice on our part.
- **RSS — Responsibility-Sensitive Safety** (Shalev-Shwartz, Shashua, Shammah,
  "On a Formal Model of Safe and Scalable Self-Driving Cars," 2017,
  arXiv:1708.06374): formally defines a "dangerous situation" via explicit
  safe-distance/proper-response conditions, distinct from an actual
  collision — a graded, formally-defined intermediate severity level between
  "nominal" and "crash," directly analogous to what a lane-deviation
  threshold is trying to capture here.
- **Surrogate Safety Measures** (traffic engineering literature already
  touching this project via the retired slide 3's TTC/PET citations —
  Hayward's Time-to-Collision, Post-Encroachment Time, Deceleration Rate to
  Avoid Crash): a whole toolkit of graded "how close to disaster" measures,
  each capturing a different point along a severity continuum rather than a
  single crash/no-crash label. Directly supports treating "first behavioral
  deviation," "first hard safety-margin violation," and "irrecoverable
  outcome" as three genuinely different, all-legitimate moments rather than
  competing definitions of one thing.
- **Runtime assurance / Simplex architecture** (Sha, 2001; broader runtime-
  assurance literature, already cited in `active_rectification_feasibility_
  2026.md`): monitor-and-switch architectures are built around staged
  responses (warn → degrade → hand off to a certified fallback), the same
  shape as Autoware's own MRM ladder above.
- **AV safety/incident reporting practice** (California DMV's mandatory
  disengagement reporting; NHTSA's standing general order on ADS/L2+ crash
  reporting): both require graded severity categories in practice (property
  damage / injury / fatality / no-harm disengagement), not a single
  bucket — real-world regulatory precedent that "was there harm" is never
  asked as one binary question.

## What this means for this project's fatal-moment definition

Recommend a staged trace, not a single anchor — reusing the same four-level
shape Autoware's own MRM ladder and this literature both converge on:

1. **Behavioral onset** — first calibrated alarm crossing (already built,
   Stage 4's conformal threshold).
2. **Hard safety-margin violation** — first sustained EKF-vs-ground-truth
   deviation past a real threshold (`compute_fatal_moments.py`'s
   `lane_deviation_crossing_s`, sweep in progress to pick the value).
3. **System response** — Autoware's own MRM state, if/when Arm B exists to
   make it trustworthy (right now flapping too much in Arm A to use, per
   prior notes).
4. **Irrecoverable outcome** — the existing stuck/collision heuristic
   (`first_stop_s`, forward-scan-corrected version).

Lead time can then be reported as a set of gaps between adjacent stages (1→2,
2→4, etc.) instead of one number against one anchor — closer to what the
literature above actually treats as the graded-severity norm, and more
informative for the dissertation than collapsing everything into a single
"fatal moment."
