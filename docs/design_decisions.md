# Design Decisions

**Last Updated:** 2026-07-24 — restructured around the belief-divergence /
calibrated-confidence / lead-time reframe (see `TODO.md` and
`docs/theoretical_framework.md`). Decisions 1–4 below (all RISE/active-control
design decisions) are marked **legacy** — not wrong, just no longer decisions
that bear on the claim under defense. Decision 7 is marked **stale/superseded**
— it's directly contradicted by work already done in this repo. New decisions
(8–11) cover the mechanism, the two-arm design, and the scenario-avoidance
scoping call.

---

## 1. Constraint Tightening vs Weight Modulation — LEGACY (control-track, not blocking)

**Decision:** Use constraint tightening, not MPC weight modulation, *if and when*
the active-control future-work chapter (Section 6 of `theoretical_framework.md`)
is picked back up.

**Rationale (unchanged, kept for reference):**
- Constraint tightening is interpretable in physical units (meters, m/s)
- Provides a path to formal probabilistic guarantees
- Weight modulation has no clear relationship to violation probability and no
  principled way to set the weights from a risk signal
- Stronger research contribution (ties to robust/tube MPC literature)

**Status:** not a defense-blocking decision as of 2026-07-24 — see `TODO.md`'s
scoping corollary. Kept as a sound decision *for the control chapter*, not for
the verification claim.

---

## 2. Residual-to-Constraint Mapping — LEGACY (control-track, not blocking)

**Original decision:** the mapping from residual anomaly signal to constraint
value was left open pending Phase 2 calibration data (CVaR-based threshold,
uncertainty propagation, or empirically calibrated mapping).

**Status:** superseded as a priority by Decision 9 below (residual-to-*calibrated-
confidence* mapping is now the core question; residual-to-*constraint* mapping is
downstream future work, not on the critical path).

---

## 3. Handling Detection Latency — PARTIALLY SUPERSEDED

**Original problem framing:** the residual signal rises as fault effects
accumulate — there is inherent latency before the signal is strong enough to act
*on* (i.e., for a controller to react to).

**Original solution (control-track):** trend-based early warning, scenario
targeting (tight-reaction-window obstacles), continuous margin framing ("RISE...
is not detecting a fault and reacting — it is always active and always
calibrating").

**What survives the reframe:** the trend-based / early-warning idea is exactly
what the **lead-time** pillar of the new claim needs (`TODO.md` Priority 1, P1.3
Horizon study) — but the target is no longer "soon enough for a controller to
act," it's "how many seconds before Autoware's own MRM trigger fires (Arm B),
measured and reported, not consumed by a controller." Same underlying signal,
different consumer and different success criterion (a lead-time number with a
calibration curve, not a constraint-tightening trigger).

---

## 4. Fail-Operational Design — LEGACY (control-track, not blocking)

**Original decision:** RISE must preserve mission completion, not just prevent
violations — graceful degradation over binary stop/avoid.

**Status:** this was a design goal *for the controller*. Under the current claim,
there is no controller under defense, so "mission completion" is not a metric the
dissertation is scored against. Kept as a real, sound design principle for the
future-work control chapter.

---

## 5. MRM Gating (Disabled for Research) — UPDATED, now the Arm A / Arm B question

**Original decision (unchanged, still in effect):** removed `perception` and
`planning` (and, per README.md item 3, `localization`) from the Autoware MRM
diagnostic gate.

**Original rationale (still valid):** Autoware's built-in MRM triggers an
emergency stop when perception or planning report diagnostic errors. Any fault
injection caused those diagnostics to fire, triggering MRM and amputating the
fault before it could be studied.

**Reframed 2026-07-24:** this configuration is exactly **Arm A** in the two-arm
fault-injection design (`docs/theoretical_framework.md` §4, `TODO.md`) — the
"science condition" that lets a fault propagate to its full effect. What the
original decision record didn't anticipate is **Arm B**: a configuration that
restores Autoware's full/stock diagnostic gate so its own MRM trigger can serve
as a labeled, ground-truth "this was objectively unsafe" timestamp for measuring
lead time.

**Gap, not yet resolved:** Arm B does not exist as a configuration in this repo.
Building it without reintroducing the MRM deadlocks that motivated the original
gating change (README.md items 3/4/7 — routing resets, TF drops during
teleports, rosbag2_recorder double-registration, EKF twist-queue overflow) is
real engineering work. Likely needs to preserve the teleport/reset-specific
exclusions (localization TF drops, routing resets between trials — these are
experiment-harness artifacts, not genuine safety signal) while restoring the
perception/planning links that actually carry the "is this unsafe" signal Arm B
needs. Not solved by this documentation pass.

**What is preserved (unchanged):** control and vehicle MRM gates remain active in
both arms as originally decided.

---

## 6. Perception Injection as Scenario Mechanism — unchanged, still active

**Decision:** Use PerceptionInterceptor (ROS2 node) for all scenario creation and
fault injection, rather than AWSIM source modification or OpenSCENARIO.

**Rationale (unchanged):**
- AWSIM has no Python/ROS2 NPC scripting API
- OpenSCENARIO requires per-scenario pre-authored files and a full Autoware rebuild
- The interceptor gives full programmatic control without modifying simulator internals
- From Autoware's perspective, injected objects/faults are indistinguishable from
  real detections
- All runs go through the interceptor in passthrough mode, ensuring fair
  comparison

**Status:** still the correct infrastructure decision under the new framing —
this is *how* faults get injected for Priority 0/1 work, independent of whether
the consumer is a controller or a calibration/verification pipeline.

---

## 7. Fault Scope: Perception Layer Only — STALE, SUPERSEDED BY ACTUAL WORK

**Original decision (March 2026):** focus fault experiments on the perception
layer; raw sensor faults (LiDAR noise, IMU bias, localization drift) out of
scope.

**Original rationale:** ST-GAT behavioral residuals respond to what happens
*after* perception, not raw sensor noise; sensor-level faults have an indirect,
confounded signal path.

**This is directly contradicted by work already done in this repo.** IMU gyro-bias
fault campaigns (`imu_fault_s1..s4`) have been running since at least
2026-07-22/23 (see `TODO.md` §0.3) and are core to the current mechanism work —
IMU is explicitly one of the two fault classes (with Camera) that Experiment A
(`TODO.md` Priority 0) needs to disambiguate the negative-evidence claim.
IMU is a sensor-level fault by the original decision's own definition.

**Corrected scope statement (2026-07-24):** fault scope is not "perception layer
only" — it is **whatever channel the mechanism claim is about**: Camera (via the
TL detection channel, where the map prior is hard) and IMU (via EKF twist/heading,
where no map prior exists — the negative control the mechanism needs) are both in
scope and both required. LiDAR (via the object-detection-flag channel, soft
prior) is in scope for Experiment A using the published paper's existing data,
and an open question for new data collection in this repo (see `TODO.md`'s
"Explicitly Out of Scope").

**Why this matters for the defense:** the March 2026 rationale — "sensor faults
have an indirect, confounded path" — is now something the dissertation needs to
directly address rather than route around, since IMU fault detection quality
(and its *absence* of a map-grounded negative-evidence channel) is exactly the
contrast case that makes Experiment A's prediction falsifiable. Scoping IMU out
would have quietly removed the ability to disprove the mechanism.

**See:** `docs/research_notes/dropout_sweep_analysis_mar2026.md` for the original
finding that motivated this now-superseded pivot — kept as historical record.

---

## 8. Mechanism Choice: Negative Evidence vs. Generic Anomaly Detection (new, 2026-07-24)

**Decision:** frame the detection mechanism as belief divergence against a
map-grounded prior (with negative-evidence detection as its strong form), not as
generic residual-based anomaly detection.

**Rationale:** "residuals spike under fault" is true but not explanatory — it
doesn't predict *which* features should be most discriminative, or *why* LiDAR
fault confuses with Camera/IMU more than Camera and IMU confuse with each other.
The map-grounded-prior framing makes falsifiable, feature-specific predictions
(Experiment A) and a load-bearing structural prediction (Experiment B) that a
generic anomaly-detection framing does not.

**What is NOT considered a substitute:** describing the same empirical result
("TL Status Flag is most important") without the mechanistic claim behind it —
that's the T-ITS paper's own framing, and the dissertation's stated job is to go
beyond it.

**For defense:** "The mechanism is falsifiable, not just descriptive — Experiment
B either collapses Camera-fault detection when the map prior is removed, or it
doesn't. Either outcome is a real result."

---

## 9. Calibration as a Core, Non-Deferred Contribution (new, 2026-07-24)

**Decision:** calibrated confidence (Section 3, piece 2 of the claim) is a
required contribution, not a candidate mechanism that might be dropped if it
doesn't pan out (as the 2026-07-22 TODO.md language had it).

**Rationale:** a fault detector that reports "fault" or "no fault" with no
confidence signal cannot support a safety-verification claim — a wrong-but-
confident monitor is worse than no monitor, because it invites misplaced trust.
The calibration curve (reported confidence vs. empirical accuracy) is what turns
a classifier into something a safety case can cite.

**What is NOT considered:** distribution-free conformal prediction as the *only*
acceptable calibration mechanism. The ST-GAT's native Gaussian/Bernoulli outputs
are evaluated on equal footing (see `TODO.md`'s "Distributions over
distribution-free").

---

## 10. Scenario-Based Avoidance: Deliberately Scoped Out of the Verification Claim (new, 2026-07-24)

**Decision:** static-obstacle and cut-in avoidance demonstrations (`obs_*`
campaigns, the old RISE Phase 2/3/4 validation work) are not evidence for the
safety-verification claim and are not blocking for the defense.

**Rationale:** the claim under defense is entirely about *detecting and
reporting* belief divergence with calibrated confidence and measured lead time.
Whether that signal is subsequently used to relax a velocity constraint and
successfully avoid an obstacle is a *downstream control question* — a real and
interesting one, but a different claim, requiring its own validation (does the
constraint-relaxation logic correctly trade off safety and progress?) that has
no bearing on whether the detection/calibration/lead-time claim is true.

**What is preserved:** the infrastructure (PerceptionInterceptor, `obs_*`
campaign definitions, `collect.sh` commands) is left in place and working — this
is cheap to keep and expensive to rebuild if the control chapter is picked back
up later. Just not run or analyzed as part of the current priority queue.

**For defense, if asked "why no closed-loop avoidance demo":** "That's a control
contribution, and it's future work by design — the claim being defended here is
about detection quality, calibration, and lead time, which are independently
measurable without closing the loop into a controller."

---

## 11. Two-Arm Fault Injection as the Validation Design (new, 2026-07-24)

**Decision:** all fault-injection experiments run in both Arm A (Autoware safety
disabled) and Arm B (Autoware safety enabled, used as ground-truth oracle),
rather than comparing against an external/synthetic ground-truth label.

**Rationale:** Autoware's own reactive MRM trigger is a real, already-implemented,
domain-appropriate definition of "this became unsafe" — using it as the oracle
avoids inventing and defending a separate synthetic ground-truth criterion, and
directly supports the "lead time before Autoware's own hard trigger" framing of
the claim.

**What is NOT considered:** treating Autoware's MRM as a *competing baseline* to
be beaten on accuracy — it's reactive and rule-based by design, this framework is
predictive and epistemic; they're not doing the same job, so a head-to-head
accuracy comparison would be a category error. See
`docs/theoretical_framework.md` §4.

**Open gap:** Arm B's diagnostic-gate configuration doesn't exist yet (Decision
5, above). This decision documents the *design*, not a completed implementation.
