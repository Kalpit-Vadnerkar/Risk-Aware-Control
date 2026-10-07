# Fault-scenario redesign plan (2026-10-05) — draft for the next lab session

## Why the current data can't support the Layer 2 claim
The claim needs (a) real consequence events (positives) from more than one
fault family, (b) the same fault producing different consequences in
different geometry (that's what Layer 2 adds over an anomaly detector),
(c) matched nominal controls for false-alarm rates, (d) enough positives
for a reliability diagram of P(violation) (rule of thumb: >= 30 events
spread over the P range).

Current fault data (38 trials, Arm A, NPC density 10):

| family | trials | real consequences (GT) | notes |
|---|---|---|---|
| IMU (bias 0.03/0.08, scale 1.8, stuck) | 8 | 4 road departures | 1 trial per mode x goal_007/012 only |
| TL (oscillate, unknown, blackout, confidence ramp, fixed confidence x3) | 30 | 0 | every mode is fail-safe by Autoware design |

**TL faults as implemented cannot produce a safety consequence.** An
unknown / blacked-out / oscillating / low-confidence light makes the
planner stop or hesitate — the conservative failure. The dangerous
perception failure, a red light reported as green (or a stale green), was
never implemented. Whatever consequence TL faults have now is mobility
(unneeded stops), not safety.

**Ground truth already available in every bag:** AWSIM pose
(`/awsim/ground_truth/localization/kinematic_state`), and BOTH the faulted
(`traffic_signals_faulted`, what the planner used) and unfaulted
(`traffic_signals`, what perception saw) signal topics. So a second
consequence type — **ego crosses a stop line while the real light is red**
— is labelable without new collection.

## Proposed experiment matrix (prioritized)

**P1. IMU: severity x geometry (most direct value, no new code).**
- Modes: gyro bias {0.02, 0.04, 0.08} rad/s (fixed, not cycling), scale
  1.8, stuck — 5 modes.
- Geometry contrast: pick goals so the same fault onset lands (i) on a wide
  straight, (ii) approaching a turn / island / narrow segment. Use the
  existing zone-gating (bias-leadin / turn zones) for (ii) and an
  open-road gate for (i).
- 3 trials per cell. Minimum useful set: 5 modes x 2 geometries x 3 = 30;
  across 2 goals per geometry = 60.
- Expected: departures concentrated in (ii) at moderate/high severity,
  rare in (i) — the anomaly-vs-consequence separation Layer 2 is supposed
  to show.

**P2. TL: add a false-green fault + a stop-line safe-set term.**
- New `fault_injector.py` mode `tl_false_green`: when the governing light
  is red, publish green on `traffic_signals_faulted` (zone-gated like the
  existing TL faults). Arm A so it isn't masked.
- New Layer 2 safe-set term: crossing the governing stop line while the
  unfaulted signal is red, within H. Map-grounded (lanelet2 traffic-light
  regulatory element ref_line), ground truth from the unfaulted topic.
- Before collecting: run that labeler over the existing 30 TL + 45 nominal
  trials (should be ~0 positives; validates the labeler).

**P3. Cross traffic (object-conflict consequences).** NPC density > 10 —
blocked on the one-time AWSIM Unity rebuild (config plumbing already
patched). Needed for false-green to produce conflicts, and to exercise the
object half of the safe set at all.

**P4. Arm B subset.** A few P1/P2 cells with the stock MRM gate, to compare
Layer 2 lead time against Autoware's own MRM trigger (the original oracle
design). Watch for the known MRM deadlocks.

**P5. Nominal controls.** >= 4 nominal trials per goal used in P1/P2, same
start/goal, so per-goal false-alarm rates are measured, not borrowed.

## Analysis-side work (can start now)
1. Stop-line / red-light labeler + Layer 2 regulatory term (P2 prep).
2. Reliability diagram for P(violation) once positives exist.
3. Per-family reporting: IMU (localization) faults act through the pose
   coupling; TL faults would act through behaviour (the forecast itself),
   so they need the forecast to reflect the faulted perception — check
   that the model's TL inputs use the faulted topic (they do: config
   TOPICS['traffic_lights'] = traffic_signals_faulted).

## Open questions for Kalpit
- Which goals give the best wide-straight vs tight-geometry contrast?
  (Candidates from the IMU departures: goal_007's island-adjacent turn.)
- Is a false-green fault acceptable to add to the injector (it's the
  safety-relevant TL failure, but a new fault mode)?
- Arm B: worth the deadlock risk this round, or after P1/P2?
- Unity rebuild for NPC density: who/when?
