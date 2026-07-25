# Validation Strategy

**Last Updated:** 2026-07-24 — restructured around the belief-divergence /
calibrated-confidence / lead-time claim (see `TODO.md`,
`docs/theoretical_framework.md`, `docs/design_decisions.md`). The old
RISE-comparison validation plan (staged obstacle/fault scenarios, collision-proxy
metrics) is preserved in Section 6 as a deprioritized future-work appendix, not
deleted — it validates a control claim the dissertation is no longer defending as
its core contribution.

---

## 0. What This Document Validates

The claim: this framework detects when the autonomy stack's perceptual belief has
diverged from a map-grounded, independently derived expectation of the world; it
reports that divergence with calibrated confidence, a bounded number of seconds
of lead time before the divergence degrades vehicle safety.

Three things need validating, in priority order:

1. **The mechanism is real** (belief divergence / negative evidence) — Section 1,
   blocking.
2. **The confidence is trustworthy** (calibration) — Section 3.
3. **The warning is early enough to matter** (lead time) — Section 4, depends on
   the two-arm design (Section 2).

Sections 1–5 are the active validation plan. Section 6 is the deprioritized
RISE/control validation plan, kept for reference.

---

## 1. Priority 0 — Mechanism Validation (BLOCKING)

Mirrors `TODO.md`'s Priority 0 exactly; restated here as validation methodology.

### 1.1 Experiment A: Per-fault-class feature importance

**Method:** recompute the published T-ITS feature-importance analysis (Fig. 9/10)
sliced by fault class instead of aggregated.

**Pass/fail criterion:** traffic-light-CUSUM importance is selectively high for
Camera faults and low for IMU faults → mechanism supported. Uniform importance
across fault classes → mechanism is a clean-signal artifact, not evidence of
negative-evidence detection. This is a genuine falsification test, not a
confirmation exercise — a uniform result should be reported as a negative result,
not explained away.

**Data source:** published paper's existing results — see the sourcing gap noted
in `TODO.md` Priority 0 / P0.1.

### 1.2 Experiment B: HD map ablation on detection performance

**Method:** strip map-derived traffic-signal annotations from the ST-GAT's graph
node features, retrain, re-measure detection accuracy on Camera-fault trials
specifically (isolating the channel the map prior should matter for).

**Pass/fail criterion:** Camera-fault detection accuracy collapses without the
map prior → mechanism established. Accuracy holds → the model was using
deceleration/timing patterns instead, and the mechanistic claim needs revision.

**This is the single experiment the whole dissertation's mechanistic framing
depends on.** Report the outcome as-is, including a negative result — see
`TODO.md` for what a negative result implies for scope.

---

## 2. Two-Arm Experimental Design

Every fault-injection experiment below (Sections 1, 3, 4, 5) runs in both arms:

| Arm | Autoware safety features | Purpose |
|-----|---------------------------|---------|
| A | Disabled | Science condition — observe the full healthy → degraded → dangerous trajectory |
| B | Enabled | Ground-truth oracle — Autoware's own MRM trigger timestamp = labeled "unsafe" moment |

Arm B is not a baseline being competed against; it's the yardstick lead time is
measured against. **Open implementation gap:** Arm B's diagnostic-gate
configuration does not exist yet in this repo (see `docs/design_decisions.md`
item 5). Sections 3–5 below that depend on Arm B cannot run until it does.

---

## 3. Calibration Validation

**Method:** across the fault sweep (Section 5), bin the monitor's reported
confidence and compute empirical accuracy within each bin.

**Headline artifact:** a calibration curve (reported confidence vs. empirical
accuracy), not a single accuracy number. Report separately per fault type,
per fault magnitude tier, and per arm — calibration is allowed to hold in some
regions and break down in others; that's a finding, not a failure, as long as
it's characterized rather than hidden in an aggregate.

**Secondary check:** empirical coverage ≥ 0.94 at δ=0.05 (the original Phase 3
conformal-prediction coverage check), reported as a supporting number, not the
headline.

**Pass/fail:** there is no single pass/fail gate here — the deliverable is an
honest characterization of where calibration holds and where it breaks. A
defense-ready result includes the breakdown regions, not just the regions where
it works.

---

## 4. Lead-Time Validation

**Method:** for each fault trial run in both arms, measure the time between (a)
the framework crossing its calibrated-confidence threshold for "fault present"
and (b) Arm B's Autoware MRM trigger firing on the matched trial. Report as lead
time in seconds, not as an accuracy delta.

**Also vary:** history and prediction window lengths (the old "horizon study"
hyperparameter sweep) — report lead time as a function of window length, not as
a validation-set accuracy curve.

**Pass/fail:** no fixed numeric threshold is set here by design — the dissertation
claim is "a bounded number of seconds," and what that bound turns out to be *is*
the result, not a target to hit. A lead time near zero (the framework fires at
the same moment as Arm B) is a valid, reportable, if less impressive, outcome.

**Depends on:** Section 2's Arm B gap being resolved first.

---

## 5. Supporting Validation

### 5.1 Systematic fault sweep (type × magnitude)

Structured across fault type (Camera/TL, IMU, and — data-source permitting —
LiDAR) and magnitude (10%–75% degradation, per the T-ITS paper's flagged but
unaddressed limitation). Deliverable: the detectable boundary — the magnitude at
which residual signal disappears into nominal noise — and whether the model's
own confidence output tracks proximity to that boundary (a calibration check at
the edge, not just in-distribution).

### 5.2 Interpretability

Does residual attribution correctly localize the injected fault to the right
feature/channel? This is an independent line of evidence for the mechanism claim
(Section 1) — if attribution points at the TL channel for TL faults and the IMU
twist channel for IMU faults, that's corroborating evidence alongside Experiments
A and B, not just a nice-to-have visualization.

### 5.3 Generalization (map annotation density)

Contingent on Experiment B confirming the map prior is load-bearing. If so,
detection performance should be predictable from map annotation density/quality
— converting the current single-map (Nishishinjuku) limitation from an
unaddressed gap into a testable, principled statement of where the method
applies. Not runnable until Section 1's mechanism validation completes.

---

## 6. Deprioritized: Active-Control (RISE) Validation — kept for reference, not blocking

**Status as of 2026-07-24:** this section is the original validation strategy,
written for the active risk-aware control framing. Per the scoping decision in
`docs/design_decisions.md` item 10, it validates a control/handling contribution,
not the safety-verification claim under defense. Preserved as-is below for a
future control-extension chapter.

### 6.1 Phase 1: Nominal Baseline — ✅ COMPLETE (Mar 2026)

Run routes without any fault injection or scenario injection. Establish the floor
for safety and reliability metrics.

**Results:** 14/15 goal runs succeed (93%). MRM rate 124 ± 40 /km (transient,
< 0.15s duration). Zero near-misses. Mean velocity 4.46 m/s. See
`docs/research_notes/nominal_baseline_analysis_mar2026.md`. This result still
stands and is useful context (it characterizes nominal system behavior) even
though it was collected under the old framing.

### 6.2 Phase 2: Scenario Baseline + Fault Characterization

For each risk scenario (static obstacle, cut-in): scenario-only run to
characterize Autoware's natural handling; fault+scenario run to find combinations
producing near-misses or MRM events.

| Scenario | Parameters | Goal |
|----------|-----------|------|
| Static obstacle | 20m, 30m in-lane | Find blocking threshold |
| Cut-in | 40m ahead, 2s / 4s duration | Find tracking failure conditions |
| Fault overlays | Dropout 30%, position noise 0.5m | Degrade Autoware handling |

**See:** `docs/research_notes/rise_experiment_plan_mar2026.md`,
`docs/research_notes/scenario_framework.md`.

### 6.3 Phase 3: RISE Comparison

Re-run Phase 2 fault+scenario combinations with RISE active. Compare
`min_object_distance`/collision proxy count, MRM_SUCCEEDED rate, and route
completion time (fault+scenario vs. fault+scenario+RISE).

### 6.4 What Constitutes a Useful Control-Track Test Scenario (unchanged)

1. Creates a real threat (object in path at a distance requiring response)
2. Produces observable behavioral deviation (braking, velocity change)
3. Generates measurable residuals in the ST-GAT signal
4. Has a fault condition that degrades response
5. Has a constraint RISE can tighten

The Feb 2026 perception dropout sweep failed criterion 1 (no real collision
threat); static obstacles at 20–30m with fault overlays were the corrected
design.

### 6.5 Control-Track Metrics (unchanged)

**Safety:** `min_object_distance`/collision proxy count, `min_closing_time`, MRM
rate/duration.
**Reliability:** route completion rate, mission time vs. nominal, mean velocity.
**RISE-specific:** residual-spike-to-constraint-tightening latency, velocity at
closest-approach, false-positive tightening rate on nominal runs.

### 6.6 Control-Track Success Criteria (unchanged)

| Criterion | Threshold |
|-----------|-----------|
| Collision proxy reduction (fault+scenario) | Statistically significant vs. no-RISE |
| Route completion rate with RISE | ≥ baseline |
| False positive rate on nominal runs | < 5% of timesteps |

### 6.7 Fault Types (as originally scoped — superseded, see `docs/design_decisions.md` item 7)

The original scope below excluded IMU/LiDAR ("sensor-level faults") — this is now
stale; IMU fault campaigns are core to Section 1's mechanism validation.
Preserved here only as historical record of what Phase 2/3 above were actually
tested against.

| Fault | Implementation | Severity Range |
|-------|----------------|----------------|
| Object detection dropout | Drop N% of objects randomly | 10%, 30%, 50% |
| Position noise | Add Gaussian noise to object positions | σ = 0.25m, 0.5m, 1m |
| Object delay | (planned) Buffer objects by Nms | 100ms, 200ms |
