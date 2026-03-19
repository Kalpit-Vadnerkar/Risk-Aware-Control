# RISE Experiment Plan — March 2026

**Date:** 2026-03-05
**Status:** Active direction

---

## Research Context

RISE (Residual-Informed Safety Envelopes) uses CVaR computed from ST-GAT prediction
residuals to dynamically tighten control constraints before constraint violations occur.
The core claim: when the digital twin detects anomalous vehicle behavior (via residuals),
it can preemptively tighten velocity or safety margins to prevent an impending violation.

To validate this, we need experiments that:
1. Create real behavioral anomalies (measurable residuals)
2. Show that anomalies correlate with collision risk
3. Demonstrate that constraint tightening (RISE) prevents violations that would otherwise occur

---

## Key Design Decisions

### Decision 1: Disable MRM for Research Purposes ✅

**What we did:** Removed `perception` and `planning` from the `/autoware/modes/autonomous`
diagnostic gate in `autoware-main.yaml`.

**Rationale:** Autoware's MRM system triggers an emergency stop when perception or planning
components report diagnostic errors. During our experiments, any fault injection (dropout,
noise, delay, obstacle injection) caused diagnostic errors in those components, which
triggered MRM and locked the vehicle in a permanent emergency state (the "MRM wedge").

This meant the existing fail-safe system was preemptively "solving" the problem we are
trying to study. RISE is meant to be a *risk assessment layer* that operates before
violations occur — but MRM was preventing us from ever seeing what violations look like.

By disabling MRM for perception/planning, we:
- Remove the competing safety system that interferes with RISE evaluation
- Can study the vehicle's behavior under degraded conditions directly
- Can measure what happens when RISE is vs. is not active
- Preserve the localization/control/vehicle MRM gates (real hardware failures)

**This is the correct approach for research validation of a new safety module.**

### Decision 2: Perception Injection as Scenario Creation Mechanism ✅

**What we did:** Built a `PerceptionInterceptor` ROS2 node that sits between Autoware's
perception output and planning input. It can inject fake objects into the perception stream
or apply faults (dropout, delay, noise).

**Rationale:** Neither AWSIM Labs nor Autoware provide a convenient API for creating custom
risk scenarios programmatically during a run:
- AWSIM has no Python/ROS2 NPC scripting API
- Autoware's scenario system requires pre-authored OpenSCENARIO files per scenario
- Both approaches would require per-scenario simulator changes

The interceptor gives us full control over what the planner sees, without modifying
Autoware's internals. From Autoware's perspective, injected objects are indistinguishable
from real detected objects. This is intentional — we want Autoware to react as it would
to a real risk.

**Architecture:**
```
AWSIM → Autoware Perception → /objects (real detections)
                                       ↓
                             PerceptionInterceptor
                             [inject scenarios / apply faults]
                                       ↓
                         /objects_filtered → Planning → Control
```

All runs — including baseline — go through the interceptor in passthrough mode,
ensuring fair comparison across conditions.

---

## Experiment Structure

### Stage 1: Scenario Baseline (Current)

Run each risk scenario **without any perception faults**. Goal: characterize how
Autoware naturally handles each scenario type and what the resulting behavioral
signal looks like.

**Current finding (50m static obstacle, Mar 2026):**
- Autoware successfully navigates around the obstacle via lane change in all 6 runs
- Behavioral signal IS present: hard brakes +59–83% vs nominal, max decel up to 22 m/s²
- But: vehicle always completes the route — no blocking, no virtual collision
- **Implication:** 50–100m gives Autoware too much planning time; need 20–30m for
  stronger response

**Scenarios to cover:**

| Scenario | Distances / Params | Status |
|----------|-------------------|--------|
| Static obstacle | 20m, 30m, 50m, 100m × CAR/TRUCK | 🔄 In progress (50m done) |
| Cut-in | 40m/80m × 2s/4s duration | ⏳ |
| Slow lead vehicle | 50m, 30m, lead_speed 2–4 m/s | ⏳ Need to implement |
| Sudden close appearance | 10–15m, trigger after 5s | ⏳ Need to implement |
| Pedestrian crossing | 20m, perpendicular crossing | ⏳ Need to implement |

### Stage 2: Fault + Scenario Combinations

Apply perception faults on top of risk scenarios. Hypothesis: faults degrade Autoware's
ability to handle scenarios, producing outcomes closer to violations.

**Priority combinations:**

| Scenario | Fault | Hypothesis |
|----------|-------|------------|
| Static obstacle 30m | Dropout 30% | Obstacle intermittently invisible → late reaction |
| Static obstacle 30m | Delay 200ms | Detection late → harder braking or collision proxy |
| Static obstacle 30m | Position noise 0.5m | Avoidance displaced → closer pass |
| Cut-in 40m | Dropout 30% | Cut-in car not tracked consistently → tracking failure |

**Expected outcome:** Some fault+scenario combinations produce `min_object_distance < 2m`
(collision proxy) where the clean scenario did not. These are the cases RISE must prevent.

### Stage 3: RISE Validation

Re-run Stage 2 conditions with RISE active. RISE monitors ST-GAT residuals in real time
and tightens the velocity constraint when CVaR exceeds a threshold. Expected result:
the vehicle slows preemptively, maintaining clearance above the violation threshold.

**Comparison metric:** min_object_distance, collision_proxy_count, and MRM_SUCCEEDED rate
under (fault+scenario) vs. (fault+scenario+RISE).

---

## What Makes a Good RISE Scenario

For RISE to work, a scenario must satisfy:
1. **Create a real threat** — an object in the vehicle's path at a distance that requires
   a response
2. **Produce behavioral deviation** — the vehicle reacts differently than in nominal driving
   (braking, lane change, velocity change)
3. **Generate observable residuals** — the behavioral deviation must show up as ST-GAT
   prediction errors
4. **Have a fault condition that degrades the response** — the fault makes the vehicle's
   handling of the scenario worse
5. **Have a constraint that RISE can tighten** — slowing down earlier would have prevented
   the violation

The dropout sweep (Feb 2026) failed criterion 1 — NPCs yield to ego so no real threat.
The static obstacle at 50m meets criteria 1–3 but barely meets 4 (Autoware still handles it).
**20–30m obstacles and fault combinations are the correct next step.**

---

## Open Issues

1. **Route geometry unknown:** Are there single-lane segments on goals 007/011/021 where
   Autoware cannot lane-change? If not, a static obstacle will always be navigable.
   Need to check map data or observe rosbag lane-change events.

2. **TTC metric not useful for static scenarios:** TTC = ∞ when relative velocity = 0.
   Need a "closing time" metric: `(current_distance - safety_margin) / ego_speed`.

3. **Slow lead vehicle not yet implemented:** This scenario requires the injected object
   to have a persistent position that the ego vehicle closes on over time. Different from
   static obstacle (needs a well-chosen starting position far enough to create sustained
   interaction). Implementation is straightforward — extend ObjectFactory with forward vx.
