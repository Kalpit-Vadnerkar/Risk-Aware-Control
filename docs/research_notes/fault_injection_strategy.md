# Fault Injection Strategy

**Last Updated:** 2026-03-20

> **Note:** This document describes the implemented approach. The original planning doc
> (pre-Feb 2026) proposed a generic ROS2 fault injection layer and covered localization
> and sensor faults. That direction was superseded — see below.

---

## Implemented Approach: PerceptionInterceptor

All fault injection and scenario creation is done through the `PerceptionInterceptor`
ROS2 node. This node sits between Autoware's perception output and the planning input,
giving full control over what the planner sees.

```
AWSIM → Autoware Perception → /perception/object_recognition/objects
                                          ↓
                                PerceptionInterceptor
                                [inject scenarios + apply faults]
                                          ↓
                          /objects_filtered → Planning → Control
```

**Key design properties:**
- All runs, including baseline, go through the interceptor in passthrough mode (fair comparison)
- Planning always reads from `objects_filtered` (configured in
  `tier4_planning_component.launch.xml` line 11)
- Interceptor must be running for planning to receive any objects
- Fault and scenario are **composable**: the interceptor applies a scenario (inject object)
  then overlays a fault (drop some objects) in a single pass

**See `experiments/lib/perception_interceptor.py` for implementation.**

---

## Why Not Sensor-Level Faults (LiDAR, IMU)

The original strategy considered injecting faults at the raw sensor level (LiDAR dropout,
IMU noise, localization drift). This was deprioritized for the following reasons:

1. **Wrong research question:** RISE operates on behavioral residuals (position, velocity,
   steering) from the ST-GAT digital twin. Raw sensor faults (LiDAR noise) affect the
   perception pipeline before ST-GAT sees them — the signal path is indirect and
   confounded by Autoware's internal processing.

2. **Dropout pivot finding:** The Feb 2026 perception dropout sweep showed that NPCs
   in AWSIM yield to ego — no real collision threat is created even at 100% dropout.
   Sensor-level dropout would have the same problem plus an indirect signal path.
   See `docs/research_notes/dropout_sweep_analysis_mar2026.md`.

3. **Perception scenarios are more direct:** Injecting a static obstacle or cut-in
   directly into the perception stream creates a clear, reproducible collision threat.
   The vehicle must react; faults on top of that test how degraded perception affects
   that reaction.

---

## Active Fault Types

### 1. Perception Dropout
Drop N% of objects randomly per message. Simulates intermittent detection failures
(occlusion, weather, sensor limitations).

**Parameters:** `drop_rate: 0.0–1.0`
**Active range:** 30% and 50% for scenario experiments

### 2. Position Noise
Add zero-mean Gaussian noise to each object's position. Simulates uncertain localization
of perceived objects.

**Parameters:** `position_noise_std: σ in meters`
**Active range:** σ = 0.25m, 0.5m, 1.0m for scenario experiments

### 3. Object Injection (Scenarios, not fault)
Inject a synthetic object at a specified distance along the planned trajectory (static
obstacle or cut-in). See `docs/research_notes/scenario_framework.md` for details.

---

## Fault + Scenario Composition

The interceptor supports overlaying a fault on top of a scenario:

```bash
# Example: static obstacle at 30m + 30% dropout
python run_scenario_sweep.py \
  --scenario static_obstacle \
  --fault-strategy perception_dropout \
  --fault-params '{"drop_rate": 0.3}'
```

The fault is applied **after** injection — the injected obstacle can also be dropped,
which tests the edge case where the primary threat is intermittently invisible.

---

## Out-of-Scope Faults (Deferred)

| Fault Type | Reason Deferred |
|------------|----------------|
| LiDAR sensor dropout | Indirect signal path; AWSIM source modification needed for realism |
| IMU noise / bias | Affects localization, not ST-GAT behavioral residuals directly |
| Localization drift | Out of MRM gate scope; requires different residual signal |
| Camera blackout | Traffic light detection only; not primary risk signal |
| OpenSCENARIO NPC scripting | Complex setup; PerceptionInterceptor achieves same goal |

These may be revisited if RISE is extended beyond behavioral residuals, but are not
needed for the current research question.
