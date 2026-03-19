# Risk-Aware Control — Task List

**Last Updated:** 2026-03-17

---

## Development Philosophy

1. **Work in small chunks** — Pick specific tasks, not entire phases
2. **Validation-first** — Know what we're measuring before building
3. **Document decisions** — Rationale for committee defense
4. **Scenarios first, faults second** — Establish clean risk scenarios, then degrade perception

---

## Phase 0: Environment Setup ✅ COMPLETE

- [x] AWSIM + Autoware verified working
- [x] ROS2 topics documented
- [x] Constraint parameters identified
- [x] MRM wedge diagnosed and fixed (removed perception/planning from diagnostic gate)
- [x] Autoware headless mode working

---

## Phase 1: Experiment Infrastructure ✅ COMPLETE

- [x] Experiment runner with goal tracking, stuck detection, metrics
- [x] PerceptionInterceptor ROS2 node (6 strategies: passthrough, static_obstacle, cut_in, delay, dropout, noise)
- [x] Scenario YAML config system + parameter sweep runner
- [x] Metrics framework (safety, reliability, fail-operational, comfort)
- [x] Campaign-based data organization
- [x] `run_sweep.sh` convenience launcher (sources ROS2 + Autoware automatically)
- [x] Trajectory-based obstacle placement (obstacles placed on actual planned route, not heading projection)
- [x] Fault+scenario composition (`--fault-strategy` overlay on interceptor)
- [x] Closing-time metric (`min_closing_time`) — TTC valid for static obstacles
- [x] Metrics reads from `objects_filtered` — injected obstacles now tracked in safety metrics

**Key files:**
- `experiments/lib/perception_interceptor.py` — Core injection node
- `experiments/lib/scenarios.py` — ObjectFactory, ObjectPlacer, ScenarioConfig
- `experiments/scripts/run_scenario_sweep.py` — Sweep runner
- `run_sweep.sh` — Shell launcher (no manual sourcing needed)

---

## Phase 2: Scenario Baseline Experiments 🔄 CURRENT

### Goal
Establish how Autoware behaves under risk scenarios **without any perception faults**.
This gives us the "expected risk behavior" baseline that RISE will be measured against.

### 2.1 Static Obstacle Sweep 🔄 IN PROGRESS

**Infrastructure fixes applied (Mar 2026):**
- Obstacle now placed on planned trajectory (arc-length), not heading projection → guaranteed in-path
- Distances updated: [20, 30, 50, 100]m (dropped 150m)
- Metrics now read from `objects_filtered` → injected obstacle visible in safety metrics
- `min_closing_time` metric added (= distance/ego_speed, valid for static obstacles)
- Fault overlay support: `--fault-strategy perception_dropout --fault-params '{"dropout_rate":0.3}'`

**3-experiment pattern per distance:**
1. Nominal: `./run_sweep.sh static_obstacle.yaml ...` (no fault)
2. Fault:   `./run_sweep.sh static_obstacle.yaml ... --fault-strategy perception_dropout --fault-params '{"dropout_rate":0.3}'`
3. RISE:    same as fault + RISE controller (Phase 4)

| Status | Config | Notes |
|--------|--------|-------|
| ✅ | 50m × CAR × 3 goals × 2 trials | Done (old heading-based placement) |
| ⏳ | **Full sweep 20/30/50/100m × CAR/TRUCK** | Re-run with trajectory placement |

Command: `./run_sweep.sh static_obstacle.yaml --goals "goal_007,goal_011,goal_021" --trials 2 --stuck-timeout 180`

### 2.2 Cut-in Sweep ⏳

| Status | Config |
|--------|--------|
| ⏳ | 40m × 2s cut-in duration × 3 goals × 2 trials |
| ⏳ | 80m × 4s cut-in duration × 3 goals × 2 trials |

Command: `./run_sweep.sh cut_in.yaml --goals "goal_007,goal_011,goal_021" --trials 2 --stuck-timeout 120`

### 2.3 Additional Scenarios ⏳ (NEW — TO IMPLEMENT)

These three add meaningful scenario variety and are straightforward to implement
via the existing PerceptionInterceptor + ObjectFactory infrastructure:

**a) Slow Lead Vehicle**
A car ahead of ego moving at reduced speed (2–4 m/s). Ego is doing ~5 m/s,
so there is a closing distance. More realistic than a static obstacle — simulates
slow urban traffic or a decelerating car. Expected: ego slows, possible MRM.
- Params: `distance`, `lead_speed`, `obj_type`
- Implementation: `ObjectFactory.create_object(vx=lead_speed)` injected every frame at fixed world position

**b) Sudden Close Appearance**
An object that appears at close range (10–15m) after the vehicle has been
moving for N seconds. Simulates a pedestrian or cyclist emerging from behind
a parked car. Forces a hard reaction from the planner.
- Params: `trigger_time`, `distance`, `obj_type`
- Implementation: passthrough until trigger_time, then inject at locked world position

**c) Pedestrian Crossing**
Small pedestrian object with lateral (perpendicular) velocity crossing the road
at a specified distance ahead. Tests the planner's response to a different object
class and motion type.
- Params: `distance`, `crossing_speed`, `trigger_time`
- Implementation: inject PEDESTRIAN with vy set to crossing_speed, position anchored

### 2.4 Scenario Placement Investigation ⏳

A key open question: are the injected objects landing on road segments where lane
changes are physically possible? The static obstacle analysis shows Autoware always
finds an escape route. Need to:
- Map the first 100–200m of each goal route to understand lane structure
- Identify any single-lane segments (no lane change possible)
- If goals 007/011/021 don't have such segments, identify alternative goals

---

## Phase 3: Fault + Scenario Combinations ⏳

Once Phase 2 establishes "clean" risk scenario baselines, combine with perception faults.
The hypothesis: perception faults degrade Autoware's ability to handle the risk scenarios,
producing more dangerous outcomes than either fault or scenario alone.

### Combination Matrix (priority order)

| Scenario | Fault | Expected Effect |
|----------|-------|-----------------|
| Static obstacle 30m | Dropout 30% | Obstacle may not be detected in time → no lane change |
| Static obstacle 30m | Position noise 0.5m | Avoidance trajectory displaced → closer pass |
| Static obstacle 30m | Delay 200ms | Detection arrives late → harder braking, possible collision proxy |
| Cut-in | Dropout 30% | Cut-in car intermittently invisible → tracking instability |
| Cut-in | Delay 200ms | Reaction delayed → closer approach |

### What "Virtual Collision" Looks Like

In simulation, the injected object has no physical presence — the vehicle cannot actually
hit it. A "virtual collision" in our framework is defined as:
- `min_object_distance < 1.5m` at any point
- Or `collision_proxy_count > 0` (clearance below 2m threshold)
- Or MRM_SUCCEEDED (emergency stop triggered by the scenario)

---

## Phase 4: RISE Implementation ⏳

**Prerequisite:** Phase 2 and 3 data available to characterize what residuals look like
under risk scenarios.

| Status | Task |
|--------|------|
| ⏳ | Collect training data covering nominal + risk scenario conditions |
| ⏳ | Adapt ST-GAT: add scenario-relevant features if needed |
| ⏳ | Train model, verify residuals spike during obstacle encounters |
| ⏳ | Implement CVaR computation from residual stream |
| ⏳ | Design CVaR → velocity limit mapping |
| ⏳ | Integrate with Autoware velocity_smoother |
| ⏳ | Validate: Phase 3 fault+scenario with RISE vs without RISE |

---

## Key Configuration

| Item | Value | File |
|------|-------|------|
| Sweep goals | goal_007 (915m), goal_011 (780m), goal_021 (849m) | `experiments/configs/goals.json` |
| Excluded goal | goal_024 (bad map area, MRM_SUCCEEDED always) | — |
| MRM fix | perception + planning removed from autonomous mode gate | `autoware/.../autoware-main.yaml` |
| Planning input | `objects_filtered` (interceptor must be running) | `tier4_planning_component.launch.xml:11` |

---

## Open Questions

1. **Are injected objects landing on single-lane road segments?** Current evidence says
   Autoware always lane-changes. Need to verify route geometry for goals 007/011/021.

2. **Is 20–30m close enough to prevent lane change?** At 20m, Autoware may still have
   time. May need to also test with lateral_offset partially in adjacent lane to block both.

3. **TTC metric is broken for static scenarios.** Static objects have zero velocity →
   TTC = ∞ always. Need a dedicated closing-distance metric (time to reach obstacle
   at current speed) for static obstacle scenarios.
