# Risk-Aware Control — Task List

**Last updated:** 2026-10-07. Design: `docs/architecture.md`. Measured state:
`docs/findings.md`. Collection plan: `docs/experiment_plan.md`. This file is
only the actionable list; completed items move to `findings.md`.

## 1. Build the architecture (CURRENT)

- [x] ego_v3 predictor (ego-frame map, route fix, route-ahead input,
      normalized selection) — accepted by Kalpit, not yet promoted.
- [x] Model safety benchmark / ablation table.
- [x] Layer 2 machinery: safe sets (road, objects), 2a pools, matched-FAR
      evaluation against AWSIM ground truth; hand-built dead-reckoning
      baseline (4/4 departures).
- [x] **ego_v4 multi-source predictor**: redundant inputs + consistency head,
      `--selection kinematic`; first model to pass the benchmark gate.
- [ ] Layer 1 scores for the consistency heads (conformal, per step) and a
      combined onset statistic over all channels.
- [ ] Layer 2b: integrate calibrated consistency gaps from onset into the
      pose-error distribution (replaces the dead-reckoning coupling as the
      method; keep dead-reckoning as a baseline).
- [ ] Layer 2a: lane-keeping prior under trust (fixes the nominal open-loop
      tail that sets every threshold).
- [ ] Baselines from Autoware's own monitors (gyro bias/scale validator, pose
      instability detector, EKF gate).
- [ ] Rerun all 38 fault trials; update `findings.md` §4.
- [ ] Promote the chosen model via `promote_model.py` (wire the benchmark
      gate into it), then rerun the Layer 1 results for Paper 1.

## 2. Violation types beyond road departure

- [ ] Red-light running: stop-line safe set + labeller on the unfaulted
      signal topic; validate on existing TL + nominal data.
- [ ] Speed-limit safe set.
- [ ] Object conflicts with real ground truth (needs cross traffic).

## 3. Experiments (Kalpit runs collection) — see `docs/experiment_plan.md`

- [ ] IMU severity x geometry campaign.
- [ ] `tl_false_green` injector mode (decision pending).
- [ ] AWSIM rebuild for traffic density; Arm B batch.
- [ ] More nominal trials per goal.
- [ ] Open from the TL severity sweep: goal_026 vs the ramp pilot.
- [ ] Root-cause the goal_026 failures (try it first in a batch).

## 4. Writing and scope

- [ ] Broader literature review (reachability, conformal-for-planning,
      AV safety verification) before claiming the calibration x
      consequence synthesis is novel.
- [ ] Layer 3 scope: signal definition only, or closed-loop response.
- [ ] Revisit embedding k (k = 150 under-covers at 0.857).

## 5. Ablation backlog (lower priority)

Graph re-pooling cadence; capacity sweep (128/128/128/2/4 never revisited);
object-set pooling; sparse-adjacency GCN; `MAX_GRAPH_NODES` as a variable.
Each gets a row in the benchmark table.

## Out of scope

- New obstacle/staged-avoidance experiments (`obs_*`): control contribution,
  not safety evidence.
- New LiDAR fault collection here (the T-ITS paper's data is the reference).
- Multiple speed levels (all runs at 11.11 m/s).
- Reviving the closed-set / belief-divergence framing (`findings.md` §6).

## Key parameters

| Parameter | Value | Notes |
|---|---|---|
| Speed | 11.11 m/s (map limit) | single condition |
| NPC density | 10 | single condition; override plumbed, needs AWSIM rebuild |
| Conformal target | 90% | reliability diagram is the headline |
| Fault goals | goal_007, goal_012, goal_026 | most TL-zone entries per trial |
| Canonical model | `st_gat/models/h30_30/st_gat_rise.pth` (bbox_v1) | best: `h30_30_ego3` |
| Calibration trials | 17 (`CAL_DIR`) | more needed beyond goals 007/012/026 |
