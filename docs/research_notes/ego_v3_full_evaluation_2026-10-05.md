# ego_v3 full evaluation: Layer 1 calibration, fault visibility, Layer 2 on all 38 fault trials (2026-10-05)

Model: `st_gat/models/h30_30_ego3/st_gat_rise.pth` (ego-frame map, past-only
graph, lane direction, route fix, route-ahead input, normalized selection).
Run with `STGAT_GRAPH_FRAME=ego_v3`; all outputs under
`experiments/analysis/h30_30_ego3/` (new `cfg.ANALYSIS_ROOT`; canonical
bbox_v1 results untouched). Fair canonical baseline recalibrated on the SAME
17 CAL trials: `experiments/analysis/h30_30_bbox_v1_cal17/`.

## Layer 1 calibration (17 CAL trials, LOO cross-conformal)
Pooled coverage 0.896-0.900 for every feature/horizon (both models).
Width at 3s, canonical -> ego_v3: position 5.25 -> 2.79m (-47%), steering
-23%, lateral velocity -10%, longitudinal velocity +27%, acceleration +14%,
TL confidence +13%, TL color +5%. Caveat favouring the canonical model: it
trained (Aug) on goal_007/012 trials that are now in CAL; ego_v3 never saw
any CAL trial.

Minority-scenario audit: steering under-covers in turns (canonical 0.83,
ego_v3 0.80) and TL zones (0.80 / 0.82), over-covers on open road -- the
cancellation pattern. **Mondrian closes it for ego_v3**: every group x
feature 0.885-0.914. Embedding k-NN (k=150) adapts in the right direction
(steering in turns 0.68 -> 0.92 coverage with 2.5x wider intervals, calm
road narrower) but under-covers overall (0.857 per-step) -- revisit k.

## Layer 1 vs faults (same 8 campaigns as the Aug canonical run; clean =
windows before the first fault -- the script's default "clean" counts the
post-fault stuck aftermath of IMU trials as clean, which is what produced
the Aug 13.66x acceleration headline)

| active/clean exceed ratio | IMU canonical -> ego_v3 | TL canonical -> ego_v3 |
|---|---|---|
| position | 7.1 -> 27.9 | 4.4 -> 75 |
| velocity long. | 16.4 -> 23.5 | 8.9 -> 45 |
| velocity lat. | 7.2 -> 29.1 | 4.6 -> 18.7 |
| steering | 3.5 -> 6.1 | 1.5 -> 1.8 |
| acceleration | 9.6 -> 7.2 | 3.8 -> 5.6 |
| TL confidence | 4.1 -> 13.3 | 2.9 -> 6.8 |

Mechanism: a map-consistent forecast has a far quieter nominal baseline
(clean position exceedance 7.5% -> 2.1% IMU trials, 4.0% -> 0.3% TL trials).

## Layer 2 (all 38 fault trials, every campaign; thresholds at 1% nominal FAR)
Nominal floor (6,915 windows): embedding pool mean P 0.037-0.040,
threshold 0.82-0.83 (was ~0.95-0.98); pooled/Mondrian thresholds ~0.96.

Ground-truth departures: 4 trials, all IMU (s3/goal_007 88.5s,
scale/goal_007 54.5s, stuck/goal_007 60.0s, stuck/goal_012 103.3s). TL
faults never caused a road departure in any of 27 TL trials.

| departure trial | Layer-1-only | embedding static | embedding adaptive |
|---|---|---|---|
| imu_fault_s3/goal_007 | 3.0s lead | "25.0s" * | "25.0s" * |
| imu_fault_scale/goal_007 | miss | miss | miss |
| imu_fault_stuck/goal_007 | 2.0s lead | miss | miss (peak P 0.73 < 0.82) |
| imu_fault_stuck/goal_012 | 0.2s lead | miss | miss |

\* not a real detection: a spike at ~64s while s(t)=1 (static tube at a
tight spot), counted because the lead-time metric accepts ANY alarm in
[fault start, departure]. The metric needs tightening.

Specificity is good: in TL trials (no departures) in-fault alarm rates are
0.00-0.10 for every variant; pre-fault 0.00-0.10.

## Why Layer 2 misses: the Layer1->Layer2 coupling is the wrong shape
The adaptive coupling multiplies whole motion-residual trajectories by
s(t) (up to 10x). Two failure modes, both visible in the data:
1. Scaled residuals imply impossible accelerations, so the physics trim
   removes 54-98% of counterfactuals exactly when s(t) is high; survivors
   are the near-nominal ones.
2. For a stopped/slow ego the nearest-neighbour residuals are ~0, and
   10 x 0 = 0 (stuck: P(untrimmed) only 0.14-0.53 at s > 5).
An IMU fault corrupts WHERE the vehicle is (localization), not how it will
move. The counterfactual should be a rigid pose perturbation (position
offset + heading error, magnitude from Layer 1's anomaly), which the
physics trim correctly leaves alone -- not a multiplied motion residual.

## What's needed next (proposal)
1. Redesign the coupling as a localization/pose-uncertainty perturbation.
2. Tighten the lead-time metric (alarm must persist / be in the final
   alarm run before departure; report false alarms separately).
3. Evaluation power: 4 departures, all IMU, all goal_007/012. TL faults'
   consequence is intersection conflict, not road departure -- needs the
   object safe set with real cross-traffic (NPC density > 10) and/or Arm B.
4. The open-loop p99 tail at tight spots (closed-loop lane-keeping).
