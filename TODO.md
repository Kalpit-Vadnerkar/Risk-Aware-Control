# Risk-Aware Control — Task List

**Last Updated:** 2026-05-03

---

## Research Direction (as of May 2026)

**Core contribution:** ST-GAT residuals continuously characterize the uncertainty
landscape of an AV's situation. When Autoware stops at a static obstacle (one point
in safety×progress space), the residuals can tell us *how recoverable* the situation
is — and what velocity constraint lets the vehicle safely proceed. This moves the
vehicle toward the Pareto frontier of (safety, progress) rather than defaulting to
the trivially safe but zero-progress corner.

**Claim to establish:**
1. Residuals from a nominal ST-GAT model are low during nominal driving and elevated
   during obstacle encounters
2. The residual pattern differs between solvable scenarios (adjacent lane exists) and
   unsolvable ones (single-lane, no escape)
3. Conformal prediction over nominal residuals at each velocity level gives a
   distribution-free coverage guarantee on prediction error
4. Constraint adjustment (velocity cap) enables forward progress in solvable cases
   without violating the conformal bound

**What this is NOT:** Re-implementing Autoware's planner. Autoware already has
binary stop/avoidance behavior. Our signal is continuous, calibrated, and
distribution-free — not a fixed policy.

---

## Phase 0: Data Collection ← CURRENT

### 0.1 Nominal Campaigns

Nominal data is the calibration foundation for conformal prediction.
Each velocity level needs its own calibration set (residuals depend on speed).

| Campaign | Status | Command |
|----------|--------|---------|
| `nom_v5`  | ⏳ Needed | `./collect.sh nom_v5` |
| `nom_v7`  | ✅ Done (20 runs) | `./collect.sh nom_v7` |
| `nom_v11` | 🔄 In progress | `./collect.sh nom_v11` |

### 0.2 Obstacle Campaigns

Three obstacle conditions bracket the (solvable, unsolvable, recovery) space:

| Campaign | Status | What it shows |
|----------|--------|---------------|
| `obs_stuck`       | ⏳ Need | Autoware stops — baseline conservative behavior |
| `obs_recovery`    | ⏳ Need | Autoware swerves (policy=auto) — actual recovery |
| `obs_noescape`    | ⏳ Need | Single-lane (LL 241), no path — stopping IS optimal |
| `obs_singlelane`  | ⏳ Need | 30m obstacle, no adjacent lane — Signal 1 validation |
| `obs_tooclosetoreact` | ⏳ Need | 5–8m obstacle, multi-lane — Signal 2 (TTC) validation |

Commands:
```bash
./collect.sh obs_stuck
./collect.sh obs_recovery        # sets policy to "auto", restores on exit
./collect.sh obs_noescape        # verify obstacle lands in LL 241 (single-lane)
./collect.sh obs_singlelane
./collect.sh obs_tooclosetoreact
```

**Before running obs_noescape / obs_singlelane:** Verify obstacle placement lands in
LL 241 by checking rosbag `/tf` ego position when obstacle appears. `min_travel_before_placement: 60.0` + 30m ahead ≈ 90–100m arc — confirm empirically on first run.

---

## Phase 1: ST-GAT Training

Port and train the model from the T-ITS paper. Reference implementation is
READ-ONLY at `../Graph-Scene-Representation-and-Prediction/` — code we need from it
(`Point`, `GraphBuilder`, `MapProcessor`) is vendored into `st_gat/pipeline/vendor/`,
not imported live.

Work goes in `st_gat/` within this repo.

### 1.1 Data Extraction

- [ ] Write `st_gat/extract.py`: reads rosbag files from `experiments/data/baseline_all/`
      and all `nom_v*/` campaigns, extracts per-timestep features:
      position(2), velocity(2), steering(1), accel(1), obj_distance(1), traffic_light(1)
- [ ] Output: `st_gat/data/nominal_features.pkl` — one row per timestep, labeled by
      run_id and velocity_cap
- [ ] Verify feature distributions look sane (no NaNs, velocity capped at expected values)

### 1.2 Model Training

- [ ] Write `st_gat/config.py`: hyperparams matching T-ITS paper
- [ ] Write `st_gat/train.py`: train ST-GAT on nominal data (baseline_all + nom_v5/v7/v10),
      80/20 train/val split
- [ ] Train and save model checkpoint to `st_gat/checkpoints/`
- [ ] Verify residuals are low on held-out nominal runs (raw residual < 1σ in majority
      of timesteps)

**Success criterion:** On held-out nominal data, mean raw residual < 0.5 m/s for
velocity features, < 1.0m for position features.

### 1.3 Residual Computation

- [ ] Write `st_gat/residuals.py`: run trained model over all experiment rosbags,
      compute per-timestep Raw, KL, and CUSUM residuals
- [ ] Output: one residual CSV per run in `st_gat/residuals/`
- [ ] Plot residual traces for:
      - A clean nominal run (should be flat near zero)
      - An obs_stuck run (should spike when obstacle appears)
      - An obs_recovery run (should spike, then return to nominal)
      - An obs_noescape run (should spike and stay elevated — no escape)

---

## Phase 2: Signal Validation

**Goal:** Confirm the residual signal empirically differentiates solvable from
unsolvable scenarios before building any control logic.

### 2.1 Solvable vs Unsolvable Separation

- [ ] Compare residual traces: obs_recovery vs obs_noescape
- [ ] Key question: does the residual pattern (shape, peak, or duration) differ
      between the two? If yes, it's a viable signal. If no, the hypothesis fails.
- [ ] Compute a simple scalar: AUC of residual during obstacle window (t_appear to
      t_escape or timeout) — does it separate the two populations?

**This is the empirical test that validates or invalidates the core hypothesis.**
If residuals don't separate solvable from unsolvable, redesign before proceeding.

### 2.2 Velocity-Dependent Residual Baseline

- [ ] For each velocity level (5, 7, 10 m/s), compute the 95th-percentile residual
      during nominal driving → this is A₀(v), the "what's normal for this speed"
- [ ] Check: does A₀ vary significantly across velocity levels? (Expected: yes, higher
      speed = richer dynamics = higher variance)
- [ ] These per-speed baselines will anchor conformal calibration in Phase 3

---

## Phase 3: Conformal Prediction Calibration

**Goal:** Get a distribution-free guarantee: at velocity cap v, P[prediction error ≤
r(A)] ≥ 1-δ, where r(A) is the conformal bound at anomaly score A.

### 3.1 Per-Speed Calibration

For each velocity level (v5, v7, v10):
- [ ] Split nominal runs into D_train (fit residual→error mapping) / D_conf (calibrate)
- [ ] Fit isotonic regression: anomaly score Aₜ → expected position error at t+H
- [ ] Compute conformity scores on D_conf; find q̂_{1-δ} quantile
- [ ] Verify empirical coverage ≥ 0.94 at δ=0.05

**Success criterion:** Coverage check passes for all three velocity levels.

### 3.2 Coverage Visualization

- [ ] Plot: residual score vs. actual prediction error, with conformal bound overlaid
- [ ] Show coverage fraction across the calibration set
- [ ] This becomes Figure X in the thesis

---

## Phase 4: Constraint Mapping and Validation

**Goal:** Show the framework can select a velocity constraint that enables forward
progress while satisfying the conformal bound.

### 4.1 Constraint Selection Logic

- [ ] Given residual A at time t and obstacle distance d:
      v_max(t) = max(v_min, (d - d_min - r(A)) / H)
- [ ] Implement as `st_gat/constraint.py`
- [ ] Test on obs_recovery rosbag data (offline): does the computed v_max allow the
      vehicle to reach the obstacle swerve zone?

### 4.2 ROS2 Constraint Publisher (if offline validation passes)

- [ ] Write a ROS2 node that subscribes to residual stream, publishes v_max to
      `/planning/scenario_planning/max_velocity`
- [ ] Run live on obs_stuck scenario: does constraint relaxation let Autoware proceed?
- [ ] Compare: obs_stuck baseline (stops) vs. framework-active (swerves or proceeds)

---

## Key Design Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Velocity levels | 5, 7, 11.11 m/s | Nominal campaign caps (low / medium / map-limit) |
| Obstacle distance | 30m | obs_* scenarios |
| Conformal δ | 0.05 | Standard 95% coverage |
| Planning horizon H | 3s | Kinematic stopping distance |
| Min clearance d_min | 2m | Physical buffer |
| ST-GAT features | pos(2), vel(2), steer(1), accel(1), obj_dist(1), tl(1) | T-ITS 2025 |
| Goals | 007, 011, 021 | Verified live on P5000 (2026-06-27), all routes feasible |

---

## Explicitly Out of Scope

- Perception faults (dropout30, position noise) — not needed for core Pareto claim
- Obstacle removal / disappearing obstacle — artificial, doesn't reflect real recovery
- Multi-fault type comparison matrix
- Scenarios other than static obstacle (cut-in, pedestrian, slow lead vehicle)
- Distance sweep (20m, 50m, 100m) — 30m is the one scenario
- Weight modulation of ST-GAT (constraint tightening is the intervention)
- RISE name and framing

These are valid future work for the thesis "Limitations and Future Work" section.
