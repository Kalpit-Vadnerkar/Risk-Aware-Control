# Proposal: alarm logic, evaluation metrics, fault campaign, paper figures

Independent review, 2026-10-07. Based on `architecture.md`, `findings.md`,
`experiment_plan.md`, `experiments/scripts/layer2_envelope.py` and the ego_v4
run in `experiments/analysis/h30_30_ego4/layer2_envelope/` (38 fault trials,
6,915 nominal windows ≈ 58 min of CAL driving). Assumes the ego_v5
pose-based consistency channel. This is a proposal. Nothing in the code has changed.

---

## 1. Diagnosis of the current alarm logic

### 1.1 What is principled
- **Conformal residuals, calibrated leave-one-trial-out.** This is sound and is the backbone of the method.
- **CUSUM for onset, with the change-point taken as the last time S = 0.**
  This is the textbook Page/Lorden detector, and its change-point estimate is the standard
  one. Starting the 2b integration at that point (rather than at the alarm time)
  is correct.
- **Thresholds at a matched nominal false-alarm budget** for every variant.
  This is the right way to compare methods.
- **Lead time = onset of the episode still active at the event.** This fixed the
  earlier "25 s lead from a stale spike" error.

### 1.2 What is ad hoc or broken (with evidence)

| # | Problem | Evidence |
|---|---|---|
| D1 | **CUSUM saturates and never ends the alarm.** The increments z_t are unbounded: z is \|r\|/q90, and faults reach about 30x. The drain is only kappa per step, so S can need minutes to return to 0. "End when S = 0" means distrust lasts as long as the fault lasted × (z/kappa). | `imu_fault_s3/goal_007`: the fault is active for 2 × 15 s (≈50–65 s, 78–93 s), but distrust runs from 50 s to the end of the trial (193 s). `imu_fault_s1/goal_007`: 60 s of fault, distrust from 54 s to the end (170 s), which bridges both 10–35 s off-cycles. Layer-1 in-fault alarm rate = 1.00 on every s1/s3 trial. |
| D2 | **Single-window spikes trigger distrust.** Because z is unbounded, one window with z > kappa + h crosses h in one step. In that regime the CUSUM is just a raw threshold. | Pre-fault spikes of S/h > 4 in one window: s1/007 at 29 s and 34 s, scale/007 at 25 s, stuck/012 at 4–6 s and 63 s. |
| D3 | **The max over heterogeneous channels, scaled by q90, has no known null distribution.** q90 does not equalise tails. kappa is set to the 0.99 quantile because kappa = q90 had no downward drift (median S 5, p90 119 on nominal). | Comment in `build_pool`. kappa and h are tuning knobs with no stated guarantee. |
| D4 | **The calibration targets a window share, not an operational rate.** "1% of nominal windows distrusted" or "P > θ on 1% of windows" says nothing about how many alarm *episodes* per hour a driver sees. The CUSUM h and kappa are also fitted in-sample: `build_pool` uses the same CAL windows with no leave-one-trial-out. | `nominal_floor.json`: `layer1_flag_rate` 0.0097. That figure is in-sample by construction. |
| D5 | **The end of rate evidence is confused with the end of pose error.** When S returns to 0, `evidence_state` resets Psi, X and Y to 0. A corrupted EKF heading does not heal when the gyro recovers, and it does not stay wrong once NDT corrects it either. Both cases need a pose-level test. | scale/007: true heading error stays at −28° after the fault (the vehicle has left the road). s3/007: true error returns to about 0 at 65 s (scan-matching correction) while Psi keeps growing. |
| D6 | **The 2b spread grows with time in distrust, not with evidence.** sY = v·σ·τ²/2 assumes the rate errors are perfectly correlated and Gaussian (q90/1.645). Combined with D1, P(violation) goes to 1 after a long enough distrust even when e = 0. | s1/007: P_evidence = 1.0 through the 104–140 s off-cycle, where e ≈ 0 and the true heading error is about 0°. |
| D7 | **Layer 1 is blind to traffic-light faults by construction.** `cusum_z` uses only the consistency and lateral/heading channels. TL faults act on longitudinal behaviour and the TL heads. | Layer-1 in-fault rate on all 30 TL trials: 0.00–0.07, the same as the pre-fault rate (0.00–0.02). The "anomaly ≠ consequence" claim cannot be shown if the anomaly is never flagged. |
| D8 | **The Layer-2 threshold is set by the nominal open-loop tail, not by 2b.** The evidence threshold equals the nominal threshold (0.824 for both): 2b is almost never active on nominal, so it uses none of the false-alarm budget. Every alarm therefore means "P ≈ 1". Graded levels between 0.2 and 0.8 cannot be calibrated while nominal P has its mass at 1 near tight spots. | `nominal_floor.json`: p95 0.21, p99 0.82, 7.1% of windows have P > 0.1. stuck/012 has pre-fault P of 0.6–0.75 at a 0.5 m-clearance segment (55–65 s). |
| D9 | **The table mixes three incomparable things.** Lead times, in-fault/pre-fault window rates, and no count of false-alarm episodes. `summarize` counts off-cycle and post-fault windows in neither rate, and counts post-departure windows as "in-fault". A lead is reported even when the episode began at the fault onset. | s3/007, Layer 1: "38.6 s lead" is just departure − fault start (88.5 − 49.9). The evidence variant's "32.2 s" is the same saturated episode. Neither is a consequence prediction. |
| D10 | **Monte Carlo noise at the threshold.** With K = 200, the standard deviation of P at 0.82 is about 0.027. Alarms flicker across θ, which the 1 s merge gap hides. | P traces in every timeseries PNG. |
| D11 | **The EKF-twist channel over-integrates** (the reason for ego_v5). | s1/007: Psi > 60° against a true value ≤ 2°. The EKF yaw-bias state absorbs the bias, so the twist gap is not the pose error. As a result, evidence alarms on 75–89% of fault time in three non-consequential IMU trials (s1/007 0.84, s1/012 0.75, s3/012 0.89). |

Net reading of the current numbers, with no reinterpretation:
- **Real departures:** evidence 4/4, dead-reckoning 4/4, Layer-1-only 4/4, nominal 2/4.
- **Non-saturated leads:** evidence 1.0, 3.4 and 4.4 s. Dead-reckoning 2.2, 2.8 and 4.0 s.
- **Selectivity:** evidence is poor on non-consequential IMU faults, and Layer 1 does not detect TL faults at all.

So the data supports "the evidence path catches what dead-reckoning catches". It does not yet support "anomaly ≠ consequence".

---

## 2. Alarm strategy

The core recommendation is to separate four things that the current code
couples through one variable S:

1. **Onset:** is the evidence stream off-nominal now?
2. **Recovery:** has it been nominal again for long enough?
3. **Carried pose error:** how far off the state still is, which survives the end of evidence.
4. **Consequence alarm:** a graded level based on P and its imminence.

### 2.1 Layer 1: common input (replaces `cusum_z`)

Per channel c, form a **conformal p-value** against the nominal LOO scores,
Mondrian by zone (TL zone / turn / open road, as in `conformal_mondrian_calibration.py`):

    p_{c,t} = (1 + #{nominal scores of channel c in zone g(t) >= s_{c,t}}) / (n_g + 1)

Channels: the 2 EKF consistency channels, the 2 pose consistency channels (ego_v5), and the
behavioural lateral, heading, along-track and speed channels. Add **TL-head
residuals** to fix D7.

Combine the channels with Bonferroni: p_t = min(1, C · min_c p_{c,t}). Then
define the bounded score u_t = −log p_t ∈ [0, log(n+1)].

- **Bounded increments** fix D2: a single window adds at most log(n+1) ≈ 10, and that size is reached only at the extreme rank.
- **Every channel is on the same, distribution-free scale**, which fixes D3.
- **The Bonferroni combination is valid under any dependence between channels.**

### 2.2 Layer 1: onset/distrust detector options compared

| Option | Statistic | Threshold calibration | Guarantee | How the alarm ends | Verdict |
|---|---|---|---|---|---|
| A. Current CUSUM | S_t = max(0, S + z − κ) | h = window quantile, in-sample | none stated | S → 0 (unbounded delay) | replace |
| B. **Capped CUSUM on u_t** | S_t = min(S_cap, max(0, S + u_t − κ)), κ = nominal median of u + margin | h set for a target FA **episodes/hour** on LOO nominal | empirical ARL₀; with S_cap the recovery delay is ≤ S_cap/κ steps | S < h_off, or the recovery test (below) | **recommended for onset** |
| C. Windowed scan | N_t = Σ_{s∈(t−W,t]} 1[p_s < α], W = 2 s | n* = LOO nominal quantile for the FA target | delay ≤ W after evidence stops | N_t < n_off | **recommended for recovery** |
| D. Exponential forgetting | E_t = λE + (1−λ)u_t | same as B | none; equivalent to EWMA charts | E < h_off | worse ARL than B for step changes; no gain over B+C |
| E. Conformal e-CUSUM (Shin–Ramdas–Rinaldo) | log M_t = max(0, log M) + log e_t, e_t = κ p_t^{κ−1} | 1/α (Ville) | ARL₀ ≥ 1/α **if the p-values are independent and uniform**. That fails here: 30 overlapping forecasts give strong autocorrelation | same drain problem as A unless capped | use for the paper's *theory* paragraph. Thin to non-overlapping 0.5 s blocks and still calibrate empirically. Do not claim Ville's bound on 10 Hz data. |
| F. Bayesian online changepoint / GLR with recovery | run-length posterior | priors (hazard) | model-based | posterior on a second change | too heavy; needs a likelihood the project chose not to have |

**Recommendation:** use B for onset and C for the end.

- **Distrusted** is set when S_t > h (option B).
- **Distrusted** is cleared when N_t ≤ n_off (the last 2 s look nominal) *and* S_t < h. On release, set S ← 0, which is an explicit reset on recovery.
- **S_cap = 2h**, so re-detection after a short gap stays fast but the drain cannot run for minutes.
- **The change-point** is still the last S = 0.

Calibration, all on held-out nominal trials (LOO by trial, the same stride as the fault runs):
- κ is fixed (for example the nominal 0.9 quantile of u).
- h is the smallest value giving ≤ λ₀ false episodes per hour, with λ₀ = 1/h as a starting value (§3).
- n_off is the nominal 95th percentile of N_t, so 95% of nominal 2 s windows would pass the recovery test.

Code changes:
- `cusum_z` → `channel_pvalues` and `combined_score`.
- `cusum` gains `cap`.
- New `scan_count`.
- `evidence_state` gains a recovery branch.
- `build_pool` computes κ, h and n_off with a LOO loop over trials and reports episodes/hour.

### 2.3 Carried pose error (2b), decoupled from distrust (fixes D5, D6)

- **Integrate the ego_v5 pose-based channel from the change-point.** Do not reset it when distrust ends. A scan-matching correction appears in that channel as an opposite-sign residual, so Psi comes back to about 0 by itself (s3/007 at 65 s is the test case).
- **Release the carried error** only when |Psi| < b_Ψ(τ), |Y| < b_Y(τ) *and* the detector is in the recovered state.
- **b(τ) is the nominal quantile of the integrator itself.** Run the same integrator on nominal trials from random start times. For each τ, take the 95% quantile of |Psi(τ)| and |Y(τ)|.

This replaces the Gaussian `se_y·τ` and `v·se_y·τ²/2` with a **distribution-free band for integrated nominal drift**. It also fixes D6: on nominal data the integrated drift is small, so the band stays narrow unless e is actually non-zero.

Reuse the same quantiles as the spread used to sample counterfactuals in `evidence_bundle`, and check the band's coverage of the true error over all distrusted windows (metric M7).

### 2.4 Layer 2: consequence alarm

**Report two numbers, not one.**
- **P_t:** absolute P(violation within 3 s), the input for Layer 3.
- **ΔP_t = P_evidence − P_nominal:** the risk attributable to the evidence. ΔP is about 0 on nominal by construction, so the open-loop tail (D8) cancels.

**Graded levels by imminence, not by P bands.** The counterfactuals already give a time of first violation. Change `L2.score` to return `first_violation_step` (K,), then form P_h = share violating within h, for h ∈ {1, 2, 3} s.

| Level | Condition | Meaning for Layer 3 |
|---|---|---|
| 0 Nominal | not distrusted, P₃ ≤ θ₃ | none |
| 1 Anomaly (unsized or benign) | distrusted, ΔP₃ ≤ δ₃ | log, raise monitoring |
| 2 Caution | ΔP₃ > δ₃, or P₃ > θ₃ | slow down, increase gaps |
| 3 Warning | ΔP₁.₅ > δ₁.₅ | stop or hand over |

- **Calibration:** each threshold is set on LOO nominal so that level ≥ k gives ≤ λ_k false *episodes* per hour, with λ₂ = 1/h and λ₃ = 0.2/h as starting points. Where θ₃ for absolute P is stuck near 1 (D8), keep θ₃ only until the lane-keeping prior lands. After that it becomes meaningful.
- **The ΔP thresholds are the method's alarms**, and they are reportable now.
- **Hysteresis and dwell:** turn on at ΔP ≥ δ_on for 2 consecutive windows. Turn off when ΔP < δ_off = δ_on/2 for 1 s. This replaces the 1 s merge gap and handles D10.
- **Growth component:** use it as an *escalation* rule, not a separate alarm. Escalate one level if the slope of the 1 s least-squares fit of ΔP₃ exceeds g*, the nominal 99th percentile of the slope. It catches a fast-ramping departure like scale/007 (1.0 s lead) earlier. Report it as an ablation.
- **Mitigation readout:** the slope is also the Layer-3 feedback signal. Slowing down should drive the slope to ≤ 0.

**Guarantee statement for the paper.** For any fixed scene distribution matching the nominal calibration trials, the expected number of level-≥k false episodes per hour is ≤ λ_k, up to the LOO estimate's error. Report it with a Poisson upper bound.

Make no claim about conditional validity in tight geometry. The Mondrian zones give it partially, and the audit should show it.

**Code changes:**
- `summarize` → `alarm_state(t, P, dP, P_h, D)`, returning the level per window.
- `mode_nominal` computes ΔP and P_h and calibrates per-level thresholds on episodes per hour.
- `mode_faults` stores per-window levels.
- `L2._alarm_episodes` gains `min_on` and `off_hold`.
- Also raise K to 500, or use a fixed common random-number seed per window across variants, so ΔP is not dominated by Monte Carlo noise.

---

## 3. Evaluation metrics (each readable without context)

**Definitions.**
- **Event:** the first time the AWSIM-true footprint has a road margin < 0, a red-light stop-line crossing while the unfaulted light is red, or an object overlap. At most one event per trial; analysis is censored after it.
- **Near-miss:** no event, but the minimum true margin during the fault is < 0.2 m.
- **Fault onset t_f:** the first fault window in `fault_log.jsonl`.

| ID | Metric | Precise definition | Reported as |
|---|---|---|---|
| M1 | **Events caught** | An event is caught if an alarm episode (level ≥ 2) has onset in [t_f, t_e − 0.5 s] and is still active at t_e. An episode starting before t_f does not count. | k/N with a Clopper–Pearson 95% CI |
| M2 | **Warning time** | t_e − onset of that episode. Report it alongside **M2b detection delay** = onset − t_f, so a reader can tell "warned 3 s before" from "alarmed since the fault began". | median [IQR], per family |
| M3 | **False alarms per hour** | Alarm episodes (level ≥ 2) on held-out nominal driving plus pre-fault segments of fault trials, divided by hours driven. | rate with a Poisson 95% upper bound (0 FA in T h → 3/T) |
| M4 | **Nuisance time** | Share of the *exposure period* (from t_f until the fault ends plus 10 s, or until recovery) spent at level ≥ 2, in trials with no event and no near-miss. | median per family (IMU, TL fail-safe) |
| M5 | **Anomaly detected** | The trial reaches level ≥ 1 within the exposure period. This is the Layer-1 recall, needed so that "Layer 2 silent" means "benign", not "missed". | k/N per family |
| M6 | **Anomaly ≠ consequence separation** | Among trials where the anomaly is detected (M5), the AUROC of the peak ΔP₃ during exposure for event-or-near-miss vs benign. Also report the 2×2 table of level ≥ 2 vs consequence. | AUROC with a bootstrap CI over trials, plus the 2×2 table |
| M7 | **Sizing accuracy** | During distrust: the coverage of the 2b band, i.e. the share of windows with the true heading/lateral error inside [Psi ± b_Ψ], plus the median \|Psi − true\|. | coverage (target ≥ 0.9), error in degrees and metres |
| M8 | **Risk calibration** (once ≥ 30 events) | Window label y = 1[true margin < 0 within 3 s]. Report a reliability curve of P₃ against y. | ECE and Brier score, trial-cluster bootstrap |

Drop the "in-fault/pre-fault window rate" pairs from the headline. They survive only as a diagnostic.

---

## 4. Fault experiment run strategy

### 4.1 What the evidence needs (rough power)
- **Detection claim, "≥ 90% of events caught":** with zero misses you need N ≥ 29 events for the Clopper–Pearson lower bound to exceed 0.90, and 14 for 0.80. The current 4/4 gives a lower bound of 0.40. **Target: 30 or more events, spread over at least 2 families and 4 goals.**
- **Method comparison on paired warning time** (evidence vs dead-reckoning vs Autoware monitors): with a paired standard deviation of about 1 s, detecting a 0.5 s median difference at 80% power (Wilcoxon) needs about 35 events. Thirty events is enough for differences of ≥ 0.6 s.
- **False alarms:** to claim < 1 FA/h with zero observed, you need ≥ 3 h of nominal driving the model never trained on (rule of three). That is about 60–70 new 3-minute nominal trials. Add the pre-fault segments of fault trials, which give about 40 s each.
- **Separation (M6):** an AUROC CI of width ±0.1 needs about 25 benign and about 25 consequential detected anomalies.
- **Event yield:** IMU s3/scale/stuck in tight geometry gave 4 departures in 6 trials on goals 007/012. Assuming roughly 40–50% yield in tight geometry and roughly 10% on wide straights, about 60–70 high-severity tight-geometry trials give 30 events.

### 4.2 Matrix

| Factor | Levels | Notes |
|---|---|---|
| IMU mode × severity | bias 0.02 / 0.04 / 0.08 rad/s (sustained, `off_seconds: 0`); scale 1.4 / 1.8 / 2.5; stuck | Scale and stuck only bite in turns, so they are inherently tight-geometry. Bias carries the geometry contrast. |
| Geometry | wide straight vs approach to turn, island or narrow segment | the existing `turn_zones.json` / lead-in zones; add an open-road gate |
| TL mode | **false-green (new)** · false-green while the true light is already green (matched no-consequence control) · fail-safe modes (unknown, blackout, conf 0.3) as benign controls | gives events and benign anomalies with the same injector |
| Goals | ≥ 2 per geometry (goal_007 and goal_012, plus 2 new ones) | do not rely on goal_026 until its failures are understood |
| Reps | 3 per cell (IMU), 4 per cell (TL) | |
| Nominal controls | ≥ 4 per goal per session, same start/goal, interleaved; more until ≥ 3 h held out in total | the CAL split manifest must mark them as held out |

Approximate budget:

| Block | Trials |
|---|---|
| IMU: (3 bias × 2 geometries + 3 scale + 1 stuck) × 3 reps × 2 goals | 60 |
| TL: 5 modes × 4 reps × 2–3 goals | about 50 |
| Nominal | about 70 |
| **Total** | **about 180 trials, about 10–12 sessions** |

### 4.3 Design rules
- **One onset per trial.** Cycling faults (s1/s3 on/off, zone-repeating TL) muddle onset, recovery and "in-fault" accounting. Add a `one_shot: true` param to `_imu_bias_loop` and the TL arming state machine in `experiments/lib/fault_injector.py`: arm once, hold for `--fault-duration`, never re-arm. Keep the cycling campaigns as a separate *recovery* study, which is exactly the test of §2.2's end-of-alarm logic.
- **False-green fault.**
  - **Why existing faults gave no consequences:** `tl_oscillate` (tl_fault_s2) already forces GREEN for 2.5 s of every 5 s, and produced no consequences. Flapping makes Autoware hold or stop, and the fault was not conditioned on the true light being red.
  - **New mode `tl_false_green`:** substitute GREEN at confidence 1.0 for the governing group, *only while the unfaulted light is red*, sustained from zone entry until the stop line is passed.
  - **Severity knob:** the distance to the stop line at injection (60 / 30 / 15 m).
  - **Event:** stop-line crossing while the true light is red. This is a rule-violation consequence that needs no cross traffic.
  - **Validate first:** run the labeller on all existing data, which should find about 0.
- **What Layer 2 can say about TL faults.** No redundant source exists, so declare that TL gives "detected, unsized" (level 1) unless a second source is added. The candidate is the model's own TL-state prediction head. If the perceived green lies outside its conformal set, treat P(true red) as the redundant estimate, and compute P(red-light violation) = P(true red) × P(cross stop line within 3 s | counterfactuals). Whichever way it goes, say so in the paper.
- **Use of Arm B.** Arm B is a *baseline comparator*, not a consequence source, because the stock MRM may prevent the departure. Run Arm B on a matched subset (the 2 most consequential IMU cells and false-green, 3 reps each, about 20 trials) and compare our alarm onset with Autoware's MRM trigger time.
- **Offline baselines on Arm A, at no cost:** Autoware's gyro-bias monitor, pose-instability detector, EKF Mahalanobis gate and NDT score, all from `/diagnostics` in every bag. These should be in the main table.
- **Randomisation.**
  - Within a session, use a randomised block order over {fault cell, nominal}, and rotate goal order between sessions to settle the goal_026 question.
  - Log the seed and order in the campaign `_meta/`.
  - Keep the existing standing checklist (`compare_fault_vs_nominal.py`, `verify_*_zone_arming.py`).
- **Freeze before test.** The current 38 trials become the *development set*. Freeze the alarm hyperparameters (κ, λ_k, W, S_cap) and the model, then run the new campaign once as a *test set*. Report development and test separately. This is the single biggest credibility gain available.

---

## 5. Paper figures and tables

**T1 — Main results table.** One row per method, all at the same nominal budget (1 FA/h), columns grouped by question:

```
                         Consequences (N=30 events)      Nuisance                      Anomaly
Method                   caught      warning s         FA/h nominal  benign time       detected
                         k/N [CI]    median [IQR]      [95% UB]      in alarm IMU|TL   IMU|TL
------------------------------------------------------------------------------------------------
Autoware monitors        ...         ...               ...           ... | ...         ...|...
Layer 1 only (no map)    ...
2a nominal envelope      ...
2a+2b dead-reckoning     ...
2a+2b evidence (ours)    ...
```

The reader gets the answer from one row: how many caught, how early, at what nuisance cost. Put per-trial details in an appendix table, one row per trial, with every campaign kept.

**F1 — Event swimlane (the main figure).**
- **Layout:** one row per test fault trial, grouped by family and sorted by consequence (event, near-miss, benign).
- **x-axis:** seconds relative to fault onset (−10 to +60).
- **Per row:** the fault-active bar (orange), Layer-1 distrust (grey), the Layer-2 level as a colour strip (yellow = 2, red = 3), and the event marker (×).
- **Message:** Layer 1 lights up for every fault; Layer 2 lights up only on rows that end in ×.

```
imu_scale  g007 r1  ▓▓▓▓▓▓▓░░░░░░░▒▒▒██×
imu_bias04 g012 r2  ▓▓▓▓▓▓▓▓░░░░░░░░░░░░░          (benign)
tl_fgreen  g007 r1   ▓▓▓▓░░░░░▒▒██×
tl_unknown g012 r3   ▓▓▓▓░░░░░░░░░░░               (benign, fail-safe)
                    0s        10s       20s
```

**F2 — Operating characteristic.**
- **Axes:** x is false alarms per hour of nominal driving (log, 0.1–100); y is the share of events caught with ≥ 1 s warning (left panel) and the median warning time (right panel).
- **Curves:** one per method, made by sweeping its threshold.
- **Message:** the evidence method dominates at every budget, and the conclusion does not depend on a chosen threshold.

**F3 — Anomaly vs consequence scatter.**
- **One point per detected fault trial.**
- **Axes:** x is Layer-1 anomaly strength (peak combined score or distrust duration); y is the peak ΔP₃ during exposure.
- **Encoding:** colour by outcome (event, near-miss, benign); marker by family. Show the δ thresholds as a horizontal line.
- **Message:** large anomalies are not necessarily consequential, and ΔP separates the two (annotate with the AUROC from M6). TL fail-safe faults should sit at high x and ΔP ≈ 0. False-green and IMU tight-geometry trials should sit high on y.

**F4 — Mechanism, one trial in 3 panels** (a cleaned-up version of today's timeseries).
- **Panels:**
  - combined Layer-1 score with the distrust state and change-point
  - 2b Psi and Y with the band against AWSIM truth
  - the ΔP₃ and P₁.₅ traces with the level strip
- **Filmstrip:** add 3 snapshots underneath.
- **Message:** when → how much → consequence, with truth overlaid.
- **Pair it with F4b, a sizing scatter:** estimated against true heading and lateral error at all distrusted windows of the event trials, the y = x line, and band coverage printed (M7).

**F5 (optional) — Severity × geometry heatmap.** Cells give the event rate and alarm rate per IMU mode/severity and geometry. Message: consequences depend on context, not only on fault size, and the alarm follows the consequences.

---

## 6. Prioritized actions

1. **Fix the end of alarms (D1, D5, D6).**
   - Bounded conformal score (`channel_pvalues`), capped CUSUM, scan-count recovery test, and pose-error carry with the nominal integrator band.
   - Re-run `layer2_envelope.py` on the existing 38 trials. Expected: s1/s3 distrust ends within about 2 s after each off-cycle, and P drops between cycles.
2. **Add the longitudinal and TL channels to Layer 1 (D7).** Check M5 on the 30 TL trials. If TL faults are still not detected, the separation story needs another anomaly family.
3. **Report ΔP plus imminence levels, calibrated on episodes per hour with LOO (D4, D8).** Implement M1–M7 in a new `event_metrics()` and replace `print_table` with T1. Add the Autoware `/diagnostics` baselines offline.
4. **Measure the false-alarm rate honestly.** Episodes per hour on the 17 CAL trials plus all pre-fault segments, with a Poisson bound. If it is above 1/h, tune before collecting.
5. **Injector changes.** `one_shot` arming, `tl_false_green` conditioned on the true red, and a red-light stop-line labeller validated on existing data.
6. **Freeze the method and hyperparameters, then collect the test campaign** (§4.2, about 180 trials including about 70 nominal), randomised and interleaved, with goal order rotated.
7. **Arm B matched subset** (about 20 trials) for the MRM comparison.
8. **Produce T1 and F1–F4 on the test set.** Keep the development-set results in the appendix.
9. **Later:** the lane-keeping prior. It makes absolute P (θ₃) meaningful and enables M8 reliability once 30 or more events exist.
