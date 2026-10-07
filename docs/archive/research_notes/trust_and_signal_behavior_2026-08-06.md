# Trust-building and signal-behavior plots (2026-08-06)

**Context.** Per the 2026-08-06 reframe (session memory
`project_operational_degradation_reframe`), work stopped chasing
detection-rate/lead-time numbers within a binary-alarm framing and instead
built the two things that should exist *before* any of that: (1) plots that
ground the belief-divergence mechanism's own credibility, and (2) plots of
what the SPRT/sequential-evidence signal (`p_fault_motion`, `p_fault_tl`,
`p_fault_combined`) is actually measuring, judged on interpretability and
shape, not threshold-crossing. Step 3 (Autoware planning/control interfaces)
is explicitly gated on 1 and 2 "holding up."

**They don't cleanly hold up.** Both sets of plots surfaced concrete,
diagnosable problems — not politely-inconclusive ones. Per this project's
standing principle, that's the actual finding, not a plotting bug to route
around. Documented here so the next session designs against it instead of
re-discovering it.

New tooling: `experiments/scripts/plot_calibration_diagrams.py` (step 1,
needs ROS+model) and `experiments/scripts/plot_sprt_signal_behavior.py`
(step 2, pure pandas over `st_gat/results/h30_30/traces/*.csv`, no ROS
needed). Outputs in `experiments/analysis/calibration_diagrams/` and
`experiments/analysis/sprt_signal_behavior/`.

---

## 1. Trust-building plots (step 1)

Computed on the held-out calibration split (`cfg.CAL_DIR`), same data
`check_calibration.py`/`calibrate_tl_discrepancy.py` already use.

### 1.1 Matching std(z)≈1 is not the same as being calibrated

`check_calibration.py`'s headline numbers (std(z) 0.85–1.13 across all 6
Gaussian heads) read as "solid calibration" as a single summary statistic.
The full coverage curve (`coverage_curves.png`) tells a different story: at
every nominal confidence level from 5% to 99%, empirical coverage
**overshoots** nominal for all 6 features — e.g. `traffic_light_color`'s
max gap is 0.275 (nominal 60% interval actually catches ~85% of real
outcomes). `zscore_histograms.png` shows why: the observed z-distribution
is **leptokurtic**, not Gaussian — visibly over-peaked near zero relative
to the N(0,1) reference curve, which is exactly the signature of a
variance that's *on average* right (std≈1) but composed of "mostly small
errors + occasional large ones" rather than a genuinely Gaussian spread.
A single std(z) number can't see this; the coverage curve and histogram
shape can.

**Why this matters beyond step 1 itself:** the SPRT statistic in
`_gaussian_llr` is a *variance-inflation* test — it's specifically testing
whether the observed z² looks like it came from a c²-inflated Gaussian.
A leptokurtic nominal residual distribution (heavier tails than N(0,1) at
matched variance) means nominal data will *routinely* produce individual
z² values large enough to look like "the variance just inflated by 2x,"
even with no fault present. This is the direct mechanistic explanation for
finding 2.1 below — the two findings are not independent.

### 1.2 Predicted uncertainty doesn't widen with horizon for 5 of 6 Gaussian heads

`horizon_widening.png`: for each feature, predicted std (the model's own
claim) vs. actual RMSE (ground truth), both as a function of seconds into
the 3s predicted horizon.

- **`position`** is the one feature that behaves as expected: predicted
  std grows roughly in step with actual RMSE across the full horizon
  (slightly over, consistent with the overcoverage in 1.1).
- **`velocity`, `steering`, `acceleration`, `traffic_light_color`,
  `traffic_light_confidence`**: predicted std stays nearly flat (some even
  dip slightly before rising) across the full 3s horizon, while actual
  RMSE grows monotonically and substantially — by t=3s, actual error is
  roughly **2–4x** the model's claimed uncertainty for these features.

This is computed only at 1-step-ahead (t=0) currently by
`check_calibration.py`/`residuals.py`'s SPRT statistics — so the
"calibrated" story (1.1's coverage curves, and every downstream
`p_fault_*` number) is only being checked at the *easiest* point of the
horizon. If the "lead time" pillar of the dissertation claim (P1.3 in
TODO.md) ever reports confidence further out than 1 step ahead, this gap
needs to be closed first — right now the model's own uncertainty
literally doesn't know it should be less sure 3 seconds out for 5 of 6
tracked signals.

**Not yet diagnosed further this session** (would need to look at whether
this is a decoder architecture issue — e.g. the LSTM/transformer variance
head effectively ignoring the horizon-step embedding for these heads — or
something more specific to how those heads are trained). Flagged as the
top open question from step 1, not resolved.

### 1.3 traffic_light_discrepancy: temperature scaling barely moves the shape

`tl_discrepancy_reliability.png`: raw (T=1.0) and temperature-scaled
(T=1.20) reliability curves sit almost on top of each other, both bowed
well above the diagonal in the low-to-mid predicted-probability range
(e.g. at predicted≈0.45, actual frequency≈0.43–0.48; at predicted≈0.55,
actual≈0.86–0.94). Temperature scaling is a single global scalar — it can
sharpen or flatten the whole curve but can't fix a shape that's
non-monotonically miscalibrated like this. The practical read: when this
head reports "possible discrepancy" in the 0.4–0.6 range, the real
frequency is often much higher than that (badly *under*confident in that
band specifically), which combined with finding 2.2 below is the more
useful diagnosis than "ECE improved from X to Y."

---

## 2. SPRT signal-behavior plots (step 2)

Read directly from the already-computed traces (`nom_v11` + all 8 fault
campaigns, `imu_fault_s1/s3/scale/stuck`, `tl_fault_s2/s3/s4/ramp`). Recall
`p_fault_* = sigmoid(S_t)` where `S_t` is a reset-at-zero SPRT accumulator
(`S_t >= 0` always) — the floor is 0.5, not 0.0.

### 2.1 The signal does not stay quiet under nominal noise

`nominal_floor.png` (all 39 `nom_v11` trials overlaid) visually reads as
near-constant spiking rather than a flat floor. Checked numerically rather
than trusting the visual impression (a plotting line-width artifact was a
real possibility given 10Hz sampling): confirmed real. Averaged across all
39 nominal trials:

| | mean value |
|---|---|
| fraction of nominal time at the floor (0.5) | 92.0% |
| fraction of nominal time with `p_fault_combined` > 0.99 | 6.2% |
| mean number of full resets-to-floor per trial | 17.6 (range 5–35) |
| implied mean interval between full saturation events | ~9s (trial length ÷ resets) |

So the signal is quiet *most* of the time by row-count, but it fully
saturates to near-maximum confidence-of-fault, from a standing start, on
the order of **every 9 seconds**, during ordinary nominal driving with no
fault present. A monitor with that behavior can't be read as "confidence a
fault has occurred" in any usable sense yet — the false-alarm floor is not
low enough for the signal's peaks to mean anything on their own.

### 2.2 Root cause found: it's overwhelmingly the TL-discrepancy branch

Broke the same check down by sub-feature SPRT accumulator (fraction of
nominal time each spends above S_t≈4.6, i.e. p>0.99):

| feature | frac. nominal time p > 0.99 |
|---|---|
| `sprt_position` | 5.5% |
| `sprt_velocity` | 5.2% |
| `sprt_steering` | 6.9% |
| `sprt_acceleration` | 6.5% |
| `sprt_tl_color` | 8.4% |
| `sprt_tl_confidence` | 10.0% |
| **`sprt_tl_discrepancy`** | **95.8%** |

The motion features and the two Gaussian TL heads are noisy but roughly in
the same range (5–10%, itself worth revisiting given 1.1/1.2, but not the
dominant effect). `traffic_light_discrepancy`'s Bernoulli SPRT branch
alone spends **96% of nominal driving time already past p=0.99** —
it single-handedly explains why `p_fault_tl` (mean 23.3% of time >0.99)
and, through the combined LLR sum, `p_fault_combined` are chronically
saturated.

**Mechanism, consistent with 1.3 above:** `_bernoulli_llr` compares
`actual ~ Bernoulli(pred_prob)` against `H0: actual ~
Bernoulli(base_rate=0.093)` — and `traffic_light_discrepancy` is genuinely
positive ~9.3% of 1-step predictions *even in pure nominal driving*
(that's where `base_rate` itself comes from). At 10Hz that's not a rare
tail event, it's a recurring ~1-in-11-steps occurrence, structurally
incapable of giving the reset-at-zero accumulator enough quiet interval to
decay back to floor between hits — so it saturates almost continuously
regardless of fault state. This is a different, more specific diagnosis
than the earlier "0/8 conformal detection" framing (see
`calibrate_tl_discrepancy.py`'s docstring) — that was about the
*conformal-threshold* framing being the wrong lens for a Bernoulli head;
this is about the *base rate itself* being too high, relative to 10Hz
sampling, for a memoryless sequential accumulator to treat it as rare
evidence at all.

### 2.3 Fault-onset-aligned view: no clearly visible qualitative change at onset

`fault_reaction_grid.png` (all 8 fault campaigns, x-axis = seconds
relative to fault onset): the same saturate/reset pattern from 2.1
continues through both the pre-onset and post-onset windows in every
panel, without an obvious visual step-change right at t=0 (the red line).
Given 2.1/2.2, this is the expected consequence, not a separate finding —
if the nominal floor is already saturating every ~9s, a real fault's
effect on top of it is not going to be visually legible until the
TL-discrepancy branch's false-alarm rate is brought down. This plot should
be re-examined once 2.2 is addressed, not read as "the mechanism doesn't
react to faults" — the discriminability test (referenced in the session
brief) already established real, selective per-feature NLL elevation
under fault at the residual level; this SPRT view is currently drowned out
by one branch's noise floor, not evidence against that underlying result.

---

## 3. First calibration-fix attempt (same day, follow-up): Gaussian -> Student-t + horizon-conditioned decoder

Per Kalpit's explicit direction: since 1.2's architecture problem needs
fixing regardless of what a cheap post-hoc recalibration would show, skip
straight to the full fix and retrain, rather than diagnosing further with
a fast patch first. Two changes made together in `st_gat/model/model.py`/
`loss.py` (see that commit's message for the full mechanism):

1. Every continuous head now predicts Student-t `mean`/`scale`/`dof`
   instead of Gaussian `mean`/`var` — lets the model represent heavy
   tails directly instead of being forced into a bell curve (targets 1.1's
   leptokurtosis finding).
2. The flat single-shot `Linear(d_h, dim*T_out)` heads were replaced with
   `_StepConditionedHead` — a small MLP conditioned on `[h_last ; a
   learned per-horizon-step embedding]`, applied independently at each of
   the 30 future steps, instead of one weight matrix with no structural
   mechanism for step *t*'s prediction to differ from step *t+1*'s
   (targets 1.2's horizon-widening finding).

Retrained (`python3 -m st_gat.train`, same hyperparameters as before —
200-epoch cap, patience 20, batch 128, lr 4e-4). Early-stopped at **epoch
55**, best val raw error (position L2 + 0.8·velocity L1) **0.0158** —
slightly *better* point accuracy than the pre-fix model's 0.0168.

Validation required rebuilding the calibration check itself: a raw
z-score has no fixed reference distribution to check against once dof
varies per sample, so `check_calibration.py`/`plot_calibration_diagrams.py`
were rewritten around the **probability integral transform (PIT)** —
`u = F(actual; that sample's own predicted mean/scale/dof)`, which should
be Uniform(0,1) under correct calibration regardless of family (Gneiting
et al., JRSS-B 2007). Caught and fixed a real bug in the first version of
this rewrite along the way: averaging position/velocity's two per-dim PIT
values per sample instead of flattening across dims — the mean of two
independent Uniform(0,1) values is NOT itself Uniform(0,1) (triangular,
std ≈0.204 not ≈0.289), which manufactured apparent miscalibration for
exactly those two features. Fixed by flattening (pooling x/y as separate
observations, same convention the original z-score version used), then
cross-checked `plot_calibration_diagrams.py` against `check_calibration.py`
independently — both now agree exactly.

**Result: mixed, not a clean fix.** Coverage-curve max-gap, before
(Gaussian, §1.1) vs. after (Student-t, corrected PIT):

| feature | before (Gaussian) | after (Student-t) | direction |
|---|---|---|---|
| position | 0.197 | **0.066** | better (3x) |
| steering | 0.156 | **0.125** | better |
| velocity | 0.235 | 0.243 | ~unchanged |
| acceleration | 0.270 | **0.310** | **worse** |
| traffic_light_color | 0.275 | **0.376** | **worse** |
| traffic_light_confidence | 0.263 | **0.360** | **worse** |

Every feature still decisively fails a Kolmogorov-Smirnov test against
Uniform(0,1) (KS statistic 0.08–0.32, p-values effectively zero at
n=11684) — not surprising at this sample size, but real. `check_calibration.py`'s
central-interval numbers give the practically-readable version: cov68%/
cov95% are 73–89%/90–96% across all 6 features (want 68%/95%) — still
systematically overcovered (predicted intervals too wide), just less
uniformly and less severely than the old Gaussian model's shape mismatch.

**Horizon widening is still broken for the same 5 of 6 features** —
`horizon_widening.png` looks essentially unchanged from §1.2: only
`position`'s implied std (now `sqrt(scale² · dof/(dof-2))`) tracks actual
RMSE growth across the 3s horizon; velocity/steering/acceleration/both TL
heads still show a nearly flat or dipping predicted std while actual RMSE
grows 2–4x. **The architecture change did NOT fix this on its own** —
giving the decoder per-step conditioning capacity is necessary but
evidently not sufficient; the model apparently didn't learn to use that
capacity for most heads.

**Root-cause hypothesis, not yet tested:** `Trainer`'s checkpoint-selection
/ early-stopping criterion (`v_raw = position_l2_raw + 0.8·velocity_l1_raw`,
see `trainer.py`'s own docstring for why NLL was deliberately excluded
from it — a real, separate, previously-discovered pathology) is blind to
calibration quality for every feature, and blind to accuracy *or*
calibration for `acceleration`/both TL heads entirely. Training stops the
moment position/velocity point-accuracy plateaus (epoch 55 here),
regardless of whether the scale/dof heads for other features — which get
weaker, more indirect gradient signal via beta-NLL reweighting — have
converged. This plausibly explains the mixed result directly: whichever
epoch happened to have the best position/velocity accuracy is not
necessarily the epoch where acceleration/TL-heads' calibration was best,
and there's currently zero training signal rewarding horizon-widening
specifically for the checkpoint that gets saved. Not yet fixed or
retrained against — a real design decision (how to fold a calibration
signal into the stopping criterion without reintroducing the "NLL
prefers an undertrained model" pathology `v_raw` was built to avoid)
that needs a choice before the next retrain, not a guess.

`experiments/analysis/tl_calibration/temperature.json` refit against the
new model's logits (T 1.20 → 1.40, ECE 0.0168 → 0.0133) — the
`traffic_light_discrepancy` Bernoulli head itself is unaffected by the
Gaussian→Student-t question, its reliability-curve shape (underconfident
in the 0.3–0.6 predicted-probability range) is essentially unchanged.

`st_gat/residuals.py`'s trace schema updated to match (`_pred_scale`/
`_pred_dof`/`_pred_implied_var` columns, `_nll`/`_residual` column NAMES
unchanged so downstream scripts keep working, values now from the real
Student-t density). All 59 nominal+fault trials' traces regenerated
against the new model. `_gaussian_llr` (the SPRT test) is an explicitly
documented stopgap — standardizes by the Student-t scale now but its own
derivation still assumes both hypotheses are Gaussian; per Kalpit, not
re-derived until the calibration fix above is actually validated, which
per this section it isn't yet.

---

## Where this leaves step 3

Per the reframe's explicit gating ("only if 1 and 2 hold up"): still no,
but the open items are now sharper:

1. **Calibration is measurably better for `position`/`steering`, flat for
   `velocity`, and measurably worse for `acceleration`/both TL heads** —
   likely explained by the checkpoint-selection-criterion gap above, not
   yet fixed. Next step is a decision (how to make the stopping criterion
   calibration-aware) followed by another retrain, not more diagnosis.
2. **Horizon widening still doesn't work for 5 of 6 heads** despite the
   architecture now having the capacity for it — same likely cause as
   above (training/selection never rewards it), not a representational
   limit anymore.
3. **2.2 (TL-discrepancy base rate)** — the dominant driver of the
   original SPRT-saturation finding, still entirely unaddressed. Explicitly
   deferred again this session (per Kalpit: revisit SPRT only after
   calibration is actually validated, which it still isn't).

All three are concrete enough to be next session's actual work, ahead of
any Autoware planning/control interface research.
