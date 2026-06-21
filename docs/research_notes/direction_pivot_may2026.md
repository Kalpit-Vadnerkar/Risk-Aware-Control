# Research Direction Pivot — May 2026

**Date:** 2026-05-03
**Status:** Active direction. Supersedes all prior RISE framing.

---

## What Changed and Why

The prior RISE framing treated constraint tightening as the primary output: residuals → CVaR → tighten velocity limit. This had a fundamental problem: **it replicates what Autoware already does with extra complexity**. Autoware already stops conservatively when it detects a problem. Wrapping that in a CVaR signal that tells it to stop more adds no value.

The core reframe: **this is a research contribution about the Pareto frontier between safety and progress, not an engineering contribution about better stopping.**

---

## The Research Claim

Autonomous vehicles operating under conservative planners (Autoware, etc.) occupy a single point in the (safety × goal-achievement) space when they encounter unexpected obstacles: stop, zero progress, zero collision risk. This is not on the Pareto frontier — it sacrifices all progress even in situations where a safe path forward exists.

**Claim:** Residuals from a learned predictive model (ST-GAT) carry a continuous, calibrated signal about which side of the Pareto frontier a given situation falls on — solvable vs. unsolvable. For solvable situations, the residual signal can be used to identify the constraint tightness (speed, margin) at which the vehicle can safely proceed. This moves the vehicle from its stopped state to the Pareto frontier.

**This is different from Autoware because:** Autoware's planner reasons about the obstacle geometrically, at decision time, with fixed policies. The ST-GAT model reasons about the uncertainty of the situation continuously, calibrated against nominal behavior distributions, and outputs a graded signal rather than a binary decision.

---

## Scenario Focus

**Unexpected static obstacle, nominal conditions only.**

- 30m static obstacle in ego lane, goals 007/011/021 (Shinjuku map)
- No perception faults — that's a separate problem and not needed to establish the core claim
- Adjacent lane confirmed available at obstacle placement zone (LL 238/237, ~180m arc from spawn)

The Pareto question for this scenario:
- Situations where stopping IS optimal: no adjacent lane, no intersection nearby, no path forward. The framework should identify these and agree with Autoware.
- Situations where stopping is suboptimal: adjacent lane clear, road geometry allows lane change. The framework should identify these and enable forward progress.

---

## What the Framework Does (not does not do)

**Does:**
- Run the predictive model continuously (not just when stuck)
- Use residuals to characterize the current situation's uncertainty level
- Map that uncertainty level to a constraint value (speed or margin) that satisfies the safety coverage guarantee (conformal prediction)
- Adjust Autoware's operating constraints to that value
- In solvable scenarios: the adjusted constraint enables Autoware's avoidance module to execute the lane change it was already planning but couldn't approve

**Does not:**
- Detect "stuck" as a trigger event and then run a query chain
- Remove obstacles from perception (artificial, meaningless)
- Build a rule-based decision tree over planner state
- Replicate Autoware's geometric obstacle reasoning with a neural network

---

## Data Still Needed

The critical gap: **nominal runs under different constraint values.**

Currently we have ~24 nominal runs at Autoware's default speed constraint. We need nominal runs at reduced speed limits (e.g., 5, 7, 10 m/s) — same routes, same goals — to characterize the residual distribution at each constraint level. This is the calibration set for conformal prediction.

**Why:** Conformal prediction guarantees coverage for a specific distribution. If we want to claim "at 5 m/s, the constraint is safe with 95% coverage," we need nominal data at 5 m/s to calibrate against. We cannot extrapolate from the default constraint calibration set.

Possible mechanism for constraint variation: change the speed limit param in Autoware's route planner, or use the PerceptionInterceptor infrastructure to cap the commanded velocity.

---

## Open Research Questions

In priority order:

1. **Does the residual pattern actually differentiate solvable from unsolvable obstacle situations?** This must be shown empirically. If residuals look identical regardless of whether an adjacent lane exists, the claim collapses. This is the first thing to check on the data we already have.

2. **What is the correct constraint space?** Velocity limit is the obvious first choice. Lateral safety margin (the clearance parameter in the avoidance module) is an alternative. The choice affects what "nominal data at different constraint levels" looks like.

3. **What is the mapping from residual level → safe constraint?** This is the core technical mechanism. Conformal prediction provides the calibration; the architecture of the mapping (linear? learned?) is an open question.

4. **Does the constraint adjustment actually unlock Autoware's avoidance?** When we set the speed limit to V m/s during obstacle avoidance, does Autoware successfully execute the lane change? Or does the geometry/margin prevent it? This requires a test run with `avoidance_for_ambiguous_vehicle.policy: "auto"` active.

---

## Notes on Prior Files

The following research notes are **superseded** by this pivot and should not be used as design references:
- `rise_theory_derivation_may2026.md` — the CVaR → velocity tightening derivation is not the contribution anymore
- `rise_experiment_plan_mar2026.md` — experiment plan from the old framing
- `fault_injection_strategy.md` — faults are out of scope for now
- `rise_theoretical_contribution.md` — contribution statement was wrong

Keep for reference (literature, infrastructure facts):
- `literature_review_rise_gap.md` — lit review still valid
- `experiment_findings_and_mrm_analysis.md` — infrastructure facts still valid
- `metrics_framework.md` — metrics may need updating but infrastructure is useful
