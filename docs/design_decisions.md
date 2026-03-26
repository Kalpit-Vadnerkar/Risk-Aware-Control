# Design Decisions

**Last Updated:** 2026-03-20

This document records key design decisions with rationale for committee defense and future reference.

---

## 1. Constraint Tightening vs Weight Modulation

**Decision:** Use constraint tightening, not MPC weight modulation.

**Rationale:**
- Constraint tightening is interpretable in physical units (meters, m/s)
- Provides a path to formal probabilistic guarantees
- Weight modulation has no clear relationship to violation probability and no principled
  way to set the weights from a risk signal
- Stronger research contribution (ties to robust/tube MPC literature)

**For defense:** "Weight modulation provides no formal guarantees on constraint satisfaction.
Constraint tightening can, in principle, be derived from an uncertainty propagation argument."

---

## 2. Residual-to-Constraint Mapping (Method Under Development)

**Decision:** The mapping from residual anomaly signal to constraint value is an open
research question. We will determine the method after Phase 2 experiments provide
calibration data.

**Approaches under consideration:**
- CVaR-based threshold mapping (principled tail-risk estimate)
- Uncertainty propagation / covariance inflation (tube-MPC style, no training needed)
- Empirically calibrated monotonic mapping (fit from Phase 2 data)

**What is NOT considered:**
- Learned sensitivity matrix (requires RL-like training; not feasible in this setup)
- Manual tuning without a calibration principle

**For defense:** "The mapping method is selected based on what the Phase 2 data supports.
The principle — monotonically relate residual deviation to constraint tightening — is
fixed. The exact form will be chosen for calibrability and interpretability."

---

## 3. Handling Detection Latency

**Problem:** The residual signal rises as fault effects accumulate — there is inherent
latency before the signal is strong enough to act on.

**Solution:**
1. **Trend-based component:** React to rising residuals (first derivative) before the
   signal crosses an absolute threshold — early warning rather than threshold detection
2. **Scenario targeting:** Focus on scenarios where Autoware's reaction window is tight
   (20–30m obstacles) so preemptive tightening makes a measurable difference
3. **Continuous framing:** RISE maintains margins proportional to current reliability;
   it is not detecting a fault and reacting — it is always active and always calibrating

---

## 4. Fail-Operational Design

**Decision:** RISE must preserve mission completion, not just prevent violations.

**Rationale:** A system that always says "slow to 1 m/s" achieves zero violations but
zero utility. The research claim is graceful degradation — the vehicle continues the
route at a reduced but appropriate speed, and MRM rate decreases because violations
are handled preemptively.

**Success condition:** Under fault+scenario, RISE reduces collision proxies AND
maintains route completion rate close to the nominal baseline.

---

## 5. MRM Gating (Disabled for Research)

**Decision:** Removed `perception` and `planning` from the Autoware MRM diagnostic gate.

**Rationale:** Autoware's built-in MRM triggers an emergency stop when perception or
planning report diagnostic errors. Any fault injection (dropout, noise, injected objects)
caused those diagnostics to fire, triggering MRM and blocking RISE evaluation. We cannot
evaluate a new safety layer if the existing safety layer preemptively solves the problem.

**What is preserved:** Localization, control, and vehicle MRM gates remain active. Only
perception/planning are excluded.

**File changed:** `autoware/src/launcher/autoware_launch/autoware_launch/config/system/
diagnostics/autoware-main.yaml`

---

## 6. Perception Injection as Scenario Mechanism

**Decision:** Use PerceptionInterceptor (ROS2 node) for all scenario creation and
fault injection, rather than AWSIM source modification or OpenSCENARIO.

**Rationale:**
- AWSIM has no Python/ROS2 NPC scripting API
- OpenSCENARIO requires per-scenario pre-authored files and a full Autoware rebuild
- The interceptor gives full programmatic control without modifying simulator internals
- From Autoware's perspective, injected objects are indistinguishable from real detections
- All runs go through interceptor in passthrough mode, ensuring fair comparison

---

## 7. Fault Scope: Perception Layer Only

**Decision:** Focus fault experiments on the perception layer. Raw sensor faults (LiDAR
noise, IMU bias, localization drift) are out of scope for this research phase.

**Rationale:**
- ST-GAT behavioral residuals respond to what happens *after* perception — they capture
  velocity/position/steering deviations, not raw sensor noise
- Sensor-level faults have an indirect and confounded signal path through Autoware's
  internal processing before reaching ST-GAT
- Perception scenarios (static obstacles, cut-ins) create clear, reproducible collision
  threats that directly test RISE's core function

**See:** `docs/research_notes/dropout_sweep_analysis_mar2026.md` for the experimental
finding that motivated this pivot.
