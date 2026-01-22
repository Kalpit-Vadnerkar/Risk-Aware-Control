# Research Note: Intrinsic Delay Problem Analysis

**Date:** 2026-01-21
**Topic:** Addressing committee concern about detection latency
**Status:** Key defense point

## The Committee Concern

> "There's an intrinsic delay - you can't detect a fault until it has affected the AV's behavior. For mild faults, you don't need such a complex system."

## Problem Analysis

### Timeline of Fault Detection

```
t=0:   Fault occurs (e.g., IMU drift begins)
t=1:   Fault affects vehicle state (steering overcorrects)
t=2:   Residual spikes (we detect deviation from prediction)
t=3:   CVaR increases → constraints tighten
t=4:   Safer behavior kicks in
```

### Detection Latency Components

1. **Observation delay:** Time to observe faulty behavior
2. **Prediction delay:** One prediction window (up to 3 seconds at 10Hz, 30 steps)
3. **CVaR estimation delay:** Window size for reliable statistics
4. **Control response delay:** Time for new constraints to affect behavior

**Total estimated latency:** 1-5 seconds depending on fault severity

## Our Multi-Pronged Solution

### Solution 1: Preemptive Tightening via Trend Detection

Instead of reacting to CVaR magnitude, react to CVaR *rate of change*:

```python
# Reactive (original)
if CVaR > threshold:
    tighten_constraints()

# Proactive (our approach)
if d(CVaR)/dt > trend_threshold:  # CVaR is INCREASING
    preemptively_tighten_constraints()
```

**Key insight:** Rising CVaR indicates degrading situation BEFORE it becomes critical.

### Solution 2: Severity-Targeted Design

**Honest framing:** Our system is NOT designed for mild faults.

| Severity | Baseline Outcome | Our System Value |
|----------|------------------|------------------|
| Mild | Handles fine | No benefit (acknowledged) |
| Moderate | Degraded but safe | Marginal benefit |
| Severe | Collision likely | **Collision prevented** |
| Critical | Certain collision | **Reduced severity** |

**Key point:** For severe faults, residuals spike quickly, making detection delay proportionally smaller.

### Solution 3: Continuous Safety Margin Adjustment

Reframe from "fault detection → reaction" to:

> "Continuous risk monitoring that maintains safety margins proportional to prediction reliability, ensuring graceful degradation when nominal behavior assumptions weaken."

This shifts focus:
- ❌ "We detect faults and react" (delay problem)
- ✅ "We continuously adapt safety margins" (no specific detection needed)

## Quantitative Justification

### Severe Faults → Faster Detection

Empirical observation from T-ITS paper (Table 4):
- Severe IMU faults: Residuals exceed threshold within 0.5-1.0 seconds
- Mild IMU faults: May take 2-5 seconds to detect

### Even Delayed Response Helps

For a vehicle traveling at 10 m/s approaching an obstacle:
- Without our system: Collision at t=5s
- With our system (2s detection delay): Emergency response at t=2s, collision averted

**Key metric:** $TTC - t_{detection} > t_{response\_required}$

If TTC is 5s, detection at 2s still leaves 3s for response.

## Validation Plan

1. **Characterize detection latency:** Measure $t_{detection}$ for each fault type/severity
2. **Show severity correlation:** Severe faults → faster detection
3. **Demonstrate value:** Collision rate reduction for severe faults
4. **Define operating envelope:** "Designed for faults of severity X or greater"

## Committee Defense Script

> "You're correct that there's inherent detection latency. We address this in three ways:
>
> First, our preemptive tightening responds to CVaR trends, not just thresholds. When CVaR starts rising, we tighten constraints before reaching critical levels.
>
> Second, we're honest about our operating envelope. This system is not designed for mild faults - existing safety systems handle those adequately. Our contribution is for severe faults where baseline approaches fail.
>
> Third, severe faults cause large residual spikes quickly, making detection proportionally faster. In our experiments, severe IMU faults are detected within 1 second, leaving adequate time for response.
>
> The key metric is TTC minus detection latency. If that exceeds the required response time, our system provides value. Our experiments demonstrate X% collision reduction for severe faults where baseline systems fail."

## References

- Detection latency characterization: [To be added from experiments]
- Preemptive control in safety-critical systems: [Standard practice in ABS, ESP]
- Operating envelope definitions: [SAE J3016 ODD definitions]
