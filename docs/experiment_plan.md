# Experiment plan (next lab sessions)

As of 2026-10-07. Why: current data has 4 real consequences, all IMU, on 2
goals, and every traffic-light fault mode is fail-safe (`findings.md` §5) —
not enough to show anomaly ≠ consequence, or to draw a reliability diagram
(needs roughly 30+ real violations spread over the P range).

## Priorities

1. **IMU severity x geometry** (no new code). Gyro bias 0.02 / 0.04 / 0.08
   rad/s (fixed, not cycling), scale 1.8, stuck — the same onset on a wide
   straight vs approaching a turn / island / narrow segment (existing zone
   gating for the latter, an open-road gate for the former). 3 trials per
   cell: 5 modes x 2 geometries x 3 = 30 trials minimum, 60 across 2 goals per
   geometry. Expectation to test: departures concentrate in tight geometry at
   moderate/high severity.
2. **False-green traffic-light fault + red-light safe set.** New injector
   mode `tl_false_green` (red → green on `traffic_signals_faulted`, zone-gated,
   Arm A). Layer 2 term: crossing the governing stop line while the unfaulted
   signal is red. Validate the labeller on existing TL + nominal data first
   (should find ~0).
3. **Cross traffic** above 10 NPCs for object conflicts. Blocked on one AWSIM
   Unity rebuild (config plumbing already patched,
   `experiments/configs/baseline.json` `targetVehicleCount`).
4. **Arm B batch** (stock MRM gate) on a few P1/P2 cells, to compare lead time
   with Autoware's own emergency stop. Risk: the original MRM deadlocks.
5. **Nominal controls:** ≥ 4 trials per goal used above, same start/goal.

## Open decisions (Kalpit / advisor)

- Goals for wide-straight vs tight-geometry cells.
- Adding `tl_false_green` to the injector.
- Arm B now or later.
- Who rebuilds AWSIM for traffic density, and when.

## Standing checklist for any new data

`analyse_experiments.py --campaign X` → `analyze_mrm_diagnostics.py --batch` →
`plot_routes.py`; for fault campaigns also `compare_fault_vs_nominal.py` and
`verify_{imu,tl}_zone_arming.py`. Delete broken trial directories before
re-running (auto-resume counts any trial with a `result.json`). Regenerate
`turn_zones.json` / `tl_zones.json` when goals change.
