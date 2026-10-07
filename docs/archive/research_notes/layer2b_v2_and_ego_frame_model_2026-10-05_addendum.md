# Addendum (2026-10-05): stale-route bug, route-ahead input, selection fix — and where the model work stops paying

Continues `layer2b_v2_and_ego_frame_model_2026-10-04.md`.

## Findings
1. **Stale-route data bug.** In 38/45 nominal bags and ~half the fault bags the
   first `/planning/mission_planning/route` message is a leftover route from the
   PREVIOUS goal; the real route arrives ~1.2s later and every later message
   repeats it. `extract_route_from_bag()` took the first, so path_node flags and
   the TL expectation checker (`traffic_light_discrepancy`) were built on the
   wrong route for most trials. Fixed for the ego schemas (last message);
   bbox_v1 frozen on the old rule for consistency with the canonical model.
   **Consequence for existing results:** every bbox_v1-based result (canonical
   model, Layer 1 calibration, fault inspection, TL severity sweep) used the
   stale route for those trials. Route reliance was ~0 for that model, so the
   impact is probably small, but traffic_light_discrepancy is a model INPUT —
   worth re-checking Layer 1 numbers once an ego model is promoted.
2. **Checkpoint selection was driven by the TL heads.** Legacy selection = raw
   sum of head val losses; TL/accel heads are ~20x position's scale and overfit
   within a few epochs. ego_v3's first run selected epoch ~3 (before position
   learned to use the route): under-turn 100%, wrong-direction 20%. Added
   `--selection normalized` (mean of each head's val loss / its epoch-1 value).
3. **Training is ~19x faster** (per-graph tensor cache, uint8 adjacency, batched
   GCN; identical outputs): ~58s/epoch incl. validation, was ~9.5 min.

## Benchmark (held-out CAL, one seed each — expect ~±0.5-1pp off-road / ±0.15m FDE run-to-run)
See `experiments/analysis/model_safety_benchmark/benchmark_table.md`. Best
balanced model: **ego_v3 (route fix + route-ahead), normalized selection** —
FDE p50 0.63m, lateral 3s p50 0.13m / p90 0.71m, under-turn 35%, wrong-direction
0%, off-road 3.3% (gate 2%: FAIL). The route-ahead input itself is excellent
(heading change at actual travel distance correlates 0.98 with the real turn).

## Where model work stops paying
Remaining off-road forecasts are concentrated at 13 TL-intersection locations
where the REAL path's clearance to the road edge is only 0.33-0.75m: the
forecast point is inside a road lanelet in 69/69 cases, the footprint clips the
edge by ~0.3m. The Layer 2 nominal P(violation) floor:

| model | embedding mean | p95 | p99 (sets 1%-FAR threshold) |
|---|---|---|---|
| bbox_v1 canonical | 0.125 | 0.71 | ~0.98 |
| ego_v2 (stale route) | 0.042 | 0.31 | ~0.95 |
| ego_v2 route fix, normsel | 0.039 | 0.24 | 0.93 |
| ego_v3 route-ahead, normsel | 0.038 | 0.22 | 0.89 |

The mean and p95 kept improving; the p99 tail did not. That tail is no longer
the point forecast — it's the open-loop bootstrap tube: its honest 3s lateral
spread (p90 ~0.7m) exceeds the clearance at those tight spots, so many
counterfactuals clip the edge, while real nominal driving never does because
the controller keeps the car in lane. Open-loop counterfactuals ignore that
closed-loop correction. That is a Layer 2 calibration question (is
P(violation)=0.9 calibrated if nominal violations are 0?), not a Layer 1 one.
