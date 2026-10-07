| model | date | fde_p50 | lat_err_3s_p50 | lat_err_3s_p90 | lon_err_3s_p50 | offroad_rate | phantom_turn_rate | wrong_direction_rate | under_turn_rate | map_reliance_fde_delta_graph_swap | route_reliance_fde_delta_zero_path_node | gate |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v2_zone_weighted_canonical | 2026-10-04 | 1.597 | 0.975 | 4.036 | 1.781 | 0.186 | 0.349 | 0.217 | 0.256 | 0.187 | 0.023 | FAIL |
| ego_v2_A+B+C | 2026-10-04 | 0.733 | 0.149 | 0.847 | 0.847 | 0.036 | 0.009 | 0.000 | 0.630 | 0.526 | 0.041 | FAIL |
| ego_v2 (stale route, re-benchmarked on fixed-route data) | 2026-10-05 | 0.740 | 0.154 | 0.806 | 0.879 | 0.036 | 0.008 | 0.000 | 0.642 | 0.515 | 0.034 | FAIL |
| ego_v2_routefix | 2026-10-05 | 0.591 | 0.114 | 0.678 | 0.739 | 0.035 | 0.015 | 0.000 | 0.404 | 0.606 | 0.144 | FAIL |
| ego_v3_routefix+route_ahead | 2026-10-05 | 1.270 | 0.134 | 1.687 | 0.944 | 0.064 | 0.000 | 0.196 | 1.000 | 0.389 | -0.005 | FAIL |
| ego_v2_routefix_normsel | 2026-10-05 | 0.756 | 0.162 | 0.774 | 1.002 | 0.028 | 0.014 | 0.000 | 0.452 | 0.862 | -0.002 | FAIL |
| ego_v3_routefix+route_ahead_normsel | 2026-10-05 | 0.626 | 0.133 | 0.710 | 0.727 | 0.033 | 0.021 | 0.000 | 0.352 | 0.544 | 0.020 | FAIL |
| ego_v4_multisource_normsel | 2026-10-07 | 0.771 | 0.133 | 0.768 | 0.804 | 0.030 | 0.005 | 0.000 | 0.928 | 0.370 | 0.008 | FAIL |
| ego_v4_multisource_kinsel | 2026-10-07 | 0.579 | 0.112 | 0.520 | 0.759 | 0.017 | 0.020 | 0.000 | 0.202 | 0.370 | -0.001 | PASS |
