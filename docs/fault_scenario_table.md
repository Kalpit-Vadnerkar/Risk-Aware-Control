# Fault Injection Scenario Table

This table is the scope definition for the data collection campaigns: for
each fault type, it states exactly where the fault is allowed to arm, why
that scenario is the one that matters, and what outcome is expected if the
gating and the underlying hypothesis are both correct. It is the basis the
trial structure is built from — a campaign only exists because a row here
says a given (fault type, scenario) combination is worth collecting.

## How zones are computed

`experiments/scripts/compute_turn_zones.py` derives, per goal, from the
actual Autoware-planned route (`/planning/mission_planning/route`, recorded
in `nom_v11`) rather than driven GT trajectories:

- **Turn zones**: contiguous stretches of the route's lanelet-centerline
  geometry where curvature exceeds `0.02 rad/m`, merged if separated by
  ≤20m, discarded if shorter than 6m (filters single-vertex map-digitization
  kinks, not real turns). Only used for gating if at least one spanned
  lanelet is explicitly tagged `turn_direction=left/right` in the Lanelet2
  map — an unambiguous, sharp, real intersection turn.
- **Bias lead-in zones**: the point 10m of arc length before each turn zone,
  for `imu_bias` (which needs accumulation time before the turn, not the
  turn itself — see Mechanism column). Same first-entry-vs-closest-approach
  distinction as TL zones applies here in miniature: `plot_fault_plan.py`'s
  injection marker is the first route point within the zone's radius (15m
  turn / 8m lead-in), a few metres before the zone center itself, not the
  center — small at this radius, but the same principle.
- **Curved-road zones (excluded from gating, future work)**: detected
  curvature where no spanned lanelet is tagged but all are sequentially
  connected (a real, if gentle, curve — e.g. a diagonal street bending into
  the grid near goal_007/012's shared start). Decided 2026-07-27 not to
  gate on these: real yaw rate through a gentle curve is meaningfully
  smaller than through a tagged intersection turn, and folding both into
  one "turn" scenario risks labeling a weak, noise-adjacent divergence
  signal identically to a strong, unambiguous one. Worth its own dedicated
  "curved road" scenario later, once its yaw-rate magnitude has been
  checked to actually be distinguishable from EKF noise.
- **Lane-change zones (excluded from gating, future work)**: detected
  curvature that does NOT classify as a real turn (spans lanelets not
  sequentially connected) — logged for visibility. None of the three
  current production routes (goal_007/012/026) contain one; kept in the
  schema for when a route does.
- **Unreachable (pre-runway) zones are flagged, NOT dropped from the data.**
  `fault_injector.py`'s runway gate — shared by TL and IMU arming alike —
  only clears once the vehicle has entered AND EXITED the first real TL
  zone it passes, so that first zone (every goal, by construction) and
  occasionally an early turn zone too (confirmed for goal_007: its first
  real turn at 22m arc-length sits before its own 62m runway-clear point)
  can never actually host a fault. Both `compute_turn_zones.py` and
  `compute_tl_zones.py` compute the runway-clear arc-length and mark any
  zone at or before it `"reachable": false`. **This was originally
  implemented by dropping the zone from the output file entirely — that
  broke the runway-clear detection itself** (2026-07-27 incident):
  `fault_injector.py` loads TL zone positions from this same file to
  figure out where the runway clears in the first place, so removing the
  first zone made it search for "enter then exit a zone" starting from the
  *second* real intersection instead, pushing the computed runway-clear
  point a full intersection too late — live, not just in the plots, since
  the plot's runway computation intentionally mirrors the same loader. No
  filtering is actually needed for correctness: the arming state machine
  (`waiting_runway` → `waiting_zone` only happens exactly when the vehicle
  has just exited that same first zone) already makes it structurally
  impossible to arm there, with or without the flag. `reachable` exists
  purely for reporting/plotting, and downstream code must never filter a
  loaded zone list by it before using it for runway or arming logic.

Using route geometry instead of driven GT eliminates position noise as a
source of false positives entirely (no arc-length resampling of noisy GPS,
no jitter-driven threshold tuning) and ties zones to the same lanelet
sequence Autoware itself plans against, not to trial-to-trial driving
variance.

Current zone counts: goal_007 = 4 turns (1 unreachable), goal_012 = 3 turns
(0 unreachable), goal_026 = 2 turns (0 unreachable) —
`experiments/configs/turn_zones.json`. Visual verification:
`experiments/scripts/plot_fault_plan.py --campaign <name>` renders each
campaign's actual gating zones, radii, runway-clear point (correct
position, post-fix), and route start/goal over the route map; unreachable
zones are shown greyed out, not hidden.

**TL zones carry a real `traffic_light_group_id`, not just a position.**
`experiments/scripts/compute_tl_zones.py` walks the same route and records
one zone per traffic_light regulatory element actually attached to a
lanelet the route drives through — confirmed empirically that a regulatory
element's own map id IS the `traffic_light_group_id` published in
`/perception/traffic_light_recognition/traffic_signals`. That message
routinely carries 2-6 groups at once (other intersections/approaches in the
perception lookahead), so `fault_injector.py` now captures the relevant
group id at the moment a fault arms (`_tl_fault_group_id`) and scopes every
mutation to only that group — `tl_blackout` drops just that one light from
the array, `tl_confidence`/`tl_oscillate`/`tl_unknown` only touch its
elements. This is also the exact mechanism a future ST-GAT `tl_confidence`/
`tl_is_green` feature should use to know which light's state is the
relevant one at a given point on the route, rather than picking arbitrarily
from whatever the message happens to contain. The TL zone injection marker
in `plot_fault_plan.py` is the FIRST point along the route (walking forward
from route start) that falls within the zone's radius of its real center —
not the point of closest approach overall, which visual review showed
sitting so close to the light that it looked like there was barely any
approach distance left for the fault to matter (closest-approach and
first-entry can differ by a full zone diameter for an off-route center — a
regulatory element's position is the average of its physical signal-head
positions, which sit beside/above the road, not on the driven centerline).
The zone CIRCLE itself is still drawn at the true (possibly off-route)
center, since that's the real position `fault_injector.py` measures
distance against; a black square marks that center explicitly now too, so
it's clear what each circle is centered on.

**TL zone radius increased 20m -> 40m (2026-07-27), after the above
visualization made the problem visible.** The 20m default's original
justification (avoiding merging adjacent intersections into one zone) had
been validated against `_traffic_light_points_in_bbox`'s pre-fix behavior —
one point per signal-head linestring, not per regulatory element — so its
"~5 real TL points cluster within 12-40m" finding was mostly counting
multiple heads of the SAME intersection, not 5 distinct ones. Measuring
real, distinct regulatory-element spacing on the current 3 routes (after
that fix) gives a minimum gap of 112m between consecutive real
intersections — 40m leaves a comfortable 32m margin against that (2×40=80 <
112) while roughly doubling how far out a fault can start influencing the
vehicle's stop/go decision, which a real behavior planner typically
commits to well before the physical stop line. Current counts: goal_007 = 5 TL zones (1
unreachable — the vehicle's first-ever TL zone is always consumed by the
runway gate, see above), goal_012 = 6 (1 unreachable), goal_026 = 6 (1
unreachable) — `experiments/configs/tl_zones.json`. Fewer than early
bbox-based counts even before accounting for reachability, because a lane
that goes straight through an intersection without its own `traffic_light`
regulatory element (an uncontrolled or right-of-way-governed approach)
correctly gets no zone at all now, instead of one borrowed from a nearby
cross-street signal the vehicle was never actually reacting to. Their
radii are the literal `_DEFAULT_TL_ZONE_RADIUS_M` /
`_DEFAULT_IMU_TURN_ZONE_RADIUS_M` / `_DEFAULT_IMU_LEADIN_ZONE_RADIUS_M`
constants imported from `fault_injector.py`, not independently guessed —
the plotted circles are the real gating radii.

## Scenario table

| Fault type | Magnitude | Scenario | Consequential? | Mechanism / why | Expected behavior | Mission outcome |
|---|---|---|---|---|---|---|
| `nominal` (nom_v11) | none — no fault injected | entire route | **No** | Baseline: no fault configured at all. Not a negative control for any specific fault (that's `imu_bias`/imu_fault_s1's job below) — it's the reference distribution everything else is compared against (`compare_fault_vs_nominal.py`'s pooled stats, `plot_fault_impact.py`'s overlaid nominal series) | Normal driving behavior throughout; whatever "no fault" baseline variance looks like | mission_complete |
| `imu_bias` (imu_fault_s1) | gyro 0.03 rad/s | bias lead-in (10m) + turn | **No** | Bias is within EKF/localization's noise-rejection band — confirmed absorbed even during a turn | EKF-vs-GT divergence stays within nominal noise; no MRM | mission_complete |
| `imu_bias` (imu_fault_s3) | gyro 0.08 rad/s | bias lead-in (10m) + turn | **Yes** | Constant bias integrates against genuine nonzero yaw rate during the turn → EKF heading diverges from GT | Visible EKF-vs-GT divergence during/after the turn | mission_complete or MRM-triggered (both valid outcomes) |
| `imu_scale_factor` (imu_fault_scale) | gyro ×1.8 | turn zone only | **Yes** | Scale-factor error is proportional to true yaw rate — zero effect at zero yaw rate, so it is turn-triggered by construction | No effect on straights (by design); measurable EKF-vs-GT divergence during the turn | mission_complete or MRM-triggered |
| `imu_stuck_at` (imu_fault_stuck) | gyro frozen at activation value | turn zone only | **Yes** | Freezing gyro mid-turn stops reported yaw rate from tracking true yaw rate the moment curvature changes | No effect if frozen during a straight (frozen-at-~0 ≈ correct); divergence during the turn | mission_complete or MRM-triggered |
| `tl_oscillate` (tl_fault_s2) | 5s period, 15s cap | TL zone (near intersection) | **Yes** | Oscillating color confuses the stop/go decision only when a decision is actually being made, i.e. near a signal | Hesitation / wrong stop-go decision near the intersection | mission_complete or MRM-triggered |
| `tl_unknown` (tl_fault_s3) | 15s cap | TL zone | **Yes** | UNKNOWN classification forces over-cautious behavior (unnecessary stop) at a real signal | Unwarranted stop/delay at the intersection; possible timeout-style MRM | mission_complete (delayed) or MRM-triggered |
| `tl_blackout` (tl_fault_s4) | 15s cap | TL zone | **Yes** | No signal at all forces the planner into its no-information fallback right at a decision point | Fallback behavior at the intersection (proceed-with-caution or stop) | mission_complete or MRM-triggered |
| `tl_confidence_ramp` (tl_fault_ramp) | confidence decays 0.1/s to 0 | TL zone | **Yes** | Same mechanism as oscillate/unknown but confidence degrades continuously rather than switching — tests threshold-crossing behavior specifically | Behavior tracks confidence crossing the planner's internal threshold, not fault onset | mission_complete or MRM-triggered |

## Not currently in scope

- **Curved-road zones are computed but no longer plotted.** They were
  briefly drawn as a point marker in `plot_fault_plan.py`, which is the
  wrong representation — a curved road is a stretch of route (the run that
  triggered curvature detection has a real start and end arc-length), not
  a discrete corner the way a tagged intersection turn is. Removed from the
  plot rather than shown misleadingly; if this becomes an active future
  scenario, it should be drawn as a highlighted sub-segment of the route
  line (using the run's actual start/end indices, already computed in
  `compute_turn_zones.py`, just not currently written to the output file),
  not a single point.
- **`imu_bias_ramp` (`imu_fault_ramp`, `collect.sh`) is kept as a working
  campaign but not part of the active scenario table.** It has no zone
  gating at all (accumulates continuously from runway-clear regardless of
  geometry, deliberately — see its own design rationale), which doesn't fit
  this table's per-scenario structure. Not deleted — "good to have" if a
  geometry-independent accumulation case is ever needed again — just not
  routinely collected or plotted alongside the zone-gated fault types.
- **Deliberate "fault active but inconsequential" (negative-control)
  scenarios for scale/stuck/bias on a straight segment — considered and
  deferred, not adopted.** The question: since `imu_bias_ramp`'s value
  partly comes from showing a fault firing on straights too, should
  scale/stuck/bias(0.08) also get an intentional "arm here, expect no
  effect" scenario for calibration/interpretability purposes? Recommendation
  is not to add this now: `imu_bias` (imu_fault_s1, 0.03 rad/s) already
  supplies a real, always-inconsequential negative control, and for
  scale/stuck specifically "inconsequential" is synonymous with "zero yaw
  rate" by construction — deliberately firing them on a straight would
  mostly re-confirm something already implied by the fault's own
  definition, not reveal new information, while reopening the exact
  contamination/scope debate this session's zone-based redesign was meant
  to settle. Revisit if ST-GAT results suggest the negative-control coverage
  is actually insufficient.

## Open items, not yet resolved

- **TL phase during IMU-turn zones is uncontrolled.** Every turn zone in
  this map sits at a signalized intersection, so an IMU fault firing during
  a turn also coincides with whatever TL phase AWSIM happens to be showing
  at that moment — not held constant across trials. Decided (2026-07-26) not
  to control for this now; worth revisiting once ST-GAT results are in, in
  case it's informative signal rather than noise to eliminate.
- **Lane changes as an active IMU scenario are future work, not current
  scope.** Mechanistically identical to turn-triggered scale/stuck faults
  (real yaw rate is real yaw rate regardless of why the vehicle is turning),
  but adding it as a scenario requires a third scenario axis without
  changing the underlying hypothesis being tested (turning invokes
  divergence). Lane-change identification itself is cheap when it's needed
  — via routing-graph adjacency (`following` vs lateral) on the planned
  route, the same mechanism `compute_turn_zones.py` already uses to exclude
  lane-change zones from turn-zone gating — and lane-change events are
  independently derivable post-hoc from the already-recorded
  `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
  topic without any new instrumentation.

## Changing this table

Any addition, removal, or reclassification here should cascade into the
collection plan (`collect.sh` campaign list) and into
`compute_turn_zones.py`'s zone computation — this table is the scope
definition, not a description of what happened to get collected.

Right now that "cascade" is manual: the campaign list lives in `collect.sh`
(bash case blocks), the fault-type -> zone-kind mapping lives in
`plot_fault_plan.py`'s `CAMPAIGN_ZONE_KIND` dict, the goal set lives in
`captured_goals.json`, and this table lives here — four places that all
need to agree, with nothing enforcing it. The 2026-07-27 runway-filtering
incident above is exactly the failure mode this invites: two files
(`turn_zones.json`/`tl_zones.json`) that different code reads for different
purposes, edited in one without checking who else depends on it.

A cleaner structure: one YAML/JSON file (e.g.
`experiments/configs/scenarios.yaml`) listing every row of this table as
data — fault type, magnitude/params, zone kind, goals — with `collect.sh`,
`compute_turn_zones.py`/`compute_tl_zones.py`, and `plot_fault_plan.py` all
reading campaign definitions from it instead of each hardcoding their own
copy. This table would then be generated FROM that file (or checked
against it) rather than hand-maintained in parallel. Worth doing, but it's
a real refactor touching bash, Python, and this doc together — flagging it
here as a concrete next step rather than folding it into today's changes
unannounced.
