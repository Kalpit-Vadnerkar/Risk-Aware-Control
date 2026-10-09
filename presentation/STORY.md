# Presentation story, slide by slide (rewritten 2026-10-09)

The bird's-eye walkthrough of the deck, for getting the whole story in one
pass before the per-slide detail (`points.txt` / `notes.txt` in each
`slide_NN_*/` folder). Slides 01-13 are the background and the Layer 1
results from the 2026-08-26 meeting (05, 04 and 06 refreshed); 14-25 are
this update. The August interpretation / next-steps slides are in
`_retired_2026-08_update/`.

**Core message:** Layer 2 now works on the real map, and the advisor's
critique shaped it: the envelope is Layer 1's own calibrated tube,
shaped a priori by the scene (2a) and a posteriori by evidence (2b). The
key finding is that size and direction of a state error need a second
measurement of the same quantity - so redundant sources went inside the
predictor. Knowing how much and which way turns warning into correction,
which sets up the closed-loop result.

**01-13 - Background (unchanged except 04-06).** Motivation, gap, the
three-layer architecture, platform, data, and the Layer 1 calibration
story (audit -> fix -> conditional calibration -> fault validation). 05
now shows the current framework; 04 and 06 carry short "update" notes.

**14 - Where we left off.** Last meeting's story, the 2026-09-03
feedback (the interval already is a reachable tube: shape it, don't
rebuild it), and the five things this update shows.

**15 - The predictor left the road.** Building 2a/2b on the drivable area
exposed 19% off-road nominal forecasts. Fixed in the model (ego-centred
map, lane direction, route bug, route ahead, selection): lateral error
0.98 -> 0.13 m, wrong-way turns 22% -> 0%.

**16 - When, not how much.** Residual-only couplings catch 0 of 4 real
departures; size and direction need redundancy. The design principle.

**17 - The architecture now.** Behaviour heads (when) + consistency heads
(how much, which way) in one ST-GAT; 2a / 2b / 2c; Layer 3. The three
contributions marked.

**18 - The envelope is a distribution.** Samples = forecast + recorded
error trajectories of similar scenes, moved by pose-error draws;
P(violation) = share outside the road. Why samples, not a Gaussian tube.

**19 - What each module adds.** One real departure, four rows: Layer 1
band follows the corrupted estimate; 2a checks it against the map; 2b
moves it onto the true path (P 0.14 -> 1.00 two seconds before the
departure); correction puts the past path on the truth.

**20 - How much and which way.** 2b's heading-error estimate vs AWSIM
ground truth on all 8 IMU trials.

**21 - Anomaly is not consequence.** Layer 1 flags every IMU fault;
Layer 2 warns on the 4 departures (2.0-3.4 s) and stays mostly quiet on
harmless faults and all 30 traffic-light trials.

**22 - One model for the pose error.** Exact error dynamics from the
consistency gaps; LiDAR matching as the one direct measurement; one
filter replaces three hand-made mechanisms; invariants every change must
pass.

**23 - From warning to correction.** Subtracting the estimate cuts the
pose error at departure from 23-95 deg to 1.0-2.6 deg (open loop).
Caveats stated.

**24 - Where this goes.** The closed-loop comparison (Autoware / +L1 /
+L2 / +L3), what it needs, the paper plan.

**25 - Questions.** Capstone, paper split, Autoware interface, scenario
priorities.

**B1 - Backup.** Operating curves and alarm rates: internal sanity
checks, shown only if asked.
