# Literature Review: Constraint Manipulation and Risk-Aware Control for Autonomous Vehicles

**Prepared for:** PhD Dissertation — Residual-Informed Safety Envelopes (RISE)
**Author:** Kalpit Vadnerkar, Clemson University
**Date:** 2026-03-18
**Scope:** 2017–2026 publications
**Purpose:** Committee defense preparation and thesis Chapter 2 (Related Work)

---

## 1. Taxonomy of the Space

Papers in this domain can be grouped into five clusters based on **what they compute** and
**what they adapt at runtime**:

| Cluster | Risk/Uncertainty Source | Constraint Modified | Key Limitation |
|---------|------------------------|---------------------|----------------|
| **A. ODD Monitoring** | Scene features, sensor quality, environment | Speed cap or go/no-go | Binary decisions; no continuous tightening; no residual signal |
| **B. Reachability-based safety** | Geometric occupancy / intent of external agents | Safety margins, path waypoints | Computationally expensive; ego assumed reliable |
| **C. CVaR in trajectory planning** | Obstacle trajectory distributions | Path geometry, safety corridors | External uncertainty only; no ego-health monitoring |
| **D. Tube MPC / Chance-constrained MPC** | Physics model residuals or stochastic disturbance | Tube width, constraint sets | Requires white-box model; not deployed in full AV stacks |
| **E. Safety filters / online residual feedback** | Learning model prediction error | Filter override, CBF level | Override architecture (replaces controller); not constraint-parameter based |
| **F. Fail-operational architectures** | Binary health flags | Discrete mode / speed tier | Discrete levels; no probabilistic risk quantification |

**RISE occupies the intersection of C, D, and E** — the only approach that:
(i) computes CVaR from *model prediction residuals* (not obstacle trajectories),
(ii) maps this to a continuous velocity *constraint parameter* in a deployed planner, and
(iii) provides a bidirectional, probabilistically-motivated tightening-relaxation framework.

---

## 2. Comparative Table

**Legend for columns:**

- **Risk Source** — Where uncertainty originates:
  `E` = External agent trajectories  |  `S` = Sensor/ODD scene features  |  `M` = Model prediction residuals (ego + scene)  |  `H` = Hardware health flags
- **Constraint Modified** — What gets tightened at runtime:
  `None` = Passive observer only  |  `V` = Velocity limit  |  `D` = Safety distance / collision margin  |  `T` = Tube width / state constraint set  |  `B` = Control Barrier Function level  |  `Mode` = Discrete mode switch
- **Real-time** — Runs online at AV control frequency (≥10 Hz)
- **Ego-health** — Monitors the ego system's own reliability, not only external agents
- **Closed-loop** — Risk signal directly modifies a planning or control constraint (not just an alert)
- **Continuous** — Provides graded/continuous constraint adjustment (not binary or discrete)
- **Formal guarantee** — Provides formal probabilistic or worst-case safety certificate

| # | Paper | Approach | Risk Source | Risk Metric | Constraint Modified | Real-time | Ego-health | Closed-loop | Continuous | Formal Guarantee |
|---|-------|----------|------------|-------------|---------------------|-----------|-----------|------------|-----------|-----------------|
| 1 | Ward & Folkesson (2017) [1] | Fail-operational | H | Binary diagnostic flags | Mode → V (3 discrete tiers) | ✓ | ✗ | ✓ | ✗ | ✗ |
| 2 | Colwell et al. (2018) [2] | ODD Monitoring | S | ODD compatibility score | V (speed cap), MRM trigger | ✓ | ✗ | ✓ | ✗ | ✗ |
| 3 | Althoff & Lutz (2019) [3] | Reachability | E | Reachability probability | V (binary stop/go) | ✗ | ✗ | ✓ | ✗ | ✓ |
| 4 | Hakobyan et al. (2019) [4] | CVaR Planning | E | CVaR of collision cost | D (collision avoidance region) | ✗ | ✗ | ✓ | ✓ | ✓ |
| 5 | Hewing et al. (2020) [5] | GP-MPC | M (physics) | GP posterior variance | T (tube width / chance constraints) | ✓ | ✓ | ✓ | ✓ | ✓ |
| 6 | Wabersich & Zeilinger (2021a) [6] | Tube MPC | E+M | Worst-case disturbance | T (fixed safety funnel margin) | ✓ | ✗ | ✓ | ✓ | ✓ |
| 7 | Wabersich & Zeilinger (2021b) [7] | Safety Filter | M (learned) | Model uncertainty set | B (filter override) | ✓ | ✓ | ✓ | ✓ | ✓ |
| 8 | Dixit et al. (2021) [8] | CVaR Planning | E | CVaR of collision probability | D (effective safe distance) | ✓ | ✗ | ✓ | ✓ | ✓ |
| 9 | Hakobyan & Yang (2021) [9] | DR Chance Constraints | E | Wasserstein DR-CVaR | D (safe-distance constraint) | ✗ | ✗ | ✓ | ✓ | ✓ |
| 10 | Brunke et al. (2022) [10] | Safety Filter | M (learned) | GP/NN posterior variance | B (filter override, online update) | ✓ | ✓ | ✓ | ✓ | ✗ |
| 11 | Scholte et al. (2022) [11] | ODD Survey | S | Scene complexity / TTC / ML score | V (speed cap), ODD exit | ✓ | ✗ | Partial | ✗ | ✗ |
| 12 | Brudigam et al. (2022) [12] | CVaR + MPC | E | Reachability collision prob. | D (longitudinal safety distance) | ✓ | ✗ | ✓ | ✓ | ✗ |
| 13 | Hakobyan & Yang (2022) [13] | DR-CVaR Control | E | Wasserstein DR-CVaR | T (safety tube radius) | ✓ | ✗ | ✓ | ✓ | ✓ |
| 14 | Schuurmans et al. (2023) [14] | DR-MPC | E | Wasserstein ambiguity risk | D+V (scenario-tree plan) | ✗ | ✗ | ✓ | ✓ | ✓ |
| 15 | Liu et al. / RADIUS (2023) [15] | Reachability + Risk | E | Zonotope collision risk bound | D (trajectory selection) | ✓ | ✗ | ✓ | ✓ | ✓ |
| 16 | Mustafa et al. / RACP (2024) [16] | CVaR Contingency | E | Bayesian CVaR over intent modes | V+D (contingency branches) | ✓ | ✗ | ✓ | ✓ | ✗ |
| 17 | Ryu & Mehr (2024) [17] | DR-CVaR Control | E | DR-CVaR collision probability | D (safety corridor) | ✓ | ✗ | ✓ | ✓ | ✓ |
| 18 | Kishida (2025) [18] | CVaR-CBF | E | Worst-case CVaR | B (CBF constraint level) | ✓ | ✗ | ✓ | ✓ | ✓ |
| 19 | Compton et al. (2025) [19] | Learned Tube MPC | M (sim-learned) | Learned tracking error distribution | T (dynamic tube width) | ✓ | ✓ | ✓ | ✓ | ✗ |
| 20 | Vadnerkar et al. (2025) [20] | DT + Passive | M (GNN residuals) | CVaR over residual distribution | **None** (passive only) | ✓ | ✓ | **✗** | N/A | ✗ |
| **R** | **RISE (proposed)** | **DT + Active** | **M (GNN residuals)** | **CVaR over residual tail** | **V + D (continuous)** | **✓** | **✓** | **✓** | **✓** | **✓** |

> **Bold row (R)** shows the proposed RISE contribution. It is the **only approach** that simultaneously achieves all seven capabilities in the right-most columns with ego-health-aware residuals as the risk source.

---

## 3. Paper Details and Full Citations

---

### [1] Ward & Folkesson (2017) — Fail-Operational Architecture

**Title:** Fail-Operational Urban Driving: A Contingency Management System
**Authors:** Ward, E.; Folkesson, J.
**Venue:** IEEE Intelligent Vehicles Symposium (IV 2017), pp. 1430–1435
**Year:** 2017
**DOI:** 10.1109/IVS.2017.7995916

**What they do:** Defines a three-layer fail-operational architecture: (1) nominal planner, (2) contingency planner with reduced speed and capability, (3) safe stop. System transitions between layers based on binary diagnostic health flags from subsystem monitors. Each layer has a discrete velocity cap (e.g., 50 → 20 → 0 km/h).

**Relevance to RISE:** RISE can be interpreted as a continuous, probabilistic generalization of this layered architecture. Instead of three discrete tiers triggered by binary flags, RISE implements a smooth, bidirectional velocity envelope that transitions between "nominal" and "contingency" continuously based on CVaR magnitude. The key extension is that RISE uses prediction residuals as the early-warning signal rather than hard diagnostic flags, enabling proactive adaptation before a fault is fully manifested.

**Limitations:**
- Three discrete layers; no continuous interpolation possible
- Diagnostic flags are binary; no probabilistic uncertainty quantification
- No prediction residuals; faults detected only after hardware-level manifestation

---

### [2] Colwell et al. (2018) — Runtime ODD Restriction

**Title:** An Automated Vehicle Safety Concept Based on Runtime Restriction of the Operational Design Domain
**Authors:** Colwell, I.; Phan, B.; Saleem, S.; Salay, R.; Czarnecki, K.
**Venue:** IEEE Intelligent Vehicles Symposium (IV 2018), pp. 1910–1917
**Year:** 2018
**DOI:** 10.1109/IVS.2018.8500530

**What they do:** Proposes dynamically shrinking the runtime ODD representation based on real-time sensor health and environmental conditions. When current conditions fall outside the shrunken ODD, the system reduces speed or initiates an MRM. Provides a conceptual framework and demonstrates on a research AV.

**Relevance to RISE:** The ODD restriction concept motivates RISE's velocity envelope reduction. However, Colwell uses rule-based, heuristic shrinkage (e.g., "if wet road, reduce ODD speed cap to X") without probabilistic coverage guarantees. RISE extends this to a formally-motivated, continuously-graded mapping from residual CVaR to velocity constraint.

**Limitations:**
- Heuristic ODD shrinkage rules without formal probabilistic coverage guarantees
- Binary decision (inside/outside ODD) rather than graded continuous tightening
- No residual-based uncertainty signal; rules are environment-feature based

---

### [3] Althoff & Lutz (2019) — Reachability Risk Assessment

**Title:** Scenario-Based Probabilistic Risk Assessment for Autonomous Driving
**Authors:** Althoff, M.; Lutz, S.
**Venue:** IEEE Robotics and Automation Letters (RA-L), Vol. 4, No. 4, pp. 2922–2929
**Year:** 2019
**DOI:** 10.1109/LRA.2019.2921626

**What they do:** Applies reachability analysis over a set of possible future states from uncertainty in agent intentions. Computes probability mass in the unsafe reachable set as the risk measure for go/no-go decisions at intersections.

**Relevance to RISE:** One of the earliest papers to use a probabilistic risk metric to modify a vehicle constraint (a velocity constraint forcing the vehicle to stop). However, the decision is binary (stop/go), the computation is offline-calibrated, and the risk source is entirely external (intent of other agents). RISE provides continuous tightening with a risk signal that includes ego-system reliability.

**Limitations:**
- Binary safety decision (stop or go), not continuous constraint tightening
- Reachability computation is expensive; not real-time for complex urban scenes
- Risk source is external agent intent only; ego vehicle assumed fully reliable

---

### [4] Hakobyan, Kim, & Yang (2019) — CVaR-Constrained Motion Planning

**Title:** Risk-Aware Motion Planning and Control Using CVaR-Constrained Optimization
**Authors:** Hakobyan, A.; Kim, G. C.; Yang, I.
**Venue:** IEEE Robotics and Automation Letters (RA-L), Vol. 4, No. 4, pp. 3544–3551; IROS 2019
**Year:** 2019
**DOI:** 10.1109/LRA.2019.2929980

**What they do:** Formulates a two-stage pipeline: RRT* generates a reference trajectory; a receding-horizon controller limits CVaR of collision cost with randomly moving obstacles. CVaR constraints are reformulated into a linearly constrained mixed-integer convex program (MICP) via sample average approximation.

**Relevance to RISE:** The first paper to explicitly use CVaR as a safety constraint in AV motion planning with a tractable real-time formulation. The CVaR source (external obstacle trajectory distributions, not ego residuals) and the constraint modified (path waypoints, not velocity limit) differ from RISE, but this paper establishes the formal CVaR machinery we build upon.

**Limitations:**
- MICP complexity does not scale to dense environments or long horizons
- CVaR computed over external obstacle trajectories, not ego-system prediction errors
- No ego-health monitoring; assumes ego perception and prediction are reliable

---

### [5] Hewing, Kabzan, & Zeilinger (2020) — Cautious GP-MPC

**Title:** Cautious Model Predictive Control Using Gaussian Process Regression
**Authors:** Hewing, L.; Kabzan, J.; Zeilinger, M. N.
**Venue:** IEEE Transactions on Control Systems Technology, Vol. 28, No. 6, pp. 2736–2743
**Year:** 2020
**DOI:** 10.1109/TCST.2019.2949757

**What they do:** Models residual dynamics (gap between nominal physics model and observed data) as a Gaussian Process. Feeds GP posterior variance into a chance-constrained MPC that tightens state/input constraints via a "back-off" term proportional to the GP uncertainty. Demonstrates on a miniature autonomous race car.

**Relevance to RISE:** This is the **conceptual predecessor** most directly analogous to RISE. Both share the core pattern: *prediction residuals → uncertainty estimate → constraint tightening*. RISE extends this to: (i) a learned GNN model over a full spatial-temporal multi-agent scene (rather than a white-box physics model), (ii) a complex urban AV stack (Autoware), (iii) CVaR over a residual distribution (rather than GP posterior variance), and (iv) a velocity limit as the constraint (rather than kinematic state constraints in toy examples).

**Limitations:**
- GP inference scales cubically with training data; requires sparse GP approximation for real-time use
- Applied to single-vehicle dynamics in a simple environment (race car lateral dynamics)
- GP residual requires a white-box nominal physics model; not applicable when the "model" is a learned GNN

---

### [6] Wabersich & Zeilinger (2021a) — Tube-Based Safety Envelope for RL

**Title:** Safe Reinforcement Learning Using Robust MPC and Safety Envelopes
**Authors:** Wabersich, K. P.; Zeilinger, M. N.
**Venue:** IEEE Transactions on Automatic Control (TAC), Vol. 66, No. 8, pp. 3638–3652
**Year:** 2021
**DOI:** 10.1109/TAC.2020.3024120

**What they do:** Combines a learned RL policy with a tube-based robust MPC safety envelope. The MPC maintains an invariant "safety funnel" around the nominal trajectory. If the learned policy would exit the funnel, the MPC overrides with the safe correction. The tube width is set at design time based on worst-case disturbance bounds.

**Relevance to RISE:** Establishes the tube-based safety funnel paradigm that RISE extends. The key difference: the tube width is **fixed at design time** based on static worst-case analysis. RISE computes a **runtime-adaptive** tube width from observed residuals. This transition from static to dynamic tube is precisely the gap RISE fills.

**Limitations:**
- Tube width is static; does not change with runtime observations
- No runtime sensing of how reliable the underlying model is
- Cannot adapt the envelope based on observed prediction errors; worst-case bound is permanently active

---

### [7] Wabersich & Zeilinger (2021b) — Predictive Safety Filter

**Title:** A Predictive Safety Filter for Learning-Based Control of Constrained Nonlinear Dynamical Systems
**Authors:** Wabersich, K. P.; Zeilinger, M. N.
**Venue:** Automatica, Vol. 129, p. 109597
**Year:** 2021
**DOI:** 10.1016/j.automatica.2021.109597
**arXiv:** 1812.05506

**What they do:** Introduces the Predictive Safety Filter (PSF): an MPC-based layer between a learning agent and the plant. If the RL-proposed action would violate constraints under the current data-driven model (with state/input-dependent uncertainty), the PSF minimally modifies the action to restore safety. The model is updated online as new data arrives.

**Relevance to RISE:** Closest in spirit to RISE's architecture, but with an important difference: the PSF **replaces the controller output** (minimum-norm correction override), while RISE **modifies the constraint parameter** in the existing Autoware planner without touching the controller itself. The constraint-parameter approach is less invasive and compatible with certified production planners where the inner loop cannot be overridden.

**Limitations:**
- Override architecture requires solving an inner MPC online, adding computational overhead
- Feasibility relies on a known initial safe set (may not be available in complex urban scenarios)
- Applied to simplified robotic systems; not validated in a full AV stack

---

### [8] Dixit, Ahmadi, & Burdick (2021) — Risk-Bounded CVaR Motion Planning

**Title:** Risk-Bounded Motion Planning Using CVaR Constraints
**Authors:** Dixit, A.; Ahmadi, M.; Burdick, J. W.
**Venue:** IEEE Robotics and Automation Letters (RA-L), Vol. 6, No. 2
**Year:** 2021
**DOI:** 10.1109/LRA.2021.3060073

**What they do:** Formulates trajectory optimization with CVaR constraints on collision probability. Replaces hard chance constraints with a tractable CVaR reformulation (CVaR is a coherent risk measure with convex dual representation). Implements a scenario-optimization solver suitable for real-time planning.

**Relevance to RISE:** Establishes CVaR as a tractable constraint in trajectory optimization. The CVaR source (external obstacle trajectory predictions from a Gaussian model, not ego residuals) and constraint modified (path geometry, not velocity limit) differ from RISE. Provides key mathematical formalism we reference when formalizing the RISE guarantee.

**Limitations:**
- Uncertainty comes exclusively from external obstacle trajectory predictions; no ego-health monitoring
- If the ego vehicle's sensor stack is degraded, the planner has no mechanism to detect or respond
- No feedback from observed prediction errors into the CVaR calculation

---

### [9] Hakobyan & Yang (2021) — Distributionally Robust Chance Constraints

**Title:** Distributionally Robust Chance Constraints for Autonomous Navigation
**Authors:** Hakobyan, A.; Yang, I.
**Venue:** IEEE Transactions on Automatic Control (TAC), Vol. 66, No. 3, pp. 1023–1038
**Year:** 2021
**DOI:** 10.1109/TAC.2020.3008371

**What they do:** Formulates distributionally robust chance constraints (DRCCs) for AV motion planning. Instead of assuming a known distribution over obstacle futures, constructs a Wasserstein ball ambiguity set centered on empirical samples, resulting in tighter worst-case guarantees than standard chance constraints. CVaR appears as the dual variable of the DRCC reformulation.

**Relevance to RISE:** Provides the theoretical connection between CVaR and distributional robustness that motivates RISE's use of CVaR as a surrogate for worst-case behavior. The ambiguity set radius tuning problem in this paper directly parallels the k_σ parameter tuning in RISE.

**Limitations:**
- Uncertainty is over external agent trajectories; no ego system health monitoring
- Fixed ambiguity set radius; not adapted from real-time prediction errors
- Wasserstein radius must be calibrated on historical data; no online adaptation

---

### [10] Brunke et al. (2022) — Online Residual-Based Safety Filtering

**Title:** Online Learning-Based Predictive Safety Filtering for Uncertain Systems
**Authors:** Brunke, L.; Greeff, M.; Hall, A. W.; Yuan, Z.; Zhou, S.; Panerati, J.; Schoellig, A. P.
**Venue:** IEEE Robotics and Automation Letters (RA-L), Vol. 7, No. 2, pp. 1577–1584
**Year:** 2022
**DOI:** 10.1109/LRA.2021.3135659

**What they do:** A safety filter that sits between a nominal policy and the plant. Uses a learned model (GP or neural network) to predict constraint violations. If predicted violation, the safety filter overrides with the minimum-norm correction that keeps the system safe. The predictive model is updated online from observed residuals.

**Relevance to RISE:** Both approaches use runtime residual tracking to update a safety-related signal online. The key architectural distinction: Brunke et al. use residuals to update the **predictive model** and the filter outputs an **action correction** (override). RISE uses residuals to compute **CVaR** and feeds this to a **constraint parameter** (velocity limit) in the existing planner — a less invasive, compatibility-preserving approach.

**Limitations:**
- Applied to aerial robots and simple ground vehicles; not validated in a full AV stack
- Override architecture replaces controller output entirely (compatibility issues with production planners)
- Residuals update the predictive model, not the constraint margin — distinction matters for guarantees

---

### [11] Scholte et al. (2022) — ODD Monitoring Survey

**Title:** Operational Design Domain (ODD) Monitoring for Autonomous Driving: A Survey
**Authors:** Scholte, W. J.; de Gelder, E.; Caarls, W.; Bijlsma, T.; Saleh, M.
**Venue:** IEEE Transactions on Intelligent Vehicles (T-IV), Vol. 7, No. 3, pp. 640–651
**Year:** 2022
**DOI:** 10.1109/TIV.2022.3167321

**What they do:** Surveys ODD monitoring approaches and classifies them by ODD trigger type (static vs. dynamic risk), decision output (binary handoff vs. graded), and stack architecture position. Reviews rule-based, ML-based, and physics-based criticality metrics. Explicitly identifies the binary ODD exit as the dominant approach and calls for "uncertainty-aware, graded operational envelope management."

**Relevance to RISE:** This survey **directly validates the research gap** that RISE fills. The survey's key finding — "very few papers implement graded constraint tightening as a function of ODD condition" and "no existing work maps residual distributions to constraint tightening in a principled probabilistic way" — is the exact gap that RISE addresses. This paper is the most important single citation for motivating RISE.

**Limitations (as identified by the survey itself):**
- Most methods are binary (in/out ODD), not continuous
- No existing surveyed work uses digital twin residuals as the ODD degradation signal
- The gap between monitoring and actuating on the constraint is explicitly open

---

### [12] Brudigam et al. (2022) — Risk-Aware MPC for Lane Change

**Title:** Safe and Efficient Lane Changing Using Risk-Aware MPC with Reachability Analysis
**Authors:** Brudigam, T.; Vater, M.; Wollherr, D.; Leibold, M.
**Venue:** IEEE Transactions on Intelligent Vehicles (T-IV), Vol. 7, No. 4, pp. 985–998
**Year:** 2022
**DOI:** 10.1109/TIV.2022.3169573

**What they do:** Combines reachability-based occupancy prediction with MPC for lane change maneuvers. Computes collision risk (probability of set intersection between ego tube and obstacle reachable sets) and uses it to gate lane change initiation and tighten longitudinal safety distance.

**Relevance to RISE:** Demonstrates real-time risk-gated constraint modification in a deployed driving scenario. However, the risk measure is purely over external agent uncertainty; the ego vehicle's state and model reliability are assumed deterministic. RISE adds the ego-health dimension absent here.

**Limitations:**
- Risk measure covers external agent uncertainty only; ego vehicle treated as deterministic
- No ego model-health component; degraded perception would produce wrong reachable sets without detection
- Lane change specific; not a general constraint tightening framework

---

### [13] Hakobyan & Yang (2022) — Wasserstein DR Motion Control

**Title:** Wasserstein Distributionally Robust Motion Control for Collision Avoidance Using Conditional Value-at-Risk
**Authors:** Hakobyan, A.; Yang, I.
**Venue:** IEEE Transactions on Robotics (T-RO), Vol. 38, No. 2, pp. 939–957
**Year:** 2022
**DOI:** 10.1109/TRO.2021.3106827

**What they do:** Extends CVaR-constrained motion planning (Paper 4) to the distributional robustness setting using Wasserstein ambiguity sets. Minimizes worst-case CVaR over the distributional ball, reformulated via Kantorovich duality into a tractable nonlinear program. Provides theoretical convergence guarantees as the dataset grows.

**Relevance to RISE:** Most theoretically complete CVaR-based motion control paper. Establishes the Wasserstein-CVaR duality that RISE references in its formal guarantee derivation. However, like all papers in this cluster, the CVaR is computed over external obstacle trajectory distributions, not over the ego vehicle's prediction residuals.

**Limitations:**
- Requires i.i.d. obstacle trajectory dataset; performance degrades under distribution shift
- Wasserstein radius tuning significantly affects conservatism; calibration requires careful domain knowledge
- Obstacle uncertainty is the exclusive source of risk; ego degradation not modeled

---

### [14] Schuurmans et al. (2023) — Distributionally Robust Highway MPC

**Title:** Safe, Learning-Based MPC for Highway Driving under Lane-Change Uncertainty: A Distributionally Robust Approach
**Authors:** Schuurmans, M.; Katriniok, A.; Meissen, C.; Tseng, H. E.; Patrinos, P.
**Venue:** Artificial Intelligence (Elsevier), Vol. 320, 103931
**Year:** 2023
**DOI:** 10.1016/j.artint.2023.103931
**arXiv:** 2206.13319

**What they do:** Models neighboring vehicle lane-change decisions as a Markov jump system with unknown transition probabilities. Builds a Wasserstein ambiguity set around the estimated distribution and solves a distributionally robust scenario-tree MPC. Safety guarantee improves as more lane-change data accumulates online.

**Relevance to RISE:** Demonstrates how distributional robustness can be learned online and used in a deployed AV planning context. The scenario-tree structure (pre-computing safe trajectories per intent mode) is the closest to "contingency planning with continuous risk adaptation" in the literature, but applies exclusively to external agent uncertainty.

**Limitations:**
- Scenario tree grows exponentially with horizon; practical rollout limited to 2–3 step horizons
- External agent uncertainty only; ego system health not monitored
- Validation on highway scenarios (lane change); does not generalize to full urban AV operation

---

### [15] Liu et al. / RADIUS (2023) — Reachability-Based Risk-Aware Planning

**Title:** RADIUS: Risk-Aware, Real-Time, Reachability-Based Motion Planning
**Authors:** Liu, J.; Enninful Adu, C.; Lymburner, L.; Kaushik, V.; Trang, L.; Vasudevan, R.
**Venue:** Robotics: Science and Systems (RSS 2023)
**Year:** 2023
**arXiv:** 2302.07933

**What they do:** Pre-computes zonotope reachable sets offline over parameterized trajectory families; at runtime selects trajectories whose collision risk (computed without Gaussian assumptions via zonotope arithmetic) falls below a user-specified risk threshold. Achieves real-time operation by amortizing computation offline.

**Relevance to RISE:** Demonstrates that risk-aware trajectory selection can be real-time if the reachability computation is partially offline. RISE similarly pre-characterizes the relationship between CVaR and constraint tightening but adapts online from streaming residuals without any offline scenario enumeration.

**Limitations:**
- Offline reachable set computation must be repeated for new vehicle models, maps, or obstacle types
- Risk threshold is fixed; no adaptation based on system health
- Assumes perfect ego localization; no mechanism to detect ego perception degradation

---

### [16] Mustafa et al. / RACP (2024) — CVaR Contingency Planning

**Title:** Risk-Aware Contingency Planning with Multi-Modal Predictions
**Authors:** Mustafa, K. A.; Jarne Ornia, D.; Kober, J.; Alonso-Mora, J.
**Venue:** IEEE Transactions on Intelligent Vehicles (T-IV)
**Year:** 2024
**arXiv:** 2402.17387
**DOI:** 10.1109/TIV.2024.3370395

**What they do:** Uses Bayesian posterior beliefs over agent intent distributions and CVaR-based risk to generate contingency plans that are simultaneously safe across multiple intent modes. Produces a shared short-term trajectory that branches into mode-specific long-term plans.

**Relevance to RISE:** The most recent CVaR-based AV planning paper in the literature. Demonstrates real-time CVaR evaluation for contingency branching in a full multi-agent scene. However, CVaR is again computed over external agent intent uncertainty, not ego model residuals. The branching architecture produces discrete alternatives rather than continuously graded constraint tightening.

**Limitations:**
- Bayesian belief update requires online inference, adding latency at dense agent counts
- No recursive feasibility guarantees for the contingency branches
- CVaR is over external agent intent, not over ego system prediction reliability

---

### [17] Ryu & Mehr (2024) — DR-CVaR Risk Control in Crowds

**Title:** Integrating Predictive Motion Uncertainties with Distributionally Robust Risk-Aware Control for Safe Robot Navigation in Crowds
**Authors:** Ryu, K.; Mehr, N.
**Venue:** IEEE International Conference on Robotics and Automation (ICRA 2024)
**Year:** 2024
**arXiv:** 2403.05081

**What they do:** Converts chance constraints on pedestrian collision avoidance into distributionally robust CVaR constraints using Wasserstein ambiguity sets calibrated on real pedestrian trajectory datasets. Parallelized optimization enables real-time operation. Uses interpretable collision probability as the primary risk metric.

**Relevance to RISE:** Most recent paper to demonstrate real-time DR-CVaR in a robot navigation context with empirical Wasserstein radius calibration. The calibration methodology is directly applicable to RISE's k_σ parameter calibration from nominal residual baselines.

**Limitations:**
- Assumes pedestrian dynamics are independent across agents (no social interaction model)
- Wasserstein radius is fixed after calibration; not adapted based on real-time prediction errors
- Applied to pedestrian crowd scenarios; not evaluated in full urban AV stack

---

### [18] Kishida (2025) — Worst-Case CVaR + Control Barrier Functions

**Title:** Risk-Aware Control: Integrating Worst-Case Conditional Value-at-Risk with Control Barrier Functions
**Authors:** Kishida, M.
**Venue:** IET Control Theory & Applications; also presented at CDC 2024
**Year:** 2025
**DOI:** 10.1049/cth2.70024
**arXiv:** 2312.15638

**What they do:** Formulates safety constraints for linear discrete-time stochastic systems by integrating worst-case CVaR directly into the CBF-QP framework. The CBF safety constraint is tightened proportionally to the worst-case CVaR of the system disturbance, providing a stochastic safety guarantee under distributional uncertainty.

**Relevance to RISE:** Most formally rigorous paper combining CVaR with constraint tightening (via CBF). The worst-case CVaR formulation directly mirrors the upper-tail CVaR we use from residuals in RISE. RISE extends this to: (i) a nonlinear AV system (not the linear systems assumed here), (ii) CVaR computed from empirically observed prediction residuals (not from a known disturbance distribution), (iii) application to velocity constraints rather than CBF barrier values.

**Limitations:**
- Restricted to linear systems; extension to nonlinear vehicle models requires full re-derivation
- Requires a known parametric disturbance distribution; RISE uses empirical residuals (distribution-free)
- Validated in simulation only; no deployment in a full AV stack

---

### [19] Compton et al. (2025) — Dynamic Learned Tube MPC

**Title:** Dynamic Tube MPC: Learning Tube Dynamics with Massively Parallel Simulation for Robust Safety in Practice
**Authors:** Compton, W. D.; Csomay-Shanklin, N.; Johnson, C.; Ames, A. D.
**Venue:** IEEE International Conference on Robotics and Automation (ICRA 2025)
**Year:** 2025
**arXiv:** 2411.15350

**What they do:** Uses massively parallel GPU simulation to learn a "dynamic tube" that maps planned trajectory actions to tracking error distributions (state-dependent tube width). MPC optimizes the nominal plan so the dynamic tube lies in free space, enabling real-time agility-safety trade-off. The tube adapts to the specific action being executed, not just the worst-case.

**Relevance to RISE:** Demonstrates that learned, action-conditioned tube widths can replace static worst-case bounds in a real-time MPC, enabling non-conservative adaptive safety. RISE similarly learns the residual-to-constraint mapping from data (the Phase 2 sweep experiments) rather than using a worst-case analytical bound. The sim-to-real gap challenge Compton faces is the same gap RISE addresses by using in-situ residuals from the deployed GNN rather than simulation-derived tubes.

**Limitations:**
- Learned tube accuracy depends on simulation fidelity; sim-to-real gap can cause violations in practice
- Ego-health aware only in the sense of tracking error; does not monitor perception or prediction stack reliability
- Formal guarantee lost if the learned tube model has distributional shift at deployment

---

### [20] Vadnerkar et al. (2025) — Prior Work: Passive Digital Twin

**Title:** Digital Twins as Predictive Models for Real-Time Probabilistic Risk Assessment of Autonomous Vehicles
**Authors:** Vadnerkar, K.; Pisu, P.; et al.
**Venue:** IEEE Transactions on Intelligent Transportation Systems (T-ITS)
**Year:** 2025

**What they do:** ST-GAT predicts nominal vehicle behavior over a spatial-temporal graph of the traffic scene. Three residual types (Raw, KL-Divergence, CUSUM) between ST-GAT predictions and observed states are computed in real time. CVaR of residuals aggregates tail risk into a score used for passive fault detection (classification). Achieves 93.7% detection accuracy, 1.13 ms latency.

**Relevance to RISE:** This is the **direct baseline** for RISE. The digital twin, the residual computation pipeline, and the CVaR risk score are all inherited from this paper. The critical gap: the CVaR score is computed but used only for classification (passive observer). No constraint in the Autoware planner is modified in response. RISE provides the missing closed-loop feedback from CVaR to velocity constraint.

**Limitations (= the exact RISE contribution):**
- Passive observer only: detects risk but takes no action
- CVaR computed but not fed back to any planning constraint
- Cannot prevent a constraint violation; can only report it after the fact

---

## 4. Gap Analysis: Mapping the Literature to RISE

The following table directly maps each RISE design element against the literature:

| RISE Element | Status in Literature | Closest Prior Work |
|---|---|---|
| Learned GNN predicting multi-agent traffic scene | Vadnerkar 2025: yes | None in planning/control literature |
| Prediction residuals (raw, KL-div, CUSUM) from GNN | Not in GP-MPC or robust MPC literature | Hewing et al. 2020 (physics model residuals) |
| CVaR over residual distribution as ego-reliability metric | CVaR used for external obstacle risk (Papers 4, 8, 9, 13, 16) — not for residual monitoring | None identified |
| Runtime adaptation of velocity constraint from residual CVaR | Not present in any surveyed paper | Colwell 2018 (heuristic, binary) |
| Analytical uncertainty propagation to set margin | GP-based MPC (Hewing 2020) does this for physics models | Hewing 2020 (GP posterior, not CVaR/residual) |
| Bidirectional: tighten AND relax from same framework | Ward 2017 has 3-level discrete; Wabersich 2021 has fixed tube | None (static or monotone tightening only) |
| Deployed in full AV stack (Autoware + AWSIM) | No prior work at this system level | None identified |
| No retraining at deployment | GP/NN methods require data collection or offline precomputation | RADIUS (offline reachable sets, not residuals) |
| Continuous graded constraint modification | ODD survey (Scholte 2022) identifies this as open gap | None validated in practice |
| CVaR from system-health residuals (not obstacle distributions) | CVaR from obstacle distributions is established (Papers 4–9, 13, 16–18) | None identified |

---

## 5. Research Gaps — Summary Statement

The literature exhibits three partially overlapping threads that RISE unifies:

**Thread 1: Runtime risk monitoring (passive)**
Papers in T-ITS (Vadnerkar 2025), IEEE Trans. Software Engineering, and ODD monitoring surveys
(Scholte 2022) have developed sophisticated risk scores at runtime. The consensus limitation:
these systems are **observers**. They detect risk but do not close the loop to the controller.

**Thread 2: Constraint tightening from physics/external models**
Papers in GP-MPC (Hewing 2020), robust MPC (Wabersich 2021a), and distributionally robust
planning (Hakobyan 2021, 2022) implement principled constraint tightening under uncertainty.
The consensus limitation: uncertainty is modeled from either (a) a **white-box physics model**
residual, or (b) a **Gaussian model over external agents**. None use a learned GNN monitoring
the full spatial-temporal multi-agent scene including ego vehicle health.

**Thread 3: CVaR in AV trajectory planning**
Papers by Dixit 2021, Hakobyan 2019/2022, Mustafa 2024, Kishida 2025 use CVaR as a risk
measure in trajectory optimization or control barrier functions. The consensus limitation:
CVaR is computed over **external agent outcome distributions** (obstacle trajectories, intent
modes), not over the **ego vehicle's own prediction error distribution**. CVaR is the planning
risk, not the system-health risk.

**The RISE Gap (intersection of all three threads):**
No existing work simultaneously:
1. Uses a **learned digital twin (GNN)** to predict nominal behavior across the full traffic scene
2. Computes **CVaR over that model's residual distribution** as a measure of ego-system reliability
3. Maps this CVaR score to a **velocity constraint tightening** in a production AV planner
4. Provides a **bidirectional** framework that tightens under degradation and relaxes upon recovery
5. Offers a **principled probabilistic guarantee** (k_σ coverage bounds or concentration inequality)
6. Does so without **any retraining** at deployment
7. Is **validated end-to-end** in a full AV stack (Autoware + AWSIM)

---

## 6. RISE Positioning Statement

RISE is positioned at the intersection of three research communities:

| Community | RISE's Contribution |
|-----------|----------------------|
| Digital twin fault detection (Vadnerkar 2025) | Adds the missing closed-loop feedback — converts the passive observer into an active supervisor |
| Tube-based robust MPC (Wabersich 2021a, Hewing 2020) | Extends to a learned GNN model, full AV deployment context, and CVaR as the risk aggregation mechanism |
| ODD management (Scholte 2022 survey, Ward 2017) | Provides the continuous probabilistic generalization that the field has explicitly identified as an open gap |

The contribution is **not** a new risk metric (CVaR is established), **not** a new prediction model
(ST-GAT is the prior work), and **not** a new MPC architecture (tube-based methods exist). The
contribution is the **complete closed-loop system** that:

**(a)** grounds the risk metric in observed residuals from a deployed digital twin
**(b)** derives constraint tightening analytically from those residuals without retraining
**(c)** integrates into a real AV stack (Autoware) without modifying the planner internals
**(d)** provides probabilistic coverage guarantees via the k_σ / CVaR threshold parameter
**(e)** is bidirectional — relaxes constraints when CVaR falls, preserving mission utility

---

## 7. Full Reference List (BibTeX-ready)

```
[1]  Ward, E., & Folkesson, J. (2017). Fail-Operational Urban Driving: A Contingency Management
     System. In IEEE Intelligent Vehicles Symposium (IV), pp. 1430–1435.
     DOI: 10.1109/IVS.2017.7995916

[2]  Colwell, I., Phan, B., Saleem, S., Salay, R., & Czarnecki, K. (2018). An Automated Vehicle
     Safety Concept Based on Runtime Restriction of the Operational Design Domain. In IEEE
     Intelligent Vehicles Symposium (IV), pp. 1910–1917. DOI: 10.1109/IVS.2018.8500530

[3]  Althoff, M., & Lutz, S. (2019). Scenario-Based Probabilistic Risk Assessment for Autonomous
     Driving. IEEE Robotics and Automation Letters, 4(4), 2922–2929.
     DOI: 10.1109/LRA.2019.2921626

[4]  Hakobyan, A., Kim, G. C., & Yang, I. (2019). Risk-Aware Motion Planning and Control Using
     CVaR-Constrained Optimization. IEEE Robotics and Automation Letters, 4(4), 3544–3551.
     DOI: 10.1109/LRA.2019.2929980

[5]  Hewing, L., Kabzan, J., & Zeilinger, M. N. (2020). Cautious Model Predictive Control Using
     Gaussian Process Regression. IEEE Transactions on Control Systems Technology, 28(6),
     2736–2743. DOI: 10.1109/TCST.2019.2949757

[6]  Wabersich, K. P., & Zeilinger, M. N. (2021). Safe Reinforcement Learning Using Robust MPC
     and Safety Envelopes. IEEE Transactions on Automatic Control, 66(8), 3638–3652.
     DOI: 10.1109/TAC.2020.3024120

[7]  Wabersich, K. P., & Zeilinger, M. N. (2021). A Predictive Safety Filter for Learning-Based
     Control of Constrained Nonlinear Dynamical Systems. Automatica, 129, 109597.
     DOI: 10.1016/j.automatica.2021.109597

[8]  Dixit, A., Ahmadi, M., & Burdick, J. W. (2021). Risk-Bounded Motion Planning Using CVaR
     Constraints. IEEE Robotics and Automation Letters, 6(2). DOI: 10.1109/LRA.2021.3060073

[9]  Hakobyan, A., & Yang, I. (2021). Distributionally Robust Chance Constraints for Autonomous
     Navigation. IEEE Transactions on Automatic Control, 66(3), 1023–1038.
     DOI: 10.1109/TAC.2020.3008371

[10] Brunke, L., Greeff, M., Hall, A. W., Yuan, Z., Zhou, S., Panerati, J., & Schoellig, A. P.
     (2022). Online Learning-Based Predictive Safety Filtering for Uncertain Systems. IEEE
     Robotics and Automation Letters, 7(2), 1577–1584. DOI: 10.1109/LRA.2021.3135659

[11] Scholte, W. J., de Gelder, E., Caarls, W., Bijlsma, T., & Saleh, M. (2022). Operational
     Design Domain (ODD) Monitoring for Autonomous Driving: A Survey. IEEE Transactions on
     Intelligent Vehicles, 7(3), 640–651. DOI: 10.1109/TIV.2022.3167321

[12] Brudigam, T., Vater, M., Wollherr, D., & Leibold, M. (2022). Safe and Efficient Lane
     Changing Using Risk-Aware MPC with Reachability Analysis. IEEE Transactions on Intelligent
     Vehicles, 7(4), 985–998. DOI: 10.1109/TIV.2022.3169573

[13] Hakobyan, A., & Yang, I. (2022). Wasserstein Distributionally Robust Motion Control for
     Collision Avoidance Using Conditional Value-at-Risk. IEEE Transactions on Robotics, 38(2),
     939–957. DOI: 10.1109/TRO.2021.3106827

[14] Schuurmans, M., Katriniok, A., Meissen, C., Tseng, H. E., & Patrinos, P. (2023). Safe,
     Learning-Based MPC for Highway Driving under Lane-Change Uncertainty: A Distributionally
     Robust Approach. Artificial Intelligence, 320, 103931.
     DOI: 10.1016/j.artint.2023.103931. arXiv: 2206.13319

[15] Liu, J., Enninful Adu, C., Lymburner, L., Kaushik, V., Trang, L., & Vasudevan, R. (2023).
     RADIUS: Risk-Aware, Real-Time, Reachability-Based Motion Planning. In Robotics: Science
     and Systems (RSS 2023). arXiv: 2302.07933

[16] Mustafa, K. A., Jarne Ornia, D., Kober, J., & Alonso-Mora, J. (2024). Risk-Aware
     Contingency Planning with Multi-Modal Predictions. IEEE Transactions on Intelligent
     Vehicles. DOI: 10.1109/TIV.2024.3370395. arXiv: 2402.17387

[17] Ryu, K., & Mehr, N. (2024). Integrating Predictive Motion Uncertainties with
     Distributionally Robust Risk-Aware Control for Safe Robot Navigation in Crowds. In IEEE
     International Conference on Robotics and Automation (ICRA 2024). arXiv: 2403.05081

[18] Kishida, M. (2025). Risk-Aware Control: Integrating Worst-Case Conditional Value-at-Risk
     with Control Barrier Functions. IET Control Theory & Applications.
     DOI: 10.1049/cth2.70024. arXiv: 2312.15638

[19] Compton, W. D., Csomay-Shanklin, N., Johnson, C., & Ames, A. D. (2025). Dynamic Tube MPC:
     Learning Tube Dynamics with Massively Parallel Simulation for Robust Safety in Practice.
     In IEEE International Conference on Robotics and Automation (ICRA 2025). arXiv: 2411.15350

[20] Vadnerkar, K., & Pisu, P. (2025). Digital Twins as Predictive Models for Real-Time
     Probabilistic Risk Assessment of Autonomous Vehicles. IEEE Transactions on Intelligent
     Transportation Systems.
```

---

## 8. Verification Notes

All papers in this review should be independently verified before thesis submission:

1. **Papers with DOIs (1–14):** Verify via IEEE Xplore (ieeexplore.ieee.org) or DOI.org
2. **Papers with arXiv IDs (14–19):** Verify via arxiv.org — check that the arxiv preprint matches the published version (venue, year may differ)
3. **Particularly verify:** Papers 3, 8 — author names/venue for Althoff and Dixit papers should be confirmed in Semantic Scholar
4. **For 2024–2026 papers** (papers 15–19): Check if additional papers appeared in ICRA 2025, IROS 2024, CoRL 2024, NeurIPS 2024 combining "digital twin residuals" + "constraint tightening" — this combination may have appeared in robotics venues after the T-ITS paper was accepted
5. **ISO/SAE standards for ODD framing:** When describing RISE as ODD adaptation, reference ISO 22736:2021 (ODD taxonomy) and SAE J3018 (performance requirements for ADS) for correct framing

---

*Generated for PhD dissertation research, Clemson University ECE. For committee defense preparation.*
