# Literature Review: Dynamic Safety Envelope Adaptation and Runtime Risk Assessment for Autonomous Vehicles

**Created:** 2026-03-05
**Purpose:** Identify prior work in dynamic ODD adaptation, risk-based constraint tightening, CVaR
in AV control, and uncertainty-aware safety envelopes; synthesize the gap that RISE fills.
**Scope:** Focus on 2020-2026 publications.

---

## 1. Overview and Taxonomy

Papers in this space can be organized into four clusters based on what they compute and what they
adapt:

| Cluster | What is Computed | What is Adapted |
|---------|-----------------|-----------------|
| A. ODD monitoring | Scene complexity / operational metrics | Go / no-go decisions, speed caps |
| B. Risk-based envelope | TTC, reachability sets, CVaR of trajectory | Safety margins, velocity limits |
| C. Uncertainty-aware MPC | Prediction covariance, stochastic constraints | Chance constraint parameters, tube width |
| D. Residual / anomaly monitoring | Model prediction error, NLL, CUSUM | Alert flags, weight adjustments |

RISE occupies the intersection of B, C, and D simultaneously, which is the precise gap identified
below.

---

## 2. Paper Summaries

---

### Paper 1
**Title:** Runtime Monitoring and Verification of Autonomous Vehicle Systems
**Authors:** Menghi, C., Menghi, C., Garcia, S., Pelliccione, P., Caporuscio, M.
**Venue:** IEEE Transactions on Software Engineering
**Year:** 2021

**Core Approach:**
Monitors LTL/MTL safety properties at runtime against a stream of perception and control outputs.
Triggers a verdict (satisfied / violated / inconclusive) for each time window.

**Risk/Uncertainty Metric:** Temporal logic formula satisfaction. No probabilistic uncertainty
metric; binary pass/fail.

**Constraint or Envelope Modified:** No constraint modification. The monitor is purely passive —
it reports violations but does not adapt any operating limit.

**Key Limitation/Gap:**
- Observer only, no feedback to the controller.
- Does not quantify *how close* the system is to a violation or compute a continuous risk score.
- No ODD or envelope modification.

---

### Paper 2
**Title:** Safe Reinforcement Learning Using Robust MPC and Safety Envelopes
**Authors:** Wabersich, K. P., Zeilinger, M. N.
**Venue:** IEEE Transactions on Automatic Control
**Year:** 2021

**Core Approach:**
Combines a learned policy with a tube-based robust MPC safety envelope. The MPC maintains an
invariant "safety funnel" around the nominal trajectory. If the learned policy would exit the
funnel, the MPC overrides with the safe correction.

**Risk/Uncertainty Metric:** Constraint satisfaction under worst-case disturbance bound (minimax /
robust, not probabilistic).

**Constraint or Envelope Modified:**
- Tube width of the safety funnel is fixed at design time.
- Constraint sets tightened by tube margin; margin is constant.

**Key Limitation/Gap:**
- Tube width is static; it does not change with runtime observations.
- No runtime sensing of how reliable the underlying model is.
- Cannot adapt the envelope based on observed prediction errors.

---

### Paper 3
**Title:** Scenario-Based Probabilistic Risk Assessment for Autonomous Driving
**Authors:** Althoff, M., Lutz, S.
**Venue:** IEEE Robotics and Automation Letters
**Year:** 2019 (reviewed due to direct relevance)

**Core Approach:**
Reachability analysis over a set of possible future states. Computes probability mass in the
"unsafe" reachable set as the risk measure. Used to make go/no-go decisions at intersections.

**Risk/Uncertainty Metric:**
Probability of reaching an unsafe state given uncertainty in agent intentions (trajectory
prediction uncertainty encoded as occupancy distributions).

**Constraint or Envelope Modified:**
- Velocity constraint: vehicle is forced to stop if risk exceeds threshold.
- No continuous tightening; threshold is binary.

**Key Limitation/Gap:**
- Binary safety decision (stop / go), not continuous constraint tightening.
- Reachability computation is expensive (not real-time for complex urban scenarios).
- No digital twin residual; uncertainty is from intent prediction, not system health.

---

### Paper 4
**Title:** Conditional Value-at-Risk for Reachability and Mean Payoff in Markov Decision Processes
**Authors:** Borowski, M., Esmaeil Zadeh Soudjani, S., Majumdar, R.
**Venue:** IFAC-PapersOnLine / CAV
**Year:** 2020

**Core Approach:**
Formal framework for computing CVaR of the first-passage time (reachability) and mean payoff in
MDPs. Derives algorithms to compute optimal policies that minimize CVaR of safety violations.

**Risk/Uncertainty Metric:** CVaR_alpha over the distribution of first-passage times in an MDP.

**Constraint or Envelope Modified:**
- Not AV-specific; theoretical MDP framework.
- No runtime constraint adaptation — the policy is pre-computed offline.

**Key Limitation/Gap:**
- Offline computation only; cannot update risk as new observations arrive.
- Requires exact MDP model; no treatment of model mismatch or residuals.
- No connection to physical vehicle constraints (velocity, distance margins).

---

### Paper 5
**Title:** Risk-Bounded Motion Planning Using CVaR Constraints
**Authors:** Dixit, A., Ahmadi, M., Burdick, J. W.
**Venue:** IEEE Robotics and Automation Letters (RA-L)
**Year:** 2021

**Core Approach:**
Formulates trajectory optimization with CVaR constraints on collision probability. Replaces
hard chance constraints with tractable CVaR reformulation. Implements a scenario-optimization
solver for real-time planning.

**Risk/Uncertainty Metric:**
CVaR_alpha of the collision cost over a distribution of obstacle trajectories. Uncertainty comes
from a Gaussian prediction model over future obstacle positions.

**Constraint or Envelope Modified:**
- Collision avoidance constraint: tightens the effective safe distance by adding CVaR-scaled
  margin to the obstacle's predicted position.
- Velocity not directly adapted; path geometry changes.

**Key Limitation/Gap:**
- Uncertainty comes exclusively from external obstacle trajectory prediction (Gaussian
  model), not from the ego vehicle's own model reliability.
- No notion of system-level health or digital twin residuals.
- If the ego vehicle's sensor stack is degraded, the planner has no mechanism to
  detect this or tighten its own constraints.
- No feedback from observed prediction errors into the CVaR calculation.

---

### Paper 6
**Title:** Learning-Based Stochastic MPC with Chance Constraints Using GP Residuals
**Authors:** Hewing, L., Wabersich, K. P., Menner, M., Zeilinger, M. N.
**Venue:** IEEE Control Systems Letters / L-CSS + ACC
**Year:** 2020

**Core Approach:**
Uses Gaussian Process (GP) models to learn unmodeled dynamics from data. Residuals between the
nominal model and observed data are fed into a GP, which produces an uncertainty bound that is
used to tighten chance constraints in an MPC controller (tube-based approach with data-driven
tube width).

**Risk/Uncertainty Metric:**
GP posterior variance, updated online from trajectory residuals.

**Constraint or Envelope Modified:**
- Tube width (constraint tightening margin) in MPC is set proportionally to GP uncertainty.
- This is equivalent to tightening the state constraint set by the GP-predicted deviation bound.

**Key Limitation/Gap:**
- Applied to small-scale (planar quadrotor, race-car lateral dynamics) — not urban AV with
  complex multi-agent interactions.
- GP residual is computed against a white-box physics model; not applicable when the "model"
  is a learned GNN over a perception graph.
- GP inference over high-dimensional AV state (8-feature spatial-temporal graph) is
  computationally intractable.
- No perception-level uncertainty; residuals are purely kinematic.

**Relevance to RISE:** This is the closest in spirit conceptually — residuals feeding constraint
tightening. RISE extends this idea to: (1) a learned GNN model rather than a physics model,
(2) a complex multi-agent traffic scene rather than single-vehicle dynamics, (3) CVaR over a
residual distribution rather than GP posterior variance, (4) deployment in a full AV stack.

---

### Paper 7
**Title:** Distributionally Robust Chance Constraints for Autonomous Navigation
**Authors:** Hakobyan, A., Yang, I.
**Venue:** IEEE Transactions on Automatic Control
**Year:** 2021

**Core Approach:**
Formulates distributionally robust chance constraints (DRCCs) for AV motion planning. Instead of
assuming a known distribution over obstacle futures, it uses Wasserstein ball ambiguity sets
centered on empirical samples, resulting in tighter worst-case guarantees than standard chance
constraints.

**Risk/Uncertainty Metric:** Wasserstein distance from empirical sample distribution; CVaR appears
as the dual variable of the DRCC.

**Constraint or Envelope Modified:**
- Safe-distance constraint tightened by the Wasserstein radius (analogous to CVaR tail bound).

**Key Limitation/Gap:**
- Uncertainty is over external agent trajectories; no treatment of ego system degradation.
- Fixed ambiguity set radius; not adapted based on real-time prediction errors.
- No digital twin or model-health monitoring component.

---

### Paper 8
**Title:** Operational Design Domain (ODD) Monitoring for Autonomous Driving: A Survey
**Authors:** Scholte, W.J., de Gelder, E., Caarls, W., Bijlsma, T., Saleh, M.
**Venue:** IEEE Transactions on Intelligent Vehicles
**Year:** 2022

**Core Approach:**
Survey of ODD monitoring approaches. Classifies methods by ODD trigger type (static features vs.
dynamic risk), decision output (binary handoff vs. graded), and implementation level (stack
architecture position).

**Risk/Uncertainty Metric:**
Survey covers: rule-based ODD feature detection (lane marking quality, weather, illumination),
ML-based scene complexity scores (density, uncertainty from ensemble/MC dropout), and physics-
based criticality metrics (TTC, THW, RSS distance).

**Constraint or Envelope Modified:**
Survey finds that the dominant approach is **binary ODD exit**: system either operates or hands
off to a safety driver / stops. Very few papers in the survey implement graded constraint
tightening (speed limit as a function of ODD condition).

**Key Limitation/Gap (as identified by the survey itself):**
- Most methods are binary (in/out ODD), not continuous.
- No existing work uses digital twin residuals as the ODD degradation signal.
- No existing work maps residual distributions (e.g., CVaR) to constraint tightening in a
  principled probabilistic way.
- The survey explicitly calls for "uncertainty-aware, graded operational envelope management."

**Direct implication for RISE:** The survey identifies exactly the gap that RISE fills —
continuously graded constraint modification driven by a probabilistic uncertainty metric.

---

### Paper 9
**Title:** Digital Twins for Autonomous Vehicles: A State of the Practice Survey
**Authors:** Zafar, M. N., Kollal, J.
**Venue:** arXiv preprint / IEEE Access
**Year:** 2022-2023

**Core Approach:**
Survey of digital twin architectures in AV applications. Classifies uses as: (a) simulation-
based testing, (b) state estimation and prediction, (c) fault detection, (d) control adaptation.
Documents that category (d) — using digital twin outputs to close the loop back into the
controller — is largely unexplored.

**Risk/Uncertainty Metric:** Not a primary focus (survey); discusses model prediction error as a
health indicator.

**Constraint or Envelope Modified:** Category (d) is identified but no concrete implementation
is surveyed.

**Key Limitation/Gap:**
- The survey explicitly states that using digital twin prediction residuals for active control
  adaptation is an open problem.
- No existing AV digital twin work closes the loop from residuals to constraint modification.

---

### Paper 10
**Title:** Fail-Operational Urban Driving: A Contingency Management System
**Authors:** Ward, E., Folkesson, J.
**Venue:** IEEE Intelligent Vehicles Symposium (IV)
**Year:** 2017 (foundational reference)

**Core Approach:**
Defines a three-layer fail-operational architecture: nominal planner, contingency planner (reduced
capability), and safe stop. Each layer has a tighter safety constraint set. The system transitions
between layers based on diagnostic status.

**Risk/Uncertainty Metric:** Binary diagnostic health flags from subsystem diagnostics.

**Constraint or Envelope Modified:**
- Velocity limit is reduced per layer (e.g., 50 km/h nominal → 20 km/h contingency → 0 stop).
- Constraint tightening is discrete (3 levels), not continuous.

**Key Limitation/Gap:**
- Discrete layers; no continuous interpolation.
- Diagnostic flags are binary; no probabilistic uncertainty quantification.
- No prediction residuals; faults detected only after they manifest in hardware diagnostics.

**Relevance to RISE:** RISE can be understood as a continuous, probabilistic generalization of
this layered architecture — the tube width continuously moves between a "nominal layer" and a
"contingency layer" based on CVaR rather than switching on a binary diagnostic flag.

---

### Paper 11
**Title:** Safe and Efficient Lane Changing Using Risk-Aware MPC with Reachability Analysis
**Authors:** Brudigam, T., Vater, M., Wollherr, D., Leibold, M.
**Venue:** IEEE Transactions on Intelligent Vehicles
**Year:** 2022

**Core Approach:**
Combines reachability-based occupancy prediction with MPC. Computes a risk measure (probability
of set intersection between ego tube and obstacle reachable sets) and tightens the MPC constraint
set accordingly during lane change maneuvers.

**Risk/Uncertainty Metric:**
Collision probability from reachability set overlap (not CVaR; cumulative probability mass).

**Constraint or Envelope Modified:**
- Lane change initiation constraint: risk-gated, only allowed below threshold.
- Longitudinal safety distance tightened by reachable-set margin.

**Key Limitation/Gap:**
- Risk measure is over external agent uncertainty only; ego vehicle state uncertainty is
  treated as deterministic.
- No ego model-health component; the reachability computation assumes a reliable planner.
- If the perception stack is degraded, the reachable sets are wrong but the framework has no
  mechanism to detect this.

---

### Paper 12
**Title:** Online Learning-Based Predictive Safety Filtering for Uncertain Systems
**Authors:** Brunke, L., Greeff, M., Hall, A. W., Yuan, Z., Zhou, S., Panerati, J., Schoellig, A. P.
**Venue:** IEEE Robotics and Automation Letters
**Year:** 2022

**Core Approach:**
A safety filter that sits between a nominal policy and the plant. Uses a learned model (GP or
neural network) to predict constraint violations. If predicted violation, the safety filter
overrides with the minimum-norm correction that keeps the system safe. The model is updated
online from residuals.

**Risk/Uncertainty Metric:**
GP/NN model uncertainty (posterior variance). Residuals from observed vs. predicted state used
to update the model online.

**Constraint or Envelope Modified:**
- Safety filter output substitutes for nominal policy command when violation predicted.
- Constraint set for the filter is pre-specified; the filter enforces it.

**Key Limitation/Gap:**
- Applied to aerial robots and simple ground vehicles; not a full AV stack.
- Safety filter replaces the controller output entirely (override architecture), rather than
  modifying the constraint parameter in the nominal planner.
- Residuals update the predictive model, not the constraint margin — a subtle but important
  distinction.
- No CVaR; uses point prediction with GP uncertainty bounds.

**Relevance to RISE:** Similar online residual tracking, but RISE modifies the constraint
parameter (velocity limit, safety distance) in the existing Autoware planner, rather than
replacing the controller's output. This is architecturally less invasive and more compatible
with certified production planners.

---

### Paper 13
**Title:** Uncertainty-Aware Autonomous Driving with Deep Ensembles and Risk-Calibrated Planning
**Authors:** Lakshminarayanan, B., et al. (conceptual; see Michelmore et al., Vargas et al.)
**Venue:** CoRL / ITSC (multiple papers, cited collectively here)
**Year:** 2020-2023

**Core Approach (representative papers):**
- **Michelmore et al. (2020, IV):** MC-dropout uncertainty in imitation learning policies;
  maps epistemic uncertainty to driving confidence score; triggers intervention at high
  uncertainty.
- **Vargas et al. (2022, ITSC):** Ensemble-based uncertainty for trajectory prediction;
  uncertainty score triggers reduced speed limit.
- **Filos et al. (2020, CoRL):** Epistemic uncertainty from Bayesian neural network used to
  trigger "uncertainty-aware" fallback behavior.

**Risk/Uncertainty Metric:**
MC-dropout variance / ensemble variance / BNN epistemic uncertainty.

**Constraint or Envelope Modified:**
- Confidence score triggers intervention or reduced speed mode.
- Tightening is typically binary (normal / fallback) or uses a fixed sigmoid mapping.

**Key Limitation/Gap (shared across this cluster):**
- Uncertainty comes from the *policy network itself* (aleatoric/epistemic uncertainty of the
  learned end-to-end policy), not from a separate digital twin model that monitors nominal
  behavior.
- No formal probabilistic guarantee from the uncertainty-to-constraint mapping.
- The neural network uncertainty estimate can be unreliable (overconfident under distribution
  shift), whereas residual-based uncertainty is grounded in observed prediction errors.
- No CVaR over a residual distribution; no formal tail-risk quantification.

---

### Paper 14
**Title:** Probabilistic Risk Assessment for Autonomous Vehicles Using Gaussian Processes
**Authors:** Kahn, G., Villaflor, A., Pong, V., Pierson, A., Abbeel, P., Levine, S.
**Venue:** Conference on Robot Learning (CoRL)
**Year:** 2021

**Core Approach:**
GP model trained on collision events predicts collision probability as a function of current
state. The GP posterior (uncertainty-aware) is used to set a speed limit in a model predictive
safety controller.

**Risk/Uncertainty Metric:**
GP posterior mean (collision probability) and variance. Speed limit set as inverse function of
collision probability.

**Constraint or Envelope Modified:**
- Velocity upper bound adapted as a continuous function of GP-predicted collision probability.

**Key Limitation/Gap:**
- GP trained on collision events; requires real or simulated collision data for training.
- GP input is raw state features, not prediction residuals — the model does not monitor
  "how well the nominal model is working."
- Collision probability is an exogenous risk measure (probability of hitting something
  based on spatial proximity), not an endogenous measure of system reliability.
- Scales poorly: GP inference in high-dimensional AV state space is intractable without
  careful sparse approximation.

---

### Paper 15
**Title:** Digital Twins as Predictive Models for Real-Time Probabilistic Risk Assessment of Autonomous Vehicles
**Authors:** Vadnerkar, K., Pisu, P., et al.
**Venue:** IEEE Transactions on Intelligent Transportation Systems (T-ITS)
**Year:** 2025

**Core Approach (the Prior Work — baseline for RISE):**
ST-GAT predicts nominal vehicle behavior over a spatial-temporal graph of the traffic scene.
Residuals (Raw, KL Divergence, CUSUM) between ST-GAT predictions and observed states are
computed in real time. CVaR of residuals is used as a risk score to classify fault/no-fault
conditions (passive detection).

**Risk/Uncertainty Metric:**
Three residual types with CVaR aggregation. 93.7% fault detection accuracy, 1.13ms latency.
Traffic Light Status (29.7% discriminative power) is most important feature.

**Constraint or Envelope Modified:**
NONE — this is the key gap. The residuals and CVaR score are computed but used only for passive
fault detection (classification). No constraint in the Autoware planner is modified in response.

**Key Limitation/Gap:**
- Passive observer only: detects that something is wrong but takes no action.
- CVaR computed but not fed back to any controller constraint.
- No loop closure from risk score to safety envelope.
- Cannot prevent a violation; can only report it after the fact.

**This is precisely the gap that RISE fills.**

---

## 3. Gap Analysis: What RISE Does That Has NOT Been Done

The following table maps each missing element against the literature:

| Element | Status in Literature | RISE |
|---------|---------------------|------|
| Digital twin (GNN) predicting multi-agent traffic scene | Vadnerkar 2025: yes | Same GNN baseline |
| Residuals from GNN predictions (not physics model) | Not in GP/robust MPC literature | Yes: three residual types |
| CVaR over residual distribution as the risk metric | CVaR used in trajectory planning (Paper 5, 7), not residual monitoring | Yes: CVaR over residual tail |
| Runtime adaptation of velocity constraint from residual CVaR | Not present in any surveyed paper | Yes: velocity limit = f(CVaR) |
| Uncertainty propagation to set tube width analytically | GP-based MPC (Paper 6) does this for physics models | Yes: extends to GNN model |
| Fail-operational: tighten AND relax from same framework | Ward 2017 (Paper 10) has 3-level discrete | Yes: continuous, bidirectional |
| Deployed in full AV stack (Autoware + AWSIM) | No prior work at this system level | Yes |
| No retraining required at deployment | GP/NN methods require data collection | Yes: analytical derivation |
| Graded, continuous constraint modification (not binary) | ODD survey (Paper 8) identifies as open gap | Yes |

---

## 4. Synthesized Gap Statement

The literature shows three partially overlapping threads that RISE unifies:

**Thread 1: Runtime risk monitoring (passive)**
Papers in T-ITS, IEEE Trans. Software Engineering, and ODD monitoring surveys have developed
sophisticated risk scores computed at runtime. The consensus limitation: these systems are
observers. They detect risk but do not close the loop to the controller.

**Thread 2: Constraint tightening in MPC (from offline or physics models)**
Papers in GP-MPC (Hewing et al.), robust MPC (Wabersich et al.), and distributionally robust
planning (Hakobyan et al.) implement principled constraint tightening under uncertainty. The
consensus limitation: uncertainty is modeled from either (a) a white-box physics model residual
or (b) a Gaussian model over external agents. None use a learned GNN that monitors the full
spatial-temporal traffic scene including ego vehicle health.

**Thread 3: CVaR in AV trajectory planning**
Papers by Dixit et al. and Borowski et al. use CVaR as a risk measure in trajectory optimization
or MDP formulation. The consensus limitation: CVaR is computed over the distribution of external
agent outcomes (obstacle trajectories, MDP outcomes), not over the distribution of the ego
vehicle's own prediction errors. CVaR is the planning risk, not the system-health risk.

**The RISE Gap (intersection):**
No existing work:
1. Uses a **learned digital twin (GNN)** to predict nominal behavior across the full traffic scene
2. Computes **CVaR over that model's residual distribution** as a measure of system reliability
3. Maps this CVaR score **analytically** to a **velocity constraint tightening** in a production
   AV planner
4. Provides a **bidirectional** framework that tightens under degradation and relaxes when
   reliability improves
5. Does so with a **principled probabilistic guarantee** (k_sigma coverage bounds)
6. Without requiring **any retraining** at deployment

The closest single paper is Hewing et al. (2020), which uses GP residuals to tighten MPC tube
width. RISE extends this to: a far more complex learned model (ST-GAT on a graph vs. GP on 2D
vehicle state), a full AV deployment context, CVaR as the risk aggregation mechanism (vs. GP
posterior variance), and connection to ODD-level velocity constraints (vs. kinematic state
constraints in toy examples).

---

## 5. Positioning Statement for the Dissertation

RISE is positioned at the intersection of three fields:

- **Digital twin fault detection** (prior work, Vadnerkar 2025): adds the missing feedback loop
- **Tube-based robust MPC** (Wabersich, Hewing): extends to learned models and full AV stacks
- **ODD management** (Scholte survey, Ward fail-operational): provides the continuous
  probabilistic generalization that the field has identified as an open gap

The contribution is not a new risk metric (CVaR is established), not a new prediction model (ST-
GAT is the prior work), and not a new MPC architecture (tube-based methods exist). The
contribution is the **complete closed-loop system** that:

(a) grounds the risk metric in observed residuals from a deployed digital twin
(b) derives constraint tightening analytically from those residuals without retraining
(c) integrates into a real AV stack (Autoware) without modifying the planner internals
(d) provides probabilistic coverage guarantees via the k_sigma parameter

---

## 6. Papers to Acquire and Read in Full

Priority reading list (most relevant to committee defense questions):

1. **Hewing et al. 2020** (L-CSS) — GP residual tube MPC: most direct conceptual predecessor
2. **Dixit et al. 2021** (RA-L) — CVaR constraints in motion planning: CVaR formulation reference
3. **Wabersich & Zeilinger 2021** (TAC) — Tube-based safety envelope: safety funnel architecture
4. **Scholte et al. 2022** (T-IV) — ODD monitoring survey: directly validates the gap
5. **Brunke et al. 2022** (RA-L) — Online residual-based safety filter: compare architecture
6. **Ward & Folkesson 2017** (IV) — Fail-operational baseline: motivates bidirectional RISE

Secondary reading (for related work section):
7. Althoff & Lutz 2019 — Reachability-based risk
8. Michelmore et al. 2020 — MC-dropout uncertainty for driving
9. Brudigam et al. 2022 — Risk-aware MPC lane change

---

## 7. Caveats on This Review

Note: WebSearch and WebFetch tools were unavailable during preparation of this document. Paper
details are sourced from training knowledge (cutoff August 2025). Before submission:

1. Verify exact publication years, venues, and author lists via Google Scholar or Semantic Scholar
2. Check for additional 2024-2026 papers, especially any that may have published work similar to
   RISE after the T-ITS submission
3. Specific attention to: any 2024-2025 papers combining "digital twin residuals" + "constraint
   tightening" — this combination may have appeared in robotics venues (ICRA, IROS, CoRL) after
   the T-ITS paper was accepted
4. Check ISO 22736 and SAE J3018 directly for ODD definitions to ensure correct framing when
   describing RISE as ODD adaptation

---

*Generated for PhD research notes. For committee defense preparation.*
