# P-MOCHA Research Plan

## 0. Working Thesis

This work should not be framed as `MOCHA + RAST`.

The stronger framing is:

> Perception uncertainty changes not only local collision cost, but also the safe topology and the best observation strategy. We therefore convert dynamic risk, perception uncertainty, and occlusion into persistent multi-homotopy risk structures, and optimize MOCHA's low-dimensional trajectory variables to jointly improve motion safety and future observability.

Tentative name:

**P-MOCHA: Perception-aware Multi-Homotopy Trajectory Optimization under Dynamic Uncertainty and Occlusion**

Short Chinese description:

**感知不确定性驱动的多同伦主动观测轨迹优化。**

The core research question is:

> Given an uncertain dynamic map, should the robot merely avoid high-risk regions, or should it actively choose a trajectory that reveals future risk earlier and more reliably?

The answer we want to build is the second one.

## 1. Current Codebase Reading

The current workspace contains two relevant code families.

### 1.1 DYNUS

Path: `dynus/`

DYNUS is an uncertainty-aware trajectory planner in dynamic unknown environments. It already contains several pieces that are close to the desired direction:

- Dynamic trajectory management through `dynTraj`.
- Unknown-cell based uncertainty in `DYNUS::computeRisk`.
- Dynamic obstacle proximity/count risk in `DYNUS::computeRisk`.
- Safe corridor generation through `DYNUS::getSafeCorridor`.
- Replanning pipeline through `DYNUS::replan`.
- Global path generation and path pushing through `DYNUS::generateGlobalPath` and `DYNUS::pushPath`.
- Local trajectory optimization through `DYNUS::generateLocalTrajectory`.
- Frontier extraction through `DYNUS::findFrontiers`, `DYNUS::extractFrontierPoints`, and `DYNUS::computeAllFrontierPoints`.
- Yaw/observation planning through `YawSolver`, including collision likelihood, time-since-observed, proximity, and yaw change costs.

Important observation:

DYNUS already has uncertainty and some active observation ingredients, but they are not yet a unified differentiable perception-planning objective. Frontier selection and yaw tracking are mostly separate from the core trajectory optimization.

### 1.2 RAST Corridor Planning

Path: `RAST_corridor_planning/`

The RAST-style code contains:

- Future risk map input through `future_risk_global`.
- Risk-aware kinodynamic A* in `risk_aware_kinodynamic_a_star.h`.
- Motion primitive validation using risk thresholds.
- Corridor expansion using `findCorridors`.
- Mini-snap optimization inside risk-constrained dynamic corridors.

Important observation:

RAST's pipeline is:

```text
future risk map -> risk-aware kinodynamic A* -> one corridor chain -> polynomial optimization
```

This is useful as a baseline and implementation reference, but not enough as the new contribution. Its key limitation for our purpose is that the trajectory is risk-aware but not actively perception-improving.

### 1.3 Implication for Our Work

The codebase already has enough infrastructure for a staged implementation:

- Use DYNUS for map, dynamic obstacles, frontiers, yaw, and replanning.
- Use RAST as a reference for risk map and corridor construction.
- Add a new layer that generates and evaluates multiple risk/visibility homotopy candidates.
- Add active observation terms before attempting full differentiable integration into the optimizer.

## 2. Related Paper Positioning

### 2.1 RAST

Paper:

**RAST: Risk-Aware Spatio-Temporal Safety Corridors for MAV Navigation in Dynamic Uncertain Environments**, IEEE RA-L 2023.

Key idea:

- Use a particle-based dynamic map.
- Predict future risk.
- Build risk-aware spatio-temporal safety corridors.
- Optimize a dynamically feasible MAV trajectory inside the corridor.

What we borrow:

- Risk map over space-time.
- Corridor construction from uncertain dynamic predictions.
- Validation idea under different uncertainty levels.

What we should not copy:

- Single selected corridor as the main representation.
- Risk-aware but passive planning.
- MAV-centric formulation as the main novelty.

Our separation:

> RAST transforms uncertainty into a safety corridor. P-MOCHA transforms uncertainty into multiple persistent risk-topological candidates and optimizes both safety and future observability.

### 2.2 Occlusion-Aware MPC

Representative paper:

**OA-MPC: Occlusion-Aware MPC for Guaranteed Safe Robot Navigation with Unseen Dynamic Obstacles**, 2022/2023.

Key idea:

- Model possible hidden agents behind occlusions.
- Use reachable sets to guarantee safety.
- Optimize with MPC and terminal stopping constraints.

What we borrow:

- Occlusion is not merely unknown occupancy; it is potential future dynamic risk.
- Safety around unseen agents must be evaluated before they appear.

Our separation:

- We do not only enforce worst-case safety.
- We want a trajectory that actively reduces occlusion-induced uncertainty.
- We integrate this into multi-homotopy trajectory selection and MOCHA's low-dimensional optimization.

### 2.3 Alternate Perspective / Visibility Cost Maps

Representative line:

**Estimating Visibility from Alternate Perspectives for Motion Planning with Occlusions**, 2024.

Key idea:

- Estimate what would become visible from alternative viewpoints.
- Construct a visibility/information cost map for planning.

What we borrow:

- Visibility from candidate future poses can be approximated before reaching them.
- The planner can prefer paths that reveal occluded regions earlier.

Our separation:

- We do not want only a grid cost map.
- We want visibility to affect homotopy selection and continuous trajectory optimization.

### 2.4 Active Perception and Observability-Aware Planning

Representative directions:

- Active perception with NeRF or semantic maps.
- Observability-aware trajectory optimization.
- Viewpoint planning / next-best-view methods.
- Target tracking with field-of-view and line-of-sight constraints.

Their common limitation for our target:

- Many methods choose viewpoints first, then plan a trajectory.
- Many do not handle dynamic obstacle risk and vehicle dynamics in one continuous-time optimization.
- Many do not use the low-dimensional structure that MOCHA provides.

Our separation:

> The same low-dimensional variables should control both motion feasibility and future observability.

## 3. Mathematical Formulation

### 3.1 State, Trajectory, and Low-Dimensional Variables

Let the robot trajectory be:

```text
q(t) = [p(t), psi(t)]
```

where:

- `p(t)` is position.
- `psi(t)` is yaw or sensing direction.

MOCHA reduces trajectory optimization from high-dimensional waypoint variables to low-dimensional parameters:

```text
z = [lambda_1, ..., lambda_M, tau_1, ..., tau_N]
```

or the equivalent low-dimensional parameterization used by the optimizer.

The important point is that every cost below should ultimately be evaluable as:

```text
J(z) = integral over t of cost(q(t; z), t)
```

This makes the perception objective part of MOCHA rather than an external module.

### 3.2 Planner-Facing Perception Representation

The perception module should expose a planner-facing belief field:

```text
B(x, t) = { R(x, t), U(x, t), O(x, t), S(x, t) }
```

where:

- `R(x,t)` is dynamic collision risk.
- `U(x,t)` is perception/prediction uncertainty.
- `O(x,t)` is occlusion or unobserved-shadow risk.
- `S(x,t)` is semantic/task importance if needed later.

Minimal first version:

```text
B_0(x,t) = { R(x,t), U(x,t) }
```

Recommended first publishable version:

```text
B_1(x,t) = { R(x,t), U(x,t), O(x,t), V(q,x,t) }
```

where `V(q,x,t)` is visibility from robot state `q` to point or region `x`.

### 3.3 Risk and Uncertainty Cost

For a trajectory `gamma`, define:

```text
J_risk(gamma) = integral_0^T phi(R(p(t), t) - R_safe) dt
```

where `phi` is a smooth hinge, for example:

```text
phi(s) = log(1 + exp(beta s)) / beta
```

Uncertainty cost:

```text
J_unc(gamma) = integral_0^T U(p(t), t) dt
```

This handles the passive part:

- Avoid high risk.
- Prefer lower uncertainty paths.

But this alone is not enough for the paper. It is still mostly risk-aware planning.

### 3.4 Occlusion Shadow Risk

Define occluded or unknown regions:

```text
Omega_occ(t)
```

For each occluded cell or region `u`, define a hidden-agent probability or dynamic relevance:

```text
P_hidden(u, t)
```

The occlusion shadow risk of a trajectory can be:

```text
J_shadow(gamma) =
integral_0^T sum_{u in Omega_occ(t)} P_hidden(u,t) * A(u, p(t), t) dt
```

where `A(u,p,t)` measures whether an unseen agent from `u` can affect the robot near future. For a first implementation:

```text
A(u,p,t) = exp(-d(u, p)^2 / sigma_a^2)
```

For a stronger version:

```text
A(u,p,t) = 1 if Reach(u, t -> t + Delta) intersects robot swept volume
```

This term says:

> Unknown space is not uniformly bad. It is bad if it can hide something that may soon interact with the robot.

### 3.5 Visibility and Active Observation Gain

Define visibility from robot pose `q(t)` to point or region `x`:

```text
V(q(t), x, t) =
I_FOV(q(t), x) * I_LOS(q(t), x) * Q_dist(d(q(t),x)) * Q_angle(theta(q(t),x))
```

Hard indicators can be softened for optimization:

```text
I_FOV -> sigmoid(theta_max - abs(theta))
I_LOS -> exp(-k * accumulated_occupancy_along_ray)
Q_dist -> exp(-(d - d_best)^2 / sigma_d^2)
```

Active observation gain:

```text
G_obs(gamma) =
integral_0^T sum_{u in Omega_interest(t)} W(u,t) * V(q(t), u, t) dt
```

where `Omega_interest` can include:

- Occlusion boundaries.
- Frontiers.
- Predicted dynamic obstacles.
- High uncertainty cells.
- Targets likely to be lost.

The active perception loss is:

```text
J_obs(gamma) = -G_obs(gamma)
```

or equivalently:

```text
J_obs(gamma) =
integral_0^T sum_u W(u,t) * (1 - V(q(t),u,t)) dt
```

This is the central new term.

### 3.6 Persistent Risk Homotopy

A single risk threshold is unstable:

```text
R(x,t) > rho
```

Small perception noise can flip a passage from free to blocked.

Instead use multiple thresholds:

```text
rho_1 < rho_2 < ... < rho_L
```

At each threshold, build a risk-obstacle set:

```text
O_rho_l(t) = { x | R(x,t) + eta U(x,t) > rho_l }
```

For a path `gamma`, compute a signature:

```text
H_risk(gamma) = [ h_{rho_1}(gamma), ..., h_{rho_L}(gamma) ]
```

where each `h_rho` encodes which side of each risk component the path passes.

A path is topologically stable if its signature does not change across nearby thresholds or nearby perception updates:

```text
Stability(gamma) = consistency(H_risk(gamma; B_t), H_risk(gamma; B_{t-1}))
```

First implementation does not need full algebraic topology. A practical version can label paths by:

- Which connected high-risk components they pass left/right of.
- Which occlusion/frontier components they observe.
- Which dynamic obstacles they pass before/after.

This creates:

```text
H_perception(gamma) = [ H_geo, H_risk, H_occ, H_vis ]
```

This is stronger than ordinary homotopy because two geometrically similar paths can have different visibility signatures.

### 3.7 Final Objective

For each candidate homotopy/corridor `k`, optimize:

```text
min_z J_k(z)
```

with:

```text
J_k =
w_s J_smooth
+ w_t J_time
+ w_d J_dyn
+ w_c J_collision
+ w_r J_risk
+ w_u J_unc
+ w_o J_shadow
+ w_v J_obs
+ w_h J_corridor
```

Subject to:

- Vehicle dynamics.
- Collision clearance.
- Corridor constraints if using hard corridors.
- Platform-specific constraints for omnidirectional or Ackermann chassis.

Key design choice:

Do not add every term at once. First implement candidate-level scoring, then continuous optimization.

## 4. What Is Mathematically Strong and What Is Risky

### 4.1 Strong Points

The direction is mathematically coherent because:

- RAST-style risk maps already provide `R(x,t)`.
- DYNUS already computes unknown-cell risk and dynamic obstacle risk.
- Frontier extraction gives a natural approximation of `Omega_interest`.
- Yaw solver already has time-since-observed and dynamic obstacle tracking logic.
- MOCHA's low-dimensional parameterization gives a clean place to add costs without exploding variable dimension.
- Multi-homotopy candidates give a natural way to avoid local minima.

### 4.2 Risky Points

There are three mathematical risks.

First, exact homotopy in dynamic uncertain fields may become too complex. We should not overclaim formal topological guarantees at the beginning.

Practical answer:

Use persistent signatures as candidate labels and stability metrics first. Formalize only the parts we can evaluate.

Second, visibility and line-of-sight are non-smooth. Direct gradient optimization may be unstable.

Practical answer:

Start with discrete candidate scoring and soft approximations. Only after the behavior is verified, add differentiable visibility costs.

Third, active observation can fight motion efficiency. The robot may detour too much to see uncertain regions.

Practical answer:

Use risk-weighted observation gain. The robot should observe unknown regions only when they can affect near-future safety or task success.

## 5. Code Integration Strategy

### 5.1 Phase 0: Baseline Reproduction

Goal:

Establish reliable baselines before adding new terms.

Use:

- Original DYNUS pipeline.
- RAST corridor planner as a separate baseline.
- MOCHA original implementation if available in another branch or repo.

Metrics:

- Success rate.
- Collision rate.
- Planning latency.
- Dynamic obstacle near-miss distance.
- Path length/time.
- Replan count.

Expected output:

A repeatable script/config set for dynamic occlusion scenes.

### 5.2 Phase 1: Risk Field Adapter

Goal:

Create a common planner-facing risk interface:

```cpp
struct PerceptionRiskField {
  double risk(const Eigen::Vector3d& x, double t) const;
  double uncertainty(const Eigen::Vector3d& x, double t) const;
  double occlusion(const Eigen::Vector3d& x, double t) const;
};
```

Implementation options:

- First version wraps DYNUS unknown cells and dynamic obstacles.
- Second version imports RAST-style future risk maps.
- Later version adds occlusion-shadow risk.

Where it connects:

- `DYNUS::computeRisk` for global scalar risk.
- `DYNUS::generateGlobalPath` or path-push stage for path scoring.
- `DYNUS::generateLocalTrajectory` or solver objective for continuous optimization.

### 5.3 Phase 2: Multi-Homotopy Candidate Generation

Goal:

Generate multiple candidate guide paths/corridors instead of one.

Minimal implementation:

- Run graph search with different risk thresholds.
- Run graph search with inflated/deflated dynamic obstacle risk.
- Keep K distinct candidates by spatial dissimilarity and risk signature.

Better implementation:

- Label candidates by `H_risk` and `H_occ`.
- Keep one or two representatives per label.

Where it connects:

- DYNUS global planner output currently gives one `global_path`.
- Replace or wrap it with `std::vector<GlobalCandidate>`.
- Each candidate carries path, risk score, uncertainty score, visibility score, and corridor.

### 5.4 Phase 3: Candidate-Level Active Observation Scoring

Goal:

Before changing the optimizer, prove that active observation changes the selected topology in useful ways.

For each candidate path `gamma_k`, evaluate:

```text
Score(gamma_k) =
J_motion
+ w_r J_risk
+ w_u J_unc
+ w_o J_shadow
- w_g G_obs
```

Use simple sampled trajectory points.

Visibility approximation:

- Check FoV angle.
- Check range.
- Optionally raycast in octomap for line-of-sight.
- Count visible frontiers, occlusion boundaries, or dynamic obstacles.

Where it connects:

- `DYNUS::computeAllFrontierPoints` already produces frontier points and directions.
- `YawSolver` already reasons about which obstacle to observe.
- Candidate scoring can be implemented before modifying Gurobi/MOCHA.

This phase should be the first real experiment because it is low-risk and directly tests the idea.

### 5.5 Phase 4: Occlusion Shadow Field

Goal:

Convert unknown/occluded regions into future dynamic risk rather than generic unknown cost.

Minimal version:

- Detect frontier cells near the planned path.
- Assign higher risk to unknown regions with line-of-sight blocked by obstacles.
- Decay risk with distance and time.

Better version:

- From each occlusion boundary, propagate possible hidden-agent reachable sets.
- Penalize paths whose swept volume intersects these reachable sets.

Where it connects:

- Octomap unknown cells from DYNUS.
- Frontier directions from `computeAllFrontierPoints`.
- Dynamic obstacle prediction structures from `dynTraj`.

### 5.6 Phase 5: Low-Dimensional Optimization Integration

Goal:

Move from candidate scoring to actual trajectory optimization.

First integration:

- Add sampled perception costs as soft reference/cost terms.
- Keep hard corridor constraints unchanged.
- Optimize within each selected corridor.

Second integration:

- Add differentiable sampled costs:

```text
sum_i R(p(t_i),t_i)
sum_i U(p(t_i),t_i)
sum_i O(p(t_i),t_i)
sum_i (1 - V(q(t_i), x_i, t_i))
```

For the existing DYNUS Gurobi solver, arbitrary nonlinear risk fields are not directly compatible with a quadratic objective. Therefore there are two options:

1. Use candidate-level scoring and keep Gurobi quadratic.
2. Add these costs in the MOCHA nonlinear optimizer if MOCHA uses gradient-based nonlinear optimization.

Recommendation:

Do not force nonlinear perception costs into the current Gurobi DYNUS solver. Use DYNUS for infrastructure and baselines, and integrate full perception costs into the MOCHA optimizer.

### 5.7 Phase 6: Full Closed-Loop Replanning

Goal:

Run:

```text
low-frequency: update risk topology and candidate guides
high-frequency: optimize within selected topology using MOCHA
```

Low frequency:

- Risk field update.
- Persistent homotopy candidate generation.
- Candidate selection.

High frequency:

- MOCHA re-optimization under current selected class.
- Safety check.
- Emergency fallback if risk topology changes abruptly.

This matches the desired paper story:

> Topology changes slowly and robustly; trajectory optimization runs fast.

## 6. Experiment Plan

### 6.1 Experiment A: Static Occlusion Toy World

Purpose:

Validate active observation without dynamic obstacle complexity.

Scene:

- Corridor corner.
- Static occluder.
- Goal behind or near occluded area.

Compare:

- Shortest/smoothest path.
- Risk-only path.
- Active observation path.

Metrics:

- Time-to-first-observation of hidden region.
- Occlusion exposure ratio.
- Path length.
- Planning time.

Expected result:

Active observation path detours slightly but reveals the hidden area earlier.

### 6.2 Experiment B: Hidden Dynamic Obstacle

Purpose:

Show why occlusion is safety-critical.

Scene:

- A pedestrian or moving obstacle emerges from behind a wall, car, shelf, or corner.
- The robot must reach a goal while avoiding collision.

Compare:

- DYNUS/MOCHA baseline.
- RAST-like risk corridor.
- P-MOCHA without active observation.
- P-MOCHA full.

Metrics:

- Collision rate.
- Near-miss distance.
- Braking/emergency replan count.
- Time-to-first-observation of hidden obstacle.
- Success rate.

Expected result:

Risk-only planners either become conservative or react late. P-MOCHA should reveal the occluded source earlier and reduce emergency replans.

### 6.3 Experiment C: Multi-Homotopy Selection

Purpose:

Prove that perception changes topology selection.

Scene:

- Two routes around an obstacle or shelf.
- Both are geometrically feasible.
- One route has poor visibility into a critical dynamic region.

Compare:

- Geometry-only homotopy selection.
- Risk-only homotopy selection.
- Risk + visibility homotopy selection.

Metrics:

- Selected homotopy class.
- Risk signature stability under noisy perception.
- Occlusion exposure ratio.
- Collision/success rate.

Expected result:

The selected path should sometimes be longer but safer because it observes the risky region earlier.

### 6.4 Experiment D: Perception Noise Robustness

Purpose:

Show value of persistent risk topology.

Scene:

- Same map with injected risk map noise.
- Dynamic obstacle prediction covariance varies.

Compare:

- Single risk threshold corridor.
- Multi-threshold persistent risk homotopy.

Metrics:

- Number of topology switches.
- Replan jitter.
- Success rate.
- Computation time.

Expected result:

Persistent signatures reduce topology flicker and unstable replanning.

### 6.5 Experiment E: Platform Comparison

Purpose:

Connect back to MOCHA's original strength.

Compare:

- Omnidirectional chassis.
- Ackermann or yaw-constrained chassis.

Metrics:

- Active observation success.
- Extra path length needed for observability.
- Planning latency.
- Trajectory feasibility.

Expected result:

The same perception-aware risk topology can be used across platforms, while MOCHA handles platform-specific constraints.

## 7. Ablation Plan

Ablations should be designed so each contribution can be defended independently.

Recommended variants:

- Full P-MOCHA.
- Without multi-homotopy candidates.
- Without persistent risk thresholds.
- Without occlusion shadow risk.
- Without active observation gain.
- Without visibility-aware yaw.
- Risk-only RAST-like corridor.
- Original MOCHA/DYNUS baseline.

The two most important ablations are:

- Full method vs risk-only.
- Persistent multi-homotopy vs single threshold/single corridor.

If those two are strong, the paper story is solid.

## 8. Implementation Order

Recommended order:

1. Create scenario scripts for static occlusion and hidden dynamic obstacle.
2. Add a risk/uncertainty/occlusion field adapter.
3. Add candidate path scoring without touching the optimizer.
4. Generate multiple candidate guides/corridors.
5. Add visibility/frontier/occlusion exposure metrics.
6. Run first ablation: risk-only vs active observation candidate selection.
7. Add persistent multi-threshold labels.
8. Integrate selected candidates into MOCHA low-dimensional optimization.
9. Add differentiable perception costs if the nonlinear optimizer supports them cleanly.
10. Run full closed-loop replanning experiments.

This order is important because it prevents over-investing in optimizer modifications before proving that active observation changes behavior.

## 9. Near-Term Coding Tasks

First coding milestone:

- Add a standalone module for sampled trajectory evaluation:

```cpp
struct TrajectoryPerceptionScore {
  double risk = 0.0;
  double uncertainty = 0.0;
  double occlusion_shadow = 0.0;
  double visibility_gain = 0.0;
  double total = 0.0;
};
```

- Input:

```text
sampled path or PieceWisePol
current octomap
dynamic obstacle predictions
frontier/occlusion points
sensor model
```

- Output:

```text
score and debug metrics
```

Second coding milestone:

- Wrap multiple guide paths:

```cpp
struct PerceptionHomotopyCandidate {
  vec_Vecf<3> path;
  std::vector<LinearConstraint3D> corridor;
  TrajectoryPerceptionScore score;
  std::string risk_signature;
  std::string visibility_signature;
};
```

Third coding milestone:

- Modify global planning selection:

```text
generate K candidates -> score each -> choose best -> pass to existing local optimizer
```

Only after these are working should we modify the MOCHA objective.

## 10. Paper Contribution Draft

Contribution 1:

**Persistent Perception-Risk Homotopy.**

We construct multi-threshold risk-topological signatures from dynamic risk and perception uncertainty, enabling stable multi-homotopy guide generation under noisy perception.

Contribution 2:

**Occlusion-Aware Active Observation Objective.**

We model occluded unknown regions as future interaction risk and define visibility/information gain costs that encourage the robot to reveal safety-critical uncertainty earlier.

Contribution 3:

**Low-Dimensional Perception-Aware MOCHA Optimization.**

We embed risk, uncertainty, occlusion, and observability costs into MOCHA's low-dimensional trajectory parameterization, preserving real-time optimization while coupling motion feasibility with future observability.

## 11. Main Risk and Fallback

The main risk is trying to make everything differentiable too early.

Fallback strategy:

1. Publishable version can use multi-homotopy candidate generation and active observation scoring, then optimize the selected candidate with MOCHA.
2. Stronger version adds differentiable perception costs inside MOCHA.
3. Full version adds closed-loop persistent topology update and high-frequency low-dimensional re-optimization.

This gives a safe development path:

```text
candidate scoring -> multi-homotopy selection -> MOCHA integration -> differentiable active perception
```

## 12. Current Recommendation

Start with:

> Multi-homotopy candidate generation plus active observation scoring in occluded dynamic scenes.

Do not start by rewriting the optimizer.

The first convincing result should show:

- Two or more feasible homotopy paths.
- The shortest/risk-only path has worse future visibility.
- The active observation path reveals the hidden risk earlier.
- Collision/emergency-replan rate drops under hidden dynamic obstacles.

Once this is demonstrated, the mathematical story and the code path both become clear.
