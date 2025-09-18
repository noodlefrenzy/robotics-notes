---
title: "Robotics Deep Dive: Control Theory"
description: "From classical loops to optimal, robust, nonlinear, adaptive, and learning-augmented control for autonomous robotic systems."
tags: [robotics, deep-dive, control, optimal, robust]
last_reviewed: 2025-09-18
version: 0.1.0
---

*Back to primer: [Basic Robotics Notes](basic_robotics_notes.md)*

---

## Summary (TL;DR)

Purpose: Stabilize or track trajectories while respecting physical limits, uncertainty, and safety constraints.

Core Families (one-liners):

- PID / Cascaded PID: Fast-to-tune baseline for single loops or loosely coupled axes.
- State Feedback (Pole Placement + Observer): Systematic eigenvalue shaping for linearized MIMO plants.
- LQR / TV-LQR: Infinite-horizon quadratic optimal (regulation / trajectory tracking) with minimal runtime cost.
- MPC (Linear / Nonlinear): Finite-horizon optimization adding explicit constraints & multi-objective weighting; heavier compute.
- Robust (H∞, μ, Sliding Mode): Performance guarantees under bounded worst-case disturbances/model uncertainty.
- Nonlinear (Feedback Linearization, Backstepping, Passivity, CLF): Structure-exploiting designs for broader operating envelopes.
- Safety Layer (CBF / CLF-CBF-QP, Reachability): Enforce forward invariance of safe sets; wrap around nominal controller.
- Adaptive (MRAC, Gain Scheduling, Disturbance Observer): Online parameter or residual adaptation for drift / scheduling.
- Learning-Augmented: Residual dynamics models, policy proposals filtered by safety (CBF/MPC), iterative refinement (ILC).

Quick Selection Guide:

- Simple regulation, no tight constraints → PID → (if coupling significant) LQR.
- Tight actuator/state constraints or multi-objective trade-offs → MPC.
- Large param uncertainty / disturbance rejection focus → Robust (H∞) or disturbance observer + LQR/MPC.
- Strong nonlinearities / underactuation (quadrotor full pose, legged COM) → Geometric / TV-LQR / Nonlinear MPC.
- Safety-critical sets (keep-out zones, collision margins) → Nominal controller + CBF filter or constrained MPC.
- Repetitive precision tasks → Add ILC or feedforward learned residual.
Layering Pattern:
Planner/Trajectory → (TV-LQR or MPC tracker) → Inner rate/torque loops → Safety (CBF / limits) → Fault & logging.

Common Pitfalls & Mitigations:

- Saturation & windup → Anti-windup schemes; include limits in MPC.
- Poor scaling → Normalize states before choosing Q, R.
- Model mismatch → Add disturbance observer or learned residual; enlarge R for robustness.
- Constraint infeasibility (MPC) → Soft constraints + priority weighting; fallback LQR.
- Chattering (sliding/CBF switching) → Add smoothing, relax barrier margin.
Metrics Snapshot: Stability margins, RMS tracking error, constraint violation count, control energy, barrier slack distribution.

Read further sections for depth, design equations, and architecture details.

## 1. Scope

Survey of control paradigms for robotic systems (manipulation, locomotion, mobile, aerial) spanning classical feedback, modern state-space, optimal (LQR / MPC), robust (H_infinity), nonlinear (feedback linearization, passivity), safety (control barrier functions), adaptive, and learning-based integration. Emphasis on practical selection, trade-offs, and architectural integration.

## 2. Control Stack Placement

| Layer | Examples | Purpose |
|-------|----------|---------|
| Planning / Trajectory Gen | RRT*, CHOMP, iLQR (trajectory), cuMotion | Produce feasible reference path/state sequence |
| Tracking Controller | PID, LQR, MPC, Geometric, Inverse Dynamics | Drive system along reference |
| Low-Level Actuation | Motor current loops, impedance control | Enforce torque/force commands |
| Safety / Override | CBF, emergency stop, envelope protection | Ensure constraint & safety adherence |

## 3. Fundamentals

- Feedback: Use measured state/output to correct deviations; improves robustness vs open-loop.
- Stability (Lyapunov): All trajectories converge to equilibrium (asymptotic) or remain bounded (Lyapunov / ISS notions).
- Performance Trade Space:
  - Transient: rise time, overshoot, settling time.
  - Steady-state error: offset / bias.
  - Robustness: gain & phase margins, param uncertainty tolerance.
  - Effort / energy: actuator usage, thermal limits.
  - Safety: constraint adherence, fail-safe transitions.
- Linearization: Jacobian expansion of nonlinear dynamics around equilibrium / trajectory for local controller synthesis (TV-LQR, gain scheduling).

## 4. Classical Control (Loop-Shaping)

- PID (P, PI, PD, full): Proportional error shaping + integral bias removal + derivative damping.
- Lead / Lag Compensation: Adjust phase/gain to meet margins & bandwidth.
- Frequency-Domain Tools: Bode, Nyquist, Nichols for SISO design & robustness visualizations.
- Strengths: Low computational cost, intuitive tuning, minimal modeling.
- Limitations: Weak for strong coupling, MIMO interactions, constraints.

## 5. Modern State-Space Control

- State Feedback: u = -K x to place closed-loop poles (Ackermann for controllable SISO; numerical methods for MIMO).
- Observers: Luenberger observer; chooses gains for estimation error dynamics. Separation principle for linear time-invariant systems (design estimator + regulator independently).
- Controllability / Observability: Kalman rank tests; drive design feasibility.
- Advantages: Handles MIMO coupling, systematic pole assignment, integrates with observers.

## 6. Optimal Control (Deterministic)

### 6.1 LQR (Regulation)

- Solve Algebraic Riccati Equation for P; u = -K x minimizes J = ∫ (x^T Q x + u^T R u) dt.
- Provides systematic tuning via Q, R weighting state vs control effort.

### 6.2 LQT / TV-LQR (Tracking / Time-Varying)

- Linearize about nominal trajectory; solve time-varying Riccati backward; apply u = u_ref - K(t)(x - x_ref).

### 6.3 MPC (Model Predictive Control)

- Finite horizon optimization with constraints:
  - Minimize Σ x_k^T Q x_k + u_k^T R u_k + x_N^T P x_N subject to dynamics & constraints.
  - Provides constraint handling (state, input, rate) & multi-objective weighting.
- Terminal ingredients: Choose P, K from LQR for stability, add terminal set.
- Variants: Nonlinear MPC (direct transcription), Economic MPC, Learning-based warm starts.

### 6.4 Trajectory Optimization & iLQR/DDP

- Iteratively optimize nonlinear trajectories via local quadratic approximations of cost + dynamics (Differential Dynamic Programming second-order variant).
- Produces both feedforward and time-varying feedback gains; bridges planning-control boundary.

## 7. Robust Control

| Method | Idea | When Useful |
|--------|------|-------------|
| Loop-Shaping (H_infinity norm bounds) | Minimize worst-case gain from disturbance to output | Disturbance attenuation critical |
| H_infinity State-Space | Solve Riccati inequalities for performance guarantees | Structured uncertainty, performance specs |
| μ-Synthesis | Structured singular value minimization | Parametric, repeated uncertainties |
| Sliding Mode | Discontinuous control to enforce invariance & robustness | Matched disturbances, high certainty bounds |

- Trade-off: Robust margin vs nominal performance (conservatism). Hybrid approach: nominal MPC + robust backup.

## 8. Nonlinear Control

### 8.1 Feedback Linearization

- Cancel nonlinearities via exact model inversion → yields linear closed-loop if model exact.
- Sensitive to model mismatch; regularize with robust/adaptive augmentation.

### 8.2 Passivity & Energy Methods

- Enforce energy shaping + damping injection (e.g., robot impedance control). Naturally stable and intuitive for physical interaction tasks.

### 8.3 Lyapunov / Control Lyapunov Functions (CLF)

- Construct V(x) > 0 with derivative ˙V <= -α||x||^2; synthesize control via optimization (e.g., CLF-QP blending stabilization + constraints).

### 8.4 Backstepping

- Recursive virtual control design for strict-feedback structures (common in underactuated aerial/marine systems).

## 9. Safety & Constraint Enforcement

### 9.1 Control Barrier Functions (CBF)

- Define h(x) >= 0 safe set; enforce ˙h(x,u) >= -α h(x) via QP to keep invariance.
- Combine with CLF (stabilization) in single CLF-CBF Quadratic Program.

### 9.2 Set Invariance / Reachability

- Compute forward reachable sets for worst-case state envelopes; schedule constraint tightening (MPC robust tubes).

### 9.3 Emergency / Fallback Layers
- Supervisory arbitration logic: override commanded torque when approaching envelope (velocity, temperature, proximity). Can be simple finite state runtime monitor.

## 10. Adaptive & Learning-Augmented Control

| Approach | Mechanism | Notes |
|----------|----------|-------|
| MRAC (Model Reference Adaptive Control) | Adjust parameters to drive plant toward reference model response | Requires persistence of excitation |
| Gain Scheduling | Interpolate controllers across operating points | Needs coverage map & smooth blending |
| Iterative Learning Control (ILC) | Use previous trial error to improve feedforward next trial | Repetitive tasks (pick-place, cyclic motion) |
| Disturbance Observers / DOB | Estimate lumped disturbance; cancel feedforward | Enhances robustness without full robust design |
| Learning Residual Models | Train Δf(x,u) to correct nominal model online | Pair with MPC for improved prediction |
| Policy + Safety Filter | RL policy proposed action filtered by CBF/MPC | Guarantees constraints while exploring |

- Data-Driven Robustness: Distribution shift mitigated via uncertainty quantification (ensembles) + risk-sensitive MPC (CVaR).

## 11. Hybrid Force / Impedance Control (Interaction)

- Impedance: Desired dynamic relation F = K(x - x_ref) + B(ẋ - ẋ_ref) + M(¨x - ¨x_ref).
- Hybrid Position/Force: Partition task space (tangent vs normal directions) controlling position and force concurrently (Peg-in-hole insertion etc.).
- Passivity-based designs ensure stable energy exchange with environment.

## 12. Architecture & Implementation Concerns

- Loop Rates: Inner (1–2 kHz torque/current), mid-level (100–500 Hz joint/attitude), outer (10–100 Hz task/trajectory), planning (1–10 Hz or event-driven).
- Latency Budgeting: Sensing→estimate→plan→control; timestamp & predict forward (state propagation) to compensate.
- Discretization: Zero-order hold assumptions; choose sample time T << dominant system time constant (≥ 10× faster).
- Actuator Saturation: Implement anti-windup (integral clamping, back-calculation), consider saturation in MPC constraints.
- Numerical Stability: Use stable Riccati solvers, QP warm starts, pre-conditioning for poorly scaled states.
- Fault Detection: Residual generation (observer mismatch), statistical change detection for sensor drift.
- Logging & Telemetry: Closed-loop stability analysis via eigenvalue estimates, frequency response (chirp injection during commissioning).

## 13. Selecting a Control Strategy

| Scenario | Recommended Baseline | Upgrade Path |
|----------|---------------------|--------------|
| Simple joint regulation | PI / PID | LQR (coupled), MPC (constraints) |
| Quadrotor hover & small maneuvers | Cascaded PID | LQR / geometric SE(3) control |
| Mobile robot trajectory tracking | PID on (v, ω) | TV-LQR / MPC |
| Manipulator with joint limits & collision constraints | LQR + saturation | MPC (task space), CLF-CBF-QP |
| Bipedal locomotion stabilization | Foot placement heuristics + PD | Hybrid zero dynamics + TV-LQR / MPC |
| Contact-rich assembly | Impedance | Hybrid force-position + learning correction |
| High-speed autonomous racing | PID steering | Nonlinear MPC + learned tire model residual |

## 14. Metrics & Validation

| Category | Metric | Description |
|----------|--------|-------------|
| Stability | Pole / eigenvalue locations | Negative real parts (CT) / inside unit circle (DT) |
| Tracking | RMS error, max deviation | Reference adherence |
| Robustness | Gain & phase margins; param sweep | Sensitivity to uncertainty |
| Constraint | Violation count / min margin | Safety adherence |
| Efficiency | Control energy ∑ u^T u | Actuator usage |
| Safety | Near-miss distance, barrier slack | Proximity to unsafe set |
| Learn-Augmented | Generalization error | Performance on shifted conditions |

## 15. Failure Modes

- Integrator Windup: Leads to overshoot & slow recovery after saturation.
- Model Drift: Learned residual stale → prediction degradation.
- Poor Scaling: Ill-conditioned Q, R causing numerical Riccati instability.
- Constraint Infeasibility (MPC): Leads to fallback or stale command hold.
- Chattering (Sliding / CBF-QP switching): Induces actuator wear; add smoothing / hysteresis.
- Observer Divergence: Unmodeled sensor bias accumulation.
- Latency-Induced Instability: Networked or distributed control loops missing deadlines.

## 16. Research / Expansion Tasks

| Task | Goal | Priority |
|------|------|---------|
| Implement CLF-CBF-QP layer atop existing LQR | Safety-constrained stabilization | High |
| Add learned dynamics residual into MPC model | Reduce prediction error | High |
| Benchmark TV-LQR vs NMPC for mobile base | Performance vs compute trade | Medium |
| Integrate disturbance observer for arm joints | Improved robustness | Medium |
| Evaluate risk-sensitive (CVaR) MPC | Tail-risk mitigation | Low |

## 17. References

1. Slotine & Li, "Applied Nonlinear Control"
2. Khalil, "Nonlinear Systems"
3. Bryson & Ho, "Applied Optimal Control"
4. Rawlings, Mayne, Diehl, "Model Predictive Control: Theory, Computation, and Design"
5. Skogestad & Postlethwaite, "Multivariable Feedback Control"
6. Ogata, "Modern Control Engineering"
7. Dean et al., "Guaranteeing Safety in Learning-Based Control" (robust learning)
8. Ames et al., "Control Barrier Functions" (safety filtering)
9. Isidori, "Nonlinear Control Systems"
10. Wie, "Space Vehicle Dynamics and Control" (aerospace attitude)
11. Levine, "Reinforcement Learning and Control" (survey)
12. Recht, "A Tour of Reinforcement Learning" (system-centric RL view)
13. Tassa et al., "Control-Limited Differential Dynamic Programming"
14. Mayne, "Nonlinear MPC: Status and Challenges"
15. Freeman & Kokotovic, "Backstepping Design"
16. Spong, Hutchinson, Vidyasagar, "Robot Modeling and Control"
17. Ames et al., CLF-CBF-QP formulations (various papers)
18. Anderson & Moore, "Optimal Control: Linear Quadratic Methods"
19. Bertsekas, "Dynamic Programming and Optimal Control"
20. Qin & Badgwell, "A Survey of Industrial MPC" (practical adoption)

---

See also: `robotics_deep_dive_planning_manipulation.md` (trajectory optimization interplay) and `robotics_deep_dive_rl_frameworks.md` (learning-based policy training integration).
