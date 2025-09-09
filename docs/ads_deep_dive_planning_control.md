---
title: "ADS Deep Dive: Planning & Control"
description: "Behavioral, tactical, trajectory planning and vehicle-level control with uncertainty & safety."
tags: [ads, deep-dive, planning, control]
last_reviewed: 2025-09-09
version: 0.1.0
---

*Back to ADS primer: [Basic ADS Notes](basic_ads_notes.md)*  
> Robotics foundational context: [Basic Robotics Notes](basic_robotics_notes.md)

---

## 1. Scope

End-to-end path from behavioral decision to trajectory optimization and vehicle-level actuation for autonomous driving.

## 2. Layered Planning Recap

| Layer | Function | Horizon | Representative Methods |
|-------|----------|---------|------------------------|
| Behavioral | Maneuver choice (lane change, yield) | 5–15 s | Rule sets, finite state machines, RL |
| Tactical | Lane / gap selection, path branch | 3–6 s | Graph search on lane graph, decision trees |
| Trajectory | Continuous feasible path | 1–4 s | Polynomial splines, MPC, lattice sampling |

## 3. Behavioral Planning

- Finite state machine with guard conditions (speed, relative distance, route).
- Rulebook layering (traffic law → safety → efficiency).
- RL / imitation hybrid approaches for nuanced merges & yields.

## 4. Tactical Path Selection

- Lane graph abstraction (Lanelet2) with connectivity & regulation attributes.
- Gap acceptance modeling (lead/lag vehicle time headway).
- Conflict zone reasoning at intersections (priority, right-of-way encoding).

## 5. Trajectory Generation

| Method | Pros | Cons | Notes |
|--------|------|------|------|
| Lattice sampling + selection | Diverse candidates | May miss optima | Good with cost tuning |
| Polynomial/Jerk Minimizing | Smooth comfort | Limited obstacle adaptation | Use for base curve |
| MPC (linearized / nonlinear) | Constraint handling | Solve time & tuning | Real-time feasibility gating |
| Sampling-based (Hybrid A*) | Handles kinematics | Larger compute | Parking / low-speed |
| Optimization (SQP/ILQR) | High-quality smoothness | Sensitive to init | Use warm starts |

## 6. Cost Function Design

Components: safety (distance, TTC), comfort (jerk, accel), legality (speed limit, lane adherence), progress, energy, risk (uncertainty-weighted).
Weight tuning with data-driven inverse optimization or Bayesian optimization.

## 7. Constraints & Feasibility

- Kinematic bicycle or dynamic vehicle model (slip constraints, curvature limits).
- Actuator latency compensation (preview horizon shift).
- Soft vs hard constraints (slack variables for mild violation tolerance).

## 8. Uncertainty Handling

- Inflate obstacles by predicted covariance (ellipsoidal expansion).
- Risk-sensitive objective: CVaR weighting on high-cost outcomes.
- Multi-trajectory contingency planning (primary + evasive).

## 9. Control Layer

| Aspect | Options | Notes |
|--------|--------|-------|
| Lateral | Stanley, Pure Pursuit, MPC, LQR | Curvature vs path tracking trade-offs |
| Longitudinal | PID, Adaptive Cruise (gap control), MPC | Comfort vs responsiveness |
| Combined | Nonlinear MPC | Coupled optimization |

Feedforward term from curvature / desired acceleration + feedback error correction.

## 10. Actuation & Execution

- Rate limiting & jerk filtering.
- Command arbitration with safety supervisor (emergency braking override).
- Health monitoring (actuator saturation, divergence detection).

## 11. Fallback & Degraded Modes

- Safe stop trajectory generation when critical faults (sensor loss, localization uncertainty spike).
- Reduced-speed cautious mode (tightened safety cushions).
- Manual takeover request logic & HMI signaling.

## 12. Metrics & KPIs

| Category | Metric |
|----------|--------|
| Safety | Min distance, collision-free episodes |
| Comfort | Jerk RMS, lateral acceleration |
| Efficiency | Average velocity / legal limit ratio |
| Accuracy | Lateral/longitudinal path error |
| Robustness | Replan latency, success under disturbances |

## 13. Failure Modes

| Failure | Cause | Mitigation |
|---------|------|-----------|
| Oscillation in lateral controller | Gain mismatch / delay | Retune, add feedforward, latency compensation |
| Planner stall in dense traffic | Over-constrained cost | Adaptive relaxation, rule-based escape |
| Unsafe lane change | Prediction uncertainty ignored | Risk-aware filtering |
| Jerk spikes | Poor spline stitching | Overlap smoothing, continuity constraint |
| Late braking | Latency underestimation | Explicit delay model in MPC |

## 14. Research / Expansion Tasks

| Task | Goal | Priority |
|------|------|---------|
| Add CVaR risk-weighted planner variant | Safety robustness | High |
| MPC vs lattice hybrid comparative benchmark | Performance trade study | High |
| Adaptive comfort weighting based on passenger state | UX optimization | Medium |
| Multi-contingency branch planner prototype | Fault response | Medium |
| Automated cost weight tuning pipeline | Reduced manual labor | Medium |

## 15. References

- Lanelet2 framework
- Hybrid A* & lattice planning literature
- MPC design for autonomous vehicles
- Stanley/Pure Pursuit controller papers
- Risk-aware planning & CVaR references
