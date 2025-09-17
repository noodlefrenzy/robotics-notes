---
title: "Robotics Deep Dive: Planning & Manipulation"
description: "Sampling, optimization, GPU acceleration, behavior trees, grasp synthesis and constraint handling."
tags: [robotics, deep-dive, planning, manipulation]
last_reviewed: 2025-09-09
version: 0.1.0
---

*Back to primer: [Basic Robotics Notes](basic_robotics_notes.md)*

---
## 1. Scope

Covers motion & task-level planning for mobile manipulators: sampling, optimization, GPU-accelerated planning, behavior trees, grasp synthesis, and constraint handling.

## 2. Taxonomy

| Layer | Examples | Purpose |
|-------|----------|---------|
| Task Sequencing | Behavior Trees, HTN, PDDL planners | High-level goal decomposition |
| Motion Planning | OMPL (RRT*, PRM*), cuMotion, CHOMP, STOMP | Feasible/optimized trajectories |
| Trajectory Optimization | MPC, MPPI, ILQR | Refinement & dynamic feasibility |
| Grasp Planning | Dex-Net, GQ-CNN, geometric heuristics | End-effector pose selection |
| Control | PID, Model-based, Learned policies | Execution tracking |

## 3. GPU-Accelerated Planning (cuMotion)

- Parallel candidate generation & optimization.
- Collision checking acceleration with distance fields (nvBlox ESDF integration).
- Latency benefits for reactive re-planning in dynamic scenes.

## 4. Behavior Trees (Nav2 & Beyond)

- Reactive fallback & recovery nodes.
- Blackboard state pattern for context sharing.
- Determinism concerns: guard concurrency & racey condition checks.

## 5. Constraints & Kinematics

- Redundant manipulator null-space optimization (secondary objectives: joint limits, manipulability maximization).
- Time-parameterization (TOTG vs Ruckig) smoothing velocity/acceleration profiles.
- Collision avoidance via signed distance field gradient costs.

## 6. Grasp & Manipulation Pipelines

1. Perception (pose estimation / segmentation) → grasp candidate generation.
2. Filter candidates (reachability, IK feasibility, collision).
3. Score (quality metrics / learned grasp success models).
4. Motion plan to pre-grasp → approach → closure → retreat.

## 7. Dynamic & Unstructured Environments

- Replanning triggers: obstacle insertion, costmap delta threshold, human detection.
- Time-budgeted planning: anytime algorithms returning progressively better paths.
- Safety buffer inflation scaling with velocity.

## 8. Metrics

| Category | Metric |
|----------|--------|
| Efficiency | Path length, execution time |
| Smoothness | Jerk, acceleration norms |
| Safety | Min distance to obstacles, collision count |
| Success | Plan feasibility rate, grasp success |
| Reactivity | Replan latency |

## 9. Failure Modes

- IK singularity trapping.
- Local minima in gradient-based optimization (CHOMP) on cluttered scenes.
- Stale costmaps causing collision in execution.
- Behavior tree infinite retry loops.

## 10. Research / Expansion Tasks

| Task | Goal | Priority |
|------|------|---------|
| Integrate ESDF-based gradient costs into planner | Improved clearance | High |
| Benchmark cuMotion vs OMPL RRTConnect | Latency comparison | Medium |
| Add adaptive inflation layer experiment | Dynamic safety | Medium |
| Implement anytime partial-refinement pipeline | Reactivity | High |
| Compare Ruckig vs TOTG smoothing on jerk metrics | Kinematic quality | Low |

## 11. References

- MoveIt 2 docs
- cuMotion technical briefs
- OMPL planning algorithms
- Behavior tree design patterns
- Ruckig time parameterization
- See also: `robotics_deep_dive_spot_platform.md` for mission sequencing and locomotion navigation interplay (GraphNav + foothold planning abstractions).
