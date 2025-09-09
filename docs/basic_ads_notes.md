---
title: "Basic Autonomous Driving (ADS) Notes"
description: "Primer on autonomous driving stack: sensing, perception, prediction, planning, control, safety, maps."
tags: [ads, primer, autonomous-driving]
last_reviewed: 2025-09-09
version: 0.1.0
---

This primer summarizes the layered architecture of modern autonomous driving systems. For robotics fundamentals see: [Basic Robotics Notes](basic_robotics_notes.md).

---

## 1. High-Level Stack Layers

1. Sensing & Ingestion (LiDAR, camera, radar, IMU, GNSS, HD map).
2. Perception (detection, tracking, freespace, drivable area, traffic lights, signs, lanes).
3. Prediction (agent intent / trajectory distribution estimation).
4. Localization & HD Mapping (multi-sensor fusion, map alignment, lane-level positioning).
5. Planning (behavior, tactical, trajectory generation with constraints/comfort/safety).
6. Control (longitudinal + lateral: MPC, Stanley, Pure Pursuit, LQR).
7. System / Runtime (data recording, monitoring, failover, safety supervisors).

## 2. Key Open / Reference Stacks

| Stack | Focus | Notes |
|-------|-------|-------|
| Autoware (Universe) | Open-source full stack | ROS 2 based, modular pipelines |
| Apollo (Baidu) | Production oriented | Extensive HD map & prediction modules |
| CARLA + ROS Bridge | Simulation + integration | Scenario generation & sensor realism |
| LGSVL / SVL Simulator | Scenario simulation | Co-sim with planning modules |

## 3. Sensor Modalities & Fusion

| Modality | Role | Challenges |
|----------|------|-----------|
| LiDAR | 3D structure, obstacles | Weather susceptibility, cost |
| Camera | Semantics, color, lane markings | Lighting variation |
| Radar | Velocity, robustness in weather | Lower spatial resolution |
| IMU | High-frequency motion | Bias drift |
| GNSS | Global position | Urban canyon multipath |
| HD Map | Lane geometry, regulatory features | Change detection / staleness |

Fusion tiers:

- Time synchronization & extrinsic calibration.
- Ego-motion via LiDAR-inertial odometry or visual-inertial + GNSS constraints.
- Probabilistic occupancy & semantic layers (BEV fusion).

## 4. Perception Tasks

- 3D object detection (anchor-free, transformer BEV approaches).
- Multi-object tracking (track-by-detect, joint detection-tracking transformers).
- Lane & drivable area segmentation (BEV encoders, top-view lifting from multi-camera rigs).
- Traffic signal / sign recognition.
- Free space & dynamic occupancy grid generation.

## 5. Bird's-Eye View (BEV) Transformation

Core for unifying multi-camera + LiDAR into structured grid for downstream detection, prediction, and planning. Methods: inverse perspective mapping, Lift-Splat-Shoot, BEVFormer (temporal attention), sparse voxel encoders.

## 6. Prediction Layer

Outputs multi-modal future trajectories with probability weights for surrounding agents. Common approaches:

- Kinematic + heuristic baselines.
- Recurrent / graph neural networks encoding agent interactions.
- Transformer-based scene-token attention (agent-lane-object tokens).
- Diffusion or generative models for diverse hypothesis sets.

Evaluation: minADE, minFDE, miss rate @ horizon, calibrated probability (Brier score).

## 7. Planning Hierarchy

| Layer | Horizon | Function |
|-------|---------|----------|
| Behavioral | ~5–15 s | High-level maneuver (lane change, yield, stop) |
| Tactical | ~3–6 s | Path segment selection, gap acceptance |
| Trajectory | ~1–4 s | Continuous, jerk/comfort/constraints optimized |

Algorithms: Hybrid A*, lattice planners, graph search over lane graph, polynomial (quintic / jerk-minimizing) curves, MPC with dynamic & comfort constraints.

## 8. Safety & Risk Assessment

- Threat assessment (time-to-collision, time-to-react).
- RSS (Responsibility Sensitive Safety) formal constraints.
- Rulebook layering: traffic rules → comfort → efficiency.
- Redundancy: independent perception checkers, fallback deceleration policy.

## 9. Control Strategies

Longitudinal: PID vs model predictive speed control (speed limit, curvature adaptation).
Lateral: Stanley, Pure Pursuit, MPC (coupled lateral-longitudinal), LQR.
Feedforward + feedback blending; actuator delay compensation; state estimation smoothing.

## 10. HD Maps & Localization

- Lanelets, junction topology, speed restrictions, traffic light associations.
- Map-relative pose via LiDAR reflectivity intensity matching, feature scan matching, or semantic localization.
- Change detection pipeline to flag stale segments (delta LiDAR intensity / occupancy drift).

## 11. Simulation Ecosystem

- CARLA scenario DSL (traffic actors, weather, time-of-day).
- Co-simulation with control loops and sensor models.
- Edge cases: occluded pedestrians, rare emergency vehicles, unusual weather.

## 12. Metrics & KPIs

| Category | KPI |
|----------|-----|
| Safety | Collision rate, disengagements/km |
| Compliance | Traffic rule violation count |
| Comfort | Longitudinal/lateral jerk stats |
| Efficiency | Average speed vs speed limit, progress rate |
| Prediction | minADE/minFDE, miss rate |
| Planning | Constraint violation count |

## 13. System Engineering

- Data logging (ring buffer + trigger events).
- Online model monitoring (distribution drift alerts on embeddings / detections).
- Fail-operational vs fail-safe design; degraded mode (reduced speed).
- Over-the-air update pipeline & rollback strategy.

## 14. Edge Compute & Acceleration

- Multi-sensor RDMA ingestion, GPU BEV fusion, TensorRT inference.
- Workload partition across SoC (CPU scheduling isolation for real-time threads).
- Thermal & power budgeting (DVFS impacts latency stability).

## 15. Open Challenges

- Long-tail rare events modeling.
- Robustness under severe weather & sensor occlusion.
- Calibration drift detection & auto-correction.
- Multi-agent coordination & V2X integration.

## 16. Research / Expansion Tasks

| Task | Goal | Priority |
|------|------|---------|
| Benchmark BEV transformer variants on scenario set | Model selection | High |
| Integrate RSS constraint checker into planner loop | Formal safety | High |
| Add drift monitoring for camera extrinsics | Reliability | Medium |
| Prediction calibration analysis pipeline | Risk estimation | Medium |
| Rare event synthetic data generation harness | Long-tail coverage | Medium |

## 17. Cross-links to Robotics Notes

| ADS Topic | Robotics Reference |
|-----------|--------------------|
| BEV fusion & mapping | Mapping & World Models deep dive |
| GPU inference & transport | Hardware Acceleration deep dive |
| Planning hierarchy | Planning & Manipulation deep dive |
| Simulation scenario generation | Simulation deep dive |
| Multimodal grounding (VLM) | World & VLM Models deep dive |

## 18. References

- Autoware Universe documentation
- Apollo open platform papers
- CARLA simulator & scenario runner docs
- Lanelet2 map framework
- RSS safety papers
- BEVFormer / Lift-Splat-Shoot / scene transformer papers
