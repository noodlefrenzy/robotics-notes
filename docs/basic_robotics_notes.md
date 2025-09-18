---
title: "Basic Robotics Notes"
description: "Primer on modern robotics stack: simulation, middleware, perception, planning, learning, acceleration."
tags: [robotics, primer]
last_reviewed: 2025-09-09
version: 0.1.0
---

# This file is a concise primer. For deeper context see:
`../.copilot-tracking/research/20250909-robotics-foundations-research.md`

---
## 1. Simulation & Digital Twins

- **Isaac Sim** – NVIDIA RTX / PhysX USD-based high-fidelity simulator; supports headless scaling & synthetic data.
- **Gazebo (Harmonic)** – Modular open-source simulator recommended alongside ROS 2 Jazzy.
- **Webots** – Lightweight educational & prototyping simulator.
- **Mujoco** – High-performance dynamics engine popular in RL research.
- **Habitat-Sim** – Photorealistic embodied AI navigation/manipulation research simulator.

## 2. ROS & Middleware Layer

- **ROS 2** – Middleware (pub/sub, services, actions, QoS, parameters) atop DDS.
- **micro-ROS** – ROS 2 for microcontrollers (resource-constrained endpoints).
- **Isaac ROS** – GPU-accelerated perception & transport (NITROS pipelines, GEMs) building on ROS 2.

### ROS 2 Distributions Mentioned

- **Humble Hawksbill ("Humble")** – Earlier stable LTS-era release (still widely deployed in industry labs).
- **Jazzy Jalisco ("Jazzy")** – 10th ROS 2 release (2024); refined Gazebo integration & expanded tooling (e.g., service recording in rosbag2).

## 3. Perception, Mapping & World Representation

- **FoundationPose** – 6D novel object pose estimation foundation model (Isaac ROS component).
- **nvBlox** – Real-time TSDF→ESDF + costmap generation for navigation & planning.
- **Visual SLAM (e.g., Isaac ROS Visual SLAM / ORB-SLAM3)** – Localization + mapping.
- **OctoMap** – Probabilistic 3D occupancy mapping.
- **Scene Graphs** – Semantic relational world structure (objects, rooms, affordances).

## 4. Planning & Control

- **MoveIt 2** – Manipulation planning framework (OMPL + plugins, grasp pipeline).
- **cuMotion** – GPU-accelerated trajectory optimization running parallel candidate solves.
- **Nav2** – ROS 2 navigation stack (behavior trees, planners, controllers, costmaps).
- **Trajectory / Sampling Planners** – OMPL (sampling), CHOMP/STOMP (gradient/optimization), RRT* (asymptotic optimal), MPC / MPPI (predictive control).

## 5. Learning & Policy Training

- **Isaac Lab** – Parallel RL on Isaac Sim (multi-environment scalable training + domain randomization hooks).
- **RL Frameworks (Refs)** – RLlib, Stable Baselines3 (SB3), Habitat RL, Brax (for large-scale physics); see deep dive file.

## 6. Deployment, Tooling & Acceleration

- **NITROS** – Type adaptation & zero-copy GPU transport layer in Isaac ROS.
- **TensorRT** – Inference optimization runtime for NVIDIA GPUs.
- **rosbag2** – Data recording & playback (Jazzy adds service recording/playback & QoS enhancements).
- **RViz2** – 3D visualization & debugging of ROS 2 graphs and data streams.

## 7. Core Terms (Mini Glossary)

- **Perception** – Deriving semantic & geometric understanding from sensor data.
- **Localization** – Robot pose estimation in a known or evolving map.
- **Mapping** – Building/updating a representation of the environment.
- **SLAM** – Simultaneous Localization and Mapping.
- **Planning** – Computing a feasible (often optimized) action/trajectory sequence.
- **Control** – Executing trajectories while ensuring stability & tracking.
- **World Model** – Unified geometric + semantic + affordance representation.
- **Digital Twin** – High-fidelity synchronized simulation of the real system.
- **Domain Randomization** – Systematic sim variation to improve sim→real transfer.
- **Costmap** – Grid encoding obstacle & traversal costs used by planners.
- **Pose Estimation** – 6D position & orientation retrieval for robot or object.
- **Zero-Copy Transport** – Message passing without buffer duplication.

## 8. World & Vision-Language(-Action) / Foundation Models

- **FoundationPose** – Foundation 6D pose for novel objects.
- **RT-1 / RT-2 / RT-X lineage** – Vision-Language-Action models mapping natural instructions to low-level or mid-level robot actions.
- **PaLM-E** – Embodied multimodal language model integrating sensory + language tokens.
- **OpenVLA** – Open-source baseline VLA for instruction-conditioned manipulation/navigation.
- **nvBlox (World Layer)** – Metric + traversability fused representation powering navigation.
- **Scene Graph Pipelines** – Semantic + relational structure enabling task reasoning.
- **NeRF / 3D Gaussian Splatting** – Neural / splat-based radiance & geometry fields for photorealistic reconstruction assisting perception alignment.

## 9. Related Deep Dives

Created separate deep dive files:

- [Hardware Acceleration & Transport](robotics_deep_dive_hardware_acceleration.md)
- [RL Frameworks & Training Pipelines](robotics_deep_dive_rl_frameworks.md)
- [World, Vision-Language(-Action) & Foundation Models](robotics_deep_dive_models_world_vlm.md)
- [Simulation & Digital Twins](robotics_deep_dive_simulation.md)
- [Planning & Manipulation](robotics_deep_dive_planning_manipulation.md)
- [Control Theory & Controllers](robotics_deep_dive_control_theory.md)
- [Mapping & World Models](robotics_deep_dive_mapping_world_models.md)
- [Spot Platform & SDK](robotics_deep_dive_spot_platform.md)

---
Related domain: Autonomous Driving – see [Basic ADS Notes](basic_ads_notes.md) for perception→planning pipelines, with cross-links back to these robotics foundations.


