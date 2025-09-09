---
title: "Documentation Index"
description: "Navigation hub for robotics & ADS primers and deep dives."
tags: [index, navigation, docs]
last_reviewed: 2025-09-09
version: 0.1.0
---

Central navigation hub for robotics and autonomous driving knowledge base.

---

## Primers

- [Basic Robotics Notes](basic_robotics_notes.md) – Core robotics layers.
- [Basic Autonomous Driving (ADS) Notes](basic_ads_notes.md) – Driving-specific stack.

## Robotics Deep Dives

- [Hardware Acceleration & Transport](robotics_deep_dive_hardware_acceleration.md)
- [RL Frameworks & Training Pipelines](robotics_deep_dive_rl_frameworks.md)
- [World & VLM / Foundation Models](robotics_deep_dive_models_world_vlm.md)
- [Simulation & Digital Twins](robotics_deep_dive_simulation.md)
- [Planning & Manipulation](robotics_deep_dive_planning_manipulation.md)
- [Mapping & World Models](robotics_deep_dive_mapping_world_models.md)

## ADS Deep Dives

- [Perception](ads_deep_dive_perception.md)
- [Prediction](ads_deep_dive_prediction.md)
- [Planning & Control](ads_deep_dive_planning_control.md)

## Cross-Domain Link Themes

| Theme | Robotics Source | ADS Source |
|-------|-----------------|-----------|
| GPU Acceleration | Hardware Acceleration deep dive | BEV fusion & perception (ADS perception deep dive) |
| Mapping to Planning | Mapping & World Models | BEV fusion + Planning & Control |
| Multimodal / VLM | World & VLM Models | ADS perception + future expansion |
| Simulation Strategy | Simulation deep dive | ADS primer (simulation ecosystem) |
| Planning Hierarchies | Planning & Manipulation | ADS Planning & Control |

## Research Tracking

Full foundational research log: `../.copilot-tracking/research/20250909-robotics-foundations-research.md`

## Suggested Next Extensions

1. Add front-matter metadata (tags, version, last-reviewed date).
2. Introduce benchmarking template (latency, accuracy, resource usage).
3. Add glossary centralization (dedupe between primers).
4. Create CONTRIBUTING guide for documentation standards.

---
Last updated: 2025-09-09
