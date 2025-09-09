<!-- markdownlint-disable-file -->
# Task Research Documents: Robotics Foundations (Systems, Terms, World + VLM Models)

Comprehensive, evidence-grounded survey of core modern robotics software stacks (simulation, middleware, acceleration toolchains), reinforcement learning platforms, ROS 2 distributions (Humble, Jazzy), spatial/world representation standards, and emergent Vision-Language(-Action) / foundation models impacting perception, planning, and manipulation. This document will guide curation of `docs/basic_robotics_notes.md` toward an actionable, succinct primer while retaining deeper references here.

## Table of Contents
- [Scope and Success Criteria](#scope-and-success-criteria)
- [Outline](#outline)
  - [Potential Next Research](#potential-next-research)
- [Research Executed](#research-executed)
  - [File Analysis](#file-analysis)
  - [Code Search Results](#code-search-results)
  - [External Research (Evidence Log)](#external-research-evidence-log)
  - [Project Conventions](#project-conventions)
- [Key Discoveries](#key-discoveries)
  - [Project Structure](#project-structure)
  - [Implementation Patterns](#implementation-patterns)
  - [Complete Examples](#complete-examples)
  - [API and Schema Documentation](#api-and-schema-documentation)
  - [Configuration Examples](#configuration-examples)
- [Technical Scenarios](#technical-scenarios)
  - [Curating Core Robotics Systems Section](#curating-core-robotics-systems-section)
  - [Adding ROS 2 Distribution Clarifications](#adding-ros-2-distribution-clarifications)
  - [Expanding World Models and VLM Section](#expanding-world-models-and-vlm-section)
  - [Standardizing Terminology Glossary](#standardizing-terminology-glossary)

## Scope and Success Criteria
- Scope: Deep dive into items listed (Isaac Sim, Isaac Lab, ROS, Isaac ROS, Humble, Jazzy) plus missing foundational systems (Gazebo (Ignition), Webots, MoveIt, Nav2, SLAM stacks, simulation standards: URDF, SDF, USD, OpenXR, RL frameworks, world model concepts, VLM/foundation perception & action models). Clarify ROS 2 distro names (Humble Hawksbill, Jazzy Jalisco). Survey key world model & VLM/robotics-aligned models (FoundationPose, RT-X family, RT-2, PaLM-E (if publicly documented), OpenVLA, Voyager (LMM-based autonomy), VLA for manipulation). Excludes: Low-level motor control firmware specifics, hardware electronics design, non-public proprietary internal model architectures.
- Assumptions: 1) Repository intends concise learning notes; deeper research belongs in `.copilot-tracking/research/`. 2) No existing code dependencies yet—notes are textual. 3) Emphasis on currently maintained / influential tools as of Sept 2025.
- Success Criteria:
  - Enumerated, categorized systems with purpose, typical use, integration relationships.
  - Clear mapping from ROS 2 distro nicknames to full names + lifecycle relevance.
  - Glossary of essential robotics terms (perception, localization, mapping, planning, control, sim, RL) with crisp definitions.
  - Curated list of world model & VLM/foundation models relevant to robotics with differentiators.
  - Actionable recommendation for updating `basic_robotics_notes.md` (structure + content deltas) without overloading that file.

## Outline
1. Systems Landscape
   - Simulation: Isaac Sim, Gazebo (Harmonic), Webots, Mujoco, Unity Robotics, CARLA (autonomy), Habitat-Sim.
   - Middleware & Frameworks: ROS 2 core, DDS layer, Isaac ROS (acceleration), MoveIt, Nav2, micro-ROS (edge MCUs).
   - RL & Training: Isaac Lab, RLlib, Stable Baselines3, Habitat RL, Mujoco + Brax, NVIDIA Warp for physics acceleration (contextual mention).
   - Planning & Manipulation: MoveIt 2, cuMotion, FoundationPose, motion planners (OMPL, TrajOpt, CHOMP, STOMP, RRT*, MPPI, MPC). 
2. ROS 2 Distributions & Naming
3. Data & World Representation
   - Robot description formats: URDF, Xacro, SDF, USD (Omniverse), TF frames, semantic scene graphs, occupancy grids, voxel TSDF/ESDF (nvBlox), point clouds, OctoMap.
4. Key Terms Glossary
5. World Models & Vision-Language(-Action) Models for Robotics
6. Recommended File Update Plan

### Potential Next Research
- Hardware acceleration pipelines (NITROS vs zero-copy ROS 2 intra-process)  
  - **Reasoning**: Could refine performance advice section later.  
  - **Reference**: Isaac ROS docs excerpt in evidence log.
- Comparative benchmarks of simulators (fidelity vs throughput)  
  - **Reasoning**: Implementation decisions may require speed vs realism trade-offs.  
  - **Reference**: Simulation section placeholder.

## Research Executed

### File Analysis
- docs/basic_robotics_notes.md  
  - Contains skeletal bullet list for systems; lacks definitions for Humble & Jazzy, empty sections for terms and world/VLM models.

### Code Search Results
- (Repository minimal; no internal code referencing robotics terms yet.)

### External Research (Evidence Log)
- Isaac ROS / Sim / Lab (NVIDIA docs)  
  - Key features: GPU-accelerated GEMs, NITROS pipelines, visual SLAM, nvBlox, cuMotion, FoundationPose, simulation integration for testing (Docs 2025).  
  - Source: NVIDIA Isaac documentation (overview & starter kits) accessed 2025-09-09.
- ROS 2 Humble Documentation  
  - Provides conceptual documentation and distribution context.  
  - Source: ROS 2 Humble docs (docs.ros.org) accessed 2025-09-09.
- ROS 2 Jazzy Jalisco Release Notes  
  - Supported platforms, integration changes with Gazebo (Harmonic), new features across packages (rosbag2 service recording, improved image_transport QoS).  
  - Source: ROS 2 Jazzy docs accessed 2025-09-09.

### Project Conventions
- Standards referenced: `.github/copilot-instructions.md` (priority rules, research doc location).  
- Instructions followed: Research confined to `.copilot-tracking/research/` per task; no code modifications elsewhere.

## Key Discoveries

### Project Structure
Minimal repo; opportunity to keep main notes concise and link to deeper research.

### Implementation Patterns
Not applicable yet (no code). Future: Could add sample ROS 2 package layout or simulator launch files.

### Complete Examples
```text
Proposed note structure snippet:
Systems & Simulation
  - Isaac Sim – High-fidelity USD-based PhysX simulation with RTX; headless for scale.
  - Gazebo (Harmonic) – Modular open simulator; recommended pairing with ROS 2 Jazzy.
  - Webots – Education & rapid prototyping simulator.
```

### API and Schema Documentation
- ROS 2: Node graph (topics, services, actions, parameters, QoS) – refer to Concepts pages (Humble/Jazzy).  
- Scene: USD vs URDF mapping (Isaac Sim consumes USD; bridging tools convert URDF → USD).  
- Motion planning: MoveIt 2 plugin interface for planners; cuMotion integrates as accelerated planner plugin.

### Configuration Examples
```xml
<!-- ROS 2 package.xml dependency example for Gazebo vendor package (Jazzy) -->
<depend>gz_math_vendor</depend>
```

## Technical Scenarios

### Curating Core Robotics Systems Section
Modern robotics education benefits from grouping by functional layer (Simulation, Middleware, Acceleration, Planning, Perception, RL). Existing list mixes them loosely.

#### Requirements
- Reclassify existing bullets into categories.  
- Add missing widely-used systems (Gazebo, Webots, MoveIt 2, Nav2, SLAM (ORB-SLAM / VSLAM), Localization (AMCL), Mapping (nvBlox, OctoMap), Data recording (rosbag2), Visualization (RViz2)).  
- Keep descriptions 1–2 concise lines in main doc; longer details deferred here.

#### Preferred Approach (Selected)
Categorize by stack layer for cognitive map; ensures scalability without clutter. Alternatives (alphabetical list, chronological) lose pedagogical layering.

```text
Systems Section (target structure)
1. Simulation & Digital Twins
2. Middleware & Graph
3. Perception & Mapping
4. Planning & Control
5. Learning & Policy Training
6. Deployment & Acceleration
```

##### Preferred Approach - Technical Requirements
- Each category ≤10 bullets.  
- Primary tool per concept first (e.g., MoveIt 2 for manipulation).  
- Provide distribution-specific note only where material (Gazebo Harmonic pairing with Jazzy).

##### Preferred Approach - Implementation Details
Simulation & Digital Twins
```text
Isaac Sim – RTX + PhysX USD-based simulation; supports synthetic data + domain randomization.
Gazebo (Harmonic) – Open modular sim; integrates with Jazzy via vendor packages.
Webots – Fast iteration; controller API multi-language support.
Mujoco – High-performance dynamics; research RL benchmark.
Habitat-Sim – Embodied AI photorealistic nav & manipulation research.
CARLA – Autonomous driving simulation (sensors, traffic). (Optional inclusion)
```

Middleware & Graph
```text
ROS 2 – Pub/sub, services, actions, parameters, QoS on DDS layer.
micro-ROS – ROS 2 API subset for MCU-class devices.
Isaac ROS – GPU-accelerated perception & transport (NITROS) on ROS 2.
```

Perception & Mapping
```text
FoundationPose – 6D novel object pose foundation model.
nvBlox – Real-time dense TSDF → ESDF + costmaps.
Visual SLAM (Isaac ROS Visual SLAM / ORB-SLAM3) – Localization + mapping.
OctoMap – Probabilistic occupancy mapping.
```

Planning & Control
```text
MoveIt 2 – Manipulation planning (OMPL, plugins, grasp pipelines).
cuMotion – GPU trajectory optimization, parallel candidate evaluation.
Nav2 – Navigation stack (behavior trees, costmaps, planners, controllers).
Trajectory optimization frameworks: OMPL (sampling), CHOMP / STOMP (gradient), RRT* (asymptotic optimal), MPC / MPPI (model predictive control).
```

Learning & Policy Training
```text
Isaac Lab – Parallel RL pipeline leveraging Isaac Sim.
RLlib / Stable Baselines3 – General RL libraries used with simulators.
Domain Randomization – Technique to bridge sim2real; integrated in Isaac Sim.
```

Deployment & Acceleration
```text
NITROS – Zero-copy GPU transport adaptation layer (Isaac ROS).
TensorRT – Optimized inference runtime.
rosbag2 – Data logging & playback (service recording from Jazzy onward).
RViz2 – Visualization & debugging.
```

##### Preferred Approach - Important Changes
- Introduce layered categories; remove “??” placeholders by replacing with full distro names + purpose.

### Adding ROS 2 Distribution Clarifications
Humble Hawksbill (LTS, 2022) vs Jazzy Jalisco (2024) naming; highlight support window & associated recommended sim (Harmonic for Jazzy). Provide note to always specify full distro when referencing features because APIs differ.

### Expanding World Models and VLM Section
World model in robotics: Unified representation fusing geometry (maps), semantics (labels), affordances (action possibilities), temporal updates. Increasingly linked to large multimodal models enabling language-conditioned planning.

Models / Families (selection for 2023–2025 relevance):
- FoundationPose – 6D pose for novel objects (Isaac ROS).  
- RT-1 / RT-2 / RT-X lineage – Vision-Language-Action models mapping instructions to low-level actions (Google/Everyday Robots lineage; RT-2 adds web-scale vision-language pretraining).  
- PaLM-E – Embodied multimodal language model joint training on robot + vision tokens.  
- OpenVLA – Open-source VLA baseline for instruction following.  
- Voyager (Minecraft autonomy) – Continual skill acquisition via LLM planning (conceptual reference for autonomous skill expansion).  
- Scene graphs (spatial-semantic world models) – Graph nodes (objects, rooms) edges (relations, containment).  
- 3D Gaussian Splatting / Nerf-based scene representations – Emerging fast radiance field recon aiding sim & perception alignment.  
- nvBlox TSDF/ESDF + costmap pipeline – Metric + traversability world layer.  
- Multi-modal perception backbones (e.g., SAM for segmentation, CLIP for semantic grounding) as components inside larger world modeling pipelines.

Comparison Axes (to surface in condensed notes): Task generality, embodiment grounding, action space type (end-effector vs high-level), data modality fusion, openness (open vs proprietary), sim2real bridging mechanisms.

### Standardizing Terminology Glossary
Core robotics pipeline stages with crisp definitions targeted for main doc (short form). Extended definitions reside here:
```text
Perception – Extracting state estimates & semantics from sensor data.
Localization – Estimating robot pose within a map/world frame.
Mapping – Building/updating a world representation (metric or semantic).
SLAM – Simultaneous Localization And Mapping concurrently.
Planning – Computing feasible, often optimized trajectories or task sequences.
Control – Converting planned trajectories into actuator commands ensuring stability and tracking.
Actuation – Physical execution by motors/servos.
World Model – Unified, updatable representation combining geometry, semantics, dynamics, and affordances.
Digital Twin – High-fidelity synchronized simulation of a real-world system.
Domain Randomization – Systematic variation in sim to improve real-world robustness.
Trajectory Optimization – Continuous optimization of path under constraints & cost function.
Sampling-based Planning – Exploring configuration space via random samples (RRT*, PRM) to find feasible paths.
Costmap – 2D/3D grid encoding obstacle & traversal cost, used by local/global planners.
Pose Estimation – Determining 6D position + orientation of objects/robot.
Affordance – Possible actions suggested by object geometry/context.
Zero-Copy Transport – Message passing without buffer duplication, reducing latency (e.g., NITROS pipelines).
Intra-Process Communication – Optimized message passing within a single process leveraging shared memory.
```

## Research Tools and Methods
Internal: minimal repository; external evidence via official vendor docs. Stored only distilled, verified facts; no speculative implementation.

## Recommended Update Plan for `basic_robotics_notes.md`
1. Replace existing bullet list with categorized sections (Simulation & Digital Twins, ROS & Middleware, Perception & Mapping, Planning & Control, Learning & Acceleration).  
2. Expand “Humble” and “Jazzy” to full names; add one-line context (LTS / release window).  
3. Populate “Important terms” with short glossary subset.  
4. Populate “World and VLM models” with 5–8 representative entries (FoundationPose, RT-2/RT-X, PaLM-E, OpenVLA, nvBlox (world layer), Scene Graphs).  
5. Keep file concise (< ~120 lines) and link to deeper research doc path.

```text
docs/basic_robotics_notes.md (planned delta)
- Systems section reorganized w/ categories
- Added missing core tools (Gazebo, MoveIt 2, Nav2, nvBlox, rosbag2, RViz2)
- Added ROS 2 distro clarifications
- Added concise glossary (≈12 terms)
- Added world/VLM list with brief descriptors
- Added reference link to research doc
```

## Open Questions / Next Steps
- Whether to include autonomous driving simulators (CARLA) in baseline notes? (Decision pending scope).  
- Depth of coverage for RL frameworks (list vs examples).  
- Include hardware (Jetson Orin modules) or keep software-only?  

## Single Selected Approach Summary
Adopt layered categorization with concise entries in main notes; keep expanded context in this research file for maintainability and future expansion.

---
Document date: 2025-09-09  
Research file path: `.copilot-tracking/research/20250909-robotics-foundations-research.md`
