---
title: "Robotics Deep Dive: Mapping & World Models"
description: "Occupancy, TSDF/ESDF, semantic & relational fusion into actionable world models."
tags: [robotics, deep-dive, mapping, world-models]
last_reviewed: 2025-09-09
version: 0.1.0
---

*Back to primer: [Basic Robotics Notes](basic_robotics_notes.md)*

---
## 1. Scope

Explores mapping layers (occupancy, TSDF/ESDF, semantic, relational) and their fusion into actionable world models for navigation, manipulation, and instruction grounding.

## 2. Layered Mapping Stack

| Layer | Example | Function |
|-------|---------|----------|
| 2D Occupancy | Costmap2D, OctoMap projected | Local navigation |
| 3D Occupancy | OctoMap | Sparse probabilistic structure |
| TSDF | nvBlox TSDF | Surface reconstruction |
| ESDF | nvBlox ESDF | Distance queries for planning |
| Semantic Voxels | Labeled TSDF / voxel grids | Category-aware planning |
| Relational Graph | Scene graph | Object affordances & context |

## 3. Fusion Pipelines

1. Depth / LiDAR → TSDF integration (voxel hashing).
2. TSDF → ESDF incremental update (wavefront / sweeping algorithms).
3. Semantic overlay: project segmentation masks onto voxels.
4. Graph augmentation: object node insertion with pose & uncertainty.

## 4. Performance Considerations

- Voxel size trade-off: resolution vs memory vs update latency.
- Hash map vs dense grid vs octree (cache coherence, random access cost).
- GPU residency (nvBlox) enabling real-time ESDF updates for reactive planners.

## 5. Uncertainty & Probabilistic Handling

- Occupancy log-odds update rules & clamping.
- Pose graph uncertainty propagation into voxel update weighting.
- Confidence decay for stale observations (temporal forgetting).

## 6. Scene Graph Integration

- Node schema: {id, class, pose, bbox, affordances, confidence}.
- Edge types: spatial adjacency, containment, support, interaction history.
- Update triggers: new detection, tracking lost, significant pose drift.

## 7. Quality Metrics

| Metric | Purpose |
|--------|---------|
| Surface RMSE vs ground truth mesh | Reconstruction fidelity |
| ESDF query latency | Planner responsiveness |
| Semantic IoU | Label accuracy |
| Graph consistency score | Structural integrity |
| Memory footprint (MB) | Resource budgeting |

## 8. Failure Modes

- Dynamic objects polluting static map (need temporal filtering / classification).
- Loop closure shifts causing misaligned fused volumes.
- Overconfidence accumulation (lack of decay) -> stale obstacles.
- Semantic drift from mis-segmentation cascades into planning.

## 9. Research / Expansion Tasks

| Task | Goal | Priority |
|------|------|----------|
| Implement dynamic object masking pipeline | Reduce map corruption | High |
| Add ESDF latency benchmarking harness | Performance tracking | Medium |
| Prototype semantic + ESDF joint planner | Context-aware routing | Medium |
| Integrate uncertainty-weighted voxel updates | Robustness | Medium |
| Scene graph temporal decay model | Maintain relevance | Low |

## 10. References

- nvBlox documentation
- OctoMap papers
- Probabilistic robotics (Thrun et al.)
- Scene graph survey literature
- See also: `robotics_deep_dive_spot_platform.md` for an applied example of topological (GraphNav) + local terrain modeling integration.
