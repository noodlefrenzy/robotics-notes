---
title: "Robotics Deep Dive: World, Vision-Language(-Action) & Foundation Models"
description: "Integration of world models, scene graphs, VLM/VLA, pose and multimodal grounding for robotics."
tags: [robotics, deep-dive, world-models, vlm, multimodal]
last_reviewed: 2025-09-09
version: 0.1.0
---

# Back to primer: [Basic Robotics Notes](basic_robotics_notes.md)

---
## 1. Scope

Unifies geometric world representations (maps, TSDF/ESDF, scene graphs) with semantic & multimodal foundation models (VLM/VLA) enabling instruction-following and generalized manipulation/navigation.

## 2. Representation Layers

| Layer | Example | Purpose |
|-------|---------|---------|
| Metric | TSDF / ESDF (nvBlox) | Collision + traversability |
| Occupancy | OctoMap | Probabilistic free/occ space |
| Semantic | Segmentation masks, labels | Object category grounding |
| Relational | Scene Graph | Object-object / affordance reasoning |
| Neural Radiance | NeRF / Gaussian Splatting | Photoreal rendering, view synthesis |
| Policy-Conditioned | Language tokens fused with perception | Task grounding |

## 3. Key Models & Families

- FoundationPose – Zero-shot 6D object pose.
- RT-1 / RT-2 / RT-X – Instruction to action tokenization evolution (scaling data mixture).
- PaLM-E – Multimodal language model embedding vision & proprioception.
- OpenVLA – Open baseline for VLA tasks.
- CLIP-like Embedders – Vision-text alignment for retrieval & grounding.
- SAM / Segment Anything – Universal segmentation proposals.

## 4. Integration Patterns

1. Scene Graph Augmentation: Periodic fusion of detection + pose into relational graph.
2. Language Query Routing: Parse instruction → retrieve relevant nodes (objects/locations) → generate action plan.
3. Multi-Stage Control: High-level language policy → mid-level skill library → low-level controllers.
4. Pose Estimation Assist: FoundationPose bootstraps grasp or manipulation frame selection.

## 5. Evaluation Dimensions

| Dimension | Example Metric |
|-----------|----------------|
| Grounding Accuracy | Object/reference resolution success |
| Instruction Fidelity | Task completion vs natural language spec |
| Generalization | Novel object category performance |
| Embodied Success | Multi-step plan completion rate |
| Latency | Instruction→first action delay |

## 6. Safety & Reliability

- Hallucination filtering: Cross-check language model outputs with world state graph (entity existence).
- Affordance validation: Reject actions violating kinematic / safety constraints.
- Uncertainty propagation: Maintain confidence scores for pose & detection entries.

## 7. Data & Annotation Strategy

- Synthetic data generation (Isaac Sim domain randomization) feeding CLIP/SAM fine-tunes.
- Weak supervision: Web-scale captions + robot interaction logs.
- Active dataset expansion: Query failures → targeted collection.

## 8. Caching & Performance

- Embedding cache for frequent instruction templates.
- Graph indices (spatial hash, semantic label inverted index).
- On-device quantized vision encoders; offload large language backbone to edge server if necessary.

## 9. Open Challenges

- Temporal grounding across long-horizon tasks.
- Continual adaptation without catastrophic forgetting.
- Alignment of geometric uncertainty with language abstractions.
- Real-time large-model inference under power constraints.

## 10. Research / Expansion Tasks

| Task | Goal | Priority |
|------|------|---------|
| Add uncertainty-aware scene graph prototype | Robust planning | High |
| Benchmark instruction latency across model sizes | Deployment tuning | Medium |
| Evaluate RT-2 vs OpenVLA on household task set | Model selection | Medium |
| Integrate SAM proposals into object persistence module | Recall boost | Medium |
| Implement hallucination guard with entity existence checks | Safety | High |

## 11. References

- RT-1 / RT-2 / RT-X papers
- PaLM-E paper
- OpenVLA repository/docs
- FoundationPose resources
- CLIP & SAM original papers
- Scene graph & NeRF survey literature
