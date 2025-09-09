---
title: "ADS Deep Dive: Prediction"
description: "Multi-agent trajectory forecasting, interaction modeling, uncertainty calibration and evaluation."
tags: [ads, deep-dive, prediction, forecasting]
last_reviewed: 2025-09-09
version: 0.1.0
---

*Back to ADS primer: [Basic ADS Notes](basic_ads_notes.md)*  
> Robotics foundational context: [Basic Robotics Notes](basic_robotics_notes.md)

---

## 1. Scope

Covers multi-agent future behavior estimation: trajectory set forecasting, intent classification, interaction modeling, probabilistic distribution representation, and evaluation.

## 2. Problem Formulations

| Formulation | Description | Notes |
|-------------|-------------|-------|
| Deterministic | Single best future trajectory | Lacks multimodality |
| Multi-modal Set | K candidate trajectories with probabilities | Supports branch behaviors |
| Prob. Distribution | Continuous density over future states | Hard to evaluate directly |
| Intent Classification + Conditional Trajectories | Discrete maneuver + conditioned forecast | Hierarchical clarity |
| Joint Scene Forecast | All agents simultaneously | Captures interaction coupling |

## 3. Feature Encoders

- Agent-centric temporal encoders (GRU/LSTM/Temporal CNN).
- Scene context: lane graph GNNs (LaneGCN style).
- BEV transformer embeddings (shared with perception).
- Interaction modeling: pairwise attention or social pooling grids.

## 4. Output Parameterizations

| Type | Example | Pros | Cons |
|------|---------|------|------|
| Discrete Lane-anchored | Offsets along lane centerlines | Structured, map-aware | Lane dependency |
| Gaussian Mixture (per step) | GMM over (x,y) | Probabilistic | Limited sharp modes |
| Polynomial / Spline | Coefficients for basis functions | Smooth | Hard with branching |
| Set of Sampled Paths | Learned distribution sampler | Flexible | Diversity collapse risk |
| Occupancy Flow | Grid with prob & velocity | Rich spatial coverage | Memory & resolution trade-offs |

## 5. Uncertainty & Calibration

- Temperature scaling for probability sets.
- CRPS / Brier score tracking.
- Aleatoric vs epistemic decomposition (MC Dropout, ensembles).

## 6. Interaction Modeling Strategies

| Strategy | Key Idea | Strengths | Considerations |
|----------|---------|-----------|---------------|
| Social Pooling | Spatial grid pooling | Simple spatial reasoning | Rigid discretization |
| Attention (Transformer) | Token-based relations | Flexible, global context | Compute cost |
| Graph Neural Networks | Edge message passing | Structured lane-agent interplay | Edge design complexity |
| Game-Theoretic Layers | Nash / Stackelberg approximations | Strategic behavior modeling | Parameter sensitivity |
| Diffusion-based Joint Modeling | Iterative denoising of scene state | Rich diversity | Inference time |

## 7. Map Integration

- Lane graph extraction (lanelets) → node embeddings (curvature, speed limit).
- Relative coordinate frames (Frenet) for stability.
- Semantic zone labels (crosswalk, intersection box) gating motion priors.

## 8. Metrics & Evaluation

| Metric | Purpose |
|--------|---------|
| minADE | Best trajectory average displacement error |
| minFDE | Final displacement vs ground truth |
| Miss Rate @ horizon | Safety-critical miss detection |
| Calibration error (Brier) | Prob correctness |
| Diversity (pairwise distance) | Mode coverage |
| Overlap penalty | Physical feasibility |

## 9. Failure Modes

| Failure | Cause | Mitigation |
|---------|------|-----------|
| Mode collapse | Loss encourages single trajectory | Diversity regularizers, curriculum |
| Overconfident probabilities | Poor calibration | Temperature scaling, ensembles |
| Lane graph mismatch | Map staleness | Dynamic map validation |
| Interaction neglect | Missing cross-agent features | Add attention / GNN with edges |
| Unrealistic kinematics | Unconstrained outputs | Kinematic feasibility projection |

## 10. Downstream Coupling

- Planner risk weighting using predicted occupancy mass.
- Behavior selection gating by predicted maneuver probabilities.
- Uncertainty propagation into safe corridor generation.

## 11. Research / Expansion Tasks

| Task | Goal | Priority |
|------|------|---------|
| Add diffusion-based joint predictor baseline | Diversity & richness | High |
| Implement calibration monitoring dashboard | Reliability | High |
| Lane-relative coordinate ablation study | Representation choice | Medium |
| Interaction GNN vs Transformer benchmark | Efficiency vs accuracy | Medium |
| Uncertainty-aware planner cost integration | Safety coupling | Medium |

## 12. References

- Waymo Open Motion dataset papers
- Argoverse motion forecasting challenges
- LaneGCN, VectorNet, Motion Transformer papers
- Diffusion-based motion prediction literature
- Calibration & uncertainty quantification references
