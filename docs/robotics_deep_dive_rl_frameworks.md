---
title: "Robotics Deep Dive: RL Frameworks & Training Pipelines"
description: "Comparison and best practices for Isaac Lab, RLlib, SB3, Habitat RL, Brax in robotics policy training."
tags: [robotics, deep-dive, reinforcement-learning, training]
last_reviewed: 2025-09-09
version: 0.1.0
---

*Back to primer: [Basic Robotics Notes](basic_robotics_notes.md)*

---
## 1. Scope

Covers reinforcement learning ecosystems used with robotics simulators and real-to-sim loops: Isaac Lab, RLlib, Stable Baselines3, Habitat RL, Brax, plus integration patterns with ROS 2.

## 2. Framework Comparison

| Framework | Strengths | Weaknesses | Typical Use |
|-----------|-----------|-----------|-------------|
| Isaac Lab | GPU-parallel sim, domain randomization hooks | NVIDIA stack dependency | High-fidelity robot policy training |
| RLlib | Distributed (ray) scaling, algorithm zoo | More overhead for small jobs | Large scale hyperparam sweeps |
| SB3 | Simplicity, readable ref impls | Limited distributed tooling | Rapid prototyping / baselines |
| Habitat RL | Embodied nav semantics | Focused domain | Instructional nav & embodied AI |
| Brax | JIT (XLA) large batched physics | Simplified physics fidelity | Massive parallel coarse training |

## 3. Policy Architecture Patterns

- Vision → Encoder (CNN/Vision Transformer) → Latent → Policy + Value heads.
- Language-conditioned: Text encoder (e.g., T5 / ViT-L) fused with visual tokens → cross-attn → action head.
- Hybrid: State (proprioception) concatenated with latent embeddings.

## 4. Data & Curriculum Strategies

1. Progressive task difficulty (waypoint distance shrink, grasp object variety scale).
2. Domain Randomization axes: textures, lighting, dynamics params, sensor noise, kinematics jitter.
3. Mixed offline + online RL (behavior cloning warm start + PPO fine-tuning).
4. Replay buffers stratified by success/failure or novelty (RND-based prioritization).

## 5. Sim-to-Real Transfer Checklist

- Dynamics parameter sweep coverage documented.
- Sensor noise injection approximates empirical variance.
- Latency & actuation delay modeled (temporal jitter augmentation).
- Policy frozen & tested with hardware-in-the-loop (HIL) before full deployment.

## 6. Evaluation Metrics

| Category | Metrics |
|----------|---------|
| Task Success | Success rate, completion time |
| Efficiency | Energy per episode, distance overhead |
| Robustness | Performance under perturbation set |
| Safety | Collision count, joint limit violations |
| Generalization | Unseen object/scene success |

## 7. Distributed Training Topologies

- Parameter server (legacy) vs. all-reduce (NCCL) synchronous PPO.
- Asynchronous actors with central learner (IMPALA / Ape-X style) for throughput.
- Replay multiplexing across tasks (multi-task latent distillation).

## 8. Safety & Constraints

- Action projection layer enforcing joint/velocity limits.
- Shielded RL: backup model predictive controller overrides unsafe outputs.
- Reward shaping anti-patterns: over-penalizing exploration, implicit sparse reward collapse.

## 9. Logging & Experiment Management

Tools: Weights & Biases, MLflow, ClearML. Essential artifacts: config yaml, git commit hash, environment randomization seed manifest, reward component breakdown.

## 10. Failure Modes

- Reward hacking (proxy exploitation) → Add invariance tests.
- Overfitting to synthetic textures → Increase photoreal variation / real image fine-tuning.
- Catastrophic forgetting in continual updates → Elastic weight consolidation or rehearsal buffers.

## 11. Real-Time Considerations

Inference latency budget vs. control loop period (e.g., 10 ms @100Hz). Use TorchScript / TensorRT conversion for deployment; ensure deterministic ops where required.

## 12. Research / Expansion Tasks

| Task | Goal | Priority |
|------|------|----------|
| Compare PPO vs. SAC on grasp robustness | Algorithm selection | High |
| Integrate language-conditioned policy baseline | Multimodal expansion | Medium |
| Add shielded RL layer prototype | Safety validation | High |
| Domain randomization coverage report generator | Transfer QA | Medium |
| Investigate Brax pretraining + Isaac fine-tune | Hybrid scaling | Low |

## 13. References

- Isaac Lab papers / docs
- OpenAI PPO, SAC, IMPALA foundational papers
- RLlib documentation
- Habitat challenge reports
- Sim-to-real survey literature
