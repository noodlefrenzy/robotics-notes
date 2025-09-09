---
title: "ADS Deep Dive: Perception"
description: "End-to-end ADS perception: calibration, fusion, BEV architectures, detection, tracking, occupancy."
tags: [ads, deep-dive, perception, fusion]
last_reviewed: 2025-09-09
version: 0.1.0
---

# Back to ADS primer: [Basic ADS Notes](basic_ads_notes.md)  
> Robotics foundational context: [Basic Robotics Notes](basic_robotics_notes.md)

---

## 1. Scope

Covers end-to-end autonomous driving perception: sensor ingest, calibration, synchronization, 2D/3D detection, tracking, drivable area, lanes, traffic elements, freespace, and BEV fusion architectures.

## 2. Pipeline Overview

| Stage | Subtasks | Outputs |
|-------|----------|---------|
| Ingestion | Time sync, ego-motion preintegration | Aligned sensor packets |
| Calibration | Intrinsic/extrinsic validation & drift check | Updated calibration set |
| Preprocessing | Undistortion, motion compensation, ground removal | Normalized sensor frames |
| Feature Extraction | Camera encoders, LiDAR voxelization, radar spectral features | Multi-modal features |
| Fusion / BEV | Lift-splat, deformable attention, voxel/pillar pooling | Unified BEV tensor |
| Detection & Segmentation | 3D boxes, lanes, drivable area, traffic lights/signs | Structured scene graph |
| Tracking | Data association, motion filtering | Persistent agent tracks |
| Occupancy / Freespace | Dynamic/static grid, freespace mask | Occupancy state |

## 3. Key Model Families

- Voxel / Pillar Networks (VoxelNet, PointPillars).
- Sparse Convolutions (SECOND, CenterPoint variants).
- Transformer BEV (BEVFormer, PETR, StreamPETR).
- Multi-modal LiDAR + Camera (TransFusion, BEVFusion).
- Lane & Topology Extractors (STSU, LaneGCN).
- Dynamic Occupancy Grid Networks (DOCTR, OGC fusion approaches).

## 4. Calibration & Synchronization

- Continuous extrinsic refinement via ICP residual trending.
- Online camera extrinsic drift detection using feature reprojection error distributions (KS test drift triggers).
- Hardware timestamp alignment; fallback interpolation logic for missing stamps.

## 5. Tracking & Data Association

| Approach | Strengths | Weaknesses | Notes |
|----------|-----------|-----------|-------|
| Kalman + Hungarian | Simple / efficient | Fragile in occlusion | Legacy baseline |
| JPDA / MHT | Handles ambiguity | Computationally heavier | Crowded scenes |
| Deep Embedding Association | Strong appearance cues | Needs robust embeddings | Multi-camera synergy |
| Joint Detection-Tracking Transformer | End-to-end | Training complexity | Produces track IDs directly |

Metrics: HOTA, MOTA, IDF1, track fragmentation.

## 6. Occupancy & Freespace

Dynamic grid: probability per cell of occupancy + velocity vectors (flow field).
Multi-sweep LiDAR accumulation + ego-motion compensation.
Incorporate radar Doppler for velocity prior.

## 7. BEV Fusion Strategies

| Strategy | Fusion Level | Pros | Cons |
|----------|-------------|------|------|
| Early (raw projection) | Sensor frame | Max info | Heavy compute |
| Mid (feature-level) | Learned embeddings | Trade-off accuracy & cost | Architecture complexity |
| Late (decision-level) | Object lists | Simple | Loses cross-modal synergy |

## 8. Latency & Throughput Optimization

- Layer fusion + TensorRT int8 calibrations (per-channel).
- Multi-stream scheduling (per sensor class).
- ROI cropping pipeline early to reduce BEV extent.
- Asynchronous radar ingestion (lower frequency) with temporal buffering.

## 9. Failure Modes & Mitigations

| Failure | Cause | Mitigation |
|---------|------|-----------|
| Ghost detections in rain | LiDAR backscatter | Rain filtering + radar corroboration |
| Lane marking loss at night | Low SNR | Temporal lane history & HDR models |
| Track ID switches | Occlusion / association ambiguity | Re-ID embeddings + motion gating |
| Calibration drift | Thermal / mechanical shift | Continuous eval + incremental extrinsic solve |
| BEV aliasing at distance | Sparse feature support | Multi-scale BEV pyramid |

## 10. Evaluation Suite

- Benchmark sets: nuScenes, Waymo, Argoverse, KITTI (legacy), ONCE, Lyft.
- Metrics: mAP (3D & 2D), NDS (nuScenes), mIoU (segmentation), latency P95, occupancy F1.
- Rare-case bucket analysis (night, rain, snow, construction).

## 11. Research / Expansion Tasks

| Task | Goal | Priority |
|------|------|---------|
| Add online extrinsic drift alert module | Reliability | High |
| Benchmark BEVFormer vs BEVFusion on internal scenarios | Model selection | High |
| Integrate dynamic occupancy velocity head | Prediction coupling | Medium |
| Radar-camera early fusion prototype | Occlusion robustness | Medium |
| Long-tail failure analysis automation | Dataset curation | Medium |

## 12. References

- BEVFormer / BEVFusion papers
- CenterPoint & PointPillars
- nuScenes & Waymo benchmarks
- Dynamic occupancy grid literature
- Lane graph extraction studies
