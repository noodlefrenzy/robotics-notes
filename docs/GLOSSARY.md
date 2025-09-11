# Glossary

## General, or Common Terms

- [3D Gaussian Splatting](https://arxiv.org/abs/2308.04079): Real-time scene representation using view-dependent Gaussian primitives for fast rendering.
- [A* Search](https://en.wikipedia.org/wiki/A*_search_algorithm): Best-first graph search using admissible/consistent heuristics to guarantee optimality.
- [Affordance](https://en.wikipedia.org/wiki/Affordance): Action possibility suggested by an object's geometry or context.
- [Aleatoric Uncertainty](https://en.wikipedia.org/wiki/Uncertainty_quantification): Irreducible data noise uncertainty inherent in observations.
- [Brier Score](https://en.wikipedia.org/wiki/Brier_score): Proper scoring rule measuring the accuracy/calibration of probabilistic predictions.
- [Control](https://en.wikipedia.org/wiki/Control_theory): Converting planned trajectories or setpoints into actuator commands while ensuring stability and performance. A common method for feedback control is PID control (Proportional-Integral-Derivative).
- [Digital Twin](https://en.wikipedia.org/wiki/Digital_twin): High-fidelity, continuously synchronized virtual replica of a physical system.
- [Domain Randomization](https://arxiv.org/abs/1703.06907): Systematic variation of simulation assets/physics to improve sim-to-real transfer robustness.
- [Dynamic A* (D*)](https://en.wikipedia.org/wiki/D*): Incremental heuristic search algorithm that efficiently repairs paths when graph costs change (robot replanning).
- [Epistemic Uncertainty](https://en.wikipedia.org/wiki/Uncertainty_quantification): Model uncertainty reducible with additional data.
- [Hardware-in-the-Loop (HIL)](https://en.wikipedia.org/wiki/Hardware-in-the-loop_simulation): Testing methodology embedding real hardware components within a simulated environment.
- [Hungarian Algorithm](https://en.wikipedia.org/wiki/Hungarian_algorithm): Polynomial-time algorithm for optimal assignment in bipartite graphs (data association, tracking).
- [Joint Probabilistic Data Association (JPDA)](https://en.wikipedia.org/wiki/Probabilistic_data_association_filter): Multi-target data association method weighting multiple measurement–track hypotheses.
- [Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter): Recursive Bayesian state estimator for linear Gaussian dynamic systems.
- [Latency](https://en.wikipedia.org/wiki/Latency_(engineering)): Time delay between an input stimulus and system response.
- [Mapping](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping): Building or updating a representation (metric, semantic, relational) of the environment.
- [Model Predictive Control (MPC)](https://en.wikipedia.org/wiki/Model_predictive_control): Receding-horizon optimal control solving a finite-horizon constrained optimization each cycle (see also PID Controller, Trajectory Optimization).
- [Multiple Hypothesis Tracking (MHT)](https://en.wikipedia.org/wiki/Multiple_hypothesis_tracking): Tracking approach maintaining a tree of competing association hypotheses until disambiguated.
- [Neural Radiance Field (NeRF)](https://arxiv.org/abs/2003.08934): Neural scene representation producing view-consistent radiance and density for novel view synthesis.
- [Occupancy Grid](https://en.wikipedia.org/wiki/Occupancy_grid_mapping): Discretized spatial grid with per-cell probability of occupancy.
- [Pose](https://en.wikipedia.org/wiki/Pose_(computer_vision)): Position and orientation of an object or robot in a reference frame.
- [Pose Estimation](https://en.wikipedia.org/wiki/Pose_(computer_vision)): Determination of 6-DoF position and orientation from sensor data.
- [Rapidly-exploring Random Tree (RRT)](https://msl.cs.uiuc.edu/rrt/): Sampling-based motion planning algorithm rapidly covering high-dimensional state spaces (non-optimal baseline variant of RRT*).
- [Reinforcement Learning (RL)](https://en.wikipedia.org/wiki/Reinforcement_learning): Learning control or decision policies via reward-driven interaction with an environment.
- [Scene Graph](https://en.wikipedia.org/wiki/Scene_graph): Graph structure capturing entities and their semantic/spatial relationships.
- [Simultaneous Localization and Mapping (SLAM)](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping): Concurrent estimation of agent pose and environment map.
- [Trajectory](https://en.wikipedia.org/wiki/Trajectory): Time-parameterized sequence of system states or poses.
- [Trajectory Optimization](https://en.wikipedia.org/wiki/Optimal_control): Optimization of a trajectory under dynamics and constraints for objectives like smoothness, energy, risk.
- [Uncertainty (General)](https://en.wikipedia.org/wiki/Uncertainty_quantification): Quantified doubt in estimates (combining aleatoric and epistemic components).
- [Zero-Copy Transport](https://en.wikipedia.org/wiki/Zero-copy): Data transfer without buffer duplication to reduce latency and CPU overhead.

## Robotics Specific Terms

- [Automated Guided Vehicle (AGV)](https://en.wikipedia.org/wiki/Automated_guided_vehicle): Industrial vehicle following fixed guidance (tracks, magnetic tape, QR, or LiDAR reflectors) in structured environments; lacks dynamic free-space path planning of AMRs.
- [Autonomous Mobile Robot (AMR)](https://en.wikipedia.org/wiki/Autonomous_mobile_robot): Mobile robot capable of autonomous navigation in unstructured or dynamic environments using onboard perception, localization, and planning (contrast with fixed-path AGVs).
- [Behavior Tree](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)): Hierarchical task execution graph enabling reactive control flow.
- [CHOMP](https://personalrobotics.cs.washington.edu/publications/rglchomp.pdf): Covariant Hamiltonian Optimization for Motion Planning (gradient-based trajectory optimizer).
- [CUDA Graphs](https://developer.nvidia.com/blog/cuda-graphs/): Captured DAG of CUDA kernel launches reducing per-launch overhead.
- [cuMotion](https://developer.nvidia.com/blog/accelerating-robot-motion-generation-with-gpus/): GPU-parallel trajectory optimization library for manipulators.
- [Euclidean Signed Distance Field (ESDF)](https://ieeexplore.ieee.org/document/6379367): Grid storing signed distance to nearest surface for collision and clearance queries.
- [FoundationPose](https://developer.nvidia.com/blog/foundationpose/): Foundation model for generalizable zero-shot 6D object pose estimation.
- [GPUDirect RDMA](https://developer.nvidia.com/gpudirect): Direct memory access path allowing peer devices to read/write GPU memory without host copies.
- [Inertial Measurement Unit (IMU)](https://en.wikipedia.org/wiki/Inertial_measurement_unit): Sensor measuring linear acceleration & angular velocity for orientation and motion estimation.
- [Intra-Process Communication (ROS 2)](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Intra-Process-Communication.html): Optimized message passing within a process avoiding serialization/copies.
- [Isaac ROS](https://docs.nvidia.com/isaac/isaac-ros): GPU-accelerated ROS 2 package collection (GEMs) incl. NITROS type adaptation.
- [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com): RTX/PhysX USD-based robotics simulation platform.
- [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html): 2024 ROS 2 distribution with expanded tooling and features.
- [Linear Quadratic Regulator (LQR)](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator): Optimal state-feedback controller minimizing quadratic cost.
- [micro-ROS](https://micro.ros.org/): ROS 2 API subset and tooling targeting microcontroller-class devices.
- [Model Predictive Control (MPC)](https://en.wikipedia.org/wiki/Model_predictive_control): Receding-horizon optimization-based control enforcing constraints.
- [Model Predictive Path Integral (MPPI)](https://arxiv.org/abs/1909.04502): Sampling-based stochastic control method using path integral formulation.
- [MoveIt 2](https://moveit.picknik.ai): Motion planning framework integrating kinematics, planning plugins, grasp generation.
- [NITROS](https://docs.nvidia.com/isaac/isaac-ros): Isaac ROS type adaptation enabling GPU-resident zero-copy message passing.
- [OctoMap](https://octomap.github.io): Probabilistic octree-based 3D occupancy mapping framework.
- [Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org): Library of sampling-based motion planning algorithms.
- [PID Controller](https://en.wikipedia.org/wiki/PID_controller): Proportional-integral-derivative feedback law for setpoint tracking.
- [RRT*](https://msl.cs.uiuc.edu/rrt/): Asymptotically optimal sampling-based motion planner variant of RRT.
- [ROS 2](https://docs.ros.org): DDS-based robotics communication framework with pub/sub, services, actions, QoS.
- [rosbag2](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Ros2bag/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html): ROS 2 data recording & playback tool.
- [Ruckig](https://github.com/pantor/ruckig): Online jerk-constrained time-parameterization library for motion generation.
- [STOMP](https://personalrobotics.cs.washington.edu/publications/stomp.pdf): Stochastic Trajectory Optimization for Motion Planning via noisy update refinement.
- [TensorRT](https://developer.nvidia.com/tensorrt): High-performance deep learning inference optimizer/runtime for NVIDIA GPUs.
- [Time-Optimal Trajectory Generation (TOTG)](https://moveit.picknik.ai/main/doc/concepts/time_parameterization/time_parameterization.html): Parameterization minimizing execution time under kinematic limits.
- [Truncated Signed Distance Field (TSDF)](https://ieeexplore.ieee.org/document/6138516): Volumetric grid storing truncated signed distance enabling surface extraction.
- [Visual SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping): Vision-based SLAM leveraging camera data (feature or direct methods).
- [VSLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping): Abbreviation for Visual SLAM; vision-based SLAM leveraging camera data.
- [nvBlox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nv_blox): GPU TSDF→ESDF fusion and 2D costmap generation library.
- [Zero-Copy (ROS 2)](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Intra-Process-Communication.html): Mechanisms (loaned messages/shared memory) avoiding data copies in message passing.

## Autonomous Vehicle Specific Terms

- [BEV Fusion](https://arxiv.org/abs/2203.17270): Multi-modal (camera/LiDAR/radar) feature projection & aggregation into BEV space.
- [BEVFormer](https://arxiv.org/abs/2203.17270): Temporal deformable attention transformer for BEV feature generation.
- [Bird's-Eye View (BEV)](https://arxiv.org/abs/2203.17270): Top-down spatial representation aggregating multi-sensor features.
- [CenterPoint](https://arxiv.org/abs/2006.11275): Anchor-free 3D object detection using center-based representation.
- [Conditional Value at Risk (CVaR)](https://en.wikipedia.org/wiki/Conditional_value_at_risk): Risk metric: expected loss in the worst α-tail; used for risk-sensitive planning.
- [Diffusion-based Motion Prediction](https://arxiv.org/abs/2303.12074): Generative denoising approach producing diverse future trajectories (representative paper).
- [Drivable Area](https://www.nuScenes.org): Segmented region of road surface navigable by the vehicle.
- [Dynamic Occupancy Grid](https://arxiv.org/abs/2111.12717): Time-varying grid modeling occupancy probability and per-cell motion.
- [Fallback (Degraded) Mode](https://www.iso.org/standard/81169.html): Safe reduced-capability operational state after critical fault detection (ISO 26262 / ADS safety context).
- [Higher Order Tracking Accuracy (HOTA)](https://arxiv.org/abs/2009.07736): Tracking metric combining detection, association & localization quality.
- [Hybrid A*](https://doi.org/10.1109/ROBOT.2008.4543489): Search-based planner combining discrete search with continuous state refinement for kinematic feasibility.
- [IDF1](https://motchallenge.net/): Multi-object tracking metric emphasizing identity preservation accuracy.
- [Lane Graph (Lanelet)](https://github.com/fzi-forschungszentrum-informatik/Lanelet2): Graph abstraction of lane segments with connectivity & regulatory attributes.
- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2): Open-source HD lane-level map framework.
- [Lattice Planner](https://ieeexplore.ieee.org/document/5206621): Candidate trajectory generation via structured sampling then cost selection.
- [Lift-Splat-Shoot](https://arxiv.org/abs/2008.05711): Camera feature lifting & splatting method creating BEV features.
- [minADE](https://arxiv.org/abs/2005.04259): Minimum average displacement error across predicted trajectory set vs. ground truth.
- [minFDE](https://arxiv.org/abs/2005.04259): Minimum final displacement error of trajectory endpoints vs. ground truth.
- [Miss Rate](https://arxiv.org/abs/1906.10840): Fraction of objects/trajectories not sufficiently covered by predictions/detections.
- [Mode Collapse (Prediction)](https://arxiv.org/abs/2007.06831): Reduction of multi-modal forecast diversity to a single dominant trajectory.
- [Multiple Object Tracking Accuracy (MOTA)](https://motchallenge.net/): Aggregate multi-object tracking accuracy error metric.
- [nuScenes Detection Score (NDS)](https://www.nuscenes.org): Composite 3D detection benchmark metric.
- [Occupancy Flow](https://arxiv.org/abs/2006.11867): Grid representation predicting future occupancy probability and motion vectors.
- [Polynomial / Jerk-Minimizing Trajectory](https://arc.aiaa.org/doi/10.2514/3.20031): Use of quintic/septic polynomials optimizing smoothness subject to jerk bounds.
- [Prediction (Automated Driving)](https://arxiv.org/abs/2005.04259): Estimation of multi-agent future behaviors and trajectories.
- [Responsibility Sensitive Safety (RSS)](https://arxiv.org/abs/1708.06374): Formal driving safety model defining conservative distance & response rules.
- [Social Pooling](https://arxiv.org/abs/1609.04836): Local interaction feature aggregation for trajectory prediction networks.
- [Time-to-Collision (TTC)](https://en.wikipedia.org/wiki/Collision_avoidance_system): Estimated remaining time before collision on current relative motion.
- [TransFusion](https://arxiv.org/abs/2203.11496): LiDAR-camera transformer fusion for 3D detection.
- [Tracking (MOT)](https://motchallenge.net/): Maintaining consistent identities & state estimates of dynamic agents over time.
- [TTC-based Threat Assessment](https://doi.org/10.1109/ITSC.2013.6728476): Risk scoring using temporal collision proximity.
- [Vehicle Fallback Trajectory](https://www.iso.org/standard/84126.html): Emergency deceleration or controlled stop path under faults (ADS safety design).

## References

1. [ROS 2 Documentation][ref-ros2]
2. [Isaac Sim Documentation][ref-isaac-sim]
3. [Isaac ROS (GEMs / NITROS)][ref-isaac-ros]
4. [MoveIt / Motion Planning Docs][ref-moveit]
5. [Nav2 Documentation][ref-nav2]
6. [OMPL Library][ref-ompl]
7. [RRT* Resources][ref-rrt]
8. [TensorRT / CUDA][ref-tensorrt]
9. [nvBlox Repository][ref-nvblox]
10. [OctoMap][ref-octomap]
11. [Lanelet2][ref-lanelet2]
12. [RSS Paper][ref-rss]
13. [Domain Randomization][ref-domain-rand]
14. [NeRF][ref-nerf]
15. [3D Gaussian Splatting][ref-gauss]
16. [FoundationPose][ref-foundationpose]
17. [HOTA Metric][ref-hota]
18. [nuScenes Dataset][ref-nuscenes]
19. [CenterPoint][ref-centerpoint]
20. [MPPI Control][ref-mppi]
21. [TSDF (KinectFusion)][ref-tsdf]
22. [BEVFormer][ref-bevformer]
23. [Lift-Splat-Shoot][ref-liftsplat]
24. [TransFusion][ref-transfusion]
25. [CVaR][ref-cvar]
26. [LQR Overview][ref-lqr]
27. [Hardware-in-the-Loop][ref-hil]
28. [SLAM / Occupancy][ref-slam]
29. [Kalman Filter][ref-kalman]
30. [Hungarian Algorithm][ref-hungarian]
31. [JPDA][ref-jpda]
32. [MHT][ref-mht]
33. [Brier Score][ref-brier]
34. [Aleatoric/Epistemic Uncertainty][ref-uncertainty]
35. [Zero-Copy][ref-zerocopy]
36. [PID Controller][ref-pid]
37. [Social Pooling][ref-social]
38. [Occupancy Flow][ref-occupancy-flow]
39. [minADE/minFDE][ref-minade]
40. [Mode Collapse (Prediction)][ref-modecollapse]
41. [Diffusion Motion Prediction][ref-diffusion]
42. [Hybrid A*][ref-hybrid-a]
43. [Lattice Planner][ref-lattice]
44. [Polynomial / Jerk-Minimizing Trajectory][ref-jerk-poly]
45. [TTC Threat Assessment][ref-ttc-threat]
46. [Vehicle Fallback Safety (ISO)][ref-vehicle-fallback]
47. [Fallback Mode (ISO 26262)][ref-fallback-mode]
48. [GPUDirect RDMA][ref-gpudirect]
49. [CUDA Graphs][ref-cuda-graphs]
50. [Ruckig][ref-ruckig]
51. [CHOMP][ref-chomp]
52. [STOMP][ref-stomp]
53. [ESDF Mapping][ref-esdf]
54. [A* Search][ref-astar]
55. [Dynamic A* (D*)][ref-dstar]

[ref-ros2]: https://docs.ros.org
[ref-isaac-sim]: https://docs.isaacsim.omniverse.nvidia.com
[ref-isaac-ros]: https://docs.nvidia.com/isaac/isaac-ros
[ref-moveit]: https://moveit.picknik.ai
[ref-nav2]: https://navigation.ros.org
[ref-ompl]: https://ompl.kavrakilab.org
[ref-rrt]: https://msl.cs.uiuc.edu/rrt/
[ref-tensorrt]: https://developer.nvidia.com/tensorrt
[ref-nvblox]: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nv_blox
[ref-octomap]: https://octomap.github.io
[ref-lanelet2]: https://github.com/fzi-forschungszentrum-informatik/Lanelet2
[ref-rss]: https://arxiv.org/abs/1708.06374
[ref-domain-rand]: https://arxiv.org/abs/1703.06907
[ref-nerf]: https://arxiv.org/abs/2003.08934
[ref-gauss]: https://arxiv.org/abs/2308.04079
[ref-foundationpose]: https://developer.nvidia.com/blog/foundationpose/
[ref-hota]: https://arxiv.org/abs/2009.07736
[ref-nuscenes]: https://www.nuscenes.org
[ref-centerpoint]: https://arxiv.org/abs/2006.11275
[ref-mppi]: https://arxiv.org/abs/1909.04502
[ref-tsdf]: https://ieeexplore.ieee.org/document/6138516
[ref-bevformer]: https://arxiv.org/abs/2203.17270
[ref-liftsplat]: https://arxiv.org/abs/2008.05711
[ref-transfusion]: https://arxiv.org/abs/2203.11496
[ref-cvar]: https://en.wikipedia.org/wiki/Conditional_value_at_risk
[ref-lqr]: https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator
[ref-hil]: https://en.wikipedia.org/wiki/Hardware-in-the-loop_simulation
[ref-slam]: https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping
[ref-kalman]: https://en.wikipedia.org/wiki/Kalman_filter
[ref-hungarian]: https://en.wikipedia.org/wiki/Hungarian_algorithm
[ref-jpda]: https://en.wikipedia.org/wiki/Probabilistic_data_association_filter
[ref-mht]: https://en.wikipedia.org/wiki/Multiple_hypothesis_tracking
[ref-brier]: https://en.wikipedia.org/wiki/Brier_score
[ref-uncertainty]: https://en.wikipedia.org/wiki/Uncertainty_quantification
[ref-zerocopy]: https://en.wikipedia.org/wiki/Zero-copy
[ref-pid]: https://en.wikipedia.org/wiki/PID_controller
[ref-social]: https://arxiv.org/abs/1609.04836
[ref-occupancy-flow]: https://arxiv.org/abs/2006.11867
[ref-minade]: https://arxiv.org/abs/2005.04259
[ref-modecollapse]: https://arxiv.org/abs/2007.06831
[ref-diffusion]: https://arxiv.org/abs/2303.12074
[ref-hybrid-a]: https://doi.org/10.1109/ROBOT.2008.4543489
[ref-lattice]: https://ieeexplore.ieee.org/document/5206621
[ref-jerk-poly]: https://arc.aiaa.org/doi/10.2514/3.20031
[ref-ttc-threat]: https://doi.org/10.1109/ITSC.2013.6728476
[ref-vehicle-fallback]: https://www.iso.org/standard/84126.html
[ref-fallback-mode]: https://www.iso.org/standard/81169.html
[ref-gpudirect]: https://developer.nvidia.com/gpudirect
[ref-cuda-graphs]: https://developer.nvidia.com/blog/cuda-graphs/
[ref-ruckig]: https://github.com/pantor/ruckig
[ref-chomp]: https://personalrobotics.cs.washington.edu/publications/rglchomp.pdf
[ref-stomp]: https://personalrobotics.cs.washington.edu/publications/stomp.pdf
[ref-esdf]: https://ieeexplore.ieee.org/document/6379367
[ref-astar]: https://en.wikipedia.org/wiki/A*_search_algorithm
[ref-dstar]: https://en.wikipedia.org/wiki/D*
