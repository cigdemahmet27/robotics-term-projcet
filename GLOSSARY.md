# Glossary (A–Z)

Short, plain-language definitions. For longer explanations see [`02_primary_paper_deep_dive/01_background_knowledge.md`](02_primary_paper_deep_dive/01_background_knowledge.md).

| Term | Meaning |
|---|---|
| **A\*** | Grid/graph shortest-path search guided by an estimate of the remaining distance |
| **Ablation** | An experiment that removes or changes one part of the method to measure its contribution |
| **AMR** | Autonomous Mobile Robot |
| **APE** | Absolute Pose Error: RMSE between estimated and true trajectory (localization accuracy) |
| **ARD kernel** | Automatic Relevance Determination: a GP kernel with one length-scale per feature, which learns which features matter |
| **AUROC / AUPRC** | Area under the ROC / precision-recall curve. Summarizes detector quality over all thresholds |
| **BARN** | Benchmark Autonomous Robot Navigation: 300 cluttered Gazebo worlds plus a yearly ICRA challenge |
| **BARN score** | 1_success × OT / clip(AT, 2·OT, 8·OT). Rewards reaching the goal quickly |
| **Behavior Tree (BT)** | How Nav2 organizes navigation logic and recovery behaviors (a tree of actions/conditions) |
| **Bernoulli variable** | A random variable that is 1 with probability p and 0 otherwise |
| **Bézier curve** | A smooth curve defined by control points. It lies inside their convex hull |
| **Body frame / world frame** | Coordinate axes attached to the robot vs fixed to the world |
| **Calibration (statistical)** | Making a score/threshold mean what it claims (e.g., "5% false alarms" really gives ~5%) |
| **Conformal prediction** | A distribution-free method to choose thresholds/sets with guaranteed error rates, using a held-out calibration set |
| **Costmap** | Nav2's grid of traversal costs (free, inflated, lethal, unknown). There are global and local ones |
| **Costmap filter** | A Nav2 mechanism that modifies costmaps using a mask image/topic (e.g., keepout zones) |
| **Controller (local planner)** | Computes velocity commands that follow the global path and avoid obstacles (Nav2: DWB, MPPI, …) |
| **Convex set / polytope** | A shape where any straight line between two inside points stays inside / a flat-sided shape |
| **Cost-to-go** | The remaining path cost from a point to the goal |
| **Degeneracy (LiDAR)** | The scan geometry can't constrain motion in some direction (long corridor), so localization drifts |
| **Differential drive** | Left/right wheels driven separately. Moves forward/back and rotates, never sideways |
| **DWA / DWB** | Dynamic Window Approach / Nav2's implementation (DWB). Samples velocities, simulates, scores |
| **EDT** | Euclidean Distance Transform: the distance to the nearest obstacle for every cell |
| **ESKF** | Error-State Kalman Filter, used in LiDAR-inertial odometry |
| **Exchangeability** | A statistical assumption behind conformal guarantees: the data order doesn't matter (like i.i.d.) |
| **False alarm / miss** | The alarm fires without a failure (false positive) / a failure happens without an alarm (false negative) |
| **FASTER** | The safe-corridor trajectory planner used in the primary paper |
| **Feature** | A number computed from raw data that is fed to a model |
| **Front-end / back-end** | The path search + corridor building / the trajectory optimization, in the primary paper's planner |
| **Gazebo (Harmonic)** | Open-source robot simulator. Harmonic is the version paired with ROS 2 Jazzy |
| **GP (Gaussian Process)** | A probabilistic model giving a mean prediction plus uncertainty for any input |
| **Hit-and-run sampling** | A method to sample uniform random points inside a convex region |
| **Holonomic** | Can move in any direction instantly (a point robot). The opposite of a car-like or diff-drive robot |
| **Hysteresis** | Different thresholds to turn ON and OFF, preventing flickering |
| **IMU** | Inertial Measurement Unit (accelerometer + gyroscope) |
| **Inducing points** | A small set of representative points that make GPs scale to big data (sparse GP) |
| **JPS** | Jump Point Search, a fast A* variant on grids |
| **Kernel (RBF)** | A similarity function between inputs. RBF = exp(−distance²/2ℓ²) |
| **Lead time** | How long before a failure the alarm fires |
| **LiDAR** | A laser range sensor. 2D = one horizontal ring of distances |
| **LIO** | LiDAR-Inertial Odometry |
| **MAV** | Micro Aerial Vehicle (a small drone) |
| **McNemar / Wilcoxon test** | Paired statistical tests for binary outcomes / continuous values |
| **MPC** | Model Predictive Control: repeatedly optimize the next few seconds and apply the first step |
| **MPPI** | Model Predictive Path Integral: sampling-based MPC (a Nav2 controller) |
| **Nav2** | The ROS 2 navigation stack (planners, controllers, costmaps, behavior trees) |
| **Occlusion** | Something hidden behind another object, invisible to the sensor |
| **Occupancy grid** | A map of cells marked free / occupied / unknown |
| **Odometry** | An estimate of the robot's own motion/pose over time |
| **OOD** | Out-of-distribution: test conditions unlike the training data |
| **Progress checker** | Nav2 component that flags a robot not making progress (stuck) |
| **Receding horizon** | Plan a short window ahead, execute a bit, re-plan, repeat |
| **Risk metric** | One number summarizing possible bad outcomes (expectation, max, CVaR, …) |
| **ROS 2 (Jazzy / Lyrical)** | Robot Operating System 2. Jazzy = LTS for Ubuntu 24.04; Lyrical = LTS for Ubuntu 26.04 |
| **rosbag2 / mcap** | ROS 2 recording format for logged topics |
| **SDF** | Simulation Description Format (Gazebo world/model files) |
| **Safe corridor** | A chain of obstacle-free convex regions along a path |
| **Sim-to-real** | Training in simulation and deploying on a real robot |
| **SLAM** | Simultaneous Localization And Mapping (e.g., Gmapping, slam_toolbox) |
| **Sparse GP** | A GP approximated with inducing points, which scales to large datasets |
| **TPR / FPR** | True positive rate (recall) / false positive rate |
| **TTC / TTI** | Time-to-collision / time-to-intersect: distance divided by approach speed |
| **Tortuosity** | How twisty a path is (total turning per meter) |
| **UCB (upper confidence bound)** | mean + β × std: an optimistic or (here) cautious estimate |
