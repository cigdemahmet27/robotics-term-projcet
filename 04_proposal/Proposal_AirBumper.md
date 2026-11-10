# Calibrated collision detection and selective memory for aerial robots among invisible obstacles

YZV406E Robotics: Term Project Proposal (Round 1, 10.11.2026)
Team: Egemen Çevik (150220012), Cumali Sarı (150230038), Ahmet Enes Çiğdem (150220079)

> The submitted version is [`Proposal_AirBumper.pdf`](Proposal_AirBumper.pdf), with source in `latex/proposal.tex`. This Markdown copy is kept in sync for easy reading.
> We use a caged quadrotor MAV, as in the original paper, with PX4 SITL, ROS 2 Jazzy and Gazebo Harmonic.

## 1. Problem description and primary reference paper

Autonomous micro aerial vehicles (MAVs) use LiDAR to navigate cluttered indoor spaces. Their navigation depends on detecting obstacles, but two common cases leave gaps in perception:

1. Glass walls, doors and windows, transparent curtains and thin wires can let laser beams pass through. The map then shows free space, and the planner directs the MAV into the obstacle.
2. Obstacles can enter from above or below the LiDAR's limited vertical field of view without being seen.

Perception alone cannot prevent collisions in these cases. The MAV needs to detect contact, react before it crashes, and remember the obstacle to avoid another hit.

Permanent memory creates a problem when a person moves away, an object is removed or a door opens. The map retains a phantom wall that can make the goal unreachable. False detections can block routes in the same way.

Our project addresses the suggested topic Robot Failure Detection and Recovery. We aim to detect MAV collisions at a known false-alarm rate and retain collision memories selectively: invisible static obstacles should remain in the map, while transient obstacles and false alarms should be forgotten.

Our primary reference is R. Wang et al., "Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation," *IEEE ICRA 2024*, pp. 15735–15741 [1]. This original research paper was published in 2024 at ICRA, an approved venue. Its caged MAV has a 3D LiDAR and uses the IMU as its only collision sensor. The framework:

- Detects collisions when any body-axis acceleration exceeds a fixed threshold (a* = 20 m/s², window N = 10).
- Estimates the collision direction and contact point on a spherical cage.
- Reacts immediately by setting a position away from the contact, blended with the gradient of the EDT, which gives the distance to known obstacles.
- Adds the contact patch to the volumetric map as a "collision sensor" mark that LiDAR observations never clear.

The paper demonstrates recovery from glass doors in Gazebo and from glass and stick impacts on a real MAV. Across 10 real trials, detection takes approximately 0.18 s and recovery takes less than 1 s.

We chose Air Bumper because it is an early framework that combines collision detection, reaction and mapping in a standard LiDAR stack using only the IMU for collision sensing. It treats failure as a sensor by adding contact information to the map, which fits our project's focus on detection and recovery. The implementation is open source [2] and has been validated in simulation and on hardware. Its fixed threshold and permanent collision marks also give us specific design choices to investigate within a term project.

Follow-up work cites Air Bumper, including whisker-based tactile navigation in *Nature Communications* [3] and a survey of LiDAR-based aerial vehicles in *IEEE/ASME T-Mech* [4]. Semantic Scholar lists 7 citing papers in Nov. 2026. Related work [5] predicts planner failures from LiDAR geometry before they occur, while [6] adapts edge costs using contact feedback. Both assume that LiDAR detects every obstacle, an assumption that fails with glass.

Indoor MAVs used for inspection, warehouse inventory and search and rescue encounter glass facades, windows and partitions, as well as people and moving objects. False reactions interrupt flight, and phantom walls block routes. We retain the paper's caged MAV platform and its detection, reaction and memory pipeline, focusing on how the system declares a collision and how long it remembers the contact.

## 2. Environment, benchmark, and dataset

| Component | Choice | Justification |
|---|---|---|
| Middleware | ROS 2 Jazzy [7] in Docker | A free LTS release that provides the same setup on every laptop |
| Autopilot | PX4 SITL [8] via uXRCE-DDS (`px4_msgs`), offboard position/velocity setpoints | Uses the same autopilot as [1]; reaction setpoints go directly to the low-level controller |
| Simulator | Gazebo Harmonic [9] (PX4 `gz` integration), headless batch runs | Uses the same simulator family as [1], provides ground truth at no cost and supports gusts through *WindEffects* |
| Vehicle | PX4 x500 quadrotor with a spherical collision cage (radius l = 0.25 m), a 3D LiDAR configured like a Livox Mid-360 (360° × 59°), IMU (250 Hz, noise + bias), cage contact sensor | Matches the paper's caged MAV; the contact sensor provides ground-truth labels and is not an input to the detector |
| Map & planner | Local voxel occupancy map (0.1 m) with EDT [10], A* path search, velocity-limited setpoint follower | A lightweight Python replacement for the paper's GPU mapper and GTO-MPC, with a cost and speed limit for each voxel |
| Invisible obstacles | Models with collision geometry but no visual geometry | The GPU LiDAR renders only visual geometry, so beams pass through while the physics engine registers contact, reproducing the glass setup in [1] |
| Moving / transient | Joint/trajectory-driven panels and poles, doors opening at random times, objects removed after contact, a "stick" hitting from outside the LiDAR FOV | Tests memories of obstacles that have disappeared and reproduces the paper's stick test |

We will generate the scenario suite procedurally with fixed seeds, including a recreation of the paper's layout.

| Set | Content | Purpose | # worlds |
|---|---|---|---|
| S1: Static glass | Rooms and corridors with 1–4 glass walls, glass doors (recreating the paper's three-door world) and windows at random positions and heights | Detection and memory for invisible static obstacles | 60 |
| S2: Transient | (a) glass door opening 20–120 s after the first contact, (b) visible panel on the shortest route, removed after contact, (c) pole hitting the cage from below/above the LiDAR FOV (the stick test) | Phantom-wall failures | 45 |
| S3: False-alarm stress | Collision-free flights with wind gusts, aggressive stop-and-go and sharp turns, high-speed flight, doubled IMU noise | False alarms per hour | 30 |

Each set is split by world into 50/20/30% train/calibration/test subsets. Gust and maneuver profiles absent from calibration form an out-of-distribution test. We use speeds v_max ∈ {0.5, 1.0, 1.5} m/s.

We plan to collect approximately 10 h of collision-free flight for calibration, a separate collision-free test set of ≥ 50 h to verify false-alarm rates, and approximately 2,000 labeled collisions. Headless runs keep the cost of this data collection low.

## 3. Challenges and limitations

| # | Limitation of [1] (evidence) | Why it matters |
|---|---|---|
| L1 | Fixed, hand-tuned threshold on raw acceleration, a* = 20 m/s² (Sec. IV-A, "experimentally determined"). No false-alarm or miss rates are reported | Aggressive maneuvers and wind gusts also cause acceleration spikes, while soft or low-speed contacts stay below a*. The false-alarm rate is unknown and varies with platform, speed and wind |
| L2 | Per-axis test \|a_x\| > a* or \|a_y\| > a* (Sec. IV-A) | A diagonal hit splits the acceleration between axes: 25 m/s² at 45° gives 17.7 m/s² per axis, so the detector misses it |
| L3 | Direction from raw acceleration, a_c = −a (Eqs. 1–4) | Commanded acceleration biases the estimated direction, recovery setpoint and mapped patch |
| L4 | Permanent, top-priority marks: "the robot remembers all obstacles it ever collides with" (Sec. V-C) | Permanent marks retain static glass, but also leave phantom walls after people move, objects are removed or doors open. The map also preserves false detections from L1 |
| L5 | LiDAR evidence at the contact is ignored | A LiDAR return distinguishes visible obstacles, which are already mapped and may move, from invisible obstacles that need collision memory |
| L6 | Small, qualitative evaluation: 1 simulated world, 10 real trials | The evaluation provides no TPR, false alarms/hour, latency or direction-error metrics, baselines or statistics |

These limitations raise two questions. Can we give MAV collision detection a guaranteed false-alarm rate in place of a hand-tuned threshold? Can the map retain invisible static obstacles while forgetting transient obstacles and false detections?

## 4. Proposed method and contributions

We will reimplement Air Bumper as baseline B1 in ROS 2 with PX4 SITL. This port is not a contribution. It includes:

- The per-axis detector (a*, N).
- The sphere contact model c_c = Ĉ·l.
- The recovery setpoint p_r = (w·G/‖G‖ − (1−w)·Ĉ)·R_d with R_d = 2l, sent directly as a PX4 offboard position setpoint.
- Permanent, lethal collision voxels.

### C1: Model-aware, calibrated collision detection (L1–L3)

(a) Residual signal. We compute r(t) = a_IMU(t) − â(t), where â is the acceleration explained by the MAV's commands. To obtain â, we pass the commanded setpoints through a closed-loop response model of the PX4 controller, identified from collision-free flights, then rotate the predicted acceleration into the body frame and remove gravity. This removes the contribution of braking and turning (L1, L3). Model-based residuals are a standard tool for collision detection on manipulators [11]; we apply them to IMU-based detection on MAVs.

(b) Features and score. Over a window W = 0.1 s, we extract:

- The peak 3D residual norm ‖r‖, which removes the per-axis issue in L2.
- The peak jerk.
- The angular-rate residual.
- The setpoint-tracking error, which captures pushing against an obstacle.

A small logistic regression model maps these features to a score s ∈ [0, 1]. We also test s = ‖r‖ as an ablation.

(c) Calibrated threshold. We apply split conformal calibration [12], [13] to n collision-free calibration windows, setting τ̂ to the ⌈(n+1)(1−α)⌉-th smallest score.

- A new collision-free window triggers with probability ≤ α, giving a false-alarm rate set in advance of approximately α·3600/W per hour.
- For 1 FA/h with W = 0.1 s, α ≈ 2.8·10⁻⁵ and n ≥ 36,000 windows. This requires approximately 1 h of flight; we collect approximately 10 h.
- We target {1, 5, 20} FA/h and verify these rates on the ≥ 50 h test set using exact Poisson 95% intervals, with approximately 50 expected events at 1/h.

(d) Residual-based estimation. We compute the direction Ĉ from −r in place of −a, using azimuth and polar angle as in [1]. This removes the MAV's own acceleration from the direction estimate, recovery setpoint and mapped patch (L3).

### C2: Evidence-aware collision memory with forgetting (L4, L5)

(a) LiDAR evidence. When the detector triggers, we check LiDAR points in a ±10° cone around the contact direction and assign one of three classes:

- Visible: a return lies within contact distance + 0.15 m. The obstacle is already mapped and may be a person or moving object.
- Invisible: beams pass beyond the contact point, as with glass.
- Unseen: the direction lies outside the vertical FOV, as with a stick hitting from below.

(b) Decaying belief. Each mark k stores its patch, hit count h_k and last-hit time t_k. Its belief decays as b_k(t) = exp(−(t − t_k)/τ_class), with:

- τ_invisible = 600 s
- τ_unseen = 60 s
- τ_visible = 10 s

We tune all three values on calibration worlds and run a sensitivity study. A new hit within 0.3 m merges with the existing mark, increments the hit count (h_k + 1) and resets t_k. Repeated contacts keep glass in memory, while memories of transient obstacles and false alarms fade.

(c) Graded costs and speed limits. The voxel cost is c = c_max·b_k, and a mark is lethal only if h_k ≥ 2. A single unconfirmed mark therefore cannot make the goal unreachable. Near decayed or unconfirmed marks, we limit the setpoint speed to 0.3 m/s. Revisiting a marked location then causes at most a gentle cage contact that C1 detects quickly.

We will implement the method as Python (`rclpy`) nodes: `imu_residual`, `collision_detector`, `reaction`, `collision_memory` and `voxel_planner`.

We expect the following improvements over [1]:

1. A false-alarm rate set before deployment and verified on test worlds, with robustness to wind and aggressive maneuvers.
2. Detection of diagonal and low-speed contacts.
3. Smaller direction errors and more accurately placed recovery setpoints.
4. Avoidance of phantom walls at the cost of a small, measured increase in repeated collisions on static glass. RQ3 tests this trade-off.

## 5. Research questions and evaluation plan

- RQ1 (Detection): Does C1 achieve a higher TPR than Air Bumper's fixed per-axis threshold at the same false-alarm rate across speeds, maneuvers and wind? Does the calibrated rate hold under unseen gust profiles?
- RQ2 (Estimation): How much does residual-based estimation reduce direction error and improve recovery distance?
- RQ3 (Memory): Does C2 avoid phantom-wall failures, and how many repeat collisions on static glass does it add compared with permanent memory?
- RQ4 (System): Does the full system increase mission success, reduce crashes and shorten time-to-goal?

| ID | Detection | Memory | Represents |
|---|---|---|---|
| B0 | none (LiDAR-only navigation) | none | Standard MAV stack, which crashes as in [1] Fig. 4b |
| B1 | Air Bumper [1]: a* = 20 m/s², and a* re-tuned on calibration worlds | permanent, lethal | Primary paper |
| B2 | Oracle (cage contact sensor) | permanent | Perfect detection to isolate the effect of memory |
| B3 | C1 | permanent | C1 alone |
| B4 | Air Bumper | C2 | C2 alone |
| Ours | C1 | C2 | Full method |

We will measure:

- Detection: TPR, false alarms/hour (exact Poisson 95% intervals), empirical versus target rate, latency and ROC-AUC.
- Estimation: direction error (deg) and post-recovery distance to the obstacle (m).
- Memory: phantom volume (m³ × time), repeated collisions within 0.5 m of an earlier collision, and blocked-goal events.
- System: mission success, crashes (attitude > 60° or uncommanded altitude loss), collisions per run, time-to-goal and node run time (< 5 ms per cycle).

| # | Experiment | Data | RQ |
|---|---|---|---|
| E1 | Offline detection: ROC, TPR at matched FA rate; features vs ‖r‖ vs raw acceleration; per speed and wind level | S1–S3 test logs | RQ1 |
| E2 | Calibration validity: target {1, 5, 20}/h vs measured; unseen gusts/maneuvers | ≥ 50 h collision-free test set | RQ1 |
| E3 | Estimation: raw vs residual direction; recovery distance | S1, S2 | RQ2 |
| E4 | Memory (closed loop, oracle detector): permanent vs decay-only vs C2 | S1, S2 | RQ3 |
| E5 | Full comparison B0–B4 vs Ours × 3 speeds × 3 seeds | S1–S3 test | RQ4 |
| E6 | Ablations/sensitivity: τ, lethal-after-h, speed limit, LiDAR evidence, W | Calibration worlds | RQ3, RQ4 |

The compute budget covers approximately 41 test worlds: 6 × 3 × 3 × 41 ≈ 2,200 episodes, each lasting approximately 60 s, for approximately 37 h of simulated flight. At a PX4 SITL real-time factor of approximately 1, running 3 containers in parallel takes approximately 12 h of wall-clock time. If needed, we will reduce the full grid to one seed or two speeds while retaining all seeds for B1 versus Ours.

Worlds are the unit of statistical analysis. We will report 95% bootstrap CIs over worlds, McNemar tests and Wilcoxon signed-rank tests with Holm correction. All hyperparameters, including the B1 threshold, will be tuned only on calibration worlds.

The main threats to validity are:

- Simulated contacts and IMU readings are idealized. We add noise/bias and report results for each noise level.
- Temporal correlation makes the conformal guarantee approximate, so we also report the empirical false-alarm rate.
- We do not test sim-to-real transfer.

During the first week, we will check whether collision-only models are invisible to `gpu_lidar` and whether the caged x500 survives contacts in SITL without flipping. If the first check fails, we will use a LiDAR visibility mask. If the second fails, we will tune contact stiffness and lower the speed range.

## 6. Team roles and individual contributions

| Member | Role | Responsibilities and deliverables |
|---|---|---|
| Egemen Çevik (150220012) | Simulation, scenarios and experiments | Docker/ROS 2/PX4 SITL/Gazebo setup; caged x500 with LiDAR, IMU, contact sensor; invisible-obstacle models; S1–S3 generators; voxel map and planner; episode runner, ground-truth labels; batch runs of E5 |
| Cumali Sarı (150230038) | Detection and estimation (C1) | Air Bumper detector for B1; controller-response model, residual node, features, logistic regression, conformal calibration, residual-based direction estimation; E1–E3; statistics |
| Ahmet Enes Çiğdem (150220079) | Reaction and memory (C2) | Air Bumper recovery control and permanent memory for B1; collision-memory node, LiDAR evidence check, graded costs and speed limits; B3/B4 configs; E4, E6; figures |

Each member will write and present their own module. We will document contributions through per-member git history and a weekly `CONTRIBUTIONS.md`.

Our timeline is:

| Week | Milestone |
|---|---|
| 1 (11–16 Nov) | PX4 SITL + ROS 2 running; verify invisible obstacles and IMU spikes |
| 2 | S1–S3, voxel planner, B1 |
| 3 | Data collection, controller-response model |
| 4 | C1, E1–E3 |
| 5 | C2, E4 |
| 6 | E5, E6, statistics |
| 7 | Report and slides |
| 30.12.2026 | Presentation |

The minimum scope is B1 + C1 + E1–E3. Our target adds C2 and E4–E5. If time permits, we will also test a LiDAR model with partial glass returns and conduct a low-speed contact study.

## References

[1] R. Wang, Z. Guo, Y. Chen, X. Wang, and B. M. Chen, "Air Bumper: A collision detection and reaction framework for autonomous MAV navigation," in *Proc. IEEE ICRA*, 2024, pp. 15735–15741, doi: 10.1109/ICRA57147.2024.10611410.
[2] R. Wang et al., "Air Bumper," GitHub repository, 2024. https://github.com/ryrobotics/air_bumper
[3] C. Ye, G. de Croon, and S. Hamaza, "Whisker-based tactile flight for tiny drones," *Nature Communications*, 2025 (arXiv:2510.03119).
[4] Y. Ren, Y. Cai, H. Li, N. Chen, F. Zhu, L. Yin, F. Kong, R. Li, and F. Zhang, "A survey on LiDAR-based autonomous aerial vehicles," *IEEE/ASME Trans. Mechatronics*, 2025, doi: 10.1109/TMECH.2025.3599633.
[5] N. Mohammad, J. Higgins, and N. Bezzo, "A GP-based robust motion planning framework for agile autonomous robot navigation and recovery in unknown environments," in *Proc. IEEE ICRA*, 2024, pp. 2418–2424, doi: 10.1109/ICRA57147.2024.10610382.
[6] B. He et al., "Interactive-FAR: Interactive, fast and adaptable routing for navigation among movable obstacles in complex unknown environments," in *Proc. IEEE/RSJ IROS*, 2024, pp. 5402–5409, doi: 10.1109/IROS58592.2024.10802169.
[7] S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild," *Science Robotics*, vol. 7, no. 66, eabm6074, 2022.
[8] L. Meier, D. Honegger, and M. Pollefeys, "PX4: A node-based multithreaded open source robotics framework for deeply embedded platforms," in *Proc. IEEE ICRA*, 2015, pp. 6235–6240.
[9] N. Koenig and A. Howard, "Design and use paradigms for Gazebo, an open-source multi-robot simulator," in *Proc. IEEE/RSJ IROS*, 2004, pp. 2149–2154.
[10] P. F. Felzenszwalb and D. P. Huttenlocher, "Distance transforms of sampled functions," *Theory of Computing*, vol. 8, pp. 415–428, 2012.
[11] S. Haddadin, A. De Luca, and A. Albu-Schäffer, "Robot collisions: A survey on detection, isolation, and identification," *IEEE Trans. Robot.*, vol. 33, no. 6, pp. 1292–1312, 2017.
[12] V. Vovk, A. Gammerman, and G. Shafer, *Algorithmic Learning in a Random World*. Springer, 2005.
[13] A. N. Angelopoulos and S. Bates, "Conformal prediction: A gentle introduction," *Found. Trends Mach. Learn.*, vol. 16, no. 4, pp. 494–591, 2023.
