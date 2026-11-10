# Bump Once, Remember Wisely: Calibrated Collision Detection and Forgetting Memory for Aerial Robots Among Invisible Obstacles

**YZV406E Robotics – Term Project Proposal (Round 1, 10.11.2026)**
**Team:** Egemen Çevik (150220012), Cumali Sarı (150230038), Ahmet Enes Çiğdem (150220079)

> The submitted version is [`Proposal_AirBumper.pdf`](Proposal_AirBumper.pdf) (source `latex/proposal.tex`). This Markdown is kept in sync for easy reading.
> **Platform: aerial robot (caged quadrotor MAV), as in the original paper.** PX4 SITL + ROS 2 Jazzy + Gazebo Harmonic.

---

## 1. Problem Description and Primary Reference Paper

**Problem.** Autonomous micro aerial vehicles (MAVs) navigate cluttered indoor spaces with LiDAR, assuming every obstacle is seen. Two common cases break this:
1. **Invisible obstacles:** glass walls, doors and windows, transparent curtains and thin wires let laser beams pass through. The map shows free space and the planner flies into them.
2. **Obstacles that enter unseen:** e.g., from above or below the LiDAR's limited vertical field of view.

In both cases a collision cannot be avoided from perception alone. The MAV must therefore (i) **detect** that it collided, (ii) **react** safely before it crashes, and (iii) **remember** the obstacle so it does not hit it again.

Remembering creates a new difficulty: if the obstacle was a person, an object that is later moved, or a door that later opens, a permanent memory turns it into a **phantom wall**. A *false* detection does the same and can make the goal unreachable.

The project addresses the suggested topic **Robot Failure Detection and Recovery**. **Our problem:** detect MAV collisions with a *known* false-alarm rate, and remember them selectively, keeping invisible static obstacles while forgetting transient ones and false alarms.

**Primary reference paper.** R. Wang et al., "Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation," *IEEE ICRA 2024*, pp. 15735–15741 [1]. It is an original research paper from an approved venue (ICRA), published 2024. A caged MAV with a 3D LiDAR uses only its IMU to:
- **Detect** a collision: any body-axis acceleration above a fixed threshold (a* = 20 m/s², window N = 10).
- **Estimate** the collision direction and the contact point on a spherical cage.
- **React** immediately: a position setpoint away from the hit, blended with the gradient of the EDT (distance to known obstacles).
- **Remember** it: the contact patch goes into the volumetric map as a "collision sensor" whose marks are **never cleared** by the LiDAR.

The paper shows recovery from glass doors in Gazebo, and from glass and stick hits on a real MAV (detection ≈ 0.18 s, recovery < 1 s, 10 trials).

**Why this paper.**
- It is an early framework combining collision detection, reaction **and** mapping in a standard LiDAR stack using only the IMU.
- It turns failures into map knowledge ("failure as a sensor"), which is directly a failure detection and recovery problem.
- It is open source [2] and validated in simulation and on hardware.
- **Cited by follow-up work** on contact-aware flight, e.g., whisker-based tactile navigation in *Nature Communications* [3] and a LiDAR-based aerial vehicle survey in *IEEE/ASME T-Mech* [4] (7 citing papers on Semantic Scholar, Nov. 2026).
- Its design choices are simple and explicit (fixed threshold, permanent marks), so their limitations can be analyzed and improved within a term project.
- Related work: [5] predicts planner failures *before* they happen from LiDAR geometry, and [6] adapts edge costs from contact feedback. Both assume the LiDAR sees every obstacle, which is exactly what fails with glass.

**Significance.** Indoor MAVs for inspection, warehouse inventory and search and rescue fly around glass facades, windows and partitions, and among people and moving objects. A detector with an unknown false-alarm rate and a map that never forgets both cause mission failures: false reactions interrupt flight, and phantom walls block routes. We **keep the paper's platform (a caged MAV)** and its detect–react–remember pipeline, and improve the two weakest parts: **how** a collision is declared and **how long** it is remembered.

## 2. Environment, Benchmark, and Dataset

| Component | Choice | Justification |
|---|---|---|
| Middleware | **ROS 2 Jazzy** [7] in Docker | LTS, free, the same setup on every laptop |
| Autopilot | **PX4 SITL** [8] via uXRCE-DDS (`px4_msgs`), offboard position/velocity setpoints | Same autopilot as [1]. Reaction setpoints go directly to the low-level controller |
| Simulator | **Gazebo Harmonic** [9] (PX4 `gz` integration), headless batch runs | Same simulator family as [1]. Free ground truth. *WindEffects* for gusts |
| Vehicle | PX4 **x500 quadrotor with a spherical collision cage** (radius l = 0.25 m), a 3D LiDAR configured like a Livox Mid-360 (360° × 59°), IMU (250 Hz, noise + bias), cage contact sensor | Mirrors the paper's caged MAV. The contact sensor gives ground-truth labels and is **not** used by the detector |
| Map & planner | Local voxel occupancy map (0.1 m) with EDT [10], A* path search, velocity-limited setpoint follower | A lightweight Python replacement for the paper's GPU mapper and GTO-MPC. Exposes a cost and a speed limit per voxel |
| Invisible obstacles | Models with collision geometry but no visual geometry | The GPU LiDAR renders visuals only, so beams pass through while physics blocks: glass as in [1] |
| Moving / transient | Joint/trajectory-driven panels and poles, doors opening at random times, objects removed after contact, a "stick" hitting from outside the LiDAR FOV | Creates "remembered but gone" cases and reproduces the paper's stick test |

**Scenario suite** (procedurally generated, seeded, including the paper's own layout):

| Set | Content | Purpose | # worlds |
|---|---|---|---|
| **S1 – Static glass** | Rooms and corridors with 1–4 glass walls, glass doors (re-creating the paper's three-door world) and windows at random positions and heights | Detection + memory on invisible static obstacles | 60 |
| **S2 – Transient** | (a) glass door opening 20–120 s after the first contact, (b) visible panel on the shortest route, removed after contact, (c) pole hitting the cage from below/above the LiDAR FOV (the stick test) | Phantom-wall problem | 45 |
| **S3 – False-alarm stress** | **No collisions**: wind gusts, aggressive stop-and-go and sharp turns, high-speed flight, doubled IMU noise | False alarms per hour | 30 |

**Split:** by world, 50/20/30% train/calibration/test per set. Gust and maneuver profiles unseen during calibration form an out-of-distribution test. **Speeds:** v_max ∈ {0.5, 1.0, 1.5} m/s.

**Data:** ~10 h of collision-free flight for calibration, a **separate** ≥ 50 h collision-free test set to verify false-alarm rates (headless runs are cheap), and ~2,000 labeled collisions.

## 3. Challenges and Limitations

| # | Limitation of [1] (evidence) | Why it matters |
|---|---|---|
| **L1** | Fixed, hand-tuned threshold on raw acceleration, a* = 20 m/s² (Sec. IV-A, "experimentally determined"). No false-alarm or miss rates are reported | The MAV's **own** aggressive maneuvers and **wind gusts** also cause spikes, while soft or low-speed hits stay below a*. The false-alarm rate is unknown and changes with platform, speed and wind |
| **L2** | Per-axis test \|a_x\| > a* **or** \|a_y\| > a* (Sec. IV-A) | A diagonal hit splits between axes: 25 m/s² at 45° gives 17.7 m/s² per axis and is **missed** |
| **L3** | Direction from raw acceleration, a_c = −a (Eqs. 1–4) | Commanded acceleration biases the direction, and therefore the recovery setpoint and the mapped patch |
| **L4** | Permanent, top-priority marks: "the robot remembers all obstacles it ever collides with" (Sec. V-C) | Right for static glass. Wrong for people, moved objects, opened doors and **false detections**, which become phantom walls. Detection errors (L1) are made **permanent** by the map |
| **L5** | LiDAR evidence at the contact is ignored | Whether the LiDAR *saw* the obstacle separates visible (already mapped, maybe moving) from invisible (must be remembered) |
| **L6** | Small, qualitative evaluation: 1 simulated world, 10 real trials | No TPR, false alarms/hour, latency or direction error; no baselines or statistics |

**Open questions:** Can MAV collision detection have a **guaranteed** false-alarm rate instead of a hand-tuned threshold? Can collision memory be **selective**, keeping invisible static obstacles while forgetting transient ones and its own mistakes?

## 4. Proposed Method and Contributions

**Baseline reimplementation (B1, not a contribution).** We port Air Bumper to ROS 2 + PX4 SITL:
- the per-axis detector (a*, N)
- the sphere contact model c_c = Ĉ·l
- the recovery setpoint p_r = (w·G/‖G‖ − (1−w)·Ĉ)·R_d with R_d = 2l, sent directly as a PX4 offboard position setpoint
- permanent, lethal collision voxels

### C1 – Model-aware, calibrated collision detection (L1–L3)

**(a) Residual signal.** r(t) = a_IMU(t) − â(t). â is the acceleration explained by the MAV's **own commands**: the commanded setpoints pass through a closed-loop response model of the PX4 controller, identified on collision-free flights, then are rotated into the body frame with gravity removed. Own braking and turning are thus removed (L1, L3). Model-based residuals are the standard tool for collision detection on manipulators [11]; we bring the idea to IMU-based detection on MAVs.

**(b) Features → score.** Over a window W = 0.1 s:
- the peak 3D residual **norm** ‖r‖ (removes the per-axis issue, L2)
- the peak jerk
- the angular-rate residual
- the setpoint-tracking error (pushing against an obstacle)

A small logistic regression maps them to a score s ∈ [0, 1]. As an ablation, s = ‖r‖.

**(c) Calibrated threshold.** Split conformal calibration [12], [13] on n collision-free calibration windows: τ̂ = the ⌈(n+1)(1−α)⌉-th smallest score.
- A new collision-free window triggers with probability ≤ α, so the false-alarm rate is fixed in advance: ≈ α·3600/W per hour.
- For 1 FA/h with W = 0.1 s, α ≈ 2.8·10⁻⁵ and n ≥ 36,000 windows (≈ 1 h of flight; we collect ~10 h).
- Targets are {1, 5, 20} FA/h, verified on the ≥ 50 h test set with exact Poisson 95% intervals (~50 expected events at 1/h).

**(d) Residual-based estimation.** The direction Ĉ (azimuth and polar angle, as in [1]) is computed from −r instead of −a. The recovery setpoint and the mapped patch are therefore not biased by the MAV's own acceleration (L3).

### C2 – Evidence-aware, forgetting collision memory (L4, L5)

**(a) LiDAR evidence.** At detection, check the LiDAR points in a ±10° cone around the contact direction:
- **visible:** a return within contact distance + 0.15 m. Already mapped; possibly a person or a moving object.
- **invisible:** the beams pass beyond the contact point. The glass case.
- **unseen:** the direction is outside the vertical FOV (e.g., the stick from below).

**(b) Decaying belief.** Each mark k stores its patch, hit count h_k and last-hit time t_k, with belief b_k(t) = exp(−(t − t_k)/τ_class):
- τ_invisible = 600 s
- τ_unseen = 60 s
- τ_visible = 10 s

All three values are tuned on calibration worlds, with a sensitivity study. A new hit within 0.3 m merges with the mark (h_k + 1, t_k reset). Real glass is hit again and stays remembered; transient obstacles and false alarms fade away.

**(c) Graded costs and speed limits, not walls.**
- Voxel cost c = c_max·b_k, **lethal only if h_k ≥ 2**. A single unconfirmed mark can never make the goal unreachable.
- Near decayed or unconfirmed marks, the setpoint speed is limited to 0.3 m/s, so re-testing a spot causes at most a gentle cage contact that C1 detects quickly.

**Implementation.** Python (`rclpy`) nodes: `imu_residual`, `collision_detector`, `reaction`, `collision_memory`, `voxel_planner`.

**Expected improvements over [1]:**
1. A false-alarm rate fixed before deployment and verified on test worlds, robust to wind and aggressive maneuvers.
2. Detection of diagonal and low-speed contacts.
3. Smaller direction errors and better-placed recovery setpoints.
4. No phantom walls, at the cost of a small, measured increase in repeated collisions on static glass. This trade-off is tested in RQ3.

## 5. Research Questions and Evaluation Plan

- **RQ1 – Detection:** Does C1 achieve a higher TPR than Air Bumper's fixed per-axis threshold at the **same false-alarm rate**, across speeds, maneuvers and wind? Does the calibrated rate hold under unseen gust profiles?
- **RQ2 – Estimation:** How much does residual-based estimation reduce the direction error and improve the recovery distance?
- **RQ3 – Memory:** Does C2 avoid phantom-wall failures, and how many extra repeated collisions on static glass does that cost compared with permanent memory?
- **RQ4 – System:** Does the full system improve mission success, crashes and time-to-goal?

| ID | Detection | Memory | Represents |
|---|---|---|---|
| **B0** | none (LiDAR-only navigation) | none | Standard MAV stack (crashes, as in [1] Fig. 4b) |
| **B1** | **Air Bumper [1]**: a* = 20 m/s², and a* re-tuned on calibration worlds | permanent, lethal | Primary paper |
| **B2** | Oracle (cage contact sensor) | permanent | Perfect detection; isolates memory |
| **B3** | C1 | permanent | C1 alone |
| **B4** | Air Bumper | C2 | C2 alone |
| **Ours** | C1 | C2 | Full method |

**Metrics.**
- **Detection:** TPR, false alarms/hour (exact Poisson 95% intervals), empirical vs target rate, latency, ROC-AUC.
- **Estimation:** direction error (deg), post-recovery distance to the obstacle (m).
- **Memory:** phantom volume (m³ × time), repeated collisions (within 0.5 m of an earlier one), blocked-goal events.
- **System:** mission success, crashes (attitude > 60° or uncommanded altitude loss), collisions per run, time-to-goal, node run time (< 5 ms per cycle).

| # | Experiment | Data | RQ |
|---|---|---|---|
| E1 | Offline detection: ROC, TPR at matched FA rate; features vs ‖r‖ vs raw acceleration; per speed and wind level | S1–S3 test logs | RQ1 |
| E2 | Calibration validity: target {1, 5, 20}/h vs measured; unseen gusts/maneuvers | ≥ 50 h free test set | RQ1 |
| E3 | Estimation: raw vs residual direction; recovery distance | S1, S2 | RQ2 |
| E4 | Memory (closed loop, oracle detector): permanent vs decay-only vs C2 | S1, S2 | RQ3 |
| E5 | Full comparison B0–B4 vs Ours × 3 speeds × 3 seeds | S1–S3 test | RQ4 |
| E6 | Ablations/sensitivity: τ, lethal-after-h, speed limit, LiDAR evidence, W | Calibration worlds | RQ3, RQ4 |

**Compute budget.** ~41 test worlds → 6 × 3 × 3 × 41 ≈ 2,200 episodes × ~60 s ≈ 37 h of simulated flight. With PX4 SITL at a real-time factor ≈ 1 and 3 parallel containers, that's ≈ 12 h of wall-clock time. Fallback: one seed or two speeds for the full grid, keeping all seeds for B1 vs Ours.

**Statistics.** Worlds are the unit of analysis. We report 95% bootstrap CIs over worlds, McNemar tests and Wilcoxon signed-rank tests, with Holm correction. All hyperparameters, including the B1 threshold, are tuned only on calibration worlds.

**Threats to validity.**
- The simulated contacts and IMU are idealized, so we add noise/bias and report results per noise level.
- The conformal guarantee is approximate under temporal correlation, so we report the empirical rate.
- Sim-to-real transfer is not tested.

**Week-1 risk checks:**
- Are collision-only models invisible to `gpu_lidar`? Fallback: a LiDAR visibility mask.
- Does the caged x500 survive contacts in SITL without flipping? Fallback: tuned contact stiffness and a lower speed range.

## 6. Team Roles and Individual Contributions

| Member | Role | Responsibilities and deliverables |
|---|---|---|
| Egemen Çevik (150220012) | **Simulation, Scenarios & Experiments** | Docker/ROS 2/PX4 SITL/Gazebo bring-up; caged x500 with LiDAR, IMU, contact sensor; invisible-obstacle models; S1–S3 generators; voxel map and planner; episode runner, ground-truth labels; batch runs of E5 |
| Cumali Sarı (150230038) | **Detection & Estimation (C1)** | Air Bumper detector for B1; controller-response model, residual node, features, logistic regression, conformal calibration, residual-based direction estimation; E1–E3; statistics |
| Ahmet Enes Çiğdem (150220079) | **Reaction & Memory (C2)** | Air Bumper recovery control and permanent memory for B1; collision-memory node, LiDAR evidence check, graded costs and speed limits; B3/B4 configs; E4, E6; figures |

Each member writes and presents their own module. Contributions are documented by per-member git history and a weekly `CONTRIBUTIONS.md`.

**Timeline:**

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

**Scope:**
- **Minimum:** B1 + C1 + E1–E3.
- **Target:** + C2, E4–E5.
- **Stretch:** a LiDAR model with partial glass returns, and a low-speed contact study.

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
