# Bump Once, Remember Wisely: Calibrated IMU Collision Detection and Evidence-Aware, Forgetting Collision Memory for Mobile Robots Among Invisible and Moving Obstacles

**YZV406E Robotics – Term Project Proposal (Round 1, 10.11.2026)**
**Team:** [Member 1], [Member 2], [Member 3], [Member 4]

---

## 1. Problem Description and Primary Reference Paper

**Problem.** LiDAR-based navigation assumes that every obstacle is seen by the LiDAR. Two common cases break this:
1. **Invisible obstacles.** Glass walls and doors, transparent plastic curtains and thin objects let laser beams pass through. The map shows free space and the planner drives straight into them.
2. **Obstacles that enter unseen.** Something moving in from the sensor's blind spot (e.g., behind a 270° LiDAR) is not detected in time.

In both cases a collision is **not avoidable from perception alone**. The robot must therefore (i) **detect** that it collided, (ii) **react** safely, and (iii) **remember** the obstacle so it does not hit it again. Remembering creates a new difficulty: if the obstacle was a person, a cart that is later moved, or a door that later opens, a permanent memory turns it into a **phantom wall**. A *false* collision detection does the same, and can make the goal unreachable. **Our problem:** detect collisions reliably with a *known* false-alarm rate, and remember collisions in a way that keeps invisible static obstacles while forgetting transient ones and false alarms.

**Primary reference paper.** R. Wang, Z. Guo, Y. Chen, X. Wang, B. M. Chen, "Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation," *IEEE ICRA 2024*, pp. 15735–15741 [1]. Air Bumper is an original research paper from an approved venue (ICRA) in the required period (2024).

**What Air Bumper does.** A caged drone flying with a 3D LiDAR uses only its **IMU** to:
- **Detect** a collision: any body-axis acceleration above a fixed threshold, with a* = 20 m/s² and a sliding window of N = 10 samples.
- **Estimate** the collision direction and the contact point, assuming a spherical cage.
- **React** immediately: move away from the hit, blended with the gradient of the distance to known obstacles.
- **Remember** the collision: the contact patch is inserted into the volumetric map as a "collision sensor" whose marks are **never cleared** by the LiDAR.

The paper shows recovery from glass doors in Gazebo and from glass and stick hits on a real drone (detection ≈ 0.18 s, recovery < 1 s, 10 trials).

**Why this paper.**
- It is one of the first frameworks to put collision detection, reaction **and** mapping into one standard LiDAR navigation stack using only the IMU.
- It turns failures into map knowledge ("failure as a sensor"), which is directly a **failure detection and recovery** problem, one of the suggested topics.
- It is open source [2] and validated in simulation and on hardware.
- Its design choices are simple and explicit (a fixed threshold, permanent marks, a spherical body), which makes their limitations clear to analyze and possible to improve in a term project.
- Related work handles failures differently: [13] predicts planner failures *before* they happen from LiDAR geometry, and [12] adapts edge costs online from contact feedback. Both assume the LiDAR sees every obstacle, which is exactly what fails with glass.

**Relevance beyond drones.** Glass and transparent surfaces are common in offices, airports, malls and hospitals. Ground AMRs in such places mostly rely on physical bumper switches that only *stop* the robot and do not remember anything [3]. We therefore study the Air Bumper pipeline on a **ground robot**, where the same three needs (detect, react, remember) appear with new failure modes: braking, floor bumps, and a box-shaped body.

## 2. Environment, Benchmark, and Dataset

| Component | Choice | Justification |
|---|---|---|
| Middleware | **ROS 2 Jazzy** [11] (Docker, `osrf/ros:jazzy-desktop`) | LTS, free, and the paper's stack is ROS-based. Docker gives the same setup on every team laptop |
| Navigation | **Nav2** [4]: Smac/NavFn global planner, MPPI controller, layered costmaps [5] with **costmap filters** (keepout and speed masks) | The industry-standard ROS 2 navigation stack. Costmap filters let us inject collision memory **without modifying Nav2** |
| Simulator | **Gazebo Harmonic** [6] via `ros_gz`, headless for batch runs | The same simulator family as Air Bumper's evaluation. Gives free ground truth |
| Robot | **Clearpath Jackal** (diff drive, 0.51 × 0.43 m) with a 270° 2D LiDAR, an IMU (100–200 Hz, Gaussian noise + bias), wheel odometry, and a **contact sensor on the chassis** | A common research AMR. The contact sensor gives **ground-truth collision labels** (time, location) and is **not** used by our detector |
| Invisible obstacles | Gazebo models with **collision geometry but no visual geometry**: the GPU LiDAR renders visuals, so the beams pass through them while the physics still blocks the robot | Reproduces glass exactly as in [1] ⚠️ to verify in week 1 |
| Moving / transient obstacles | Models moved by joint or trajectory controllers (crossing boxes, carts), doors that open at a random time, objects removed after the first contact (despawn service) | Creates the "remembered but no longer there" cases that permanent memory fails on |

**Scenario suite (our benchmark, procedurally generated, seeded):**

| Set | Content | Purpose | # worlds |
|---|---|---|---|
| **S1 – Static glass** | Office/warehouse layouts with glass walls, glass doors (as in [1]) and transparent partitions. Random positions, 1–4 per world | Detection + memory on invisible **static** obstacles | 60 |
| **S2 – Transient** | (a) glass door that opens 20–120 s after the first contact, (b) visible cart blocking the shortest route, removed after contact, (c) box crossing from the LiDAR blind spot (the analog of the paper's stick test) | Phantom-wall problem of permanent memory | 45 |
| **S3 – False-alarm stress** | **No collisions**: floor bumps / cable ramps (2–4 cm), rough floor patches, aggressive stop-and-go goals, IMU noise ×2 | False alarms per hour | 30 |
| **S4 – BARN-Glass** (stretch) | BARN worlds [7] where 10–30% of obstacles are made invisible | Dense clutter, a known benchmark | 50 |

**Data split (by world):** 50% train / 20% calibration / 30% test for each set. S3 terrain types not seen in calibration are kept for an **out-of-distribution** test. Speeds: v_max ∈ {0.5, 1.0, 1.5} m/s.

**Data volume:** ~10 h of simulated collision-free driving (for the false-alarm calibration, see §5) plus ~2,000 labeled collisions. At a real-time factor ≥ 1 with 2–3 parallel containers, this fits in a few overnight runs.

## 3. Challenges and Limitations of the Primary Paper

| # | Limitation of [1] | Evidence | Why it matters |
|---|---|---|---|
| **L1** | **Fixed, hand-tuned threshold on raw acceleration** (a* = 20 m/s², N = 10) | Sec. IV-A: "experimentally determined". No false-alarm or miss rates are reported | The robot's **own** maneuvers (hard braking, turns, floor bumps) also cause acceleration spikes. Soft or low-speed collisions stay below 20 m/s². The false-alarm rate is unknown and changes with platform, speed and floor |
| **L2** | **Per-axis test** (\|a_x\| > a* **or** \|a_y\| > a*) | Sec. IV-A | A diagonal hit splits between axes: a 25 m/s² hit at 45° gives 17.7 m/s² per axis, so it is **missed**, although the total exceeds a* |
| **L3** | **Direction from raw acceleration** | Eqs. 1–4 use a_c = −a | The robot's commanded acceleration biases the estimated direction, especially while braking or turning |
| **L4** | **Spherical-body contact model** (c_c = Ĉ·l) | Eq. 5 | For a box-shaped ground robot the contact point and the mapped patch are misplaced, and the map marks the wrong cells |
| **L5** | **Permanent, top-priority collision marks** | Sec. V-C: marks "will not be cleared by onboard sensors… the robot remembers all obstacles it ever collides with" | Correct for static glass. Wrong for people, carts, doors that open, and **false detections**: they become **phantom walls** that can make the goal unreachable. Errors in detection (L1) are thus **made permanent** by the map (L5) |
| **L6** | **No use of LiDAR evidence at the contact** | The collision sensor ignores what the LiDAR saw at the contact point | Whether the LiDAR *saw* something at the contact is valuable: a visible obstacle is already mapped and may move, while an invisible one must be remembered |
| **L7** | **Small, qualitative evaluation** | 1 simulated world, 10 real trials, examples of time series | No detection metrics (TPR, false alarms/hour, latency, direction error), no baselines, no statistics. A failure-detection method needs these to be trusted |

**Open questions motivating the project:** Can collision detection have a **guaranteed** false-alarm rate instead of a hand-tuned threshold? And can collision memory be **selective**, keeping invisible static obstacles while forgetting transient ones and its own mistakes?

## 4. Proposed Method and Contributions

We first **reimplement Air Bumper in ROS 2 for a ground robot** (our baseline **B1**, a necessary adaptation and not a contribution):
- Same per-axis threshold detector, sphere contact model and permanent marks.
- Reaction adapted to a diff-drive robot, which cannot move sideways: back off along the projection of −Ĉ on the heading, or rotate away and then move, for R_d = 0.3 m. It is sent through a priority `twist_mux` channel, so it does not wait for the planner.
- Marks written as lethal cells into a Nav2 keepout mask.

On top of this baseline we propose two contributions.

### C1 – Model-aware, calibrated collision detection and estimation (addresses L1–L4)

**(a) Residual signal.** Instead of raw acceleration, we use the **unexpected** acceleration:
- r(t) = a_IMU,xy(t) − â_xy(t)
- â is the acceleration the robot *should* have from its own commands: the commanded velocity (v_cmd, ω_cmd) is passed through a first-order response model identified from collision-free data, giving â_x = dv̂/dt and â_y = v̂·ω̂.
- The robot's own braking and turning are thus removed from the signal (L1, L3).

**(b) Window features → score.** Over a sliding window W = 0.2 s we compute:
- the peak **horizontal norm** ‖r_xy‖ (removes the per-axis issue, L2)
- the peak jerk
- the vertical residual and the pitch rate (separate floor bumps from collisions)
- the command-tracking error between v_cmd and the odometry speed (pushing against an obstacle).

A **logistic regression** (small, interpretable) maps these features to a collision score s ∈ [0, 1]. As an ablation, the score is ‖r_xy‖ alone.

**(c) Calibrated threshold.** We choose the alarm threshold by **split conformal calibration** [8], [9] on **collision-free** windows of the calibration worlds:
- τ̂ = the ⌈(n+1)(1−α)⌉-th smallest score among n calibration windows.
- Under exchangeability, a new collision-free window triggers with probability ≤ α. Counting non-overlapping windows, this sets the **false alarms per hour** in advance: ≈ α·3600/W.
- We target {0.5, 1, 5} false alarms/h. For 1 FA/h with W = 0.2 s, α ≈ 5.6·10⁻⁵, which needs n ≥ 1/α ≈ 18,000 windows (≈ 1 h of driving; we collect ~10 h, see §2).
- Consecutive windows are correlated, so we also report the **empirical** rate.

**(d) Footprint-aware estimation.**
- The direction is φ = atan2(−r_y, −r_x) computed from the **residual**, not the raw acceleration.
- The contact point is the intersection of this ray with the robot's **footprint polygon** instead of a sphere (L4).
- The mapped patch is a 0.3 m segment perpendicular to φ.

### C2 – Evidence-aware, forgetting collision memory (addresses L5, L6)

**(a) LiDAR evidence at contact.** At detection, we check the LiDAR rays within ±10° of the contact bearing:
- **visible:** a return exists within (contact distance + 0.15 m). The obstacle is already in the LiDAR map and may be a person or a cart.
- **invisible:** the beams pass beyond the contact point. This is the glass case.
- **unseen:** the bearing is in the LiDAR blind spot.

**(b) Decaying belief per mark.** Each mark k stores its position, extent, number of hits h_k and the time of its last hit t_k. Its belief is b_k(t) = exp(−(t − t_k)/τ_class), with:
- τ_invisible = 600 s
- τ_unseen = 60 s
- τ_visible = 10 s (just enough to prevent an immediate re-hit; the LiDAR covers the rest)

All three values are tuned on calibration worlds, with a sensitivity study. A new hit within 0.3 m of a mark **merges** with it: h_k + 1 and t_k reset. A real invisible wall is hit again if the robot returns to it, so it stays remembered. A transient obstacle or a false alarm fades away.

**(c) Graded costs, not walls.** Marks are published as two Nav2 costmap-filter masks:
- a **keepout mask** with cost c = c_max·b_k, which is **lethal only if h_k ≥ 2** (confirmed) and otherwise high but passable. A single unconfirmed mark can therefore never make the goal unreachable.
- a **speed mask** that limits speed to 0.3 m/s near decayed or unconfirmed marks. If the robot re-tests a remembered spot, a repeated contact is gentle (low speed) and is quickly detected by C1.

**Implementation.** All nodes are Python (`rclpy`): `imu_residual_node`, `collision_detector_node`, `reaction_node`, `collision_memory_node`. Memory is injected only through Nav2's standard costmap filters, so the method is **planner-agnostic** and needs no Nav2 changes.

**Expected improvements over [1]:**
1. A false-alarm rate fixed **before deployment** and verified on test worlds.
2. Detection of diagonal and low-speed contacts.
3. Smaller direction and contact-point errors on a non-spherical body.
4. As few repeated collisions on static glass as permanent memory, but **no phantom walls** with transient obstacles or false alarms.

## 5. Research Questions and Evaluation Plan

**RQ1 – Detection.** Does residual-based, calibrated detection (C1) achieve a higher true-positive rate than Air Bumper's fixed per-axis threshold **at the same false-alarm rate**, across speeds and floors? Does the calibrated rate hold on unseen terrain?
**RQ2 – Estimation.** How much does footprint-aware, residual-based estimation reduce direction and contact-point errors?
**RQ3 – Memory.** Does evidence-aware forgetting memory (C2) keep repeated collisions with static invisible obstacles as low as permanent memory, while avoiding phantom-wall failures with transient obstacles and false alarms?
**RQ4 – System.** Does the full system improve task success, collisions and time-to-goal over the baselines?

**Baselines.**

| ID | Detection | Memory | Represents |
|---|---|---|---|
| **B0** | none (Nav2 + its LiDAR-based collision monitor and default recoveries) | none | Standard Nav2 |
| **B1** | **Air Bumper [1]**, reimplemented (a* = 20 m/s², and a\* re-tuned on calibration worlds for fairness) | permanent lethal marks | Primary paper |
| **B2** | **Oracle bumper** (Gazebo contact sensor, perfect detection) | permanent marks | Industry bumper; isolates the effect of memory |
| **B3** | Ours C1 | permanent marks | C1 alone |
| **B4** | Air Bumper detection | Ours C2 | C2 alone |
| **Ours** | C1 | C2 | Full method |

**Metrics.**
- **Detection** (per collision event, ground truth from the contact sensor):
  - TPR
  - **false alarms per hour** of collision-free driving
  - empirical vs target false-alarm rate (calibration validity)
  - detection latency (ms)
  - ROC-AUC of the score
- **Estimation:** direction error (deg) and contact-point error (m).
- **Memory/map:** **phantom area** (m² of marked cost with no real obstacle, integrated over time), **repeated collisions** (a contact within 0.5 m of an earlier one), and **blocked-goal events** (no path because of marks).
- **System:**
  - success rate
  - collisions per run
  - time-to-goal
  - SPL [10]
  - run time of our nodes (< 5 ms per cycle)

**Experiments.**

| # | Experiment | Data | RQ |
|---|---|---|---|
| E1 | Offline detection: ROC, TPR at matched false-alarm rate; features vs ‖r‖ only vs raw acceleration; per speed | S1+S2+S3 test logs | RQ1 |
| E2 | Calibration validity: target {0.5, 1, 5} FA/h vs measured; in-distribution vs unseen terrain | S3 + S1 collision-free segments | RQ1 |
| E3 | Estimation accuracy: sphere vs footprint; raw vs residual | S1, S2 | RQ2 |
| E4 | Memory study (closed loop): permanent vs decay-only vs evidence-aware (C2), with the oracle detector to isolate memory | S1 (static), S2 (transient) | RQ3 |
| E5 | Full comparison B0–B4 vs Ours × 3 speeds × 3 seeds | S1–S3 test (+ S4 stretch) | RQ4 |
| E6 | Ablations and sensitivity: τ values, lethal-after-h hits, speed mask on/off, LiDAR evidence on/off, window W | Calibration worlds | RQ3, RQ4 |

**Statistics.** The world is the unit of analysis. We report 95% bootstrap confidence intervals over worlds, McNemar tests for success/failure, and Wilcoxon signed-rank tests for continuous metrics, with Holm correction. All hyperparameters (including the B1 threshold) are tuned only on calibration worlds.

**Threats to validity.**
- Simulated contacts and IMU are idealized. We add IMU noise/bias and report results per noise level.
- The conformal guarantee is approximate under temporal correlation, so we report the empirical rate.
- Sim-to-real transfer is not tested (a physical robot is not required by the course).

## 6. Team Roles and Individual Contributions

| Member | Role | Responsibilities | Deliverables |
|---|---|---|---|
| [Member 1] | **Simulation & Scenarios Lead** | Docker/ROS 2/Nav2/Gazebo bring-up; Jackal with IMU, LiDAR, contact sensor; invisible-obstacle models; S1–S4 generators; episode runner, logging, ground-truth labels | Scenario suite, dataset, run scripts |
| [Member 2] | **Detection & Estimation Lead (C1)** | Command-response model, residual node, features, logistic regression, conformal calibration, footprint contact estimation; E1–E3 | `collision_detector_node`, detection results |
| [Member 3] | **Reaction & Memory Lead (C2)** | Diff-drive reaction node with `twist_mux`, collision-memory node, keepout/speed masks, LiDAR evidence check; E4, E6 | `reaction_node`, `collision_memory_node`, memory results |
| [Member 4] | **Baselines & Evaluation Lead** | Air Bumper reimplementation (B1), B0/B2–B4 configs, metrics, batch experiments E5, statistics, figures | Baselines, result tables, statistical analysis |

Everyone writes the report section and presents the part for their own module. Contributions are documented through per-member git history and a weekly `CONTRIBUTIONS.md`.

**Timeline:**

| Week | Milestone |
|---|---|
| 1 (11–16 Nov) | Stack running. ⚠️ Verify invisible-obstacle trick and IMU collision spikes |
| 2 (17–23 Nov) | Scenarios S1–S3 and B1 working |
| 3 (24–30 Nov) | Data collection, command-response model |
| 4 (1–7 Dec) | C1 + calibration, E1–E3 |
| 5 (8–14 Dec) | C2, E4 |
| 6 (15–21 Dec) | E5, E6, statistics |
| 7 (22–28 Dec) | Report and slides |
| 30 Dec | Presentation |

**Minimum deliverable:** B1 + C1 + E1–E3. **Target:** + C2 + E4–E5. **Stretch:** S4 BARN-Glass, low-speed contact study.

## References

[1] R. Wang, Z. Guo, Y. Chen, X. Wang, and B. M. Chen, "Air Bumper: A collision detection and reaction framework for autonomous MAV navigation," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2024, pp. 15735–15741, doi: 10.1109/ICRA57147.2024.10611410.
[2] R. Wang *et al.*, "Air Bumper," GitHub repository, 2024. [Online]. Available: https://github.com/ryrobotics/air_bumper
[3] S. Haddadin, A. De Luca, and A. Albu-Schäffer, "Robot collisions: A survey on detection, isolation, and identification," *IEEE Trans. Robot.*, vol. 33, no. 6, pp. 1292–1312, 2017.
[4] S. Macenski, F. Martín, R. White, and J. Ginés Clavero, "The Marathon 2: A navigation system," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2020, pp. 2718–2725.
[5] D. V. Lu, D. Hershberger, and W. D. Smart, "Layered costmaps for context-sensitive navigation," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2014, pp. 709–715.
[6] N. Koenig and A. Howard, "Design and use paradigms for Gazebo, an open-source multi-robot simulator," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2004, pp. 2149–2154.
[7] D. Perille, A. Truong, X. Xiao, and P. Stone, "Benchmarking metric ground navigation," in *Proc. IEEE Int. Symp. Safety, Security, and Rescue Robotics (SSRR)*, 2020, pp. 116–121.
[8] V. Vovk, A. Gammerman, and G. Shafer, *Algorithmic Learning in a Random World*. New York, NY, USA: Springer, 2005.
[9] A. N. Angelopoulos and S. Bates, "Conformal prediction: A gentle introduction," *Found. Trends Mach. Learn.*, vol. 16, no. 4, pp. 494–591, 2023.
[10] P. Anderson *et al.*, "On evaluation of embodied navigation agents," arXiv:1807.06757, 2018.
[11] S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild," *Science Robotics*, vol. 7, no. 66, eabm6074, 2022.
[12] B. He, G. Chen, W. Wang, J. Zhang, C. Fermüller, and Y. Aloimonos, "Interactive-FAR: Interactive, fast and adaptable routing for navigation among movable obstacles in complex unknown environments," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2024, pp. 5402–5409, doi: 10.1109/IROS58592.2024.10802169.
[13] N. Mohammad, J. Higgins, and N. Bezzo, "A GP-based robust motion planning framework for agile autonomous robot navigation and recovery in unknown environments," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2024, pp. 2418–2424, doi: 10.1109/ICRA57147.2024.10610382.
