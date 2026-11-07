# Paper 2 – Air Bumper (ICRA 2024)

> R. Wang, Z. Guo, Y. Chen, X. Wang, B. M. Chen, "Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation," *ICRA 2024*, pp. 15735–15741. doi:10.1109/ICRA57147.2024.10611410
> Chinese University of Hong Kong. Code: github.com/ryrobotics/air_bumper (ROS 1). Video: youtu.be/FVQGmqUTyp4

## 1. The problem in plain words

Small drones (**MAVs**, Micro Aerial Vehicles) fly through cluttered indoor spaces using a LiDAR to see obstacles. Some obstacles are **invisible to the LiDAR**: glass, transparent plastic, thin objects, or things outside its field of view. The planner then happily plans *through* them. When the drone hits one, a normal drone keeps pushing toward its target velocity, keeps hitting, and crashes.

**Air Bumper's philosophy:** "We can't avoid every collision, so let's **detect** collisions instantly, **bounce away** safely, and **remember** where the invisible obstacle is so we never hit it again."

## 2. Background you need

| Concept | Simple explanation |
|---|---|
| **IMU** | Inertial Measurement Unit: a sensor measuring acceleration (accelerometer) and rotation rate (gyroscope). Every drone has one |
| **Body frame vs world frame** | Body frame = axes attached to the drone (x forward, y left, z up). World frame = fixed axes of the room. A rotation matrix R converts between them |
| **PX4** | Open-source drone autopilot firmware. It runs the low-level controllers (cascaded PID) |
| **FAST-LIO2** | A popular LiDAR-inertial SLAM, giving the drone's position (odometry) |
| **Occupancy Grid Map (OGM)** | 3D grid of voxels (small cubes), each storing the probability that it is occupied |
| **EDT (Euclidean Distance Transform)** | For every voxel: the distance to the nearest obstacle. Its gradient ∇D points "away from obstacles" |
| **MPC** | Model Predictive Control: optimize the next few seconds of motion repeatedly |
| **Collision-resilient cage** | A protective cage around the drone so that hitting things doesn't break it |

## 3. Method, step by step

**Pipeline:** IMU → collision detection → collision estimation → (a) recovery control **and** (b) collision point cloud → collision-aware map → re-planning.

### 3.1 Collision detection (Sec. IV-A)
- A collision adds a sudden spike in acceleration.
- **Rule:** if |a_x| > a* or |a_y| > a* (for z: |a_z − g| > a*), then it is a **potential collision**. They use a* = 20 m/s².
- Several abnormal samples come from one hit, so a **sliding window** of N = 10 samples is used and the **maximum** one is taken as the representative sample.
- Only 2 parameters to tune: a* and N.

### 3.2 Collision estimation (Sec. IV-B)
- Collision acceleration = the opposite of the measured one: a_c = −a (gravity removed on z).
- **Intensity** C = |a_c| (Eq. 1). **Direction** Ĉ is computed from spherical angles θ (polar) and φ (azimuth) (Eqs. 2–4).
- **Collision point** c_c = Ĉ · l, assuming the cage is a **sphere** of radius l (Eq. 5).

### 3.3 Recovery control (Sec. V-A)
- The goal is to move away from the obstacle **immediately**, *without* waiting for the planner.
- Recovery position p_r = (w · G/‖G‖ − (1−w) · Ĉ) · R_d (Eq. 6):
  - −Ĉ = "bounce opposite to the hit"
  - G = ∇EDT = "direction away from all *known* obstacles"
  - w (Eq. 7) blends the two: when known obstacles are close, trust the EDT more
  - R_d = 2l: move one drone diameter
- This is sent directly as a position setpoint to PX4's PID controller, which is fast.

### 3.4 Collision point cloud (Sec. V-B)
- Model the contact patch as a **disc** (circle of radius r_c around the collision point, perpendicular to the hit direction) (Eq. 8). Generate points on it and transform them to the world frame.

### 3.5 Collision-aware mapping (Sec. V-C) ⭐ the idea we reuse
- The collision is treated as a **virtual sensor** ("collision sensor").
- Its points are wrapped into an oriented bounding box (OBB) and marked occupied in the map.
- **Collision marks have priority** and are **never cleared** by the LiDAR. That is on purpose: the LiDAR "sees through" glass and would otherwise erase the mark.
- The planner (GTO-MPC, Eq. 10) then avoids the invisible obstacle.

## 4. Experiments & results

- **Simulation:** Gazebo, 3 doors (2 with glass), waypoints through them. Without Air Bumper the drone crashes at the glass; with it, the drone bounces, maps the glass, and re-plans around it.
- **Real:** a 1.45 kg caged drone (Livox Mid-360 LiDAR, Xavier NX). Glass wall: detection in ~0.18 s, a safe distance of ~0.46 m after recovery, 10 trials shown. A stick hit from below/side shows 3D recovery.
- Recovery and stabilization < 1 s. The planner is given a 5 s window to re-plan.

## 5. Strengths

- ✅ Uses only the IMU, which every robot has, with no extra sensors
- ✅ Full 3D (collisions from any direction, not just horizontal)
- ✅ Detection, reaction and memory are integrated with a standard SLAM/planner stack
- ✅ Open source

## 6. Limitations (critical analysis)

| # | Limitation | Why it matters |
|---|---|---|
| 1 | **Reactive**: the collision has already happened | Fine for a caged drone, not acceptable for a heavy ground robot or near people |
| 2 | **Fixed hand-tuned threshold** a* = 20 m/s² | Aggressive maneuvers or wind cause false alarms; soft or slow contacts are missed. No detection statistics are reported |
| 3 | **Collision marks are permanent** | If the "obstacle" was a person or a door that later opens, the map stays wrong forever, and areas can become permanently blocked |
| 4 | Spherical-cage assumption for the contact point | Inaccurate for non-spherical robots |
| 5 | Planner needs up to 5 s to re-plan | Slow recovery in dynamic scenes |
| 6 | Small evaluation (1 sim world, ~10 real trials) | No TPR/FPR, no comparison with other detectors |

## 7. What we can implement / take from this paper

| Idea | How we use it | Status |
|---|---|---|
| ⭐ **"Failure as a virtual sensor" + map memory** | Our **failure-memory layer** (Contribution 2): mark places where a failure was predicted/observed so the planner avoids them | ✅ Adopted |
| Fix limitation 3 | Our marks **decay over time** (exponential forgetting) instead of being permanent | ✅ Our improvement |
| Bounce-away rule (blend "opposite of hit" + "away from known obstacles") | Inspiration for scoring recovery candidates (risk + clearance) | ✅ Inspiration |
| IMU collision detector | Cheap **reactive baseline / ground-truth labeler** for collisions in Gazebo (or we use the Gazebo contact sensor) | ⚠️ Optional |
| Drone + PX4 + 3D LiDAR stack | ❌ Not used, too heavy and off-platform | ❌ |

**How we cite it:** "Air Bumper [2] showed that remembering failure locations in the map prevents repeated collisions; we adopt this for *predicted* failures and add temporal decay to handle transient obstacles."
