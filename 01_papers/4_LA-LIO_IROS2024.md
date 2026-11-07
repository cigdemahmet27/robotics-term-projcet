# Paper 4 – LA-LIO: Localizability-Aware LiDAR-Inertial Odometry (IROS 2024)

> J. Huang, Y. Zhang, Q. Xu, S. Wu, J. Liu, G. Wang, W. Liu, "LA-LIO: Robust Localizability-Aware LiDAR-Inertial Odometry for Challenging Scenes," *IROS 2024*, pp. 10145–10152. doi:10.1109/IROS58592.2024.10802825
> Northeastern University (China) + Jiangsu Shuguang Optoelectronics. No public code found.

## 1. The problem in plain words

A robot must know **where it is** (localization / odometry) before it can plan. LiDAR odometry works by matching the current 3D scan to the map built so far. It **fails** in two situations:
1. **Aggressive motion:** the LiDAR spins and collects points over ~0.1 s. If the robot shakes or turns fast during that time, the scan is **distorted** (like a blurry photo), and matching breaks.
2. **Geometric degeneracy:** in a long, featureless corridor or a big empty hall, the scan looks **the same** no matter where you are along some direction (e.g., along the corridor). Matching can't tell how far you moved, so the position **drifts**.

When localization fails, everything downstream (map, planner) fails too. LA-LIO **detects degeneracy** and **adapts** the estimator.

## 2. Background you need

| Concept | Simple explanation |
|---|---|
| **Odometry / SLAM** | Estimating your own motion (odometry), plus building a map at the same time (SLAM) |
| **LIO (LiDAR-Inertial Odometry)** | Fuses LiDAR scans with IMU readings. The IMU predicts motion between scans, and the LiDAR corrects it |
| **Point-to-plane residual** | For each scan point, find the nearby map plane; residual = distance from the point to that plane. Minimizing all residuals aligns the scan |
| **Normal vector** | The direction perpendicular to a surface. A point-to-plane residual only constrains motion *along the normal* |
| **Degeneracy** | If all normals point the same way (e.g., only the floor is visible), motion perpendicular to them is unconstrained, so it drifts |
| **PCA / eigenvalues** | Principal Component Analysis: finds the main directions of spread of a set of vectors. The eigenvalues say how much spread there is in each direction |
| **Condition number** | Ratio of the smallest to the largest eigenvalue. Small = very uneven = degenerate |
| **ESKF (Error-State Kalman Filter)** | A Kalman filter estimating the *error* of a nominal state. Standard in FAST-LIO2 |
| **Kalman gain / covariance R** | How much to trust each measurement. Large R (covariance) = trust less |
| **FAST-LIO2, LIO-SAM, Point-LIO, Faster-LIO** | Popular open-source LIO systems, used as baselines |
| **APE (Absolute Pose Error)** | RMSE between the estimated trajectory and ground truth |
| **Hysteresis** | Using two thresholds (enter at t1, leave at t2) so the system doesn't flicker between states |

## 3. Method, step by step

**Pipeline (Fig. 2):** LiDAR + IMU → **degeneration detector** → *good*: high-frequency sub-frame odometry; *bad*: degeneration-processing odometry with **adaptive weights** → shared ESKF propagation/update.

### 3.1 Degeneration detector (Sec. III-A) ⭐
- Collect the unit normal vectors of all matched planes in a frame: N_pre = [n₁ … n_m] (Eq. 1).
- Add their opposites as well (a plane constrains both ways): N = [N_pre, −N_pre] (Eq. 2). Picture all normals as points on a unit **"constraint sphere"**.
- Covariance C = (1/2m) N Nᵀ (Eq. 3) → eigenvalues λ₁ ≤ λ₂ ≤ λ₃ (Eq. 4).
- **Degeneracy number** k = λ_min / λ_max (Eq. 5). A uniform sphere gives k ≈ 1 (good). Normals clustered in one direction give k ≈ 0 (degenerate).
- **Two thresholds**, t1 (enter degenerate mode) < t2 (exit), give **hysteresis**: "degeneration occurs continuously", so this avoids rapid switching.
- Cost: **0.11 ms**, reusing normals already computed.

### 3.2 Sub-frame generation (Sec. III-B), for aggressive motion
- Treat the LiDAR as a **stream**: split each scan into **sub-frames**, one per IMU sample (e.g., 15 per scan), and update the state after each one. The odometry rate rises to the IMU rate, so distortion and IMU drift shrink.

### 3.3 Adaptive weights (Sec. III-C), for degenerate scenes
- Divide the constraint sphere into latitude/longitude patches (Fig. 6).
- Weight of the points in patch i: w_i = area_i / (#points in patch i) (Eq. 7). **Rare directions get high weight, crowded directions get low weight.** The few constraints that "see" the missing direction are amplified.
- Convert the weights into per-point measurement covariance R (Eq. 8) and use it in the Kalman gain (Eq. 14).

### 3.4 Propagation & update (Sec. III-D/E)
- Standard ESKF with an 18-dim error state (position, velocity, rotation, IMU biases, gravity) (Eqs. 9–16), as in FAST-LIO2.

## 4. Experiments & results

- **Datasets:** M2DGR (street), Newer College (incl. "dynamic spinning", 183°/s), HILTI 2022 Exp07 Long Corridor, the FAST-LIVO degeneration dataset, and their own Livox recordings.
- **Normal scenes (Table I):** accuracy comparable to FAST-LIO2 etc., not clearly better (as the authors admit).
- **Aggressive motion:** sharper maps than FAST-LIO2 / Faster-LIO (qualitative, Fig. 7–8).
- **Degenerate scenes:** the others drift or lose tracking, while LA-LIO keeps a consistent map and returns to its start (qualitative, Fig. 9).
- **Runtime (Table II):** ~34 ms per frame vs 28 ms for FAST-LIO2, still real time.

## 5. Strengths

- ✅ Cheap, interpretable degeneracy measure (eigenvalues of normals)
- ✅ Hysteresis switching between two estimator modes
- ✅ Handles two failure sources (fast motion + degeneracy)

## 6. Limitations (critical analysis)

| # | Limitation |
|---|---|
| 1 | Thresholds t1, t2 are **still hand-set**, even though the paper criticizes earlier work for thresholds "lacking physical interpretation" |
| 2 | Degenerate-scene results are **qualitative only** (map pictures, no APE numbers) |
| 3 | Detection never reaches the **planner**: no navigation-level reaction (e.g., slow down, avoid corridors) |
| 4 | No detection metrics (how often degeneracy is detected correctly or falsely) |
| 5 | 3D LiDAR + C++ estimator, so it is heavy to port; no public code |

## 7. What we can implement / take from this paper

| Idea | How we use it | Status |
|---|---|---|
| ⭐ **Hysteresis (two thresholds)** | Our alarm turns ON at ψ̂ and OFF at a lower ψ̂′, so it doesn't chatter between stop and go | ✅ Adopted |
| "Detect, then switch to a safer mode" pattern | The same pattern as our monitor → recovery switch | ✅ Conceptual |
| Eigenvalue-based degeneracy score | ⚠️ *Stretch / future work:* a 2D version on our laser scan as an extra feature ("is localization about to fail here?") | ⚠️ Optional |
| Full LIO system | ❌ Out of scope. We use ground-truth odometry in sim to **isolate planning failures** | ❌ |

**How we cite it:** "As in LA-LIO [4], we use hysteresis between two thresholds to avoid rapid mode switching; unlike [4], our thresholds are calibrated from data rather than hand-set." And in future work: "localization degeneracy [4] is a further failure source that our predictor could incorporate."
