# 01 – Problem & Goals

## 1. The problem

A mobile robot navigating a cluttered, partially unknown space **will** sometimes fail:

| Failure type | What it looks like | How we detect it in simulation |
|---|---|---|
| **Controller failure** | The local controller can't find any valid command ("no valid trajectory") | The Nav2 controller server reports a failure |
| **Collision** | The robot touches an obstacle | Gazebo contact sensor, or LiDAR range < footprint + margin |
| **Stuck / oscillation** | The robot keeps trying but makes no progress | Nav2 progress checker: < 0.1 m in 5 s |

**Today's standard (ROS 2 Nav2)** handles these **reactively**. After the failure, a Behavior Tree runs recoveries: clear the costmap, spin, back up, wait. By then the robot is often already pressed against the obstacle or deep in a dead end. It may also loop: back up, re-plan into the same trap, fail again.

**What we want:** a **monitor** that says *"you are about to fail in the next ~2 s"* early enough to act, with a **known, bounded false-alarm rate**, and a **recovery** that puts the robot somewhere it can actually make progress, without returning to the same trap.

## 2. Why it matters (industry motivation)

- 🏭 **Warehouses / logistics AMRs:** every collision is a safety incident, and every unnecessary stop or recovery loop is lost throughput. Operators care about *both*, which is why the false-alarm rate must be controllable.
- 🏥 **Hospitals, offices, retail:** clutter changes daily (carts, boxes), so robots constantly meet layouts they haven't seen, and a detector must not become over-confident there.
- 🔧 **Integration:** most such robots run **Nav2**. A method that only works with one research planner (as in the primary paper) won't be adopted. One that plugs into any Nav2 controller can be.
- 📉 **Evidence:** the BARN challenge (ICRA 2022–2025) shows state-of-the-art planners still fail often in clutter. DDP [3] reports success dropping from ~91% to ~63% (DWA) when max speed goes from 1.0 to 2.0 m/s.

## 3. Scope

| In scope ✅ | Out of scope ❌ |
|---|---|
| Ground robot (Jackal), 2D LiDAR, differential drive | Drones, legged robots, manipulation |
| Simulation in Gazebo with ROS 2 + Nav2 | Real-robot experiments |
| Static clutter for training. Moving obstacles only as a robustness (OOD) test | Solving dynamic-obstacle navigation |
| Ground-truth odometry (to isolate *planning* failures) | Localization failures (LA-LIO-type degeneracy), which are future work |
| Nav2's DWB and MPPI controllers as the planners under test | Designing a new planner (that is what DDP does) |
| Predicting + recovering from failures | Formal safety guarantees (HJI reachability) |

## 4. Goals

| ID | Goal | Measured by |
|---|---|---|
| G1 | Build a **reproducible ROS 2 benchmark** for navigation failures (worlds, runner, automatic labels) | Runs headless, one command per experiment |
| G2 | **Reimplement** the primary paper's idea in Nav2 as a baseline (B2) | Works on both controllers |
| G3 | **C1:** a planner-agnostic, uncertainty-aware, calibrated failure predictor | AUROC, TPR at α, empirical FPR ≈ α, lead time |
| G4 | **C2:** progress-aware recovery with decaying failure memory | Success ↑, collisions ↓, repeated-failure count ↓ |
| G5 | A **rigorous comparison** with baselines + ablations | Tables with CIs and significance tests |

## 5. Hypotheses (what we expect, and will test)

- **H1 (generality):** scan-based features predict Nav2 controller failures about as well as corridor-style features predict corridor-planner failures (AUROC within a few points of each other). A model trained on DWB data transfers to MPPI with a moderate drop.
- **H2 (uncertainty):** on out-of-distribution worlds, using σ (β > 0) **increases recall and lead time** compared to mean-only (β = 0) at the same false-alarm level.
- **H3 (calibration):** with conformal thresholds, the measured false-alarm rate on test worlds stays close to the target α, whereas a fixed ψ gives an unpredictable false-alarm rate that changes with speed and horizon.
- **H4 (recovery):** the progress term + memory **reduce repeated failures** at the same location and raise success compared to min-risk recovery and Nav2 defaults, with a modest time cost.

If a hypothesis turns out false, that is still a valid, reportable result. The evaluation is designed to show it clearly.

## 6. What "success" means for the project (for us, not the robot)

- **Minimum (must-have):** benchmark + baseline B0/B2 + C1 working + detection results (RQ1, RQ2).
- **Target:** + C2 + closed-loop comparison of all baselines (RQ3).
- **Stretch:** + DWB→MPPI transfer study, moving-obstacle OOD set, a LiDAR-degeneracy feature (inspired by LA-LIO).
