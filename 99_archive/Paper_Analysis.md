# Analysis of the 4 Candidate Papers (team topic: Failure Detection & Recovery)

> Read in full on 07.11.2026. All four are on the approved list and fall within 2024–2026 ✅
> ⚠️ Check citation counts on Google Scholar before submitting ("well-cited" is a requirement). I could not verify them offline.

## Quick comparison

| # | Paper | Venue | Topic fit (F.D.&R.) | ROS 2 + sim feasibility | Code | Verdict |
|---|---|---|---|---|---|---|
| 1 ⭐ | Mohammad, Higgins, Bezzo, *A GP-based Robust Motion Planning Framework for Agile Autonomous Robot Navigation and Recovery in Unknown Environments* | ✅ ICRA 2024, pp. 2418–2424 | ✅✅ Proactive failure **prediction + recovery** is the whole paper | ✅ Ground robot (Jackal) + 2D LiDAR + Gazebo → maps onto Nav2 directly | ⚠️ No public repo found (only the [project page](https://www.bezzorobotics.com/nm-icra24)) → we reimplement (small: 2-feature GP + sampler) | **PRIMARY** |
| 2 | Wang et al., *Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation* | ✅ ICRA 2024, pp. 15735–15741 | ✅ **Reactive** collision detection + recovery | ⚠️ MAV + PX4 SITL + 3D LiDAR + GPU mapper: heavy | ✅ github.com/ryrobotics/air_bumper (ROS 1) | Secondary: borrow the *collision-aware map* idea |
| 3 | Lu, Xu, Wang, Hawes, Xiao, *Decremental Dynamics Planning for Robot Navigation* | ✅ IROS 2025, pp. 4559–4565 | ⚠️ A planning paper. Recovery is only a speed-threshold FSM | ✅ Jackal + BARN | ✅ github.com/yuanjielu-64/barn_challenge_lu (ROS 1) | Secondary: source of the **BARN benchmark + score metric** |
| 4 | Huang et al., *LA-LIO: Robust Localizability-Aware LiDAR-Inertial Odometry for Challenging Scenes* | ✅ IROS 2024, pp. 10145–10152 | ⚠️ Detects *localization* degeneracy. No navigation-level recovery | ⚠️ 3D LiDAR + IMU, C++ ESKF; evaluated on bags | ❌ not found | Secondary: borrow the *hysteresis thresholds* (t1 < t2); localization failure = future work |

---

## 1 ⭐ GP-based proactive failure prediction & recovery (ICRA 2024): primary

**What it does.** A safe-corridor planner (JPS front-end → FASTER/MINVO Bézier back-end → MPC tracker) sometimes cannot find a trajectory (*back-end failure*), especially when it discovers occluded walls or dead ends at speed. The paper:
1. Trains a **GP regression** in simulation (Gazebo, Jackal, 10 Poisson-forest worlds) that maps 2 hand-picked features, **minimum time-to-intersect corridor walls `t_C`** and **number of polytopes `|C|`**, to P(back-end failure). It uses one GP per `|C|` value.
2. Evaluates the GP on the MPC's predicted future states. Risk **ρ = Σᵢ μᵢ** (the expected number of failing states). If ρ > ψ (ψ = 3, hand-set), the robot stops.
3. **Recovery**: hit-and-run sampling of candidate positions in a polytope around the robot, assumed at rest. It picks argmin of predicted failure and drives there with a go-to-goal MPC, then resumes.
4. Results: success-rate bars over 4 room worlds × 30 runs (120 sims), plus Jackal and Spot hardware demos. Sim-to-real works without retraining.

**Limitations we can attack (all from the text):**

| # | Limitation | Evidence in paper | Why it matters |
|---|---|---|---|
| L1 | **Planner-specific features.** `t_C`, `|C|` only exist for safe-corridor planners | Sec. IV-B/C. Only back-end failures are modeled | Most deployed robots (Nav2: DWB, MPPI) have no corridors, so the method cannot be reused |
| L2 | **GP uncertainty is thrown away.** Only the posterior mean μ* is used (Eq. 10); σ* is computed (Eq. 9) but ignored | Eq. 10 | Out-of-distribution states fall back to the prior mean, so the predictor is *confidently wrong* exactly when the environment is new |
| L3 | **Uncalibrated, scale-dependent threshold.** ρ = Σ μᵢ grows with the number of MPC horizon samples. ψ = 3 is hand-tuned, with no false-alarm guarantee | Eq. 12, Sec. VI | Retune for every robot or horizon. No principled trade-off between safety and false stops |
| L4 | **Goal-blind, memoryless recovery.** The recovery point minimizes risk only. It ignores progress to the goal and does not remember where failure happened | Eq. 13. The paper itself warns that retracing "causes oscillatory behavior" | Can pick points behind the robot, re-enter the same trap, and loop |
| L5 | **Weak evaluation.** Only success rate vs. "no method". No TPR/FPR, lead time, false-stop rate, or baseline detectors/recoveries | Fig. 6 | Cannot tell whether gains come from good prediction or just from stopping often |
| L6 | Static obstacles only. Multiple GPs per `|C|` | Sec. VII (future work) | Dynamic scenes are untested |

**Industry problem:** warehouse and service AMRs running Nav2 recover *reactively*. The BT recoveries (clear costmap, spin, back-up, wait) fire only **after** the controller has already failed. The BARN challenge showed no team could navigate clutter collision-free. Proactive and calibrated failure prediction is a real gap.

## 2 Air Bumper (ICRA 2024)

**What it does:** IMU-only collision detection for a caged MAV (|a_x| or |a_y| > a* = 20 m/s², sliding window N = 10). It estimates the collision direction and contact point (sphere-cage assumption), applies an EDT-weighted bounce-away position command straight to PX4, and inserts a **collision point cloud into the volumetric map** so the planner avoids the unseen obstacle (e.g., glass) afterwards.

**Limitations:** a fixed hand-tuned threshold (false alarms in aggressive flight, misses soft contacts); purely **reactive** (the collision already happened); collision marks are **permanent** ("never cleared by onboard sensors"), which is wrong for moving or transient obstacles; a spherical cage assumption; a 5 s re-plan window; evaluation is 1 sim world + 10 real trials with no detection metrics.
**Use for us:** the *collision-aware map* idea becomes our **decaying failure-memory layer** (fixing the permanent-mark limitation). It also gives us a reactive-detector baseline.

## 3 Decremental Dynamics Planning, DDP (IROS 2025)

**What it does:** sampling-based planners (DWA, MPPI, Log-MPPI) roll out with high-fidelity dynamics early and coarser dynamics later (integration step grows per Eq. 4, collision points shrink per Eq. 5). 2nd place in BARN 2025. Evaluated on the **300 BARN worlds** at 1.0/1.5/2.0 m/s with success, collision, timeout, and the BARN score.
**Fit:** not a failure-detection paper. Its own "recovery" is a velocity-threshold mode switch. Its planners are evaluated "without backup planners and recovery behaviors". Also the newest paper of the four (2025), so the fewest citations.
**Use for us:** the **benchmark protocol** (BARN worlds, multiple max speeds to induce failures, the BARN score formula) and evidence that DWA/MPPI fail much more often at higher speeds. That is how we will generate failure data.

## 4 LA-LIO (IROS 2024)

**What it does:** detects LiDAR geometric degeneracy by PCA of point-to-plane normals on a "constraint sphere" (condition number λmin/λmax with hysteresis thresholds t1 < t2). It switches between a high-rate sub-frame ESKF and an adaptive-weight ESKF that up-weights rare constraint directions.
**Limitations:** thresholds are still hand-set, although the paper criticizes prior work for exactly that; degenerate-scene results are **qualitative only** (map images, no APE); detection only feeds the estimator and never tells the planner, so there is no navigation-level recovery; 3D LiDAR + heavy C++.
**Use for us:** the hysteresis idea (avoids alarm chattering). Localization failure as a failure source is **out of scope** (we use ground-truth odometry to isolate planning failures) and is listed as future work.

---

## Recommendation

**Primary = Paper 1.** It is literally "failure prediction + recovery", the instructor's research area, an ICRA paper, a ground robot that maps 1:1 onto ROS 2 Nav2 + Gazebo, CPU-only (a GP needs no GPU), and it has clear, citable limitations (L1–L5).
Papers 2, 3, 4 become **secondary references**, each contributing one concrete piece (failure memory, benchmark/metric, hysteresis).

➡️ The full draft is in `Proposal_Draft.md`.

⚠️ This replaces the old Option A (Sentinel / manipulation) in `Project_Options.md`. The team chose these navigation papers instead.
