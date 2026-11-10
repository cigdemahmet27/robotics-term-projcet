# Know Before You Fail: Calibrated, Uncertainty-Aware Prediction of and Recovery from Navigation Failures in ROS 2

*YZV406E Robotics, Term Project Proposal (Round 1 draft), Fall 2026–27*
*Team: [Member 1], [Member 2], [Member 3], [Member 4]*

> ⚠️ Draft notes (delete before submitting): target ≤ 4 pages + 1 page of references. Fill in names. Check citation counts for [1] on Google Scholar. The bracketed `[n]` refer to the reference list.

---

## 1. Problem Description & Primary Reference Paper

Mobile robots in warehouses, hospitals and offices navigate partially unknown, cluttered spaces at speed. Their local planners regularly fail: no feasible trajectory exists, the robot hits an obstacle that was occluded a moment ago, or it gets stuck in a dead end. In the standard ROS 2 navigation stack (Nav2 [7]), failure handling is **reactive**. Recovery behaviors (clear costmap, spin, back up, wait) run only *after* the controller has already failed, often when the robot is too close to an obstacle to escape gracefully. The BARN challenge showed how hard this is in practice: no team navigated its cluttered worlds without collisions [6].

**Problem.** *Predict, before it happens, that the local planner is about to fail, with a calibrated and bounded false-alarm rate. Then move the robot to a state from which it can make progress again, without falling back into the same trap.*

**Primary reference paper [1].** N. Mohammad, J. Higgins, N. Bezzo, *"A GP-based Robust Motion Planning Framework for Agile Autonomous Robot Navigation and Recovery in Unknown Environments,"* **ICRA 2024**. The authors train a Gaussian Process (GP) in simulation that maps two hand-crafted features to the probability that a safe-corridor trajectory optimizer fails: minimum time-to-intersect the corridor, `t_C`, and corridor length, `|C|`. They evaluate it on the MPC-predicted future states and stop when the expected number of failing states exceeds a threshold. The same GP then selects a low-risk recovery point. The method is trained in simulation only and transfers to a Jackal UGV and a Spot quadruped.

**Why this paper.** (i) It addresses exactly our topic, *proactive* failure detection **and** recovery, unlike most work, which does only one of the two. (ii) It comes from a team that placed 2nd in the BARN challenge, and it appeared at a top venue (ICRA). (iii) It is lightweight (a GP over a few features, so no GPU), which makes a careful extension and a rigorous evaluation feasible within one term. (iv) Its weaknesses (Sec. 3) are clear and fixable.

**Secondary papers** supply concrete pieces. *Air Bumper* [2] (ICRA 2024) contributes collision-aware mapping. *DDP* [3] (IROS 2025) contributes the BARN benchmark protocol and the score metric. *LA-LIO* [4] (IROS 2024) contributes hysteresis-based switching between detector states.

## 2. Environment, Benchmark, or Dataset

| Component | Choice | Justification |
|---|---|---|
| Middleware | **ROS 2 Jazzy + Nav2** in Docker (`ros:jazzy`, Ubuntu 24.04 image) | Nav2 has not yet been released for ROS 2 Lyrical (Ubuntu 26.04). Docker gives every team member an identical, free setup |
| Simulator | **Gazebo Harmonic** (`ros_gz`) | Free, the official ROS 2 simulator, GPU-optional |
| Robot | Clearpath **Jackal** (`clearpath_gz`) with 270° 2D LiDAR | Same platform as [1] and [3], so results are comparable |
| Planners under test | Nav2 global planner (NavFn/Smac) + local controllers **DWB** (≈DWA [13]) and **MPPI** [14] | Two very different controllers let us test whether the predictor is *planner-agnostic* |
| Benchmark worlds | **W1:** 300 BARN-style worlds [5] (cellular-automaton obstacle grids, regenerated as SDF by our script). **W2:** room worlds as in [1] (1 m / 2 m doors, sparse/dense clutter, *occluded dead ends*). **W3 (out-of-distribution):** narrower gaps, unseen obstacle shapes, moving obstacles | W1 = a standard, well-understood difficulty distribution. W2 = reproduces [1]'s failure scenario. W3 = tests generalization, where uncertainty should matter |
| Failure induction | Max speed ∈ {0.5, 1.0, 1.5, 2.0} m/s (following [3]) | Higher speed means more planner failures, so we get enough positive labels |
| Compute | Laptop CPU + RTX 4060 (not required) | GP training and inference run on the CPU. **Zero cost** |

**Failure ground truth (logged automatically per control cycle at 10 Hz).** A *failure event* is (a) the controller returns `FAILED` / "no valid trajectory", (b) a **collision** (Gazebo contact sensor, or LiDAR range < footprint margin), or (c) **stuck**, meaning the Nav2 progress checker fails (< 0.1 m in 5 s). A time step is labeled *positive* if a failure occurs within the horizon `T_F` (default 2 s). We use ground-truth odometry so that planning failures are isolated from localization failures. Localization degeneracy [4] is explicitly out of scope.

**Data split.** Train on BARN worlds 0–179, calibrate on 180–239, test on 240–299 + W2 + W3. Each run is repeated with 3 seeds.

## 3. Challenges & Limitations of the Primary Paper

- **L1 – Planner-specific features.** `t_C` and `|C|` exist only for safe-corridor planners, and only the optimizer's (back-end) failures are modeled. The method cannot be applied to sampling-based controllers (DWB, MPPI) that power most deployed robots, and it ignores collisions and stuck events.
- **L2 – The GP's uncertainty is discarded.** Failure probability is taken as the posterior mean only (Eq. 10 of [1]), although the variance is computed. Far from the training data a GP reverts to its prior mean, so the detector becomes *confidently wrong precisely in unfamiliar environments*, which is when failures are most likely.
- **L3 – Uncalibrated, scale-dependent alarm threshold.** Risk is the *sum* of per-state probabilities over the MPC horizon, so its value changes with horizon length and discretization. The threshold (ψ = 3) is hand-tuned and gives no guarantee on the false-stop rate. Every false stop costs time, and every miss risks a crash.
- **L4 – Partly uninformative, progress-blind, memoryless recovery.** Recovery candidates are evaluated *at rest*. With zero velocity, the time-to-intersect feature equals its cap for every candidate, so the ranking relies mainly on the integer corridor size and produces many ties. The selection minimizes predicted risk only, so it ignores how much progress toward the goal is given up. Nothing remembers where the failure happened, so the robot can re-plan into the same trap and oscillate. The paper itself notes this risk for naive back-tracking.
- **L5 – Limited evaluation.** Results are success rates of "with vs. without" over 120 runs in 4 worlds. There are no detection metrics (TPR/FPR, lead time, false-stop rate) and no comparison to other detectors or to standard recovery behaviors. So it is unclear whether gains come from *accurate prediction* or simply from *stopping often*.

These matter because, in industry, a detector that is planner-specific, uncalibrated, and that stops too often (or too rarely) will not be adopted. Air Bumper [2] and LA-LIO [4] share the problem of hand-set thresholds, which suggests that calibrated triggering is an open issue across the field.

## 4. Proposed Method & Contributions

We build a ROS 2 *failure monitor + recovery* layer on top of Nav2 that works with any local controller.

```
 Gazebo (Jackal, LiDAR) ─► Nav2 (planner + DWB/MPPI controller) ─► /cmd_vel
          │ /scan, /odom            │ /local_plan (predicted states)
          ▼                         ▼
   feature_node ───────────► risk_node (sparse GP: μ, σ  →  calibrated risk)
                                    │ alarm (with hysteresis)
                                    ▼
                    recovery_node (Nav2 BT action) ─► failure-memory costmap layer
```

**C1 – Planner-agnostic, uncertainty-aware, calibrated failure prediction (addresses L1–L3).**
- *Features* use only the LiDAR scan, the robot state, and the controller's predicted local trajectory, which both DWB and MPPI publish, so no corridor is needed. (f1) minimum time-to-collision along the predicted trajectory, the scan-based analogue of `t_C`; (f2) minimum clearance along the predicted trajectory; (f3) global-path tortuosity over the next *L* m, the analogue of `|C|`; (f4) linear speed; (f5) free-space and unknown-space ratio in the forward sector, which captures occlusion.
- *Model:* a single sparse GP classifier with an ARD-RBF kernel [8], [15] (GPyTorch, CPU), replacing [1]'s one-GP-per-`|C|`. For each predicted state *xᵢ* it outputs μᵢ and σᵢ.
- *Risk:* the GP's latent output for state *xᵢ* is N(mᵢ, sᵢ²). We use the uncertainty-inflated probability pᵢ⁺ = Φ(mᵢ + β sᵢ), where Φ is the standard normal CDF and β = 0 recovers mean-only behavior. We then take the **normalized** risk ρ̄ = (1/K) Σᵢ pᵢ⁺. It is independent of the horizon discretization, and it grows on out-of-distribution inputs.
- *Calibrated trigger:* we choose the threshold ψ̂ by **split conformal calibration** [9], [10] as the (1−α) quantile of ρ̄ over *non-failure* windows of the calibration worlds. Under exchangeability this bounds the false-alarm rate by α (e.g., α = 0.05). To limit temporal correlation we calibrate on one randomly chosen window per episode segment. **Hysteresis** (alarm on at ψ̂, off at a lower ψ̂′), borrowed from LA-LIO [4], prevents chattering.

**C2 – Progress-aware recovery with decaying failure memory (addresses L4).**
When the alarm fires, a Nav2 Behavior-Tree action stops the robot and samples N candidate poses in free space within radius R. Each candidate is scored by
`J(x) = p⁺(x) + λ · d_goal(x) / d_goal(x₀) + γ · M(x)`.
Here p⁺ is evaluated at a realistic departure velocity, not at rest. d_goal is a cost-to-go field toward the goal (one wavefront computation), which rewards progress. M is a **failure-memory layer**. Inspired by Air Bumper's collision-aware map [2], this layer marks regions where failures were predicted or observed. Unlike [2], marks **decay over time**, so transient or moving obstacles are not remembered forever. The layer is also published to Nav2's global costmap (via a costmap-filter mask), so the global planner routes around the trap and the robot does not oscillate. A recovery budget (3 attempts) falls back to Nav2's default recoveries.

**How this differs from [1].** It works with any controller instead of one corridor planner. It handles three failure types instead of one. It uses mean + variance instead of the mean only. It uses a calibrated α-level trigger instead of a hand-tuned ψ. Recovery is goal- and memory-aware instead of risk-only. **Expected improvements:** a higher success rate and fewer collisions than Nav2 defaults and than [1]'s reimplementation, at a bounded false-stop rate, with the largest gains on out-of-distribution worlds (W3).

## 5. Research Questions & Evaluation Plan

- **RQ1 (generality):** Can scan-based features predict Nav2 controller failures (DWB and MPPI) as well as [1]'s corridor-style features? Does a predictor trained on DWB transfer to MPPI?
- **RQ2 (uncertainty & calibration):** Does uncertainty-inflated risk with a conformal threshold keep the empirical false-alarm rate ≤ α, while improving recall and lead time over mean-only risk with a fixed threshold, especially on out-of-distribution worlds?
- **RQ3 (recovery):** Does progress-aware recovery with failure memory raise task success and reduce repeated failures and oscillation compared with [1]'s min-risk recovery and Nav2's default recoveries? What does it cost in traversal time?

**Baselines.**
| ID | Detector | Recovery |
|---|---|---|
| B0 | none (reactive Nav2) | Nav2 default BT (clear, spin, back-up, wait) |
| B1 | time-to-collision threshold (non-learned, reactive) | Nav2 default |
| B2 | **[1] reimplemented**: GP mean, Σμ risk, fixed ψ | [1]'s min-risk sampling |
| B3 | same features as ours + logistic regression / random forest, conformal-calibrated | ours |
| **Ours** | C1 | C2 |

**Metrics.** *Detection (offline, on logged test episodes):* AUROC, AUPRC, TPR at the target FPR, empirical false-alarm rate vs. α (calibration validity), and lead time (seconds from alarm to failure). *Navigation (online):* success, collision, timeout and stuck rates, the BARN score [3], [6], traversal time, alarms per episode, recovery success (no failure within 10 s after recovering), and monitor latency per cycle (must stay < 10 ms).

**Experiments.**
1. *Detection study (RQ1, RQ2):* compare feature sets and models on W1-test, W2 and W3, both within controller (DWB→DWB) and across controllers (DWB→MPPI).
2. *Closed-loop study (RQ2, RQ3):* B0–B3 vs. Ours on all test worlds × 4 speeds × 3 seeds (≈ 2,000+ runs, scripted headless).
3. *Ablations:* β ∈ {0, 1, 2} (β = 0 is mean-only); α ∈ {0.01, 0.05, 0.1}; horizon `T_F`; each feature removed in turn; recovery with/without progress term, with/without failure memory, with/without decay.
4. *Statistics:* 95% bootstrap confidence intervals; paired tests across worlds (McNemar for success, Wilcoxon for time).

**Timeline.** Wk 1–2 (to 24.11): Docker + Nav2 + Gazebo, world generator, logging and labeling. Wk 3–4: data collection, B2 reimplementation, C1. Wk 5–6: C2 + BT integration. Wk 7 (to 25.12): full experiments, ablations. 30.12: presentation.
**Risks & mitigations.** Too few failures → raise the speed limit or density. GP scaling → sparse GP / subsampling. Calibration windows are time-correlated → subsample per episode and report empirical coverage honestly. BARN conversion issues → the regenerated worlds use the same cellular-automaton procedure [5].

## 6. Team Roles & Individual Contributions

| Member | Main responsibility | Deliverables |
|---|---|---|
| [Member 1] | **Simulation & benchmark:** Docker image, Gazebo/Jackal/Nav2 bring-up, world generator (W1–W3), headless episode runner, automatic failure labeling | `sim/` package, world sets, dataset |
| [Member 2] | **Failure prediction (C1):** feature node, sparse-GP training, uncertainty-inflated risk, conformal calibration, hysteresis, offline detection evaluation (RQ1–RQ2) | `risk_node`, detection results |
| [Member 3] | **Recovery & Nav2 integration (C2):** BT recovery action, candidate sampling and scoring, decaying failure-memory layer, fallback logic | `recovery_node`, BT XML, costmap layer |
| [Member 4] | **Baselines, experiments & analysis:** B0–B3 (incl. reimplementation of [1]), experiment scripts, ablations, statistics, plots | results tables, figures |

All members write the report sections for their own module. Integration and the presentation are shared. (If the team has 3 members, Member 4's tasks are split between Members 2 and 3.)

---

## References

[1] N. Mohammad, J. Higgins, and N. Bezzo, "A GP-based robust motion planning framework for agile autonomous robot navigation and recovery in unknown environments," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2024, pp. 2418–2424, doi: 10.1109/ICRA57147.2024.10610382.
[2] R. Wang, Z. Guo, Y. Chen, X. Wang, and B. M. Chen, "Air Bumper: A collision detection and reaction framework for autonomous MAV navigation," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2024, pp. 15735–15741, doi: 10.1109/ICRA57147.2024.10611410.
[3] Y. Lu, T. Xu, L. Wang, N. Hawes, and X. Xiao, "Decremental dynamics planning for robot navigation," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2025, pp. 4559–4565, doi: 10.1109/IROS60139.2025.11246580.
[4] J. Huang, Y. Zhang, Q. Xu, S. Wu, J. Liu, G. Wang, and W. Liu, "LA-LIO: Robust localizability-aware LiDAR-inertial odometry for challenging scenes," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2024, pp. 10145–10152, doi: 10.1109/IROS58592.2024.10802825.
[5] D. Perille, A. Truong, X. Xiao, and P. Stone, "Benchmarking metric ground navigation," in *Proc. IEEE Int. Symp. Safety, Security, Rescue Robot. (SSRR)*, 2020, pp. 116–121.
[6] X. Xiao *et al.*, "Autonomous ground navigation in highly constrained spaces: Lessons learned from the benchmark autonomous robot navigation challenge at ICRA 2022," *IEEE Robot. Autom. Mag.*, vol. 29, no. 4, pp. 148–156, 2022.
[7] S. Macenski, F. Martín, R. White, and J. Ginés Clavero, "The Marathon 2: A navigation system," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2020.
[8] C. E. Rasmussen and C. K. I. Williams, *Gaussian Processes for Machine Learning*. Cambridge, MA, USA: MIT Press, 2006.
[9] A. N. Angelopoulos and S. Bates, "Conformal prediction: A gentle introduction," *Found. Trends Mach. Learn.*, vol. 16, no. 4, pp. 494–591, 2023.
[10] V. Vovk, A. Gammerman, and G. Shafer, *Algorithmic Learning in a Random World*. New York, NY, USA: Springer, 2005.
[11] J. Tordesillas and J. P. How, "FASTER: Fast and safe trajectory planner for navigation in unknown environments," *IEEE Trans. Robot.*, 2021.
[12] T. Ji, A. N. Sivakumar, G. Chowdhary, and K. Driggs-Campbell, "Proactive anomaly detection for robot navigation with multi-sensor fusion," *IEEE Robot. Autom. Lett.*, vol. 7, no. 2, pp. 4975–4982, 2022.
[13] D. Fox, W. Burgard, and S. Thrun, "The dynamic window approach to collision avoidance," *IEEE Robot. Autom. Mag.*, vol. 4, no. 1, pp. 23–33, 1997.
[14] G. Williams, P. Drews, B. Goldfain, J. M. Rehg, and E. A. Theodorou, "Aggressive driving with model predictive path integral control," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2016, pp. 1433–1440.
[15] J. Hensman, N. Fusi, and N. D. Lawrence, "Gaussian processes for big data," in *Proc. Conf. Uncertainty Artif. Intell. (UAI)*, 2013.
