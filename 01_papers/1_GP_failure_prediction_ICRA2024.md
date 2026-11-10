# Paper 1 ⭐ – GP-based Failure Prediction & Recovery (ICRA 2024)

> N. Mohammad, J. Higgins, N. Bezzo, "A GP-based Robust Motion Planning Framework for Agile Autonomous Robot Navigation and Recovery in Unknown Environments," *ICRA 2024*, pp. 2418–2424. doi:10.1109/ICRA57147.2024.10610382 · arXiv:2402.01617 · Project page: bezzorobotics.com/nm-icra24
> Authors: University of Virginia (Bezzo lab), **2nd place at the ICRA 2022 BARN navigation challenge**.

This is the **short version**. The full, from-zero explanation is in [`../99_archive/gp_direction/02_primary_paper_deep_dive/`](../99_archive/gp_direction/02_primary_paper_deep_dive/README.md) (archived: this was the primary paper until 10.11.2026).

## 1. The problem in plain words

A robot drives **fast** through a building it has **never seen**. Its sensor (LiDAR) only shows what is visible right now, so walls hidden behind obstacles appear suddenly. The motion planner recomputes a trajectory many times per second. Sometimes it **cannot find any feasible trajectory**: this is a *planner failure*. If several happen in a row while the robot moves fast, it crashes.

Existing solutions are either **reactive** (stop when something is already close, which is too late for a fast or heavy robot, and overly cautious), or **proactive but heavy** (deep networks needing lots of real-world data), and they **don't recover** afterwards.

## 2. What the paper proposes

| Step | What happens | Key idea |
|---|---|---|
| 1. Offline data | Drive a simulated Jackal through 10 forest worlds × 90 start/goal pairs, and log every planner success/failure | Failures are learned **in simulation only** |
| 2. Features | Two numbers per state: **t_C** = time until the robot would leave its current safe region at its current velocity, and **\|C\|** = number of convex regions the path needs (how twisty the path is) | Hand-chosen features that are cheap and sensor-agnostic |
| 3. Model | A **Gaussian Process** regression: features → probability of planner failure | Works with small data, < 10 ms inference |
| 4. Online prediction | Evaluate the GP on the controller's **predicted future states** (next ~T_F seconds) and add the probabilities up: **ρ = Σ P(fail)** = expected number of failures | Look ahead, not just now |
| 5. Trigger | If ρ > ψ (they use ψ = 3), **stop** | Proactive stop before the crash |
| 6. Recovery | Sample random free positions nearby, ask the GP which one is safest, drive there, resume | The same model is used for detection *and* recovery |

## 3. Experiments & results

- **Simulation:** Gazebo + ROS Noetic, Jackal with a 270° 2D LiDAR. 4 room worlds (doorways 1 m / 2 m × sparse/dense obstacles), 3 goals × 10 runs per world = 120 runs. Success rate improves with the method in every world/goal combination (Fig. 6).
- **Real robots:** Jackal (occluded obstacles around a corner) and Boston Dynamics Spot (unexpected dead end), with the model trained **only in simulation**, no retraining.
- The learned failure distribution matches the distribution in unseen test worlds (Fig. 4c–d).

## 4. Strengths

- ✅ Detection **and** recovery in one framework, with the same model
- ✅ Tiny, interpretable model (2 features), fast (< 10 ms), CPU only
- ✅ Sim-to-real without retraining, across two very different robots
- ✅ Clear failure taxonomy (front-end vs back-end failure)

## 5. Limitations (details in the deep dive)

| # | Limitation |
|---|---|
| L1 | Features exist only for **safe-corridor planners**, and only optimizer failures are modeled (not collisions or getting stuck) |
| L2 | The GP's **uncertainty (σ) is computed but never used**; only the mean is used |
| L3 | The threshold ψ = 3 is **hand-tuned**; risk is a *sum*, so its scale depends on horizon length; there is **no false-alarm guarantee** |
| L4 | Recovery candidates are assumed **at rest**, so the time feature becomes uninformative; the choice ignores **progress** to the goal and has **no memory** of failed spots, so oscillation is possible (the paper itself warns of this) |
| L5 | Evaluation reports **success rate only**: no TPR/FPR, warning lead time, false-stop rate, or baseline detectors/recoveries |
| L6 | Static obstacles only; one separate GP per value of \|C\| |

## 6. What we can implement from this paper

| Component | Implement? | Notes |
|---|---|---|
| Data collection in sim + labeling of failures | ✅ Yes | Re-done in ROS 2 Gazebo; labels extended to collision / stuck |
| TTI-style "time to boundary" feature | ✅ Adapted | Computed from the LiDAR scan instead of corridors |
| Path-complexity feature (\|C\|) | ✅ Adapted | Becomes global-path tortuosity |
| GP model | ✅ Yes | Reimplemented exactly (mean-only, Σ risk) as **baseline B2**, then improved |
| Risk over predicted future states | ✅ Yes | Using the Nav2 controller's local plan |
| Threshold trigger | ✅ Yes, then replaced | Fixed ψ (baseline) vs calibrated ψ̂ (ours) |
| Sampling-based recovery | ✅ Yes, then improved | Min-risk (baseline) vs progress + memory (ours) |
| FASTER/MINVO safe-corridor planner | ❌ No | Heavy, ROS 1, and not used in industry Nav2 stacks. We use Nav2 DWB/MPPI instead, which is exactly limitation L1 |
| Real-robot experiments | ❌ No | Simulation only (allowed) |

## 7. Why this is our primary paper

1. It is literally our topic: **failure prediction + recovery** (the instructor's research area).
2. Approved venue (ICRA), 2024, original research.
3. Its setup (ground robot, 2D LiDAR, Gazebo) maps **1:1 onto ROS 2 Nav2**.
4. Small and CPU-only, so most of our time goes into the improvement and the evaluation.
5. Its weaknesses are concrete and fixable, which gives a clear "beyond reimplementation" story.
