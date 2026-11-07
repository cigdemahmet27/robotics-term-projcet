# Paper Walkthrough: Section by Section

Read this side by side with the PDF. Page numbers refer to the ICRA proceedings (pp. 2418–2424). Concepts marked (→A3) etc. are explained in [`01_background_knowledge.md`](01_background_knowledge.md).

---

## Abstract & I. Introduction (p. 2418)

**Story:**
- Robust navigation in unknown, cluttered environments is still unsolved. At the BARN challenge, **no team** navigated without collisions (the authors placed 2nd).
- Runtime failures are usually caused by the **motion-planning pipeline**.
- **Reactive** approaches (detect risk as it happens) are conservative and too slow for high-inertia robots, which need time to brake.
- **Proactive** approaches (classify future states as safe/unsafe) usually need big deep-learning models and a lot of real-world data, **and they don't do recovery**.
- **Fig. 1 example:** a robot drives fast toward an occluded dead end. Without prediction it discovers the dead end too late to stop. With prediction it stops at x_B, moves to a recovery point x_r, then continues.

**Claimed contributions:**
1. A **proactive, model-agnostic, GP-based planner-failure detector** over a future horizon, needing **no retraining from sim to real**.
2. A **GP-based, sampling-based recovery** that drives the robot to a safe point.
They claim to be the first to use one learning component for **both** proactive failure prediction and recovery.

> 💡 "Model-agnostic" here means independent of the *robot dynamics model*, **not** independent of the *planner*. The features are tied to the safe-corridor planner (our limitation L1).

---

## II. Related Work (pp. 2418–2419)

| Family | What they say | Why it isn't enough (per the authors) |
|---|---|---|
| Hard-constrained corridor planners [6–8], FASTER [9] | Optimize inside safe corridors; FASTER also keeps a backup "stop" trajectory | Random disturbances and occlusions cause infeasibility; FASTER has no recovery if even stopping fails |
| Soft-constrained planners [10, 11] | Always output something | Conflicting cost terms produce low-quality or unsafe trajectories |
| HJI reachability [13] and learned approximations [14, 15] | Formal safety | Need an accurate system model; curse of dimensionality |
| Learned anomaly/collision predictors [4] (Ji et al. RA-L 2022), [5] (BADGR) | Predict anomalies and stop | Trivial recovery (back up & rotate) or a human performs the recovery |

**Takeaway:** the niche is "learned **proactive** prediction **plus** autonomous, **planner-aware recovery**".

---

## III. Problem Formulation (p. 2419)

**Setup:**
- Robot dynamics ẋ = f(x) + g(x)u (→A2).
- A trajectory τ(t; t₀) is planned at time t₀ over horizon T_H, avoiding the obstacles known *at t₀*, X_O(t₀).
- Obstacles get discovered while driving: X_O(t) ≠ X_O(t₀), so re-planning is needed, and re-planning can **fail** (infeasible constraints).
- One failure is OK. **Several failures within the horizon** are dangerous.

**Problem 1 (proactive detection):**
- {x̂_i} = predicted future states over the control horizon T_F ≤ T_H.
- Z_τ ∈ {0, 1, 2, …} = **number of planning failures** that would occur along those states.
- Find a **risk metric ρ_τ** that maps the distribution of Z_τ to one number.

**Problem 2 (recovery):**
- If ρ_τ > ψ_ρ, stop and recover.
- Z_x ∈ {0, 1} = failure of the planner Π *if started from state x*.
- Find a nearby free state: **x_r = argmin_{x ∉ X_O} E[Z_x]** (Eq. 1), the spot where planning is most likely to succeed.

---

## IV. Approach (pp. 2419–2422)

### Fig. 2: the overall pipeline
- **Offline:** simulation → logged successes/failures → GP training.
- **Online:** map → front-end (JPS + corridor) → back-end (Bézier optimization) → MPC → predicted states {x̂_i} → GP risk ρ → if ρ > ψ, recovery (sample → GP scoring → go-to-goal MPC).

### IV-A. Motion planner preliminaries
1. **Front-end:** occupancy map M + current state + goal → **JPS** path (→B1) → **corridor C** of convex polygons C_i = {p | A_i p ≤ b_i} (Eq. 2, →B2, B3), built with the method of [10].
2. **Back-end:** trajectory τ(t) = N cubic **Bézier** pieces; keep the control points inside the polygons (→B4). Solved by **FASTER** [9], modified to use the **MINVO** basis (→B5) for a higher success rate. Then an **MPC** tracks τ (→B8).

### IV-B. Failure modes: front-end vs back-end
- **Front-end failure:** no corridor can be built, because there is no path (start/goal overlaps obstacles) or the JPS parameters are poorly conditioned (horizon too long, |C| too large). **Cheap (< 1 ms), so just run it.** P(Z^f) ∈ {0, 1}.
- **Back-end failure:** the corridor exists, but the optimizer finds no feasible trajectory (e.g., a newly discovered wall needs an avoiding maneuver that isn't feasible at the current speed; see Fig. 3c). **Expensive (> 100 ms) and hard to predict analytically.**
- The full pipeline failure probability is P(Z_{x,C}) = P(Z^b | Z^f) · P(Z^f) (Eq. 3). Front-end failure implies back-end failure: P(Z^b = 1 | Z^f = 1) = 1.
- **The contribution:** estimate P(Z^b | Z^f = 0), written as P(Z^b).

> 💡 This is a clever decomposition: compute the cheap part exactly, and learn only the expensive part.

### IV-C. GP for predicting back-end failure

**Data collection.** Poisson random forest worlds [18]: **10 worlds × 90 start/goal pairs**. A simulated Jackal drives each one, and **every planning iteration** is logged as success or failure.

**Feature design rule:** features should depend only on the corridor C and the robot position and its derivatives, *not on the sensor*, so the model is sensor-agnostic and transfers sim → real.

**IV-C1. Features.** Two features were found to be good predictors:

**(f1) Minimum time-to-intersect (TTI), t_C.** For each wall (row r_i, b_i) of the polygon containing the robot (Eq. 4):

  t_H = (b_i − r_i·p) / (r_i·v)  if r_i·v > 0 (moving *toward* that wall), otherwise γ_t (a cap value)

  t_C = min over the walls of t_H  (Eq. 5)

- Numerator = the distance to the wall. Denominator = the speed component toward the wall.
- 🧮 *Toy example:* polygon wall x ≤ 2 (r = (1, 0), b = 2). Robot at p = (1.5, 0.5) with v = (1, 0) m/s, so t_H = (2 − 1.5)/1 = **0.5 s**. At 2 m/s this becomes 0.25 s, which is more dangerous.
- Why it works: it combines **distance to the boundary, speed, and heading** in one number.

**(f2) Corridor size |C|.** The number of polygons. More polygons means a more non-direct, twisty path, which makes the optimizer more likely to fail.

**Fig. 4a/b:** failure probability **rises as t_C falls** and **rises as |C| grows**. This is the empirical validation of the features.

**How labels become regression targets:** the binary success/failure data are **binned** by feature value d = [t_C, |C|], and the **fraction of failures in each bin** is the target probability.

**IV-C2. GP regression.** Inputs = features of the bins, outputs = binned failure probabilities. Joint Gaussian (Eq. 6), RBF kernel, posterior mean μ* (Eq. 8) and variance σ*² (Eq. 9) (→C3).
- **Eq. 10: P̂(Z^b | d) = μ\***. Only the **mean** is used.
- **Fig. 4c/d:** the learned curves vs the test-world distributions for |C| = 2 and |C| = 3. They match well, so the model generalizes to new worlds.
- Fig. 4c/d is shown per |C| value, and the conclusion mentions "eliminating the need for multiple GP regressions". This suggests **one GP per |C| value**, with t_C as the input.

**IV-C3. Planning risk.**
- Z_τ = Σ over the predicted states of Z_{x̂,C} (Eq. 11): the number of failing future states.
- **ρ_τ = E[Z_τ] = Σ P(Z_{x̂,C})** (Eq. 12): the expected number of failures (→C4). The text loosely calls it "expected number of collisions".
- If ρ_τ > ψ_ρ, trigger recovery. The real-robot experiments use **ψ_ρ = 3** (Sec. VI).

🧮 *Toy example:* the MPC predicts 15 future states. The GP gives p = 0.05 for the first 5, 0.2 for the next 5, and 0.4 for the last 5 (approaching a hidden dead end). Then ρ = 5·0.05 + 5·0.2 + 5·0.4 = 0.25 + 1 + 2 = **3.25 > 3**, so the robot stops.

### IV-D. Recovering after predicted failures
1. **Stop** the robot.
2. *Why not just reverse along the old path?* The planner would produce **the same problematic trajectory again**, causing **oscillation**. (Remember this sentence; it motivates our Contribution 2.)
3. Build a polygon C_r around the current position. **Hit-and-run sampling** (→D2) gives N_p candidate positions P_c.
4. Convert them to states X_c **assuming the robot starts from rest** (velocity 0). This shrinks the search space, and "sampling only positions was enough in practice".
5. For each candidate compute P(Z_{x_i, C_i}) (its corridor C_i toward the goal), and also check **neighboring states** "for consistency".
6. If **all** candidates have failure probability > η (risk-aversion parameter), resample.
7. **x_r = argmin E[Z_{x_i, C_i}]** (Eq. 13).
8. Drive to x_r with the **go-to-goal MPC**, constrained to stay inside C_r (obstacle-free).
9. Switch back to the nominal planner. The whole process repeats as needed.

> ⚠️ **Subtle point (important for our critique):** at rest, v = 0, so r_i·v = 0 for every wall, and Eq. 4 gives t_H = γ_t (the cap) everywhere. **The TTI feature becomes constant for all candidates.** The GP score then depends essentially on |C_i| (the corridor complexity from that candidate to the goal) plus the front-end check. |C| is a small integer, so many candidates **tie**. The choice is not informed by the distance to obstacles at the speed the robot will actually have, nor by progress toward the goal.

---

## V. Simulations (pp. 2422–2423)

- **Gazebo + ROS Noetic, Ubuntu 20.04.** Jackal with a 270° 2D LiDAR.
- **Test worlds:** a series of connected rooms, **sparse or dense** obstacles × **1 m or 2 m doorways**, giving 4 worlds. Same start, **3 goals**, **10 runs per goal**: 30 runs per world, **120 total**.
- **Fig. 5:** an obstacle hides a wall. Without the method the robot collides at x_A. With it, the robot stops at x_B, reverses to x_r, and continues.
- **Fig. 6:** success rate with vs without the method, for each world and goal. The method improves success everywhere. *Only success rate is reported.*

## VI. Physical Experiments (p. 2423)

- **Spot** (Ouster LiDAR) and **Jackal** (Velodyne), with Gmapping SLAM for the map and state. The same planner pipeline and MPC.
- The GP is **trained only in simulation**, which supports the sensor- and model-agnostic claim.
- Case 1: Jackal, occluded obstacles behind a corner (Fig. 7). Case 2: Spot, an unexpected dead end (Fig. 1).
- **ψ_ρ = 3** expected failures over the MPC horizon.
- Results are shown qualitatively (snapshots + video), not as statistics.

## VII. Conclusions & Future Work (p. 2423)

- Claims: a GP-based proactive detection + recovery scheme that improves a safe-corridor planner, is model- and sensor-agnostic, and needs no real-world training data.
- **Future work (their own words):** (1) **distributional learning** for failure detection, "eliminating the need for multiple GP regressions"; (2) **planner switching** in a Simplex Architecture; (3) **dynamic obstacles**.

---

## 🧾 One-paragraph summary (for presentations)

> Mohammad et al. predict, in real time, whether a hard-constrained safe-corridor planner will fail over the next few seconds. They learn, in simulation, a Gaussian-Process map from two geometric features (time-to-intersect the corridor wall, and corridor size) to failure probability. They sum these probabilities over the MPC-predicted states into an expected-failure risk and stop when it exceeds a threshold. Recovery samples nearby free positions and moves to the one with the lowest predicted failure. The same model detects and recovers, trains only in simulation, and transfers to a Jackal and a Spot. It improves navigation success in 120 simulated runs.
