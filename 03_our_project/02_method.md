# 02 – Method (Contributions C1 and C2)

Notation: t = current time; control cycle at 10 Hz. The Nav2 controller outputs a **local plan**, a short sequence of predicted poses x̂₁ … x̂_K over its horizon (~1.5–2 s). The global planner outputs a **global path** to the goal. The latest LiDAR scan is S_t.

---

## Contribution C1 – Planner-agnostic, uncertainty-aware, calibrated failure prediction

### C1.1 What exactly we predict

We learn a **per-state** model, like the paper:

  g(φ(x)) ≈ P( a failure occurs within h seconds | the robot is at state x )

where φ(x) is a feature vector and h is short (0.5–1.0 s). At runtime we evaluate g at **each predicted state** x̂_i of the local plan. Together with the plan horizon, this lets us look ~2–3 s ahead.

- **Training labels:** for each logged time t with actual state x_t: y_t = 1 if a failure event (controller failure, collision, or stuck onset) happens in (t, t+h], else 0.
- **Why per-state:** it matches the paper (good for the B2 comparison) and lets the same model score **recovery candidates** (C2).

### C1.2 Features φ(x): planner-agnostic replacements for t_C and |C|

All features use only the **LiDAR scan, the robot state, and the plans**, which every Nav2 controller provides. There is no corridor.

| # | Feature | Definition | Replaces / why |
|---|---|---|---|
| f1 | **Time-to-collision along heading**, TTC | Cast a ray of robot width from x along its velocity direction through the scan points; d_ray = distance to the first point hit, minus the robot's front extent. TTC = d_ray / max(‖v‖, ε), capped at γ_t (e.g., 5 s) | Direct analogue of **t_C**: distance + speed + heading in one number |
| f2 | **Min clearance** | min over scan points of (distance to x) − robot radius | How tight the space is, independent of heading |
| f3 | **Global-path tortuosity** | Σ \|Δheading\| along the next L = 3 m of the global path, divided by L | Analogue of **\|C\|**: twisty path = hard |
| f4 | **Speed** ‖v‖ | From odometry (current) or plan spacing (predicted states) | Faster = less time to react |
| f5 | **Unknown-space ratio ahead** | Fraction of *unknown* cells in the costmap within a forward sector (±30°, 3 m) | Captures **occlusion**, the paper's motivating failure (hidden walls) |
| f6 | *(optional)* Free-space ratio ahead | Fraction of free cells in the same sector | Openness of the area |

All features are standardized (zero mean, unit variance, using the training statistics).

> Note: the predicted states x̂_i lie in the future, but their features are computed from the **current** scan/map. That is exactly the information the robot has now, as in the paper.

### C1.3 Model: one sparse GP classifier

- **Why a GP:** we need **calibrated uncertainty** (σ) that grows away from the training data. That is the property the paper ignores.
- **Type:** variational sparse GP **classifier** (GPyTorch `ApproximateGP` + `BernoulliLikelihood`), ARD-RBF kernel (one length-scale per feature, so we also learn which features matter), M ≈ 256–512 inducing points (handles 10⁵ samples on a CPU).
- **Output per state:** a latent Gaussian f ~ N(m, s²). Mean probability p = Φ(m / √(1+s²)), where Φ = the standard normal CDF. Uncertainty = s.
- **Training data balance:** failures are rare, so keep all positives and subsample negatives (e.g., 1:5). We do *not* re-weight at calibration time, because calibration uses natural frequencies.
- **Inference cost:** K ≈ 20 states × one GP evaluation is under a few ms on a CPU. Target: < 10 ms per cycle.

Compared with the paper: one model instead of one per |C|; classification on raw labels instead of regression on binned fractions; outputs always lie in [0, 1].

### C1.4 Uncertainty-inflated, normalized risk

Per predicted state i: **p_i⁺ = Φ(m_i + β·s_i)**, an *upper-confidence* failure probability. β = 0 gives (roughly) the mean-only behavior of the paper. β = 1 or 2 means "if unsure, assume more danger".

Aggregate over the K states with a **normalized** risk that does not depend on K:

  ρ̄_t = (1/K) Σ_i p_i⁺   (mean, default)   or   ρ_t^max = max_i p_i⁺   (ablation)

The paper's Σ_i p_i is kept for baseline B2 only.

### C1.5 Calibrated alarm threshold (split conformal)

**Goal:** choose the threshold so that, in situations that are actually safe, the alarm fires at most a fraction α of the time (e.g., α = 0.05).

**Procedure:**
1. Run the **unmodified** Nav2 stack (monitor passive, logging only) on the **calibration worlds**. Passive matters: if the monitor intervened, failures would be prevented and the labels would be biased.
2. Collect **safe windows**: times t with *no* failure in (t, t + T_eval], T_eval = 3 s.
3. To reduce time correlation, take **one random t per 2 s block** per episode, giving n scores s_j = ρ̄_{t_j}.
4. Threshold **ψ̂ = the ⌈(n+1)(1−α)⌉-th smallest s_j**.
5. **Guarantee (split conformal [9], [10]):** for a new safe window exchangeable with the calibration ones, P(ρ̄ > ψ̂) ≤ α. Because real windows are not perfectly exchangeable, we treat this as approximate and **always report the measured false-alarm rate**.

**Hysteresis (from LA-LIO [4]):** the alarm turns **ON** when ρ̄ > ψ̂ and **OFF** only when ρ̄ < ψ̂_off, with ψ̂_off = the (1 − 2α) quantile (lower). This prevents stop/go flickering.

**Why this is better than ψ = 3:** the user picks the false-alarm rate (an operational KPI) instead of a meaningless number; the threshold is re-derived automatically for a new robot, speed or controller; and it's normalized, so the horizon discretization no longer matters.

### C1.6 Pseudocode (monitor, every control cycle)

```
on new (scan, odom, local_plan, global_path):
    states  = local_plan.poses (K of them), with velocities from spacing/time
    for each state x_i: φ_i = features(x_i, scan, costmap, global_path)
    (m_i, s_i) = GP(φ_i)
    p_i⁺ = Φ(m_i + β s_i)
    ρ̄ = mean(p_i⁺)
    if not alarm and ρ̄ > ψ̂:      alarm = True  → publish /failure_alarm (+ argmax_i location)
    if alarm and ρ̄ < ψ̂_off:      alarm = False
```

---

## Contribution C2 – Progress-aware recovery with decaying failure memory

### C2.1 When the alarm fires

1. **Stop:** the supervisor node cancels the current Nav2 goal and commands a controlled stop (zero velocity, within acceleration limits).
2. **Remember:** add a **failure mark** at the location of the riskiest predicted state, x̂_{argmax p_i⁺} (and at the actual location if a real failure occurred).
3. **Choose a recovery point** (C2.2) and drive there at reduced speed.
4. **Resume:** re-send the original goal. The global planner now sees the failure memory (C2.3) and tends to route around the trap.
5. **Budget:** at most 3 recoveries per goal. After that, fall back to Nav2's default recovery tree, and log it as a "budget exhausted" event.

### C2.2 Scoring recovery candidates

- **Sample** N ≈ 200 candidate positions c uniformly in **free** local-costmap cells within radius R = 2 m (rejection sampling, which is simpler than the paper's hit-and-run but serves the same purpose).
- **Cost-to-go field:** compute **once** a Dijkstra/wavefront distance d_goal(·) from the goal over the global costmap. Each lookup is O(1), so there are no 200 planner calls.
- **Realistic departure state:** for each c, assume the robot leaves c along the descending direction of d_goal at speed v_dep = 0.5·v_max (**not at rest**, which fixes the uninformative-TTI issue of the paper).
- **Score** (lower is better):

  J(c) = p⁺(c) + λ · d_goal(c)/d_goal(x₀) + γ · M(c) + κ · ‖c − x₀‖/R

  | Term | Meaning |
  |---|---|
  | p⁺(c) = Φ(m + βs) at the departure state | Risk of failing when leaving from c (the paper's term, now uncertainty-aware) |
  | d_goal(c)/d_goal(x₀) | **Progress:** < 1 means closer to the goal than now; penalizes retreating far |
  | M(c) | **Failure memory:** penalizes known trap areas |
  | ‖c − x₀‖/R | Short recovery motions are preferred |

- **Filter:** discard candidates with p⁺(c) > η. If none are left, resample with a larger R (as in the paper).
- **Pick** c* = argmin J and send it as a NavigateToPose goal with a reduced speed limit (Nav2 speed-limit topic).
- **Weights:** λ, γ, κ are tuned on the *calibration* worlds only (never on test worlds).

### C2.3 Decaying failure memory

Inspired by Air Bumper's collision-aware map [2], with one key change: **marks decay**.

  M(x, t) = Σ_k exp(−(t − t_k)/τ_d) · exp(−‖x − p_k‖² / 2ς²)  (clipped to [0, 1])

- p_k, t_k = location and time of the k-th failure mark. ς ≈ 0.5 m spatial spread. τ_d ≈ 60 s memory time.
- **Why decay:** Air Bumper's marks are permanent, so a temporarily blocked spot (a person, a cart, a door) would stay blocked forever. Decay lets the robot re-try later.
- **Soft, not lethal:** M is converted to a *high but non-lethal* costmap cost, so the planner avoids the trap if any alternative exists, but can still pass if it's the only way. This avoids "artificially blocking the only corridor".
- **Integration with Nav2:** publish M as an occupancy-grid **mask** used by a Nav2 **costmap filter** (keepout-style) on the global and local costmaps. *Fallback:* a small custom costmap-layer plugin.

### C2.4 Why this fixes limitation L4

| Paper's recovery | Ours |
|---|---|
| Evaluates candidates at rest, so TTI is constant (uninformative) | Evaluates at a realistic departure velocity |
| Only minimizes risk, so it may retreat far | Adds a progress term |
| No memory, so it can re-enter the same trap (oscillation) | Failure memory in the costmap, so it re-routes |
| Fixed behavior | Recovery budget + fallback |

---

## Summary of design choices and their ablations

| Choice | Default | Ablation alternatives |
|---|---|---|
| Uncertainty weight β | 1 | 0 (mean-only = paper), 2 |
| Risk aggregation | mean | max, Σ (paper) |
| Target false-alarm α | 0.05 | 0.01, 0.10 |
| Threshold | conformal + hysteresis | fixed ψ (paper), conformal w/o hysteresis |
| Feature set | f1–f5 | leave-one-out each; paper-style {f1, f3} only |
| Model | sparse GP | logistic regression, random forest (+ same calibration) |
| Recovery | full J | without progress (λ = 0), without memory (γ = 0), memory without decay (τ_d = ∞) |
| Horizon h | 0.75 s | 0.5, 1.0 s |
