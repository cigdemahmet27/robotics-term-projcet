# Limitations & Open Questions of the Primary Paper

Each limitation has: **what** it is, **evidence** in the paper, **why it matters** (including for industry), and **what we do about it** (linked to `03_our_project/`). This is the material for proposal Section 4 ("Challenges & Limitations").

| # | Short name | Addressed by us? |
|---|---|---|
| L1 | Planner-specific features, one failure type | ✅ C1 |
| L2 | GP uncertainty thrown away | ✅ C1 |
| L3 | Hand-tuned, scale-dependent threshold; no false-alarm guarantee | ✅ C1 |
| L4 | Recovery: uninformative at rest, not progress-aware, memoryless | ✅ C2 |
| L5 | Evaluation reports only success rate | ✅ Evaluation plan |
| L6 | Static obstacles only | ⚠️ Partially (OOD test set W3 with moving obstacles) |
| L7 | One GP per \|C\| value; binned regression | ✅ Side effect of C1 (single GP classifier) |
| L8 | Localization assumed reliable | ❌ Out of scope (future work, see LA-LIO) |

---

## L1 – Features are tied to one planner, and only one failure type is modeled

- **What:** t_C needs the polygon walls of a safe corridor, and |C| is the number of polygons. **Planners without corridors**, which includes the sampling-based controllers that most deployed ROS robots use (DWB/DWA, MPPI), don't produce these quantities. Also, only **back-end optimizer infeasibility** is predicted, not collisions or getting stuck/oscillating (which are what users actually experience as "failure").
- **Evidence:** Sec. IV-A/B/C. Features are defined from C (Eqs. 4–5). The label is Z^b (back-end failure).
- **Why it matters:** the method can't be dropped into a standard Nav2 stack, which is how most industrial AMRs are built. "Model-agnostic" holds for the robot model, not the planner.
- **Our answer → C1:** planner-agnostic features computed from the **LiDAR scan + robot state + the controller's predicted local trajectory** (scan-based TTC, clearance along the local plan, global-path tortuosity, speed, free/unknown space ahead), and labels that cover **controller failure, collision, and stuck**. Tested on **two** controllers (DWB, MPPI) and across them (train on one, test on the other).

## L2 – The GP's uncertainty is computed but never used

- **What:** P̂ = μ* only (Eq. 10). σ* (Eq. 9) is ignored.
- **Evidence:** Eq. 10. The risk in Eq. 12 uses only the probabilities.
- **Why it matters:** far from the training data an RBF-GP's mean **reverts to the prior mean** (e.g., a low failure probability), while σ* becomes large. The detector is therefore **most over-confident exactly in new environments**, which are where failures are most likely (e.g., a world much denser than the training forests, or obstacle shapes never seen). The paper's generalization check (Fig. 4c–d) covers only |C| = 2, 3 in similar worlds.
- **Our answer → C1:** **uncertainty-inflated risk**, using μ + β·σ instead of μ ("if unsure, assume more danger"). β = 0 recovers the paper's behavior, so the effect of uncertainty is a clean ablation.

## L3 – Hand-tuned, scale-dependent threshold with no false-alarm guarantee

- **What:** ρ = Σ_i P_i (Eq. 12) is a **sum over the predicted states**. Its value grows with **how many** states the MPC outputs (horizon length × discretization). ψ = 3 is set by hand.
- **Evidence:** Eq. 12; "the risk threshold was ψ_ρ = 3" (Sec. VI). No tuning procedure is described.
- 🧮 **Illustration:** the same per-state danger p = 0.2 gives ρ = 2 with 10 predicted states (no stop) but ρ = 4 with 20 states (stop). Changing the MPC's step size silently changes the detector.
- **Why it matters:** in practice every **false stop** costs time and throughput (a warehouse KPI), and every **miss** risks a collision. Without a principled threshold you can't state, or guarantee, either rate. Air Bumper [2] and LA-LIO [4] also use hand-set thresholds, so this is a recurring gap.
- **Our answer → C1:** (a) a **normalized** risk (the mean, or max, over states, independent of the horizon discretization), (b) a threshold chosen by **split conformal calibration** so the false-alarm rate on safe situations is ≤ α (user-chosen, e.g. 5%), and (c) **hysteresis** (from LA-LIO [4]) against flickering.

## L4 – Recovery is not progress-aware, is partly uninformative, and has no memory

- **What:**
  1. Candidates are evaluated **at rest** (v = 0). In Eq. 4, r·v = 0 gives t_H = γ_t for all walls, so **t_C is the same constant for every candidate**. The GP ranking then depends essentially on |C_i| (a small integer), which produces many ties, and on a "neighbor consistency" check that is not fully specified.
  2. The selection (Eq. 13) minimizes failure probability only. **It doesn't consider how much progress toward the goal** a candidate gives up (it may pick a spot far behind the robot).
  3. There is **no memory**: after recovering, the nominal planner may plan into the **same trap** again. The paper itself notes that naive retracing causes **oscillatory behavior** (Sec. IV-D) but provides no mechanism against repeated visits.
- **Evidence:** Sec. IV-D, Eq. 13.
- **Why it matters:** recovery loops (stop → back up → re-enter → stop) are a classic real-world AMR failure. They waste time and eventually time out.
- **Our answer → C2:** candidates are scored with (i) risk (μ + βσ) evaluated at a **realistic departure speed**, not at rest, (ii) a **progress term** (global-planner cost-to-go from the candidate), and (iii) a **failure-memory map** (inspired by Air Bumper [2]) that marks predicted/observed failure regions with **decaying** cost and is also fed to the global planner, so the robot routes around the trap.

## L5 – Evaluation shows only the end result

- **What:** results are the success rate with vs without the method, 120 sim runs in 4 worlds (Fig. 6), plus qualitative real-robot demos.
- **Missing:** detection quality (TPR/FPR, AUROC), **lead time** (how early the warning comes), **false-stop rate**, the number of triggers per run, the time cost, comparisons with **other detectors** (e.g., a simple time-to-collision rule, a classifier) and **other recoveries** (e.g., ROS's default spin/back-up), and statistical significance.
- **Why it matters:** we can't tell whether success improves because the predictions are **accurate** or because the robot simply **stops more often** and drives more cautiously. A failure-detection reviewer (our graders) would ask exactly this.
- **Our answer → evaluation plan:** offline detection metrics, online navigation metrics, 4 baselines, ablations, ~2,000+ runs, confidence intervals and paired tests (`03_our_project/04_evaluation_plan.md`).

## L6 – Static obstacles only
- The authors list dynamic obstacles as future work. **We** include moving obstacles only in the out-of-distribution test set W3, to measure robustness. We don't claim to solve dynamic scenes.

## L7 – Multiple GPs and binned regression
- One GP per |C| value, regressing on **binned** failure fractions. Binning throws away information, and a regressed mean can leave [0, 1]. The authors themselves want to "eliminate the need for multiple GP regressions".
- **Our answer:** a **single sparse GP classifier** (Bernoulli likelihood) on all features, so the outputs are proper probabilities. The paper's binned version is kept as baseline B2.

## L8 – Localization is assumed reliable
- The paper uses Gmapping and does not consider localization failures (e.g., long featureless corridors, which LA-LIO [4] studies).
- **Our scope:** we use ground-truth odometry in simulation to **isolate planning failures**, and list localization failures as future work.

---

## ❓ Open questions we could get from the instructors (prepare answers)

| Question | Our answer |
|---|---|
| "Isn't this just a reimplementation?" | No. The paper is our *baseline B2*. Our contributions C1 and C2 change the features, the risk, the threshold and the recovery, and we test claims the paper doesn't evaluate |
| "Why not use deep learning?" | A GP gives calibrated uncertainty and works with little data on a CPU. Uncertainty is central to our C1. We compare against simple classifiers (B3) |
| "Conformal guarantees need exchangeable data, but time steps are correlated." | Correct. We subsample one window per episode segment for calibration, treat the guarantee as approximate, and **report the empirical false-alarm rate** to check it |
| "How do you define failure?" | Controller FAILED / no valid trajectory, collision (contact sensor), or stuck (progress checker). Each is logged automatically |
| "Why is simulation enough?" | The guidelines allow it. The original paper's training was also sim-only. Simulation allows thousands of controlled, repeatable runs, which a rigorous evaluation needs |
