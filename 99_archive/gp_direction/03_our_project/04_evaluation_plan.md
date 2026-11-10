# 04 – Evaluation Plan

## 1. Research questions

| RQ | Question | Hypothesis | Main evidence |
|---|---|---|---|
| **RQ1** – Generality | Can planner-agnostic, scan-based features predict Nav2 controller failures (DWB and MPPI) as well as the paper's corridor-style features? Does a predictor trained on DWB transfer to MPPI? | H1 | Offline detection: AUROC, AUPRC, lead time; cross-controller matrix |
| **RQ2** – Uncertainty & calibration | Does uncertainty-inflated risk with a conformal threshold keep the false-alarm rate ≤ α while improving recall and lead time over mean-only risk with a fixed threshold, especially out of distribution? | H2, H3 | TPR at target α, empirical FPR vs α, per-world-set breakdown |
| **RQ3** – Recovery | Does progress-aware recovery with failure memory raise success and reduce repeated failures compared with min-risk recovery and Nav2 defaults? At what time cost? | H4 | Closed-loop success, collisions, repeated failures, BARN score, time |

## 2. Baselines

| ID | Detector | Recovery | Represents |
|---|---|---|---|
| **B0** | none | Nav2 default BT (clear costmap, spin, back up, wait) | Industry standard, purely reactive |
| **B1** | TTC < τ rule (non-learned; τ tuned on calibration worlds) | Nav2 default | Simple reactive/heuristic monitoring (cf. Air Bumper-style thresholding) |
| **B2** | **Primary paper reimplemented:** GP *regression* on binned failure rates, features {f1 ≈ t_C, f3 ≈ \|C\|}, mean only, Σ-risk, fixed ψ | Paper's min-risk sampling, candidates at rest | The primary paper |
| **B3** | Our features + logistic regression / random forest, same conformal threshold | Ours (C2) | "Is the GP needed?" |
| **Ours** | C1 (sparse GP, μ + βσ, normalized risk, conformal ψ̂, hysteresis) | C2 | Full method |

For B2's fixed ψ we report both the paper's value (ψ = 3, Σ-risk) and the best ψ tuned on calibration worlds, to be fair to the baseline.

## 3. Metrics (with exact definitions)

### 3.1 Offline detection (monitor runs passively on logged test episodes)
| Metric | Definition |
|---|---|
| **AUROC** | Area under the TPR-vs-FPR curve of ρ̄ over all thresholds. Positive time step = failure within (t, t + 3 s] |
| **AUPRC** | Area under precision-recall (more informative when failures are rare) |
| **TPR @ α** | Fraction of failure **events** preceded by an alarm within the 3 s before them, at the calibrated threshold |
| **Empirical FPR** | Fraction of safe windows in which the alarm is on. Compared with the target α (calibration validity) |
| **False alarms per minute** | Alarm onsets not followed by a failure within 3 s, per minute of driving (operator-friendly) |
| **Lead time** | Time from the first alarm to the failure event (median, IQR). Only for detected events |
| **Latency** | Monitor compute time per cycle (mean, 99th percentile). Must be < 10 ms |

### 3.2 Closed-loop navigation (monitor + recovery active)
| Metric | Definition |
|---|---|
| **Success rate** | Episodes reaching the goal with no collision within the timeout |
| **Collision rate** | Episodes with ≥ 1 contact |
| **Timeout / stuck rate** | Episodes ending by timeout or stuck-abort |
| **BARN score** | s = 1_success · OT / clip(AT, 2·OT, 8·OT) [3], [6] (OT = optimal time, AT = actual time) |
| **Traversal time** | For successful episodes |
| **Alarms & recoveries per episode** | How often we intervene |
| **Recovery success** | Fraction of recoveries followed by ≥ 10 s without a failure |
| **Repeated-failure count** | Failures within 1 m of an earlier failure in the same episode (measures oscillation/loops) |

## 4. Experiments

| # | Experiment | Data | Answers |
|---|---|---|---|
| E1 | **Feature & model study (offline):** paper-style features {f1, f3} vs ours {f1–f5}; GP vs LR vs RF | Train W1-train (DWB), test W1-test / W2 / W3 | RQ1 |
| E2 | **Cross-controller transfer (offline):** train on DWB → test on MPPI, and the reverse, and joint training | Same | RQ1 |
| E3 | **Calibration study (offline):** target α ∈ {0.01, 0.05, 0.10} vs measured FPR; fixed ψ vs conformal ψ̂; per speed and per world set | W1-cal → W1-test, W2, W3 | RQ2 |
| E4 | **Uncertainty study (offline):** β ∈ {0, 1, 2}; TPR at equal FPR, in-distribution vs OOD | W1-test vs W3 | RQ2 |
| E5 | **Closed-loop comparison:** B0, B1, B2, B3, Ours × {DWB, MPPI} × speeds {0.5, 1.0, 1.5} (+ 2.0 on W3) × 3 seeds | W1-test (60) + W2 (~20) + W3 (~30) worlds | RQ2, RQ3 |
| E6 | **Recovery ablation (closed loop):** full C2 vs λ = 0 vs γ = 0 vs no decay vs the paper's recovery, with C1 fixed | W2 + hardest W1-test worlds | RQ3 |
| E7 | **Sensitivity:** horizon h, aggregation (mean / max / Σ), hysteresis on/off, number of candidates N | Subset | Robustness of the conclusions |

**Run count (E5):** ~110 worlds × 5 methods × 2 controllers × 3 speeds × 3 seeds ≈ 9,900 episodes at most. If time is short, we cut to 1 controller × 2 speeds (≈ 3,300) or fewer seeds, and **state this cut explicitly** in the report.

## 5. Statistics

- **Unit of analysis = the world** (runs in the same world are not independent).
- 95% **bootstrap confidence intervals** (resampling worlds) for every rate.
- **Paired tests** between methods on the same worlds: **McNemar** for success/failure, **Wilcoxon signed-rank** for time and score. Holm correction for multiple comparisons.
- Report effect sizes, not just p-values.

## 6. Planned result tables and figures (templates)

**Table A – offline detection** (rows: B1, B2, B3-LR, B3-RF, Ours-β0, Ours-β1, Ours-β2; columns: AUROC / AUPRC / TPR@α / FPR / lead time, for W1-test · W2 · W3).

**Table B – closed loop** (rows: B0–B3, Ours; columns: Success / Collision / Timeout / BARN score / Time / Alarms / Repeated failures; per controller).

**Figures:** ROC curves per world set; target α vs measured FPR (the diagonal = perfect calibration); lead-time histograms; success vs speed curves (as in DDP Table I); example trajectories (trap → recovery → reroute).

## 7. Threats to validity (we will discuss them honestly)

- **Sim-only:** results may not transfer to real robots. We mitigate this by following the established BARN protocol, and the primary paper itself transferred sim-trained models.
- **Exchangeability:** the conformal guarantee is approximate under time correlation, so we report the empirical FPR.
- **Ground-truth odometry:** this removes localization errors, so real-world failure rates are likely higher.
- **Tuning leakage:** all hyperparameters (β, λ, γ, κ, ψ for B1/B2) are tuned only on the calibration worlds.
