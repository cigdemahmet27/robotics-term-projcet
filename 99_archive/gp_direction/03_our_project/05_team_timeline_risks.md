# 05 – Team, Timeline & Risks

> ⚠️ Replace `[Member n]` with real names. This assumes **4 members**. For 3 members, Member 4's tasks are split between Members 2 and 3.

## 1. Roles (distinct and documented, as the guidelines require)

| Member | Role | Owns | Concrete deliverables |
|---|---|---|---|
| [Member 1] | **Simulation & Benchmark Lead** | Docker image, Gazebo + Jackal + Nav2 bring-up, world generators (W1 BARN-style, W2 rooms, W3 OOD), `episode_runner`, failure labeling, logging | `sim/` package, world sets, dataset (parquet), "how to run" doc |
| [Member 2] | **Failure Prediction Lead (C1)** | `feature_node`, GP training (GPyTorch), uncertainty-inflated risk, conformal calibration, hysteresis, offline detection experiments E1–E4 | `risk_node`, trained models, Table A, ROC/calibration figures |
| [Member 3] | **Recovery & Integration Lead (C2)** | `supervisor_node`, candidate sampling & scoring, cost-to-go field, `memory_node` + costmap filter, recovery budget, ablation E6 | Working closed-loop system, recovery ablation results |
| [Member 4] | **Baselines & Evaluation Lead** | B0–B3 (incl. reimplementing the primary paper as B2), experiment configs, batch execution E5/E7, statistics, result tables | `baselines/`, Table B, statistical analysis, plots |

**Shared:** everyone writes the report section for their own module. Weekly integration meeting (30 min). The presentation is split by module.
**Documentation of contributions:** a git history per member + a short `CONTRIBUTIONS.md` updated weekly.

## 2. Timeline (today: 07.11.2026)

| Week | Dates | Goal | M1 Sim | M2 Predict | M3 Recover | M4 Eval |
|---|---|---|---|---|---|---|
| 0 | 07–10 Nov | **Submit the round-1 proposal** | Proposal sec. 3 | Proposal sec. 5 | Proposal sec. 5 | Proposal sec. 6, refs |
| 1 | 10–16 Nov | Stack runs end-to-end | Docker + Jackal + Nav2 in an empty world ⚠️ verify LiDAR, topics | Read GPyTorch docs; feature prototypes on a toy bag | Study the Nav2 action API and costmap filters | Metric definitions → code skeleton |
| 2 | 17–23 Nov | Benchmark ready | BARN conversion + W2 generator, runner, labels | Feature node on live data | Supervisor skeleton (cancel/stop/resume) | B0, B1 working |
| 3 | 24–30 Nov | Data collected | Overnight data runs (train + cal) | First GP; E1 | Candidate sampling + cost-to-go | B2 (paper) reimplementation |
| 4 | 1–7 Dec | C1 done | W3 OOD worlds | Conformal + hysteresis; E2–E4 | Failure memory + mask | B3; batch scripts |
| 5 | 8–14 Dec | C2 done | Support / bug fixing | Integrate risk_node online | Full C2; E6 | Pilot E5 (small) |
| 6 | 15–21 Dec | Main experiments | Run E5 batches | Help analysis | Help E5 | Statistics, Tables A/B |
| 7 | 22–28 Dec | Write & rehearse | Report sections | Report sections | Report sections | Figures, slides |
| — | **30 Dec** | 🎤 **Presentation** | | | | |

Round-2 proposal revisions (date TBA) are handled by whoever owns the affected section.

**Integration milestones:** ✅ M-A (end wk 1): robot drives to a goal in Gazebo via Nav2. ✅ M-B (end wk 2): 10 episodes logged and labeled automatically. ✅ M-C (end wk 4): offline detection table. ✅ M-D (end wk 5): closed loop with all baselines.

## 3. Risks & fallbacks

| Risk | Likelihood | Impact | Mitigation / fallback |
|---|---|---|---|
| Nav2 / Gazebo / Clearpath packages don't work on our machines | Medium | High | Docker Jazzy image (week 1 priority). Fallback: a simple custom diff-drive URDF + `gpu_lidar` |
| BARN worlds hard to convert | Medium | Medium | Regenerate with the cellular-automaton procedure [5] directly in SDF |
| Too few failures in the data | Medium | High | Increase max speed, clutter density and narrow gaps (DDP shows failures rise sharply with speed) |
| Too many samples for a GP | Low | Medium | Sparse GP (inducing points), subsample negatives |
| The MPPI predicted trajectory isn't published | Low | Low | Enable visualization, or roll out the current command with a unicycle model |
| Costmap filter can't take a dynamic mask | Low | Medium | Small custom costmap layer, or inject the marks as virtual obstacles |
| Simulation too slow for ~10k runs | Medium | Medium | Headless mode, parallel containers, then cut seeds/speeds and **report the cut** |
| Conformal guarantee is violated in practice | Medium | Low (it's a finding) | Report the empirical FPR and discuss correlation. That is still a scientific result |
| A team member drops out | Low | High | Modules are independent. The minimum deliverable (C1 + detection results) needs only M1 + M2 |

## 4. Definition of done (per scope level)

- **Minimum:** benchmark (W1 + W2), B0 + B2, C1, offline detection results (RQ1, RQ2).
- **Target:** + C2, closed-loop E5 for at least one controller, E6.
- **Stretch:** both controllers, W3 with moving obstacles, transfer study E2, a LiDAR-degeneracy feature.
