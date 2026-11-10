# 03 – Our Project: What We Are Going to Do

**Working title:** *Know Before You Fail: Calibrated, Uncertainty-Aware Prediction of and Recovery from Navigation Failures in ROS 2*

| File | Content |
|---|---|
| [`01_problem_and_goals.md`](01_problem_and_goals.md) | The problem, industry motivation, scope (in/out), goals, hypotheses |
| [`02_method.md`](02_method.md) | ⭐ The two contributions in technical detail: features, model, risk, calibration, recovery, memory |
| [`03_system_and_environment.md`](03_system_and_environment.md) | ROS 2 architecture (nodes/topics), Docker/Nav2/Gazebo setup, robot, worlds, data pipeline, failure labels |
| [`04_evaluation_plan.md`](04_evaluation_plan.md) | Research questions, baselines, metrics (with definitions), experiments, ablations, statistics |
| [`05_team_timeline_risks.md`](05_team_timeline_risks.md) | Roles, week-by-week plan to 30.12, risks & fallbacks |

## The plan on one page

```
 PRIMARY PAPER (Mohammad et al., ICRA 2024)            OUR PROJECT
 ─────────────────────────────────────────            ───────────────────────────────────────
 Planner: FASTER safe-corridor (ROS 1)        →   Nav2 DWB + MPPI controllers (ROS 2)
 Features: t_C, |C| (need corridors)          →   scan-based TTC, clearance, tortuosity, speed, unknown-ratio
 Labels: back-end infeasible only             →   controller failure + collision + stuck
 Model: GP regression per |C|, mean only      →   one sparse GP classifier, mean AND variance     ┐
 Risk: Σ p_i  (scale depends on horizon)      →   normalized, uncertainty-inflated risk            ├ C1
 Threshold: ψ = 3 by hand                     →   conformal-calibrated ψ̂ (false alarms ≤ α) + hysteresis ┘
 Recovery: min-risk point, at rest, no memory →   risk + progress + decaying failure memory        ] C2
 Evaluation: success rate, 120 runs           →   detection + navigation metrics, 4 baselines, ablations, ~2000+ runs
```

## Where each secondary paper plugs in

| From | We take |
|---|---|
| Air Bumper [2] | Failure location as a "virtual sensor" written into the map → our failure-memory layer (with decay added) |
| DDP [3] | BARN worlds, the speed sweep to induce failures, the BARN score metric |
| LA-LIO [4] | Hysteresis between two thresholds |
