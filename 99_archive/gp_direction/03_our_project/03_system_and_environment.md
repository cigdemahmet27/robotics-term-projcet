# 03 – System & Environment

> This is a **plan**, not an installed setup. Items marked ⚠️ must be verified during week 1.

## 1. Software stack (all free)

| Layer | Choice | Notes |
|---|---|---|
| Host | Ubuntu 26.04, RTX 4060 Laptop 8 GB | The team laptop |
| Container | **Docker**, image based on `osrf/ros:jazzy-desktop` (Ubuntu 24.04) | ⚠️ Nav2 had no release for ROS 2 *Lyrical* (the 26.04 distro) at the last check (May 2026). Jazzy is LTS (to 2029) and Nav2 is mature there. The same image for every member means "works on my machine" problems disappear |
| Middleware | **ROS 2 Jazzy** | |
| Navigation | **Nav2** (planner server: NavFn or Smac 2D; controller server: **DWB** and **MPPI**; BT navigator; costmap filters) | Both controllers come with Nav2 |
| Simulator | **Gazebo Harmonic** via `ros_gz` | Headless server mode (`gz sim -s`) for batch runs |
| Robot model | **Clearpath Jackal** via `clearpath_simulator` (Jazzy), with a 2D LiDAR | ⚠️ Verify that a 270° 2D LiDAR accessory is configurable in `robot.yaml`. Fallback: our own simple diff-drive URDF with a `gpu_lidar` sensor |
| ML | Python 3, **GPyTorch** / PyTorch (CPU), scikit-learn (baselines), NumPy, pandas | No GPU needed |
| Analysis | Jupyter, matplotlib, SciPy (tests) | |
| Version control | Git (GitHub, private repo) | |

Optional GPU use: Gazebo rendering through the NVIDIA container toolkit (only for GUI or visualization; batch runs are headless).

## 2. ROS 2 architecture

```
┌──────────────────── Gazebo Harmonic ─────────────────────┐
│ Jackal + 2D LiDAR + contact sensor + ground-truth odom   │
└──────┬───────────────┬───────────────┬───────────────────┘
       │ /scan         │ /odom (GT)    │ /contacts
       ▼               ▼               ▼
┌─────────────────────────── Nav2 ─────────────────────────┐
│ planner_server ─/plan─► controller_server (DWB | MPPI)   │
│                         ─/local_plan─►     ─/cmd_vel─► Gazebo
│ global & local costmaps (+ failure-memory filter mask)   │
│ bt_navigator (NavigateToPose)                            │
└──────────────────────────────────────────────────────────┘
       │ /plan, /local_plan, costmaps, controller status
       ▼
┌──────────────────── Our nodes (Python) ──────────────────┐
│ feature_node   : computes f1–f5 for current + predicted states
│ risk_node      : sparse GP → p_i⁺ → ρ̄ → alarm (hysteresis) ─► /failure_alarm
│ supervisor_node: on alarm: cancel goal, stop, pick recovery point (C2),
│                  send NavigateToPose(recovery), re-send goal, budget
│ memory_node    : failure marks → decaying mask M ─► /failure_memory_mask
│ episode_runner : spawns world, sends goal, detects success/failure,
│                  timeouts, writes logs (rosbag2 / mcap)
└──────────────────────────────────────────────────────────┘
```

Design notes:
- All our nodes are **Python** (rclpy), so the team doesn't need C++. A C++ costmap plugin is only a fallback.
- The supervisor sits **outside** the Behavior Tree (it calls Nav2 actions). That is simpler than writing a BT plugin. A BT plugin is a stretch goal.
- ⚠️ Verify which topic each controller uses to publish its predicted trajectory (DWB publishes the local plan; MPPI publishes its optimal trajectory when visualization is enabled). If one is missing, roll the current command forward with a unicycle model as a fallback.

## 3. Robot & failure definitions

| Item | Value |
|---|---|
| Robot | Jackal (≈ 0.5 × 0.43 m footprint), differential drive |
| Sensor | 2D LiDAR, 270° FOV, ~10 m range, 10–40 Hz (as in [1], [3]) |
| Localization | **Ground-truth odometry** from Gazebo (isolates planning failures) |
| Map | No prior map. Rolling costmaps in the odom frame, unknown space allowed for planning (as in BARN) |
| Max speed | {0.5, 1.0, 1.5} m/s training/test; **2.0 m/s** only in OOD tests |

**Failure events (automatic):**
| Event | Rule |
|---|---|
| Controller failure | The controller server reports FAILED (no valid trajectory), or an action abort with that code |
| Collision | A Gazebo contact sensor fires on the robot body (backup check: min LiDAR range < footprint margin) |
| Stuck | < 0.1 m progress in 5 s (Nav2 progress checker semantics) |

**Episode outcome:** *success* (reached the goal, no collision), *collision*, *timeout* (> 3× nominal time), or *stuck-abort*.

## 4. Worlds

| Set | Description | Count | Purpose |
|---|---|---|---|
| **W1 – BARN-style** | Cellular-automaton clutter as in the BARN dataset [5] (30 × 30 grid ≈ 4.5 × 4.5 m navigation area plus margins), easy → very hard | 300 | Main train/calibration/test distribution |
| **W2 – Rooms with occlusion** | Connected rooms like [1]: doorways 1 m / 2 m × sparse/dense clutter, with **hidden walls/dead ends behind obstacles** | ~20 (4 types × variations) | Reproduces the primary paper's motivating failure |
| **W3 – Out-of-distribution** | Denser fill than training; gaps of robot width + 10 cm; new shapes (thin poles, L-walls); **moving obstacles**; speed 2.0 m/s | ~30 | Tests generalization, where uncertainty (β) should help |

**Getting BARN into Gazebo Harmonic:** no ROS 2 port exists (checked Nov 2026). Options (⚠️ decide in week 1):
1. Convert the original BARN `.world` files to SDF for Harmonic with a script (they are simple static shapes), **or**
2. Regenerate worlds with the published cellular-automaton procedure [5] directly into SDF.
Either way, we keep the BARN difficulty ordering so results remain interpretable.

**Split (by world, never by time step):** W1 worlds 0–179 train · 180–239 calibration · 240–299 test. W2 and W3 are **test only**.

## 5. Data pipeline

```
episode_runner ──► rosbag2 (mcap): /scan /odom /plan /local_plan /cmd_vel /contacts controller status
                    │
                    ▼  offline extraction script (Python)
              per-time-step table (parquet):
              episode_id, world, speed, controller, t, features f1–f5 (current state),
              predicted-state features (K × f), event flags, label y (failure in (t, t+h]),
              label y_eval (failure in (t, t+3 s])
                    │
                    ├──► train GP (W1 train)        ├──► conformal ψ̂ (W1 calibration, passive logs)
                    └──► offline detection eval (W1 test, W2, W3)
```

**Data volume estimate:** 180 train worlds × 3 speeds × 2 controllers × 1–3 runs = ~1,000–3,200 episodes × ~20–40 s × 10 Hz, roughly 0.2–1 M samples. We keep all positives and subsample negatives for GP training.
**Simulation time:** with headless Gazebo at a real-time factor > 1 and 2–3 parallel containers, the data collection fits in a few overnight runs. ⚠️ Measure the real-time factor in week 1.

## 6. Reproducibility rules

- Fixed random seeds per (world, run). All configs in YAML, versioned in git.
- One script per experiment: `run_experiment.py --config configs/exp_X.yaml`.
- Every result table is generated from logs by a script; nothing is hand-copied.
- Docker image tag recorded in every log.
