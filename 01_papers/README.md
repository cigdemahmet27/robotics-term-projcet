# 01 – Papers: Overview

The team collected four papers. All four were read in full (07.11.2026). Venues and years were verified from each PDF's IEEE header ✅.

| # | Short name | Full title | Venue | Pages | DOI | Role in our project |
|---|---|---|---|---|---|---|
| 1 ⭐ | **GP failure prediction** | A GP-based Robust Motion Planning Framework for Agile Autonomous Robot Navigation and Recovery in Unknown Environments | ICRA 2024 | 2418–2424 | 10.1109/ICRA57147.2024.10610382 | **Primary paper** |
| 2 | **Air Bumper** | Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation | ICRA 2024 | 15735–15741 | 10.1109/ICRA57147.2024.10611410 | Secondary: failure-memory map idea, reactive baseline |
| 3 | **DDP** | Decremental Dynamics Planning for Robot Navigation | IROS 2025 | 4559–4565 | 10.1109/IROS60139.2025.11246580 | Secondary: BARN benchmark, metrics, speed protocol |
| 4 | **LA-LIO** | LA-LIO: Robust Localizability-Aware LiDAR-Inertial Odometry for Challenging Scenes | IROS 2024 | 10145–10152 | 10.1109/IROS58592.2024.10802825 | Secondary: hysteresis switching; localization failures as future work |

PDFs are in [`pdf/`](pdf/).

## How the four papers relate to each other

They all deal with **things going wrong while a robot navigates an unknown place**, but at different layers of the navigation stack and at different times:

```
                 BEFORE failure (proactive)          AFTER failure (reactive)
               ┌─────────────────────────────┐   ┌──────────────────────────┐
 Planning layer│ Paper 1: predict planner    │   │ Paper 3: speed-based     │
               │ failure, then recover       │   │ recovery mode (simple)   │
               │ Paper 3: better planner     │   │                          │
               │ (fewer failures)            │   │                          │
               ├─────────────────────────────┤   ├──────────────────────────┤
 Physical layer│                             │   │ Paper 2: detect collision│
 (contact)     │                             │   │ by IMU, bounce, remember │
               ├─────────────────────────────┤   ├──────────────────────────┤
 Localization  │ Paper 4: detect "LiDAR can't│   │                          │
 layer         │ localize here" & adapt      │   │                          │
               └─────────────────────────────┘   └──────────────────────────┘
```

Our project lives in the **top-left box** (Paper 1). We take one idea each from Papers 2, 3 and 4.

## Comparison for our needs

| Criterion | 1 GP | 2 Air Bumper | 3 DDP | 4 LA-LIO |
|---|---|---|---|---|
| Is it about failure detection **and** recovery? | ✅✅ both, proactive | ✅ both, reactive | ⚠️ planning (recovery minor) | ⚠️ detection only (localization) |
| Robot type | Ground (Jackal), also Spot | Drone (MAV) | Ground (Jackal) | Handheld/robot 3D LiDAR |
| Sensors | 2D LiDAR | IMU + 3D LiDAR | 2D LiDAR | 3D LiDAR + IMU |
| Simulation used | Gazebo + ROS 1 | Gazebo + PX4 + ROS 1 | Gazebo + ROS 1 (BARN) | None (datasets) |
| Ease of moving to ROS 2 + Nav2 | ✅ high | ⚠️ low (drone stack) | ✅ high | ⚠️ low (C++ odometry) |
| Compute | CPU | GPU mapper | CPU | CPU |
| Public code | ⚠️ not found | ✅ ROS 1 | ✅ ROS 1 | ❌ not found |
| Size of what we'd reimplement | Small | Large | Medium | Large |

## Decision

**Paper 1 is the primary paper.** The reasons are explained in [`1_GP_failure_prediction_ICRA2024.md`](1_GP_failure_prediction_ICRA2024.md) and in depth in [`../02_primary_paper_deep_dive/`](../02_primary_paper_deep_dive/README.md).

⚠️ Open item: check citation counts on Google Scholar for all four (the "well-cited" requirement).
