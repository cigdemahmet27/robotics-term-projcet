# 01 – Papers: Overview

The team collected five papers. All five were read in full (07.11.2026; paper 5 added later the same day). Venues and years were verified from each PDF's IEEE header ✅.

| # | Short name | Full title | Venue | Pages | DOI | Role in our project |
|---|---|---|---|---|---|---|
| 1 | **GP failure prediction** | A GP-based Robust Motion Planning Framework for Agile Autonomous Robot Navigation and Recovery in Unknown Environments | ICRA 2024 | 2418–2424 | 10.1109/ICRA57147.2024.10610382 | Related work (proactive failure prediction). Was primary until 10.11.2026 |
| 2 ⭐ | **Air Bumper** | Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation | ICRA 2024 | 15735–15741 | 10.1109/ICRA57147.2024.10611410 | ⭐ **Primary paper** (decided 10.11.2026) |
| 3 | **DDP** | Decremental Dynamics Planning for Robot Navigation | IROS 2025 | 4559–4565 | 10.1109/IROS60139.2025.11246580 | Secondary: BARN benchmark, metrics, speed protocol |
| 4 | **LA-LIO** | LA-LIO: Robust Localizability-Aware LiDAR-Inertial Odometry for Challenging Scenes | IROS 2024 | 10145–10152 | 10.1109/IROS58592.2024.10802825 | Secondary: hysteresis switching; localization failures as future work |
| 5 | **Interactive-FAR** | Interactive-FAR: Interactive, Fast and Adaptable Routing for Navigation Among Movable Obstacles in Complex Unknown Environments | IROS 2024 | 5402–5409 | 10.1109/IROS58592.2024.10802169 | Secondary: try → sense → update cost → re-plan loop; SPL metric |

PDFs are in [`pdf/`](pdf/).

## How the five papers relate to each other

They all deal with **things going wrong while a robot navigates an unknown place**, but at different layers of the navigation stack and at different times:

```
                 BEFORE failure (proactive)          AFTER failure (reactive)
               ┌─────────────────────────────┐   ┌──────────────────────────┐
 Planning layer│ Paper 1: predict planner    │   │ Paper 3: speed-based     │
               │ failure, then recover       │   │ recovery mode (simple)   │
               │ Paper 3: better planner     │   │                          │
               │ (fewer failures)            │   │                          │
               │ Paper 5: push movable       │   │ Paper 5: push failed →   │
               │ obstacles instead of detour │   │ update cost, re-plan     │
               ├─────────────────────────────┤   ├──────────────────────────┤
 Physical layer│                             │   │ Paper 2: detect collision│
 (contact)     │                             │   │ by IMU, bounce, remember │
               ├─────────────────────────────┤   ├──────────────────────────┤
 Localization  │ Paper 4: detect "LiDAR can't│   │                          │
 layer         │ localize here" & adapt      │   │                          │
               └─────────────────────────────┘   └──────────────────────────┘
```

Our project builds on **Paper 2** (physical layer: detect a collision, react, remember it), applied to a ground robot. Papers 1 and 5 are cited as related work.

## Comparison for our needs

| Criterion | 1 GP | 2 Air Bumper | 3 DDP | 4 LA-LIO | 5 Interactive-FAR |
|---|---|---|---|---|---|
| Is it about failure detection **and** recovery? | ✅✅ both, proactive | ✅ both, reactive | ⚠️ planning (recovery minor) | ⚠️ detection only (localization) | ⚠️ planning; failure handling is a small rule |
| Robot type | Ground (Jackal), also Spot | Drone (MAV) | Ground (Jackal) | Handheld/robot 3D LiDAR | Ground vehicle (diff drive) |
| Sensors | 2D LiDAR | IMU + 3D LiDAR | 2D LiDAR | 3D LiDAR + IMU | 3D LiDAR + 1-D force sensor |
| Simulation used | Gazebo + ROS 1 | Gazebo + PX4 + ROS 1 | Gazebo + ROS 1 (BARN) | None (datasets) | Simulation only (simulator not named) |
| Ease of moving to ROS 2 + Nav2 | ✅ high | ⚠️ low (drone stack) | ✅ high | ⚠️ low (C++ odometry) | ⚠️ low (pushing physics) |
| Compute | CPU | GPU mapper | CPU | CPU | CPU |
| Public code | ⚠️ not found | ✅ ROS 1 | ✅ ROS 1 | ❌ not found | ✅ Docker demo |
| Size of what we'd reimplement | Small | Large | Medium | Large | Large |

## Decision

**Paper 2 (Air Bumper) is the primary paper** (team decision, 10.11.2026). See [`2_Air_Bumper_ICRA2024.md`](2_Air_Bumper_ICRA2024.md) and the proposal [`../04_proposal/Proposal_AirBumper.md`](../04_proposal/Proposal_AirBumper.md). The earlier GP-based plan is kept in [`../99_archive/gp_direction/`](../99_archive/gp_direction/).

⚠️ Open item: check the Google Scholar citation count of Air Bumper (the "well-cited" requirement).
