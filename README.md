# YZV406E Robotics: Term Project Research Repo

**Topic:** Robot Failure Detection & Recovery for mobile-robot navigation
**Primary paper:** ⭐ *Air Bumper* (Wang et al., ICRA 2024): IMU-based collision detection, reaction and collision-aware mapping
**Title:** *Bump Once, Remember Wisely: Calibrated Collision Detection and Forgetting Memory for Robots Among Invisible Obstacles*
**Team:** Egemen Çevik, Cumali Sarı, Ahmet Enes Çiğdem
**Course:** YZV406E Robotics, Prof. Dr. Sanem Sarıel, Fall 2026–27
**Status:** 📚 Proposal stage (no code yet)

| Date | Milestone |
|---|---|
| **10.11.2026** | ⚠️ Round-1 proposal deadline (missing it = VF grade) |
| TBA | Round-2 (revised) proposal |
| **30.12.2026** | Project presentation |

---

## 🗺️ How to read this repo (suggested order)

| Step | Read | Why |
|---|---|---|
| 1 | [`00_course/requirements_summary.md`](00_course/requirements_summary.md) | What the instructor wants from us |
| 2 | [`04_proposal/00_Template_and_Checklist.md`](04_proposal/00_Template_and_Checklist.md) | ⭐ What each proposal section needs, and what is still left to do |
| 3 | [`01_papers/2_Air_Bumper_ICRA2024.md`](01_papers/2_Air_Bumper_ICRA2024.md) | ⭐ The primary paper explained |
| 4 | [`04_proposal/Proposal_AirBumper.md`](04_proposal/Proposal_AirBumper.md) | ⭐ The full proposal text we submit (≤ 4 pages + 1 page refs) |
| 5 | [`01_papers/README.md`](01_papers/README.md) | The other papers and how they relate |
| — | [`GLOSSARY.md`](GLOSSARY.md) | Any unknown term, explained simply |

## 📁 Folder structure

```
Term_projcet/
├── README.md                     ← you are here
├── GLOSSARY.md                   ← terms explained (A–Z)
├── CLAUDE.md                     ← notes for the AI assistant (you can ignore it)
│
├── 00_course/                    ← what the teacher wants
│   ├── Project Proposal Guidelines.pdf
│   └── requirements_summary.md
│
├── 01_papers/                    ← the 5 papers the team collected
│   ├── README.md                 ← comparison + which paper plays which role
│   ├── pdf/                      ← original PDFs
│   ├── 1_GP_failure_prediction_ICRA2024.md
│   ├── 2_Air_Bumper_ICRA2024.md  ⭐ primary
│   ├── 3_DDP_IROS2025.md
│   ├── 4_LA-LIO_IROS2024.md
│   └── 5_Interactive-FAR_IROS2024.md
│
├── 04_proposal/                  ← what we submit
│   ├── 00_Template_and_Checklist.md  ← ⭐ homework analysis + section template + to-do list
│   └── Proposal_AirBumper.md         ← ⭐ the proposal text
│
└── 99_archive/                   ← old material, kept for history (you can ignore it)
    ├── gp_direction/             ← the earlier GP-paper plan (deep dive, plan, draft)
    ├── Project_Options.md
    └── Paper_Analysis.md
```

## 🧭 The project in 30 seconds

1. **The problem:** a robot's LiDAR can't see glass, and can't see things coming from its blind spot. So collisions will happen.
2. **The primary paper (Air Bumper)** detects a collision from an IMU spike (fixed threshold 20 m/s²), bounces away, and draws the obstacle into the map **forever**.
3. **Its weaknesses:**
   - The fixed threshold gives an unknown number of false alarms and misses (e.g., the robot's own braking, diagonal hits).
   - Permanent marks turn people, moved carts, opened doors and false alarms into **phantom walls**.
   - There are no detection statistics.
4. **We will:**
   - **(C1)** detect collisions from the *unexpected* acceleration (measured minus what the robot's own commands explain), with a threshold **calibrated** to a chosen false-alarm rate.
   - **(C2)** use a collision memory that **checks the LiDAR** (glass or a visible object?), **fades over time** unless the obstacle is hit again, and slows the robot near uncertain spots instead of blocking them.
5. **We test it** with ROS 2 + Nav2 + Gazebo on a Jackal robot, in generated worlds with glass, moving/removed obstacles, and bumpy floors. Zero cost, no real robot.

## 🏷️ Conventions

- Status markers: ✅ done/verified · ⚠️ needs attention · ❌ not available · ⭐ important
- Citations use IEEE style, `[n]`, numbered as in `04_proposal/Proposal_AirBumper.md`.
- "Paper 1/2/3/4/5" always means the numbering in `01_papers/`.
