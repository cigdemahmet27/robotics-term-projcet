# YZV406E Robotics: Term Project Research Repo

**Topic:** Robot Failure Detection & Recovery for mobile-robot navigation
**Working title:** *Know Before You Fail: Calibrated, Uncertainty-Aware Prediction of and Recovery from Navigation Failures in ROS 2*
**Course:** YZV406E Robotics, Prof. Dr. Sanem Sarıel, Fall 2026–27
**Status:** 📚 Research & proposal stage (no code yet)

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
| 2 | [`GLOSSARY.md`](GLOSSARY.md) | Every robotics/ML term we use, explained simply. Keep it open while reading |
| 3 | [`01_papers/README.md`](01_papers/README.md) | Overview + comparison of the 5 papers |
| 4 | `01_papers/1_…` → `5_…` | One careful explanation per paper + "what we can take from it" |
| 5 | [`02_primary_paper_deep_dive/`](02_primary_paper_deep_dive/README.md) | ⭐ The primary paper: simple explanation → background knowledge → walkthrough → limitations. **New teammates: read `00_simple_explanation.md` first** |
| 6 | [`03_our_project/`](03_our_project/README.md) | ⭐ What *we* are going to build and test, in detail |
| 7 | [`04_proposal/Proposal_Draft.md`](04_proposal/Proposal_Draft.md) | The proposal text we submit (≤ 4 pages + 1 page refs) |

## 📁 Folder structure

```
Term_projcet/
├── README.md                     ← you are here
├── GLOSSARY.md                   ← terms explained (A–Z)
├── CLAUDE.md                     ← notes for the AI assistant
│
├── 00_course/                    ← official course material
│   ├── Project Proposal Guidelines.pdf
│   └── requirements_summary.md
│
├── 01_papers/                    ← the 5 papers the team collected
│   ├── README.md                 ← comparison table + which paper plays which role
│   ├── pdf/                      ← original PDFs
│   ├── 1_GP_failure_prediction_ICRA2024.md   ⭐ primary (short version)
│   ├── 2_Air_Bumper_ICRA2024.md
│   ├── 3_DDP_IROS2025.md
│   ├── 4_LA-LIO_IROS2024.md
│   └── 5_Interactive-FAR_IROS2024.md
│
├── 02_primary_paper_deep_dive/   ← the primary paper, explained from zero
│   ├── README.md
│   ├── 00_simple_explanation.md     ← ⭐ start here: the project in simple words
│   ├── 01_background_knowledge.md   ← what the paper assumes you already know
│   ├── 02_paper_walkthrough.md      ← section by section, every equation
│   └── 03_limitations_and_open_questions.md
│
├── 03_our_project/               ← OUR plan, in detail
│   ├── README.md
│   ├── 01_problem_and_goals.md
│   ├── 02_method.md                 ← Contribution 1 + Contribution 2
│   ├── 03_system_and_environment.md ← ROS 2 / Nav2 / Gazebo / data
│   ├── 04_evaluation_plan.md        ← RQs, baselines, metrics, experiments
│   └── 05_team_timeline_risks.md
│
├── 04_proposal/
│   └── Proposal_Draft.md         ← submission text
│
└── 99_archive/                   ← old / superseded notes (kept for history)
    ├── Project_Options.md        ← early topic comparison (Sentinel etc.), superseded
    └── Paper_Analysis.md         ← first quick analysis, superseded by 01_papers/
```

## 🧭 The project in 30 seconds

1. **The primary paper** (Mohammad, Higgins, Bezzo, ICRA 2024) teaches a robot to *predict* that its motion planner is about to fail, stop early, and move to a safe spot.
2. **Its weaknesses:** it works for one planner type only, it ignores its own uncertainty, its alarm threshold is hand-tuned, and its recovery can loop back into the same trap.
3. **We will:** (C1) make the predictor work with any ROS 2 Nav2 planner, use its uncertainty, and *calibrate* the alarm so false alarms stay ≤ α; (C2) make recovery goal-aware, with a fading memory of where failures happened.
4. **We test it** in Gazebo simulation with ROS 2 + Nav2 on a Jackal robot, over hundreds of cluttered worlds. Zero cost, no real robot.

## 🏷️ Conventions

- Status markers: ✅ done/verified · ⚠️ needs attention · ❌ not available · ⭐ important
- Citations use IEEE style, `[n]`, numbered as in `04_proposal/Proposal_Draft.md`.
- "Paper 1/2/3/4" always means the numbering in `01_papers/`.
