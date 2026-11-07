# Course Requirements: Summary

> The authoritative source is `Project Proposal Guidelines.pdf` in this folder. This file only summarizes it. If they disagree, the PDF wins.

## Deadlines & submission

| Item | Rule |
|---|---|
| Round-1 proposal | **10.11.2026**, mandatory. No submission = **VF** grade |
| Round-2 | The instructors may request revisions. We apply them in round 2 (date TBA) |
| Presentation | **30.12.2026** |
| Length | **Max 4 pages + 1 extra page for references** |
| Who submits | Only **one** member, on behalf of the group |
| Late | ❌ Not accepted |

## Required sections (in this order)

1. **Title**: concise, reflects the problem + main idea
2. **Problem Description & Primary Reference Paper**: define the problem, name the paper, justify it (significance, impact)
3. **Environment, Benchmark, or Dataset**: simulator/benchmark/dataset + why it fits
4. **Challenges & Limitations**: critical analysis of the primary paper; why the gaps matter
5. **Proposed Method & Novel Contributions**: technical detail, how it differs, expected improvements
6. **Research Questions & Evaluation Plan**: RQs, baselines, metrics, experiments
7. **Team Roles & Individual Contributions**: distinct responsibility for each student
8. **References**: complete, one consistent style

## Primary paper rules

- ✅ From the approved list (e.g., ICRA, IROS, CoRL, RSS, NeurIPS, ICLR, ICML, AAAI, IJCAI, CVPR, ICCV, HRI; T-RO, RA-L, IJRR, Science Robotics, RAS, Autonomous Robots, …)
- ✅ Published **2024–2026**
- ✅ **Original research**: not a survey, review, editorial or tutorial
- ⚠️ **Well-cited**: check Google Scholar for our primary paper
- Older works may be used as *additional* references

## What the graders look for

- "**Go beyond simply implementing an existing system**": identify a limitation and propose a potentially effective solution
- A **systematic investigation** with a well-designed experimental evaluation
- **Feasible, clearly scoped**
- A physical robot is optional. **Simulation is fine.**
- The graders' research area is **failure detection & recovery**, so expect a rigorous review of our detection metrics and methodology.

## Our own constraints (team decisions)

| Constraint | Consequence |
|---|---|
| 💸 Zero budget | No paid APIs, no rented GPUs |
| 🤖 ROS 2 + simulation only | Gazebo + Nav2; every component is a ROS 2 node |
| 🖥️ Laptop: Ubuntu 26.04, RTX 4060 8 GB | Work runs in a ROS 2 Jazzy Docker container (Nav2 was not released for Lyrical at the last check) |
| 📏 Small scope | 2 main contributions |

## ✅ Checklist of our proposal against the guidelines

| Requirement | Where we satisfy it | Status |
|---|---|---|
| Approved venue, 2024–26 | ICRA 2024 | ✅ |
| Original research | Yes | ✅ |
| Well-cited | Check Scholar | ⚠️ |
| Limitation identified | `02_primary_paper_deep_dive/03_…` | ✅ |
| Solution proposed | `03_our_project/02_method.md` | ✅ |
| RQs + baselines + metrics + experiments | `03_our_project/04_evaluation_plan.md` | ✅ |
| Distinct roles | `03_our_project/05_…` | ⚠️ names missing |
| ≤ 4 + 1 pages | `04_proposal/` | ⚠️ check after formatting |
