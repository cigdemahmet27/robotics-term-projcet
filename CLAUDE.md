# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

This is the term project for YZV406E Robotics (Prof. Dr. Sanem Sarıel, Fall 2026–27). It is a **research repo at the proposal stage**: there is no code, build system, or tests yet. Topic: failure detection & recovery for mobile-robot navigation (ROS 2 Nav2 + Gazebo). `README.md` holds the folder map and reading order. Update this file once an implementation exists.

Layout:
- `00_course/`: `Project Proposal Guidelines.pdf` (the **authoritative source**; read it with `pdftotext`) and `requirements_summary.md`.
- `01_papers/`: the 5 team papers (`pdf/`) plus one explanation file per paper and a comparison `README.md`. Venues and years were verified from the PDFs.
- `02_primary_paper_deep_dive/`: the **primary paper** (Mohammad, Higgins, Bezzo, ICRA 2024, GP-based proactive failure prediction + recovery): background knowledge, walkthrough, limitations L1–L8.
- `03_our_project/`: our plan. C1 = planner-agnostic, uncertainty-aware (Φ(m+βs)), conformal-calibrated risk with hysteresis. C2 = progress-aware recovery with a decaying failure-memory costmap mask. Also system/env, evaluation (RQ1–3, baselines B0–B3), team/timeline.
- `04_proposal/Proposal_Draft.md`: the submission text (keep it consistent with `03_our_project/`).
- `99_archive/`: superseded notes (old Sentinel-based options).
- `GLOSSARY.md`: definitions of terms.

## Hard constraints from the guidelines

- **Deadlines (guidelines updated 05.11.2026):** first-round proposal is due **10.11.2026**. Not submitting a draft means a VF grade. Round 1 may come back with revision requests, which must be made in round 2 (no date announced yet). Presentations are on 30.12.2026. Late submissions are not accepted, and only one group member submits. The PDF overrides any older announcements.
- **Length:** max 4 pages, plus 1 page for references.
- **Primary paper:** an original research paper (not a survey, review, or tutorial), well-cited, published **2024–2026**, from the approved conference/journal list in the PDF. Always **verify the venue and year of any new paper** against the PDF's list.
- **Required proposal sections, in this order:** Title; Problem Description & Primary Reference Paper; Environment, Benchmark, or Dataset; Challenges & Limitations; Proposed Method & Novel Contributions; Research Questions & Evaluation Plan; Team Roles & Individual Contributions; References. Use one consistent citation style throughout.
- **The project must go beyond reimplementation:** it has to identify a limitation of the primary paper and propose a potentially effective solution (the updated PDF dropped the word "novel"). The evaluation needs research questions, baselines, metrics, and planned experiments. Ablations are no longer required, but include them anyway.
- Each team member needs distinct, documented responsibilities.
- A physical robot is not required. Simulation (MuJoCo, PyBullet, LIBERO, robosuite, and similar) and public benchmarks are acceptable.

## Working notes

- Written documents are Markdown, and the tables use emoji status markers (✅ ⚠️ ❌ ⭐). Keep new documents in the same style and place them in the numbered folders.
- When the method changes, update `03_our_project/` **and** `04_proposal/Proposal_Draft.md` together.
- The graders are specialists in failure detection and recovery, so methods and evaluation in that area need to be rigorous.
