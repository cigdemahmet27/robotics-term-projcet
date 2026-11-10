# Proposal Template & Checklist (Air Bumper project)

> Use this file to check what is done and what is left. The finished text is in [`Proposal_AirBumper.md`](Proposal_AirBumper.md). The source of every rule here is [`../00_course/Project Proposal Guidelines.pdf`](../00_course/Project%20Proposal%20Guidelines.pdf). If they disagree, the PDF wins.

## 0. 🚨 Submission today: 10.11.2026

- [x] Team names and student IDs (3 members) in the title block and Table V
- [x] "Well-cited": 7 citing papers on Semantic Scholar (Nov. 2026), incl. Nature Communications and IEEE/ASME T-Mech; mentioned in Section 1. ⚠️ Optionally replace with the Google Scholar count
- [x] Friend's review applied (shorter title, citation evidence, Haddadin claim fixed, FA-rate test set + Poisson intervals, RQ3 rephrased, compute budget, risks, topic statement)
- [x] Platform changed back to the **aerial robot** (caged quadrotor, PX4 SITL), as the team wants to improve the drone itself
- [x] PDF in IEEE two-column format: [`Proposal_AirBumper.pdf`](Proposal_AirBumper.pdf) (source: `latex/proposal.tex`). **4 pages in total, references included** ✅ within the limit
- [ ] After any text change: edit `latex/proposal.tex`, run `pdflatex proposal.tex` twice in `latex/`, copy `proposal.pdf` to `Proposal_AirBumper.pdf`
- [ ] One member submits (only one!). Late submission is **not accepted**. Not submitting = **VF**
- [ ] Everyone reads the final PDF once before submission

## 1. What the homework asks (analysis)

| Rule from the PDF | What it means for us | Status |
|---|---|---|
| Group project on a specific robotics problem with a suitable solution | Problem: collisions with obstacles the LiDAR can't see, and memory of them | ✅ |
| Suggested topic list | "**Robot Failure Detection and Recovery**", our exact topic | ✅ |
| Physical robot not required | Gazebo simulation + ROS 2 only | ✅ |
| Primary paper from the approved list | Air Bumper, **ICRA** | ✅ |
| Published 2024–2026 | 2024 | ✅ |
| Original research (not a survey/review/tutorial) | Yes, a new framework + experiments | ✅ |
| **Well-cited** | Not checked yet | ⚠️ |
| Critically analyze the paper, find limitations / open questions | L1–L7 in Section 3 | ✅ |
| Propose a **potentially effective solution** | C1 (calibrated detection) + C2 (forgetting memory) | ✅ |
| **Go beyond implementing an existing system** | The reimplementation is only baseline B1. C1 and C2 are new | ✅ |
| **Systematic investigation, well-designed evaluation** | 4 RQs, 6 baselines, metrics, 6 experiments, statistics | ✅ |
| Feasible, clearly scoped | Python nodes, PX4 SITL, lightweight voxel planner, minimum/target/stretch levels | ✅ |
| Each student has distinct, documented responsibilities | 3 named roles in Section 6 (Table V) | ✅ |
| Complete references, one consistent style | IEEE style, [1]–[13], all cited in the text | ✅ |
| Max 4 pages + 1 page of references | PDF is 4 pages including references | ✅ |

**Graders' point of view:** the instructors work on **failure detection and recovery**. They will look closely at:
- how we **measure** detection (false alarms, misses, latency)
- whether thresholds are **justified**
- whether the baselines are fair

All three are central in our plan. Keep it that way when shortening.

## 2. Section-by-section template

Each section shows: what the PDF requires → what must be inside → our status → a page budget.

### Title
- **Required:** clear, concise, reflects the problem **and** the main idea.
- **Must contain:** the problem (collisions with invisible/moving obstacles) + the idea (calibrated detection, forgetting memory).
- **Ours:** "Bump Once, Remember Wisely: Calibrated IMU Collision Detection and Evidence-Aware, Forgetting Collision Memory for Mobile Robots Among Invisible and Moving Obstacles" ✅
- If too long, use the short version: "Calibrated Collision Detection and Forgetting Collision Memory for Mobile Robots Among Invisible Obstacles".

### 1. Problem Description and Primary Reference Paper (~0.6 page)
- [x] The problem in 3–5 sentences (glass and blind-spot obstacles → collisions can't be avoided → detect, react, remember → permanent memory creates phantom walls)
- [x] Full citation of the primary paper + venue + year
- [x] What the paper does (detect → estimate → react → remember, with its numbers)
- [x] **Why this paper**: significance, impact, relevance to our problem
- [x] Significance (indoor MAVs for inspection/warehouses/search and rescue fly near glass and people)
- [ ] ⚠️ Add the citation count once checked ("cited N times as of Nov 2026")

### 2. Environment, Benchmark, or Dataset (~0.6 page)
- [x] Simulator, middleware, robot, sensors, **and why each one**
- [x] How "invisible" obstacles are simulated (collision without visual)
- [x] How moving/transient obstacles are simulated
- [x] Scenario sets S1–S4 with counts and purpose
- [x] Ground truth (contact sensor, not used by the detector)
- [x] Train/calibration/test split by world, plus an out-of-distribution set
- [x] Data volume and feasibility

### 3. Challenges and Limitations (~0.6 page)
- [x] Each limitation with **evidence** from the paper (section/equation)
- [x] **Why it matters** for each
- [x] The open research questions that follow
- Key items: L1 fixed threshold, L2 per-axis miss, L5 permanent marks, L7 no detection metrics.

### 4. Proposed Method and Contributions (~1.1 pages)
- [x] Clear separation: **baseline reimplementation** vs **our contributions**
- [x] C1 in technical detail (residual, features, score, conformal threshold, footprint contact)
- [x] C2 in technical detail (LiDAR evidence, decaying belief, merging, graded keepout + speed masks)
- [x] Each contribution mapped to the limitations it fixes
- [x] Expected improvements, stated explicitly
- [x] Implementation plan (ROS 2 Python nodes + PX4 offboard setpoints)

### 5. Research Questions and Evaluation Plan (~0.8 page)
- [x] Specific RQs (RQ1–RQ4)
- [x] Baselines including **the primary paper** (B1) and a fair tuned version of it
- [x] Metrics with definitions (detection, estimation, memory, system)
- [x] Experiments table linking each experiment to its RQ
- [x] Ablations (E6)
- [x] Statistics + threats to validity

### 6. Team Roles and Individual Contributions (~0.3 page)
- [ ] ⚠️ Real names
- [x] Distinct role per member + concrete deliverables
- [x] How contributions are documented (git history, `CONTRIBUTIONS.md`)
- [x] Timeline with milestones; minimum/target/stretch scope

### References (separate page, max 1)
- [x] Complete entries (authors, title, venue, year, pages, DOI)
- [x] One style (IEEE)
- [x] Every reference is cited in the text, and every citation is in the list

## 3. If we are over 4 pages: what to cut first

1. Shorten the timeline to one line (it isn't a required section).
2. Remove S4 (stretch) from the scenario table.
3. Merge baselines B3/B4 into one row "ablations of C1/C2".
4. Shorten "Threats to validity" to one sentence.
5. Keep untouched: the limitations table, C1/C2 details, metrics, baselines.

## 4. Things to verify in week 1 (written as ⚠️ in the proposal)

| Item | How to check | Fallback |
|---|---|---|
| GPU LiDAR ignores models without visuals (our "glass") | Spawn a collision-only box and look at `/scan` in RViz | Use a LiDAR visibility mask / custom sensor filter |
| A collision produces a clear IMU spike in Gazebo | Drive into a wall at 0.5/1.0/1.5 m/s and plot `/imu` | Raise IMU rate, tune contact stiffness |
| Contact sensor gives time + position of the contact | Echo `/contacts` during a crash | Use the min LiDAR range to the true obstacle geometry |
| Caged x500 survives contacts in PX4 SITL without flipping | Fly into a wall at 0.5/1.0/1.5 m/s | Tune contact stiffness, lower the speed range |
| Moving models in Gazebo Harmonic (joint/trajectory controller) | Box crossing a corridor | Teleport with a `set_pose` service at a fixed rate |

## 5. After round 1

- The instructors may ask for revisions. Each section owner applies the changes for their section in round 2.
- Keep `Proposal_AirBumper.md` as the single source of truth. Regenerate the PDF from it.
