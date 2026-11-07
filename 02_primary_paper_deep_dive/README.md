# 02 – Primary Paper Deep Dive

**Paper:** N. Mohammad, J. Higgins, N. Bezzo, *"A GP-based Robust Motion Planning Framework for Agile Autonomous Robot Navigation and Recovery in Unknown Environments,"* ICRA 2024, pp. 2418–2424. PDF: [`../01_papers/pdf/1_GP_failure_prediction_ICRA2024.pdf`](../01_papers/pdf/1_GP_failure_prediction_ICRA2024.pdf)

The paper is 7 pages written for robotics researchers, so it **assumes a lot of background knowledge**. This folder explains it from zero.

| File | Content | Read when |
|---|---|---|
| [`01_background_knowledge.md`](01_background_knowledge.md) | Everything the paper assumes you know: navigation stacks, maps, planners, safe corridors, Bézier curves, MPC, Gaussian Processes, probability/risk, sampling, ROS/Gazebo | **First**, before opening the PDF |
| [`02_paper_walkthrough.md`](02_paper_walkthrough.md) | Section-by-section explanation of the paper, every equation in words, with a toy numeric example | Read side by side with the PDF |
| [`03_limitations_and_open_questions.md`](03_limitations_and_open_questions.md) | Critical analysis: what is missing or weak, with evidence, and what that means for us | After the walkthrough. This feeds our proposal Section 4 |

**Goal:** after reading these three files, every team member should be able to explain the paper on a whiteboard and answer the instructor's questions about it.
