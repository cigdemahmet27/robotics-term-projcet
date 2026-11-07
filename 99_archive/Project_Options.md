# YZV406E Robotics – Term Project Options (v2, updated guidelines)

> **First-round proposal deadline: 10.11.2026** · Presentations: 30.12.2026 · Second-round date: not announced yet
>
> ⚠️ **Not submitting a first draft means a VF grade.** In round 1 they may ask for revisions, which must be made in round 2.
> ⚠️ **Page limit: max 4 pages + 1 page for references.**

---

## Key Requirements (from the 05.11.2026 guidelines PDF)

| Requirement | Detail |
|---|---|
| Primary paper | Original research (no surveys), **well-cited**, published **2024–2026**, on the approved venue list |
| Core expectation | "Go beyond simply implementing an **existing** system": find a **limitation** and propose a **potentially effective solution** (the word "novel" is no longer required) |
| Evaluation | Research questions, baselines, metrics, planned experiments (ablations are no longer required, but we keep them because the graders work in this area) |
| Team | Each member needs **distinct, documented** responsibilities |
| Length | **≤ 4 pages + 1 page of references** |

## Our own constraints

| Constraint | Consequence |
|---|---|
| 💸 Zero budget | No GPT-4o or paid APIs, and no rented GPUs. Use open models locally (Ollama / llama.cpp / HF, 4-bit quantized) |
| 🖥️ RTX 4060 Laptop, 8 GB | Fits up to ~7–8B LLM/VLM at 4-bit, plus Diffusion Policy inference. **No** OpenVLA fine-tuning |
| 🤖 ROS 2, simulation only | Each component is a ROS 2 node (policy, monitor, planner, recovery). Simulator: MuJoCo (via a ROS 2 bridge node) or Gazebo. Ubuntu 26.04 → ROS 2 **Lyrical Luth** (verify install) |
| 📏 Small scope | **2 contributions**, not 3 |

### Required proposal sections (unchanged)
1. Title
2. Problem Description & Primary Reference Paper
3. Environment, Benchmark, or Dataset
4. Challenges & Limitations
5. Proposed Method & Contributions
6. Research Questions & Evaluation Plan
7. Team Roles & Individual Contributions
8. References

---

## Quick Comparison (only options that fit our constraints)

| # | Topic | Primary Paper | Venue checked | Compute | ROS 2 fit | Risk |
|---|---|---|---|---|---|---|
| A ⭐ | Failure Detection & Recovery | Sentinel, Agia et al. | ✅ CoRL 2024 | Medium | ✅ Good | ⚠️ Medium |
| B | Safe LLM Planning / Asking for help | Introspective Planning, Liang et al. | ✅ NeurIPS 2024 | Low | ✅ Good | ✅ Low |
| C | Multi-Robot Task Allocation | RoCo, Mandi et al. | ✅ ICRA 2024 | Low | ✅ Good | ✅ Low |

**Dropped:** OpenVLA/Octo (too much GPU), ReKep (needs Isaac Sim / OmniGibson), FoundationPose (weak robotics link), ATM (medium compute, no ROS 2 benefit).

---

## Option A ⭐ — Gated runtime failure monitoring with a free local VLM

**Primary paper:** C. Agia et al., *"Unpacking Failure Modes of Generative Policies: Runtime Monitoring of Consistency and Progress"* (Sentinel), **CoRL 2024**, arXiv:2410.04640
*Secondary:* J. Duan et al., *"AHA: A Vision-Language-Model for Detecting and Reasoning Over Failures in Robotic Manipulation"*, **ICLR 2025**

**What Sentinel does:** it monitors a Diffusion Policy at runtime with (1) a **fast statistical consistency check** that flags erratic behaviour when the action distributions shift over time, and (2) a **slow VLM progress check** that asks a VLM, through video Q&A, whether the task is still progressing. It needs no failure data for training.

**Limitations we target:**
- The progress check relies on a **large paid VLM** (GPT-4o class) queried **periodically**, which is expensive and slow, so failures are caught late
- It **only detects** failures and does nothing to recover

**Our 2 contributions:**
1. **Gated VLM querying:** the cheap consistency score runs every step and triggers the VLM **only when risk rises**. The VLM is a **small free local model** (e.g., Qwen2.5-VL-3B/7B, 4-bit).
2. **Triggered recovery:** on a confirmed failure, back off to the last safe state and re-run the policy (retry with a new random seed), with a retry budget.

**ROS 2 architecture:**
```
/sim_node (MuJoCo) ──/camera, /state──► /policy_node (Diffusion Policy) ──/action──► /sim_node
                                              │ /action_samples
                                              ▼
                                       /monitor_node (consistency score, every step)
                                              │ risk > τ
                                              ▼
                                       /vlm_node (local VLM, rare) ──failure──► /recovery_node
```

**Environment:** PushT or a robomimic task (Lift/Can) in MuJoCo, wrapped in ROS 2 nodes, with a pretrained Diffusion Policy checkpoint. Failures come from out-of-distribution perturbations: shifted object poses, pushes during execution, distractors.

**Research questions:**
- RQ1: Can a small free VLM replace GPT-4o in Sentinel's progress check, and how much detection accuracy does that cost?
- RQ2: Does gating cut VLM calls per episode without lowering the failure detection rate or increasing detection delay?
- RQ3: Does triggered recovery raise the task success rate under perturbations?

**Evaluation:**
- Baselines: consistency check only, VLM check only (periodic), Sentinel-style (both, periodic), ours (gated)
- Metrics: TPR / FPR, detection delay (steps), VLM calls per episode, wall-clock monitor time, task success with and without recovery
- Ablations: gate threshold τ, VLM size (3B vs 7B), recovery on/off

**Why it's the best fit:** Prof. Sarıel's research area, and "Robot Failure Detection and Recovery" is a suggested topic.
**Risk:** ⚠️ The hardest of the three to set up (Diffusion Policy + MuJoCo + ROS 2 bridge + local VLM). Check early whether Sentinel's code or Diffusion Policy checkpoints are public.

---

## Option B — Knowing when to ask for help: multimodal uncertainty for LLM planners

**Primary paper:** K. Liang, Z. Zhang, J. F. Fisac, *"Introspective Planning: Aligning Robots' Uncertainty with Inherent Task Ambiguity"*, **NeurIPS 2024**, arXiv:2402.06529 (code: github.com/kevinliang888/IntroPlan)
*Secondary:* A. Z. Ren et al., *"Robots That Ask For Help"* (KnowNo), CoRL 2023

**What it does:** an LLM planner reasons about its own uncertainty over candidate plans, using retrieved "introspective" examples combined with conformal prediction, and asks the human for clarification when the instruction is ambiguous.

**Limitations we target:**
- It considers only **language ambiguity**. It assumes perception is perfect, so a misdetected object leads to a confident wrong plan
- It was evaluated with large proprietary LLMs, and it's unclear whether it holds up with **small open models**

**Our 2 contributions:**
1. **Perception-aware uncertainty:** fuse object-detector confidence into the candidate-plan scores before the conformal step
2. **Run on a free local LLM** (Llama-3 / Qwen 7B, 4-bit), and measure how coverage and help rate change

**Environment:** a tabletop scene in Gazebo or MuJoCo with ROS 2 (`/detector_node` → `/planner_node` → `/executor_node`), plus the paper's released datasets. We inject perception noise: dropped or mislabelled objects.

**Research questions:**
- RQ1: Does adding perception uncertainty reduce confident-but-wrong plans under noisy detection?
- RQ2: Does the conformal coverage guarantee still hold with a small open LLM?

**Evaluation:**
- Baselines: plain LLM planner, KnowNo, Introspective Planning
- Metrics: success rate, help-request rate, empirical coverage vs target, unsafe-action rate

**Why pick it:** cheapest and safest to finish, and the code is public. **Weakness:** thinner link to "real" robot control.

---

## Option C — LLM decomposition plus classical allocation for multi-robot teams

**Primary paper:** Z. Mandi, S. Jain, S. Song, *"RoCo: Dialectic Multi-Robot Collaboration with Large Language Models"*, **ICRA 2024**, pp. 286–299, DOI 10.1109/ICRA57147.2024.10610855
*Secondary:* SMART-LLM (Kannan et al.). ⚠️ The IROS 2024 acceptance is **not confirmed**, so use it only as a secondary reference.

**What it does:** each robot arm runs an LLM agent. The agents discuss a plan in dialogue, propose subtasks and waypoints, and a motion planner checks them, feeding errors back. Tested on RoCoBench (6 MuJoCo tasks).

**Limitations we target:**
- Multi-round LLM dialogue is **slow and token-heavy**, and it gets worse with more robots
- There is **no guarantee** of a good or conflict-free allocation

**Our 2 contributions:**
1. **Hybrid planner:** one LLM call breaks the task into subtasks and constraints, and a **MILP / auction solver** (free: OR-Tools or PuLP) assigns them to robots
2. **Scaling study:** 2 → 4 robots, with a free local LLM instead of GPT-4

**Environment:** RoCoBench (MuJoCo, open source), wrapped as ROS 2 nodes (`/llm_decomposer`, `/allocator`, `/robot_i_controller`)

**Research questions:**
- RQ1: Does LLM + solver match RoCo's success rate with far fewer LLM tokens and less planning time?
- RQ2: How do both approaches scale as the number of robots grows?

**Evaluation:**
- Baselines: RoCo dialogue, a single central LLM planner, a pure classical planner
- Metrics: success rate, re-plans, LLM tokens per task, planning time

**Why pick it:** low compute, clean experiments, and the **easiest to split across team members** (each takes a module).

---

## Recommendation

| Situation | Choose |
|---|---|
| Strongest match to the instructor's research, and willing to handle more setup | **A: Failure monitoring** |
| Want the safest pass with low setup risk | **B** or **C** |

## Next steps (before 10.11.2026)
1. Pick one option as a team
2. Check that the code and checkpoints are public and run (Sentinel / Diffusion Policy, IntroPlan, RoCoBench)
3. Install ROS 2 on Ubuntu 26.04 and confirm the distro name
4. Write the 8 sections within **4 pages + 1 page of references**
