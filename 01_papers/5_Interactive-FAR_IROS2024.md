# Paper 5 – Interactive-FAR (IROS 2024)

> B. He, G. Chen, W. Wang, J. Zhang, C. Fermüller, Y. Aloimonos, "Interactive-FAR: Interactive, Fast and Adaptable Routing for Navigation Among Movable Obstacles in Complex Unknown Environments," *IROS 2024*, Abu Dhabi, pp. 5402–5409. doi:10.1109/IROS58592.2024.10802169
> University of Maryland + Carnegie Mellon University (CMU). Code: github.com/Bottle101/Interactive-FAR (Docker demo). ⚠️ ROS version not stated in the paper. Its base system (FAR planner, CMU exploration environment) is mainly ROS 1, so check the repo before relying on it.

## 1. The problem in plain words

Normal navigation treats **every obstacle as a fixed wall**. In real buildings, many obstacles can be moved: boxes, chairs, carts, trash bins. If a box blocks a corridor, a normal robot takes a long detour, or gives up if no detour exists.

**Interactive-FAR's idea:** "If a light object blocks the way, **push it out of the way**. If it turns out to be too heavy, **notice that from the force sensor**, update the map, and **re-plan** another route."

The task is called **NAMO** (Navigation Among Movable Obstacles). It is known to be **NP-hard** (very expensive to solve exactly), so the paper's focus is to make it **fast enough to run in real time** in large, unknown environments.

## 2. Background you need

| Concept | Simple explanation |
|---|---|
| **Visibility graph (V-graph)** | A map that stores only obstacle **corners** (polygon vertices) as nodes. Two corners are connected if a straight line between them is free. The shortest path in a polygon world always goes corner to corner, so this small graph is enough. It is much smaller than a grid, so search is very fast |
| **FAR planner** | "Fast, Attemptable Route planner" (IROS 2022), by the same CMU group. It builds a V-graph online from LiDAR in unknown environments. Interactive-FAR extends it |
| **Directed V-graph (DV-graph)** | A V-graph whose edges have a **direction**. Pushing a box from room A into room B is not the same action as from B to A, so the two directions get different edges and costs |
| **Polygon extraction** | Turning the point cloud into simple polygons: draw obstacles on a 2D image, trace their borders (Suzuki border following), then simplify the outlines (Douglas–Peucker) |
| **Inflation** | Growing obstacles by the robot's size so the robot can be treated as a point |
| **Topology / connected components** | Regions of free space you can reach from each other. A movable box can cut one region into 2+ **disconnected components** |
| **Affordance** | What an object "allows" you to do with it. Here it means **how pushable it is** (weight, friction) |
| **Stable pushing** (Lynch & Mason 1996) | Pushing an object so that it **does not slip or rotate** relative to the pusher. It moves as if glued to the robot. That makes it predictable |
| **SE(2)** | A 2D pose: (x, y, heading) |
| **Hybrid A\*** | A* search where each node is a continuous pose (x, y, θ), expanded with feasible motions (motion primitives). The result can actually be driven by a car-like or diff-drive robot |
| **Motion primitive** | A short, precomputed piece of motion (e.g., "drive 0.3 m while turning left") |
| **Admissible heuristic** | An estimate of the remaining cost that **never overestimates** it. With it, A* finds the optimal path |
| **Kinodynamically feasible** | Respects the robot's motion limits (can't move sideways, limited turning) |
| **SPL** | *Success weighted by Path Length* (Anderson et al. 2018). Per task: S · L_opt / max(L, L_opt), averaged. 1.0 means always succeeding with the shortest path |
| **LP1 NAMO** | A simpler NAMO class: each action moves one object, and moving it does not create a new blockage |

## 3. Method, step by step

**Pipeline (Fig. 2):** sensor data (point cloud + force) → polygons → find where movable objects disconnect space → plan pushes for each disconnection → build the local DV-graph → merge it into the global DV-graph → search the global path → execute with a normal or an "interactive" (pushing) local planner → update the object's affordance from the force sensor → re-plan if needed.

### 3.1 Two problems (Sec. III)
- **Problem 1 (routing):** on the DV-graph G, find the most **energy-efficient** path from robot to goal. Energy = travel distance **plus** the effort of pushing objects. It is solved again every planning cycle as new data arrives.
- **Problem 2 (manipulation):** a movable object splits a local area into n components. Find **n(n−1)** push strategies, one for each ordered pair (from component i to j). Solved **once per object**, then stored.
- **Important input assumption:** the point cloud arrives **already split** into movable points S_mov and static points S_bg. The paper does **not** say how the robot knows which objects are movable (in simulation, they are given).

### 3.2 Building the DV-graph (Sec. V)
1. **Polygon extraction (V-A):** inflate S_mov and S_bg by the robot size, draw them on 2D images, and extract simplified polygons. Static = red, movable = green (Fig. 1).
2. **Connectivity analysis (V-B):** static polygons alone leave free space connected. Adding a movable polygon can cut it into components C¹…Cⁿ. For each component, pick a **topological waypoint** p_topo (Eq. 1): start at the midpoint of the boundary where the object cuts the region, step along the normal by min(d_thres, d_obs/2), with d_thres = 0.5 m. Theorem 1 says this point sees at least 3 vertices of its component, so it can connect to the graph.
3. **Interaction planning (V-C):** for each pair (p_topo^i, p_topo^j), the function Γ returns a push strategy π and its **cost J**, given the current affordance guess µ. Each strategy becomes a **directed edge** with cost J (Algorithm 1, lines 4–8).
4. **Local → global merge (V-D):** match local vertices to global ones. Matched vertices are updated, new ones are added. A vertex missing from the current scan is **not deleted at once**: it is removed only after it stays unseen for several frames (**voting**), which protects against sensor noise.

### 3.3 Generating a push strategy (Sec. VI)
- **Decomposition (VI-A):** the n(n−1) strategies are searched separately (Problem 3: one start, one goal). One big search with many goals would be too hard.
- **State model (VI-B):** the robot is differential drive and only **pushes**. The object's shape is guessed from its polygon, and friction starts at an initial guess k (uniform, so the friction center is the geometric center). Only controls that give a **stable push** are allowed. Because the robot is "attached" to the object at a contact point, the state reduces to (object pose x = [p_x, p_y, ψ], contact point c). The robot may **switch contact points** (push from another side) (Fig. 5).
- **Search (VI-C):** Hybrid A* over these states. The **heuristic** comes from the "heuristic polygon" (the movable part blocking the passage): the **polygon width**, i.e., the minimum distance the object must move to free the way.
- **Guarantees (VI-D):** the heuristic is admissible when the heuristic polygon is **convex** (Lemma 1, Theorem 2). **Completeness is not guaranteed** (the state space is discretized).

### 3.4 Execution and adaptation (Sec. VII) ⭐ the part relevant to failures
- **Global search (VII-A):** add robot and goal as vertices and search G (the paper says breadth-first search). The graph can be saved and reused later as a prior map. Example: a 455 m path over 1,570 vertices in **3 ms**.
- **Execution (Algorithm 2):**
  - normal path segment → **collision-free planner** (CMU's local planner)
  - near a topological waypoint (a push is planned) → **interactive planner**: execute the push while checking push stability, collisions and affordance µ
  - while pushing, re-estimate the cost Ĵ, strategy π̂ and affordance µ̂ from the **1-D force sensor**
  - if ‖Ĵ − J‖ > τ_thres → store the new values, **update the related graph edges**, and **re-plan the whole global path**
  - otherwise → keep pushing
- **Tactile rules:**
  1. if the object doesn't move after **maximum push effort** → mark it **not pushable**
  2. if the cost estimate changes beyond the threshold → update it and re-plan
- After the push, the learned J and π are **kept in the DV-graph**, so the knowledge is reused later.

## 4. Experiments & results (Sec. VIII)

- **Setup:** simulation only. Ground vehicle with a Velodyne Puck 3D LiDAR + 1-D force sensor. i7-12700H laptop. Graph update and path search at 2.5 Hz, 0.15 m resolution, 60 × 60 m local layer.
- **Environments (Fig. 6):** Room (32 × 32 m, with and without objects), Office, Tunnel (330 × 270 m). **10 navigation tasks** in total.
- **Table I – search time:** Interactive-FAR ≈ FAR (0.3 / 0.4 / 1.32 ms). A*, RRT* and BIT* are much slower in big maps (e.g., 363 ms for A* in Tunnel). The NAMO baselines R-NAMO and LAMB (Python) take seconds and **fail at Office/Tunnel scale**.
- **Table II – SPL:** Room with objects 0.70 vs 0.58–0.61 (NAMO methods). Office **0.86** vs ≤ 0.43. Tunnel **0.97** vs ≤ 0.81.
- **Table III – full system vs FAR:** Office 109.7 m / 243 s / **5/5** vs 232.6 m / 360 s / 4/5. Tunnel 1,121 m / 1,487 s / 3/3 vs 1,490 m / 1,911 s / 3/3. This is the source of the "33% less travel time" claim.

## 5. Strengths

- ✅ Very fast: millisecond re-planning at large scale (sparse polygon graph + C++)
- ✅ **Closed loop with feedback:** it tries, measures, updates its belief and re-plans, instead of trusting a fixed model
- ✅ What it learns (object costs) is **stored in the map and reused**
- ✅ Kinodynamically feasible push plans, with an admissibility proof (for convex cases)
- ✅ Open-source Docker demo

## 6. Limitations (critical analysis)

| # | Limitation | Why it matters |
|---|---|---|
| 1 | **Movable vs static is given as input** (S_mov is pre-segmented) | The hardest perception step is skipped. In reality, misclassifying a wall as "movable" causes a failed push and a wasted attempt |
| 2 | **Failure detection of a push is crude:** "no motion after max effort" + a fixed threshold τ_thres on the cost change | No uncertainty, no calibration, no statistics on false/missed detections. The value of τ_thres is not reported |
| 3 | **Tiny evaluation:** 10 tasks, success counts 4/5, 5/5, 3/3. No repetitions, no confidence intervals | The difference 4/5 vs 5/5 is one episode. That is not statistically meaningful |
| 4 | **Simulation only**, ideal 1-D force sensor | Real contact forces are noisy. Friction is assumed uniform |
| 5 | Polygon approximation of objects (authors admit it, Sec. IX) | Contact point and friction errors make pushes less robust |
| 6 | LP1 assumption: one object per region, pushing creates no new blockage | Cluttered warehouses break this |
| 7 | Learned affordances are **kept forever** in the graph | Same issue as Air Bumper: if the object changes (a cart is unloaded), the stored cost is wrong. There's no forgetting |
| 8 | System-level baseline is only FAR (which can't push at all) | Shows "pushing helps", not that this pushing method is the best |
| 9 | Not completeness-guaranteed. Admissibility only for convex heuristic polygons | No guarantee of finding a push plan when one exists |

## 7. What we can implement / take from this paper

| Idea | How we use it | Status |
|---|---|---|
| ⭐ **"Try → sense → update cost → re-plan" loop** | Same pattern as our C2 supervisor: when a failure is predicted/observed, update the costmap (failure memory) and re-plan instead of retrying blindly | ✅ Supporting reference for C2 |
| ⭐ **Store what was learned in the map and reuse it** | Our failure-memory mask stores *where* failures happened. Interactive-FAR stores *how costly* an object is. Same idea at a different level | ✅ Supports C2 |
| **Voting before deleting** a vertex (remove only after several frames unseen) | Justifies not reacting to one noisy frame. It matches our **hysteresis** and the decay of failure marks | ✅ Inspiration |
| Fix limitation 7 (permanent stored costs) | Our **decaying** memory: the same improvement we make over Air Bumper | ✅ Shows the gap is common across papers |
| Fix limitation 2 (fixed threshold τ_thres) | Our **conformal-calibrated** threshold is the general answer to "how do you set the trigger?" | ✅ Strengthens our motivation (L3) |
| **SPL metric** | Extra closed-loop metric next to the BARN score: it penalizes long detours caused by recoveries | ⚠️ Optional, cheap to add |
| Visibility-graph global planner (FAR) | ❌ Not used. We keep the Nav2 planners, so we don't change the planner under test | ❌ |
| Pushing movable obstacles, Hybrid A* push search, force sensing | ❌ Out of scope: it needs contact physics, a force sensor and manipulation. Too big for our team and timeline | ❌ (possible "future work" sentence) |

**Is it a good primary paper for us?** ❌ No. It's a solid IROS 2024 paper (approved venue ✅), but it is about **planning with movable obstacles**, not failure detection. Its failure handling is a small, rule-based part. Switching to it would mean building push manipulation in Gazebo, which is far bigger than our scope. It works best as a **secondary paper** that supports C2 and our threshold criticism.

**How we cite it (optional):** "Interactive-FAR [5] also adapts online by updating edge costs from tactile feedback and re-planning, but uses a fixed threshold and keeps learned costs permanently; our calibrated trigger and decaying memory address both."

⚠️ Open item: check the citation count on Google Scholar, like for the other papers.
