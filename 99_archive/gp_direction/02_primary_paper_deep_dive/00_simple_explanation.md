# 00 – The Project in Simple Words (start here)

> For team members who don't know the paper yet. No background needed: every term is explained when it first appears. When you want the details, continue with [`01_background_knowledge.md`](01_background_knowledge.md) → [`02_paper_walkthrough.md`](02_paper_walkthrough.md) → [`03_limitations_and_open_questions.md`](03_limitations_and_open_questions.md), then [`../03_our_project/`](../03_our_project/README.md).

**Primary paper:** Mohammad, Higgins, Bezzo, *"A GP-based Robust Motion Planning Framework for Agile Autonomous Robot Navigation and Recovery in Unknown Environments,"* ICRA 2024.

---

## 1. What the robot in the paper does

A small wheeled robot has to drive to a goal through a place it has never seen. It sees only with a **laser scanner (LiDAR)**, which measures distances to walls around it.

Every fraction of a second, its **planner** (the planning software) does two steps:

**Step 1 – find a rough route and the "safe boxes".** It draws a rough line from the robot toward the goal. Then it covers that line with a chain of **boxes of empty space**: areas where the laser shows no obstacles.

```
   wall ███████████████████████
        ┌──────┐┌──────┐┌────┐
 robot ●│ box1 ││ box2 ││box3│ → toward goal
        └──────┘└──────┘└────┘
   wall ███████████████████████
```

This chain of safe boxes is called the **corridor**, written **C**. Think of it as the stepping stones the robot is allowed to walk on.

**Step 2 – draw the actual smooth path.** A math solver (an **optimizer**) tries to draw a smooth, drivable path that stays **inside the boxes** and respects the robot's speed and acceleration limits.

## 2. What "failure" means in the paper

Sometimes Step 2 **cannot find any answer**. For example:
- the boxes are too small or too few (the robot is in a tight spot), or
- the robot is driving fast toward the edge of the boxes and can't brake in time inside them.

Then the planner returns "no solution". The robot has no new path, so it may stop suddenly, get stuck, or crash. **That moment is the failure the paper wants to predict.**

## 3. The two numbers: |C| and t_C

The paper describes the robot's situation with just **two numbers**:

| Symbol | Plain meaning | Intuition |
|---|---|---|
| **\|C\|** | **How many safe boxes** are in the corridor right now | Many boxes = lots of known free space ahead ✅. Few boxes = cramped or unknown area ⚠️ |
| **t_C** | **How many seconds until the robot would hit the edge of its safe boxes** if it keeps its current speed and direction | Large t_C = plenty of time ✅. Small t_C = about to leave the safe area ⚠️ |

**Example:** the robot drives at 1 m/s and the edge of the current box is 0.5 m ahead. Then t_C = 0.5 / 1 = **0.5 seconds**, which is dangerous.

So the danger rule is: **few boxes + little time = the planner will probably fail soon.**

## 4. How the paper predicts failure

1. **Learning.** The authors drove the robot many times in a simulator. For each moment they recorded the two numbers (t_C, |C|) and whether the planner failed shortly after. They trained a model called a **Gaussian Process (GP)**. Treat it as a smart curve-fitting tool: give it the two numbers and it returns a **chance of failure** (e.g., 0.8 = 80%). It also says **how sure** it is about that guess, but the paper **ignores** that part.
2. **Looking ahead.** The robot's controller already predicts where the robot will be over the next moments (e.g., 15 future positions). For each future position the robot computes t_C and |C|, asks the GP for the chance of failure, and **adds up** all the chances. The total is the **risk**.
3. **Alarm.** If risk > **3** (a number the authors picked by hand) → "danger, failure is coming".
4. **Recovery.** The robot picks many random points in the free space nearby and asks the GP: "if I stood there, how likely is failure?" It drives to the point with the **lowest** chance, then starts planning to the goal again.

**Result in the paper:** in 120 simulated runs, the robot succeeded more often than without this system.

## 5. The problems we found (this is what we solve)

| # | Problem | In simple words |
|---|---|---|
| **1** | ❌ **Works only with this one planner** | t_C and \|C\| exist only for planners that build "safe boxes". The standard ROS 2 navigation software, **Nav2**, and its common path-following programs (**DWB**, **MPPI**) don't build boxes. So the method **can't be used with what industry actually uses** |
| **2** | ❌ **Ignores how sure the model is** | The GP says "80% chance of failure" *and* "I'm unsure about this". In a situation never seen in training the guess may be badly wrong, and the paper still trusts it fully |
| **3** | ❌ **The alarm level "3" is a guess** | Nobody knows what "risk > 3" means in practice: how many false alarms per minute, how many missed failures. Also, because the risk is a *sum*, it changes if you look 10 vs 20 steps ahead, so "3" means something different each time |
| **4** | ❌ **Recovery is naive** | (a) It scores each spot as if the robot were **standing still**. At zero speed, t_C is basically the same everywhere, so half the information disappears. (b) It doesn't care whether the spot is **closer to or farther from the goal**, so it may send the robot backward. (c) It has **no memory**: it can fail at the same spot again and again |
| **5** | ⚠️ **Tested only by success rate** (120 runs) | It never measured how early the alarm comes, how often it fires falsely, or how often it misses a failure. For failure detection, those are the most important numbers |

## 6. What we do: two contributions

### Contribution 1 (C1) – a better failure predictor

| Paper | Ours | Why it's better |
|---|---|---|
| Uses t_C and \|C\| from the safe boxes | Uses **5 numbers computed directly from the laser scan and the robot's motion**: (1) seconds until hitting the nearest obstacle, (2) distance to the nearest obstacle, (3) how twisty the route is, (4) current speed, (5) how much of the area ahead is still unknown | Every robot with a laser has these, so it works with **any** planner, including Nav2 |
| Uses only the model's guess | Uses **guess + uncertainty**: if the model is unsure, we treat the situation as **more** dangerous | Safer in new, unseen situations |
| Adds up the chances (sum) | Takes the **average** chance over the future positions | Always stays between 0 and 1, whatever the look-ahead length |
| Alarm level 3, chosen by hand | Alarm level **computed from data** with a statistical method called **conformal calibration** | We can say in advance "at most 5% false alarms", then check that it's true |
| — | **Two alarm levels:** turn ON at a high level, OFF at a lower one (**hysteresis**) | Stops the alarm from flickering on/off every moment |

**How the data-based alarm level works:**
1. Run the robot in some practice worlds.
2. Collect the risk values from moments when **nothing went wrong**.
3. Put the alarm level just above 95% of them.
4. Then in new worlds, only about 5% of safe moments will wrongly trigger the alarm.

### Contribution 2 (C2) – smarter recovery

When the alarm fires, we again pick candidate spots nearby, but each spot gets a **score built from 4 parts**. The **lowest score wins**:

1. **Failure chance at that spot**, computed assuming the robot is **moving** (at half its top speed), not standing still. That keeps the time-based information meaningful.
2. **Distance to the goal from that spot**, so we prefer spots that make progress.
3. **Memory of past failures:** places where the robot recently failed get a penalty. The memory **fades after about a minute**. A real problem spot keeps causing failures, so it stays marked. A temporary one (e.g., a person walking by) gets forgotten. (This is the Air Bumper paper's idea, with forgetting added.)
4. **Distance from the robot**, so we don't send it on long trips just to recover.

**Safety limit:** if it fails to recover 3 times, it falls back to Nav2's normal recovery behaviors.

## 7. How we prove it

- **Setup:** ROS 2 + Nav2 + the Gazebo simulator, the same Jackal robot as the paper.
- **Worlds:** about 300 cluttered test worlds (BARN benchmark style), rooms with hidden dead ends (like the paper), and some harder new worlds.
- **We compare:**
  - doing nothing special (Nav2 default)
  - a simple rule ("alarm if hitting an obstacle within X seconds")
  - **the paper's method, rebuilt by us**
  - simpler learning models instead of the GP
  - our method
- **We measure:**
  - how many failures we predict, and how early
  - how many false alarms
  - whether "5% false alarms" really comes out near 5%
  - in full runs: success rate, crashes, time, and how often the robot fails at the same spot again

## 8. One-sentence summary

> The paper predicts when a robot's planner is about to fail, but only for one special planner, with a hand-picked alarm level and a simple recovery. **We make the predictor work with any planner, give its alarm a guaranteed false-alarm rate, and make the recovery move toward the goal and remember recent failure spots.**
