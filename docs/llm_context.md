Below is a **clean, complete context brief** you can copy-paste into an advanced LLM.
It contains **goal, constraints, environment, current state, and what is required** — no fluff.

---

## Project Context: YOLO-RescueSim

### High-Level Goal

Build a **mobile rescue robot simulation** using **TurtleBot3** in **ROS 2 + Gazebo (gz sim / Harmonic)** that:

* Runs in a **custom Gazebo world (`world.sdf`)**
* Uses a **camera** to detect **humans and rescue markers (e.g., colored flags)**
* Uses a **lightweight YOLO model (YOLO-Lite)**
* Is structured cleanly for **future real-robot deployment**

---

### Core Requirement (Current Focus)

Create a **custom ROS 2 launch file (`launch.py`)** that:

1. Launches **Gazebo** with a **custom `world.sdf`**
2. Spawns **TurtleBot3 (burger)** into that world
3. Keeps **ROS integration intact** (`/cmd_vel`, sensors, camera topics)
4. Does **not modify** any upstream TurtleBot3 launch files

---

### What is NOT allowed

* Launching `.sdf` directly with `ros2 launch`
* Editing TurtleBot3’s default launch files
* Using `gz sim world.sdf` alone (breaks ROS integration)
* Mixing ROS launch syntax with raw SDF loading

---

### Current Project Structure

```
YOLO-RescueSim/project/
├── world.sdf                  # Custom Gazebo world
├── turtle.sdf                 # TurtleBot model reference
├── load_turtle_in_world.py    # Early experiment (optional)
└── launch/
    └── rescue_sim.launch.py   # To be created (main goal)
```

---

### Environment Details

* OS: Ubuntu (ROS 2 supported)
* ROS 2: Humble
* Gazebo: Harmonic (`gz sim`)
* Robot: TurtleBot3 (burger)
* Language: Python (launch files, nodes)
* GPU optional (YOLO-Lite target)

---

### Known Working Command (Baseline)

This works and proves installation is correct:

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

### Why Custom Launch File Is Needed

* Default launch loads **default world**
* Project requires **custom rescue environment**
* YOLO pipeline needs predictable camera topics
* Future expansion: multi-robot, markers, autonomy

---

### What the Custom `launch.py` MUST Do (Technically)

* Include Gazebo launch (`gazebo.launch.py`)
* Pass `world:=/absolute/path/to/world.sdf`
* Spawn TurtleBot using `spawn_entity.py`
* Set `TURTLEBOT3_MODEL=burger`
* Ensure `/cmd_vel`, `/camera/image_raw`, `/scan` exist

---

### Key ROS–Gazebo Rule (Critical)

* `.sdf / .world` → Gazebo
* `.launch.py` → ROS
* ROS launch **controls Gazebo**, not the other way around

---

### Future Steps (After Launch Works)

* Subscribe to camera topic
* Run YOLO-Lite inference node
* Publish detection overlays / bounding boxes
* Integrate navigation + FSM logic
* Transition from sim → real TurtleBot

---

### What Help Is Requested from Advanced LLM

* Generate a **correct, minimal, ROS 2-compliant `launch.py`**
* Respect best practices (no hardcoded paths if possible)
* Explain **why each launch action exists**
* Ensure compatibility with Gazebo Harmonic + ROS 2 Humble

---

### Output Expectation

* One **working launch file**
* Short explanation per block
* No beginner-level ROS explanations
* No unrelated features

---

If you want, next I can:

* Validate the launch file the LLM gives you
* Or compress this into an even **tighter “LLM system prompt”** version
