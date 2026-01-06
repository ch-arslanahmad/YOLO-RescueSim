# Implementation Flow

## Project Journey (What Changed Over Time)

This section tracks how the project evolved from a basic sim into a usable end-to-end pipeline.

### Phase 1: Simulation Bring-Up

- Custom Gazebo world + TurtleBot3 spawn working via `rescue_sim.launch.py`.
- Camera topic validated: `/camera/image_raw`.

### Phase 2: Detection + Tracking Pipeline

- YOLO detector added and publishes:
   - `/yolo/detections` (JSON)
   - `/yolo/detection_image` (annotated image)
- Tracker added to aggregate detections and export results.

### Phase 3: Reliability Fixes ("Sometimes 1b works")

- Launch option `1b` hardened so it starts consistently:
   - Kills stale detector/tracker processes before re-launch (avoids singleton-lock blocking restarts).
   - Waits for `/camera/image_raw` publishers before starting YOLO.
   - Uses stable model paths so current-working-directory does not matter.

### Phase 4: Performance (CPU pegged at 100%)

- Detector made CPU-bounded:
   - Inference runs on a timer (no heavy work in the image callback).
   - Default `max_inference_hz` lowered.
   - Thread pools capped (Torch/OpenCV and common BLAS env vars).

### Phase 5: Rescue Semantics

- Detection overlay and JSON class renamed from "Human"/`person` to "Survivor"/`survivor`.
- Survivor count published as a lightweight topic: `/human_tracker/survivor_count`.
- Tracker associates detections with robot pose from `/odom` and includes `robot_pose` in history.

### Phase 6: Scripted Paths (Gazebo)

- Scripted path playback exposed as a launcher option (open-loop waypoint playback).

### Dev Log (Dated)

- 2026-01-05
   - Stabilized launch reliability (stale process cleanup + wait for camera publishers).
   - Reduced CPU load by bounding YOLO inference rate and limiting thread pools.
   - Switched UI semantics from “Human” to “Survivor” and added `/human_tracker/survivor_count`.
   - Added scripted waypoint playback for repeatable mission runs.
- 2026-01-04
   - Validated `/camera/image_raw` and created an end-to-end detection pipeline publishing `/yolo/detections` and `/yolo/detection_image`.
   - Established waypoint recording/playback workflow for repeatable path demos.

## Layman Terms

The flow for now is:

- [ ] Robot sees → Camera takes an image.
- [ ] Add Robot Movement across the field.
- [ ] Image goes to brain → A small YOLO model looks at the image.
- [ ] Brain decides → “This is a human” or “This is a rescue flag”.
- [ ] Robot reacts → Marks location, logs it, or moves.

## Technical Steps

- [x] Launch Gazebo with TurtleBot3 in a custom rescue world.
- [x] Verify it with camera
- [x] See via the camera
- [ ] Take image for YOLO lite 
1. **Camera Node**: Captures images and publishes them to a ROS topic.


## Currently Completed Steps

- Created a custom ROS 2 launch file (`rescue_sim.launch.py`) that:
  - Launches Gazebo with a custom `world.sdf`.
  - Spawns TurtleBot3 (burger) into that world.
  - Maintains ROS integration (`/cmd_vel`, sensors, camera topics).
- Verified camera and sensor topics are available in ROS 2 using `ros2 topic list`.
- Used the TurtleBot3 camera to confirm image feed is accessible via `/camera/image_raw` topic.
- Basic Robot Movement to navigate the whole world easily.

### Integration Points:

- [x] Navigation of Robot across the field to cover the entire area systematically.

**Completed:**
- SLAM Toolbox integrated for real-time mapping
- Navigation2 stack configured for autonomous movement
- Frontier-based exploration algorithm implemented in `auto_navigation.py`
- Auto-generated goals based on frontiers (unexplored boundaries)
- Goal tracking to avoid revisiting locations

**Created Components:**
1. **auto_navigation.py** - Main autonomous navigation node with:
   - Frontier detection algorithm
   - Goal planning and navigation
   - SLAM integration
   - Obstacle avoidance via laser scan
   - Goal history tracking

2. **navigation_tester.py** - Interactive testing tool for manual control

3. **nav_config.py** - Centralized configuration for all navigation parameters

4. **navigation_stack.launch.py** - Complete launch file that starts:
   - Gazebo with custom world
   - TurtleBot3 spawning
   - SLAM Toolbox
   - Navigation2 stack
   - Auto Navigation node

5. **AUTO_NAVIGATION_README.md** - Comprehensive documentation

6. **setup_navigation.sh** - Automated setup script
