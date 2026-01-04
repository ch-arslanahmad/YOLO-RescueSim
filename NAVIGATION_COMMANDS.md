# TurtleBot3 Navigation - Quick Commands

## Setup & Build

```bash
cd /home/arslan/Desktop/github/YOLO-RescueSim
source /opt/ros/jazzy/setup.bash
colcon build --packages-select project
source install/setup.bash
```

---

## Running the 3-Terminal Workflow

### Terminal 1 - Launch Gazebo with TurtleBot3

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2 - Start Real-Time Keyboard Controller

```bash
source /opt/ros/jazzy/setup.bash
cd /home/arslan/Desktop/github/YOLO-RescueSim
source install/setup.bash
python3 project/navigation_scripts/calibration/keyboard_controller_simple.py
```

**Keyboard Controls:**
- **W** - Move Forward
- **S** - Move Backward
- **A** - Turn Left
- **D** - Turn Right
- **+/-** - Speed Up / Down
- **Q** - Quit

### Terminal 3 - Monitor Robot Coordinates & Record Waypoints

```bash
source /opt/ros/jazzy/setup.bash
cd /home/arslan/Desktop/github/YOLO-RescueSim
source install/setup.bash
python3 project/navigation_scripts/calibration/coordinate_tracker.py
```

**Controls:**
- **SPACE** - Save current waypoint
- **U** - Undo last waypoint
- **P** - Pause/Resume tracking
- **L** - List all saved waypoints
- **X** - Exit and save waypoints to CSV file

---

## After Recording Waypoints

### Terminal 4 - Run Automated Waypoint Navigation

```bash
source /opt/ros/jazzy/setup.bash
cd /home/arslan/Desktop/github/YOLO-RescueSim
source install/setup.bash
python3 project/navigation_scripts/navigation/waypoint_navigator.py
```

The robot will automatically navigate through all recorded waypoints sequentially.

---

## Debugging & Monitoring

### Check if /cmd_vel topic exists
```bash
ros2 topic list | grep cmd_vel
```

### Echo velocity commands being sent
```bash
ros2 topic echo /cmd_vel
```

### Echo robot position (odometry)
```bash
ros2 topic echo /odom
```

### List all active ROS nodes
```bash
ros2 node list
```

### List all active topics
```bash
ros2 topic list
```

---

## Waypoint File Location

Saved waypoints are stored in:
```
project/navigation_scripts/waypoints/waypoints_YYYY-MM-DD_HHMMSS.csv
```

Format: `x, y, theta (in radians)`

You can manually edit this CSV file to adjust waypoints if needed.

---

## Emergency Stop

If the robot is moving unexpectedly, press **Ctrl+C** in the keyboard controller terminal or any Python script.

To remotely stop via command line:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```
