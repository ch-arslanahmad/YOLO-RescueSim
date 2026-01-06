# Commands in Project

This will include list of commands used in the project.

## 1. Load ROS:

```bash
source /opt/ros/jazzy/setup.bash && echo "ROS Jazzy Loaded!"
```

## 2. Exploring ROS Packages & Gazebo Resources

### List all ROS installed packages:

```bash
ros2 pkg list
```

### Find specific ROS packages:
```bash
ros2 pkg list | grep <search_term>
```

### Get package information:
```bash
ros2 pkg prefix <package_name>  # show install location
ros2 pkg executables <package_name>  # list executables in package
```

### List Gazebo plugins:
```bash
gz plugin --list
```

### Check Gazebo/ROS environment variables:
```bash
printenv | grep ROS
printenv | grep GZ
env | grep -E '(GZ_SIM|GAZEBO)'
```

### Show Gazebo resource paths:
```bash
echo $GZ_SIM_RESOURCE_PATH
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
```

### List installed Gazebo models:
```bash
# System models
ls /usr/share/gz/gz-sim*/models 2>/dev/null || echo "Path may vary"
# User models
ls ~/.gz/models 2>/dev/null || echo "No user models found"
```

### List all active ROS nodes:
```bash
ros2 node list
```

### List all active topics:
```bash
ros2 topic list
```

### List all services:
```bash
ros2 service list
```

### List installed ROS/Gazebo packages (via apt):
```bash
apt list --installed | grep ros-jazzy-
apt list --installed | grep gz-
# or use dpkg
dpkg -l | grep ros-jazzy
dpkg -l | grep gz-
```

### Search for available (not yet installed) packages:
```bash
apt search ros-jazzy-
apt search gz-
```


## 3. Launch TurtleBot3 in Gazebo.

The following opens the default world.

```bash
export TURTLEBOT3_MODEL=burger # or add in ~/.bashrc
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Alternatively, to load a custom worlld with turtlebot3, you need to make a launch file as follows, then go to the directory and run the launch file.

```bash

# it is recommended you name the launch file as <name>.launch.py
ros2 launch ./launch/<launch_file_name>.launch.py


ros2 launch ./launch/rescue_sim.launch.py

```

### Move TurtleBot3 Keyboard

You can move TurtleBot3 via keyboard using the following command,

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```


### See all active Topics of ROS

```bash
ros2 topic list
```


## 4. Using the TurtleBot3 Camera

The camera is enabled by default in the TurtleBot3 simulation. To view or use the camera feed:

- **List camera topics:**
```bash
ros2 topic list | grep camera
```

- **View camera info:**
```bash
ros2 topic info /camera/image_raw
```

- **Visualize camera feed (GUI):**
```bash
ros2 run rqt_image_view rqt_image_view
```
	Then select `/camera/image_raw` in the GUI.

- **Visualize YOLO detections on the camera feed (bounding boxes):**
  - Raw camera is `/camera/image_raw` (no boxes)
  - Annotated detections are published on `/yolo/detection_image`

```bash
ros2 run image_view image_view --ros-args -r image:=/yolo/detection_image
```
	Or in `rqt_image_view`, select `/yolo/detection_image`.

- **Echo camera messages (raw data):**
```bash
ros2 topic echo /camera/image_raw
```

The camera is ready for use by any ROS 2 node (e.g., YOLO perception) as soon as the simulation is running.

Keep in mind, it works in X11 better...


## 5. Troubleshooting: TurtleBot Moves Unexpectedly

If your TurtleBot starts moving on its own in simulation, follow these steps:

- **Check who is publishing to /cmd_vel:**
```bash
ros2 topic info /cmd_vel
```
	If Publisher count > 0, some node is sending velocity commands.

- **List all running nodes:**
```bash
ros2 node list
```
	Look for teleop_keyboard or any custom control node.

- **Stop unwanted publishers:**
	Close or kill any node that is publishing to /cmd_vel (e.g., teleop_keyboard).

- **Emergency stop:**
	You can immediately stop the robot by sending a zero velocity command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

If Publisher count is 0 and the robot still moves, it is likely simulation inertia and will settle quickly. Always ensure only the intended node is controlling the robot.


### Quick Emergency Stop Alias/Function

To make stopping the robot even faster, add this alias or function to your `~/.bashrc`:

- **Alias:**
	```bash
	alias turtlebot_stop="ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'"
	```

- **Shell function (for more flexibility):**
	```bash
	turtlebot_stop() {
		ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
	}
	```

After adding, reload your shell or run `source ~/.bashrc`, then use `turtlebot_stop` anytime to instantly stop the robot.

