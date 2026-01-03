# Commands in Project

This will include list of commands used in the project.

## 1. Load ROS:

```bash
source /opt/ros/jazzy/setup.bash && echo "ROS Jazzy Loaded!"
```

## 2. Launch TurtleBot3 in Gazebo.

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


## 3. Using the TurtleBot3 Camera

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

- **Echo camera messages (raw data):**
```bash
ros2 topic echo /camera/image_raw
```

The camera is ready for use by any ROS 2 node (e.g., YOLO perception) as soon as the simulation is running.

Keep in mind, it works in X11 better...


## 4. Troubleshooting: TurtleBot Moves Unexpectedly

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

