# Commands in Project

This will include list of commands used in the project.

## 1. Load ROS:

```bash
source /opt/ros/jazzy/setup.bash && echo "ROS Jazzy Loaded!"
```
```
```


## 2. Launch TurtleBot3 in Gazebo:


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

