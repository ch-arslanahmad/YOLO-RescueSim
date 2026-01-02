# Basics

# What is Gazebo?

Gazebo 




## Exercise 1 – System Integration

This exercise validates the full ROS2 simulation pipeline for YOLO-RescueSim, ensuring the robot, navigation, and perception systems are correctly integrated and actively performing basic actions:

- Launch Gazebo from ROS and load a simple test world
- Spawn a robot and verify it appears in the simulation
- Bring up Nav2 and command the robot to move to a goal location
- Activate the YOLO perception pipeline to process camera data and detect test objects

> [!NOTE]
> Objective: Verify that the simulation, robot control, navigation, and vision pipeline work together correctly, producing observable robot movement and perception output before full YOLO integration.

- Explained in [Exercise#1 - System Integration](exercises/exercise1.md)

## Learn Basics

### Run Gazebo with ROS2

```bash
source /opt/ros/jazzy/setup.bash # i have alias : load-ros
gz sim # launch Gazebo with ROS2 integration
```

Alternatively, launch Gazebo from ROS2 directly:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py world:=empty.sdf # launch empty world
```


> [!NOTE]
> You can create your own world using Gazebo GUI and save it as .sdf file
> In ROS 2 + Gazebo (ros_gz), worlds are usually .sdf files. There are two main ways to store them: