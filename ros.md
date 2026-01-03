# ROS 

## Installation Guide

The latest version is ROS2.

### Step 1 - Add the ROS2 Repo
```bash

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

```

### Step 2 - Install ROS2


1. with GUI support
```bash
sudo apt update
sudo apt install ros-jazzy-desktop
```

2. Base (minimal, CLI only):

```bash
sudo apt install ros-jazzy-ros-base

```

> [!NOTE]
> To install some development tools, you can run:
> ```bash
> sudo apt install ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py
>```


## Installing Relevent Packages


### Step 1 - Add Gazebo-ROS Packages

```bash
sudo apt update
sudo apt install curl gnupg lsb-release

sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/gazebo-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
  https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update

```

There are commands for it to work alongside Gazebo-ROS, plugins etc, explained [here](commands.md)


## Using the Built-in Camera in TurtleBot3

The official TurtleBot3 models (Burger, Waffle) provided by the `turtlebot3_gazebo` package already include a simulated camera sensor in their URDF/SDF definitions. This means:

- When you spawn the TurtleBot3 model in Gazebo using standard ROS 2 launch files, the camera is automatically present and simulated.
- The camera publishes image data to the `/camera/image_raw` topic in ROS 2, which can be used directly by your perception nodes (e.g., YOLO-Lite).
- No extra configuration or model editing is needed unless you want a custom camera setup (different position, FOV, etc.).
- This ensures full ROS 2–Gazebo integration and saves development time.

**In summary:** By using the built-in camera, you get a working, simulated camera on the robot “for free” with the standard TurtleBot3 model, ready for your computer vision and perception tasks.

