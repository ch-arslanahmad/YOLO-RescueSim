# Setup

This doc is the canonical setup guide for YOLO-RescueSim on Ubuntu + ROS 2 Jazzy + Gazebo Harmonic.

## Prerequisites

- Ubuntu 22.04/24.04
- ROS 2 Jazzy
- Gazebo Harmonic (`gz-harmonic`)

## Install ROS 2 Jazzy

Add the ROS apt repository:

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

Install ROS:

```bash
sudo apt install -y ros-jazzy-desktop
```

## Install Gazebo Harmonic

Add the Gazebo (OSRF) apt repository:

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
```

Install Gazebo Harmonic:

```bash
sudo apt install -y gz-harmonic
```

Quick sanity check:

```bash
gz sim
```

## Install TurtleBot3 + ROS/Gazebo bridge packages

Install the core simulation packages used by this repo:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image
```

Optional viewer:

```bash
sudo apt install -y ros-jazzy-image-view
```

## Build this repo

From the repo root:

```bash
source /opt/ros/jazzy/setup.bash
./build_project.sh
source install/setup.bash
```

## Run the simulation

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./launch_sim.sh
```

## Next

- YOLO Python env setup: see [yolo.md](yolo.md)
- Common commands: see [commands.md](commands.md)
