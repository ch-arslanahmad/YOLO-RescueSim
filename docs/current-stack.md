# Stack

Summary of what's actually installed (detected on this machine):

- Ubuntu 24
- ROS 2 Jazzy (ROS packages are under `/opt/ros/jazzy`)
- Gazebo / Gazebo-Sim (gz-sim8, gz-gui8, gz-sensors8, gz-plugin2, gz-transport13, etc.)
- TurtleBot3 support (TurtleBot3 metapackages and simulation)
- Python (system Python used by ROS/launch files)

Already installed (key packages/plugins discovered):

- `ros-jazzy-desktop` - full desktop install with GUI tools
- `ros-jazzy-turtlebot3` - TurtleBot3 metapackage
- `ros-jazzy-turtlebot3-bringup` - TurtleBot3 bringup package
- `ros-jazzy-turtlebot3-simulations` - TurtleBot3 simulation packages
- `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-image`, `ros-jazzy-ros-gz-interfaces` (ROS–Gazebo integration)
- System Gazebo/GZ packages such as `gz-sim8`, `gz-gui8`, `gz-sensors8`, `gz-plugin2`, `gz-transport13`, `gz-tools2`, and related `libgz-*` libraries.
- `ros-jazzy-navigation2`  - ROS 2 Navigation stack
- `ros-jazzy-nav2-bringup` - Navigation2 bringup package
- `ros-jazzy-slam-toolbox` - SLAM Toolbox for mapping and localization

sudo apt-get install -y ros-jazzy-gazebo-* ros-jazzy-turtlebot3* 2>&1 | grep -E "^(Setting up|Processing|E:|done)" | tail -20