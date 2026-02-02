# Quick Setup Guide - YOLO-RescueSim

## Current Setup Issue - Missing Dependencies

You're seeing errors because required ROS 2 packages are not installed on your system.

## One-Command Fix

Run the dependency installer:

```bash
./install_dependencies.sh
```

This will install:
- ✅ TurtleBot3 packages (ros-jazzy-turtlebot3-*)
- ✅ Gazebo-ROS bridge packages  
- ✅ Navigation2 and SLAM packages
- ✅ Visualization tools (image_view, rqt_image_view)
- ✅ Python virtual environment with YOLO dependencies

## After Installation

1. **Build the project**:
   ```bash
   ./build_project.sh
   ```

2. **Launch the simulation**:
   ```bash
   ./launch_sim.sh
   ```
   Then select option `1` for basic sim or `1b` for sim + YOLO detection.

## What Was Wrong?

The errors you encountered:
- ❌ `Package 'turtlebot3_gazebo' not found` - Missing TurtleBot3 simulation package
- ❌ `Package 'turtlebot3_teleop' not found` - Missing teleop control package  
- ❌ `Package 'image_view' not found` - Missing camera viewer
- ❌ `WARN: .venv not found` - Missing Python virtual environment for YOLO
- ❌ Hardcoded paths to `/home/arslan/...` - **Already fixed!** Scripts now use dynamic paths.

## Manual Installation (Alternative)

If you prefer to install manually:

```bash
# Update package lists
sudo apt update

# Install TurtleBot3
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-simulations \
    ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-teleop

# Install Gazebo bridge
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image

# Install navigation
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox

# Install viewers
sudo apt install -y ros-jazzy-image-view ros-jazzy-rqt-image-view ros-jazzy-rviz2

# Create Python venv for YOLO
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-yolo-venv.txt
deactivate
```

## Detailed Documentation

For complete setup instructions, see:
- [docs/setup.md](docs/setup.md) - Full installation guide
- [docs/launch.md](docs/launch.md) - Launch script usage
- [docs/yolo.md](docs/yolo.md) - YOLO environment setup

## Troubleshooting

**Q: Script still shows path errors?**  
A: Make sure you're in the project directory when running scripts.

**Q: YOLO detection fails?**  
A: Activate the virtual environment first:
```bash
source .venv/bin/activate
```

**Q: Gazebo won't start?**  
A: Kill stale processes:
```bash
pkill -f "gz sim"
pkill -f "ros2 launch"
```

## Support

For issues, check the docs folder or review error messages carefully. Most errors indicate missing packages that can be installed via apt.
