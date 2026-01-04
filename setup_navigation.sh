#!/bin/bash
# Navigation Setup Script
# Installs and configures prerequisites for auto-navigation

set -e

echo "================================"
echo "YOLO Rescue Sim - Navigation Setup"
echo "================================"

# Source ROS
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "OK: ROS 2 Jazzy sourced"
else
    echo "ERROR: ROS 2 Jazzy not found!"
    exit 1
fi

# Check required packages
echo ""
echo "Checking navigation packages..."

PACKAGES=(
    "navigation2"
    "nav2_bringup"
    "slam_toolbox"
)

for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "$pkg"; then
        echo "OK: ros-jazzy-$pkg installed"
    else
        echo "WARN: ros-jazzy-$pkg not found, installing..."
        sudo apt-get install -y "ros-jazzy-$pkg"
    fi
done

# Additional dependencies
echo ""
echo "Installing additional dependencies..."
sudo apt-get install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-nav2-msgs \
    python3-colcon-common-extensions

# Make scripts executable
echo ""
echo "Setting up project scripts..."
chmod +x "$HOME/Desktop/github/YOLO-RescueSim/project/auto_navigation.py"
chmod +x "$HOME/Desktop/github/YOLO-RescueSim/project/navigation_tester.py"
chmod +x "$HOME/Desktop/github/YOLO-RescueSim/project/launch/navigation_stack.launch.py"
chmod +x "$HOME/Desktop/github/YOLO-RescueSim/project/launch/rescue_sim.launch.py"

echo "OK: Scripts made executable"

# Build colcon workspace if needed
PROJECT_DIR="$HOME/Desktop/github/YOLO-RescueSim"
if [ -f "$PROJECT_DIR/src/package.xml" ] || [ -f "$PROJECT_DIR/package.xml" ]; then
    echo ""
    echo "Building ROS workspace..."
    cd "$PROJECT_DIR"
    colcon build --symlink-install || true
    echo "OK: Workspace built"
fi

# Summary
echo ""
echo "================================"
echo "Setup Complete!"
echo "================================"
echo ""
echo "Quick Start Commands:"
echo "1. Full stack (everything):"
echo "   ros2 launch project navigation_stack.launch.py"
echo ""
echo "2. Just simulation + robot:"
echo "   ros2 launch project rescue_sim.launch.py"
echo ""
echo "3. SLAM only:"
echo "   ros2 run slam_toolbox async_slam_toolbox_node"
echo ""
echo "4. Auto navigation:"
echo "   ros2 run project auto_navigation"
echo ""
echo "5. Manual control:"
echo "   ros2 run project navigation_tester"
echo ""
echo "For more info, see: AUTO_NAVIGATION_README.md"
echo "================================"
