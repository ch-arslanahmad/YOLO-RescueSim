#!/bin/bash
# Wrapper script to run teleop with proper environment

export TURTLEBOT3_MODEL=burger

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Source TurtleBot3 workspace if available
if [ -f "$HOME/turtlebot3_ws/install/setup.bash" ]; then
    source "$HOME/turtlebot3_ws/install/setup.bash"
fi

echo "==================================="
echo "TurtleBot3 Teleop Keyboard Control"
echo "==================================="
echo "Model: $TURTLEBOT3_MODEL"
echo ""
echo "Controls:"
echo "  W - Forward"
echo "  A - Turn Left"
echo "  D - Turn Right"
echo "  X - Backward"
echo "  S - Stop"
echo ""
echo "Press CTRL+C to exit"
echo "==================================="
echo ""

ros2 run turtlebot3_teleop teleop_keyboard
