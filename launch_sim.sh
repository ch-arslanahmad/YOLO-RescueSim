#!/bin/bash
# Quick Launch Script for YOLO Rescue Simulation
# Starts the complete simulation with Gazebo, RViz, and Navigation

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}🤖 YOLO-RescueSim Launcher${NC}"
echo -e "${GREEN}================================${NC}\n"

# Check if ROS 2 is available
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo -e "${RED}✗ ROS 2 Jazzy not found!${NC}"
    exit 1
fi

# Source ROS 2
source /opt/ros/jazzy/setup.bash
echo -e "${GREEN}✓ ROS 2 Jazzy sourced${NC}"

# Navigate to project directory
cd /home/arslan/Desktop/github/YOLO-RescueSim

# Source the local setup
if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo -e "${GREEN}✓ Project setup sourced${NC}"
else
    echo -e "${YELLOW}⚠ Project not built. Building now...${NC}"
    colcon build
    source install/setup.bash
    echo -e "${GREEN}✓ Project built and sourced${NC}"
fi

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
echo -e "${GREEN}✓ TurtleBot3 model set to: $TURTLEBOT3_MODEL${NC}\n"

# Launch the complete simulation
echo -e "${YELLOW}Launching complete simulation...${NC}"
echo -e "${YELLOW}This will open:${NC}"
echo -e "${YELLOW}  - Gazebo (physics simulation)${NC}"
echo -e "${YELLOW}  - RViz (visualization)${NC}"
echo -e "${YELLOW}  - Navigation stack${NC}\n"

ros2 launch project complete_sim.launch.py
