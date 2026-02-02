#!/bin/bash
# Install TurtleBot3 packages from source for ROS 2 Jazzy
set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}TurtleBot3 Source Installation${NC}"
echo -e "${GREEN}================================${NC}\n"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Source ROS
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo -e "${RED}ERROR: ROS 2 Jazzy not found${NC}"
    exit 1
fi
source /opt/ros/jazzy/setup.bash

# Install dependencies
echo -e "${BLUE}[1/4] Installing build dependencies...${NC}"
sudo apt update
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup

# Create workspace for TurtleBot3
echo -e "\n${BLUE}[2/4] Creating TurtleBot3 workspace...${NC}"
TB3_WS="$HOME/turtlebot3_ws"
mkdir -p "$TB3_WS/src"
cd "$TB3_WS/src"

# Clone TurtleBot3 repositories
echo -e "\n${BLUE}[3/4] Cloning TurtleBot3 repositories...${NC}"

if [ ! -d "turtlebot3" ]; then
    git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
fi

if [ ! -d "turtlebot3_msgs" ]; then
    git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
fi

if [ ! -d "turtlebot3_simulations" ]; then
    git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
fi

# Build packages
echo -e "\n${BLUE}[4/4] Building TurtleBot3 packages...${NC}"
cd "$TB3_WS"
colcon build --symlink-install

echo -e "\n${GREEN}================================${NC}"
echo -e "${GREEN}TurtleBot3 Installation Complete!${NC}"
echo -e "${GREEN}================================${NC}\n"

echo -e "${YELLOW}IMPORTANT: Add this to your shell startup (~/.bashrc):${NC}"
echo -e "${BLUE}source $TB3_WS/install/setup.bash${NC}\n"

echo -e "${YELLOW}For this terminal session, run:${NC}"
echo -e "${BLUE}source $TB3_WS/install/setup.bash${NC}\n"

echo -e "${YELLOW}Then rebuild this project:${NC}"
echo -e "${BLUE}cd $SCRIPT_DIR${NC}"
echo -e "${BLUE}./build_project.sh${NC}\n"
