#!/bin/bash
# Complete ROS 2 Jazzy + TurtleBot3 Setup for YOLO-RescueSim
set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Complete ROS 2 + TurtleBot3 Setup${NC}"
echo -e "${GREEN}========================================${NC}\n"

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
    echo -e "${RED}ERROR: Do not run as root${NC}"
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Step 1: Add ROS 2 repository
echo -e "${BLUE}[1/6] Setting up ROS 2 Jazzy repository...${NC}"
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release

# Add ROS GPG key properly
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# Step 2: Install basic ROS packages
echo -e "\n${BLUE}[2/6] Installing base ROS 2 packages...${NC}"
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git

# Step 3: Build TurtleBot3 from source
echo -e "\n${BLUE}[3/6] Building TurtleBot3 from source...${NC}"
TB3_WS="$HOME/turtlebot3_ws"
mkdir -p "$TB3_WS/src"
cd "$TB3_WS/src"

# Clone repos
if [ ! -d "turtlebot3_msgs" ]; then
    git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git || \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
fi

if [ ! -d "turtlebot3" ]; then
    git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git || \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
fi

if [ ! -d "turtlebot3_simulations" ]; then
    git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git || \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
fi

# Build
echo -e "\n${BLUE}[4/6] Building TurtleBot3 packages...${NC}"
cd "$TB3_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select turtlebot3_msgs
colcon build --symlink-install

# Step 5: Install visualization tools
echo -e "\n${BLUE}[5/6] Installing visualization packages...${NC}"
sudo apt install -y \
    python3-pip \
    python3-venv

# Step 6: Python environment for YOLO
echo -e "\n${BLUE}[6/6] Setting up YOLO Python environment...${NC}"
cd "$SCRIPT_DIR"

if [ ! -d ".venv" ]; then
    python3 -m venv .venv
    source .venv/bin/activate
    pip install --upgrade pip
    
    if [ -f "requirements-yolo-venv.txt" ]; then
        pip install -r requirements-yolo-venv.txt
    else
        pip install "numpy<2" "opencv-python<4.12" ultralytics
    fi
    deactivate
fi

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}\n"

echo -e "${YELLOW}IMPORTANT: Add to ~/.bashrc:${NC}"
echo -e "${BLUE}source $TB3_WS/install/setup.bash${NC}\n"

echo -e "${YELLOW}For this session:${NC}"
echo -e "${BLUE}source $TB3_WS/install/setup.bash${NC}"
echo -e "${BLUE}cd $SCRIPT_DIR${NC}"
echo -e "${BLUE}./build_project.sh${NC}"
echo -e "${BLUE}./launch_sim.sh${NC}\n"
