#!/bin/bash
# YOLO-RescueSim Dependency Installer
# Installs all required ROS 2 and system packages

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}YOLO-RescueSim Dependency Installer${NC}"
echo -e "${GREEN}================================${NC}\n"

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
    echo -e "${RED}ERROR: Do not run this script as root (sudo)${NC}"
    echo -e "${YELLOW}The script will ask for sudo password when needed${NC}"
    exit 1
fi

# Check for ROS 2 Jazzy
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo -e "${RED}ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy${NC}"
    echo -e "${YELLOW}Please install ROS 2 Jazzy first. See docs/setup.md${NC}"
    exit 1
fi

echo -e "${BLUE}[1/6] Updating package lists...${NC}"
sudo apt update

echo -e "\n${BLUE}[2/6] Installing TurtleBot3 packages...${NC}"
sudo apt install -y \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-teleop \
    ros-jazzy-turtlebot3-msgs

echo -e "\n${BLUE}[3/6] Installing Gazebo-ROS bridge packages...${NC}"
sudo apt install -y \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image

echo -e "\n${BLUE}[4/6] Installing navigation packages...${NC}"
sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-nav2-msgs

echo -e "\n${BLUE}[5/6] Installing visualization tools...${NC}"
sudo apt install -y \
    ros-jazzy-image-view \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rviz2

echo -e "\n${BLUE}[6/6] Installing Python dependencies...${NC}"
sudo apt install -y \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions

echo -e "\n${GREEN}================================${NC}"
echo -e "${GREEN}ROS Packages Installation Complete!${NC}"
echo -e "${GREEN}================================${NC}\n"

# Create Python virtual environment for YOLO
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ ! -d ".venv" ]; then
    echo -e "${BLUE}Creating Python virtual environment for YOLO...${NC}"
    python3 -m venv .venv
    
    echo -e "${BLUE}Installing YOLO dependencies in .venv...${NC}"
    source .venv/bin/activate
    pip install --upgrade pip
    
    if [ -f "requirements-yolo-venv.txt" ]; then
        pip install -r requirements-yolo-venv.txt
        echo -e "${GREEN}OK: YOLO dependencies installed${NC}"
    else
        echo -e "${YELLOW}WARN: requirements-yolo-venv.txt not found${NC}"
        echo -e "${YELLOW}Installing ultralytics manually...${NC}"
        pip install "numpy<2" "opencv-python<4.12" ultralytics
    fi
    
    deactivate
else
    echo -e "${YELLOW}WARN: .venv already exists, skipping creation${NC}"
    echo -e "${YELLOW}To reinstall, delete .venv and run this script again${NC}"
fi

echo -e "\n${GREEN}================================${NC}"
echo -e "${GREEN}All Dependencies Installed!${NC}"
echo -e "${GREEN}================================${NC}\n"

echo -e "${YELLOW}Next Steps:${NC}"
echo -e "1. Build the project:"
echo -e "   ${BLUE}./build_project.sh${NC}"
echo -e ""
echo -e "2. Launch the simulation:"
echo -e "   ${BLUE}./launch_sim.sh${NC}"
echo -e ""
echo -e "For more information, see ${BLUE}docs/setup.md${NC}"
echo -e ""
