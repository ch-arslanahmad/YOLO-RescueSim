#!/bin/bash
# Quick Launch Script for YOLO Rescue Simulation
# Starts the complete simulation with Gazebo, RViz, and Navigation

set -e
set -o pipefail

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}YOLO-RescueSim Launcher${NC}"
echo -e "${GREEN}================================${NC}\n"

usage() {
        cat <<'EOF'
Usage:
    ./launch_sim.sh            # interactive menu
    ./launch_sim.sh --help     # show help
    ./launch_sim.sh --guide    # quick guide / tips

Notes:
    - Option 1 launches your workspace world: project/turtle.sdf
    - Option 2 launches TurtleBot3's default empty world
    - Option 3 runs teleop keyboard (Ctrl+C to stop)
    - Option 4 runs the manual waypoint recorder (type 'd' for demo)
    - Option 5 runs open-loop waypoint playback (no /odom required)
    - Option 7 opens a camera viewer for /camera/image_raw
EOF
}

guide() {
        cat <<'EOF'
Guide:
    1) If you are iterating on the world file:
             Edit:   project/turtle.sdf
             Run:    choose option 1

    2) If you want to sanity-check that ROS + Gazebo + TurtleBot work:
             choose option 2 (default world) then option 3 (teleop)

    3) Waypoint recording (manual recorder):
             choose option 4, then at the prompt type:
                 d

    4) Waypoint auto-move (open-loop playback, no /odom needed):
             In one terminal, run the sim (option 1).
             In another terminal, choose option 5 and paste your CSV path.
             (Use 'l' in the recorder to print the last exported CSV path.)

Tips:
    - If Gazebo opens multiple times, you likely have old gz processes.
        Use:  pkill -f "gz sim"  (or restart your terminal/session)
EOF
}

case "${1:-}" in
        --help|-h)
                usage
                exit 0
                ;;
        --guide)
                guide
                exit 0
                ;;
esac

# Check if ROS 2 is available
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo -e "${RED}ERROR: ROS 2 Jazzy not found!${NC}"
    exit 1
fi

# Source ROS 2
source /opt/ros/jazzy/setup.bash
echo -e "${GREEN}OK: ROS 2 Jazzy sourced${NC}"

# Navigate to project directory
cd /home/arslan/Desktop/github/YOLO-RescueSim

PROJECT_ROOT="$(pwd)"
SOURCE_WORLD_SDF="$PROJECT_ROOT/project/turtle.sdf"

ensure_project_overlay() {
    if [ -f install/setup.bash ]; then
        # shellcheck disable=SC1091
        source install/setup.bash
        echo -e "${GREEN}OK: Project setup sourced${NC}"
    else
        echo -e "${YELLOW}WARN: Project not built. Building now...${NC}"
        colcon build
        # shellcheck disable=SC1091
        source install/setup.bash
        echo -e "${GREEN}OK: Project built and sourced${NC}"
    fi
}

set_tb3_model() {
    export TURTLEBOT3_MODEL=burger
    echo -e "${GREEN}OK: TurtleBot3 model set to: $TURTLEBOT3_MODEL${NC}\n"
}

print_menu() {
    echo -e "${YELLOW}Choose an option:${NC}"
    echo "  1) Launch YOLO-RescueSim (turtle.sdf)"
    echo "  2) Launch TurtleBot3 default empty world"
    echo "  3) Run TurtleBot3 teleop keyboard"
    echo "  4) Start manual waypoint recorder (then type 'd')"
    echo "  5) Play waypoints (open-loop, no /odom)"
    echo "  6) Clean and rebuild project"
    echo "  7) View TurtleBot camera (/camera/image_raw)"
    echo "  q) Quit"
}

print_menu
read -r -p "> " choice

case "$choice" in
    1)
        ensure_project_overlay
        set_tb3_model
        echo -e "${YELLOW}Launching YOLO-RescueSim using turtle.sdf...${NC}"
        if [ -f "$SOURCE_WORLD_SDF" ]; then
            echo -e "${GREEN}OK: World: $SOURCE_WORLD_SDF${NC}\n"
            ros2 launch project rescue_sim.launch.py world:="$SOURCE_WORLD_SDF"
        else
            echo -e "${YELLOW}WARN: World not found at $SOURCE_WORLD_SDF${NC}"
            echo -e "${YELLOW}  Falling back to installed world via launch defaults.${NC}\n"
            ros2 launch project rescue_sim.launch.py
        fi
        ;;
    2)
        set_tb3_model
        echo -e "${YELLOW}Launching TurtleBot3 default empty world...${NC}\n"
        ros2 launch turtlebot3_gazebo empty_world.launch.py
        ;;
    3)
        ensure_project_overlay
        set_tb3_model
        echo -e "${YELLOW}Starting teleop (CTRL-C to stop)...${NC}\n"
        ros2 run turtlebot3_teleop teleop_keyboard
        ;;
    4)
        ensure_project_overlay
        set_tb3_model
        echo -e "${YELLOW}Starting manual waypoint recorder...${NC}"
        echo -e "${YELLOW}When prompted, type: d${NC}\n"
        # Use system python to avoid venv isolation issues.
        /usr/bin/python3 -c "from navigation_scripts.navigation.manual_record_waypoints import main; main()"
        ;;
    5)
        ensure_project_overlay
        set_tb3_model
        echo -e "${YELLOW}Starting open-loop waypoint playback...${NC}"
        echo -e "${YELLOW}Tip: Make sure the simulation is already running (option 1) in another terminal.${NC}\n"
        if [ -f "$PROJECT_ROOT/waypoints/waypoints_camera_scan.csv" ]; then
            export WAYPOINT_CSV="$PROJECT_ROOT/waypoints/waypoints_camera_scan.csv"
            echo -e "${GREEN}OK: Auto-selecting WAYPOINT_CSV=$WAYPOINT_CSV${NC}"
        elif [ -f "$PROJECT_ROOT/project/navigation_scripts/waypoints/waypoints_camera_scan.csv" ]; then
            export WAYPOINT_CSV="$PROJECT_ROOT/project/navigation_scripts/waypoints/waypoints_camera_scan.csv"
            echo -e "${GREEN}OK: Auto-selecting WAYPOINT_CSV=$WAYPOINT_CSV${NC}"
        else
            unset WAYPOINT_CSV || true
        fi
        /usr/bin/python3 -c "from navigation_scripts.navigation.open_loop_waypoint_player import main; main()"
        unset WAYPOINT_CSV || true
        ;;
    6)
        echo -e "${YELLOW}Cleaning build/install directories...${NC}"
        rm -rf build install
        echo -e "${GREEN}OK: Cleaned${NC}"
        
        echo -e "${YELLOW}Building project...${NC}"
        colcon build --packages-select project
        
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}OK: Build successful${NC}"
            # shellcheck disable=SC1091
            source install/setup.bash
            echo -e "${GREEN}OK: Project setup sourced${NC}"
        else
            echo -e "${RED}ERROR: Build failed${NC}"
            exit 1
        fi
        ;;
    7)
        ensure_project_overlay
        echo -e "${YELLOW}Opening camera viewer for /camera/image_raw...${NC}"
        echo -e "${YELLOW}Tip: Make sure the sim is running (option 1) in another terminal.${NC}\n"

        # Auto-select the topic (no GUI dropdown needed).
        if ros2 run image_view image_view --ros-args -r image:=/camera/image_raw; then
            :
        else
            echo -e "${YELLOW}WARN: image_view not available. Falling back to rqt_image_view...${NC}"
            ros2 run rqt_image_view rqt_image_view /camera/image_raw
        fi
        ;;
    q|Q)
        echo "Bye."
        exit 0
        ;;
    *)
        echo -e "${RED}Invalid option: $choice${NC}"
        exit 2
        ;;
esac
