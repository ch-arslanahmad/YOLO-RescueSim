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
    - Option 6 opens a camera viewer for /camera/image_raw
    - Option 7 opens a YOLO viewer for /yolo/detection_image
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

        4) Manual driving demo (teleop keyboard):
             choose option 1g and then pick:
                 n = start teleop now
                 d = start teleop after a short delay

        5) View YOLO annotated detections:
             choose option 7 (requires YOLO detection node running)

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
HUMAN_POSES_CSV="$PROJECT_ROOT/waypoints/human_poses.csv"
DEFAULT_AUTOPLAY_DELAY_SEC=8

ensure_project_overlay() {
    if [ -f install/setup.bash ]; then
        # shellcheck disable=SC1091
        source install/setup.bash
        echo -e "${GREEN}OK: Project setup sourced${NC}"
    else
        echo -e "${YELLOW}WARN: Project not built. Building now...${NC}"
        if [ -x "$PROJECT_ROOT/build_project.sh" ]; then
            "$PROJECT_ROOT/build_project.sh"
        else
            colcon build --packages-select project --symlink-install
        fi
        # shellcheck disable=SC1091
        source install/setup.bash
        echo -e "${GREEN}OK: Project built and sourced${NC}"
    fi
}

prepend_unique_path() {
    # Usage: prepend_unique_path VAR /some/path
    # Prepends PATH to env var VAR if not already present.
    local var_name="$1"
    local new_path="$2"
    local current_value
    current_value="${!var_name:-}"

    # Skip non-existent paths (prevents clutter and confusing warnings).
    if [ ! -d "$new_path" ]; then
        return 0
    fi

    case ":${current_value}:" in
        *":${new_path}:"*)
            return 0
            ;;
    esac

    if [ -n "$current_value" ]; then
        export "$var_name"="${new_path}:${current_value}"
    else
        export "$var_name"="${new_path}"
    fi
}

ensure_gz_resource_path() {
    # Make sure Gazebo can resolve model:// URIs (turtlebot3_world, walking_person_small, etc).
    # We do this in the launcher as a belt-and-suspenders approach in case the launch-file env
    # actions are bypassed by a different entry path.
    prepend_unique_path GZ_SIM_RESOURCE_PATH "$PROJECT_ROOT/project/models"
    prepend_unique_path GZ_SIM_RESOURCE_PATH "$PROJECT_ROOT/install/project/share/project/models"
    prepend_unique_path GZ_SIM_RESOURCE_PATH "/opt/ros/jazzy/share/turtlebot3_gazebo/models"
    echo -e "${GREEN}OK: GZ_SIM_RESOURCE_PATH configured${NC}"
}

kill_stale_sim_processes() {
    # If old Gazebo/launch processes are still running, a new launch can end up
    # spawning the robot into the already-running (often empty) world.
    # That looks like "only TurtleBot loads" even though the world file is fine.
    pkill -f "ros2 launch project rescue_sim.launch.py" 2>/dev/null || true
    pkill -f "ros2 launch project rescue_detection.launch.py" 2>/dev/null || true
    pkill -f "ros2 launch turtlebot3_gazebo" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true

    # In case a previous launch left nodes running (singleton-lock would make the next start exit).
    pkill -f "project.*yolo_detector" 2>/dev/null || true
    pkill -f "project.*human_tracker" 2>/dev/null || true
}

wait_for_topic_publisher() {
    # Usage: wait_for_topic_publisher /topic_name timeout_seconds
    local topic="$1"
    local timeout_sec="$2"
    local start_ts
    start_ts=$(date +%s)

    while true; do
        if ros2 topic info "$topic" 2>/dev/null | grep -Eq "Publisher count: [1-9]"; then
            return 0
        fi

        if [ $(( $(date +%s) - start_ts )) -ge "$timeout_sec" ]; then
            return 1
        fi
        sleep 0.5
    done
}

set_tb3_model() {
    export TURTLEBOT3_MODEL=burger
    echo -e "${GREEN}OK: TurtleBot3 model set to: $TURTLEBOT3_MODEL${NC}\n"
}

print_menu() {
    echo -e "${YELLOW}Choose an option:${NC}"
    echo "  1) Launch YOLO-RescueSim (turtle.sdf)"
    echo "     1b) Launch YOLO-RescueSim + YOLO detection (turtle.sdf)"
    echo "     1g) Launch sim + camera + teleop keyboard"
    echo "  2) Launch TurtleBot3 default empty world"
    echo "  3) Run TurtleBot3 teleop keyboard"
    echo "  4) Start manual waypoint recorder (then type 'd')"
    echo "     4b) Play scripted waypoint path (open-loop)"
    echo "  5) Clean and rebuild project"
    echo "  6) View TurtleBot camera (/camera/image_raw)"
    echo "  7) View YOLO detections (/yolo/detection_image)"
    echo "  q) Quit"
}

start_camera_viewer() {
    if ! ros2 topic info /camera/image_raw 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo -e "${YELLOW}WARN: /camera/image_raw has no publishers yet.${NC}"
        echo -e "${YELLOW}      Start the simulation first (option 1), then try again.${NC}\n"
    fi

    # Prefer image_view because it's lightweight. Fallback to rqt_image_view.
    if ros2 pkg executables image_view 2>/dev/null | grep -q "image_view"; then
        echo -e "${GREEN}OK: Starting camera viewer (image_view)${NC}"
        ros2 run image_view image_view --ros-args -r image:=/camera/image_raw >/dev/null 2>&1 &
        echo $!
        return 0
    fi

    if ros2 pkg executables rqt_image_view 2>/dev/null | grep -q "rqt_image_view"; then
        echo -e "${GREEN}OK: Starting camera viewer (rqt_image_view)${NC}"
        ros2 run rqt_image_view rqt_image_view >/dev/null 2>&1 &
        echo $!
        return 0
    fi

    echo -e "${YELLOW}WARN: No camera viewer package found (image_view or rqt_image_view).${NC}"
    return 1
}

start_yolo_viewer() {
    if ! ros2 topic info /yolo/detection_image 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo -e "${YELLOW}WARN: /yolo/detection_image has no publishers yet.${NC}"
        echo -e "${YELLOW}      Start detection first (option 1b), then try again.${NC}\n"
    fi

    if ! ros2 topic info /camera/image_raw 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo -e "${YELLOW}WARN: /camera/image_raw has no publishers yet.${NC}"
        echo -e "${YELLOW}      YOLO needs camera frames; start the simulation (option 1 or 1b).${NC}\n"
    fi

    echo -e "${GREEN}OK: Starting YOLO viewer (/yolo/detection_image)${NC}"
    ros2 run image_view image_view --ros-args -r image:=/yolo/detection_image
}

start_teleop_keyboard() {
    echo -e "${YELLOW}Starting teleop keyboard (CTRL-C to stop teleop).${NC}"
    echo -e "${YELLOW}Use WASD-like keys shown in the teleop prompt.${NC}\n"
    ros2 run turtlebot3_teleop teleop_keyboard
}

ensure_venv_for_yolo() {
    if [ -f "$PROJECT_ROOT/.venv/bin/activate" ]; then
        # shellcheck disable=SC1091
        source "$PROJECT_ROOT/.venv/bin/activate"
        if python -c "import ultralytics, torch, cv2" >/dev/null 2>&1; then
            echo -e "${GREEN}OK: YOLO venv active (.venv)${NC}"
        else
            echo -e "${YELLOW}WARN: .venv active but YOLO deps missing.${NC}"
            echo -e "${YELLOW}      Install inside venv: pip install ultralytics opencv-python${NC}"
        fi
    else
        echo -e "${YELLOW}WARN: .venv not found at $PROJECT_ROOT/.venv${NC}"
        echo -e "${YELLOW}      Create it and install deps before running detection.${NC}"
    fi
}

print_menu
read -r -p "> " choice

case "$choice" in
    1)
        kill_stale_sim_processes
        ensure_project_overlay
        set_tb3_model
        ensure_gz_resource_path
        echo -e "${YELLOW}Launching YOLO-RescueSim using turtle.sdf...${NC}"
        if [ -f "$SOURCE_WORLD_SDF" ]; then
            echo -e "${GREEN}OK: World: $SOURCE_WORLD_SDF${NC}\n"
            if [ -f "$HUMAN_POSES_CSV" ]; then
                echo -e "${GREEN}OK: Human poses CSV: $HUMAN_POSES_CSV${NC}\n"
                ros2 launch project rescue_sim.launch.py world:="$SOURCE_WORLD_SDF" world_template:="$SOURCE_WORLD_SDF" human_poses_csv:="$HUMAN_POSES_CSV"
            else
                ros2 launch project rescue_sim.launch.py world:="$SOURCE_WORLD_SDF"
            fi
        else
            echo -e "${YELLOW}WARN: World not found at $SOURCE_WORLD_SDF${NC}"
            echo -e "${YELLOW}  Falling back to installed world via launch defaults.${NC}\n"
            ros2 launch project rescue_sim.launch.py
        fi
        ;;
    1b)
        kill_stale_sim_processes
        ensure_project_overlay
        ensure_venv_for_yolo
        set_tb3_model
        ensure_gz_resource_path

        export YOLO_RESCUESIM_ROOT="$PROJECT_ROOT"

        echo -e "${YELLOW}Launching YOLO-RescueSim + YOLO detection...${NC}"
        echo -e "${YELLOW}This will start two ros2 launch processes and stop both on Ctrl+C.${NC}\n"

        SIM_CMD=(ros2 launch project rescue_sim.launch.py)
        if [ -f "$SOURCE_WORLD_SDF" ]; then
            SIM_CMD+=(world:="$SOURCE_WORLD_SDF")
            echo -e "${GREEN}OK: World: $SOURCE_WORLD_SDF${NC}"
            if [ -f "$HUMAN_POSES_CSV" ]; then
                SIM_CMD+=(world_template:="$SOURCE_WORLD_SDF" human_poses_csv:="$HUMAN_POSES_CSV")
                echo -e "${GREEN}OK: Human poses CSV: $HUMAN_POSES_CSV${NC}"
            fi
        else
            echo -e "${YELLOW}WARN: World not found at $SOURCE_WORLD_SDF (using launch default)${NC}"
        fi

        "${SIM_CMD[@]}" &
        SIM_PID=$!

        # Wait for camera publishers so detection starts reliably.
        if wait_for_topic_publisher /camera/image_raw 25; then
            echo -e "${GREEN}OK: Camera topic is publishing${NC}"
        else
            echo -e "${YELLOW}WARN: /camera/image_raw still has no publishers (starting detection anyway)${NC}"
        fi

        ros2 launch project rescue_detection.launch.py &
        DET_PID=$!

        cleanup() {
            echo -e "\n${YELLOW}Stopping simulation and detection...${NC}"
            kill "$DET_PID" "$SIM_PID" 2>/dev/null || true
            wait "$DET_PID" "$SIM_PID" 2>/dev/null || true
        }
        trap cleanup INT TERM

        wait "$SIM_PID"
        cleanup
        ;;
    1g)
        kill_stale_sim_processes
        ensure_project_overlay
        set_tb3_model
        ensure_gz_resource_path

        echo -e "${YELLOW}Launching sim + camera + teleop keyboard...${NC}"
        echo -e "${YELLOW}Ctrl+C stops everything.${NC}\n"

        SIM_CMD=(ros2 launch project rescue_sim.launch.py)
        if [ -f "$SOURCE_WORLD_SDF" ]; then
            SIM_CMD+=(world:="$SOURCE_WORLD_SDF")
            if [ -f "$HUMAN_POSES_CSV" ]; then
                SIM_CMD+=(world_template:="$SOURCE_WORLD_SDF" human_poses_csv:="$HUMAN_POSES_CSV")
            fi
        fi

        mkdir -p "$PROJECT_ROOT/log"
        SIM_LOG="$PROJECT_ROOT/log/sim_$(date +%Y%m%d_%H%M%S).log"
        echo -e "${GREEN}OK: Sim logs -> $SIM_LOG${NC}"
        ( "${SIM_CMD[@]}" >"$SIM_LOG" 2>&1 ) &
        SIM_PID=$!

        sleep 3

        VIEWER_PID=""
        if VIEWER_PID=$(start_camera_viewer); then
            true
        else
            VIEWER_PID=""
        fi

        cleanup() {
            echo -e "\n${YELLOW}Stopping sim and camera viewer...${NC}"
            if [ -n "$VIEWER_PID" ]; then kill "$VIEWER_PID" 2>/dev/null || true; fi
            kill "$SIM_PID" 2>/dev/null || true
            if [ -n "$VIEWER_PID" ]; then wait "$VIEWER_PID" 2>/dev/null || true; fi
            wait "$SIM_PID" 2>/dev/null || true
        }
        trap cleanup INT TERM

        echo -e "${YELLOW}Start teleop now or after a delay?${NC}"
        echo -e "${YELLOW}  n = now${NC}"
        echo -e "${YELLOW}  d = delay (${DEFAULT_AUTOPLAY_DELAY_SEC}s)${NC}"
        read -r -p "> " start_mode

        if [ "${start_mode,,}" = "d" ]; then
            sleep "$DEFAULT_AUTOPLAY_DELAY_SEC"
        fi

        start_teleop_keyboard

        cleanup
        ;;
    2)
        kill_stale_sim_processes
        set_tb3_model
        echo -e "${YELLOW}Launching TurtleBot3 default empty world...${NC}\n"
        ros2 launch turtlebot3_gazebo empty_world.launch.py
        ;;
    3)
        ensure_project_overlay
        set_tb3_model
        ensure_gz_resource_path
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
    4b)
        ensure_project_overlay
        set_tb3_model
        ensure_gz_resource_path
        echo -e "${YELLOW}Playing scripted waypoint path (open-loop)...${NC}"
        echo -e "${YELLOW}Tip: it will prompt you to choose a CSV (default: repo-root waypoints/).${NC}\n"
        ros2 run project play_waypoints
        ;;
    5)
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
    6)
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
    7)
        ensure_project_overlay
        echo -e "${YELLOW}Opening YOLO detections viewer for /yolo/detection_image...${NC}"
        echo -e "${YELLOW}Tip: Start YOLO detection first (option 1b or 1d).${NC}\n"
        start_yolo_viewer
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
