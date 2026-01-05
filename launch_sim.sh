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

        5) Hands-free demo (camera + auto waypoint playback):
             choose option 1e (starts after a delay)
             choose option 1f (start on demand)

        6) Manual driving demo (teleop keyboard):
             choose option 1g (start teleop now)
             choose option 1h (start teleop after a delay)

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
PERSIST_WORLD_SDF="$PROJECT_ROOT/project/turtle_from_csv.sdf"
DEFAULT_AUTOPLAY_DELAY_SEC=8

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
    echo "  1b) Launch YOLO-RescueSim + YOLO detection (turtle.sdf)"
    echo "  1c) Launch YOLO-RescueSim (cached CSV world)"
    echo "  1d) Launch YOLO-RescueSim + YOLO detection (cached CSV world)"
    echo "  1e) Launch sim + camera + auto waypoint playback (delayed)"
    echo "  1f) Launch sim + camera + auto waypoint playback (on demand)"
    echo "  1g) Launch sim + camera + teleop keyboard (on demand)"
    echo "  1h) Launch sim + camera + teleop keyboard (delayed)"
    echo "  2) Launch TurtleBot3 default empty world"
    echo "  3) Run TurtleBot3 teleop keyboard"
    echo "  4) Start manual waypoint recorder (then type 'd')"
    echo "  5) Play waypoints (open-loop, no /odom)"
    echo "  6) Clean and rebuild project"
    echo "  7) View TurtleBot camera (/camera/image_raw)"
    echo "  q) Quit"
}

pick_waypoint_csv() {
    # Priority:
    # 1) WAYPOINT_CSV env var
    # 2) common known files
    # 3) newest waypoints_*.csv (excluding human_poses.csv)
    if [ -n "${WAYPOINT_CSV:-}" ] && [ -f "$WAYPOINT_CSV" ]; then
        echo "$WAYPOINT_CSV"
        return 0
    fi

    if [ -f "$PROJECT_ROOT/waypoints/waypoints_camera_scan.csv" ]; then
        echo "$PROJECT_ROOT/waypoints/waypoints_camera_scan.csv"
        return 0
    fi

    local latest
    latest=$(ls -t "$PROJECT_ROOT"/waypoints/*.csv 2>/dev/null | grep -v "human_poses.csv" | head -n 1 || true)
    if [ -n "$latest" ] && [ -f "$latest" ]; then
        echo "$latest"
        return 0
    fi

    return 1
}

first_waypoint_pose() {
    # Print "x y yaw" for the first waypoint in a waypoint CSV.
    # Expected columns: X_Position, Y_Position, Theta_Radians (or Theta_Degrees).
    local csv_path="$1"
    if [ -z "$csv_path" ] || [ ! -f "$csv_path" ]; then
        return 1
    fi

    /usr/bin/python3 - <<'PY' "$csv_path"
import csv
import math
import sys

path = sys.argv[1]
with open(path, 'r', newline='') as f:
    reader = csv.DictReader(f)
    row = next(reader, None)
    if not row:
        sys.exit(1)

    x = float(row['X_Position'])
    y = float(row['Y_Position'])
    if 'Theta_Radians' in row and str(row['Theta_Radians']).strip() != '':
        yaw = float(row['Theta_Radians'])
    elif 'Theta_Degrees' in row and str(row['Theta_Degrees']).strip() != '':
        yaw = math.radians(float(row['Theta_Degrees']))
    else:
        yaw = 0.0

print(f"{x} {y} {yaw}")
PY
}

start_camera_viewer() {
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

start_waypoint_playback() {
    local csv_path
    if ! csv_path=$(pick_waypoint_csv); then
        echo -e "${YELLOW}WARN: No waypoint CSV found in $PROJECT_ROOT/waypoints.${NC}"
        echo -e "${YELLOW}      Create one via option 4 (recorder) or set WAYPOINT_CSV to a valid file.${NC}"
        return 1
    fi
    echo -e "${GREEN}OK: Waypoint CSV: $csv_path${NC}"
    export WAYPOINT_CSV="$csv_path"
    /usr/bin/python3 -c "from navigation_scripts.navigation.open_loop_waypoint_player import main; main()" &
    echo $!
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
        ensure_project_overlay
        set_tb3_model
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
        ensure_project_overlay
        ensure_venv_for_yolo
        set_tb3_model

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

        # Give Gazebo a moment to start and publish topics
        sleep 3

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
    1c)
        ensure_project_overlay
        set_tb3_model

        echo -e "${YELLOW}Launching YOLO-RescueSim using cached CSV world...${NC}"
        if [ -f "$SOURCE_WORLD_SDF" ] && [ -f "$HUMAN_POSES_CSV" ]; then
            echo -e "${GREEN}OK: World template: $SOURCE_WORLD_SDF${NC}"
            echo -e "${GREEN}OK: Human poses CSV: $HUMAN_POSES_CSV${NC}"
            echo -e "${GREEN}OK: Cached generated world: $PERSIST_WORLD_SDF${NC}\n"
            ros2 launch project rescue_sim.launch.py \
                world:="$SOURCE_WORLD_SDF" \
                world_template:="$SOURCE_WORLD_SDF" \
                human_poses_csv:="$HUMAN_POSES_CSV" \
                generated_world_out:="$PERSIST_WORLD_SDF" \
                cache_generated_world:=true
        else
            echo -e "${YELLOW}WARN: Missing world or CSV; falling back to option 1 behavior.${NC}\n"
            ros2 launch project rescue_sim.launch.py world:="$SOURCE_WORLD_SDF"
        fi
        ;;
    1d)
        ensure_project_overlay
        ensure_venv_for_yolo
        set_tb3_model

        echo -e "${YELLOW}Launching YOLO-RescueSim + YOLO detection (cached CSV world)...${NC}"
        echo -e "${YELLOW}This will start two ros2 launch processes and stop both on Ctrl+C.${NC}\n"

        SIM_CMD=(ros2 launch project rescue_sim.launch.py)
        if [ -f "$SOURCE_WORLD_SDF" ]; then
            SIM_CMD+=(world:="$SOURCE_WORLD_SDF")
            echo -e "${GREEN}OK: World template: $SOURCE_WORLD_SDF${NC}"
            if [ -f "$HUMAN_POSES_CSV" ]; then
                SIM_CMD+=(world_template:="$SOURCE_WORLD_SDF" human_poses_csv:="$HUMAN_POSES_CSV" generated_world_out:="$PERSIST_WORLD_SDF" cache_generated_world:=true)
                echo -e "${GREEN}OK: Human poses CSV: $HUMAN_POSES_CSV${NC}"
                echo -e "${GREEN}OK: Cached generated world: $PERSIST_WORLD_SDF${NC}"
            fi
        else
            echo -e "${YELLOW}WARN: World not found at $SOURCE_WORLD_SDF (using launch default)${NC}"
        fi

        "${SIM_CMD[@]}" &
        SIM_PID=$!

        sleep 3

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
    1e)
        ensure_project_overlay
        set_tb3_model

        echo -e "${YELLOW}Launching sim + camera + auto waypoint playback (delayed)...${NC}"
        echo -e "${YELLOW}Ctrl+C stops sim, viewer, and playback.${NC}\n"

        SIM_CMD=(ros2 launch project rescue_sim.launch.py)
        if [ -f "$SOURCE_WORLD_SDF" ]; then
            SIM_CMD+=(world:="$SOURCE_WORLD_SDF")
            if [ -f "$HUMAN_POSES_CSV" ]; then
                SIM_CMD+=(world_template:="$SOURCE_WORLD_SDF" human_poses_csv:="$HUMAN_POSES_CSV")
            fi
        fi

        # Align the robot spawn pose with the first waypoint so open-loop playback matches.
        WP_CSV=""
        if WP_CSV=$(pick_waypoint_csv); then
            if WP_POSE=$(first_waypoint_pose "$WP_CSV" 2>/dev/null); then
                WP_X=$(echo "$WP_POSE" | awk '{print $1}')
                WP_Y=$(echo "$WP_POSE" | awk '{print $2}')
                WP_YAW=$(echo "$WP_POSE" | awk '{print $3}')
                echo -e "${GREEN}OK: Spawning at first waypoint: x=$WP_X y=$WP_Y yaw=$WP_YAW${NC}"
                SIM_CMD+=(x_pose:="$WP_X" y_pose:="$WP_Y" yaw_pose:="$WP_YAW")
            fi
        fi

        mkdir -p "$PROJECT_ROOT/log"
        SIM_LOG="$PROJECT_ROOT/log/sim_$(date +%Y%m%d_%H%M%S).log"
        echo -e "${GREEN}OK: Sim logs -> $SIM_LOG${NC}"
        ( "${SIM_CMD[@]}" >"$SIM_LOG" 2>&1 ) &
        SIM_PID=$!

        # Give Gazebo + bridges time to start publishing /camera/image_raw
        sleep 3

        VIEWER_PID=""
        if VIEWER_PID=$(start_camera_viewer); then
            true
        else
            VIEWER_PID=""
        fi

        echo -e "${YELLOW}Auto-starting waypoint playback in ${DEFAULT_AUTOPLAY_DELAY_SEC}s...${NC}"
        ( sleep "$DEFAULT_AUTOPLAY_DELAY_SEC"; start_waypoint_playback >/dev/null ) &
        PLAY_LAUNCH_PID=$!

        cleanup() {
            echo -e "\n${YELLOW}Stopping sim, camera viewer, and playback...${NC}"
            kill "$PLAY_LAUNCH_PID" 2>/dev/null || true
            if [ -n "$VIEWER_PID" ]; then kill "$VIEWER_PID" 2>/dev/null || true; fi
            kill "$SIM_PID" 2>/dev/null || true
            wait "$PLAY_LAUNCH_PID" 2>/dev/null || true
            if [ -n "$VIEWER_PID" ]; then wait "$VIEWER_PID" 2>/dev/null || true; fi
            wait "$SIM_PID" 2>/dev/null || true
        }
        trap cleanup INT TERM

        wait "$SIM_PID"
        cleanup
        ;;
    1f)
        ensure_project_overlay
        set_tb3_model

        echo -e "${YELLOW}Launching sim + camera + auto waypoint playback (on demand)...${NC}"
        echo -e "${YELLOW}Press Enter to start waypoint playback. Ctrl+C stops everything.${NC}\n"

        SIM_CMD=(ros2 launch project rescue_sim.launch.py)
        if [ -f "$SOURCE_WORLD_SDF" ]; then
            SIM_CMD+=(world:="$SOURCE_WORLD_SDF")
            if [ -f "$HUMAN_POSES_CSV" ]; then
                SIM_CMD+=(world_template:="$SOURCE_WORLD_SDF" human_poses_csv:="$HUMAN_POSES_CSV")
            fi
        fi

        # Align the robot spawn pose with the first waypoint so open-loop playback matches.
        WP_CSV=""
        if WP_CSV=$(pick_waypoint_csv); then
            if WP_POSE=$(first_waypoint_pose "$WP_CSV" 2>/dev/null); then
                WP_X=$(echo "$WP_POSE" | awk '{print $1}')
                WP_Y=$(echo "$WP_POSE" | awk '{print $2}')
                WP_YAW=$(echo "$WP_POSE" | awk '{print $3}')
                echo -e "${GREEN}OK: Spawning at first waypoint: x=$WP_X y=$WP_Y yaw=$WP_YAW${NC}"
                SIM_CMD+=(x_pose:="$WP_X" y_pose:="$WP_Y" yaw_pose:="$WP_YAW")
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

        read -r -p "Start waypoint playback now? (Enter) " _
        PLAY_PID=""
        PLAY_PID=$(start_waypoint_playback || true)

        cleanup() {
            echo -e "\n${YELLOW}Stopping sim, camera viewer, and playback...${NC}"
            if [ -n "$PLAY_PID" ]; then kill "$PLAY_PID" 2>/dev/null || true; fi
            if [ -n "$VIEWER_PID" ]; then kill "$VIEWER_PID" 2>/dev/null || true; fi
            kill "$SIM_PID" 2>/dev/null || true
            if [ -n "$PLAY_PID" ]; then wait "$PLAY_PID" 2>/dev/null || true; fi
            if [ -n "$VIEWER_PID" ]; then wait "$VIEWER_PID" 2>/dev/null || true; fi
            wait "$SIM_PID" 2>/dev/null || true
        }
        trap cleanup INT TERM

        wait "$SIM_PID"
        cleanup
        ;;
    1g)
        ensure_project_overlay
        set_tb3_model

        echo -e "${YELLOW}Launching sim + camera + teleop keyboard (on demand)...${NC}"
        echo -e "${YELLOW}Press Enter to start teleop. Ctrl+C stops everything.${NC}\n"

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

        read -r -p "Start teleop keyboard now? (Enter) " _
        start_teleop_keyboard

        cleanup
        ;;
    1h)
        ensure_project_overlay
        set_tb3_model

        echo -e "${YELLOW}Launching sim + camera + teleop keyboard (delayed)...${NC}"
        echo -e "${YELLOW}Teleop will start in ${DEFAULT_AUTOPLAY_DELAY_SEC}s. Ctrl+C stops everything.${NC}\n"

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

        sleep "$DEFAULT_AUTOPLAY_DELAY_SEC"
        start_teleop_keyboard

        cleanup
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
