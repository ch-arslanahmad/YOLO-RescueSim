# YOLO-RescueSim

Autonomous rescue robot simulation using ROS 2, Gazebo, and YOLOv8 for human detection.

## Overview

TurtleBot3 robot with camera detects and tracks humans in rescue scenarios using YOLOv8 nano model.

## Prerequisites

- Ubuntu 22.04 or 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12+

## Setup

```bash
# Install dependencies
./install_dependencies.sh
./install_turtlebot3.sh

# Build project
./complete_setup.sh

# Launch simulation
./launch_sim.sh
```

## Features

- Real-time human detection using YOLOv8
- Autonomous navigation with waypoints
- Human position tracking and logging
- Gazebo simulation environment

## Project Structure

- `project/project/yolo_detector.py` - YOLO detection node
- `project/project/human_tracker.py` - Human tracking system
- `project/launch/` - ROS 2 launch files
- `project/models/` - Gazebo model definitions

## License

MIT
