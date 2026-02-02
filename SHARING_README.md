# YOLO-RescueSim - Setup Instructions for Recipients

## Quick Start

This project is a ROS 2 + Gazebo autonomous rescue robot simulation with YOLO-based human detection.

### Prerequisites
- Ubuntu 24.04 LTS (recommended)
- ROS 2 Jazzy
- Python 3.12+
- 10GB free disk space

### Installation Steps

1. **Extract the archive**
   ```bash
   tar -xzf YOLO-RescueSim-clean.tar.gz
   cd YOLO-RescueSim
   ```

2. **Install ROS 2 dependencies** (if not already installed)
   ```bash
   chmod +x install_dependencies.sh
   ./install_dependencies.sh
   ```

3. **Install TurtleBot3 packages**
   ```bash
   chmod +x install_turtlebot3.sh
   ./install_turtlebot3.sh
   ```

4. **Setup Python environment and build project**
   ```bash
   chmod +x complete_setup.sh
   ./complete_setup.sh
   ```

5. **Launch the simulation**
   ```bash
   chmod +x launch_sim.sh
   ./launch_sim.sh
   ```

### What This Project Does
- Simulates a TurtleBot3 robot in Gazebo
- Uses YOLOv8 nano for real-time human detection
- Tracks detected humans and their locations
- Exports detection maps for rescue operations
- Autonomous navigation with SLAM

### Key Files
- `project/project/yolo_detector.py` - YOLO detection node
- `project/project/human_tracker.py` - Human location tracker
- `world.sdf` - Gazebo world definition
- `launch/rescue_sim.launch.py` - Main launch file

### Troubleshooting
- See `SETUP_INSTRUCTIONS.md` for detailed setup
- Check `docs/` folder for comprehensive documentation
- See `SETUP_COMPLETE.md` for verification steps

### Contact
For questions or issues with setup, refer to the project documentation in the `docs/` folder.
