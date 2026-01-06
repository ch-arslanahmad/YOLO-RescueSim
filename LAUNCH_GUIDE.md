# YOLO-RescueSim: Custom ROS 2 Launch Guide

## Quick Start

```bash
cd /home/arslan/Desktop/github/YOLO-RescueSim/project

# Set the robot model
export TURTLEBOT3_MODEL=burger

# Run the custom launch file
ros2 launch launch/rescue_sim.launch.py
```

---

## What the Launch File Does

The `rescue_sim.launch.py` is a **custom ROS 2 launch configuration** that:

1. **Launches Gazebo** with the **Gazebo Harmonic simulator** (`gz sim`)
2. **Loads your custom `world.sdf`** into the simulation
3. **Spawns TurtleBot3 (burger)** into the world
4. **Maintains ROS 2 integration** for all sensors and controls:
   - `/cmd_vel` → robot movement commands
   - `/camera/image_raw` → camera stream (for YOLO)
   - `/scan` → LiDAR data
   - Other standard TurtleBot3 topics

---

## File Structure

```
project/
├── world.sdf                      # Your custom Gazebo world
├── turtle.sdf                     # TurtleBot3 model reference
├── launch/
│   └── rescue_sim.launch.py       # Main ROS 2 launch file
└── LAUNCH_GUIDE.md                # This file
```

---

## How It Works (Technical Breakdown)

### Step 1: Set Environment Variables
```python
SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger")
```
- Tells TurtleBot3 which model to load (burger/waffle/waffle_pi)
- Must be set **before** launching

### Step 2: Include Gazebo Launch
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(gazebo_ros_pkg_share, "launch", "gazebo.launch.py")
    ),
    ...
)
```
- Loads Gazebo's standard ROS launcher
- Configures gz sim with ROS bridges

### Step 3: Pass World SDF
```python
launch_arguments={"world": world_sdf_path, "extra_gazebo_args": "--verbose"}
```
- Points Gazebo to your custom `world.sdf`
- `--verbose` enables detailed logging (remove for production)

### Step 4: Spawn Robot
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(robot_launch)
)
```
- Uses TurtleBot3's spawn script to place robot in world
- Automatically configures all ROS topics

---

## Key ROS 2 + Gazebo Concepts

| Component | Purpose | File Type |
|-----------|---------|-----------|
| **ROS 2 Launch** | Orchestrates node startup & configuration | `.launch.py` |
| **Gazebo World** | 3D environment, physics, assets | `.sdf` or `.world` |
| **ROS Bridges** | Connects ROS topics ↔ Gazebo simulation | Automatic |
| **TurtleBot Model** | Robot URDF/SDF + plugin configs | `.sdf` |

**Critical Rule**: ROS launch **controls everything**. The `.launch.py` file tells Gazebo what to load.

---

## Troubleshooting

### `ModuleNotFoundError: No module named 'ament_index_python'`
**Solution**: Source ROS 2 setup
```bash
source /opt/ros/jazzy/setup.bash
```

### `Cannot find world.sdf`
**Solution**: Ensure path is absolute
```bash
ros2 launch launch/rescue_sim.launch.py world:=/home/arslan/Desktop/github/YOLO-RescueSim/project/world.sdf
```

### Robot doesn't appear in Gazebo
**Solution**: Check `TURTLEBOT3_MODEL` is set
```bash
echo $TURTLEBOT3_MODEL  # Should print "burger"
```

### `/camera/image_raw` topic doesn't exist
**Solution**: Verify camera plugin is in `world.sdf`
```bash
ros2 topic list | grep camera
```

### ROS bridge fails to connect
**Solution**: Gazebo might not be running. Check logs:
```bash
# In separate terminal, watch Gazebo output
gazebo_log_output
```

---

## Advanced Usage

### Use a Different World
```bash
ros2 launch launch/rescue_sim.launch.py world:=/path/to/my_world.sdf
```

### Disable Verbose Logging
Edit `rescue_sim.launch.py`, remove `--verbose` from `extra_gazebo_args`

### Spawn Multiple Robots
Modify the launch file to include multiple `IncludeLaunchDescription` calls for different robot instances

### Record Sensor Data
```bash
# In separate terminal
ros2 bag record /camera/image_raw /scan /cmd_vel
```

---

## Integration with YOLO-RescueSim Pipeline

### Camera Topic Subscription (Next Steps)

Your YOLO detection node should subscribe to:
```python
from sensor_msgs.msg import Image
subscription = self.create_subscription(
    Image, 
    '/camera/image_raw',  # Published by this launch setup
    callback,
    10
)
```

### Expected Topics After Launch
```bash
ros2 topic list
# Output includes:
# /camera/image_raw
# /cmd_vel
# /scan
# /tf
# /gazebo/model_states
# ... (others)
```

### Publishing Robot Commands
```bash
# Manual test: make robot move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

---

## Environment Setup (First Time Only)

```bash
# Install ROS 2 Jazzy (if not already installed)
# See: https://docs.ros.org/en/jazzy/Installation.html

# Install TurtleBot3 packages
sudo apt install ros-jazzy-turtlebot3*

# Install Gazebo Harmonic
sudo apt install gz-harmonic

# Add to ~/.bashrc for convenience
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## What's Next?

1. **Launch file working** → Test it runs without errors
2. **Camera integration** → Subscribe to `/camera/image_raw`
3. **YOLO detection node** → Run inference on camera frames
4. **Marker detection** → Parse YOLO outputs for rescue markers
5. **Navigation stack** → Add autonomous movement logic
6. **Real robot transition** → Deploy to physical TurtleBot3

---

## References

- [ROS 2 Launch Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/Launch/Launch-system.html)
- [Gazebo + ROS Integration](https://gazebosim.org/docs/harmonic/ros/)
- [TurtleBot3 ROS 2 Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [Custom Gazebo Worlds](https://gazebosim.org/docs/harmonic/building_worlds/)

---

## Contact & Support

For issues specific to your YOLO-RescueSim project, check:
- `world.sdf` → Ensure valid Gazebo XML syntax
- `launch/rescue_sim.launch.py` → Check paths and environment variables
- ROS 2 logs → `ros2 doctor --report` for environment diagnostics
