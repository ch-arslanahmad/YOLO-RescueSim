# Auto Navigation Setup for YOLO Rescue Simulation

This guide covers the autonomous navigation system for TurtleBot3 in the rescue simulation environment.

## Prerequisites Installed

✅ **Navigation2** - Nav2 stack for autonomous navigation  
✅ **SLAM Toolbox** - Simultaneous Localization and Mapping  
✅ **Gazebo** - Simulation environment  
✅ **ROS 2 Jazzy** - Robot Operating System  

## Components

### 1. Auto Navigation Node (`auto_navigation.py`)
Main autonomous navigation system with:
- **Frontier Detection**: Identifies unexplored boundaries in the map
- **Goal Planning**: Automatically generates navigation goals at frontiers
- **SLAM Integration**: Works with SLAM Toolbox for real-time mapping
- **Obstacle Avoidance**: Uses laser scans for safe navigation

**Features:**
- Autonomous exploration of unknown environments
- Frontier-based exploration algorithm
- Goal tracking to avoid revisiting areas
- ROS Action client integration for Navigation2

### 2. Navigation Launch File (`navigation_stack.launch.py`)
Complete launch configuration that starts:
- Gazebo simulator with custom world
- TurtleBot3 (Burger model)
- State publisher for robot transforms
- SLAM Toolbox for mapping
- Navigation2 stack
- Auto Navigation node

### 3. Navigation Tester (`navigation_tester.py`)
Interactive testing utility for:
- Manual robot control (keyboard)
- Starting/stopping exploration
- Checking navigation status
- Debugging movement commands

### 4. Configuration File (`nav_config.py`)
Centralized parameters for:
- Robot specifications
- Velocity limits
- Navigation thresholds
- Sensor configurations
- Topic and service names

## How It Works

### Exploration Algorithm

```
1. Start in SLAM mapping mode → builds map
2. Scan occupancy grid for frontiers
3. Find frontier cells (free space next to unknown space)
4. Select closest unvisited frontier as goal
5. Navigate to goal using Navigation2
6. Repeat steps 2-5 until no frontiers remain
7. Complete map generated
```

### Frontier Detection

Frontiers are identified by scanning the occupancy grid:
- **Unknown cells**: -1 in map data
- **Free cells**: 0 in map data  
- **Occupied cells**: 100 in map data

A frontier is a free cell adjacent to unknown cells.

### Navigation Flow

```
┌─────────────────────────────────────────┐
│     Auto Navigation Node                │
├─────────────────────────────────────────┤
│ 1. Subscribe to map from SLAM Toolbox   │
│ 2. Process occupancy grid               │
│ 3. Find frontier cells                  │
│ 4. Select closest frontier              │
│ 5. Send goal to Navigation2             │
│ 6. Monitor progress                     │
│ 7. Repeat until exploration complete    │
└─────────────────────────────────────────┘
         ↓              ↓           ↓
    SLAM Toolbox  Navigation2   Gazebo/Robot
```

## Running the System

### Option 1: Full Navigation Stack
```bash
# Terminal 1: Launch everything
ros2 launch project navigation_stack.launch.py

# Terminal 2: Monitor topics (optional)
ros2 topic list
ros2 topic echo /map
```

### Option 2: Step-by-Step
```bash
# Terminal 1: Gazebo and robot
ros2 launch project rescue_sim.launch.py

# Terminal 2: SLAM Toolbox
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=true

# Terminal 3: Navigation2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

# Terminal 4: Auto Navigation
ros2 run project auto_navigation

# Terminal 5: Interactive control (optional)
ros2 run project navigation_tester
```

## ROS 2 Topics

**Input Topics:**
- `/map` - Occupancy grid from SLAM
- `/odom` - Odometry from robot
- `/scan` - Laser scans for mapping

**Output Topics:**
- `/cmd_vel` - Velocity commands to robot
- `/goal_pose` - Navigation goals

**Action Servers:**
- `navigate_to_pose` - Navigation2 goal action

## Parameters

Key parameters (in `nav_config.py`):

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MAX_LINEAR_VELOCITY` | 0.3 m/s | Max forward speed |
| `MAX_ANGULAR_VELOCITY` | 1.0 rad/s | Max turning speed |
| `FRONTIER_THRESHOLD` | 10 cells | Min frontier size |
| `GOAL_TOLERANCE` | 0.3 m | Goal reaching tolerance |
| `SLAM_MODE` | mapping | SLAM operation mode |
| `EXPLORATION_STRATEGY` | frontier | Exploration algorithm |

## Troubleshooting

### Robot not moving
- Check if `/cmd_vel` topic is being published: `ros2 topic echo /cmd_vel`
- Verify Gazebo simulation is running
- Check robot state: `ros2 run tf2_tools view_frames`

### No map being generated
- Verify SLAM Toolbox is running: `ros2 node list | grep slam`
- Check laser scan: `ros2 topic echo /scan`
- Monitor SLAM output: `ros2 topic echo /map`

### Navigation goals not working
- Ensure Navigation2 is running: `ros2 service list | grep nav2`
- Check transform tree: `ros2 run tf2_tools view_frames`
- Verify map frame exists: `ros2 tf2 lookup_transform map base_link`

### Performance Issues
- Reduce map resolution in `nav_config.py`
- Lower frontier threshold for larger jumps
- Decrease sensor update rates if needed

## Integration with YOLO Detection

The navigation system is designed to work alongside YOLO object detection:

1. While exploring, camera captures images
2. YOLO processes images for person/rescue flag detection
3. Detections logged with GPS coordinates from odometry
4. Navigation continues coverage pattern

Example integration:
```python
# In auto_navigation.py or separate detection node
while exploring:
    image = camera_subscriber.get_latest()
    detections = yolo_model.detect(image)
    
    for detection in detections:
        log_detection(
            type=detection.class_name,
            position=(robot_x, robot_y),
            timestamp=time.time()
        )
```

## Map Saving

After exploration completes, save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/rescue_sim_map
```

This creates:
- `/tmp/rescue_sim_map.pgm` - Map image
- `/tmp/rescue_sim_map.yaml` - Map metadata

## Next Steps

1. **Integrate YOLO Detection**: Combine navigation with object detection
2. **Add Path Recording**: Log robot path during exploration
3. **Implement Marker System**: Mark detected victims/hazards on map
4. **Add Communication**: Report findings to base station
5. **Optimize Coverage**: Implement boustrophedon coverage for complete scans

## Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/StanleyInnovation/slam_toolbox)
- [TurtleBot3 Docs](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
