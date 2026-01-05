# Copilot Instructions for YOLO-RescueSim

## Project Overview
**YOLO-RescueSim** is a ROS 2 + Gazebo-based autonomous rescue robot simulation using TurtleBot3 (burger model) with computer vision for human detection. The system integrates SLAM Toolbox for real-time mapping, Navigation2 for autonomous movement, and YOLO-Lite for edge-device human detection.

**Goal**: Detect humans/rescue markers in a simulated field and export detection markers on a map.

---

## Architecture Overview

### Core Stack
- **Simulator**: Gazebo (Harmonium) with ROS 2 (Jazzy)
- **Robot**: TurtleBot3 burger with integrated camera
- **Vision**: YOLO v8 nano (ultra-lightweight)
- **Navigation**: Navigation2 + SLAM Toolbox (frontier-based exploration)
- **Framework**: ROS 2 nodes (Python-based)

### Component Relationships
```
Gazebo World (world.sdf) 
  → TurtleBot3 spawned via rescue_sim.launch.py
    → Camera: /camera/image_raw (Image msgs)
    → Laser: /scan (sensor data)
    → Odometry: /odom (robot position)
    
YOLODetector (subscribes to /camera/image_raw)
  → Publishes: /yolo/detections (JSON), /yolo/detection_image
  
HumanTracker (subscribes to /yolo/detections + /odom)
  → Tracks human locations over time
  → Builds location database
  
MapExporter 
  → Reads tracking data
  → Exports CSV/JSON with human coordinates + robot pose
```

---

## Directory Structure & Key Files

```
project/
├── launch/
│   ├── rescue_sim.launch.py          # Main launch - Gazebo + TurtleBot3 + Camera
│   └── rescue_detection.launch.py    # YOLO detection pipeline launch
├── project/
│   ├── yolo_detector.py              # YOLO human detection node
│   ├── human_tracker.py              # Tracks human locations + robot pose
│   ├── map_exporter.py               # Exports detection map (CSV/JSON/image)
│   └── auto_navigation.py            # Frontier-based autonomous exploration
├── navigation_scripts/
│   ├── calibration/                  # Manual control & coordinate tracking
│   └── navigation/                   # Waypoint recording & playback
├── world.sdf                         # Gazebo world definition
├── turtle.sdf                        # TurtleBot3 URDF as SDF
├── package.xml                       # ROS 2 package metadata
└── setup.py                          # Entry points for console scripts
```

---

## Essential Workflows

### 1. **Launch Full Simulation + Detection**
```bash
export TURTLEBOT3_MODEL=burger
cd /home/arslan/Desktop/github/YOLO-RescueSim
source /opt/ros/jazzy/setup.bash
colcon build --packages-select project
source install/setup.bash

# Terminal 1: Gazebo + Robot
ros2 launch project rescue_sim.launch.py

# Terminal 2: YOLO Detection + Tracking
ros2 launch project rescue_detection.launch.py

# Terminal 3: Monitor topics
ros2 topic echo /yolo/detections
```

### 2. **Build & Rebuild**
```bash
colcon build --packages-select project --symlink-install  # symlink for live editing
colcon build --packages-select project --cmake-args -DCMAKE_BUILD_TYPE=Release  # optimize
```

### 3. **Debugging & Testing**
```bash
# List all active topics
ros2 topic list

# View detection data in real-time
ros2 topic echo /yolo/detections --once

# Check node communication
ros2 node list
ros2 node info /yolo_detector

# Inspect world state
ros2 param list
```

---

## ROS 2 Node Patterns (Critical)

All nodes follow this pattern (see `yolo_detector.py`, `human_tracker.py`, `auto_navigation.py`):

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        
        # Subscriptions
        self.create_subscription(MsgType, 'topic_name', self.callback, 10)
        
        # Publications
        self.pub = self.create_publisher(MsgType, 'output_topic', 10)
        
        # Timers for periodic tasks
        self.create_timer(0.1, self.timer_callback)  # 10 Hz
    
    def callback(self, msg):
        """Process incoming messages"""
        self.get_logger().info(f'Received: {msg}')
    
    def timer_callback(self):
        """Periodic work - don't block!"""
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Rules**:
- Use `self.get_logger().info()` for logging, NOT print()
- QoS = 10 (queue size for subscribers/publishers)
- Callbacks must be non-blocking; use timers for periodic work
- Always `rclpy.init()` and `rclpy.shutdown()`

---

## Gazebo World Configuration (world.sdf)

World uses ODE physics with these plugins:
- **Physics**: ODE, max_step_size=0.01s, real_time_factor=1.0
- **SceneBroadcaster**: Handles TF broadcasting
- **Sensors**: Processes camera + laser data
- **Imu**: Inertial measurement

**When adding models to world.sdf**:
1. Define `<model>` with `<link>` + `<collision>` + `<visual>`
2. Include `<plugin>` for physics behavior
3. Restart Gazebo for changes (or use `ros2 service call` for dynamic spawning)

Example human model insertion (SDF format):
```xml
<model name='human_1'>
  <pose>5.0 3.0 0.0 0 0 0</pose>
  <link name='body'>
    <collision name='collision'>
      <geometry><cylinder><height>1.7</height><radius>0.3</radius></cylinder></geometry>
    </collision>
    <visual name='visual'>
      <geometry><cylinder><height>1.7</height><radius>0.3</radius></cylinder></geometry>
      <material><color>0.8 0.3 0.3</color></material>
    </visual>
  </link>
</model>
```

---

## Message Topics & Types

| Topic | Type | Publisher | Purpose |
|-------|------|-----------|---------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin | Robot camera feed |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo laser plugin | 360° laser distance data |
| `/odom` | `nav_msgs/Odometry` | TurtleBot3 | Robot pose (x, y, theta) |
| `/yolo/detections` | `std_msgs/String` (JSON) | `yolo_detector.py` | Human bbox + confidence |
| `/yolo/detection_image` | `sensor_msgs/Image` | `yolo_detector.py` | Annotated image with boxes |
| `/cmd_vel` | `geometry_msgs/Twist` | Any controller | Robot velocity commands |

**Important**: Detection JSON format (from `yolo_detector.py`):
```json
{
  "timestamp": "2026-01-04T12:00:00",
  "detections": [
    {"class": "person", "confidence": 0.92, "bbox": [x1, y1, x2, y2]},
    {"class": "person", "confidence": 0.87, "bbox": [x1, y1, x2, y2]}
  ]
}
```

---

## Key Conventions & Patterns

1. **File Organization**: 
   - ROS nodes in `project/` (e.g., `yolo_detector.py`)
   - Navigation utilities in `navigation_scripts/`
   - Launch files in `launch/` directory

2. **Naming**:
   - Node names: lowercase with underscores (e.g., `yolo_detector`, `human_tracker`)
   - Topics: forward-slash namespaced (e.g., `/yolo/detections`, `/robot/pose`)
   - Classes: PascalCase (e.g., `YOLODetector`, `HumanTracker`)

3. **Error Handling**:
   - Log errors with `self.get_logger().error()`, don't crash
   - Gracefully handle missing YOLO model (fallback to OpenCV)
   - Test with `try/except` around model loading

4. **Performance**:
   - Image processing happens in callbacks at ~30 Hz (camera frequency)
   - YOLO inference runs on every frame (consider skip frames if too slow)
   - Use `cv_bridge.CvBridge()` for ROS ↔ OpenCV conversion

5. **Dependencies**:
   - Core: `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`
   - Vision: `opencv-python`, `ultralytics` (for YOLO)
   - Navigation: `slam-toolbox`, `nav2` packages
   - Always add to `package.xml` `<depend>` tags

---

## Common Tasks & How-To

### Add a New ROS Node
1. Create `project/my_node.py` with Node class
2. Add entry point in `setup.py`: `'my_node = project.my_node:main'`
3. Add dependencies to `package.xml`
4. Run `colcon build --packages-select project`
5. Launch with `ros2 run project my_node`

### Subscribe to Camera & Process Image
```python
from cv_bridge import CvBridge
self.bridge = CvBridge()
self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)

def image_cb(self, msg):
    cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    # Process cv_image...
```

### Publish Detection Results
```python
from std_msgs.msg import String
import json

pub = self.create_publisher(String, '/yolo/detections', 10)
msg = String(data=json.dumps({"detections": [...]}))
pub.publish(msg)
```

### Export Data to CSV
Use `pandas` + `csv` module (see `map_exporter.py` for example):
```python
import csv
with open('detections.csv', 'w') as f:
    writer = csv.DictWriter(f, fieldnames=['timestamp', 'x', 'y', 'confidence'])
    writer.writeheader()
    for detection in detections:
        writer.writerow(detection)
```

---

## Deployment Considerations

- **Robot**: TurtleBot3 burger (real hardware uses same ROS 2 stack)
- **Edge Processing**: YOLO nano (~4MB) runs on Jetson Nano / Intel NUC
- **Sim-to-Real**: Topics, message types, coordinate frames are identical
- **Time Sync**: Use `use_sim_time:=true` in launch files for synchronized simulation

---

## Troubleshooting Quick Reference

| Issue | Cause | Solution |
|-------|-------|----------|
| `/camera/image_raw` not available | Camera plugin not loaded in world.sdf | Check `rescue_sim.launch.py` camera initialization |
| YOLO model won't load | `ultralytics` not installed | `pip install ultralytics` |
| Detections lag | Inference too slow | Skip frames: process every Nth image |
| Robot doesn't move | Gazebo physics frozen or cmd_vel not subscribed | Check `ros2 topic echo /cmd_vel` |
| TF errors in logs | Coordinate frame mismatch | Verify `turtle.sdf` has `<frame_name>` tags |

---

## Resources & References

- **ROS 2 Docs**: https://docs.ros.org/en/jazzy/
- **TurtleBot3 Manual**: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- **Gazebo SDF Format**: http://gazebosim.org/docs/latest/sdf_worlds/
- **OpenCV + ROS Bridge**: http://wiki.ros.org/cv_bridge
- **YOLO v8**: https://github.com/ultralytics/ultralytics
