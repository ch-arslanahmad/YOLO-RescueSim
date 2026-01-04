# Navigation Scripts for TurtleBot3 - Scripted Path Approach

This guide covers the **3-terminal workflow** for calibrating waypoints and navigating the TurtleBot3 through a known environment using **scripted paths** instead of autonomous navigation.

## Overview

The system uses a **3-terminal workflow**:

```
Terminal 1: Launch Gazebo with TurtleBot3 (simulation)
Terminal 2: Keyboard Controller (manual robot control)
Terminal 3: Coordinate Tracker (monitor & record waypoints)
```

Once waypoints are calibrated, use **Terminal 4** to execute the path.

---

## Folder Structure

```
project/
├── navigation_scripts/
│   ├── calibration/
│   │   ├── keyboard_controller.py       # Terminal 2: Manual control
│   │   └── coordinate_tracker.py        # Terminal 3: Track & record waypoints
│   ├── navigation/
│   │   └── waypoint_navigator.py        # Terminal 4: Execute waypoints
│   └── waypoints/
│       └── *.csv                        # Saved waypoint files
```

---

## Stage 1: Calibration Workflow

### Terminal 1: Launch Gazebo

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

This launches:
- Gazebo simulator with TurtleBot3 (default world)
- ROS bridge for communication

**Expected Output:**
- Gazebo window opens with a robot in a room with walls and obstacles

---

### Terminal 2: Keyboard Controller

```bash
cd ~/Desktop/github/YOLO-RescueSim
source install/setup.bash
ros2 run project keyboard_controller.py
```

**Controls:**
| Key | Action |
|-----|--------|
| `W` | Move Forward |
| `S` | Move Backward |
| `A` | Turn Left |
| `D` | Turn Right |
| `SPACE` | Emergency Stop |
| `+` | Increase Speed |
| `-` | Decrease Speed |
| `H` | Show Help |
| `Q` | Quit |

**Tips:**
- Drive the robot slowly to explore the environment
- Note interesting locations where YOLO should detect objects
- Be ready to record coordinates at specific points

---

### Terminal 3: Coordinate Tracker

```bash
cd ~/Desktop/github/YOLO-RescueSim
source install/setup.bash
ros2 run project coordinate_tracker.py
```

**Live Display:**
- Shows real-time position in format: `X: 0.123m | Y: 0.456m | Θ: 1.57rad (90.0°)`
- Updates continuously as robot moves

**Commands (press Enter after each):**

| Command | Action |
|---------|--------|
| `S` | **Save** current position as waypoint |
| `U` | **Undo** (delete last waypoint) |
| `P` | **Print** all saved waypoints |
| `C` | **Clear** all waypoints (requires confirmation) |
| `E` | **Export** waypoints to new file |
| `H` | Show help |
| `Q` | Quit |

**Workflow:**
1. Drive robot using Terminal 2 keyboard commands
2. When robot reaches a location of interest, press `Enter` in Terminal 3
3. Type `S` to save that coordinate
4. Repeat steps 1-3 for all key locations (10-20 waypoints recommended)
5. Press `P` to review all saved waypoints

**Error Recovery:**
- If you accidentally record a waypoint, press `U` to undo
- Press `P` to verify current waypoints before starting navigation
- All waypoints auto-save to CSV for editing later

**Output Files:**
- Waypoints saved to: `/project/navigation_scripts/waypoints/waypoints_YYYYMMDD_HHMMSS.csv`
- Format: `Waypoint_ID, X, Y, Theta_Radians, Theta_Degrees`

---

## Stage 2: Waypoint Navigation

After collecting waypoints, use the Navigator to execute the path.

### Terminal 4: Waypoint Navigator

```bash
cd ~/Desktop/github/YOLO-RescueSim
source install/setup.bash
ros2 run project waypoint_navigator.py
```

**Commands:**

| Command | Action |
|---------|--------|
| `L` | **Load** waypoints from CSV file |
| `S` | **Show** all loaded waypoints |
| `N` | **Navigate** through all waypoints sequentially |
| `G` | **Go** to specific waypoint by ID |
| `H` | Show help |
| `Q` | Quit |

**Navigation Workflow:**

```
1. Type "L" to load waypoints
   > Enter CSV file path: /path/to/waypoints_20250104_150230.csv

2. Type "S" to verify waypoints are loaded

3. Type "N" to navigate all waypoints
   > Pause time at each waypoint (seconds) [3]: 3
   
   Robot will:
   - Navigate to each waypoint sequentially
   - Stop for 3 seconds at each waypoint (for YOLO scanning)
   - Move to the next waypoint
   - Repeat until all waypoints visited
```

**Navigation Parameters:**
- **Linear Tolerance:** 0.15m (stops when within 15cm of target)
- **Angular Tolerance:** 0.1rad (stops when within 0.1rad of target angle)
- **Max Speed:** 0.2 m/s linear, 0.5 rad/s angular

---

## Example Workflow

### Session 1: Calibration

```bash
# Terminal 1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2
ros2 run project keyboard_controller.py
# Output: "Keyboard Controller Initialized"

# Terminal 3
ros2 run project coordinate_tracker.py
# Output: "[LIVE] X: 0.000m | Y: 0.000m | Θ: 0.000rad"

# Now in Terminal 2, drive the robot around
# In Terminal 3, save waypoints:
S  # Save waypoint 1
S  # Save waypoint 2
...
E  # Export to a named file
  > Enter export filename: my_mission
```

**Output:** `/project/navigation_scripts/waypoints/my_mission.csv`

### Session 2: Navigation

```bash
# Terminal 1 (still running)
# Terminal 2 (close previous, not needed)

# Terminal 4 (new)
ros2 run project waypoint_navigator.py

# Commands:
L
> Enter CSV file path: /home/arslan/Desktop/github/YOLO-RescueSim/project/navigation_scripts/waypoints/my_mission.csv

S  # Show all waypoints

N  # Start navigation
> Pause time at each waypoint (seconds) [3]: 5

# Robot navigates all waypoints, pausing 5 seconds at each for YOLO detection
```

---

## CSV File Format

Saved waypoints are stored in CSV format:

```csv
Waypoint_ID,X_Position,Y_Position,Theta_Radians,Theta_Degrees,Timestamp
1,0.0000,0.0000,0.0000,0.00,2025-01-04T15:02:30.123456
2,1.5234,0.8912,1.5708,90.00,2025-01-04T15:02:45.234567
3,2.1045,2.3456,3.1416,180.00,2025-01-04T15:02:58.345678
...
```

You can:
- **View** in any text editor
- **Edit** manually (be careful with coordinates)
- **Copy** and rename for different missions
- **Combine** multiple waypoint files

---

## Coordinate System Explained

**TurtleBot3 Default World** uses standard ROS conventions:

```
       Y (Forward)
       ↑
       │
       │
   ----+----→ X (Right)
       │
       ↓
       
Z points up (out of page)
```

**Orientation (Theta in Radians):**
- `0.0 rad` = Facing +X (right)
- `π/2 rad` (1.57) = Facing +Y (forward)
- `π rad` (3.14) = Facing -X (left)
- `3π/2 rad` (4.71) = Facing -Y (backward)

**Typical World Dimensions:**
- X range: -5 to +5 meters
- Y range: -5 to +5 meters
- Z range: 0 to +2 meters (height)

---

## Troubleshooting

### Robot doesn't move with keyboard commands
- **Check:** Terminal 1 (Gazebo) is running
- **Check:** `/cmd_vel` topic is being published:
  ```bash
  ros2 topic list | grep cmd_vel
  ```

### Waypoints not saving
- **Check:** `navigation_scripts/waypoints/` directory exists
- **Check:** Write permissions on the directory
- **Solution:** Create manually:
  ```bash
  mkdir -p ~/Desktop/github/YOLO-RescueSim/project/navigation_scripts/waypoints
  ```

### Robot oscillates around waypoint
- **Cause:** Coordinates have noise from odometry drift
- **Solution:** Increase tolerances in `waypoint_navigator.py`:
  ```python
  self.linear_tolerance = 0.20  # Increase from 0.15
  self.angular_tolerance = 0.15  # Increase from 0.1
  ```

### Robot goes to wrong waypoint
- **Cause:** Odometry drift accumulated
- **Solution:** Recalibrate waypoints in fresh session
- **Tip:** Start from origin (0, 0, 0) if possible

---

## Best Practices

1. **Calibration:**
   - Start from origin (0, 0, 0)
   - Move slowly and deliberately
   - Record waypoints at key decision points
   - Include return-to-origin waypoint

2. **Mission Design:**
   - 10-20 waypoints for thorough coverage
   - Include corners and center of each room
   - Space waypoints 1-3 meters apart
   - Increase pause time for important detection areas

3. **YOLO Integration:**
   - Record detection data at each waypoint
   - Use pause time for processing
   - Save images/detections for analysis

4. **Reusability:**
   - Name waypoint files descriptively: `warehouse_mission_v1.csv`
   - Keep multiple missions for different areas
   - Version control waypoint files

---

## Integration with YOLO Detection

Note: YOLO/detection integration described below is not yet tried or validated in this repository. Treat it as a planned integration point, not a tested workflow.

Once navigation is working, integrate YOLO by:

1. **Subscribe to robot odometry** in your YOLO node
2. **Trigger detection** when robot reaches waypoint (pause event)
3. **Save detections** with waypoint ID and timestamp
4. **Analyze** spatial distribution of detections

---

## Next Steps

1. Complete calibration workflow (Terminal 1-3)
2. Save waypoints to CSV
3. Test navigation with waypoint_navigator
4. Integrate YOLO detection at waypoints
5. Create warehouse-specific world
6. Design mission for warehouse environment

---

## Files Reference

| File | Purpose |
|------|---------|
| `keyboard_controller.py` | Manual robot control via keyboard |
| `coordinate_tracker.py` | Real-time position monitoring & waypoint recording |
| `waypoint_navigator.py` | Autonomous navigation through recorded waypoints |
| `waypoints/*.csv` | Saved waypoint missions |

---

**Questions?** Check the terminal output for detailed logging and error messages.
