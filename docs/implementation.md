# Implementation Flow

## Layman Terms

The flow for now is:

- [ ] Robot sees → Camera takes an image.
- [ ] Add Robot Movement across the field.
- [ ] Image goes to brain → A small YOLO model looks at the image.
- [ ] Brain decides → “This is a human” or “This is a rescue flag”.
- [ ] Robot reacts → Marks location, logs it, or moves.

## Technical Steps

- [x] Launch Gazebo with TurtleBot3 in a custom rescue world.
- [x] Verify it with camera
- [x] See via the camera
- [ ] Take image for YOLO lite 
1. **Camera Node**: Captures images and publishes them to a ROS topic.


## Currently Completed Steps

- Created a custom ROS 2 launch file (`rescue_sim.launch.py`) that:
  - Launches Gazebo with a custom `world.sdf`.
  - Spawns TurtleBot3 (burger) into that world.
  - Maintains ROS integration (`/cmd_vel`, sensors, camera topics).
- Verified camera and sensor topics are available in ROS 2 using `ros2 topic list`.
- Used the TurtleBot3 camera to confirm image feed is accessible via `/camera/image_raw` topic.
- Basic Robot Movement to navigate the whole world easily.

### Integration Points:

- [x] Navigation of Robot across the field to cover the entire area systematically.

**Completed:**
- SLAM Toolbox integrated for real-time mapping
- Navigation2 stack configured for autonomous movement
- Frontier-based exploration algorithm implemented in `auto_navigation.py`
- Auto-generated goals based on frontiers (unexplored boundaries)
- Goal tracking to avoid revisiting locations

**Created Components:**
1. **auto_navigation.py** - Main autonomous navigation node with:
   - Frontier detection algorithm
   - Goal planning and navigation
   - SLAM integration
   - Obstacle avoidance via laser scan
   - Goal history tracking

2. **navigation_tester.py** - Interactive testing tool for manual control

3. **nav_config.py** - Centralized configuration for all navigation parameters

4. **navigation_stack.launch.py** - Complete launch file that starts:
   - Gazebo with custom world
   - TurtleBot3 spawning
   - SLAM Toolbox
   - Navigation2 stack
   - Auto Navigation node

5. **AUTO_NAVIGATION_README.md** - Comprehensive documentation

6. **setup_navigation.sh** - Automated setup script
