# Project Plan and Rationale: YOLO-RescueSim

## Why We Are Doing This
We are building a mobile rescue robot simulation using TurtleBot3 in ROS 2 and Gazebo to:
- Develop and test autonomous rescue strategies in a safe, repeatable environment
- Integrate computer vision (YOLO-Lite) for human and marker detection
- Prepare for future deployment on real robots

## What We Are Doing
- Creating a custom Gazebo world for rescue scenarios
- Launching TurtleBot3 (burger) in this world with full ROS 2 integration
Basic: Random walk, simple obstacle avoidance
Your requirement: Coverage path planning (visiting every free space systematically)
- Ensuring the robot has a working camera for perception tasks
- Bridging all necessary topics for control and sensor data
- Planning for YOLO-based perception and autonomous navigation

## Our Plan and Steps
1. Set up a custom ROS 2 launch file to:
   - Launch Gazebo with our custom world
   - Spawn TurtleBot3 with camera
   - Bridge all required topics
2. Verify camera and sensor topics are available in ROS 2
3. Develop a YOLO-Lite node to subscribe to camera images and perform detection
4. Integrate detection results with robot navigation and control
5. Expand simulation with more complex rescue scenarios and multi-robot support
6. Prepare for transition to real TurtleBot3 hardware

---
This file documents the project motivation, goals, and step-by-step plan for clarity and team alignment.
