cmake_minimum_required(VERSION 3.8)
project(project)

# Avoid CMake dev warning from ament_cmake_symlink_install about FILE(GLOB_RECURSE)
# following symlinks (CMP0009). This is safe and keeps builds clean.
if(POLICY CMP0009)
  cmake_policy(SET CMP0009 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# Install Python modules
ament_python_install_package(project)

# Install Python executables
install(PROGRAMS
  navigation_scripts/calibration/keyboard_controller.py
  navigation_scripts/calibration/coordinate_tracker.py
  navigation_scripts/navigation/waypoint_navigator.py
  project/yolo_detector.py
  project/human_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install ROS 2 node entrypoints without the .py suffix for convenience
install(PROGRAMS
  project/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detector
)

install(PROGRAMS
  project/human_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME human_tracker
)

# Install scripted path playback helper
install(PROGRAMS
  navigation_scripts/navigation/open_loop_waypoint_player.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME play_waypoints
)

# Install navigation scripts as packages
ament_python_install_package(navigation_scripts)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install world and model files
install(FILES
  turtle.sdf
  world.sdf
  DESTINATION share/${PROJECT_NAME}
)

# Install custom Gazebo models (e.g., TurtleBot3 wrapper with camera)
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
