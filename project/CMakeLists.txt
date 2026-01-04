cmake_minimum_required(VERSION 3.8)
project(project)

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
# Removed autonomous navigation executables

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

ament_package()
