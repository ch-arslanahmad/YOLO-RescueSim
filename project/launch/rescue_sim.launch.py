#!/usr/bin/env python3
#
# Custom ROS 2 launch file for YOLO Rescue Simulation
# Launches Gazebo with custom world.sdf and spawns TurtleBot3
#
# Usage:
#   export TURTLEBOT3_MODEL=burger
#   ros2 launch rescue_sim.launch.py
#
# Or from project root:
#   ros2 launch ./launch/rescue_sim.launch.py

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for custom rescue simulation."""
    
    # Print confirmation
    print("\n" + "="*60)
    print("🤖 YOLO-RescueSim Launch Script Loaded")
    print(f"   Robot Model: {os.environ.get('TURTLEBOT3_MODEL', 'burger')}")
    print(f"   World File: turtle.sdf")
    print("="*60 + "\n")
    
    # ========== CONFIGURATION ==========
    # Get TurtleBot3 model from environment
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # Paths to launch files and resources
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(turtlebot3_gazebo_dir, 'launch')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    
    # Path to custom world.sdf (absolute path from project directory)
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_sdf_path = '/home/arslan/Desktop/github/YOLO-RescueSim/project/turtle.sdf'
    
    # Launch configuration variables with defaults
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # ========== GAZEBO LAUNCH ACTIONS ==========
    
    # 1. Start Gazebo (server + GUI) with custom world
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v2 ', world_sdf_path],  # -r: run, -v2: verbose
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ========== ROBOT STATE PUBLISHER ==========
    
    # 3. Publish TF transforms and odometry (no simulation logic, just publication)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ========== TURTLEBOT3 SPAWNING & BRIDGING ==========
    
    # 4. Spawn TurtleBot3 into Gazebo using ros_gz_sim's 'create' service
    model_folder = f'turtlebot3_{TURTLEBOT3_MODEL}'
    urdf_path = os.path.join(
        turtlebot3_gazebo_dir,
        'models',
        model_folder,
        'model.sdf'
    )

    spawn_turtlebot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'  # Small offset to avoid initial collision with ground
        ],
        output='screen',
    )

    # 5. Bridge ROS 2 topics to Gazebo topics via ros_gz_bridge
    #    Uses YAML config file for flexible topic mappings
    project_dir = get_package_share_directory('project')
    bridge_params = os.path.join(project_dir, 'config', 'turtlebot3_burger_bridge.yaml')

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # 6. Bridge camera image topics (separate from generic bridge)
    #    Converts gz::/camera/image_raw to ROS 2 /camera/image_raw
    image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # ========== GAZEBO RESOURCE PATH ==========
    
    # 7. Append turtlebot3_gazebo models to Gazebo search path
    #    (allows Gazebo to find robot models and plugins)
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo_dir, 'models')
    )

    # ========== BUILD LAUNCH DESCRIPTION ==========
    
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    ))
    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial X position of robot'
    ))
    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial Y position of robot'
    ))

    # Add actions in dependency order
    ld.add_action(set_env_vars_resources)  # Set paths first
    ld.add_action(gazebo_cmd)               # Start Gazebo (server + GUI)
    ld.add_action(robot_state_publisher_cmd)  # Publish TF
    ld.add_action(spawn_turtlebot_cmd)      # Spawn robot model
    ld.add_action(bridge_cmd)               # Bridge ROS 2 ↔ Gazebo topics
    ld.add_action(image_bridge_cmd)         # Bridge camera images

    return ld
