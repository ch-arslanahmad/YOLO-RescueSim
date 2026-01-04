#!/usr/bin/env python3
"""
Navigation Stack Launch File for YOLO Rescue Simulation
Uses nav2_minimal_tb3_sim for simulation with SLAM and Auto Navigation
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for navigation stack"""
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get the turtlebot3_gazebo package for simulation
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Get slam_toolbox package
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Launch actions
    launch_description_actions = [
        # Launch turtlebot3_world (includes Gazebo and TurtleBot3)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
            }.items(),
        ),
        
        # SLAM Toolbox (for mapping)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'solver_plugin': 'solver_plugins::CeresSolver',
                'ceres_linear_solver_type': 'SPARSE_NORMAL_CHOLESKY',
                'ceres_linear_solver_threads': 1,
                'ceres_threads': 1,
                'map_name': 'rescue_world_map',
                'map_file_name': '/tmp/turtlebot3_map',
                'mode': 'mapping',
            }],
        ),
        
        # Auto Navigation Node
        Node(
            package='project',
            executable='auto_navigation.py',
            name='auto_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_frame': 'map',
                'robot_frame': 'base_link',
                'frontier_threshold': 10,
                'exploration_speed': 0.2,
                'goal_tolerance': 0.5,
                'max_angular_vel': 1.0,
                'exploration_mode': 'frontier',
            }],
        ),
    ]
    
    return LaunchDescription(launch_description_actions)
