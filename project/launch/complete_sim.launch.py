#!/usr/bin/env python3
"""
Complete Rescue Simulation Launch File
Launches Gazebo, RViz, and Navigation Stack
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate complete launch description"""
    
    # Get package directories
    project_dir = get_package_share_directory('project')
    
    # Launch rescue_sim (Gazebo)
    rescue_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(project_dir, 'launch', 'rescue_sim.launch.py')
        ),
    )
    
    # RViz with configuration
    rviz_config_file = os.path.join(
        project_dir, 'config', 'rviz_config.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )
    
    # Auto Navigation Node
    auto_nav_node = Node(
        package='project',
        executable='auto_navigation.py',
        name='auto_navigation',
        output='screen',
    )
    
    # SLAM Toolbox for mapping
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver_type': 'SPARSE_NORMAL_CHOLESKY',
            'map_name': 'rescue_world_map',
            'mode': 'mapping',
        }],
    )
    
    return LaunchDescription([
        rescue_sim_launch,
        rviz_node,
        slam_node,
        auto_nav_node,
    ])
