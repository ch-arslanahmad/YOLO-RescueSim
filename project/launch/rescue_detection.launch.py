#!/usr/bin/env python3
"""
Complete Rescue Simulation Pipeline
Launches all components: robot, navigation, YOLO detection, and tracking
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os


def generate_launch_description():
    ld = LaunchDescription()
    
    # Get package directory
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    
    # YOLO Detector Node
    yolo_detector = Node(
        package='project',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[
            {'model_path': 'yolov8n.pt'},
            {'conf_threshold': 0.5},
            {'skip_frames': 2},
        ]
    )
    
    # Human Tracker Node (delayed start to ensure camera is ready)
    human_tracker = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='project',
                executable='human_tracker',
                name='human_tracker',
                output='screen',
                parameters=[
                    {'max_distance_threshold': 1.0},
                ]
            )
        ]
    )
    
    ld.add_action(yolo_detector)
    ld.add_action(human_tracker)
    
    return ld
