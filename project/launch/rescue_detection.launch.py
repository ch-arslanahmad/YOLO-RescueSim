#!/usr/bin/env python3
"""
Complete Rescue Simulation Pipeline
Launches all components: robot, navigation, YOLO detection, and tracking
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    
    # Make the repo root available to nodes (launcher sets this too; keep launch-file usable standalone).
    ld.add_action(SetEnvironmentVariable('YOLO_RESCUESIM_ROOT', os.getcwd()))

    # Limit CPU thread pools (torch/blas/opencv) so the detector can't peg all cores.
    # Users can override these in their shell if desired.
    ld.add_action(SetEnvironmentVariable('OMP_NUM_THREADS', os.environ.get('OMP_NUM_THREADS', '2')))
    ld.add_action(SetEnvironmentVariable('MKL_NUM_THREADS', os.environ.get('MKL_NUM_THREADS', '2')))
    ld.add_action(SetEnvironmentVariable('OPENBLAS_NUM_THREADS', os.environ.get('OPENBLAS_NUM_THREADS', '2')))
    ld.add_action(SetEnvironmentVariable('NUMEXPR_NUM_THREADS', os.environ.get('NUMEXPR_NUM_THREADS', '2')))

    # Prefer a model path that works regardless of current working directory.
    # If the model is installed with the package, use that.
    model_path = 'yolov8n.pt'
    try:
        share_dir = get_package_share_directory('project')
        candidate = os.path.join(share_dir, 'yolov8n.pt')
        if os.path.exists(candidate):
            model_path = candidate
    except Exception:
        pass
    
    # YOLO Detector Node
    yolo_detector = Node(
        package='project',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[
            {'model_path': model_path},
            {'conf_threshold': 0.5},
            # Smoother output: process more often, but keep a hard cap via max_inference_hz.
            {'skip_frames': 1},
            # Performance/safety: avoid accidental multi-launch melting CPU
            {'singleton_lock': True},
            {'singleton_lock_path': '/tmp/yolo_rescuesim_yolo_detector.lock'},
            # Optional cap on inference rate (0.0 disables). Keeps CPU predictable.
            {'max_inference_hz': 5.0},
            {'torch_num_threads': 2},
            {'torch_num_interop_threads': 1},
            {'opencv_num_threads': 1},
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
                    # Performance/safety: avoid accidental multi-launch
                    {'singleton_lock': True},
                    {'singleton_lock_path': '/tmp/yolo_rescuesim_human_tracker.lock'},
                    # Reduce log spam (still logs periodically)
                    {'log_every_n_messages': 30},
                ]
            )
        ]
    )
    
    ld.add_action(yolo_detector)
    ld.add_action(human_tracker)
    
    return ld
