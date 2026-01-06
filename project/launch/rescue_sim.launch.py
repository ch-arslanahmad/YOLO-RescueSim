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
import csv
import tempfile
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_human_poses_csv_path() -> str:
    # Keep this relative so it works when the launch file is executed from the
    # installed location, but the user is running from the repo root.
    # NOTE: This is kept for backward compatibility, but the launch argument
    # default is intentionally empty so the world generation is opt-in.
    return os.path.join('waypoints', 'human_poses.csv')


def _apply_human_poses_from_csv(context, *args, **kwargs):
    csv_path = LaunchConfiguration('human_poses_csv').perform(context).strip()
    if not csv_path:
        return []

    template_path = LaunchConfiguration('world_template').perform(context).strip()
    if not template_path:
        template_path = LaunchConfiguration('world').perform(context).strip()
    if not template_path:
        raise RuntimeError('No world template available (world_template and world are empty)')

    cache_generated_world_raw = LaunchConfiguration('cache_generated_world').perform(context).strip().lower()
    cache_generated_world = cache_generated_world_raw in ('1', 'true', 'yes', 'on')

    generated_world_out = LaunchConfiguration('generated_world_out').perform(context).strip()
    if generated_world_out:
        out_candidate = Path(generated_world_out)
        if not out_candidate.is_absolute():
            out_candidate = Path.cwd() / out_candidate
        out_path = str(out_candidate)
    else:
        out_path = os.path.join(tempfile.gettempdir(), 'turtle_generated.sdf')
    csv_candidate = Path(csv_path)
    if not csv_candidate.is_absolute():
        candidates = [
            (Path.cwd() / csv_candidate),
            (Path(template_path).resolve().parent.parent / csv_candidate),
        ]
        resolved = next((p for p in candidates if p.exists()), None)
        if resolved is None:
            raise RuntimeError(
                f'human_poses_csv not found: {csv_path}. Tried: ' + ', '.join(str(p) for p in candidates)
            )
        csv_path = str(resolved)

    print(f"[rescue_sim] Human poses CSV: {csv_path}")
    print(f"[rescue_sim] World template: {template_path}")
    print(f"[rescue_sim] Generated world: {out_path}")

    if cache_generated_world and os.path.exists(out_path):
        try:
            out_mtime = os.path.getmtime(out_path)
            csv_mtime = os.path.getmtime(csv_path)
            template_mtime = os.path.getmtime(template_path)
            if out_mtime >= max(csv_mtime, template_mtime):
                print('[rescue_sim] Using cached generated world (up-to-date).')
                return [SetLaunchConfiguration('world', out_path)]
        except OSError:
            # If any stat fails, fall back to regenerating.
            pass

    tree = ET.parse(template_path)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        raise RuntimeError('No <world> element found in template SDF')

    # Build maps of existing entities by name.
    # Newer templates in this repo use world-level <include> with <name>.
    includes_by_name = {}
    for include in world.findall('include'):
        name_el = include.find('name')
        if name_el is not None and (name_el.text or '').strip():
            includes_by_name[name_el.text.strip()] = include

    models_by_name = {}
    for model in world.findall('model'):
        model_name = model.get('name')
        if model_name:
            models_by_name[model_name] = model

    prototype_include = includes_by_name.get('human_1')
    prototype_model = models_by_name.get('human_1')

    def ensure_human_entity(name: str):
        existing_include = includes_by_name.get(name)
        if existing_include is not None:
            return existing_include
        existing_model = models_by_name.get(name)
        if existing_model is not None:
            return existing_model

        if prototype_include is not None:
            new_include = deepcopy(prototype_include)
            name_el = new_include.find('name')
            if name_el is None:
                name_el = ET.SubElement(new_include, 'name')
            name_el.text = name
            pose_el = new_include.find('pose')
            if pose_el is None:
                pose_el = ET.SubElement(new_include, 'pose')
            pose_el.text = '0 0 0 0 0 0'
            world.append(new_include)
            includes_by_name[name] = new_include
            return new_include

        if prototype_model is not None:
            new_model = deepcopy(prototype_model)
            new_model.set('name', name)
            pose_el = new_model.find('pose')
            if pose_el is None:
                pose_el = ET.SubElement(new_model, 'pose')
            pose_el.text = '0 0 0 0 0 0'
            world.append(new_model)
            models_by_name[name] = new_model
            return new_model

        # Fallback: create a minimal world-level include for the local walking person
        new_include = ET.Element('include')
        uri_el = ET.SubElement(new_include, 'uri')
        uri_el.text = 'model://walking_person_small'
        name_el = ET.SubElement(new_include, 'name')
        name_el.text = name
        pose_el = ET.SubElement(new_include, 'pose')
        pose_el.text = '0 0 0 0 0 0'
        static_el = ET.SubElement(new_include, 'static')
        static_el.text = 'true'
        world.append(new_include)
        includes_by_name[name] = new_include
        return new_include

    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(
            (row for row in f if row.strip() and not row.lstrip().startswith('#'))
        )

        # If the file includes a header, DictReader will use it; otherwise user must supply headers.
        # Expected headers: name,x,y,yaw
        for row in reader:
            name = (row.get('name') or '').strip()
            if not name:
                continue

            x = (row.get('x') or '').strip()
            y = (row.get('y') or '').strip()
            yaw = (row.get('yaw') or '').strip()
            if not x or not y or not yaw:
                raise RuntimeError(f'Invalid row for {name}: expected columns name,x,y,yaw')

            target = ensure_human_entity(name)
            pose_el = target.find('pose')
            if pose_el is None:
                pose_el = ET.SubElement(target, 'pose')
            pose_el.text = f'{x} {y} 0 0 0 {yaw}'

    tree.write(out_path)
    return [SetLaunchConfiguration('world', out_path)]


def generate_launch_description():
    """Generate launch description for custom rescue simulation."""
    
    # Print confirmation
    print("\n" + "="*60)
    print("YOLO-RescueSim Launch Script Loaded")
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
    project_dir = get_package_share_directory('project')

    # Local models directory (repo working tree) and installed models directory.
    # When users run `ros2 launch ...` from the repo root, prefer `./project/models`
    # so the sim works without rebuilding and can run offline.
    cwd_models_path = str((Path.cwd() / 'project' / 'models').resolve())
    source_models_path = str((Path(__file__).resolve().parents[1] / 'models'))
    installed_models_path = os.path.join(project_dir, 'models')

    # World file
    # Prefer the repo working tree `./project/turtle.sdf` when available.
    # This makes it easier to iterate on the world without rebuilding.
    installed_world_sdf_path = os.path.join(project_dir, 'turtle.sdf')
    source_world_sdf_path = str((Path(__file__).resolve().parents[1] / 'turtle.sdf'))
    cwd_world_sdf_path = str((Path.cwd() / 'project' / 'turtle.sdf').resolve())

    if os.path.exists(cwd_world_sdf_path):
        default_world_sdf_path = cwd_world_sdf_path
    elif os.path.exists(source_world_sdf_path):
        default_world_sdf_path = source_world_sdf_path
    else:
        default_world_sdf_path = installed_world_sdf_path
    
    # Launch configuration variables with defaults
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_sdf_path = LaunchConfiguration('world', default=default_world_sdf_path)
    world_template_path = LaunchConfiguration('world_template', default='')
    human_poses_csv_path = LaunchConfiguration('human_poses_csv', default='')
    generated_world_out = LaunchConfiguration('generated_world_out', default='')
    cache_generated_world = LaunchConfiguration('cache_generated_world', default='false')
    # Default spawn pose aligned with the first waypoint in waypoints/waypoints_arslan_recorded_2026-01-04.csv
    # (x=-2.0001, y=-0.5000, yaw=0.0)
    x_pose = LaunchConfiguration('x_pose', default='-2.0001')
    y_pose = LaunchConfiguration('y_pose', default='-0.5000')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

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
    
    # 4. Spawn TurtleBot3 using ros_gz_sim
    # Prefer camera-equipped model variants when available (e.g., turtlebot3_burger_cam).
    model_folder = f'turtlebot3_{TURTLEBOT3_MODEL}'
    model_folder_cam = f'{model_folder}_cam'

    upstream_model_sdf_path = os.path.join(turtlebot3_gazebo_dir, 'models', model_folder, 'model.sdf')
    upstream_camera_model_sdf_path = os.path.join(
        turtlebot3_gazebo_dir, 'models', model_folder_cam, 'model.sdf'
    )

    # Prefer a workspace-provided wrapper model with a camera sensor.
    source_wrapper_sdf_path = str(
        (Path(__file__).resolve().parents[1] / 'models' / 'turtlebot3_burger_with_camera' / 'model.sdf')
    )
    installed_wrapper_sdf_path = os.path.join(
        project_dir, 'models', 'turtlebot3_burger_with_camera', 'model.sdf'
    )

    if os.path.exists(upstream_camera_model_sdf_path):
        urdf_path = upstream_camera_model_sdf_path
    elif os.path.exists(source_wrapper_sdf_path):
        urdf_path = source_wrapper_sdf_path
    elif os.path.exists(installed_wrapper_sdf_path):
        urdf_path = installed_wrapper_sdf_path
    else:
        urdf_path = upstream_model_sdf_path

    spawn_turtlebot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.2',
            '-Y', yaw_pose,
        ],
        output='screen',
    )

    # 5. Bridge ROS 2 topics to Gazebo topics via ros_gz_bridge
    # Direction symbol:
    #   ] == ROS -> Gazebo
    #   [ == Gazebo -> ROS
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    # 6. Bridge camera image topics (separate from generic bridge)
    # Gazebo Transport image topic for TurtleBot3 camera.
    gz_camera_image = '/camera/image_raw'
    image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[gz_camera_image],
        output='screen',
    )

    # ========== GAZEBO RESOURCE PATH ==========
    
    # 7. Append models to Gazebo search path
    #    Order matters: put workspace models first so local overrides are used.
    set_env_vars_project_models_cwd = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        cwd_models_path
    )
    set_env_vars_project_models_source = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        source_models_path
    )
    set_env_vars_project_models_installed = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        installed_models_path
    )
    set_env_vars_turtlebot_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo_dir, 'models')
    )

    # ========== BUILD LAUNCH DESCRIPTION ==========
    
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'world', default_value=default_world_sdf_path,
        description='Path to the Gazebo world SDF (turtle.sdf)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'world_template', default_value='',
        description='Template world SDF used to generate a launch-time world. If empty, uses the "world" argument.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'human_poses_csv', default_value='',
        description='CSV file with columns name,x,y,yaw. If set, generates a temporary world with updated human poses.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'generated_world_out', default_value='',
        description='Optional output path for the generated world SDF. If empty, uses /tmp/turtle_generated.sdf.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'cache_generated_world', default_value='false',
        description='If true and generated_world_out exists and is newer than CSV/template, reuse it instead of regenerating.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    ))
    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='-2.0001',
        description='Initial X position of robot'
    ))
    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='-0.5000',
        description='Initial Y position of robot'
    ))
    ld.add_action(DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Initial yaw (radians) of robot'
    ))

    # Add actions in dependency order
    # Set paths first (workspace models before installed ones)
    if os.path.isdir(cwd_models_path):
        ld.add_action(set_env_vars_project_models_cwd)
    ld.add_action(set_env_vars_project_models_source)
    ld.add_action(set_env_vars_project_models_installed)
    ld.add_action(set_env_vars_turtlebot_models)
    ld.add_action(OpaqueFunction(function=_apply_human_poses_from_csv))
    ld.add_action(gazebo_cmd)               # Start Gazebo (server + GUI)
    ld.add_action(robot_state_publisher_cmd)  # Publish TF
    ld.add_action(spawn_turtlebot_cmd)      # Spawn robot model
    ld.add_action(bridge_cmd)               # Bridge ROS 2 ↔ Gazebo topics
    ld.add_action(image_bridge_cmd)         # Bridge camera images

    return ld
