"""
Navigation Configuration for YOLO Rescue Simulation
"""

# Robot parameters
ROBOT_NAME = "turtlebot3_burger"
ROBOT_FRAME = "base_link"
MAP_FRAME = "map"
ODOM_FRAME = "odom"

# Velocity parameters
MAX_LINEAR_VELOCITY = 0.3  # m/s
MAX_ANGULAR_VELOCITY = 1.0  # rad/s
EXPLORATION_SPEED = 0.2  # m/s
TURNING_SPEED = 0.5  # rad/s

# Navigation parameters
FRONTIER_THRESHOLD = 10  # minimum cells for frontier
MIN_FRONTIER_SIZE = 5
GOAL_TOLERANCE = 0.3  # meters
ANGLE_TOLERANCE = 0.1  # radians

# Map parameters
MAP_RESOLUTION = 0.05  # meters/cell
MAP_SIZE_X = 50.0  # meters
MAP_SIZE_Y = 50.0  # meters

# SLAM parameters
SLAM_MODE = "mapping"  # mapping or localization
SLAM_SOLVER = "solver_plugins::CeresSolver"
SLAM_OUTPUT_LOCATION = "/tmp/rescue_sim_map"

# Sensor parameters
LASER_SCAN_TOPIC = "/scan"
LASER_RANGE = 4.0  # meters
LASER_ANGLE_MIN = -3.14159
LASER_ANGLE_MAX = 3.14159

# Camera parameters
CAMERA_TOPIC = "/camera/image_raw"
CAMERA_RESOLUTION = (640, 480)

# Exploration strategy
EXPLORATION_STRATEGY = "frontier"  # frontier, spiral, boustrophedon, or greedy
ENABLE_OBSTACLE_AVOIDANCE = True
ENABLE_LOOP_CLOSURE = True

# Timeout values
NAVIGATION_TIMEOUT = 300.0  # seconds
GOAL_TIMEOUT = 60.0  # seconds
MAP_UPDATE_TIMEOUT = 5.0  # seconds

# ROS 2 parameters
NODE_NAMES = {
    "auto_navigation": "auto_navigation_node",
    "slam": "slam_toolbox_node",
    "nav2": "nav2_controller",
    "camera": "camera_node",
    "yolo": "yolo_detection_node",
}

# Topic names
TOPICS = {
    "cmd_vel": "/cmd_vel",
    "odom": "/odom",
    "map": "/map",
    "scan": "/scan",
    "camera_image": "/camera/image_raw",
    "camera_info": "/camera/camera_info",
    "detections": "/yolo/detections",
    "nav_status": "/nav_status",
    "goal_pose": "/goal_pose",
}

# Service names
SERVICES = {
    "get_map": "/map_server/get_map",
    "save_map": "/map_saver/save_map",
    "start_mapping": "/slam/start_mapping",
    "stop_mapping": "/slam/stop_mapping",
}

# Action names
ACTIONS = {
    "navigate_to_pose": "navigate_to_pose",
    "compute_path_to_pose": "compute_path_to_pose",
    "follow_path": "follow_path",
}

# Debug parameters
DEBUG_MODE = False
VERBOSE_LOGGING = True
VISUALIZE_FRONTIERS = True
SAVE_MAP_ON_EXIT = True

# YOLO detection parameters
YOLO_CONFIDENCE_THRESHOLD = 0.5
YOLO_NMS_THRESHOLD = 0.45
YOLO_MODEL_PATH = "/path/to/yolo/model.pt"
YOLO_CLASSES = {
    0: "person",
    1: "rescue_flag",
    2: "victim",
    3: "hazard",
}
