from setuptools import find_packages, setup

package_name = 'project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='YOLO Rescue Sim',
    author_email='support@example.com',
    maintainer='YOLO Rescue Sim',
    maintainer_email='support@example.com',
    url='https://github.com/yourusername/YOLO-RescueSim',
    description='YOLO RescueSim - TurtleBot3 rescue simulation with autonomous navigation',
    long_description='Autonomous navigation and rescue simulation for TurtleBot3 using ROS 2, Gazebo, SLAM Toolbox, and Navigation2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_navigation = project.auto_navigation:main',
            'navigation_tester = project.navigation_tester:main',
            'yolo_detector = project.yolo_detector:main',
            'human_tracker = project.human_tracker:main',
            'record_waypoints = navigation_scripts.navigation.record_waypoints:main',
            'navigate_waypoints = navigation_scripts.navigation.waypoint_navigator:main',
            'play_waypoints = navigation_scripts.navigation.open_loop_waypoint_player:main',
        ],
    },
)
