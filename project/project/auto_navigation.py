#!/usr/bin/env python3
"""
Autonomous Navigation Node for YOLO Rescue Simulation
Implements frontier-based exploration with Nav2 integration
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler
import numpy as np
import math
from collections import deque
import time
from enum import Enum

from .nav_config import (
    ROBOT_FRAME, MAP_FRAME, GOAL_TOLERANCE,
    FRONTIER_THRESHOLD, MIN_FRONTIER_SIZE,
    EXPLORATION_SPEED, MAX_ANGULAR_VELOCITY
)


class ExplorationState(Enum):
    """Robot exploration states"""
    IDLE = 1
    EXPLORING = 2
    NAVIGATING = 3
    STUCK = 4
    COMPLETE = 5


class AutoNavigationNode(Node):
    """
    Autonomous navigation node for TurtleBot3
    Handles SLAM-based mapping and frontier exploration
    """

    def __init__(self):
        super().__init__('auto_navigation_node')
        
        # Declare parameters
        self.declare_parameter('map_frame', MAP_FRAME)
        self.declare_parameter('robot_frame', ROBOT_FRAME)
        self.declare_parameter('frontier_threshold', FRONTIER_THRESHOLD)
        self.declare_parameter('exploration_speed', EXPLORATION_SPEED)
        self.declare_parameter('goal_tolerance', GOAL_TOLERANCE)
        self.declare_parameter('max_angular_vel', MAX_ANGULAR_VELOCITY)
        self.declare_parameter('use_sim_time', True)
        
        # Get parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.frontier_threshold = self.get_parameter('frontier_threshold').value
        self.exploration_speed = self.get_parameter('exploration_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profile for map subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        
        # State variables
        self.current_map = None
        self.current_odom = None
        self.current_scan = None
        self.frontiers = []
        self.explored_goals = set()
        self.state = ExplorationState.IDLE
        self.current_goal = None
        self.stuck_count = 0
        self.last_goal_time = time.time()
        
        self.get_logger().info('Auto Navigation Node initialized')
        self.get_logger().info(f'Using map frame: {self.map_frame}')
        self.get_logger().info(f'Using robot frame: {self.robot_frame}')
        
        # Main exploration loop
        self.create_timer(2.0, self.exploration_loop)

    def map_callback(self, msg):
        """Process occupancy grid map updates"""
        self.current_map = msg
        if self.state == ExplorationState.EXPLORING:
            self.detect_frontiers()

    def odom_callback(self, msg):
        """Process odometry updates"""
        self.current_odom = msg

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.current_scan = msg

    def detect_frontiers(self):
        """
        Detect frontier cells (boundary between known and unknown space)
        Returns list of frontier clusters with centroids
        """
        if self.current_map is None:
            return
        
        data = np.array(self.current_map.data, dtype=np.int8)
        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # Frontier cells are free cells adjacent to unknown cells
        frontier_map = np.zeros_like(data, dtype=bool)
        
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                idx = i * width + j
                # Check if current cell is free (< 50)
                if 0 <= data[idx] < 50:
                    # Check 8-neighborhood for unknown cells (value = -1 or 255)
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            if di == 0 and dj == 0:
                                continue
                            ni = i + di
                            nj = j + dj
                            if 0 <= ni < height and 0 <= nj < width:
                                nidx = ni * width + nj
                                if data[nidx] == -1 or data[nidx] == 255:
                                    frontier_map[idx] = True
                                    break
        
        # Cluster frontiers using BFS
        visited = np.zeros_like(frontier_map, dtype=bool)
        self.frontiers = []
        
        for i in range(height):
            for j in range(width):
                idx = i * width + j
                if frontier_map[idx] and not visited[idx]:
                    cluster = self._bfs_cluster(frontier_map, visited, idx, width, height)
                    
                    if len(cluster) >= MIN_FRONTIER_SIZE:
                        # Calculate centroid
                        centroid_i = np.mean([idx // width for idx in cluster])
                        centroid_j = np.mean([idx % width for idx in cluster])
                        
                        # Convert to world coordinates
                        world_x = origin_x + centroid_j * resolution
                        world_y = origin_y + centroid_i * resolution
                        
                        self.frontiers.append({
                            'x': world_x,
                            'y': world_y,
                            'size': len(cluster),
                            'cells': cluster
                        })
        
        if self.frontiers:
            self.get_logger().debug(f'Detected {len(self.frontiers)} frontiers')
    
    def _bfs_cluster(self, frontier_map, visited, start_idx, width, height):
        """BFS to cluster connected frontier cells"""
        cluster = []
        queue = deque([start_idx])
        visited[start_idx] = True
        
        while queue:
            idx = queue.popleft()
            cluster.append(idx)
            
            i = idx // width
            j = idx % width
            
            # Check 8-neighborhood
            for di in [-1, 0, 1]:
                for dj in [-1, 0, 1]:
                    if di == 0 and dj == 0:
                        continue
                    ni = i + di
                    nj = j + dj
                    if 0 <= ni < height and 0 <= nj < width:
                        nidx = ni * width + nj
                        if frontier_map[nidx] and not visited[nidx]:
                            visited[nidx] = True
                            queue.append(nidx)
        
        return cluster

    
    def exploration_loop(self):
        """Main exploration loop"""
        if self.state == ExplorationState.IDLE:
            self.get_logger().info('Starting exploration...')
            self.state = ExplorationState.EXPLORING
        
        elif self.state == ExplorationState.EXPLORING:
            self.select_and_navigate_frontier()
        
        elif self.state == ExplorationState.STUCK:
            self.handle_stuck_state()
        
        elif self.state == ExplorationState.COMPLETE:
            self.get_logger().info('Exploration complete!')
    
    def select_and_navigate_frontier(self):
        """Select best frontier and navigate to it"""
        if not self.frontiers:
            self.get_logger().warn('No frontiers detected. Exploration may be complete.')
            self.state = ExplorationState.COMPLETE
            return
        
        # Get current robot position
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return
        
        # Select closest unexplored frontier
        best_frontier = None
        best_distance = float('inf')
        
        for frontier in self.frontiers:
            goal_key = (round(frontier['x'], 1), round(frontier['y'], 1))
            if goal_key in self.explored_goals:
                continue
            
            dx = frontier['x'] - robot_x
            dy = frontier['y'] - robot_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < best_distance:
                best_distance = distance
                best_frontier = frontier
        
        if best_frontier:
            self.navigate_to_frontier(best_frontier)
        else:
            self.get_logger().warn('All frontiers already explored')
            self.state = ExplorationState.COMPLETE
    
    def navigate_to_frontier(self, frontier):
        """Send navigation goal to frontier"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set goal position
        goal_msg.pose.pose.position.x = frontier['x']
        goal_msg.pose.pose.position.y = frontier['y']
        goal_msg.pose.pose.position.z = 0.0
        
        # Set goal orientation
        q = quaternion_from_euler(0, 0, 0)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.current_goal = frontier
        self.state = ExplorationState.NAVIGATING
        self.last_goal_time = time.time()
        
        self.get_logger().info(
            f'Navigating to frontier at ({frontier["x"]:.2f}, {frontier["y"]:.2f})'
        )
        
        # Send goal
        try:
            self.nav_to_pose_client.wait_for_server(timeout_sec=2.0)
            future = self.nav_to_pose_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')
            self.state = ExplorationState.EXPLORING
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected by navigation stack')
                self.state = ExplorationState.EXPLORING
                return
            
            self.get_logger().info('Goal accepted by navigation stack')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.goal_result_callback)
        except Exception as e:
            self.get_logger().error(f'Goal response error: {e}')
            self.state = ExplorationState.EXPLORING
    
    def goal_result_callback(self, future):
        """Handle goal result"""
        try:
            result = future.result()
            if result:
                self.get_logger().info('Goal reached successfully!')
                # Mark frontier as explored
                if self.current_goal:
                    goal_key = (
                        round(self.current_goal['x'], 1),
                        round(self.current_goal['y'], 1)
                    )
                    self.explored_goals.add(goal_key)
                self.stuck_count = 0
            else:
                self.get_logger().warn('Navigation failed')
                self.stuck_count += 1
        except Exception as e:
            self.get_logger().debug(f'Goal result exception: {e}')
            self.stuck_count += 1
        
        # Check if stuck
        if self.stuck_count >= 3:
            self.state = ExplorationState.STUCK
        else:
            self.state = ExplorationState.EXPLORING
    
    def handle_stuck_state(self):
        """Handle when robot gets stuck"""
        self.get_logger().warn('Robot appears stuck. Attempting recovery...')
        self.stuck_count = 0
        self.state = ExplorationState.EXPLORING


class ManualNavigationNode(Node):
    """Manual control node for testing"""

    def __init__(self):
        super().__init__('manual_navigation_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Manual Navigation Node ready')

    def move_forward(self, speed=0.2):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)

    def move_backward(self, speed=0.2):
        """Move robot backward"""
        twist = Twist()
        twist.linear.x = -speed
        self.cmd_vel_pub.publish(twist)

    def turn_left(self, angular_speed=0.5):
        """Turn robot left"""
        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)

    def turn_right(self, angular_speed=0.5):
        """Turn robot right"""
        twist = Twist()
        twist.angular.z = -angular_speed
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        """Stop robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot stopped')


def main(args=None):
    """Main entry point for autonomous navigation"""
    rclpy.init(args=args)
    
    auto_nav_node = AutoNavigationNode()
    
    try:
        rclpy.spin(auto_nav_node)
    except KeyboardInterrupt:
        auto_nav_node.get_logger().info('Shutting down...')
    finally:
        auto_nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

