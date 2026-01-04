#!/usr/bin/env python3
"""
Simple Waypoint Recorder
Uses native ROS 2 navigation to move and records waypoints automatically
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import csv
import math
import time
from datetime import datetime
import os


class SimpleWaypointRecorder(Node):
    """Record waypoints while navigating with native ROS 2"""
    
    def __init__(self):
        super().__init__('simple_waypoint_recorder')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Try odometry first, then fall back to TF
        self.use_tf = False
        self.odom_sub = None
        try:
            self.odom_sub = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10)
        except:
            self.get_logger().warn('Odometry topic not found, using TF instead')
            self.use_tf = True
        
        if self.use_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Recording
        self.waypoints = []
        self.waypoint_count = 0
        
        # Output directory
        self.output_dir = os.path.join(
            os.path.dirname(__file__), '..', 'waypoints'
        )
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info('Simple Waypoint Recorder Ready')
        self.get_logger().info(f'Output directory: {self.output_dir}')
    
    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry"""
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        self.current_theta = self._quaternion_to_yaw(pose.orientation)
    
    def update_pose_from_tf(self):
        """Update robot position from TF"""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.current_x = trans.transform.translation.x
            self.current_y = trans.transform.translation.y
            quat = trans.transform.rotation
            self.current_theta = self._quaternion_to_yaw(quat)
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
    
    def update_pose(self):
        """Update pose from either odometry or TF"""
        if self.use_tf:
            self.update_pose_from_tf()
        rclpy.spin_once(self, timeout_sec=0.01)
    
    def _quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return yaw
    
    def move_robot(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        """Send velocity command for specified duration"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            self.update_pose()
            time.sleep(0.05)
        
        # Stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def record_waypoint(self, label=""):
        """Record current position as waypoint"""
        self.waypoint_count += 1
        waypoint = {
            'Waypoint_ID': self.waypoint_count,
            'X_Position': round(self.current_x, 4),
            'Y_Position': round(self.current_y, 4),
            'Theta_Radians': round(self.current_theta, 4),
            'Label': label
        }
        self.waypoints.append(waypoint)
        
        self.get_logger().info(f'Waypoint {self.waypoint_count}: '
                              f'({waypoint["X_Position"]}, {waypoint["Y_Position"]}) '
                              f'θ={waypoint["Theta_Radians"]:.3f}')
    
    def print_waypoints(self):
        """Display all recorded waypoints"""
        if not self.waypoints:
            self.get_logger().info('No waypoints recorded yet')
            return
        
        self.get_logger().info('='*70)
        self.get_logger().info('Recorded Waypoints:')
        self.get_logger().info('='*70)
        
        for wp in self.waypoints:
            label = f' ({wp["Label"]})' if wp["Label"] else ''
            self.get_logger().info(
                f'  {wp["Waypoint_ID"]} | '
                f'X={wp["X_Position"]:8.4f} Y={wp["Y_Position"]:8.4f} '
                f'θ={wp["Theta_Radians"]:8.4f}{label}'
            )
        
        self.get_logger().info('='*70)
    
    def export_waypoints(self, filename=None):
        """Export waypoints to CSV"""
        if not self.waypoints:
            self.get_logger().error('No waypoints to export!')
            return None
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"waypoints_{timestamp}.csv"
        
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            with open(filepath, 'w', newline='') as f:
                fieldnames = ['Waypoint_ID', 'X_Position', 'Y_Position', 'Theta_Radians']
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                
                writer.writeheader()
                for wp in self.waypoints:
                    writer.writerow({
                        'Waypoint_ID': wp['Waypoint_ID'],
                        'X_Position': wp['X_Position'],
                        'Y_Position': wp['Y_Position'],
                        'Theta_Radians': wp['Theta_Radians']
                    })
            
            self.get_logger().info(f'Exported {len(self.waypoints)} waypoints to: {filepath}')
            return filepath
        except Exception as e:
            self.get_logger().error(f'Error exporting waypoints: {e}')
            return None
    
    def demo_navigation(self):
        """Demo: Move robot in a simple pattern and record waypoints"""
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('Starting DEMO: Recording waypoints while moving')
        self.get_logger().info('='*70)
        self.get_logger().info('')
        
        # Wait for odometry to be initialized
        self.get_logger().info('Waiting for pose data...')
        time.sleep(2)
        for i in range(20):
            self.update_pose()
            time.sleep(0.2)
        
        self.get_logger().info(f'Current position: X={self.current_x:.3f}, Y={self.current_y:.3f}')
        
        # Initial position
        time.sleep(1)
        self.record_waypoint("Start")
        
        # Move forward
        self.get_logger().info('Moving forward...')
        self.move_robot(linear_x=0.2, duration=3.0)
        time.sleep(0.5)
        self.record_waypoint("After forward")
        
        # Turn left
        self.get_logger().info('Turning left...')
        self.move_robot(angular_z=0.5, duration=2.0)
        time.sleep(0.5)
        self.record_waypoint("After left turn")
        
        # Move forward again
        self.get_logger().info('Moving forward again...')
        self.move_robot(linear_x=0.2, duration=3.0)
        time.sleep(0.5)
        self.record_waypoint("After second forward")
        
        # Turn right
        self.get_logger().info('Turning right...')
        self.move_robot(angular_z=-0.5, duration=2.0)
        time.sleep(0.5)
        self.record_waypoint("After right turn")
        
        # Move forward to end
        self.get_logger().info('Moving to final position...')
        self.move_robot(linear_x=0.2, duration=3.0)
        time.sleep(0.5)
        self.record_waypoint("End")
        
        self.get_logger().info('')
        self.get_logger().info('Demo complete!')
        
        # Show waypoints
        self.print_waypoints()
        
        # Export
        filepath = self.export_waypoints()
        
        self.get_logger().info('')
        self.get_logger().info('All waypoints saved! You can now navigate using these points.')
        
        return filepath


def main(args=None):
    rclpy.init(args=args)
    recorder = SimpleWaypointRecorder()
    
    try:
        # Run demo
        recorder.demo_navigation()
    except Exception as e:
        recorder.get_logger().error(f'Error: {e}')
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
