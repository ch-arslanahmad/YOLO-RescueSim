#!/usr/bin/env python3
"""
Manual Waypoint Recorder - AUTO DEMO
Shows how the manual recorder works
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv
import math
import time
from datetime import datetime
import os


class ManualWaypointRecorderDemo(Node):
    """Demo version - automatically records waypoints while moving"""
    
    def __init__(self):
        super().__init__('manual_waypoint_recorder_demo')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Use TF2 to get robot pose
        from tf2_ros import TransformListener, Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.waypoints = []
        self.waypoint_count = 0
        
        self.output_dir = os.path.join(
            os.path.dirname(__file__), '..', 'waypoints'
        )
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info('Manual Waypoint Recorder DEMO')
    
    def update_pose_from_tf(self):
        """Get robot pose from TF"""
        try:
            for frame_pair in [('map', 'base_link'), ('world', 'base_link'), ('odom', 'base_link')]:
                try:
                    trans = self.tf_buffer.lookup_transform(frame_pair[0], frame_pair[1], rclpy.time.Time())
                    self.current_x = trans.transform.translation.x
                    self.current_y = trans.transform.translation.y
                    quat = trans.transform.rotation
                    self.current_theta = self._quaternion_to_yaw(quat)
                    return True
                except:
                    continue
            return False
        except:
            return False
    
    def _quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    
    def record_waypoint(self, label=""):
        """Record current position"""
        if not self.update_pose_from_tf():
            self.get_logger().error('Cannot get pose from TF')
            return False
        
        self.waypoint_count += 1
        waypoint = {
            'Waypoint_ID': self.waypoint_count,
            'X_Position': round(self.current_x, 4),
            'Y_Position': round(self.current_y, 4),
            'Theta_Radians': round(self.current_theta, 4),
            'Label': label
        }
        self.waypoints.append(waypoint)
        
        self.get_logger().info(f'WP {self.waypoint_count}: '
                              f'({waypoint["X_Position"]:.4f}, {waypoint["Y_Position"]:.4f}) '
                              f'θ={waypoint["Theta_Radians"]:.3f}')
        return True
    
    def export_waypoints(self):
        """Export to CSV"""
        if not self.waypoints:
            self.get_logger().error('No waypoints to export')
            return None
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"waypoints_{timestamp}.csv"
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            with open(filepath, 'w', newline='') as f:
                fieldnames = ['Waypoint_ID', 'X_Position', 'Y_Position', 'Theta_Radians']
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for wp in self.waypoints:
                    writer.writerow({k: wp[k] for k in fieldnames})
            
            self.get_logger().info(f'Exported {len(self.waypoints)} waypoints to: {filepath}')
            return filepath
        except Exception as e:
            self.get_logger().error(f'Export failed: {e}')
            return None
    
    def move_and_record_demo(self):
        """Demo: Move and record waypoints"""
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('MANUAL WAYPOINT RECORDER DEMO')
        self.get_logger().info('='*70)
        self.get_logger().info('')
        self.get_logger().info('In actual use:')
        self.get_logger().info('  1. You drag robot around in Gazebo GUI')
        self.get_logger().info('  2. When positioned, you press R to record')
        self.get_logger().info('  3. Waypoints saved with exact coordinates')
        self.get_logger().info('')
        self.get_logger().info('This demo will:')
        self.get_logger().info('  - Move robot forward')
        self.get_logger().info('  - Record waypoint')
        self.get_logger().info('  - Turn and move again')
        self.get_logger().info('  - Record more waypoints')
        self.get_logger().info('')
        
        # Wait for TF to be available
        self.get_logger().info('Waiting for TF data...')
        time.sleep(3)
        for i in range(15):
            self.update_pose_from_tf()
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.get_logger().info(f'Current pose: X={self.current_x:.3f}, Y={self.current_y:.3f}\n')
        
        # Record start position
        self.record_waypoint('Start Position')
        time.sleep(1)
        
        # Move forward
        self.get_logger().info('Moving forward...')
        twist = Twist()
        twist.linear.x = 0.2
        start = time.time()
        while (time.time() - start) < 3:
            self.cmd_vel_pub.publish(twist)
            self.update_pose_from_tf()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
        time.sleep(1)
        self.record_waypoint('After Moving Forward')
        
        # Turn
        self.get_logger().info('Turning left...')
        twist.angular.z = 0.5
        start = time.time()
        while (time.time() - start) < 2:
            self.cmd_vel_pub.publish(twist)
            self.update_pose_from_tf()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        time.sleep(1)
        self.record_waypoint('After Turning Left')
        
        # Move forward again
        self.get_logger().info('Moving forward again...')
        twist.linear.x = 0.2
        start = time.time()
        while (time.time() - start) < 3:
            self.cmd_vel_pub.publish(twist)
            self.update_pose_from_tf()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
        time.sleep(1)
        self.record_waypoint('Final Position')
        
        # Show results
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('RECORDED WAYPOINTS:')
        self.get_logger().info('='*70)
        for wp in self.waypoints:
            self.get_logger().info(f'  {wp["Waypoint_ID"]}: '
                                  f'({wp["X_Position"]:.4f}, {wp["Y_Position"]:.4f}) '
                                  f'Label: {wp["Label"]}')
        
        # Export
        filepath = self.export_waypoints()
        
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('DEMO COMPLETE!')
        self.get_logger().info('')
        self.get_logger().info('This shows how the manual recorder works:')
        self.get_logger().info('  - Robot position is captured from Gazebo transforms')
        self.get_logger().info('  - Each waypoint stores exact X, Y, and angle')
        self.get_logger().info('  - Waypoints exported to CSV for autonomous navigation')
        self.get_logger().info('')
        self.get_logger().info('To use interactively:')
        self.get_logger().info('  1. Drag robot in Gazebo GUI')
        self.get_logger().info('  2. Run: python3 -c \"from navigation_scripts.navigation.manual_record_waypoints import main; main()\"')
        self.get_logger().info('  3. Press R to record each waypoint')
        self.get_logger().info('  4. Press E to export CSV')
        self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    demo = ManualWaypointRecorderDemo()
    
    try:
        demo.move_and_record_demo()
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
