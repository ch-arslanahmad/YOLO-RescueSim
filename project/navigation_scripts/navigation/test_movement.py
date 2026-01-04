#!/usr/bin/env python3
"""
Test script to verify robot movement and odometry
Shows what's actually happening in the simulation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time


class MovementTester(Node):
    """Test if robot moves and odometry works"""
    
    def __init__(self):
        super().__init__('movement_tester')
        
        # Publisher for velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.position_x = 0.0
        self.position_y = 0.0
        self.odom_received = False
        self.odom_updates = 0
        
        self.get_logger().info('Movement Tester Started')
    
    def odom_callback(self, msg: Odometry):
        """Called whenever odometry is received"""
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.odom_received = True
        self.odom_updates += 1
        
        self.get_logger().info(
            f'[ODOM UPDATE #{self.odom_updates}] '
            f'Position: X={self.position_x:.4f}, Y={self.position_y:.4f}'
        )
    
    def test_movement(self):
        """Test if robot can move"""
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('TEST 1: Checking if odometry is being published')
        self.get_logger().info('='*70)
        
        # Wait for odometry
        self.get_logger().info('Waiting 3 seconds for odometry messages...')
        time.sleep(3)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        if self.odom_received:
            self.get_logger().info(f'Odometry IS being published! Received {self.odom_updates} updates')
            self.get_logger().info(f'   Current position: ({self.position_x:.4f}, {self.position_y:.4f})')
        else:
            self.get_logger().error('NO ODOMETRY DATA! Topic /odom is not publishing')
        
        # Test movement
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('TEST 2: Sending movement command (forward for 2 seconds)')
        self.get_logger().info('='*70)
        
        initial_x = self.position_x
        initial_y = self.position_y
        
        # Send forward command
        twist = Twist()
        twist.linear.x = 0.2  # 0.2 m/s forward
        
        start_time = time.time()
        while (time.time() - start_time) < 2.0:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Check if position changed
        self.get_logger().info('Checking if robot moved...')
        time.sleep(1)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        delta_x = self.position_x - initial_x
        delta_y = self.position_y - initial_y
        distance = (delta_x**2 + delta_y**2)**0.5
        
        self.get_logger().info(f'\nResults:')
        self.get_logger().info(f'  Initial position: ({initial_x:.4f}, {initial_y:.4f})')
        self.get_logger().info(f'  Final position:   ({self.position_x:.4f}, {self.position_y:.4f})')
        self.get_logger().info(f'  Distance moved:   {distance:.4f} meters')
        
        if distance > 0.01:
            self.get_logger().info(f'ROBOT MOVED! Distance: {distance:.4f}m')
            return True
        else:
            self.get_logger().error(f'ROBOT DID NOT MOVE (distance: {distance:.4f}m)')
            return False


def main(args=None):
    rclpy.init(args=args)
    tester = MovementTester()
    
    try:
        moved = tester.test_movement()
        
        if moved:
            print('\n' + '='*70)
            print('SUCCESS! Robot movement and odometry are working!')
            print('   You can now record waypoints with real position data')
            print('='*70)
        else:
            print('\n' + '='*70)
            print('ISSUE: Robot not moving or no odometry')
            print('   Check Gazebo bridge configuration')
            print('='*70)
    
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
