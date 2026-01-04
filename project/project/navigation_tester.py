#!/usr/bin/env python3
"""
Navigation Helper - Utilities for testing and controlling auto-navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import select
import time


class NavigationTester(Node):
    """Test and debug navigation functionality"""
    
    def __init__(self):
        super().__init__('navigation_tester')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_sub = self.create_subscription(
            String, '/nav_status', self.status_callback, 10)
        
        self.get_logger().info('Navigation Tester initialized')
        self.print_menu()
    
    def print_menu(self):
        """Print control menu"""
        print("\n" + "="*50)
        print("TURTLEBOT3 NAVIGATION CONTROL")
        print("="*50)
        print("Movement Controls:")
        print("  w/W - Move forward (small/large)")
        print("  s/S - Move backward (small/large)")
        print("  a    - Turn left")
        print("  d    - Turn right")
        print("  q    - Stop")
        print("\nNavigation Commands:")
        print("  e    - Start exploration")
        print("  c    - Cancel exploration")
        print("  m    - Save map")
        print("\nOther:")
        print("  h    - Show this menu")
        print("  x    - Exit")
        print("="*50 + "\n")
    
    def status_callback(self, msg):
        """Handle status messages"""
        self.get_logger().info(f"Status: {msg.data}")
    
    def move(self, linear=0.0, angular=0.0):
        """Send movement command"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f"Command: linear={linear:.2f}, angular={angular:.2f}")
    
    def handle_input(self, key):
        """Handle keyboard input"""
        if key == 'w':
            self.move(linear=0.1)
        elif key == 'W':
            self.move(linear=0.3)
        elif key == 's':
            self.move(linear=-0.1)
        elif key == 'S':
            self.move(linear=-0.3)
        elif key == 'a':
            self.move(angular=0.3)
        elif key == 'd':
            self.move(angular=-0.3)
        elif key == 'q':
            self.move()
            self.get_logger().info("Robot stopped")
        elif key == 'h':
            self.print_menu()
        elif key == 'x':
            return False
        
        return True
    
    def interactive_control(self):
        """Interactive keyboard control"""
        print("Press 'h' for help, 'x' to exit\n")
        
        try:
            while rclpy.ok():
                # Non-blocking input
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).strip().lower()
                    if key:
                        if not self.handle_input(key):
                            break
                
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.move()  # Stop robot
            print("\nControl interrupted")


def main(args=None):
    rclpy.init(args=args)
    
    tester = NavigationTester()
    tester.interactive_control()
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
