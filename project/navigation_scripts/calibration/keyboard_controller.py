#!/usr/bin/env python3
"""
Keyboard Controller for TurtleBot3
Allows manual control via keyboard for waypoint calibration
Terminal 2 in the 3-terminal workflow
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import threading
import time

try:
    import inputimeout
    HAS_INPUTIMEOUT = True
except ImportError:
    HAS_INPUTIMEOUT = False


class KeyboardController(Node):
    """Manual keyboard control for TurtleBot3"""
    
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Velocity parameters (initialize FIRST)
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Keyboard Controller Initialized')
        self.print_help()
        
    def print_help(self):
        """Display control instructions"""
        help_text = """
╔════════════════════════════════════════════════════════╗
║      TURTLEBOT3 KEYBOARD CONTROLLER                    ║
╠════════════════════════════════════════════════════════╣
║  Movement Controls:                                     ║
║    ↑ / W  - Move Forward                               ║
║    ↓ / S  - Move Backward                              ║
║    ← / A  - Turn Left                                  ║
║    → / D  - Turn Right                                 ║
║                                                        ║
║  Speed Controls:                                        ║
║    +  - Increase Speed                                 ║
║    -  - Decrease Speed                                 ║
║                                                        ║
║  Other:                                                 ║
║    SPACE - Stop (Emergency)                            ║
║    H     - Show Help                                   ║
║    Q     - Quit                                        ║
╚════════════════════════════════════════════════════════╝
"""
        print(help_text)
        print(f"Current Speed: Linear={self.linear_speed:.2f} m/s, Angular={self.angular_speed:.2f} rad/s\n")
    
    def get_key(self):
        """Get a single key press from stdin - REMOVED (no longer used)"""
        pass
    
    def send_command(self, linear=0.0, angular=0.0):
        """Publish movement command"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
    
    def handle_input(self):
        """Main input handler loop - simplified version"""
        print("Ready! Use keyboard to control the robot.")
        print("Press keys: W=forward, S=back, A=left, D=right, SPACE=stop, Q=quit\n")
        
        try:
            while rclpy.ok():
                try:
                    # Use input with timeout
                    if HAS_INPUTIMEOUT:
                        key = inputimeout.inputimeout(timeout=0.1, prompt='')
                    else:
                        # Fallback: just use input (blocking, will pause)
                        print("Input (w/s/a/d/q): ", end='', flush=True)
                        key = input()
                    
                    key = key.lower().strip()
                    
                    if not key:
                        continue
                    
                    first_char = key[0]
                    
                    if first_char == 'w':
                        self.send_command(linear=self.linear_speed)
                        self.get_logger().info(f"Moving FORWARD: {self.linear_speed:.2f} m/s")
                    
                    elif first_char == 's':
                        self.send_command(linear=-self.linear_speed)
                        self.get_logger().info(f"Moving BACKWARD: {self.linear_speed:.2f} m/s")
                    
                    elif first_char == 'a':
                        self.send_command(angular=self.angular_speed)
                        self.get_logger().info(f"Turning LEFT: {self.angular_speed:.2f} rad/s")
                    
                    elif first_char == 'd':
                        self.send_command(angular=-self.angular_speed)
                        self.get_logger().info(f"Turning RIGHT: {self.angular_speed:.2f} rad/s")
                    
                    elif first_char == ' ':
                        self.send_command()
                        self.get_logger().info("⛔ EMERGENCY STOP")
                    
                    elif first_char == '+' or first_char == '=':
                        self.linear_speed = min(0.5, self.linear_speed + 0.05)
                        self.angular_speed = min(1.0, self.angular_speed + 0.1)
                        self.get_logger().info(f"Speed UP: L={self.linear_speed:.2f}, A={self.angular_speed:.2f}")
                    
                    elif first_char == '-':
                        self.linear_speed = max(0.05, self.linear_speed - 0.05)
                        self.angular_speed = max(0.1, self.angular_speed - 0.1)
                        self.get_logger().info(f"Speed DOWN: L={self.linear_speed:.2f}, A={self.angular_speed:.2f}")
                    
                    elif first_char == 'h':
                        self.print_help()
                    
                    elif first_char == 'q':
                        self.get_logger().info("Shutting down...")
                        self.send_command()  # Stop robot
                        break
                
                except (KeyboardInterrupt, inputimeout.TimeoutOccurred):
                    # Timeout is normal - just continue
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue
                
                rclpy.spin_once(self, timeout_sec=0.01)
        
        finally:
            # Stop robot
            self.send_command()


def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    
    try:
        controller.handle_input()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        controller.send_command()  # Stop robot
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
