#!/usr/bin/env python3
"""
Simple Real-Time Keyboard Controller for TurtleBot3
Just press keys - no Enter needed!
Use WASD keys for movement.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import threading
import time

try:
    from pynput import keyboard as pynput_keyboard
    HAS_PYNPUT = True
except ImportError:
    HAS_PYNPUT = False
    print("pynput not found. Install with: pip install pynput")


class SimpleKeyboardController(Node):
    """Real-time keyboard control - no Enter needed!"""
    
    def __init__(self):
        super().__init__('keyboard_controller_simple')
        
        # Speed parameters (INCREASED for visibility)
        self.linear_speed = 0.5   # m/s (increased from 0.2)
        self.angular_speed = 1.0  # rad/s (increased from 0.5)
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info('Real-Time Keyboard Controller Started')
        
        # Track which keys are pressed
        self.keys_pressed = set()
        
        # Print controls
        self.print_controls()
        
        if HAS_PYNPUT:
            self.start_keyboard_listener()
        else:
            self.get_logger().error("pynput required. Install: pip install pynput")
            rclpy.shutdown()
    
    def print_controls(self):
        """Display controls"""
        print("""
╔════════════════════════════════════════╗
║   REAL-TIME KEYBOARD CONTROLLER        ║
╠════════════════════════════════════════╣
║  W      - Move Forward                 ║
║  S      - Move Backward                ║
║  A      - Turn Left                    ║
║  D      - Turn Right                   ║
║  SPACE  - Stop                         ║
║  +/-    - Speed Up / Down              ║
║  Q      - Quit                         ║
╚════════════════════════════════════════╝

Speed: Linear={:.2f} m/s, Angular={:.2f} rad/s
Just press keys - no Enter needed!
""".format(self.linear_speed, self.angular_speed))
    
    def on_press(self, key):
        """Handle key press"""
        try:
            key_char = key.char.lower() if hasattr(key, 'char') else None
        except AttributeError:
            key_char = str(key).replace('Key.', '').lower()
        
        if key_char:
            self.keys_pressed.add(key_char)
            
            # Handle special keys
            if key_char == 'q':
                self.get_logger().info("Shutting down...")
                self.send_command()
                rclpy.shutdown()
    
    def on_release(self, key):
        """Handle key release"""
        try:
            key_char = key.char.lower() if hasattr(key, 'char') else None
        except AttributeError:
            key_char = str(key).replace('Key.', '').lower()
        
        if key_char:
            self.keys_pressed.discard(key_char)
    
    def start_keyboard_listener(self):
        """Start listening to keyboard in background thread"""
        listener = pynput_keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()
        
        # Run movement loop
        self.movement_loop()
    
    def movement_loop(self):
        """Continuously check pressed keys and update movement"""
        rate = self.create_rate(10)  # 10 Hz
        
        while rclpy.ok():
            linear = 0.0
            angular = 0.0
            
            # Check which keys are pressed
            if 'w' in self.keys_pressed:
                linear = self.linear_speed
            elif 's' in self.keys_pressed:
                linear = -self.linear_speed
            
            if 'a' in self.keys_pressed:
                angular = self.angular_speed
            elif 'd' in self.keys_pressed:
                angular = -self.angular_speed
            
            # Handle speed changes
            if '+' in self.keys_pressed or '=' in self.keys_pressed:
                self.linear_speed = min(1.0, self.linear_speed + 0.05)
                self.angular_speed = min(2.0, self.angular_speed + 0.1)
                self.keys_pressed.discard('+')
                self.keys_pressed.discard('=')
                self.get_logger().info(f"Speed UP: L={self.linear_speed:.2f}, A={self.angular_speed:.2f}")
            
            if '-' in self.keys_pressed:
                self.linear_speed = max(0.1, self.linear_speed - 0.05)
                self.angular_speed = max(0.2, self.angular_speed - 0.1)
                self.keys_pressed.discard('-')
                self.get_logger().info(f"Speed DOWN: L={self.linear_speed:.2f}, A={self.angular_speed:.2f}")
            
            # Send command
            if linear != 0.0 or angular != 0.0:
                self.send_command(linear, angular)
                print(f"CMD L:{linear:+.2f} A:{angular:+.2f}", end='\r')
            else:
                self.send_command()
            
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
    
    def send_command(self, linear=0.0, angular=0.0):
        """Publish movement command"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = float(linear)
        twist.twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleKeyboardController()


if __name__ == '__main__':
    main()
