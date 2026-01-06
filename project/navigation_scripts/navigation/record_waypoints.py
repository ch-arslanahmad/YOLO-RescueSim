#!/usr/bin/env python3
"""
Waypoint Recorder for TurtleBot3
Records robot position as you move it around using keyboard commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import math
import time
from datetime import datetime
import os
import sys
import threading
import termios
import tty
import select


class WaypointRecorder(Node):
    """Record robot coordinates while moving"""
    
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Current robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Recording
        self.waypoints = []
        self.waypoint_count = 0
        self.is_recording = False
        self.record_start_time = None
        
        # Movement parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Output directory
        self.output_dir = os.path.join(
            os.path.dirname(__file__), '..', 'waypoints'
        )
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Keyboard state
        self.running = True
        self.is_moving_forward = False
        self.is_turning_left = False
        self.is_turning_right = False
        
        # Terminal state
        self.old_settings = None
        
        self.get_logger().info('Waypoint Recorder Initialized')
        self.print_help()
    
    def print_help(self):
        """Display instructions"""
        help_text = """
╔════════════════════════════════════════════════════════╗
║      WAYPOINT RECORDER - Real-time Control              ║
╠════════════════════════════════════════════════════════╣
║  Movement (Press and Hold):                             ║
║    W - Move Forward                                    ║
║    A - Turn Left                                       ║
║    D - Turn Right                                      ║
║    S - Stop/Brake                                      ║
║                                                         ║
║  Recording Commands:                                    ║
║    SPACE - Record current position as waypoint         ║
║    R     - Toggle Recording Mode indicator             ║
║    P     - Print all recorded waypoints                ║
║    E     - Export waypoints to CSV file                ║
║    C     - Clear all waypoints                         ║
║    H     - Show this help menu                         ║
║    Q     - Quit                                        ║
╚════════════════════════════════════════════════════════╝
"""
        print(help_text)
    
    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry"""
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        self.current_theta = self._quaternion_to_yaw(pose.orientation)
    
    def _quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return yaw
    
    def move_robot(self, linear_x=0.0, angular_z=0.0):
        """Publish velocity command to move robot"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        """Stop the robot"""
        self.move_robot(0.0, 0.0)
    
    def _getch(self):
        """Read a single character from stdin without requiring Enter"""
        try:
            fd = sys.stdin.fileno()
            self.old_settings = termios.tcgetattr(fd)
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            return ch
        finally:
            if self.old_settings:
                termios.tcsetattr(fd, termios.TCSADRAIN, self.old_settings)
    
    def record_waypoint(self):
        """Record current position as a waypoint"""
        self.waypoint_count += 1
        waypoint = {
            'Waypoint_ID': self.waypoint_count,
            'X_Position': round(self.current_x, 4),
            'Y_Position': round(self.current_y, 4),
            'Theta_Radians': round(self.current_theta, 4),
            'Timestamp': time.time()
        }
        self.waypoints.append(waypoint)
        
        print(f"\nWaypoint {self.waypoint_count} recorded:")
        print(f"  Position: ({waypoint['X_Position']}, {waypoint['Y_Position']})")
        print(f"  Angle: {waypoint['Theta_Radians']:.4f} rad")
        print(f"  [{len(self.waypoints)} waypoints total]\n")
    
    def print_waypoints(self):
        """Display all recorded waypoints"""
        if not self.waypoints:
            print("No waypoints recorded yet!\n")
            return
        
        print("\n" + "="*70)
        print("Recorded Waypoints:")
        print("="*70)
        print(f"{'ID':>3} | {'X Position':>12} | {'Y Position':>12} | {'Theta (rad)':>12}")
        print("-"*70)
        
        for wp in self.waypoints:
            print(f"{wp['Waypoint_ID']:>3} | {wp['X_Position']:>12.4f} | {wp['Y_Position']:>12.4f} | {wp['Theta_Radians']:>12.4f}")
        
        print("="*70 + "\n")
    
    def export_waypoints(self):
        """Export waypoints to CSV file"""
        if not self.waypoints:
            print("No waypoints to export!\n")
            return
        
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
            
            print(f"\nExported {len(self.waypoints)} waypoints to:")
            print(f"  {filepath}\n")
            return filepath
        except Exception as e:
            print(f"\nError exporting waypoints: {e}\n")
            return None
    
    def clear_waypoints(self):
        """Clear all recorded waypoints"""
        if not self.waypoints:
            print("ℹ No waypoints to clear\n")
            return
        
        count = len(self.waypoints)
        self.waypoints = []
        self.waypoint_count = 0
        print(f"Cleared {count} waypoints\n")
    
    def run(self):
        """Main interactive loop with real-time keyboard input"""
        print("\n" + "="*70)
        print("KEYBOARD CONTROL ACTIVE - Press keys (NO ENTER needed)")
        print("="*70 + "\n")
        
        status_update_time = time.time()
        
        try:
            while self.running and rclpy.ok():
                # Continuous movement update
                self._update_movement()
                
                # Try to read keyboard input (non-blocking)
                try:
                    # Set a short timeout for input
                    import select
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        ch = sys.stdin.read(1).upper()
                        self._handle_key(ch)
                except:
                    pass
                
                # Periodic status update (every 0.5 seconds)
                current_time = time.time()
                if current_time - status_update_time > 0.5:
                    status_update_time = current_time
                    mode_indicator = "REC" if self.is_recording else "IDLE"
                    movement_indicator = ""
                    if self.is_moving_forward:
                        movement_indicator = " ^"
                    elif self.is_turning_left:
                        movement_indicator = " <"
                    elif self.is_turning_right:
                        movement_indicator = " >"
                    
                    sys.stdout.write(
                        f"\rX={self.current_x:7.3f} Y={self.current_y:7.3f} theta={self.current_theta:6.3f} | {mode_indicator}{movement_indicator}   "
                    )
                    sys.stdout.flush()
                
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\n\nShutting down...")
        finally:
            self.stop_robot()
            self.running = False
    
    def _update_movement(self):
        """Update continuous movement based on keyboard state"""
        if self.is_moving_forward:
            self.move_robot(linear_x=self.linear_speed, angular_z=0.0)
        elif self.is_turning_left:
            self.move_robot(linear_x=0.0, angular_z=self.angular_speed)
        elif self.is_turning_right:
            self.move_robot(linear_x=0.0, angular_z=-self.angular_speed)
        else:
            self.stop_robot()
    
    def _handle_key(self, key):
        """Handle keyboard input"""
        if key == 'W':
            self.is_moving_forward = True
            self.is_turning_left = False
            self.is_turning_right = False
        elif key == 'A':
            self.is_moving_forward = False
            self.is_turning_left = True
            self.is_turning_right = False
        elif key == 'D':
            self.is_moving_forward = False
            self.is_turning_left = False
            self.is_turning_right = True
        elif key == 'S':
            self.is_moving_forward = False
            self.is_turning_left = False
            self.is_turning_right = False
        elif key == ' ':
            self.record_waypoint()
        elif key == 'R':
            self.is_recording = not self.is_recording
            status = "ON" if self.is_recording else "OFF"
            print(f"\nRecording mode: {status}")
        elif key == 'P':
            print("\n")
            self.print_waypoints()
        elif key == 'E':
            print("\n")
            self.export_waypoints()
        elif key == 'C':
            self.clear_waypoints()
        elif key == 'H':
            print("\n")
            self.print_help()
        elif key == 'Q':
            self.running = False
            print("\nShutting down...")
        elif key == '\n':
            pass  # Ignore Enter key


def main():
    rclpy.init()
    recorder = WaypointRecorder()
    
    try:
        recorder.run()
    finally:
        recorder.stop_robot()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
