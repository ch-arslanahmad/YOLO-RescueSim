#!/usr/bin/env python3
"""
Waypoint Navigator for TurtleBot3
Navigates robot through recorded waypoints sequentially
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import csv
import math
import time
import os


class WaypointNavigator(Node):
    """Navigate robot through saved waypoints"""
    
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Current robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Navigation parameters
        self.linear_tolerance = 0.15  # meters
        self.angular_tolerance = 0.1  # radians
        self.max_linear_vel = 0.2  # m/s
        self.max_angular_vel = 0.5  # rad/s
        
        # Waypoints
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.is_navigating = False
        
        self.get_logger().info('Waypoint Navigator Initialized')
        self.print_help()
    
    def print_help(self):
        """Display instructions"""
        help_text = """
╔════════════════════════════════════════════════════════╗
║      WAYPOINT NAVIGATOR                                ║
╠════════════════════════════════════════════════════════╣
║  Commands:                                              ║
║    L - Load waypoints from CSV file                    ║
║    S - Show loaded waypoints                           ║
║    N - Navigate through all waypoints                  ║
║    G - Go to specific waypoint                         ║
║    H - Show help                                       ║
║    Q - Quit                                            ║
╚════════════════════════════════════════════════════════╝
"""
        print(help_text)
    
    def odom_callback(self, msg: Odometry):
        """Update robot position"""
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
    
    def load_waypoints(self, filepath):
        """Load waypoints from CSV file"""
        if not os.path.exists(filepath):
            print(f"File not found: {filepath}")
            return False
        
        try:
            self.waypoints = []
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.waypoints.append({
                        'id': int(row['Waypoint_ID']),
                        'x': float(row['X_Position']),
                        'y': float(row['Y_Position']),
                        'theta': float(row['Theta_Radians']),
                    })
            
            print(f"Loaded {len(self.waypoints)} waypoints from {filepath}")
            return True
        except Exception as e:
            print(f"Error loading file: {e}")
            return False
    
    def show_waypoints(self):
        """Display all waypoints"""
        if not self.waypoints:
            print("\nNo waypoints loaded!")
            return
        
        print(f"\n{'='*70}")
        print(f"{'ID':<5} {'X (m)':<12} {'Y (m)':<12} {'Theta (rad)':<12} {'Theta (°)':<12}")
        print(f"{'='*70}")
        
        for wp in self.waypoints:
            theta_deg = math.degrees(wp['theta'])
            print(f"{wp['id']:<5} {wp['x']:<12.4f} {wp['y']:<12.4f} {wp['theta']:<12.4f} {theta_deg:<12.2f}")
        
        print(f"{'='*70}\n")
    
    def _distance_to_target(self, target_x, target_y):
        """Calculate distance to target"""
        return math.sqrt((self.current_x - target_x)**2 + (self.current_y - target_y)**2)
    
    def _angle_diff(self, target_theta):
        """Calculate angle difference"""
        diff = target_theta - self.current_theta
        # Normalize to [-pi, pi]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff
    
    def _send_command(self, linear=0.0, angular=0.0):
        """Publish movement command"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
    
    def navigate_to_waypoint(self, waypoint):
        """Navigate to a specific waypoint"""
        target_x = waypoint['x']
        target_y = waypoint['y']
        target_theta = waypoint['theta']
        
        print(f"\n-> Navigating to Waypoint {waypoint['id']}: X={target_x:.3f}, Y={target_y:.3f}, Θ={target_theta:.3f}rad")
        
        timeout = time.time() + 60  # 60 second timeout
        
        while rclpy.ok():
            # Check timeout
            if time.time() > timeout:
                print(f"Timeout reaching waypoint {waypoint['id']}")
                self._send_command()
                return False
            
            dist = self._distance_to_target(target_x, target_y)
            
            # Check if reached position
            if dist < self.linear_tolerance:
                print(f"Reached position of waypoint {waypoint['id']}")
                
                # Now align rotation
                angle_diff = self._angle_diff(target_theta)
                
                if abs(angle_diff) < self.angular_tolerance:
                    print(f"Aligned rotation of waypoint {waypoint['id']}")
                    self._send_command()
                    return True
                else:
                    # Rotate to target
                    angular_cmd = 0.5 * angle_diff
                    angular_cmd = max(-self.max_angular_vel, min(self.max_angular_vel, angular_cmd))
                    self._send_command(angular=angular_cmd)
                    print(f"  Aligning: angle_diff={angle_diff:.3f}rad", end='\r')
            else:
                # Move to target
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                target_angle = math.atan2(dy, dx)
                angle_diff = self._angle_diff(target_angle)
                
                # Combined linear and angular control
                linear_cmd = 0.3 * dist if dist > 0.2 else 0.1
                linear_cmd = min(self.max_linear_vel, linear_cmd)
                angular_cmd = 0.5 * angle_diff
                angular_cmd = max(-self.max_angular_vel, min(self.max_angular_vel, angular_cmd))
                
                self._send_command(linear=linear_cmd, angular=angular_cmd)
                print(f"  Moving: dist={dist:.3f}m, angle_diff={angle_diff:.3f}rad", end='\r')
            
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def navigate_all(self, pause_time=3.0):
        """Navigate through all waypoints sequentially"""
        if not self.waypoints:
            print("No waypoints loaded!")
            return
        
        print(f"\nStarting navigation through {len(self.waypoints)} waypoints...\n")
        
        for idx, wp in enumerate(self.waypoints):
            print(f"\n[{idx+1}/{len(self.waypoints)}]", end="")
            
            if self.navigate_to_waypoint(wp):
                print(f"  Pausing for {pause_time}s at waypoint {wp['id']}...")
                time.sleep(pause_time)
            else:
                print(f"  Failed to reach waypoint {wp['id']}")
                response = input("Continue to next waypoint? (y/n): ").strip().lower()
                if response != 'y':
                    break
        
        print(f"\nNavigation complete!")
        self._send_command()
    
    def navigate_to_specific(self, wp_id):
        """Navigate to specific waypoint by ID"""
        waypoint = None
        for wp in self.waypoints:
            if wp['id'] == wp_id:
                waypoint = wp
                break
        
        if waypoint is None:
            print(f"Waypoint {wp_id} not found!")
            return
        
        self.navigate_to_waypoint(waypoint)
        print(f"\nReached waypoint {wp_id}!")
    
    def handle_input(self):
        """Main input handler"""
        while rclpy.ok():
            try:
                print("\n[L=Load | S=Show | N=Navigate All | G=Go to | H=Help | Q=Quit]", end=" > ")
                command = input().strip().upper()
                
                if command == 'L':
                    filepath = input("Enter CSV file path: ").strip()
                    self.load_waypoints(filepath)
                
                elif command == 'S':
                    self.show_waypoints()
                
                elif command == 'N':
                    pause = input("Pause time at each waypoint (seconds) [3]: ").strip()
                    pause_time = float(pause) if pause else 3.0
                    self.navigate_all(pause_time)
                
                elif command == 'G':
                    wp_id = input("Enter waypoint ID: ").strip()
                    try:
                        self.navigate_to_specific(int(wp_id))
                    except ValueError:
                        print("Invalid waypoint ID")
                
                elif command == 'H':
                    self.print_help()
                
                elif command == 'Q':
                    print("\nShutting down...")
                    self._send_command()
                    break
                
                else:
                    print("Unknown command. Press 'H' for help.")
                
                rclpy.spin_once(self, timeout_sec=0.01)
            
            except KeyboardInterrupt:
                print("\n\nInterrupted by user")
                self._send_command()
                break


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        navigator.handle_input()
    except Exception as e:
        navigator.get_logger().error(f"Error: {e}")
    finally:
        navigator._send_command()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
