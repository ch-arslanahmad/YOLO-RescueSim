#!/usr/bin/env python3
"""
Coordinate Tracker for TurtleBot3
Monitors robot position and records waypoints
Terminal 3 in the 3-terminal workflow
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import csv
import os
from datetime import datetime
import math


class CoordinateTracker(Node):
    """Track robot coordinates and save waypoints"""
    
    def __init__(self):
        super().__init__('coordinate_tracker')
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Waypoints tracking
        self.waypoints = []
        self.last_saved_pose = (0.0, 0.0, 0.0)
        self.min_distance = 0.1  # Minimum distance between waypoints (meters)
        
        # Waypoints file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        repo_root_waypoints = os.path.join(os.getcwd(), 'waypoints')
        workspace_waypoints = os.path.join(os.getcwd(), 'project', 'navigation_scripts', 'waypoints')
        if os.path.isdir(repo_root_waypoints):
            self.waypoints_dir = repo_root_waypoints
        elif os.path.isdir(workspace_waypoints):
            self.waypoints_dir = workspace_waypoints
        else:
            self.waypoints_dir = os.path.join(os.path.dirname(__file__), '..', 'waypoints')
        os.makedirs(self.waypoints_dir, exist_ok=True)
        self.waypoints_file = os.path.join(
            self.waypoints_dir, f'waypoints_{timestamp}.csv')
        
        # Create CSV header
        self._init_csv()
        
        self.get_logger().info(f'Coordinate Tracker Initialized')
        self.get_logger().info(f'Waypoints will be saved to: {self.waypoints_file}')
        self.print_help()
    
    def print_help(self):
        """Display instructions"""
        help_text = """
╔════════════════════════════════════════════════════════╗
║      COORDINATE TRACKER & WAYPOINT RECORDER            ║
╠════════════════════════════════════════════════════════╣
║  Commands:                                              ║
║    S - Save current position as waypoint               ║
║    U - Undo last waypoint                              ║
║    P - Print all saved waypoints                       ║
║    C - Clear all waypoints (CAREFUL!)                  ║
║    E - Export waypoints to new file                    ║
║    H - Show this help                                  ║
║    Q - Quit                                            ║
╚════════════════════════════════════════════════════════╝

Press Enter after each command.
Minimum distance between waypoints: {:.2f}m

""".format(self.min_distance)
        print(help_text)
    
    def _init_csv(self):
        """Initialize CSV file with headers"""
        with open(self.waypoints_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Waypoint_ID', 'X_Position', 'Y_Position', 'Theta_Radians', 'Theta_Degrees', 'Timestamp'])
    
    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry"""
        pose: Pose = msg.pose.pose
        
        # Extract x, y position
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        
        # Extract theta from quaternion
        quat = pose.orientation
        self.current_theta = self._quaternion_to_yaw(quat)
        
        # Print current position (updates in place)
        print(f"\r[LIVE] X: {self.current_x:7.3f}m | Y: {self.current_y:7.3f}m | Θ: {self.current_theta:7.3f}rad ({math.degrees(self.current_theta):7.1f}°) | Waypoints: {len(self.waypoints)}", end='')
    
    def _quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle (theta)"""
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        
        # Convert to yaw
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return yaw
    
    def _distance_from_last(self):
        """Calculate distance from last saved waypoint"""
        last_x, last_y, _ = self.last_saved_pose
        dist = math.sqrt((self.current_x - last_x)**2 + (self.current_y - last_y)**2)
        return dist
    
    def save_waypoint(self):
        """Save current position as waypoint"""
        dist = self._distance_from_last()
        
        if dist < self.min_distance and len(self.waypoints) > 0:
            print(f"\n[WARN] Too close to last waypoint ({dist:.3f}m < {self.min_distance}m). Skipped.")
            return False
        
        waypoint_id = len(self.waypoints) + 1
        timestamp = datetime.now().isoformat()
        theta_degrees = math.degrees(self.current_theta)
        
        # Add to list
        self.waypoints.append({
            'id': waypoint_id,
            'x': self.current_x,
            'y': self.current_y,
            'theta': self.current_theta,
            'theta_deg': theta_degrees
        })
        
        # Save to CSV
        with open(self.waypoints_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([waypoint_id, f"{self.current_x:.4f}", f"{self.current_y:.4f}", 
                           f"{self.current_theta:.4f}", f"{theta_degrees:.2f}", timestamp])
        
        # Update last saved pose
        self.last_saved_pose = (self.current_x, self.current_y, self.current_theta)
        
        print(f"\n[OK] Waypoint {waypoint_id} saved: X={self.current_x:.4f}, Y={self.current_y:.4f}, Θ={self.current_theta:.4f}rad")
        return True
    
    def undo_waypoint(self):
        """Remove last waypoint"""
        if not self.waypoints:
            print("\n[ERROR] No waypoints to undo!")
            return
        
        removed = self.waypoints.pop()
        
        # Rewrite CSV without last entry
        with open(self.waypoints_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Waypoint_ID', 'X_Position', 'Y_Position', 'Theta_Radians', 'Theta_Degrees', 'Timestamp'])
            for wp in self.waypoints:
                writer.writerow([wp['id'], f"{wp['x']:.4f}", f"{wp['y']:.4f}", 
                               f"{wp['theta']:.4f}", f"{wp['theta_deg']:.2f}", ''])
        
        # Update last saved pose
        if self.waypoints:
            last = self.waypoints[-1]
            self.last_saved_pose = (last['x'], last['y'], last['theta'])
        else:
            self.last_saved_pose = (0.0, 0.0, 0.0)
        
        print(f"\n[UNDO] Waypoint {removed['id']} removed. Total: {len(self.waypoints)}")
    
    def print_waypoints(self):
        """Display all saved waypoints"""
        print(f"\n{'='*70}")
        print(f"{'Waypoint ID':<12} {'X (m)':<12} {'Y (m)':<12} {'Theta (rad)':<12} {'Theta (°)':<12}")
        print(f"{'='*70}")
        
        if not self.waypoints:
            print("No waypoints saved yet!")
        else:
            for wp in self.waypoints:
                print(f"{wp['id']:<12} {wp['x']:<12.4f} {wp['y']:<12.4f} {wp['theta']:<12.4f} {wp['theta_deg']:<12.2f}")
        
        print(f"{'='*70}\n")
    
    def clear_waypoints(self):
        """Clear all waypoints (with confirmation)"""
        print("\n[WARN] This will delete all waypoints!")
        response = input("Type 'yes' to confirm: ").strip().lower()
        
        if response == 'yes':
            self.waypoints = []
            self.last_saved_pose = (0.0, 0.0, 0.0)
            self._init_csv()
            print("[OK] All waypoints cleared!")
        else:
            print("Cancelled.")
    
    def export_waypoints(self):
        """Create a copy of waypoints for use in navigation"""
        if not self.waypoints:
            print("\n[ERROR] No waypoints to export!")
            return
        
        export_name = input("Enter export filename (without .csv): ").strip()
        if not export_name:
            print("Cancelled.")
            return
        
        export_file = os.path.join(self.waypoints_dir, f'{export_name}.csv')
        
        with open(export_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Waypoint_ID', 'X_Position', 'Y_Position', 'Theta_Radians'])
            for wp in self.waypoints:
                writer.writerow([wp['id'], f"{wp['x']:.4f}", f"{wp['y']:.4f}", f"{wp['theta']:.4f}"])
        
        print(f"Exported to: {export_file}")
    
    def handle_input(self):
        """Main input handler"""
        while rclpy.ok():
            try:
                print("\n[S=Save | U=Undo | P=Print | C=Clear | E=Export | H=Help | Q=Quit]", end=" > ")
                command = input().strip().upper()
                
                if command == 'S':
                    self.save_waypoint()
                elif command == 'U':
                    self.undo_waypoint()
                elif command == 'P':
                    self.print_waypoints()
                elif command == 'C':
                    self.clear_waypoints()
                elif command == 'E':
                    self.export_waypoints()
                elif command == 'H':
                    self.print_help()
                elif command == 'Q':
                    print("\nShutting down...")
                    break
                else:
                    print("[ERROR] Unknown command. Press 'H' for help.")
                
                rclpy.spin_once(self, timeout_sec=0.01)
            
            except KeyboardInterrupt:
                print("\n\nInterrupted by user")
                break


def main(args=None):
    rclpy.init(args=args)
    tracker = CoordinateTracker()
    
    try:
        tracker.handle_input()
    except Exception as e:
        tracker.get_logger().error(f"Error: {e}")
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
