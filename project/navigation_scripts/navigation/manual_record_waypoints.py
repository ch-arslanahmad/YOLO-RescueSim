#!/usr/bin/env python3
"""
Manual Waypoint Recorder - Option 2
Reads robot position from Gazebo and lets you record waypoints manually
No odometry issues - just query the current robot state
"""

import csv
from datetime import datetime
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ManualWaypointRecorder(Node):
    """Record waypoints by querying robot position manually"""
    
    def __init__(self):
        super().__init__('manual_waypoint_recorder')
        
        # Publisher for movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Waypoint storage
        self.waypoints = []
        self.waypoint_count = 0
        self.last_export_path = None
        
        # Output directory
        repo_root_waypoints = os.path.join(os.getcwd(), 'waypoints')
        workspace_waypoints = os.path.join(os.getcwd(), 'project', 'navigation_scripts', 'waypoints')
        if os.path.isdir(repo_root_waypoints):
            self.output_dir = repo_root_waypoints
        elif os.path.isdir(workspace_waypoints):
            self.output_dir = workspace_waypoints
        else:
            self.output_dir = os.path.join(os.path.dirname(__file__), '..', 'waypoints')
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Teleop settings
        self.linear_speed = 0.2
        self.angular_speed = 0.6
        self.teleop_active = False
        self._last_twist = Twist()
        
        self.get_logger().info('Manual Waypoint Recorder Ready')
        self.print_menu()
    
    def print_menu(self):
        """Display instructions"""
        menu = """
╔════════════════════════════════════════════════════════╗
║   MANUAL WAYPOINT RECORDER - Option 2 (No Odometry)   ║
╠════════════════════════════════════════════════════════╣
║  Commands (type letter + ENTER):                       ║
║    t = Teleop drive (real-time W/A/S/D)                ║
║    r = Record waypoint (prompts for Gazebo X/Y/theta)  ║
║    p = Print recorded waypoints                        ║
║    e = Export waypoints to CSV (prints full path)      ║
║    l = Print last exported CSV path again              ║
║    d = Demo: attempt move + prompt record each step    ║
║    h = Show this help                                  ║
║    q = Quit                                             ║
║                                                         ║
║  Teleop keys (in Teleop mode):                          ║
║    W/A/D - drive (hold key)                            ║
║    S     - stop                                         ║
║    SPACE - record waypoint (prompts for Gazebo coords)  ║
║    X     - exit teleop                                  ║
║                                                         ║
║  Why it doesn't auto-record position:                  ║
║    - Auto-record needs a pose source (usually /odom or  ║
║      TF like map->base_link). In this sim setup those   ║
║      pose topics are not reliably available/bridged, so ║
║      the recorder has no way to know “current X/Y”.     ║
║    - This tool is the reliable fallback: you read the   ║
║      pose from Gazebo UI and type it in.               ║
╚════════════════════════════════════════════════════════╝
"""
        print(menu)

    def _safe_publish(self, twist: Twist) -> None:
        """Publish safely (avoid crashing during shutdown)."""
        try:
            if not rclpy.ok():
                return
            self.cmd_vel_pub.publish(twist)
        except Exception:
            # Ignore publish failures when context is shutting down
            return

    def _stop_robot(self) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self._safe_publish(twist)
        self._last_twist = twist

    def _get_key_nonblocking(self) -> str | None:
        """Return a single keypress if available (raw mode), else None."""
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    def _teleop_loop(self) -> None:
        """Real-time teleop loop. Press X to exit teleop."""
        print("\n--- TELEOP MODE ---")
        print("Hold W/A/D to move, S to stop, SPACE to record, X to exit")
        print("(You will still be prompted to type Gazebo X/Y/theta when recording.)\n")

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            self.teleop_active = True
            while self.teleop_active and rclpy.ok():
                key = self._get_key_nonblocking()
                twist = Twist()

                if key:
                    k = key.upper()
                    if k == 'W':
                        twist.linear.x = self.linear_speed
                    elif k == 'A':
                        twist.angular.z = self.angular_speed
                    elif k == 'D':
                        twist.angular.z = -self.angular_speed
                    elif k == 'S':
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    elif key == ' ':  # SPACE
                        self._stop_robot()
                        self.record_waypoint()
                        continue
                    elif k == 'X':
                        self.teleop_active = False
                        break
                    else:
                        # Unhandled key: keep last command
                        twist = self._last_twist

                    self._safe_publish(twist)
                    self._last_twist = twist
                else:
                    # No key: keep publishing last command briefly for smoother hold behavior
                    self._safe_publish(self._last_twist)

                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.02)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self._stop_robot()
            print("\n--- EXITED TELEOP MODE ---\n")
    
    def get_position_from_gazebo(self):
        """Get position by asking user to read from Gazebo GUI"""
        print("\n" + "="*70)
        print("Record Waypoint - Enter Robot Position from Gazebo")
        print("="*70)
        print("\nLook at the Gazebo window:")
        print("  • Select the robot, then use the pose/transform readout")
        print("    (many Gazebo builds show X/Y in the bottom/status bar).")
        print("  • Theta is the yaw angle in radians (about the vertical axis).")
        print("  • Enter the values below.\n")
        
        try:
            x = float(input("Enter robot X position (meters): "))
            y = float(input("Enter robot Y position (meters): "))
            theta = float(input("Enter robot angle/theta (radians, 0-2π): "))
            
            self.current_x = x
            self.current_y = y
            self.current_theta = theta
            
            return True
        except ValueError:
            print("Invalid input! Please enter numbers.")
            return False
    
    def record_waypoint(self):
        """Record current position as waypoint"""
        if not self.get_position_from_gazebo():
            return
        
        self.waypoint_count += 1
        waypoint = {
            'Waypoint_ID': self.waypoint_count,
            'X_Position': round(self.current_x, 4),
            'Y_Position': round(self.current_y, 4),
            'Theta_Radians': round(self.current_theta, 4),
            'Timestamp': datetime.now().isoformat()
        }
        self.waypoints.append(waypoint)
        
        print(f"\nWaypoint {self.waypoint_count} recorded")
        print(f"   Position: X={waypoint['X_Position']:.4f}, Y={waypoint['Y_Position']:.4f}")
        print(f"   Angle: {waypoint['Theta_Radians']:.4f} radians")
        print(f"   Total waypoints: {len(self.waypoints)}\n")
    
    def print_waypoints(self):
        """Display all recorded waypoints"""
        if not self.waypoints:
            print("\nNo waypoints recorded yet!\n")
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
        """Export waypoints to CSV"""
        print(f"\n[DEBUG] Exporting... waypoints in memory: {len(self.waypoints)}")
        
        if not self.waypoints:
            print("No waypoints to export!\n")
            return None
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"waypoints_manual_{timestamp}.csv"
        filepath = os.path.join(self.output_dir, filename)
        
        print(f"[DEBUG] Output dir: {self.output_dir}")
        print(f"[DEBUG] Full path: {filepath}\n")
        
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
            
            print(f"Exported {len(self.waypoints)} waypoints!")
            print(f"File: {filepath}\n")
            print(f"Copy/paste this path into the navigator: {filepath}\n")
            self.last_export_path = filepath
            return filepath
        except Exception as e:
            print(f"Error exporting: {e}\n")
            import traceback
            traceback.print_exc()
            return None

    def print_last_export_path(self):
        """Print the most recently exported CSV path."""
        if not self.last_export_path:
            print("\nℹ️  No CSV exported yet. Use 'e' to export first.\n")
            return
        print(f"\nLast exported CSV: {self.last_export_path}\n")
    
    def move_robot_demo(self):
        """Demo: Move robot in a pattern for testing"""
        print("\n" + "="*70)
        print("DEMO MODE: Moving robot and recording waypoints")
        print("="*70)
        print("\nThe robot will attempt to move in a pattern.")
        print("After each movement, you'll be prompted to enter the Gazebo X/Y/theta for the waypoint.")
        print("If the robot doesn't move in your sim, that's OK—just drag it in Gazebo between prompts.\n")
        
        movements = [
            {"linear": 0.2, "angular": 0.0, "duration": 2.0, "label": "Forward"},
            {"linear": 0.0, "angular": 0.6, "duration": 1.3, "label": "Left turn"},
            {"linear": 0.2, "angular": 0.0, "duration": 2.0, "label": "Forward again"},
            {"linear": 0.0, "angular": -0.6, "duration": 1.3, "label": "Right turn"},
        ]
        
        for i, move in enumerate(movements, 1):
            print(f"\n[Movement {i}/{len(movements)}] {move['label']}...")
            twist = Twist()
            twist.linear.x = move['linear']
            twist.angular.z = move['angular']
            
            start = time.time()
            while (time.time() - start) < move['duration']:
                self._safe_publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.05)
            
            # Stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self._safe_publish(twist)

            time.sleep(0.2)
            print("Movement step complete. Now record the waypoint from Gazebo.")
            self.record_waypoint()
        
        print("\n" + "="*70)
        print("Demo complete! All movements recorded.")
        print("="*70)
    
    def run(self):
        """Main interactive loop"""
        while True:
            try:
                cmd = input("\n> Enter command (t=teleop, r=record, p=print, e=export, l=last, d=demo, h=help, q=quit): ").strip().lower()
                
                if cmd == 't':
                    self._teleop_loop()
                elif cmd == 'r':
                    self.record_waypoint()
                elif cmd == 'p':
                    self.print_waypoints()
                elif cmd == 'e':
                    self.export_waypoints()
                elif cmd == 'l':
                    self.print_last_export_path()
                elif cmd == 'd':
                    self.move_robot_demo()
                elif cmd == 'h':
                    self.print_menu()
                elif cmd == 'q':
                    print("\nShutting down...")
                    break
                else:
                    print("Unknown command. Type 'h' for help.")
            
            except KeyboardInterrupt:
                print("\n\nShutting down...")
                break
            except Exception as e:
                print(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    recorder = ManualWaypointRecorder()
    
    try:
        recorder.run()
    finally:
        try:
            recorder._stop_robot()
        except Exception:
            pass
        try:
            recorder.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
