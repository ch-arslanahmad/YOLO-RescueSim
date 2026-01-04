#!/usr/bin/env python3
"""Open-loop waypoint playback for TurtleBot3.

This player drives through waypoints without relying on /odom or TF.
It uses the recorded waypoint poses as the *assumed* pose and converts
waypoint-to-waypoint deltas into timed /cmd_vel commands.

Trade-offs:
- Works even when /odom isn't available.
- Accuracy is limited (open-loop drift). Best for demos and pipeline testing.

CSV columns supported:
- Required: X_Position, Y_Position
- Heading: Theta_Radians (preferred) OR Theta_Degrees
- Waypoint id: Waypoint_ID (optional, used for printing)

Usage (from sourced ROS terminal):
python3 -c "from navigation_scripts.navigation.open_loop_waypoint_player import main; main()"
"""

from __future__ import annotations

import csv
import glob
import math
import os
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: int
    x: float
    y: float
    theta: float  # radians


def _normalize_angle(angle: float) -> float:
    """Normalize to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _workspace_waypoints_dir() -> str | None:
    """Return the workspace waypoints directory if running from repo root."""
    candidate = os.path.join(os.getcwd(), "project", "navigation_scripts", "waypoints")
    if os.path.isdir(candidate):
        return candidate
    return None


def _repo_root_waypoints_dir() -> str | None:
    """Return the repo-root waypoints directory if present (preferred for persistence)."""
    candidate = os.path.join(os.getcwd(), "waypoints")
    if os.path.isdir(candidate):
        return candidate
    return None


def _read_float(row: dict, key: str) -> float | None:
    value = row.get(key)
    if value is None:
        return None
    value = str(value).strip()
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def load_waypoints_csv(path: str) -> list[Waypoint]:
    """
    Load waypoints from CSV. If path is just a filename, search for it.
    If path is empty, use the most recent waypoint CSV found.
    """
    # If path is empty, find and select CSV interactively (or auto-use if only one)
    if not path or not path.strip():
        path = _select_csv_interactive()
    # If path is just a filename (no slashes), search for it
    elif "/" not in path and "\\" not in path:
        found = _find_csv_by_name(path)
        if not found:
            raise FileNotFoundError(f"CSV file not found: {path}")
        path = found
        print(f"Found: {path}\n")

    if not os.path.exists(path):
        raise FileNotFoundError(path)

    waypoints: list[Waypoint] = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError("CSV has no header")

        for idx, row in enumerate(reader, start=1):
            x = _read_float(row, "X_Position")
            y = _read_float(row, "Y_Position")
            if x is None or y is None:
                raise ValueError(f"Row {idx}: missing X_Position/Y_Position")

            theta = _read_float(row, "Theta_Radians")
            if theta is None:
                theta_deg = _read_float(row, "Theta_Degrees")
                theta = math.radians(theta_deg) if theta_deg is not None else 0.0

            wp_id = row.get("Waypoint_ID")
            try:
                waypoint_id = int(wp_id) if wp_id is not None and str(wp_id).strip() else idx
            except ValueError:
                waypoint_id = idx

            waypoints.append(Waypoint(waypoint_id=waypoint_id, x=float(x), y=float(y), theta=float(theta)))

    if not waypoints:
        raise ValueError("CSV contains no waypoints (no data rows)")

    return waypoints


def _find_most_recent_csv() -> str | None:
    """Find the most recently modified waypoint CSV file."""
    repo_root_dir = _repo_root_waypoints_dir()
    workspace_dir = _workspace_waypoints_dir()
    search_paths = [
        # Repo root (preferred)
        os.path.join(repo_root_dir, "*.csv") if repo_root_dir else "",
        # Workspace (preferred)
        os.path.join(workspace_dir, "*.csv") if workspace_dir else "",
        # Build directory (source)
        os.path.join(
            os.path.dirname(__file__), "..", "waypoints", "*.csv"
        ),
        # Install directory (common case)
        os.path.expanduser(
            "~/.local/lib/python*/site-packages/navigation_scripts/waypoints/*.csv"
        ),
        # Install directory (project-specific)
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "..",
            "..",
            "waypoints",
            "*.csv",
        ),
    ]

    all_csvs = []
    for pattern in search_paths:
        if not pattern:
            continue
        matches = glob.glob(pattern)
        all_csvs.extend(matches)

    if not all_csvs:
        return None

    # Sort by mtime, most recent first
    all_csvs.sort(key=lambda f: os.path.getmtime(f), reverse=True)
    return all_csvs[0]


def _find_csv_by_name(name: str) -> str | None:
    """Search for a CSV file by partial name (case-insensitive)."""
    repo_root_dir = _repo_root_waypoints_dir()
    workspace_dir = _workspace_waypoints_dir()
    search_paths = [
        os.path.join(repo_root_dir, "*.csv") if repo_root_dir else "",
        os.path.join(workspace_dir, "*.csv") if workspace_dir else "",
        os.path.join(os.path.dirname(__file__), "..", "waypoints", "*.csv"),
        os.path.expanduser("~/.local/lib/python*/site-packages/navigation_scripts/waypoints/*.csv"),
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "waypoints", "*.csv"),
    ]

    name_lower = name.lower()
    for pattern in search_paths:
        if not pattern:
            continue
        for path in glob.glob(pattern):
            if name_lower in os.path.basename(path).lower():
                return path

    return None


def _find_all_csvs() -> list[str]:
    """Find all waypoint CSV files with data, sorted by mtime (most recent first)."""
    repo_root_dir = _repo_root_waypoints_dir()
    workspace_dir = _workspace_waypoints_dir()
    search_paths = [
        os.path.join(repo_root_dir, "*.csv") if repo_root_dir else "",
        os.path.join(workspace_dir, "*.csv") if workspace_dir else "",
        os.path.join(os.path.dirname(__file__), "..", "waypoints", "*.csv"),
        os.path.expanduser("~/.local/lib/python*/site-packages/navigation_scripts/waypoints/*.csv"),
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "waypoints", "*.csv"),
    ]

    all_csvs = []
    seen = set()
    for pattern in search_paths:
        if not pattern:
            continue
        for path in glob.glob(pattern):
            if path not in seen:
                # Skip empty CSVs (header only, no data rows)
                if _has_csv_data(path):
                    all_csvs.append(path)
                    seen.add(path)

    if all_csvs:
        all_csvs.sort(key=lambda f: os.path.getmtime(f), reverse=True)
    return all_csvs


def _has_csv_data(path: str) -> bool:
    """Check if CSV file has at least one data row (not just header)."""
    try:
        with open(path, "r", newline="") as f:
            reader = csv.DictReader(f)
            for _ in reader:
                return True  # Has at least one row
        return False  # Only header, no data
    except Exception:
        return False


def _select_csv_interactive() -> str:
    """Find CSV files and let user choose (or auto-select if only one)."""
    csvs = _find_all_csvs()
    
    if not csvs:
        raise FileNotFoundError("No waypoint CSV files found in project.")
    
    if len(csvs) == 1:
        print(f"Found CSV: {os.path.basename(csvs[0])}\n")
        return csvs[0]
    
    # Multiple files: list with options
    print("Found multiple CSV files:\n")
    for idx, path in enumerate(csvs, 1):
        mtime = os.path.getmtime(path)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mtime))
        print(f"  {idx}. {os.path.basename(path)} ({timestamp})")
    
    while True:
        choice = input("\nChoose file (number): ").strip()
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(csvs):
                return csvs[idx]
        except ValueError:
            pass
        print(f"Invalid choice. Enter 1-{len(csvs)}")



class OpenLoopWaypointPlayer(Node):
    def __init__(self):
        super().__init__("open_loop_waypoint_player")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _publish(self, linear: float = 0.0, angular: float = 0.0) -> None:
        try:
            if not rclpy.ok():
                return
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.cmd_vel_pub.publish(msg)
        except Exception:
            return

    def stop(self) -> None:
        self._publish(0.0, 0.0)

    def _run_for(self, duration_s: float, linear: float, angular: float) -> None:
        end_t = time.time() + max(0.0, duration_s)
        while rclpy.ok() and time.time() < end_t:
            self._publish(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        self.stop()

    def rotate(self, delta_rad: float, angular_speed: float) -> None:
        if abs(delta_rad) < 1e-6:
            return
        direction = 1.0 if delta_rad > 0 else -1.0
        speed = abs(float(angular_speed))
        if speed < 1e-3:
            raise ValueError("angular_speed must be > 0")
        duration = abs(delta_rad) / speed
        self._run_for(duration, linear=0.0, angular=direction * speed)

    def drive_forward(self, distance_m: float, linear_speed: float) -> None:
        if abs(distance_m) < 1e-6:
            return
        speed = abs(float(linear_speed))
        if speed < 1e-3:
            raise ValueError("linear_speed must be > 0")
        duration = abs(distance_m) / speed
        self._run_for(duration, linear=speed, angular=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopWaypointPlayer()

    try:
        print("\nOPEN-LOOP WAYPOINT PLAYER")
        print("- Drives using timed /cmd_vel (no /odom required)")
        print("- Before starting: place robot at Waypoint 1 pose in Gazebo\n")

        env_csv = os.environ.get("WAYPOINT_CSV", "").strip()
        if env_csv:
            print(f"Using waypoint CSV from WAYPOINT_CSV: {env_csv}")
            csv_path = env_csv
        else:
            csv_path = input("Enter waypoint CSV path (or press ENTER to auto-select): ").strip()
        waypoints = load_waypoints_csv(csv_path)

        print(f"\nLoaded {len(waypoints)} waypoints:")
        for wp in waypoints:
            print(f"  {wp.waypoint_id}: x={wp.x:.4f}, y={wp.y:.4f}, theta={wp.theta:.4f}")

        linear_speed_in = input("Linear speed m/s [0.20]: ").strip()
        angular_speed_in = input("Angular speed rad/s [0.60]: ").strip()
        pause_in = input("Pause at each waypoint seconds [1.0]: ").strip()

        linear_speed = float(linear_speed_in) if linear_speed_in else 0.20
        angular_speed = float(angular_speed_in) if angular_speed_in else 0.60
        pause_s = float(pause_in) if pause_in else 1.0

        input("\nPlace robot at waypoint 1 in Gazebo, then press ENTER to start...")

        # Assume starting pose == waypoint[0]
        current = waypoints[0]
        print(f"\nStarting from waypoint {current.waypoint_id} (assumed pose)")
        time.sleep(max(0.0, pause_s))

        for next_wp in waypoints[1:]:
            dx = next_wp.x - current.x
            dy = next_wp.y - current.y
            distance = math.hypot(dx, dy)
            travel_heading = math.atan2(dy, dx) if distance > 1e-9 else current.theta

            # 1) Rotate from current.theta to travel_heading
            d1 = _normalize_angle(travel_heading - current.theta)
            # 2) Drive forward distance
            # 3) Rotate from travel_heading to next_wp.theta
            d2 = _normalize_angle(next_wp.theta - travel_heading)

            print(
                f"\n-> Segment {current.waypoint_id} -> {next_wp.waypoint_id}: "
                f"dist={distance:.3f}m, rot1={d1:.3f}rad, rot2={d2:.3f}rad"
            )

            node.rotate(d1, angular_speed=angular_speed)
            node.drive_forward(distance, linear_speed=linear_speed)
            node.rotate(d2, angular_speed=angular_speed)

            print(f"Reached waypoint {next_wp.waypoint_id} (open-loop assumed)")
            time.sleep(max(0.0, pause_s))
            current = next_wp

        print("\nOpen-loop playback complete")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        try:
            node.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
