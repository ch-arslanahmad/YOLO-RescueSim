#!/usr/bin/env python3
"""Fake odometry publisher for ROS 2.

Reads waypoints from a CSV and publishes `nav_msgs/Odometry` on `/odom`.
This is intended for offline pipeline testing (no Gazebo) so the tracker can
attach approximate robot pose to detections.

Behavior:
- If `waypoints_csv` is empty, it auto-picks the most recent CSV under `./waypoints`.
- Loops through the waypoint list.
"""

from __future__ import annotations

import glob
import math
import os
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


def _most_recent_waypoints_csv(repo_root: str) -> str | None:
    candidates = glob.glob(os.path.join(repo_root, "waypoints", "*.csv"))
    if not candidates:
        return None
    candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return candidates[0]


def _load_waypoints(path: str) -> list[Waypoint]:
    # Reuse the robust CSV loader if available (supports multiple formats + name search).
    try:
        from navigation_scripts.navigation.open_loop_waypoint_player import load_waypoints_csv  # type: ignore

        wps = load_waypoints_csv(path)
        return [Waypoint(x=wp.x, y=wp.y, yaw=wp.theta) for wp in wps]
    except Exception:
        # Minimal fallback loader for a simple CSV with X_Position/Y_Position/Theta_Radians.
        import csv

        out: list[Waypoint] = []
        with open(path, "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row.get("X_Position", "0") or 0.0)
                y = float(row.get("Y_Position", "0") or 0.0)
                yaw = float(row.get("Theta_Radians", "0") or 0.0)
                out.append(Waypoint(x=x, y=y, yaw=yaw))
        return out


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    # roll=pitch=0
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class FakeOdomPlayer(Node):
    def __init__(self) -> None:
        super().__init__("fake_odom_player")

        self.declare_parameter("waypoints_csv", "")
        self.declare_parameter("hz", 5.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        repo_root = os.environ.get("YOLO_RESCUESIM_ROOT", os.getcwd())
        csv_path = str(self.get_parameter("waypoints_csv").get_parameter_value().string_value).strip()
        if not csv_path:
            picked = _most_recent_waypoints_csv(repo_root)
            if not picked:
                raise RuntimeError(f"No waypoint CSVs found under: {os.path.join(repo_root, 'waypoints')}")
            csv_path = picked

        self._waypoints = _load_waypoints(csv_path)
        if not self._waypoints:
            raise RuntimeError(f"No waypoints loaded from: {csv_path}")

        self.get_logger().info(f"Publishing fake /odom from: {csv_path} ({len(self._waypoints)} waypoints)")

        self._pub = self.create_publisher(Odometry, "/odom", 10)
        self._idx = 0

        hz = float(self.get_parameter("hz").get_parameter_value().double_value)
        if hz <= 0:
            hz = 5.0

        self._frame_id = str(self.get_parameter("frame_id").get_parameter_value().string_value)
        self._child_frame_id = str(self.get_parameter("child_frame_id").get_parameter_value().string_value)

        self._timer = self.create_timer(1.0 / hz, self._tick)

    def _tick(self) -> None:
        wp = self._waypoints[self._idx]
        self._idx = (self._idx + 1) % len(self._waypoints)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.child_frame_id = self._child_frame_id

        msg.pose.pose.position.x = float(wp.x)
        msg.pose.pose.position.y = float(wp.y)
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = _quat_from_yaw(float(wp.yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Twist left as zeros.
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeOdomPlayer()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
