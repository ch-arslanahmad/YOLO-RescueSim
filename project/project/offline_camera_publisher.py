#!/usr/bin/env python3
"""Offline camera publisher for ROS 2.

Publishes frames to `/camera/image_raw` from either:
- a webcam index (e.g. "0"), or
- a video file path (e.g. "/path/to/video.mp4").

This enables running the YOLO + tracking pipeline without Gazebo.
"""

from __future__ import annotations

import os
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class OfflineCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("offline_camera_publisher")

        self.declare_parameter("source", "0")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("loop", True)

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, "/camera/image_raw", 10)

        source = str(self.get_parameter("source").get_parameter_value().string_value).strip()
        self._loop = bool(self.get_parameter("loop").get_parameter_value().bool_value)
        self._publish_hz = float(self.get_parameter("publish_hz").get_parameter_value().double_value)
        if self._publish_hz <= 0:
            self._publish_hz = 10.0

        # OpenCV VideoCapture accepts an int for webcam index.
        if source.isdigit():
            self._cap = cv2.VideoCapture(int(source))
            self.get_logger().info(f"Opening webcam index: {source}")
        else:
            self._cap = cv2.VideoCapture(source)
            self.get_logger().info(f"Opening video file: {source}")

        if not self._cap.isOpened():
            self.get_logger().error("Failed to open camera source")

        self._timer = self.create_timer(1.0 / self._publish_hz, self._tick)
        self._frame_count = 0
        self._start_ts = time.monotonic()

    def _restart_video(self) -> None:
        try:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        except Exception:
            pass

    def _tick(self) -> None:
        if not self._cap or not self._cap.isOpened():
            return

        ok, frame = self._cap.read()
        if not ok or frame is None:
            if self._loop:
                self._restart_video()
                return
            self.get_logger().warn("Camera source ended; stopping publisher")
            rclpy.shutdown()
            return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self._pub.publish(msg)

        self._frame_count += 1
        if self._frame_count % int(max(1, self._publish_hz * 5)) == 0:
            dt = max(1e-6, time.monotonic() - self._start_ts)
            fps = self._frame_count / dt
            self.get_logger().info(f"Publishing /camera/image_raw (~{fps:.1f} fps)")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OfflineCameraPublisher()
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
