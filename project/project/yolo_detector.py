#!/usr/bin/env python3
"""YOLO-Lite Survivor Detection Node for ROS 2.

Detects people in camera frames and publishes detections.
This node is intentionally CPU-bounded: it processes at a controlled rate and
avoids doing expensive work in the image subscription callback.
"""

import os
import sys
import glob
import time
import json
from datetime import datetime
import threading

import cv2
import rclpy
from rclpy.logging import get_logger
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import fcntl
except Exception:
    fcntl = None

YOLO = None


def _extend_sys_path_with_venv_site_packages(root: str) -> bool:
    """Try to add .venv site-packages to sys.path.

    This allows running the node under the system Python (used by ROS 2 entrypoints)
    while still importing YOLO deps that were installed in a project venv.
    """
    if not root:
        return False

    venv_dir = os.path.join(root, '.venv')
    if not os.path.isdir(venv_dir):
        return False

    # Typical venv layout: .venv/lib/python3.12/site-packages
    candidates = glob.glob(os.path.join(venv_dir, 'lib', 'python*', 'site-packages'))
    added = False
    for path in sorted(candidates):
        if os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)
            added = True
    return added


def _try_import_ultralytics(root_hint: str | None = None):
    global YOLO
    if YOLO is not None:
        return

    try:
        from ultralytics import YOLO as _YOLO  # type: ignore
        YOLO = _YOLO
        return
    except Exception:
        pass

    # Fallback: if deps were installed into .venv, make them importable.
    root = root_hint or os.environ.get('YOLO_RESCUESIM_ROOT', '')
    if _extend_sys_path_with_venv_site_packages(root):
        try:
            from ultralytics import YOLO as _YOLO  # type: ignore
            YOLO = _YOLO
            return
        except Exception:
            pass

    YOLO = None


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('singleton_lock', True)
        self.declare_parameter('singleton_lock_path', '/tmp/yolo_rescuesim_yolo_detector.lock')
        self._lock_fd = None
        self._acquire_singleton_lock_if_enabled()
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('skip_frames', 2)
        # Default is intentionally non-zero to avoid pegging CPU when run standalone.
        self.declare_parameter('max_inference_hz', 5.0)
        # Cap CPU usage from Torch/OpenCV thread pools.
        self.declare_parameter('torch_num_threads', 2)
        self.declare_parameter('torch_num_interop_threads', 1)
        self.declare_parameter('opencv_num_threads', 1)

        self.model = None
        self._warned_no_yolo = False

        self._apply_thread_limits()
        _try_import_ultralytics()

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if model_path and not os.path.isabs(model_path):
            # Make relative paths stable when launched from arbitrary working directories.
            root = os.environ.get('YOLO_RESCUESIM_ROOT', '')
            candidate = os.path.join(root, model_path) if root else ''
            if candidate and os.path.exists(candidate):
                model_path = candidate

        if YOLO is None:
            self.get_logger().warn(
                'ultralytics not available. Install into your ROS Python env, or ensure .venv exists. '
                'Suggested: pip3 install --user ultralytics torch opencv-python'
            )
        else:
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model ({model_path}): {e}')
                self.model = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String,
            '/yolo/detections',
            10
        )
        
        self.detection_image_pub = self.create_publisher(
            Image,
            '/yolo/detection_image',
            10
        )
        
        self.frame_count = 0
        self._last_inference_time = 0.0
        self._latest_msg = None
        self._latest_frame_count = 0
        self._latest_lock = threading.Lock()

        # Timer that drives inference. We keep a modest tick and then enforce
        # max_inference_hz via a monotonic clock check.
        self._timer = self.create_timer(0.05, self._process_latest)
        
        self.get_logger().info('YOLO Detector Node Started')

    def _apply_thread_limits(self) -> None:
        """Best-effort CPU limiting for Torch/OpenCV."""
        # OpenCV
        try:
            opencv_threads = int(self.get_parameter('opencv_num_threads').get_parameter_value().integer_value)
            if opencv_threads > 0:
                cv2.setNumThreads(opencv_threads)
        except Exception:
            pass

        # Torch (Ultralytics uses torch under the hood)
        try:
            import torch  # type: ignore

            torch_threads = int(self.get_parameter('torch_num_threads').get_parameter_value().integer_value)
            torch_interop = int(self.get_parameter('torch_num_interop_threads').get_parameter_value().integer_value)

            if torch_threads > 0:
                torch.set_num_threads(torch_threads)
            if torch_interop > 0:
                try:
                    torch.set_num_interop_threads(torch_interop)
                except Exception:
                    # Some builds disallow changing interop threads after init.
                    pass
        except Exception:
            pass

    def _acquire_singleton_lock_if_enabled(self) -> None:
        if not bool(self.get_parameter('singleton_lock').get_parameter_value().bool_value):
            return

        if fcntl is None:
            self.get_logger().warn('fcntl not available; singleton lock disabled')
            return

        lock_path = self.get_parameter('singleton_lock_path').get_parameter_value().string_value
        if not lock_path:
            self.get_logger().warn('singleton_lock_path is empty; singleton lock disabled')
            return

        try:
            self._lock_fd = open(lock_path, 'w')
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lock_fd.write(str(os.getpid()))
            self._lock_fd.flush()
        except BlockingIOError:
            self.get_logger().error(
                f'Another yolo_detector instance is already running (lock: {lock_path}). Exiting.'
            )
            raise SystemExit(0)
        except Exception as e:
            self.get_logger().warn(f'Failed to acquire singleton lock ({lock_path}): {e}')
            self._lock_fd = None
    
    def image_callback(self, msg):
        """Lightweight image callback.

        Stores the most recent eligible frame. Heavy work happens in the timer.
        """
        try:
            self.frame_count += 1

            skip_frames = int(self.get_parameter('skip_frames').get_parameter_value().integer_value)
            if skip_frames > 0 and (self.frame_count % (skip_frames + 1)) != 0:
                return

            with self._latest_lock:
                self._latest_msg = msg
                self._latest_frame_count = self.frame_count
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def _process_latest(self):
        """Run inference at a bounded rate on the latest stored frame."""
        try:
            with self._latest_lock:
                msg = self._latest_msg
                msg_frame_count = self._latest_frame_count
                # Clear so we don't re-process the same frame.
                self._latest_msg = None

            if msg is None:
                return

            max_inference_hz = float(self.get_parameter('max_inference_hz').get_parameter_value().double_value)
            if max_inference_hz > 0.0:
                now = time.monotonic()
                min_dt = 1.0 / max_inference_hz
                if (now - self._last_inference_time) < min_dt:
                    # Put it back (keep latest) if we are throttling.
                    with self._latest_lock:
                        self._latest_msg = msg
                        self._latest_frame_count = msg_frame_count
                    return
                self._last_inference_time = now

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detection_items = []

            if self.model is not None:
                conf_threshold = float(self.get_parameter('conf_threshold').get_parameter_value().double_value)
                results = self.model(cv_image, conf=conf_threshold, classes=[0])

                for result in results:
                    if result.boxes is None:
                        continue
                    for box in result.boxes:
                        if int(box.cls) != 0:
                            continue

                        x1, y1, x2, y2 = box.xyxy[0]
                        conf = float(box.conf)

                        detection_items.append({
                            'class': 'survivor',
                            'confidence': conf,
                            'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        })

                        cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(
                            cv_image,
                            f'Survivor {conf:.2f}',
                            (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                        )
            else:
                if not self._warned_no_yolo:
                    self.get_logger().warn('YOLO model not loaded; /yolo/detections will be empty')
                    self._warned_no_yolo = True

            det_msg = String()
            det_msg.data = json.dumps({
                'timestamp': datetime.now().isoformat(),
                'detections': detection_items,
            })
            self.detection_pub.publish(det_msg)

            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.detection_image_pub.publish(annotated_msg)

            if msg_frame_count % 60 == 0:
                self.get_logger().info(f'Received {msg_frame_count} frames (inference throttled)')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YOLODetector()
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        get_logger('yolo_detector').error(f'Unhandled exception: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
