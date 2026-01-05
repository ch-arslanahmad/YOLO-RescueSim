#!/usr/bin/env python3
"""
YOLO-Lite Human Detection Node for ROS 2
Detects humans in camera frames and publishes detections
"""

import json
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('skip_frames', 2)

        self.model = None
        self._warned_no_yolo = False

        if YOLO is None:
            self.get_logger().warn('ultralytics not available; install with: pip install ultralytics')
        else:
            model_path = self.get_parameter('model_path').get_parameter_value().string_value
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
        
        self.get_logger().info('YOLO Detector Node Started')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1

            skip_frames = int(self.get_parameter('skip_frames').get_parameter_value().integer_value)
            if skip_frames > 0 and (self.frame_count % (skip_frames + 1)) != 0:
                return
            
            detection_items = []
            
            if self.model is not None:
                conf_threshold = float(self.get_parameter('conf_threshold').get_parameter_value().double_value)

                # Run YOLO inference (class 0 = person)
                results = self.model(cv_image, conf=conf_threshold, classes=[0])
                
                # Process detections
                for result in results:
                    if result.boxes is not None:
                        for box in result.boxes:
                            if int(box.cls) == 0:  # Person class
                                x1, y1, x2, y2 = box.xyxy[0]
                                conf = float(box.conf)

                                detection_items.append({
                                    'class': 'person',
                                    'confidence': conf,
                                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                                })
                                
                                # Draw bounding box
                                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                                cv2.putText(cv_image, f'Human {conf:.2f}', (int(x1), int(y1)-10),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
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
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.detection_image_pub.publish(annotated_msg)
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Processed {self.frame_count} frames')
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
