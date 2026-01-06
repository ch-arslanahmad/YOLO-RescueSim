#!/usr/bin/env python3
"""
Human Tracking and Map Generation Node
Tracks detected humans, estimates their world positions, and exports map
"""

import os
import time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import String
from std_msgs.msg import Int32
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose2D
import json
import csv
from datetime import datetime
import numpy as np
from pathlib import Path
import threading
import math

try:
    import fcntl
except Exception:
    fcntl = None


class HumanTracker(Node):
    def __init__(self):
        super().__init__('human_tracker')

        self.declare_parameter('singleton_lock', True)
        self.declare_parameter('singleton_lock_path', '/tmp/yolo_rescuesim_human_tracker.lock')
        self._lock_fd = None
        self._acquire_singleton_lock_if_enabled()
        
        # Subscription to detections
        self.detection_sub = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.tracked_humans_pub = self.create_publisher(
            String,
            '/human_tracker/tracked_humans',
            10
        )

        self.survivor_count_pub = self.create_publisher(
            Int32,
            '/human_tracker/survivor_count',
            10
        )
        
        self.map_pub = self.create_publisher(
            String,
            '/human_tracker/human_map',
            10
        )
        
        # Human tracking storage
        self.tracked_humans = {}  # ID -> {position, history, confidence}
        self.detection_count = 0
        self.next_human_id = 1

        self._last_robot_pose = None  # (x, y, yaw)
        
        # Parameters
        self.declare_parameter('max_distance_threshold', 1.0)
        self.declare_parameter('log_every_n_messages', 30)

        self.max_distance_threshold = float(
            self.get_parameter('max_distance_threshold').get_parameter_value().double_value
        )  # meters, for association
        self.camera_height = 0.5  # meters above ground
        self.camera_fov_h = 62  # horizontal FOV in degrees
        self.camera_fov_v = 48  # vertical FOV in degrees
        
        # Output file
        self.output_dir = Path('/home/arslan/Desktop/github/YOLO-RescueSim/project/human_detections')
        self.output_dir.mkdir(exist_ok=True)
        
        self.get_logger().info('Human Tracker Node Started')

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
                f'Another human_tracker instance is already running (lock: {lock_path}). Exiting.'
            )
            raise SystemExit(0)
        except Exception as e:
            self.get_logger().warn(f'Failed to acquire singleton lock ({lock_path}): {e}')
            self._lock_fd = None
    
    def image_to_world_coords(self, pixel_x, pixel_y, img_width=640, img_height=480):
        """
        Convert pixel coordinates to approximate world coordinates
        Simplified conversion assuming flat ground
        """
        # Normalize to image center
        norm_x = (pixel_x - img_width/2) / (img_width/2)
        norm_y = (pixel_y - img_height/2) / (img_height/2)
        
        # Convert to world coordinates (rough estimate)
        # Assuming camera is pointing forward at robot
        world_x = norm_x * 3.0  # Scale factor
        world_y = 2.0 + norm_y * 2.0  # Forward offset + scale
        
        return world_x, world_y

    def odom_callback(self, msg: Odometry) -> None:
        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation

            # yaw from quaternion
            siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            self._last_robot_pose = (float(pos.x), float(pos.y), float(yaw))
        except Exception:
            return

    def _estimate_survivor_world_position(self, pixel_x: float, pixel_y: float) -> tuple[float, float] | None:
        """Estimate survivor position in map/world frame.

        Uses a simple camera pixel->relative projection (existing heuristic) and
        then transforms it by the current robot pose from /odom.
        """
        if self._last_robot_pose is None:
            return None

        rel_x, rel_y = self.image_to_world_coords(pixel_x, pixel_y)
        robot_x, robot_y, robot_yaw = self._last_robot_pose

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        world_x = robot_x + (rel_x * cos_yaw - rel_y * sin_yaw)
        world_y = robot_y + (rel_x * sin_yaw + rel_y * cos_yaw)
        return world_x, world_y
    
    def detection_callback(self, msg):
        """Process incoming detections and track humans"""
        try:
            payload = json.loads(msg.data)

            msg_timestamp = None
            if isinstance(payload, dict):
                msg_timestamp = payload.get('timestamp')
            if not msg_timestamp:
                msg_timestamp = datetime.now().isoformat()

            # Backward-compatible parsing:
            # - New format: {"timestamp": ..., "detections": [{"class":"person","confidence":0.9,"bbox":[x1,y1,x2,y2]}]}
            # - Old format: [{"center":[cx,cy],"confidence":0.9,...}, ...]
            if isinstance(payload, dict):
                detections = payload.get('detections', [])
            else:
                detections = payload
            self.detection_count += 1
            
            current_tracked = {}
            
            for det in detections:
                confidence = float(det.get('confidence', 0.0))

                det_timestamp = det.get('timestamp', msg_timestamp)

                if 'center' in det:
                    center_x, center_y = det['center']
                else:
                    x1, y1, x2, y2 = det.get('bbox', [0, 0, 0, 0])
                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0

                est = self._estimate_survivor_world_position(center_x, center_y)
                if est is not None:
                    world_x, world_y = est
                else:
                    # Fallback: relative heuristic only
                    world_x, world_y = self.image_to_world_coords(center_x, center_y)
                
                # Simple tracking: find nearest previous detection
                best_match = None
                best_distance = float('inf')
                
                for human_id, human_data in self.tracked_humans.items():
                    if human_id in current_tracked:
                        continue
                    
                    dist = np.sqrt((human_data['position'][0] - world_x)**2 + 
                                 (human_data['position'][1] - world_y)**2)
                    
                    if dist < best_distance and dist < self.max_distance_threshold:
                        best_distance = dist
                        best_match = human_id
                
                # Assign detection to track or create new
                if best_match:
                    track_id = best_match
                    self.tracked_humans[track_id]['position'] = [world_x, world_y]
                    self.tracked_humans[track_id]['history'].append({
                        'position': [world_x, world_y],
                        'confidence': confidence,
                        'timestamp': det_timestamp,
                        'robot_pose': list(self._last_robot_pose) if self._last_robot_pose is not None else None,
                    })
                else:
                    track_id = self.next_human_id
                    self.next_human_id += 1
                    self.tracked_humans[track_id] = {
                        'position': [world_x, world_y],
                        'history': [{
                            'position': [world_x, world_y],
                            'confidence': confidence,
                            'timestamp': det_timestamp,
                            'robot_pose': list(self._last_robot_pose) if self._last_robot_pose is not None else None,
                        }],
                        'confidence': confidence
                    }
                
                current_tracked[track_id] = True
            
            # Publish tracked humans
            tracked_msg = String()
            tracked_msg.data = json.dumps({
                'count': len(self.tracked_humans),
                'humans': {
                    str(k): v['position'] for k, v in self.tracked_humans.items()
                },
                'timestamp': datetime.now().isoformat(),
                'last_robot_pose': list(self._last_robot_pose) if self._last_robot_pose is not None else None,
            })
            self.tracked_humans_pub.publish(tracked_msg)

            self.survivor_count_pub.publish(Int32(data=len(self.tracked_humans)))
            
            # Periodically save map
            if self.detection_count % 60 == 0:
                self.export_human_map()

            log_every_n = int(self.get_parameter('log_every_n_messages').get_parameter_value().integer_value)
            if log_every_n > 0 and (self.detection_count % log_every_n) == 0:
                self.get_logger().info(f'Tracking {len(self.tracked_humans)} humans')
        
        except Exception as e:
            self.get_logger().error(f'Error in tracking: {e}')
    
    def export_human_map(self):
        """Export human positions to CSV and JSON"""
        if not self.tracked_humans:
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Export to CSV
        csv_file = self.output_dir / f'human_map_{timestamp}.csv'
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Human_ID', 'X_Position', 'Y_Position', 'Detections', 'Last_Confidence'])
            
            for human_id, data in self.tracked_humans.items():
                pos = data['position']
                writer.writerow([
                    human_id,
                    pos[0],
                    pos[1],
                    len(data['history']),
                    data['confidence']
                ])
        
        # Export to JSON
        json_file = self.output_dir / f'human_map_{timestamp}.json'
        with open(json_file, 'w') as f:
            json_data = {
                'timestamp': datetime.now().isoformat(),
                'total_humans': len(self.tracked_humans),
                'humans': self.tracked_humans
            }
            json.dump(json_data, f, indent=2)
        
        self.get_logger().info(f'Exported human map to {csv_file} and {json_file}')
        
        # Also publish map
        map_msg = String()
        map_msg.data = json.dumps({
            'humans': self.tracked_humans,
            'timestamp': datetime.now().isoformat()
        })
        self.map_pub.publish(map_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HumanTracker()
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        get_logger('human_tracker').error(f'Unhandled exception: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
