#!/usr/bin/env python3
"""
Human Tracking and Map Generation Node
Tracks detected humans, estimates their world positions, and exports map
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose2D
import json
import csv
from datetime import datetime
import numpy as np
from pathlib import Path
import threading


class HumanTracker(Node):
    def __init__(self):
        super().__init__('human_tracker')
        
        # Subscription to detections
        self.detection_sub = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        # Publishers
        self.tracked_humans_pub = self.create_publisher(
            String,
            '/human_tracker/tracked_humans',
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
        
        # Parameters
        self.max_distance_threshold = 1.0  # meters, for association
        self.camera_height = 0.5  # meters above ground
        self.camera_fov_h = 62  # horizontal FOV in degrees
        self.camera_fov_v = 48  # vertical FOV in degrees
        
        # Output file
        self.output_dir = Path('/home/arslan/Desktop/github/YOLO-RescueSim/project/human_detections')
        self.output_dir.mkdir(exist_ok=True)
        
        self.get_logger().info('Human Tracker Node Started')
    
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
    
    def detection_callback(self, msg):
        """Process incoming detections and track humans"""
        try:
            payload = json.loads(msg.data)

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

                if 'center' in det:
                    center_x, center_y = det['center']
                else:
                    x1, y1, x2, y2 = det.get('bbox', [0, 0, 0, 0])
                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0
                
                # Convert pixel to world coordinates
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
                        'timestamp': det['timestamp']
                    })
                else:
                    track_id = self.next_human_id
                    self.next_human_id += 1
                    self.tracked_humans[track_id] = {
                        'position': [world_x, world_y],
                        'history': [{
                            'position': [world_x, world_y],
                            'confidence': confidence,
                            'timestamp': det['timestamp']
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
                'timestamp': datetime.now().isoformat()
            })
            self.tracked_humans_pub.publish(tracked_msg)
            
            # Periodically save map
            if self.detection_count % 60 == 0:
                self.export_human_map()
            
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
    node = HumanTracker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
