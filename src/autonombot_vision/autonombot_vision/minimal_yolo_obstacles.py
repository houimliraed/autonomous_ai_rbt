#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import struct

class MinimalYoloObstacles(Node):
    def __init__(self):
        super().__init__('minimal_yolo_obstacles')
        
        # Initialize YOLO
        try:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info("YOLO model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO: {e}")
            return
            
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            '/vision_obstacles',
            10
        )
        
        # Enhanced obstacle classes to detect (including small objects)
        self.obstacle_classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck',
            'chair', 'sofa', 'bed', 'diningtable', 'toilet', 'refrigerator',
            'bottle', 'wine glass', 'cup', 'bowl', 'banana', 'apple',
            'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
            'cake', 'potted plant', 'tv', 'laptop', 'mouse', 'remote',
            'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
            'sink', 'book', 'clock', 'scissors', 'teddy bear', 'hair drier',
            'toothbrush', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase',
            'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
            'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'vase', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
            'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe'
        ]
        
        # Very low confidence thresholds for testing - detect anything YOLO sees
        self.person_confidence_threshold = 0.1  # Very low for testing
        self.general_confidence_threshold = 0.1  # Very low for testing
        
        self.get_logger().info("Minimal YOLO Obstacles node started")
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run YOLO detection
            results = self.model(cv_image, verbose=False)
            
            # Debug: Log all detections before filtering
            detection_count = 0
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    detection_count += len(boxes)
                    for box in boxes:
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        class_name = self.model.names[class_id]
                        self.get_logger().info(
                            f"🔍 RAW DETECTION: {class_name} - conf: {confidence:.2f}"
                        )
            
            if detection_count == 0:
                self.get_logger().info("❌ No objects detected by YOLO in this frame")
            
            # Create point cloud for obstacles
            obstacles = []
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Get class name
                        class_id = int(box.cls[0])
                        class_name = self.model.names[class_id]
                        confidence = float(box.conf[0])
                        
                        # Use different confidence thresholds for different objects
                        confidence_threshold = self.person_confidence_threshold if class_name == 'person' else self.general_confidence_threshold
                        
                        # Only process obstacle classes with appropriate confidence
                        if class_name in self.obstacle_classes and confidence > confidence_threshold:
                            # Get bounding box center
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            
                            # Simple depth estimation (assume objects are 1-2.5m away)
                            # Larger objects in image = closer
                            box_area = (x2 - x1) * (y2 - y1)
                            image_area = cv_image.shape[0] * cv_image.shape[1]
                            area_ratio = box_area / image_area
                            
                            # Estimate distance (more conservative)
                            estimated_distance = max(1.0, 2.5 - (area_ratio * 3.0))
                            
                            # Convert pixel coordinates to world coordinates (simple approximation)
                            # Assume camera FOV ~60 degrees
                            image_width = cv_image.shape[1]
                            image_height = cv_image.shape[0]
                            
                            # Horizontal angle from center
                            angle_x = (center_x - image_width/2) * (60.0 * np.pi / 180.0) / image_width
                            
                            # Calculate world position relative to robot
                            world_x = estimated_distance * np.cos(angle_x)
                            world_y = estimated_distance * np.sin(angle_x)
                            world_z = 0.0  # Ground level
                            
                            obstacles.append([world_x, world_y, world_z])
                            
                            # More informative logging
                            detection_type = "⚠️  SIM-OBJECT" if class_name == 'person' else "🚧 OBSTACLE"
                            self.get_logger().info(f"{detection_type}: {class_name} at ({world_x:.2f}, {world_y:.2f}m) - conf: {confidence:.2f}")
            
            # Publish obstacles as point cloud
            if obstacles:
                self.publish_obstacles(obstacles, msg.header.stamp)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def publish_obstacles(self, obstacles, timestamp):
        """Publish obstacles as a point cloud"""
        header = Header()
        header.stamp = timestamp
        header.frame_id = 'base_link'  # Robot's base frame
        
        # Create point cloud fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack point data
        points_data = []
        for point in obstacles:
            points_data.extend(struct.pack('fff', point[0], point[1], point[2]))
        
        # Create point cloud message
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(obstacles)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = bytes(points_data)
        cloud_msg.is_dense = True
        
        self.obstacle_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MinimalYoloObstacles()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
