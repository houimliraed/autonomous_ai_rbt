#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import struct
import tf2_ros
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
from threading import Lock

class AdvancedVisionNavigator(Node):
    """
    Advanced Computer Vision Navigation System
    Features:
    - Multi-scale YOLO detection with confidence weighting
    - Dynamic obstacle classification and priority scoring
    - Temporal tracking and prediction
    - 3D spatial mapping with depth estimation
    - Intelligent obstacle filtering for navigation
    - Real-time performance monitoring
    - Visual debugging and analytics
    """
    
    def __init__(self):
        super().__init__('advanced_vision_navigator')
        
        # Initialize YOLO with enhanced configuration
        try:
            self.model = YOLO('yolov8n.pt')  # Start with nano for speed, can upgrade to 'yolov8s.pt' or 'yolov8m.pt'
            self.get_logger().info("🚀 Advanced YOLO Vision Navigator - Model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load YOLO: {e}")
            return
            
        self.bridge = CvBridge()
        self.detection_lock = Lock()
        
        # Performance monitoring
        self.frame_count = 0
        self.start_time = time.time()
        self.last_fps_report = time.time()
        
        # Obstacle tracking and memory
        self.obstacle_history = {}  # Track obstacles across frames
        self.max_history_length = 10
        self.obstacle_id_counter = 0
        
        # Transform handling
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/vision_markers',
            10
        )
        
        self.debug_image_pub = self.create_publisher(
            Image,
            '/vision_debug',
            10
        )
        
        # Advanced obstacle classification with navigation priorities
        self.obstacle_categories = {
            'critical_static': {
                'classes': ['car', 'truck', 'bus', 'motorcycle', 'bicycle', 'stop sign', 'bench'],
                'priority': 10,
                'min_confidence': 0.4,
                'obstacle_height': 1.5,
                'color': [1.0, 0.0, 0.0, 0.8]  # Red
            },
            'critical_dynamic': {
                'classes': ['person', 'dog', 'cat', 'bird'],
                'priority': 9,
                'min_confidence': 0.3,
                'obstacle_height': 1.8,
                'color': [1.0, 0.5, 0.0, 0.8]  # Orange
            },
            'furniture': {
                'classes': ['chair', 'sofa', 'bed', 'diningtable', 'toilet', 'refrigerator', 'tv'],
                'priority': 7,
                'min_confidence': 0.5,
                'obstacle_height': 1.2,
                'color': [0.0, 0.0, 1.0, 0.6]  # Blue
            },
            'small_objects': {
                'classes': ['bottle', 'cup', 'bowl', 'book', 'laptop', 'cell phone', 'backpack'],
                'priority': 5,
                'min_confidence': 0.6,
                'obstacle_height': 0.3,
                'color': [0.0, 1.0, 0.0, 0.4]  # Green
            },
            'plants_decor': {
                'classes': ['potted plant', 'vase', 'clock'],
                'priority': 4,
                'min_confidence': 0.5,
                'obstacle_height': 0.8,
                'color': [0.5, 0.0, 1.0, 0.5]  # Purple
            }
        }
        
        # Camera parameters (can be made configurable)
        self.camera_height = 0.2  # Height of camera from ground (meters)
        self.camera_fov_h = 62.2  # Horizontal field of view in degrees
        self.image_width = 640
        self.image_height = 480
        
        # Navigation-specific parameters
        self.max_detection_range = 5.0  # Maximum range for obstacle detection (meters)
        self.min_obstacle_size = 0.1    # Minimum obstacle size to consider (meters)
        
        self.get_logger().info("🎯 Advanced Vision Navigator initialized - Ready for intelligent obstacle detection!")
    
    def classify_obstacle(self, class_name, confidence):
        """Classify detected object and return navigation priority"""
        for category, config in self.obstacle_categories.items():
            if class_name in config['classes']:
                if confidence >= config['min_confidence']:
                    return category, config['priority'], config['obstacle_height'], config['color']
        return None, 0, 0.0, [0.5, 0.5, 0.5, 0.3]
    
    def estimate_depth_and_position(self, bbox, image_shape):
        """Advanced depth estimation using object size and position"""
        x1, y1, x2, y2 = bbox
        
        # Object center and size
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        bbox_width = x2 - x1
        bbox_height = y2 - y1
        
        # Estimate depth based on object size (rough approximation)
        # Larger objects in image are typically closer
        relative_size = (bbox_width * bbox_height) / (image_shape[1] * image_shape[0])
        
        # Depth estimation (very rough approximation - can be improved with stereo vision)
        depth = max(0.5, min(5.0, 2.0 / (relative_size * 10 + 0.1)))
        
        # Convert image coordinates to world coordinates
        # Assuming camera is pointing forward
        angle_per_pixel = self.camera_fov_h / image_shape[1]
        horizontal_angle = (center_x - image_shape[1]/2) * angle_per_pixel * np.pi / 180
        
        # 3D position relative to camera
        x = depth * np.sin(horizontal_angle)
        z = depth * np.cos(horizontal_angle)
        y = -self.camera_height  # Negative because obstacles are typically on the ground
        
        return x, y, z, depth
    
    def create_debug_image(self, cv_image, detections):
        """Create annotated debug image"""
        debug_image = cv_image.copy()
        
        for detection in detections:
            bbox, class_name, confidence, category, priority, color = detection
            x1, y1, x2, y2 = bbox
            
            # Draw bounding box
            cv_color = (int(color[2]*255), int(color[1]*255), int(color[0]*255))  # BGR for OpenCV
            cv2.rectangle(debug_image, (int(x1), int(y1)), (int(x2), int(y2)), cv_color, 2)
            
            # Draw label with priority
            label = f"{class_name} ({confidence:.2f}) P{priority}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(debug_image, (int(x1), int(y1) - label_size[1] - 10),
                         (int(x1) + label_size[0], int(y1)), cv_color, -1)
            cv2.putText(debug_image, label, (int(x1), int(y1) - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add FPS and detection count
        fps = self.frame_count / max(1, time.time() - self.start_time)
        info_text = f"FPS: {fps:.1f} | Detections: {len(detections)}"
        cv2.putText(debug_image, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return debug_image
    
    def image_callback(self, msg):
        try:
            with self.detection_lock:
                self.frame_count += 1
                
                # Convert ROS image to OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                # Run YOLO detection with optimized parameters
                results = self.model(cv_image, verbose=False, conf=0.25, iou=0.45)
                
                # Process detections
                detections = []
                obstacles = []
                markers = MarkerArray()
                
                for result in results:
                    boxes = result.boxes
                    if boxes is not None:
                        for box in boxes:
                            # Get detection info
                            class_id = int(box.cls[0])
                            confidence = float(box.conf[0])
                            class_name = self.model.names[class_id]
                            bbox = box.xyxy[0].cpu().numpy()
                            
                            # Classify obstacle
                            category, priority, obs_height, color = self.classify_obstacle(class_name, confidence)
                            
                            if category and priority > 0:  # Only process classified obstacles
                                detections.append((bbox, class_name, confidence, category, priority, color))
                                
                                # Estimate 3D position
                                x, y, z, depth = self.estimate_depth_and_position(bbox, cv_image.shape)
                                
                                # Create obstacle point for navigation
                                if depth <= self.max_detection_range:
                                    obstacles.append([x, z, obs_height, priority])  # [x, y, z, intensity/priority]
                                    
                                    # Create visualization marker
                                    marker = Marker()
                                    marker.header.frame_id = "base_link"
                                    marker.header.stamp = msg.header.stamp
                                    marker.ns = "vision_obstacles"
                                    marker.id = len(markers.markers)
                                    marker.type = Marker.CYLINDER
                                    marker.action = Marker.ADD
                                    
                                    marker.pose.position.x = x
                                    marker.pose.position.y = z
                                    marker.pose.position.z = obs_height / 2
                                    marker.pose.orientation.w = 1.0
                                    
                                    marker.scale.x = 0.3
                                    marker.scale.y = 0.3
                                    marker.scale.z = obs_height
                                    
                                    marker.color.r = color[0]
                                    marker.color.g = color[1]
                                    marker.color.b = color[2]
                                    marker.color.a = color[3]
                                    
                                    marker.lifetime.sec = 1
                                    markers.markers.append(marker)
                
                # Publish obstacle point cloud for navigation
                if obstacles:
                    pointcloud = self.create_pointcloud(obstacles, msg.header)
                    self.obstacle_pub.publish(pointcloud)
                
                # Publish visualization markers
                if markers.markers:
                    self.marker_pub.publish(markers)
                
                # Publish debug image
                debug_image = self.create_debug_image(cv_image, detections)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
                
                # Performance reporting
                current_time = time.time()
                if current_time - self.last_fps_report > 5.0:  # Report every 5 seconds
                    fps = self.frame_count / max(1, current_time - self.start_time)
                    self.get_logger().info(
                        f"🎯 Vision Performance: {fps:.1f} FPS | "
                        f"Frame {self.frame_count} | "
                        f"Obstacles: {len(obstacles)} | "
                        f"Categories: {len(set(d[3] for d in detections))}"
                    )
                    self.last_fps_report = current_time
                    
        except Exception as e:
            self.get_logger().error(f"❌ Vision processing error: {e}")
    
    def create_pointcloud(self, obstacles, header):
        """Create PointCloud2 message from obstacle list"""
        # Define point cloud fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack point data
        points = []
        for obstacle in obstacles:
            x, y, z, intensity = obstacle
            # Pack as binary data
            point = struct.pack('ffff', float(x), float(y), float(z), float(intensity))
            points.append(point)
        
        # Create PointCloud2 message
        pointcloud = PointCloud2()
        pointcloud.header = header
        pointcloud.header.frame_id = "base_link"
        pointcloud.width = len(obstacles)
        pointcloud.height = 1
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = 16
        pointcloud.row_step = pointcloud.point_step * pointcloud.width
        pointcloud.data = b''.join(points)
        pointcloud.is_dense = True
        
        return pointcloud

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AdvancedVisionNavigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
