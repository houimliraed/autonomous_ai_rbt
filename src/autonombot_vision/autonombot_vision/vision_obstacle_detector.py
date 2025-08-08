#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String, Header
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from builtin_interfaces.msg import Time
import math

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed. Install with: pip install ultralytics")

class VisionObstacleDetector(Node):
    def __init__(self):
        super().__init__('vision_obstacle_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)  # Lower threshold for navigation safety
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_height', 0.2)  # Height of camera above ground (meters)
        self.declare_parameter('camera_range', 3.0)   # Max detection range (meters)
        self.declare_parameter('obstacle_classes', ['chair', 'couch', 'person', 'refrigerator', 'dining table'])
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value
        self.camera_range = self.get_parameter('camera_range').get_parameter_value().double_value
        self.obstacle_classes = self.get_parameter('obstacle_classes').get_parameter_value().string_array_value
        
        # Initialize YOLO model
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f"YOLO model loaded: {self.model_path}")
                
                # Get class names from model
                self.class_names = self.model.names
                self.get_logger().info(f"Available classes: {list(self.class_names.values())}")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO model: {e}")
                self.model = None
        else:
            self.model = None
            self.get_logger().warn("YOLO not available - running in demo mode")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.obstacles_pub = self.create_publisher(
            PointCloud2,
            '/vision_obstacles',
            10
        )
        
        self.detections_pub = self.create_publisher(
            String,
            '/vision_detections',
            10
        )
        
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/camera/annotated_image_nav',
            10
        )
        
        # Camera parameters (typical for 640x480)
        self.image_width = 640
        self.image_height = 480
        self.focal_length = 500  # Approximate focal length
        self.cx = self.image_width / 2
        self.cy = self.image_height / 2
        
        self.get_logger().info("Vision Obstacle Detector initialized")
    
    def image_callback(self, msg):
        if self.model is None:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLO detection
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
            
            # Process detections
            obstacles = []
            detections_text = []
            annotated_image = cv_image.copy()
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Get detection info
                        conf = float(box.conf[0])
                        cls_id = int(box.cls[0])
                        class_name = self.class_names[cls_id]
                        
                        # Only process obstacle classes
                        if class_name not in self.obstacle_classes:
                            continue
                        
                        # Get bounding box
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        
                        # Estimate 3D position of obstacle
                        obstacle_points = self.estimate_obstacle_position(x1, y1, x2, y2, class_name)
                        obstacles.extend(obstacle_points)
                        
                        # Add to detection text
                        detections_text.append(f"{class_name}: {conf:.2f}")
                        
                        # Draw on image
                        cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(annotated_image, f"{class_name}: {conf:.2f}", 
                                  (int(x1), int(y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish results
            self.publish_obstacle_pointcloud(obstacles, msg.header.stamp)
            self.publish_detections(detections_text)
            self.publish_annotated_image(annotated_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
    
    def estimate_obstacle_position(self, x1, y1, x2, y2, class_name):
        """Estimate 3D position of obstacle based on bounding box"""
        obstacles = []
        
        # Use bottom center of bounding box as ground contact point
        center_x = (x1 + x2) / 2
        bottom_y = y2
        
        # Convert image coordinates to 3D position
        # Assume flat ground and use simple pinhole camera model
        
        # Horizontal angle from camera center
        horizontal_angle = math.atan((center_x - self.cx) / self.focal_length)
        
        # Estimate distance based on object size and type
        bbox_height = y2 - y1
        bbox_width = x2 - x1
        
        # Rough distance estimation based on typical object sizes
        if class_name in ['chair', 'couch']:
            # Assume chair height ~0.9m, couch height ~0.8m
            expected_height = 0.85
            distance = (expected_height * self.focal_length) / bbox_height
        elif class_name == 'person':
            # Assume person height ~1.7m
            expected_height = 1.7
            distance = (expected_height * self.focal_length) / bbox_height
        elif class_name == 'refrigerator':
            # Assume refrigerator height ~1.8m
            expected_height = 1.8
            distance = (expected_height * self.focal_length) / bbox_height
        else:
            # Default estimation
            distance = 2.0
        
        # Limit distance to reasonable range
        distance = max(0.5, min(distance, self.camera_range))
        
        # Calculate position relative to camera
        x_cam = distance * math.sin(horizontal_angle)
        y_cam = distance * math.cos(horizontal_angle)
        z_cam = 0.0  # Assume objects are on ground level
        
        # Create obstacle points (add some spatial extent)
        obstacle_width = max(0.3, bbox_width / self.focal_length * distance)  # Estimate width
        num_points = max(3, int(obstacle_width / 0.1))  # Point every 10cm
        
        for i in range(num_points):
            offset = (i - num_points/2) * 0.1
            obstacles.append([
                x_cam + offset * math.cos(horizontal_angle),
                y_cam - offset * math.sin(horizontal_angle),
                z_cam
            ])
        
        return obstacles
    
    def publish_obstacle_pointcloud(self, obstacles, timestamp):
        """Publish detected obstacles as PointCloud2"""
        if not obstacles:
            return
        
        try:
            # Transform points to map frame
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_link', 
                rclpy.time.Time.from_msg(timestamp),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Create PointCloud2 message
            cloud_msg = PointCloud2()
            cloud_msg.header.stamp = timestamp
            cloud_msg.header.frame_id = 'map'
            cloud_msg.height = 1
            cloud_msg.width = len(obstacles)
            cloud_msg.is_dense = False
            cloud_msg.is_bigendian = False
            
            # Define fields
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            cloud_msg.point_step = 12
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            
            # Transform and pack points
            points_data = []
            for obstacle in obstacles:
                # Create point in camera frame
                point_camera = PoseStamped()
                point_camera.header.frame_id = 'camera_link'
                point_camera.header.stamp = timestamp
                point_camera.pose.position.x = obstacle[0]
                point_camera.pose.position.y = obstacle[1]
                point_camera.pose.position.z = obstacle[2]
                
                # Transform to map frame
                try:
                    point_map = tf2_geometry_msgs.do_transform_pose(point_camera, transform)
                    
                    # Pack as bytes
                    x_bytes = np.array([point_map.pose.position.x], dtype=np.float32).tobytes()
                    y_bytes = np.array([point_map.pose.position.y], dtype=np.float32).tobytes()
                    z_bytes = np.array([point_map.pose.position.z], dtype=np.float32).tobytes()
                    points_data.append(x_bytes + y_bytes + z_bytes)
                except:
                    continue
            
            cloud_msg.data = b''.join(points_data)
            cloud_msg.width = len(points_data)
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            
            self.obstacles_pub.publish(cloud_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Could not transform obstacles to map frame: {e}")
    
    def publish_detections(self, detections):
        """Publish detection summary"""
        detection_msg = String()
        detection_msg.data = "; ".join(detections) if detections else "No obstacles detected"
        self.detections_pub.publish(detection_msg)
    
    def publish_annotated_image(self, cv_image, header):
        """Publish annotated image"""
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            annotated_msg.header = header
            self.annotated_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing annotated image: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = VisionObstacleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
