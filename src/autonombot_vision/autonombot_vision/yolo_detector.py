#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed. Install with: pip install ultralytics")

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')  # Nano model for speed
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/camera/image_raw')
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        # Initialize YOLO model
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f"YOLO model loaded: {self.model_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO model: {e}")
                self.model = None
        else:
            self.model = None
            self.get_logger().warn("YOLO not available - running in demo mode")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String,
            '/detections',
            10
        )
        
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/camera/annotated_image',
            10
        )
        
        # COCO class names (common objects)
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
            'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
            'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
            'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
            'toothbrush'
        ]
        
        self.get_logger().info("YOLO Detector Node Started!")
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if self.model is not None:
                # Run YOLO inference
                results = self.model(cv_image, conf=self.confidence_threshold)
                
                # Process results
                detections = []
                annotated_image = cv_image.copy()
                
                for result in results:
                    boxes = result.boxes
                    if boxes is not None:
                        for box in boxes:
                            # Get box coordinates
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            confidence = box.conf[0].cpu().numpy()
                            class_id = int(box.cls[0].cpu().numpy())
                            
                            # Get class name
                            if class_id < len(self.class_names):
                                class_name = self.class_names[class_id]
                            else:
                                class_name = f"class_{class_id}"
                            
                            # Add to detections
                            detection_info = f"{class_name}: {confidence:.2f}"
                            detections.append(detection_info)
                            
                            # Draw bounding box
                            cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(annotated_image, detection_info, (int(x1), int(y1-10)), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Publish detections
                if detections:
                    detection_msg = String()
                    detection_msg.data = "; ".join(detections)
                    self.detection_pub.publish(detection_msg)
                    self.get_logger().info(f"Detected: {detection_msg.data}")
                
                # Publish annotated image
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                self.annotated_image_pub.publish(annotated_msg)
                
            else:
                # Demo mode - just add text overlay
                demo_image = cv_image.copy()
                cv2.putText(demo_image, "YOLO Demo Mode", (50, 50), 
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(demo_image, "Install ultralytics for detection", (50, 100), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                demo_msg = self.bridge.cv2_to_imgmsg(demo_image, "bgr8")
                self.annotated_image_pub.publish(demo_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    detector = YOLODetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
