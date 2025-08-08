#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import time
from std_msgs.msg import Float32

class PerformanceMonitor(Node):
    """
    Monitor system performance for Cahier des Charges compliance:
    - Latency < 200ms requirement
    - Processing time measurement
    """
    
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Subscribers for timing measurement
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.vision_obstacles_sub = self.create_subscription(
            PointCloud2, '/vision_obstacles', self.vision_callback, 10)
        
        # Publishers for performance metrics
        self.latency_pub = self.create_publisher(Float32, '/performance/latency_ms', 10)
        self.fps_pub = self.create_publisher(Float32, '/performance/fps', 10)
        
        # Performance tracking
        self.last_image_time = None
        self.last_vision_time = None
        self.frame_count = 0
        self.start_time = time.time()
        
        # Timer for periodic reporting
        self.timer = self.create_timer(1.0, self.report_performance)
        
        self.get_logger().info('Performance Monitor started - tracking latency for Cahier des Charges compliance')
    
    def image_callback(self, msg):
        """Track camera input timing"""
        self.last_image_time = time.time()
        self.frame_count += 1
    
    def laser_callback(self, msg):
        """Track laser scan timing"""
        pass  # LiDAR timing if needed
    
    def vision_callback(self, msg):
        """Track vision processing latency"""
        current_time = time.time()
        
        if self.last_image_time is not None:
            # Calculate processing latency from image to vision output
            latency_ms = (current_time - self.last_image_time) * 1000
            
            # Publish latency metric
            latency_msg = Float32()
            latency_msg.data = latency_ms
            self.latency_pub.publish(latency_msg)
            
            # Check compliance with < 200ms requirement
            status = "✅ COMPLIANT" if latency_ms < 200 else "❌ NON-COMPLIANT"
            self.get_logger().info(f'Vision Processing Latency: {latency_ms:.1f}ms {status}')
        
        self.last_vision_time = current_time
    
    def report_performance(self):
        """Report FPS and overall performance"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            fps_msg = Float32()
            fps_msg.data = fps
            self.fps_pub.publish(fps_msg)
            
            self.get_logger().info(f'Camera FPS: {fps:.1f}')
        
        # Reset counters every minute
        if elapsed > 60:
            self.frame_count = 0
            self.start_time = current_time

def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
