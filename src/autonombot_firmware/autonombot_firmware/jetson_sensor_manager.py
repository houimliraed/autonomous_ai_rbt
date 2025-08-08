#!/usr/bin/env python3
"""
Jetson Xavier Sensor Integration Manager
Handles initialization and monitoring of all sensors:
- HC-SR04 Ultrasonic Sensor
- RealSense 3D Camera
- RPLiDAR A1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Range, LaserScan, Image, PointCloud2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time
import threading

class JetsonSensorManager(Node):
    """
    Centralized sensor management for Jetson Xavier deployment
    Monitors all sensors and provides system health diagnostics
    """

    def __init__(self):
        super().__init__("jetson_sensor_manager")
        
        # Configuration parameters
        self.declare_parameter('enable_ultrasonic', True)
        self.declare_parameter('enable_lidar', True)
        self.declare_parameter('enable_camera', True)
        self.declare_parameter('diagnostics_frequency', 1.0)  # Hz
        self.declare_parameter('sensor_timeout', 5.0)  # seconds
        
        # Get parameters
        self.enable_ultrasonic = self.get_parameter('enable_ultrasonic').value
        self.enable_lidar = self.get_parameter('enable_lidar').value
        self.enable_camera = self.get_parameter('enable_camera').value
        self.diag_freq = self.get_parameter('diagnostics_frequency').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        
        # Sensor status tracking
        self.sensor_status = {
            'ultrasonic': {'active': False, 'last_msg': 0, 'msg_count': 0, 'errors': 0},
            'lidar': {'active': False, 'last_msg': 0, 'msg_count': 0, 'errors': 0},
            'camera_color': {'active': False, 'last_msg': 0, 'msg_count': 0, 'errors': 0},
            'camera_depth': {'active': False, 'last_msg': 0, 'msg_count': 0, 'errors': 0}
        }
        
        self.lock = threading.Lock()
        
        # QoS profiles for different sensor types
        sensor_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Sensor subscribers
        if self.enable_ultrasonic:
            self.ultrasonic_sub = self.create_subscription(
                Range,
                '/ultrasonic_range',
                self.ultrasonic_callback,
                qos_profile_sensor_data
            )
        
        if self.enable_lidar:
            self.lidar_sub = self.create_subscription(
                LaserScan,
                '/scan',
                self.lidar_callback,
                sensor_qos
            )
        
        if self.enable_camera:
            self.camera_color_sub = self.create_subscription(
                Image,
                '/camera/color/image_raw',
                self.camera_color_callback,
                sensor_qos
            )
            
            self.camera_depth_sub = self.create_subscription(
                Image,
                '/camera/depth/image_rect_raw',
                self.camera_depth_callback,
                sensor_qos
            )
        
        # Diagnostics publisher
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # System status publisher
        self.status_pub = self.create_publisher(
            String,
            '/sensor_manager/status',
            10
        )
        
        # Diagnostics timer
        self.diagnostics_timer = self.create_timer(
            1.0 / self.diag_freq,
            self.publish_diagnostics
        )
        
        self.get_logger().info("Jetson Xavier Sensor Manager initialized")
        self.get_logger().info(f"Enabled sensors: Ultrasonic={self.enable_ultrasonic}, LiDAR={self.enable_lidar}, Camera={self.enable_camera}")

    def ultrasonic_callback(self, msg):
        """Handle ultrasonic sensor data"""
        with self.lock:
            current_time = time.time()
            self.sensor_status['ultrasonic']['active'] = True
            self.sensor_status['ultrasonic']['last_msg'] = current_time
            self.sensor_status['ultrasonic']['msg_count'] += 1
            
            # Check for sensor errors (out of range measurements)
            if msg.range == float('inf') or msg.range < msg.min_range or msg.range > msg.max_range:
                self.sensor_status['ultrasonic']['errors'] += 1

    def lidar_callback(self, msg):
        """Handle LiDAR sensor data"""
        with self.lock:
            current_time = time.time()
            self.sensor_status['lidar']['active'] = True
            self.sensor_status['lidar']['last_msg'] = current_time
            self.sensor_status['lidar']['msg_count'] += 1
            
            # Check for LiDAR data quality
            valid_points = sum(1 for r in msg.ranges if msg.range_min <= r <= msg.range_max)
            if valid_points < len(msg.ranges) * 0.5:  # Less than 50% valid points
                self.sensor_status['lidar']['errors'] += 1

    def camera_color_callback(self, msg):
        """Handle camera color image data"""
        with self.lock:
            current_time = time.time()
            self.sensor_status['camera_color']['active'] = True
            self.sensor_status['camera_color']['last_msg'] = current_time
            self.sensor_status['camera_color']['msg_count'] += 1

    def camera_depth_callback(self, msg):
        """Handle camera depth image data"""
        with self.lock:
            current_time = time.time()
            self.sensor_status['camera_depth']['active'] = True
            self.sensor_status['camera_depth']['last_msg'] = current_time
            self.sensor_status['camera_depth']['msg_count'] += 1

    def publish_diagnostics(self):
        """Publish comprehensive sensor diagnostics"""
        current_time = time.time()
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        overall_status = DiagnosticStatus.OK
        active_sensors = 0
        total_sensors = 0
        
        with self.lock:
            # Check each sensor
            for sensor_name, status in self.sensor_status.items():
                if not self.is_sensor_enabled(sensor_name):
                    continue
                    
                total_sensors += 1
                diag_status = DiagnosticStatus()
                diag_status.name = f"Sensor: {sensor_name}"
                diag_status.hardware_id = "jetson_xavier"
                
                # Check if sensor is active and recent
                time_since_last = current_time - status['last_msg']
                
                if not status['active']:
                    diag_status.level = DiagnosticStatus.ERROR
                    diag_status.message = "Sensor not detected"
                elif time_since_last > self.sensor_timeout:
                    diag_status.level = DiagnosticStatus.ERROR
                    diag_status.message = f"No data for {time_since_last:.1f}s"
                    overall_status = max(overall_status, DiagnosticStatus.ERROR)
                elif status['errors'] > status['msg_count'] * 0.1:  # >10% error rate
                    diag_status.level = DiagnosticStatus.WARN
                    diag_status.message = f"High error rate: {status['errors']}/{status['msg_count']}"
                    overall_status = max(overall_status, DiagnosticStatus.WARN)
                else:
                    diag_status.level = DiagnosticStatus.OK
                    diag_status.message = "Operating normally"
                    active_sensors += 1
                
                # Add detailed values
                diag_status.values = [
                    KeyValue(key="message_count", value=str(status['msg_count'])),
                    KeyValue(key="error_count", value=str(status['errors'])),
                    KeyValue(key="last_message_age", value=f"{time_since_last:.2f}s"),
                    KeyValue(key="frequency", value=f"{status['msg_count']/(current_time-self.start_time) if hasattr(self, 'start_time') else 0:.1f}Hz")
                ]
                
                diag_array.status.append(diag_status)
        
        # Overall system status
        system_diag = DiagnosticStatus()
        system_diag.name = "Sensor System"
        system_diag.hardware_id = "jetson_xavier"
        system_diag.level = overall_status
        system_diag.message = f"{active_sensors}/{total_sensors} sensors active"
        system_diag.values = [
            KeyValue(key="active_sensors", value=str(active_sensors)),
            KeyValue(key="total_sensors", value=str(total_sensors)),
            KeyValue(key="system_uptime", value=f"{current_time-self.start_time if hasattr(self, 'start_time') else 0:.0f}s")
        ]
        diag_array.status.append(system_diag)
        
        # Publish diagnostics
        self.diagnostics_pub.publish(diag_array)
        
        # Publish simple status message
        status_msg = String()
        if overall_status == DiagnosticStatus.OK:
            status_msg.data = f"ALL_OK: {active_sensors}/{total_sensors} sensors active"
        elif overall_status == DiagnosticStatus.WARN:
            status_msg.data = f"WARNING: {active_sensors}/{total_sensors} sensors active with issues"
        else:
            status_msg.data = f"ERROR: {active_sensors}/{total_sensors} sensors active, system degraded"
        
        self.status_pub.publish(status_msg)
        
        # Store start time for uptime calculation
        if not hasattr(self, 'start_time'):
            self.start_time = current_time

    def is_sensor_enabled(self, sensor_name):
        """Check if a sensor is enabled"""
        if sensor_name == 'ultrasonic':
            return self.enable_ultrasonic
        elif sensor_name == 'lidar':
            return self.enable_lidar
        elif sensor_name in ['camera_color', 'camera_depth']:
            return self.enable_camera
        return False


def main(args=None):
    rclpy.init(args=args)
    try:
        sensor_manager = JetsonSensorManager()
        rclpy.spin(sensor_manager)
    except KeyboardInterrupt:
        pass
    finally:
        if 'sensor_manager' in locals():
            sensor_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
