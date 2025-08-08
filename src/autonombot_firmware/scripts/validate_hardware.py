#!/usr/bin/env python3
"""
Hardware Validation Script for Jetson Xavier Autonombot
Tests all sensors and hardware components before deployment
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range, LaserScan, Image
from std_msgs.msg import String
import time
import threading
import sys

class HardwareValidator(Node):
    """
    Comprehensive hardware validation for production deployment
    """

    def __init__(self):
        super().__init__("hardware_validator")
        
        self.test_results = {
            'ultrasonic': {'status': 'PENDING', 'messages': 0, 'last_value': None},
            'lidar': {'status': 'PENDING', 'messages': 0, 'last_value': None},
            'camera_color': {'status': 'PENDING', 'messages': 0, 'last_value': None},
            'camera_depth': {'status': 'PENDING', 'messages': 0, 'last_value': None}
        }
        
        self.start_time = time.time()
        self.test_duration = 10.0  # seconds
        self.lock = threading.Lock()
        
        # Create subscribers for all sensors
        self.create_sensor_subscribers()
        
        # Create timer for validation checks
        self.validation_timer = self.create_timer(1.0, self.check_validation_status)
        
        self.get_logger().info("Hardware validation started - testing for 10 seconds...")
        self.get_logger().info("Expected sensors: Ultrasonic, LiDAR, RealSense Camera")

    def create_sensor_subscribers(self):
        """Create subscribers for all expected sensors"""
        
        # Ultrasonic sensor
        self.ultrasonic_sub = self.create_subscription(
            Range,
            '/ultrasonic_range',
            self.ultrasonic_callback,
            qos_profile_sensor_data
        )
        
        # LiDAR sensor
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Camera color stream
        self.camera_color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_color_callback,
            10
        )
        
        # Camera depth stream
        self.camera_depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.camera_depth_callback,
            10
        )

    def ultrasonic_callback(self, msg):
        """Handle ultrasonic sensor validation"""
        with self.lock:
            self.test_results['ultrasonic']['messages'] += 1
            self.test_results['ultrasonic']['last_value'] = msg.range
            
            # Validate data quality
            if msg.range == float('inf'):
                self.test_results['ultrasonic']['status'] = 'ERROR_OUT_OF_RANGE'
            elif msg.range < msg.min_range or msg.range > msg.max_range:
                self.test_results['ultrasonic']['status'] = 'ERROR_INVALID_RANGE'
            elif self.test_results['ultrasonic']['messages'] >= 5:
                self.test_results['ultrasonic']['status'] = 'PASS'

    def lidar_callback(self, msg):
        """Handle LiDAR sensor validation"""
        with self.lock:
            self.test_results['lidar']['messages'] += 1
            
            # Check data quality
            valid_points = sum(1 for r in msg.ranges if msg.range_min <= r <= msg.range_max)
            total_points = len(msg.ranges)
            valid_percentage = (valid_points / total_points) * 100 if total_points > 0 else 0
            
            self.test_results['lidar']['last_value'] = {
                'total_points': total_points,
                'valid_points': valid_points,
                'valid_percentage': valid_percentage
            }
            
            if valid_percentage < 20:  # Less than 20% valid points
                self.test_results['lidar']['status'] = 'ERROR_LOW_QUALITY'
            elif total_points < 100:  # Very few scan points
                self.test_results['lidar']['status'] = 'ERROR_INSUFFICIENT_DATA'
            elif self.test_results['lidar']['messages'] >= 3:
                self.test_results['lidar']['status'] = 'PASS'

    def camera_color_callback(self, msg):
        """Handle camera color stream validation"""
        with self.lock:
            self.test_results['camera_color']['messages'] += 1
            self.test_results['camera_color']['last_value'] = {
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'data_size': len(msg.data)
            }
            
            # Validate image properties
            if msg.width < 320 or msg.height < 240:
                self.test_results['camera_color']['status'] = 'ERROR_LOW_RESOLUTION'
            elif len(msg.data) == 0:
                self.test_results['camera_color']['status'] = 'ERROR_NO_DATA'
            elif self.test_results['camera_color']['messages'] >= 3:
                self.test_results['camera_color']['status'] = 'PASS'

    def camera_depth_callback(self, msg):
        """Handle camera depth stream validation"""
        with self.lock:
            self.test_results['camera_depth']['messages'] += 1
            self.test_results['camera_depth']['last_value'] = {
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'data_size': len(msg.data)
            }
            
            # Validate depth image properties
            if msg.width < 320 or msg.height < 240:
                self.test_results['camera_depth']['status'] = 'ERROR_LOW_RESOLUTION'
            elif len(msg.data) == 0:
                self.test_results['camera_depth']['status'] = 'ERROR_NO_DATA'
            elif self.test_results['camera_depth']['messages'] >= 3:
                self.test_results['camera_depth']['status'] = 'PASS'

    def check_validation_status(self):
        """Check current validation status and provide updates"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed > self.test_duration:
            self.complete_validation()
            return
        
        # Print periodic status updates
        if int(elapsed) % 2 == 0 and elapsed > 1:  # Every 2 seconds after first second
            with self.lock:
                self.print_status_update(elapsed)

    def print_status_update(self, elapsed):
        """Print current validation status"""
        remaining = self.test_duration - elapsed
        self.get_logger().info(f"Validation in progress... {remaining:.0f}s remaining")
        
        for sensor, result in self.test_results.items():
            status = result['status']
            msg_count = result['messages']
            
            if status == 'PASS':
                status_str = "✓ PASS"
            elif status == 'PENDING':
                status_str = "⏳ PENDING"
            else:
                status_str = f"✗ {status}"
            
            self.get_logger().info(f"  {sensor.ljust(15)}: {status_str} ({msg_count} msgs)")

    def complete_validation(self):
        """Complete validation and print final results"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("HARDWARE VALIDATION COMPLETE")
        self.get_logger().info("=" * 60)
        
        all_passed = True
        
        with self.lock:
            for sensor, result in self.test_results.items():
                status = result['status']
                msg_count = result['messages']
                last_value = result['last_value']
                
                self.get_logger().info(f"\n{sensor.upper()} SENSOR:")
                self.get_logger().info(f"  Status: {status}")
                self.get_logger().info(f"  Messages received: {msg_count}")
                
                if status == 'PASS':
                    self.get_logger().info("  Result: ✓ OPERATIONAL")
                    
                    # Print sensor-specific details
                    if sensor == 'ultrasonic' and last_value is not None:
                        self.get_logger().info(f"  Last range: {last_value:.3f}m")
                    elif sensor == 'lidar' and last_value is not None:
                        self.get_logger().info(f"  Scan points: {last_value['total_points']}")
                        self.get_logger().info(f"  Valid data: {last_value['valid_percentage']:.1f}%")
                    elif 'camera' in sensor and last_value is not None:
                        self.get_logger().info(f"  Resolution: {last_value['width']}x{last_value['height']}")
                        self.get_logger().info(f"  Encoding: {last_value['encoding']}")
                        
                elif status == 'PENDING':
                    self.get_logger().info("  Result: ✗ NO DATA RECEIVED")
                    self.get_logger().info("  Check: Sensor connection, driver, and topic names")
                    all_passed = False
                    
                else:
                    self.get_logger().info(f"  Result: ✗ {status}")
                    all_passed = False
        
        self.get_logger().info("\n" + "=" * 60)
        
        if all_passed:
            self.get_logger().info("🎉 ALL SENSORS OPERATIONAL - ROBOT READY FOR DEPLOYMENT!")
            self.get_logger().info("Next steps:")
            self.get_logger().info("1. Launch full navigation: ros2 launch autonombot_launch real_robot.launch.py")
            self.get_logger().info("2. Monitor diagnostics: ros2 topic echo /diagnostics")
            exit_code = 0
        else:
            self.get_logger().info("❌ HARDWARE VALIDATION FAILED")
            self.get_logger().info("Fix the failed sensors before deploying the robot.")
            self.get_logger().info("Check connections, power, and driver installations.")
            exit_code = 1
        
        self.get_logger().info("=" * 60)
        
        # Shutdown the node
        rclpy.shutdown()
        sys.exit(exit_code)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        validator = HardwareValidator()
        rclpy.spin(validator)
    except KeyboardInterrupt:
        print("\nValidation interrupted by user")
    except Exception as e:
        print(f"Validation failed with error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
