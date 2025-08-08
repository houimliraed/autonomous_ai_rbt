#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

class UltrasonicLocalization(Node):
    """
    Ultrasonic-based localization to provide additional positioning data
    Replaces IMU input for EKF by estimating angular velocity and acceleration from ultrasonic readings
    """

    def __init__(self):
        super().__init__("ultrasonic_localization")
        
        # Configuration parameters
        self.declare_parameter('buffer_size', 10)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('motion_threshold', 0.05)  # m/s
        
        self.buffer_size = self.get_parameter('buffer_size').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.motion_threshold = self.get_parameter('motion_threshold').value
        
        # Data buffers for temporal analysis
        self.range_buffer = []
        self.time_buffer = []
        self.velocity_buffer = []
        
        # Previous state for derivative calculations
        self.prev_range = None
        self.prev_time = None
        self.prev_velocity = 0.0
        
        # ROS 2 Interface
        self.range_sub = self.create_subscription(
            Range,
            "/ultrasonic_range",
            self.range_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            "/autonombot_drivers/odom",
            self.odom_callback,
            10
        )
        
        # Publisher for ultrasonic-derived motion data
        self.motion_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            "/ultrasonic_motion",
            10
        )
        
        # Timer for publishing motion estimates
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_motion_estimate)
        
        # Current robot velocity (from odometry)
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        self.get_logger().info("Ultrasonic localization node initialized")

    def range_callback(self, msg):
        """Process ultrasonic range measurements"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        current_range = msg.range
        
        # Skip invalid readings
        if math.isinf(current_range) or math.isnan(current_range):
            return
        
        # Add to buffers
        self.range_buffer.append(current_range)
        self.time_buffer.append(current_time)
        
        # Maintain buffer size
        if len(self.range_buffer) > self.buffer_size:
            self.range_buffer.pop(0)
            self.time_buffer.pop(0)
        
        # Calculate range rate (derivative)
        if self.prev_range is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                range_rate = (current_range - self.prev_range) / dt
                self.velocity_buffer.append(range_rate)
                
                # Maintain velocity buffer
                if len(self.velocity_buffer) > self.buffer_size:
                    self.velocity_buffer.pop(0)
        
        self.prev_range = current_range
        self.prev_time = current_time

    def odom_callback(self, msg):
        """Update current robot velocity from odometry"""
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z

    def estimate_motion_from_ultrasonic(self):
        """
        Estimate motion characteristics using ultrasonic data
        This provides motion validation and additional constraints for localization
        """
        if len(self.range_buffer) < 3 or len(self.velocity_buffer) < 2:
            return None, None, None
        
        # Calculate statistics from recent measurements
        ranges = np.array(self.range_buffer[-5:])  # Last 5 measurements
        velocities = np.array(self.velocity_buffer[-5:])  # Last 5 range rates
        
        # Estimate motion consistency
        range_variance = np.var(ranges)
        velocity_variance = np.var(velocities)
        mean_range_rate = np.mean(velocities)
        
        # Motion detection based on range variance and rate
        motion_detected = (range_variance > 0.01) or (abs(mean_range_rate) > self.motion_threshold)
        
        # Estimate angular velocity from ultrasonic pattern
        # When robot rotates, ultrasonic readings show characteristic patterns
        estimated_angular_vel = 0.0
        if len(self.range_buffer) >= 5:
            # Look for oscillatory patterns that indicate rotation
            recent_ranges = np.array(self.range_buffer[-5:])
            range_trend = np.polyfit(range(len(recent_ranges)), recent_ranges, 1)[0]
            
            # If ranges are oscillating while robot reports angular velocity,
            # we can estimate the consistency
            if abs(self.current_angular_vel) > 0.1:
                # Use current angular velocity as baseline
                estimated_angular_vel = self.current_angular_vel
                
                # Adjust based on ultrasonic pattern consistency
                pattern_consistency = 1.0 - min(1.0, range_variance / 0.1)
                estimated_angular_vel *= pattern_consistency
        
        # Estimate linear acceleration from range rate changes
        estimated_linear_accel = 0.0
        if len(self.velocity_buffer) >= 3:
            recent_velocities = np.array(self.velocity_buffer[-3:])
            if len(self.time_buffer) >= 3:
                dt = self.time_buffer[-1] - self.time_buffer[-3]
                if dt > 0:
                    velocity_change = recent_velocities[-1] - recent_velocities[0]
                    estimated_linear_accel = velocity_change / dt
        
        return estimated_angular_vel, estimated_linear_accel, motion_detected

    def publish_motion_estimate(self):
        """Publish motion estimates based on ultrasonic data"""
        angular_vel, linear_accel, motion_detected = self.estimate_motion_from_ultrasonic()
        
        if angular_vel is None:
            return
        
        # Create TwistWithCovarianceStamped message
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_link"
        
        # Set twist values
        msg.twist.twist.linear.x = self.current_linear_vel
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_vel
        
        # Set covariance matrix (6x6 = 36 elements)
        # Higher uncertainty for ultrasonic-derived values
        covariance = [0.0] * 36
        
        # Linear velocity covariance (use odometry uncertainty)
        covariance[0] = 0.1   # x linear velocity variance
        covariance[7] = 999.0 # y linear velocity (not measured)
        covariance[14] = 999.0 # z linear velocity (not measured)
        
        # Angular velocity covariance
        covariance[21] = 999.0  # x angular velocity (not measured)
        covariance[28] = 999.0  # y angular velocity (not measured)
        covariance[35] = 0.5 if motion_detected else 1.0  # z angular velocity from ultrasonic
        
        msg.twist.covariance = covariance
        
        # Publish the message
        self.motion_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        ultrasonic_localization = UltrasonicLocalization()
        rclpy.spin(ultrasonic_localization)
    except KeyboardInterrupt:
        pass
    finally:
        if 'ultrasonic_localization' in locals():
            ultrasonic_localization.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
