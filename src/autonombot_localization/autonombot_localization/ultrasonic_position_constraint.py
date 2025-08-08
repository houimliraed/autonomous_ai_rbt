#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import math

class UltrasonicPositionConstraint(Node):
    """
    Provides position constraints based on ultrasonic obstacle detection
    Helps validate robot position by detecting consistent obstacles
    """

    def __init__(self):
        super().__init__("ultrasonic_position_constraint")
        
        # Configuration
        self.declare_parameter('min_obstacle_distance', 0.3)  # meters
        self.declare_parameter('max_obstacle_distance', 2.0)  # meters
        self.declare_parameter('constraint_strength', 0.1)    # how much to trust constraints
        
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.max_obstacle_distance = self.get_parameter('max_obstacle_distance').value
        self.constraint_strength = self.get_parameter('constraint_strength').value
        
        # Current state
        self.current_range = None
        self.last_valid_obstacle_distance = None
        self.position_constraint = 0.0
        
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
        
        # Publisher for position constraints
        self.constraint_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            "/ultrasonic_position_constraint",
            10
        )
        
        # Timer for publishing constraints
        self.timer = self.create_timer(0.1, self.publish_constraint)  # 10Hz
        
        self.get_logger().info("Ultrasonic position constraint node initialized")

    def range_callback(self, msg):
        """Process ultrasonic range measurements"""
        if not math.isinf(msg.range) and not math.isnan(msg.range):
            self.current_range = msg.range
            
            # Update obstacle tracking
            if self.min_obstacle_distance <= msg.range <= self.max_obstacle_distance:
                self.last_valid_obstacle_distance = msg.range

    def odom_callback(self, msg):
        """Process odometry for position validation"""
        # Extract current position and velocity
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_vel_x = msg.twist.twist.linear.x
        
        # Calculate position constraint based on obstacle consistency
        if self.current_range is not None and self.last_valid_obstacle_distance is not None:
            # If we're moving toward an obstacle and distance is decreasing appropriately
            if current_vel_x > 0.1:  # Moving forward
                expected_range_change = -current_vel_x * 0.1  # Expected change over 100ms
                actual_range = self.current_range
                
                # Calculate constraint strength based on consistency
                if self.min_obstacle_distance <= actual_range <= self.max_obstacle_distance:
                    # Strong constraint when obstacle is detected consistently
                    self.position_constraint = self.constraint_strength
                else:
                    # Weaker constraint when no consistent obstacle
                    self.position_constraint = self.constraint_strength * 0.5
            else:
                # Default constraint when not moving or moving backward
                self.position_constraint = self.constraint_strength * 0.3

    def publish_constraint(self):
        """Publish position constraints based on ultrasonic data"""
        if self.current_range is None:
            return
        
        # Create constraint message
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_link"
        
        # Set position constraint (primarily in x-direction for forward-facing sensor)
        msg.twist.twist.linear.x = 0.0  # No velocity constraint
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        
        # Set covariance matrix for position constraints
        covariance = [0.0] * 36
        
        # Position constraint strength based on obstacle detection
        if self.min_obstacle_distance <= self.current_range <= self.max_obstacle_distance:
            # Strong constraint when obstacle is detected
            constraint_variance = 0.1 * (1.0 - self.position_constraint)
        else:
            # Weak constraint when no obstacle or out of range
            constraint_variance = 1.0
        
        # Apply constraint primarily to x-position (forward direction)
        covariance[0] = constraint_variance    # x position variance
        covariance[7] = 999.0   # y position (unconstrained)
        covariance[14] = 999.0  # z position (unconstrained)
        covariance[21] = 999.0  # x rotation (unconstrained)
        covariance[28] = 999.0  # y rotation (unconstrained)
        covariance[35] = 999.0  # z rotation (unconstrained)
        
        msg.twist.covariance = covariance
        
        # Publish constraint
        self.constraint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        constraint_node = UltrasonicPositionConstraint()
        rclpy.spin(constraint_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'constraint_node' in locals():
            constraint_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
