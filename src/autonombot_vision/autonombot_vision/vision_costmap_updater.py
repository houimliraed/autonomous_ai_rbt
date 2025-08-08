#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import numpy as np
import struct
import math

class VisionCostmapUpdater(Node):
    def __init__(self):
        super().__init__('vision_costmap_updater')
        
        # Parameters
        self.declare_parameter('obstacle_cost', 100)  # Cost value for detected obstacles
        self.declare_parameter('obstacle_radius', 0.3)  # Radius around each obstacle point
        self.declare_parameter('decay_rate', 0.95)  # How quickly old obstacles fade
        self.declare_parameter('update_rate', 5.0)  # Hz
        
        # Get parameters
        self.obstacle_cost = self.get_parameter('obstacle_cost').get_parameter_value().integer_value
        self.obstacle_radius = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        self.decay_rate = self.get_parameter('decay_rate').get_parameter_value().double_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        # Internal state
        self.vision_obstacles = []
        self.last_costmap = None
        
        # Subscribers
        self.obstacles_sub = self.create_subscription(
            PointCloud2,
            '/vision_obstacles',
            self.obstacles_callback,
            10
        )
        
        # Publishers
        self.vision_costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/vision_costmap',
            10
        )
        
        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / update_rate, self.update_costmap)
        
        self.get_logger().info("Vision Costmap Updater initialized")
    
    def obstacles_callback(self, msg):
        """Process incoming vision obstacles"""
        try:
            # Parse PointCloud2 data
            obstacles = self.parse_pointcloud(msg)
            
            # Update obstacle list with timestamp
            current_time = self.get_clock().now()
            self.vision_obstacles = []
            
            for point in obstacles:
                self.vision_obstacles.append({
                    'x': point[0],
                    'y': point[1],
                    'z': point[2],
                    'timestamp': current_time,
                    'cost': self.obstacle_cost
                })
            
            self.get_logger().debug(f"Updated with {len(obstacles)} vision obstacles")
            
        except Exception as e:
            self.get_logger().error(f"Error processing obstacles: {e}")
    
    def parse_pointcloud(self, cloud_msg):
        """Parse PointCloud2 message to extract points"""
        points = []
        
        # Get field offsets
        x_offset = y_offset = z_offset = None
        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset is None or y_offset is None or z_offset is None:
            return points
        
        # Extract points
        point_step = cloud_msg.point_step
        for i in range(cloud_msg.width):
            base_offset = i * point_step
            
            # Extract x, y, z coordinates
            x_data = cloud_msg.data[base_offset + x_offset:base_offset + x_offset + 4]
            y_data = cloud_msg.data[base_offset + y_offset:base_offset + y_offset + 4]
            z_data = cloud_msg.data[base_offset + z_offset:base_offset + z_offset + 4]
            
            x = struct.unpack('f', x_data)[0]
            y = struct.unpack('f', y_data)[0]
            z = struct.unpack('f', z_data)[0]
            
            points.append([x, y, z])
        
        return points
    
    def update_costmap(self):
        """Periodically update and publish vision costmap"""
        if not self.vision_obstacles:
            return
        
        try:
            # Create a simple costmap around detected obstacles
            # For now, create a fixed-size map centered around robot
            map_width = 100  # cells
            map_height = 100  # cells
            resolution = 0.1  # meters per cell
            origin_x = -5.0  # meters
            origin_y = -5.0  # meters
            
            # Initialize costmap
            costmap_data = np.zeros((map_height, map_width), dtype=np.int8)
            
            # Current time for decay calculation
            current_time = self.get_clock().now()
            
            # Add obstacles to costmap
            for obstacle in self.vision_obstacles:
                # Calculate age-based decay
                age_seconds = (current_time - obstacle['timestamp']).nanoseconds / 1e9
                decay_factor = self.decay_rate ** age_seconds
                current_cost = int(obstacle['cost'] * decay_factor)
                
                if current_cost < 10:  # Skip very faded obstacles
                    continue
                
                # Convert world coordinates to grid coordinates
                grid_x = int((obstacle['x'] - origin_x) / resolution)
                grid_y = int((obstacle['y'] - origin_y) / resolution)
                
                # Add obstacle with radius
                radius_cells = int(self.obstacle_radius / resolution)
                
                for dx in range(-radius_cells, radius_cells + 1):
                    for dy in range(-radius_cells, radius_cells + 1):
                        x = grid_x + dx
                        y = grid_y + dy
                        
                        # Check bounds
                        if 0 <= x < map_width and 0 <= y < map_height:
                            # Calculate distance-based cost
                            distance = math.sqrt(dx*dx + dy*dy) * resolution
                            if distance <= self.obstacle_radius:
                                # Linear cost falloff
                                cost_factor = 1.0 - (distance / self.obstacle_radius)
                                cost = int(current_cost * cost_factor)
                                costmap_data[y, x] = max(costmap_data[y, x], cost)
            
            # Create and publish OccupancyGrid message
            costmap_msg = OccupancyGrid()
            costmap_msg.header.stamp = current_time.to_msg()
            costmap_msg.header.frame_id = 'map'
            
            costmap_msg.info.resolution = resolution
            costmap_msg.info.width = map_width
            costmap_msg.info.height = map_height
            costmap_msg.info.origin.position.x = origin_x
            costmap_msg.info.origin.position.y = origin_y
            costmap_msg.info.origin.position.z = 0.0
            costmap_msg.info.origin.orientation.w = 1.0
            
            # Flatten and convert to list
            costmap_msg.data = costmap_data.flatten().tolist()
            
            self.vision_costmap_pub.publish(costmap_msg)
            self.last_costmap = costmap_msg
            
        except Exception as e:
            self.get_logger().error(f"Error updating costmap: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = VisionCostmapUpdater()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
