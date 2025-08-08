#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    vision_dir = get_package_share_directory('autonombot_vision')
    nav_dir = get_package_share_directory('autonombot_navigation')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_vision = LaunchConfiguration('use_vision')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_dir, 'config', 'planner_server.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )
    
    declare_use_vision_cmd = DeclareLaunchArgument(
        'use_vision',
        default_value='true',
        description='Enable vision-based obstacle detection'
    )
    
    # Specify the actions
    
    # Vision Obstacle Detector
    vision_obstacle_detector_cmd = Node(
        condition=IfCondition(use_vision),
        package='autonombot_vision',
        executable='vision_obstacle_detector',
        name='vision_obstacle_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': 'yolov8n.pt',
            'confidence_threshold': 0.4,
            'camera_height': 0.2,
            'camera_range': 3.0,
            'obstacle_classes': ['chair', 'couch', 'person', 'refrigerator', 'dining table', 'bottle', 'book']
        }]
    )
    
    # Vision Costmap Updater
    vision_costmap_updater_cmd = Node(
        condition=IfCondition(use_vision),
        package='autonombot_vision',
        executable='vision_costmap_updater',
        name='vision_costmap_updater',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'obstacle_cost': 90,
            'obstacle_radius': 0.4,
            'decay_rate': 0.98,
            'update_rate': 5.0
        }]
    )
    
    # Regular YOLO Detector (for visualization)
    yolo_detector_cmd = Node(
        condition=IfCondition(use_vision),
        package='autonombot_vision',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': 'yolov8n.pt',
            'confidence_threshold': 0.5
        }]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_vision_cmd)
    
    # Add the actions to launch the nodes
    ld.add_action(vision_obstacle_detector_cmd)
    ld.add_action(vision_costmap_updater_cmd)
    ld.add_action(yolo_detector_cmd)
    
    return ld
