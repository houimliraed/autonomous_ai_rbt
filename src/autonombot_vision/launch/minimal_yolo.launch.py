from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Minimal YOLO obstacle detection
        Node(
            package='autonombot_vision',
            executable='minimal_yolo_obstacles',
            name='minimal_yolo_obstacles',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
    ])
