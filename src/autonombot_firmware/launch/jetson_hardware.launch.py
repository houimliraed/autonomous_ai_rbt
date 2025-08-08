import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file optimized for Jetson Xavier hardware deployment
    Includes all necessary drivers and monitoring for production robot
    """
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino communication'
    )
    
    ultrasonic_freq_arg = DeclareLaunchArgument(
        'ultrasonic_frequency',
        default_value='20.0',
        description='Ultrasonic sensor measurement frequency (Hz)'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='True',
        description='Enable comprehensive sensor diagnostics'
    )

    # Robot description for hardware
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(
                get_package_share_directory("autonombot_description"),
                "urdf",
                "autonombot.urdf.xacro",
            ),
            " is_sim:=False",
            " use_jetson:=True"
        ]),
        value_type=str,
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False
        }],
        output="screen"
    )

    # Controller manager with hardware interface
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False
            },
            os.path.join(
                get_package_share_directory("autonombot_drivers"),
                "config",
                "autonombot_controllers.yaml",
            ),
        ],
        output="screen"
    )

    # HC-SR04 Ultrasonic Sensor Driver
    ultrasonic_driver_node = Node(
        package="autonombot_firmware",
        executable="hcsr04_driver.py",
        name="hcsr04_driver",
        parameters=[{
            'trig_pin': 18,  # GPIO 18 for trigger
            'echo_pin': 24,  # GPIO 24 for echo
            'frequency': LaunchConfiguration('ultrasonic_frequency'),
            'field_of_view': 0.5,  # ~30 degrees
            'min_range': 0.02,     # 2cm minimum
            'max_range': 4.0,      # 4m maximum
            'timeout_ms': 30.0,    # 30ms timeout
            'use_median_filter': True,
            'filter_window_size': 5,
            'use_sim_time': False
        }],
        output="screen"
    )

    # RPLiDAR A1 Driver
    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[
            os.path.join(
                get_package_share_directory("autonombot_launch"),
                "config",
                "rplidar_a1.yaml"
            ),
            {'use_sim_time': False}
        ],
        output="screen"
    )

    # RealSense 3D Camera Driver
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense_camera",
        parameters=[{
            'use_sim_time': False,
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30.0,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30.0,
            'pointcloud.enable': True,
            'align_depth.enable': True,
            'decimation_filter.enable': True,
            'spatial_filter.enable': True,
            'temporal_filter.enable': True
        }],
        output="screen"
    )

    # Jetson Sensor Manager (optional but recommended)
    sensor_manager_node = Node(
        package="autonombot_firmware",
        executable="jetson_sensor_manager.py",
        name="jetson_sensor_manager",
        parameters=[{
            'enable_ultrasonic': True,
            'enable_lidar': True,
            'enable_camera': True,
            'diagnostics_frequency': 1.0,
            'sensor_timeout': 5.0,
            'use_sim_time': False
        }],
        output="screen",
        condition=lambda context: LaunchConfiguration('enable_diagnostics').perform(context) == 'True'
    )

    # Static transforms for sensor positioning
    ultrasonic_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.15", "0.0", "0.1",  # x, y, z (15cm forward, 10cm up)
            "0.0", "0.0", "0.0", "1.0",  # qx, qy, qz, qw (no rotation)
            "base_link", "ultrasonic_link"
        ],
        parameters=[{'use_sim_time': False}]
    )

    camera_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.2", "0.0", "0.15",  # x, y, z (20cm forward, 15cm up)
            "0.0", "0.0", "0.0", "1.0",  # qx, qy, qz, qw (no rotation)
            "base_link", "camera_link"
        ],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        ultrasonic_freq_arg,
        enable_diagnostics_arg,
        
        # Core robot nodes
        robot_state_publisher_node,
        controller_manager,
        
        # Sensor drivers
        ultrasonic_driver_node,
        rplidar_node,
        realsense_node,
        
        # Monitoring and diagnostics
        sensor_manager_node,
        
        # Static transforms
        ultrasonic_transform,
        camera_transform,
    ])
