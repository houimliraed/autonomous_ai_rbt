import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_advanced_vision = LaunchConfiguration("use_advanced_vision")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )
    
    use_advanced_vision_arg = DeclareLaunchArgument(
        "use_advanced_vision",
        default_value="true",
        description="Enable advanced YOLO vision navigation system with intelligent obstacle detection"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("autonombot_launch"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    # RealSense camera driver
    camera_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py"
        ),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "depth_module.profile": "640,480,30",
            "rgb_camera.profile": "640,480,30"
        }.items()
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_drivers"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "True"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_drivers"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    ultrasonic_driver_node = Node(
        package="autonombot_firmware",
        executable="hcsr04_driver.py"
    )
    
    ultrasonic_localization_node = Node(
        package="autonombot_localization",
        executable="ultrasonic_localization.py"
    )
    
    ultrasonic_constraint_node = Node(
        package="autonombot_localization",
        executable="ultrasonic_position_constraint.py"
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )
    
    # Advanced Vision Navigation System (optional)
    advanced_vision = Node(
        package='autonombot_vision',
        executable='advanced_vision_navigator',
        name='advanced_vision_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        condition=IfCondition(use_advanced_vision)
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_advanced_vision_arg,
        hardware_interface,
        laser_driver,
        camera_driver,
        controller,
        joystick,
        ultrasonic_driver_node,
        ultrasonic_localization_node,
        ultrasonic_constraint_node,
        localization,
        slam,
        navigation,
        advanced_vision
    ])