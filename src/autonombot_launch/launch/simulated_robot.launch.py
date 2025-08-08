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
        default_value="false",
        description="Enable advanced YOLO vision navigation system with intelligent obstacle detection"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_description"),
            "launch",
            "gazebo.launch.py"
        ),
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
            "use_sim_time": "True"
        }.items()
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
            'use_sim_time': True
        }],
        condition=IfCondition(use_advanced_vision)
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("autonombot_launch"),
                "rviz",
                "autonombot_simulation.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_advanced_vision_arg,
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        navigation,
        advanced_vision,
        rviz,
    ])