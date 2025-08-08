from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )

    use_python = LaunchConfiguration("use_python")

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.1", "--y", "0","--z", "0.05",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "ultrasonic_link"],
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("autonombot_localization"), "config", "ekf_ultrasonic.yaml")],
    )

    ultrasonic_localization_node = Node(
        package="autonombot_localization",
        executable="ultrasonic_localization.py",
        condition=IfCondition(use_python),
    )

    ultrasonic_constraint_node = Node(
        package="autonombot_localization",
        executable="ultrasonic_position_constraint.py",
        condition=IfCondition(use_python),
    )

    return LaunchDescription([
        use_python_arg,
        static_transform_publisher,
        robot_localization,
        ultrasonic_localization_node,
        ultrasonic_constraint_node,   
    ])