"""ak45_node (team-jax/ak45_ros2, its own params file) + sim2real_node.

    ros2 launch sim2real sim2real.launch.py [servo_channel:=can0] [max_speed_deg_s:=30.0]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ak45_launch = os.path.join(get_package_share_directory("ak45_ros2"), "launch", "ak45_node.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("servo_channel", default_value="can0"),
        DeclareLaunchArgument("max_speed_deg_s", default_value="30.0",
                              description="speed limit of every motor command, deg/s"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="empty: the repository's calibration.json"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ak45_launch)),
        Node(
            package="sim2real",
            executable="sim2real_node",
            name="sim2real_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "servo_channel": LaunchConfiguration("servo_channel"),
                "max_speed_deg_s": LaunchConfiguration("max_speed_deg_s"),
                "calibration_file": LaunchConfiguration("calibration_file"),
            }],
        ),
    ])
