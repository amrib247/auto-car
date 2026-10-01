import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("auto_car_control"), "config", "person_follow.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("stream_url", default_value="http://192.168.4.1:81/stream"),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            Node(
                package="auto_car_sensors",
                executable="wireless_yolo_sensor",
                name="wireless_yolo_sensor",
                output="screen",
                parameters=[
                    config_path,
                    {"stream_url": LaunchConfiguration("stream_url")},
                    {"model_path": LaunchConfiguration("model_path")},
                ],
            ),
            Node(
                package="auto_car_control",
                executable="wasd_controller",
                name="wasd_controller",
                output="screen",
                parameters=[
                    config_path,
                    {"serial_port": LaunchConfiguration("serial_port")},
                ],
            ),
        ]
    )