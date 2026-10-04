import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("auto_car_control"), "config", "person_follow.yaml"
    )
    enable_control = LaunchConfiguration("enable_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "stream_url", default_value="http://192.168.0.203:81/stream"
            ),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("inference_device", default_value="cuda:0"),
            DeclareLaunchArgument("imu_udp_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("imu_udp_port", default_value="12345"),
            DeclareLaunchArgument("imu_display", default_value="true"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("enable_control", default_value="true"),
            Node(
                package="auto_car_sensors",
                executable="wireless_yolo_sensor",
                name="wireless_yolo_sensor",
                output="screen",
                parameters=[
                    config_path,
                    {"stream_url": LaunchConfiguration("stream_url")},
                    {"model_path": LaunchConfiguration("model_path")},
                    {"inference_device": LaunchConfiguration("inference_device")},
                ],
            ),
            Node(
                package="auto_car_sensors",
                executable="imu_udp_sensor",
                name="imu_udp_sensor",
                output="screen",
                parameters=[
                    config_path,
                    {"udp_host": LaunchConfiguration("imu_udp_host")},
                    {"udp_port": ParameterValue(
                        LaunchConfiguration("imu_udp_port"), value_type=int
                    )},
                    {"display": ParameterValue(
                        LaunchConfiguration("imu_display"), value_type=bool
                    )},
                ],
            ),
            Node(
                package="auto_car_control",
                executable="person_follow_controller",
                name="person_follow_controller",
                output="screen",
                parameters=[
                    config_path,
                    {"serial_port": LaunchConfiguration("serial_port")},
                    {"enabled": ParameterValue(enable_control, value_type=bool)},
                ],
            ),
        ]
    )