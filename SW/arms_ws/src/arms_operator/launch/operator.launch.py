"""실기체: ESP32 UART 운용자 입력 노드."""
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = (
        Path(get_package_share_directory("arms_operator")) / "config" / "operator_params.yaml"
    )

    return LaunchDescription([
        Node(
            package="arms_operator",
            executable="arms_operator_hardware_node",
            name="arms_operator_hardware_node",
            output="screen",
            parameters=[str(config)],
        ),
    ])
