"""SITL/replay용 가상 운용자 입력."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="arms_operator",
            executable="arms_operator_gui_node",
            namespace="arms",
            name="arms_operator_gui_node",
            output="screen",
        ),
    ])
