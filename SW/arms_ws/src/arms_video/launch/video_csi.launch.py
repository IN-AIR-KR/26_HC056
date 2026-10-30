from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("arms_video")) / "config" / "csi_params.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("csi_socket", default_value="/run/arms-csi/camera.sock"),
        Node(
            package="arms_video", executable="arms_csi_video_node",
            namespace="arms", name="arms_video_csi_node", output="screen",
            parameters=[str(config), {"socket_path": LaunchConfiguration("csi_socket")}],
        ),
    ])
