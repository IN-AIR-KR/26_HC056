from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory("arms_detection") + "/config/detection.yaml"
    return LaunchDescription([
        Node(
            package="arms_detection",
            executable="arms_detection_node",
            name="arms_detection_node",
            output="screen",
            parameters=[config],
        ),
    ])
