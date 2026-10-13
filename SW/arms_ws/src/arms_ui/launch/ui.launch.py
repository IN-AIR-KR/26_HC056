from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory("arms_ui") + "/config/ui_params.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "fullscreen", default_value="false",
            description="true=전체화면(실기체), false=창 모드(기본)"),
        Node(
            package="arms_ui",
            executable="arms_ui_node",
            name="arms_ui_node",
            output="screen",
            parameters=[config, {
                "ui.fullscreen": ParameterValue(
                    LaunchConfiguration("fullscreen"), value_type=bool),
            }],
        ),
    ])
