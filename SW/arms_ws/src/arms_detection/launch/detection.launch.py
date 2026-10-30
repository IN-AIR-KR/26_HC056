from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory("arms_detection") + "/config/detection.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "inference_backend", description="ultralytics 또는 hailo"),
        DeclareLaunchArgument(
            "model_path", description="선택한 backend가 읽을 절대 모델 경로"),
        Node(
            package="arms_detection",
            executable="arms_detection_node",
            name="arms_detection_node",
            output="screen",
            parameters=[config, {
                "inference.backend": LaunchConfiguration("inference_backend"),
                "inference.model_path": LaunchConfiguration("model_path"),
            }],
        ),
    ])
