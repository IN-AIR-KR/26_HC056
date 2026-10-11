from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = Path(get_package_share_directory("arms_control")) / "config"
    config = config_dir / "control_params.yaml"
    # 실기체 제어 델타 전용 오버레이 (서보/게인/자동발진 등).
    hw = config_dir / "hw_overrides.yaml"

    return LaunchDescription([
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[str(config), str(hw)],
        ),
    ])
