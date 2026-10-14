"""HITL pipeline side — run this on the Jetson carried by the drone.

PX4/Gazebo live on the laptop running ``arms_hitl_host.launch.py``. Set the
same ROS_DOMAIN_ID and RMW implementation on both hosts. This launch connects
directly to the laptop PX4 MAVLink UDP endpoint.

Example:
  ROS_DOMAIN_ID=42 ros2 launch arms_bringup arms_hitl_drone.launch.py \
    px4_host:=192.168.1.50 detection:=docker
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _detection_dir():
    """Find source-only compose/models assets from ARMS_SW or installed share."""
    rel = "src/arms_detection/docker"
    sw = os.environ.get("ARMS_SW")
    if sw:
        candidate = Path(sw) / "arms_ws" / rel
        if candidate.is_dir():
            return candidate
    share = Path(get_package_share_directory("arms_bringup"))
    for base in [share, *share.parents]:
        candidate = base / rel
        if candidate.is_dir():
            return candidate
    return None


def _detection(context, *args, **kwargs):
    mode = LaunchConfiguration("detection").perform(context).strip().lower()
    model = LaunchConfiguration("det_model").perform(context).strip()
    docker_dir = _detection_dir()
    if mode == "docker":
        if docker_dir is None:
            return [LogInfo(msg=(
                "[arms_hitl_drone] detection compose directory not found; "
                "set ARMS_SW to the SW directory or start detection manually."))]
        return [ExecuteProcess(
            cmd=["docker", "compose", "-f", str(docker_dir / "docker-compose.jetson.yml"),
                 "up", "-d"],
            output="screen",
            additional_env={"ARMS_MODEL": model, "ARMS_INPUT_COMPRESSED": "1"},
        )]

    if mode != "host":
        return [LogInfo(msg=(
            f"[arms_hitl_drone] unknown detection:={mode}; use host or docker."))]
    if model.startswith("/models/") and docker_dir is not None:
        model = str(docker_dir / "models" / Path(model).name)
    else:
        model = os.path.expanduser(model)
    config = Path(get_package_share_directory("arms_detection")) / "config" / "detection.yaml"
    return [Node(
        package="arms_detection", executable="arms_detection_node",
        namespace="arms", name="arms_detection_node", output="screen",
        parameters=[str(config)], additional_env={"ARMS_MODEL": model},
    )]


def _sitl_bridge(context, *args, **kwargs):
    host = LaunchConfiguration("px4_host").perform(context).strip()
    port = LaunchConfiguration("px4_port").perform(context).strip()
    if not host:
        raise RuntimeError(
            "px4_host is required: provide the laptop LAN IP or resolvable hostname.")
    try:
        port_num = int(port)
        if not 1 <= port_num <= 65535:
            raise ValueError
    except ValueError as exc:
        raise RuntimeError(f"px4_port must be a UDP port, got {port!r}") from exc
    io_config = Path(get_package_share_directory("arms_flight_io")) / "config" / "io_params.yaml"
    return [Node(
        package="arms_flight_io", executable="sitl_bridge_node",
        namespace="arms", name="arms_sitl_bridge_node", output="screen",
        parameters=[str(io_config), {
            "connection": f"udpout:{host}:{port_num}",
            "send_rate_hz": 50.0,
        }],
    )]


def generate_launch_description():
    control_config = (
        Path(get_package_share_directory("arms_control")) / "config" / "control_params.yaml"
    )
    ui_config = Path(get_package_share_directory("arms_ui")) / "config" / "ui_params.yaml"
    recorder_config = (
        Path(get_package_share_directory("arms_logging")) / "config" / "flight_recorder_params.yaml"
    )
    operator_launch = (
        Path(get_package_share_directory("arms_operator")) / "launch" / "operator.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "px4_host", default_value="",
            description="required: laptop LAN IP/hostname where PX4 SITL is running"),
        DeclareLaunchArgument("px4_port", default_value="14540",
                              description="PX4 MAVLink UDP port on the laptop"),
        DeclareLaunchArgument("detection", default_value="docker",
                              description="detection placement: docker (Jetson default) or host"),
        DeclareLaunchArgument("det_model", default_value="/models/balloon_nano_v9.engine",
                              description="YOLO model path; /models/... is for Docker mode"),
        DeclareLaunchArgument("auto_save", default_value="true",
                              description="record each ARM-on to ARM-off flight on the Jetson"),
        DeclareLaunchArgument("fullscreen", default_value="true", description="fullscreen UI"),
        DeclareLaunchArgument("debug", default_value="false", description="UI debug overlay"),
        DeclareLaunchArgument("cv_debug", default_value="false", description="UI CV debug panel"),

        OpaqueFunction(function=_detection),
        Node(
            package="arms_control", executable="arms_control_node",
            namespace="arms", name="arms_control_node", output="screen",
            parameters=[str(control_config)],
        ),
        OpaqueFunction(function=_sitl_bridge),
        # 실기체와 같은 ESP32 UART 입력. HITL에서도 가상 조종기 GUI는 쓰지 않는다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(operator_launch)),
        ),
        Node(
            package="arms_ui", executable="arms_ui_node",
            namespace="arms", name="arms_ui_node", output="screen",
            parameters=[str(ui_config), {
                "ui.fullscreen": ParameterValue(LaunchConfiguration("fullscreen"), value_type=bool),
                "ui.debug": ParameterValue(LaunchConfiguration("debug"), value_type=bool),
                "ui.cv_debug": ParameterValue(LaunchConfiguration("cv_debug"), value_type=bool),
            }],
        ),
        Node(
            package="image_transport", executable="republish",
            namespace="arms", name="ui_image_republish",
            arguments=["raw", "compressed"],
            remappings=[
                ("in", "ui/image"),
                ("out/compressed", "ui/image/compressed"),
            ],
            parameters=[{"compressed.jpeg_quality": 60}],
        ),
        Node(
            package="arms_logging", executable="flight_recorder",
            namespace="arms", name="flight_recorder", output="screen",
            parameters=[str(recorder_config), {
                "tag": "hitl",
                "manifest.control_params_path": str(control_config),
                "manifest.arg_detection": ParameterValue(LaunchConfiguration("detection"), value_type=str),
                "manifest.arg_det_model": ParameterValue(LaunchConfiguration("det_model"), value_type=str),
                "manifest.arg_auto_save": ParameterValue(LaunchConfiguration("auto_save"), value_type=str),
                "manifest.arg_fullscreen": ParameterValue(LaunchConfiguration("fullscreen"), value_type=str),
                "manifest.arg_debug": ParameterValue(LaunchConfiguration("debug"), value_type=str),
                "manifest.arg_cv_debug": ParameterValue(LaunchConfiguration("cv_debug"), value_type=str),
            }],
            condition=IfCondition(LaunchConfiguration("auto_save")),
        ),
    ])
