"""HITL simulator host side — run this on the laptop that hosts PX4 and Gazebo.

The Gazebo-only nodes stay here because their gz-transport APIs are local-only.
ROS 2 DDS carries camera frames, operator input, mission state, and simulation
events to/from the Jetson running ``arms_hitl_drone.launch.py``.

Example:
  ROS_DOMAIN_ID=42 ros2 launch arms_bringup arms_hitl_host.launch.py \
    px4_dir:=~/PX4-Autopilot
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_share = Path(get_package_share_directory("arms_sim"))
    video_share = Path(get_package_share_directory("arms_video"))
    bringup_share = Path(get_package_share_directory("arms_bringup"))
    sim_config = sim_share / "config" / "sim_params.yaml"
    px4_launch = sim_share / "launch" / "px4_sitl.launch.py"
    video_launch = video_share / "launch" / "video_sitl.launch.py"
    rviz_cfg = bringup_share / "config" / "sitl.rviz"

    return LaunchDescription([
        DeclareLaunchArgument(
            "px4_dir",
            default_value=EnvironmentVariable("PX4_DIR", default_value="~/PX4-Autopilot"),
            description="PX4-Autopilot path on this laptop"),
        DeclareLaunchArgument("world", default_value="arms_sitl", description="Gazebo world"),
        DeclareLaunchArgument("model", default_value="gz_arms_drone", description="PX4 SITL model"),
        DeclareLaunchArgument("set_params", default_value="true",
                              description="apply ARMS PX4 SITL parameters after boot"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="run local SITL ground-truth RViz visualization"),

        # PX4 + Gazebo and the local-only sim/reset manager.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(px4_launch)),
            launch_arguments={
                "px4_dir": LaunchConfiguration("px4_dir"),
                "world": LaunchConfiguration("world"),
                "model": LaunchConfiguration("model"),
                "set_params": LaunchConfiguration("set_params"),
            }.items(),
        ),
        # Gazebo image to ROS. This publishes camera/image_raw over DDS to Jetson.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(video_launch))),
        # Docker detection on Jetson consumes this topic; it is lazy and therefore
        # costs no JPEG work until a remote compressed subscriber exists.
        Node(
            package="image_transport", executable="republish",
            namespace="arms", name="raw_image_republish",
            arguments=["raw", "compressed"],
            remappings=[
                ("in", "camera/image_raw"),
                ("out/compressed", "camera/image_raw/compressed"),
            ],
            parameters=[{"compressed.jpeg_quality": 80}],
        ),
        # Gazebo transport users must run on the same host as Gazebo.
        Node(
            package="arms_sim", executable="panel",
            namespace="arms", name="arms_panel_node", output="screen",
        ),
        Node(
            package="arms_sim", executable="referee",
            namespace="arms", name="balloon_referee", output="screen",
            parameters=[str(sim_config)],
        ),
        Node(
            package="arms_sim", executable="launcher",
            namespace="arms", name="arms_launcher", output="screen",
        ),
        Node(
            package="arms_sim", executable="sitl_viz",
            namespace="arms", name="arms_sitl_viz", output="screen",
            parameters=[str(sim_config)],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", str(rviz_cfg)] if rviz_cfg.exists() else [],
            output="log",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
