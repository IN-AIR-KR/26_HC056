"""
arms_sitl_angled_diag.launch.py — 각도 발사 비충돌 진단 시험

    ros2 launch arms_bringup arms_sitl_angled_diag.launch.py elev:=50
    ros2 launch arms_bringup arms_sitl_angled_diag.launch.py elev:=90            # 기준(수직)
    ros2 launch arms_bringup arms_sitl_angled_diag.launch.py elev:=50 use_gt:=true
    ros2 launch arms_bringup arms_sitl_angled_diag.launch.py elev:=50 target_offset_deg:=18  # 화각 가장자리

arms_sitl.launch.py 를 그대로 띄우되 다음만 바꾼다:
  - 발사기 초기 앙각/방위 = elev/az
  - release:=false  → TRACK 에 들어가도 드론을 분리하지 않는다. 기체는 발사관에 물린 채라
                      표적과 물리적으로 만날 수 없다(비충돌). 상태머신·명령·GT 경로는 그대로 돈다.
  - 표적(풍선)은 정지 상태로 카메라 광축 위 target_range m 지점(+ 화면 가로 target_offset_deg)에 둔다.
  - angled_launch_monitor 가 카메라 FPS/누락/지연, 검출 주기, 상태 전이, GT, 기대 vs 실제 자세,
    예측 vs 실제 화면 위치를 report_period 마다 로그 + JSONL 로 남긴다.
  - auto_save 기본 false (녹화가 필요하면 auto_save:=true).

표적 소실 시험(비충돌): 패널에서 ARM → LOCK 확인 → LAUNCH(드론은 안 떨어짐) → TRACK 에서
패널 표적 X/Y 슬라이더로 표적을 화면 밖으로 옮기면 track_target_lost_timeout 뒤 IDLE + 수동 인계가
일어나는지 모니터 로그([state], [cmd])로 확인한다.
"""
import os
import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _monitor(context, *args, **kwargs):
    elev = float(LaunchConfiguration("elev").perform(context))
    gt = LaunchConfiguration("use_gt").perform(context).strip().lower() in ("true", "1")
    log_dir = Path(os.path.expanduser(LaunchConfiguration("log_dir").perform(context)))
    jsonl = log_dir / f"{time.strftime('%Y%m%d_%H%M%S')}_elev{elev:.0f}{'_gt' if gt else ''}.jsonl"
    return [Node(
        package="arms_sim", executable="angled_launch_monitor",
        namespace="arms", name="arms_angled_launch_monitor", output="screen",
        parameters=[{
            "jsonl_path": str(jsonl),
            "report_period_sec": float(LaunchConfiguration("report_period").perform(context)),
            "setup.elevation_deg": elev,
            "setup.azimuth_deg": float(LaunchConfiguration("az").perform(context)),
            "setup.target_range_m": float(LaunchConfiguration("target_range").perform(context)),
            "setup.target_offset_deg": float(
                LaunchConfiguration("target_offset_deg").perform(context)),
            "setup.use_ground_truth": 1 if gt else 0,
        }],
    )]


def generate_launch_description():
    sitl = Path(get_package_share_directory("arms_bringup")) / "launch" / "arms_sitl.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("elev", default_value="50.0", description="발사 앙각[deg] (90=기준)"),
        DeclareLaunchArgument("az", default_value="0.0", description="발사 방위[deg]"),
        DeclareLaunchArgument("target_range", default_value="25.0",
                              description="카메라~정지 표적 거리[m] (광축 위)"),
        DeclareLaunchArgument("target_offset_deg", default_value="0.0",
                              description="표적을 화면 가로로 비끼는 각[deg] (HFOV/2=21.8)"),
        DeclareLaunchArgument("use_gt", default_value="false",
                              description="control.pn.use_ground_truth"),
        DeclareLaunchArgument("release", default_value="false",
                              description="true 로 주면 실제 분리 발사(비충돌 보장 없음)"),
        DeclareLaunchArgument("report_period", default_value="5.0"),
        DeclareLaunchArgument("log_dir", default_value="~/arms_diag_logs"),
        DeclareLaunchArgument("px4", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("detection", default_value="host"),
        DeclareLaunchArgument("auto_save", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sitl)),
            launch_arguments={
                "px4": LaunchConfiguration("px4"),
                "rviz": LaunchConfiguration("rviz"),
                "detection": LaunchConfiguration("detection"),
                "auto_save": LaunchConfiguration("auto_save"),
                "launch_elev": LaunchConfiguration("elev"),
                "launch_az": LaunchConfiguration("az"),
                "use_gt": LaunchConfiguration("use_gt"),
                "release": LaunchConfiguration("release"),
                "diag": "false",   # 모니터는 아래에서 setup.* 과 함께 직접 띄운다
            }.items(),
        ),
        OpaqueFunction(function=_monitor),
    ])
