"""
Full real-hardware launch: arms_video + arms_operator + arms_detection + arms_control + arms_ui
- arms_operator (ESP32 UART) → operator/input → arms_control
- arms_control → control/flight_command → arms_flight_io(CRSF serial) → ELRS TX → [RF] → ELRS RX → FC
- Detection(HSV 후보 → ROI YOLO 검증)은 GPU 도커 컨테이너로 동작. 이 런치가 'docker compose
  up -d' 로 자동 기동한다(멱등: 안 떠 있으면 띄우고, 떠 있으면 그대로). start_detection:=false
  로 끌 수 있음. 도커를 못 쓰거나 compose 를 못 찾으면 경고만 남기고 나머지는 계속 뜬다.
  로드할 가중치는 model:= 로 고른다 (기본 balloon_nano_v9.engine):
    ros2 launch arms_bringup arms.launch.py                                     # balloon nano v9(기본)
    ros2 launch arms_bringup arms.launch.py model:=/models/drone_small.engine   # 드론 추적
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _find_detection_compose():
    """실기체 detection 컨테이너 compose 경로 탐색.
    compose 의 build context 가 소스트리 기준(../..)이라 반드시 '소스' 위치에서 실행해야
    하므로, ARMS_SW 환경변수 → 설치 share 에서 워크스페이스 역산 순으로 소스 경로를 찾는다."""
    rel_src = "src/arms_detection/docker/docker-compose.jetson.yml"
    sw = os.environ.get("ARMS_SW")
    if sw:
        p = Path(sw) / "arms_ws" / rel_src
        if p.exists():
            return str(p)
    try:
        share = Path(get_package_share_directory("arms_bringup"))
        for base in [share, *share.parents]:
            c = base / rel_src
            if c.exists():
                return str(c)
    except Exception:
        pass
    return ""


def generate_launch_description():
    control_dir = Path(get_package_share_directory("arms_control")) / "config"
    control_config = control_dir / "control_params.yaml"
    # FC I/O 포트는 launch arg로 주입한다. CRSF 링크 속도는 400000 baud 고정이다.
    hw_config = control_dir / "hw_overrides.yaml"
    video_launch = (
        Path(get_package_share_directory("arms_video")) / "launch" / "video.launch.py"
    )
    operator_launch = (
        Path(get_package_share_directory("arms_operator")) / "launch" / "operator.launch.py"
    )
    detection_compose = _find_detection_compose()

    actions = [
        DeclareLaunchArgument("namespace", default_value="arms",
                              description="ROS namespace. Node code uses relative topic names."),
        PushRosNamespace(LaunchConfiguration("namespace")),
        DeclareLaunchArgument("crsf_port", default_value="/dev/ttyUSB0",
                              description="ELRS CRSF serial port (USB-C CP2102=/dev/ttyUSB0, 40핀 UART=/dev/ttyTHS1)"),
        DeclareLaunchArgument("start_detection", default_value="true",
                              description="detection(YOLO) 도커 컨테이너 자동 기동 여부"),
        DeclareLaunchArgument(
            "model", default_value="/models/balloon_nano_v9.engine",
            description="detection 컨테이너가 로드할 가중치(컨테이너 내부 경로). "
                        "기본=/models/balloon_nano_v9.engine(nano FP16 TRT), "
                        "드론추적=/models/drone_small.engine"),
        # UI 표시 옵션 (실기체 기본: 전체화면·메인화면만).
        DeclareLaunchArgument("fullscreen", default_value="true",
                              description="UI 전체화면 여부 (true/false)"),
        DeclareLaunchArgument("debug", default_value="false",
                              description="메인 화면 디버그 오버레이(err/cmd·YOLO/HSV 텍스트·화살표). "
                                          "기본 off — 실기체는 메인 화면만(표적 bbox 는 항상 표시)."),
        DeclareLaunchArgument("cv_debug", default_value="false",
                              description="검출 진단 패널(HSV 미리보기+성능 그래프). "
                                          "true 면 hsv_debug_image 를 구독 → 검출 발행률↓."),
        DeclareLaunchArgument("auto_save", default_value="false",
                              description="비행 로그 녹화. true 면 ARM 스위치 ON~"
                                          "OFF 까지 영상(UI)+검출 raw+CRSF 송·수신+상태를 "
                                          "~/arms_flight_logs_ibvs/<일시>/ 에 bag(sqlite3)으로 저장."),
    ]

    # 검출 컨테이너 자동 기동 (docker compose up -d — 멱등). 컨테이너의 arms_detection_node
    # 가 perception/detections 를 발행하므로 호스트에선 detection 노드를 따로 띄우지 않는다.
    #   model:= 런치 인자를 ARMS_MODEL 로 주입 → compose 의 ${ARMS_MODEL:-...} 가 이를 읽는다.
    if detection_compose:
        actions.append(ExecuteProcess(
            cmd=["docker", "compose", "-f", detection_compose, "up", "-d"],
            output="screen",
            additional_env={"ARMS_MODEL": LaunchConfiguration("model"),
                            "ARMS_NAMESPACE": LaunchConfiguration("namespace")},
            condition=IfCondition(LaunchConfiguration("start_detection")),
        ))
    else:
        actions.append(LogInfo(msg=(
            "[arms] detection compose 파일을 못 찾음 → 컨테이너 자동 기동 생략. "
            "ARMS_SW 를 SW 폴더로 export 하거나 수동으로 'docker compose up -d' 하세요.")))

    actions += [
        # Video capture (arms_video_node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(video_launch)),
        ),
        # 실기체 조종기 입력 (ESP32 UART → operator/input)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(operator_launch)),
        ),
        # 제어는 FlightCommand만 출력한다.
        Node(
            package="arms_control",
            executable="arms_control_node",
            name="arms_control_node",
            output="screen",
            parameters=[
                str(control_config),
                str(hw_config),
            ],
        ),
        # FC 통신의 유일한 owner: FlightCommand ↔ CRSF/ELRS telemetry.
        Node(
            package="arms_flight_io", executable="arms_crsf_io_node",
            name="arms_crsf_io_node", output="screen",
            parameters=[{
                "port": LaunchConfiguration("crsf_port"),
                "baud": 400000,
            }],
        ),
        # OpenCV UI. fullscreen/debug 는 런치 인자로 제어(실기체 기본: 전체화면·메인만).
        #   ui.debug=false → hsv_debug_image 미구독 → detection 발행 부하 최소.
        Node(
            package="arms_ui",
            executable="arms_ui_node",
            name="arms_ui_node",
            output="screen",
            parameters=[{
                "ui.fullscreen": ParameterValue(
                    LaunchConfiguration("fullscreen"), value_type=bool),
                "ui.debug": ParameterValue(
                    LaunchConfiguration("debug"), value_type=bool),
                "ui.cv_debug": ParameterValue(
                    LaunchConfiguration("cv_debug"), value_type=bool),
            }],
        ),
        # UI 화면 스트림: arms_ui_node 가 낸 raw(ui/image)를 image_transport republish 로
        #   compressed(ui/image/compressed)로 만든다 → rqt/원격에서 볼 수 있다.
        #   (파이썬 노드로는 compressed 정식 발행이 안 되므로 republish 가 담당. lazy: 구독 시만.)
        #   ★ out 베이스 리맵은 파생 서브토픽에 전파 안 되므로 out/compressed 를 직접 리맵한다.
        Node(
            package="image_transport",
            executable="republish",
            name="ui_image_republish",
            arguments=["raw", "compressed"],
            remappings=[
                ("in", "ui/image"),
                ("out/compressed", "ui/image/compressed"),
            ],
            parameters=[{"compressed.jpeg_quality": 60}],
        ),
        # 원본 카메라 영상을 녹화할 때만 compressed transport로 만든다. UI가 덧그려지기 전
        # 프레임이라 검출/제어 재현용 원본으로 남는다.
        Node(
            package="image_transport",
            executable="republish",
            name="raw_image_republish",
            arguments=["raw", "compressed"],
            remappings=[
                ("in", "camera/image_raw"),
                ("out/compressed", "camera/image_raw/compressed"),
            ],
            parameters=[{"compressed.jpeg_quality": 80}],
            condition=IfCondition(LaunchConfiguration("auto_save")),
        ),
        # 비행 로그 녹화기 (auto_save:=true 일 때만). ARM 스위치 ON~OFF 동안
        #   영상, 검출, 정규화 텔레메트리와 상태를 ~/arms_flight_logs_ibvs/<일시>/ 에 bag(sqlite3) 저장.
        Node(
            package="arms_ui",
            executable="flight_recorder",
            name="flight_recorder",
            output="screen",
            parameters=[{
                "manifest.control_params_path": str(control_config),
                "manifest.arg_crsf_port": ParameterValue(LaunchConfiguration("crsf_port"), value_type=str),
                "manifest.arg_start_detection": ParameterValue(LaunchConfiguration("start_detection"), value_type=str),
                "manifest.arg_model": ParameterValue(LaunchConfiguration("model"), value_type=str),
                "manifest.arg_fullscreen": ParameterValue(LaunchConfiguration("fullscreen"), value_type=str),
                "manifest.arg_debug": ParameterValue(LaunchConfiguration("debug"), value_type=str),
                "manifest.arg_cv_debug": ParameterValue(LaunchConfiguration("cv_debug"), value_type=str),
                "manifest.arg_auto_save": ParameterValue(LaunchConfiguration("auto_save"), value_type=str),
            }],
            condition=IfCondition(LaunchConfiguration("auto_save")),
        ),
    ]

    return LaunchDescription(actions)
