"""
arms_sitl.launch.py — SITL "날아다니는 풍선 요격" 올인원 런치

    ros2 launch arms_bringup arms_sitl.launch.py              # PX4+Gazebo 포함 전부
    ros2 launch arms_bringup arms_sitl.launch.py px4:=false   # PX4 가 이미 떠 있을 때 스택만
    ros2 launch arms_bringup arms_sitl.launch.py detection:=docker   # 검출을 컨테이너로
    ros2 launch arms_bringup arms_sitl.launch.py auto_save:=true     # 에피소드별 bag 녹화

이거 하나로 다음을 전부 띄운다:
  - PX4 SITL + Gazebo       (arms_sim/px4_sitl.launch.py include, px4:=false 로 생략)  [arms_sim]
  - gz 카메라 브리지        (/arms/image_raw)
  - arms_detection_node     (HSV 후보 → ROI YOLO 검증, detection:= 로 host/docker)
  - arms_control_node       (상태머신 + PID → FlightCommand)
  - arms_sitl_bridge_node   (FlightCommand → MAVLink RC override → PX4)
  - arms_operator_gui_node  (가상 조종기 → operator/input)
  - panel                   (튜닝/심판 콘솔)                          [arms_sim]
  - referee                 (표적 풍선/드론 비행 + 명중 판정)          [arms_sim]
  - arms_ui_node            (OpenCV 영상 오버레이)
  - sitl_viz + rviz2        (3D 궤적/기체/표적, 기본 기동; rviz:=false 로 생략) [arms_sim]
  - plotjuggler             (5초 뒤)
  - flight_recorder         (auto_save:=true 일 때만. ~/arms_flight_logs_ibvs/<일시>_sitl/) [arms_ui]

스택 노드들은 PX4 부팅을 기다리지 않는다 — sitl_bridge 는 heartbeat 올 때까지 재접속,
referee 는 gz 준비 전 실패를 무시, 카메라 브리지는 gz 토픽이 생기면 붙는다.
Ctrl+C 한 번이면 PX4/Gazebo 까지 전부 내려간다 (px4_sitl.launch.py 의 OnShutdown).
"""
import os
import shutil
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

CRSF_TX = "/tmp/crsf_tx"   # arms_control 이 쓰는 쪽
CRSF_RX = "/tmp/crsf_rx"   # sitl_bridge 가 읽는 쪽


def _detection_dir():
    """소스트리의 arms_detection/docker 디렉토리 (compose·가중치가 같이 있다).

    compose 의 build context 가 소스 기준(../..)이라 설치본이 아니라 '소스' 위치를
    찾아야 한다. ARMS_SW 환경변수 → arms_bringup share 에서 워크스페이스 역산 순.
    (arms.launch.py / arms_replay.launch.py 의 _find_detection_compose 와 같은 탐색)
    """
    rel = "src/arms_detection/docker"
    sw = os.environ.get("ARMS_SW")
    if sw:
        p = Path(sw) / "arms_ws" / rel
        if p.is_dir():
            return p
    try:
        share = Path(get_package_share_directory("arms_bringup"))
        for base in [share, *share.parents]:
            c = base / rel
            if c.is_dir():
                return c
    except Exception:
        pass
    return None


def _detection(context, *args, **kwargs):
    """검출 기동 — detection:= host(기본) | docker

    host 는 호스트에서 in-process YOLO, docker 는 실기체와 같은 검출 컨테이너.
    gz 카메라 브리지는 raw 만 내보내므로 컨테이너엔 ARMS_INPUT_COMPRESSED=0 을 같이
    넣는다 (compose 기본은 압축=1).

    ★ 둘을 동시에 띄우면 안 된다. 노드 이름(/arms_detection_node)이 같고 둘 다
      /arms/detections 를 발행해서 서로를 덮어쓴다.
    """
    mode = LaunchConfiguration("detection").perform(context).strip().lower()
    model = LaunchConfiguration("det_model").perform(context).strip()
    docker_dir = _detection_dir()

    if mode == "docker":
        if docker_dir is None:
            return [LogInfo(msg="[arms_sitl] detection compose 를 못 찾음 → 컨테이너 기동 "
                                "생략. ARMS_SW 를 SW 폴더로 export 하거나 수동으로 띄우세요.")]
        # 플랫폼 선택은 arms.launch.py 와 동일 규칙 (ARMS_DETECTION_PLATFORM 로 강제 가능)
        platform = os.environ.get("ARMS_DETECTION_PLATFORM", "").strip().lower()
        if platform not in {"jetson", "laptop"}:
            platform = "jetson" if Path("/etc/nv_tegra_release").exists() else "laptop"
        return [ExecuteProcess(
            cmd=["docker", "compose", "-f",
                 str(docker_dir / f"docker-compose.{platform}.yml"), "up", "-d"],
            output="screen",
            additional_env={"ARMS_MODEL": model, "ARMS_INPUT_COMPRESSED": "0"},
        )]

    actions = []
    if mode != "host":
        actions.append(LogInfo(msg=f"[arms_sitl] detection:={mode} 는 모르는 값 "
                                   "(host|docker) → host 로 진행"))
    # det_model 은 다른 런치와 같은 컨테이너 경로(/models/x) 표기다. host 는 같은 파일을
    # 호스트에서 직접 열어야 하므로 docker/models/ 로 되돌린다.
    if model.startswith("/models/") and docker_dir is not None:
        model = str(docker_dir / "models" / Path(model).name)
    else:
        model = os.path.expanduser(model)
    det_config = Path(get_package_share_directory("arms_detection")) / "config" / "detection.yaml"
    actions.append(Node(
        package="arms_detection", executable="arms_detection_node",
        name="arms_detection_node", output="screen",
        parameters=[str(det_config)] if det_config.exists() else [],
        # YOLO on/off 는 파라미터가 아니라 이 환경변수다 — 모델이 로드돼야 검출을
        # 발행한다(HSV 단독 발행 금지). 노드 기동 시점에 한 번만 읽는다.
        additional_env={"ARMS_MODEL": model},
    ))
    return actions


def _socat(context, *args, **kwargs):
    """CRSF 가상 시리얼 쌍 (socat PTY pair).
    arms_control → /tmp/crsf_tx → [socat] → /tmp/crsf_rx → arms_sitl_bridge"""
    if shutil.which("socat") is None:
        raise RuntimeError("socat 없음: sudo apt install socat")
    # 이전 인스턴스 정리 (socat 은 link= 대상이 이미 있으면 못 만든다)
    subprocess.run(["pkill", "-9", "-f", "socat.*crsf"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for p in (CRSF_TX, CRSF_RX):
        try:
            os.unlink(p)
        except FileNotFoundError:
            pass
    return [ExecuteProcess(
        cmd=["socat", f"PTY,raw,echo=0,link={CRSF_TX}", f"PTY,raw,echo=0,link={CRSF_RX}"],
        name="socat_crsf", output="screen",
    )]


def generate_launch_description():
    control_config = (
        Path(get_package_share_directory("arms_control")) / "config" / "control_params.yaml"
    )
    bringup_cfg = Path(get_package_share_directory("arms_bringup")) / "config"
    pj_layout = bringup_cfg / "sitl_debug.xml"
    rviz_cfg = bringup_cfg / "sitl.rviz"
    pj_cmd = ["ros2", "run", "plotjuggler", "plotjuggler", "--buffer_size", "60"]
    if pj_layout.exists():
        pj_cmd += ["--layout", str(pj_layout)]
    px4_launch = Path(get_package_share_directory("arms_sim")) / "launch" / "px4_sitl.launch.py"

    actions = [
        DeclareLaunchArgument("px4", default_value="true",
                              description="PX4 SITL + Gazebo 도 같이 띄울지 (이미 떠 있으면 false)"),
        DeclareLaunchArgument("px4_dir", default_value=os.environ.get("PX4_DIR", "~/PX4-Autopilot"),
                              description="PX4-Autopilot 경로 (px4:=true 일 때)"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="rviz 3D 시각화(기체·표적·궤적) 기동 여부. "
                                          "기본 true. sitl_viz(진실값 발행) + rviz2 가 같이 뜬다."),
        DeclareLaunchArgument("detection", default_value="host",
                              description="검출 실행 위치: host(기본, 호스트 in-process YOLO) | "
                                          "docker(검출 컨테이너). 동시에 띄우면 노드 이름이 "
                                          "겹치므로 둘 중 하나만."),
        # ★ 이름이 'model' 이면 안 된다. px4_sitl.launch.py 가 PX4 airframe 이름으로
        #   model:= 를 이미 쓰고, 부모의 launch configuration 은 include 된 런치로
        #   그대로 새어 들어간다 — 여기서 model 을 선언하면 PX4 가 가중치 경로를
        #   PX4_SIM_MODEL 로 받아 8ms 만에 exit 255 로 죽는다(Gazebo 도 안 뜬다).
        DeclareLaunchArgument(
            "det_model", default_value="/models/balloon_nano_v9.pt",
            description="YOLO 가중치. 다른 런치의 model:= 과 같은 컨테이너 경로(/models/x) "
                        "표기이며 host 모드에선 docker/models/x 로 자동 치환된다. "
                        "호스트 절대경로를 직접 줘도 된다."),

        DeclareLaunchArgument("auto_save", default_value="false",
                              description="비행 로그 녹화. true 면 가상 조종기 ARM 스위치 ON~OFF 를 "
                                          "~/arms_flight_logs_ibvs/<일시>_sitl/ "
                                          "에 bag(sqlite3) 저장(UI 화면 영상 포함)."),

        # PX4 SITL + Gazebo — 시뮬레이터 쪽은 전부 arms_sim 담당
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(px4_launch)),
            condition=IfCondition(LaunchConfiguration("px4")),
            launch_arguments={"px4_dir": LaunchConfiguration("px4_dir")}.items(),
        ),
        # 카메라 브리지 — 상방 카메라 → /arms/image_raw
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="arms_video_up", output="screen",
            arguments=["/arms_drone/upward_camera/image@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[("/arms_drone/upward_camera/image", "/arms/image_raw")],
        ),
        # 검출 (host in-process / docker 컨테이너) — detection:= 로 고른다
        OpaqueFunction(function=_detection),
        # 제어는 전송 방식과 독립적인 FlightCommand만 발행한다.
        Node(
            package="arms_control", executable="arms_control_node",
            namespace="arms", name="arms_control_node", output="screen",
            parameters=[str(control_config)],
        ),
        # SITL I/O 어댑터 (FlightCommand → MAVLink RC override).
        Node(
            package="arms_flight_io", executable="sitl_bridge_node",
            namespace="arms", name="arms_sitl_bridge_node", output="screen",
            parameters=[{"connection":   "udpin:0.0.0.0:14540",
                         "send_rate_hz": 50.0}],
        ),
        # SITL 가상 운용자 입력.
        Node(
            package="arms_operator", executable="arms_operator_gui_node",
            namespace="arms", name="arms_operator_gui_node", output="screen",
        ),
        # 튜닝/심판 콘솔 (파라미터·referee 제어). SITL 전용, /arms/command 발행 안 함.
        Node(
            package="arms_sim", executable="panel",
            name="arms_panel_node", output="screen",
        ),
        # 표적 심판 (풍선/드론 비행 + 명중 판정). SITL 전용.
        #   node name = balloon_referee (panel 의 REFEREE_NODE 와 일치必).
        Node(
            package="arms_sim", executable="referee",
            name="balloon_referee", output="screen",
        ),
        # UI 오버레이
        Node(
            package="arms_ui", executable="arms_ui_node",
            name="arms_ui_node", output="screen",
        ),
        # UI 화면 스트림: arms_ui_node 가 낸 raw(/arms/ui_image) → compressed(/arms/ui_image/compressed).
        #   arms.launch.py 와 같은 구성. 녹화기(auto_save)가 이 compressed 를 녹화한다.
        #   lazy — 구독자(녹화기·rqt) 있을 때만 변환·발행하므로 평소 부하 없음.
        #   ★ out 베이스 리맵은 파생 서브토픽에 전파 안 되므로 out/compressed 를 직접 리맵한다.
        Node(
            package="image_transport", executable="republish",
            name="ui_image_republish",
            arguments=["raw", "compressed"],
            remappings=[
                ("in", "/arms/ui_image"),
                ("out/compressed", "/arms/ui_image/compressed"),
            ],
            parameters=[{"compressed.jpeg_quality": 60}],
        ),
        # 원본 카메라 영상도 녹화 때만 JPEG compressed로 만든다. UI 오버레이 전 프레임이라
        # 추후 검출/유도 재현에 쓴다.
        Node(
            package="image_transport", executable="republish",
            name="raw_image_republish",
            arguments=["raw", "compressed"],
            remappings=[
                ("in", "/arms/image_raw"),
                ("out/compressed", "/arms/image_raw/compressed"),
            ],
            parameters=[{"compressed.jpeg_quality": 80}],
            condition=IfCondition(LaunchConfiguration("auto_save")),
        ),
        # 비행 로그 녹화기 (auto_save:=true 일 때만). 실기체와 같은 노드·같은 조건
        #   (가상 조종기 ARM 스위치 ON → 시작, OFF → 저장). UI 화면 영상도 같이 저장.
        Node(
            package="arms_ui", executable="flight_recorder",
            name="flight_recorder", output="screen",
            parameters=[{
                "tag": "sitl",
                "record_topics": [
                    "/arms/ui_image/compressed",
                    "/arms/image_raw/compressed",
                    "/arms/detections_raw",
                    "/arms/detections",
                    "/arms/detector_status",
                    "/arms/ibvs_debug",
                    "/arms/control_debug",
                    "/arms/crsf_tx",
                    "/arms/crsf_rx",
                    "/arms/attitude",
                    "/arms/battery",
                    "/arms/gps",
                    "/arms/mission_state",
                    "/arms/operator/input",
                    "/arms/control/flight_command",
                    "/arms/io/status",
                ],
                "manifest.control_params_path": str(control_config),
                "manifest.arg_px4": ParameterValue(LaunchConfiguration("px4"), value_type=str),
                "manifest.arg_px4_dir": ParameterValue(LaunchConfiguration("px4_dir"), value_type=str),
                "manifest.arg_rviz": ParameterValue(LaunchConfiguration("rviz"), value_type=str),
                "manifest.arg_detection": ParameterValue(LaunchConfiguration("detection"), value_type=str),
                "manifest.arg_det_model": ParameterValue(LaunchConfiguration("det_model"), value_type=str),
                "manifest.arg_auto_save": ParameterValue(LaunchConfiguration("auto_save"), value_type=str),
            }],
            condition=IfCondition(LaunchConfiguration("auto_save")),
        ),
        # 3D 시각화 (기본 활성). gz 진실값 → TF/궤적/마커.
        #   rviz:=false면 sitl_viz도 함께 생략한다.
        Node(
            package="arms_sim", executable="sitl_viz",
            name="arms_sitl_viz", output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", str(rviz_cfg)] if rviz_cfg.exists() else [],
            output="log",   # rviz 로그는 터미널을 뒤덮는다
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]

    # PlotJuggler — 노드 startup 후 5초 지연해서 실행
    actions.append(TimerAction(period=5.0, actions=[
        ExecuteProcess(cmd=pj_cmd, output="screen"),
    ]))

    return LaunchDescription(actions)
