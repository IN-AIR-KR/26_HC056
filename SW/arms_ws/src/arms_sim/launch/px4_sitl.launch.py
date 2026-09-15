"""
px4_sitl.launch.py — PX4 SITL + Gazebo(arms_drone) 를 ros2 launch 안에서 띄운다.

시뮬레이터에 관한 건 전부 여기(arms_sim) 담당. arms_bringup/arms_sitl.launch.py 가
이 파일을 include 해서 ROS 스택과 함께 원클릭으로 올린다.

  - 기동 전 정리: 남아있는 px4 / gz sim 프로세스 kill, 월드·모델 심볼릭 링크 갱신
  - PX4 바이너리 직접 실행 (make 안 거침 → 빌드는 setup_sim.sh 에서만)
        build/px4_sitl_default/bin/px4 -d -s etc/init.d-posix/rcS <build>/etc   (cwd=rootfs)
      -d = 데몬 모드(pxh 셸 없음). 터미널 stdin 이 없어도 안 죽는다.
  - 부팅 후 PX4 파라미터 세팅 (페일세이프 해제, ACRO 각속도, 최대 기울기)
  - 종료(Ctrl+C) 시 PX4 가 백그라운드로 띄운 gz sim 까지 같이 정리

인자:
  px4_dir     PX4-Autopilot 경로   (기본: $PX4_DIR, 없으면 ~/PX4-Autopilot)
  world       Gazebo 월드 이름     (기본: arms_sitl)
  model       PX4 SITL 모델 이름   (기본: gz_arms_drone → airframe 4022)
  set_params  부팅 후 param set 실행 여부 (기본: true)

주의: airframe(assets/px4/airframes/4022_gz_arms_drone) 수정은 PX4 ROMFS 재빌드가 필요
      → setup_sim.sh 다시 실행.
"""
import os
import shutil
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnShutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 자동 disarm / 페일세이프 끄기 (GCS/RC 없는 SITL 자율비행 셋업).
#   COM_DISARM_PRFLT 0  = arm 후 이륙 지연 자동 disarm 해제 (가만히 둬도 안 멈춤)
#   NAV_DLL_ACT      0  = GCS(데이터링크) 연결 끊김 페일세이프 해제
#   NAV_RCL_ACT      0  = RC 연결 끊김 페일세이프 해제
#   MC_ACRO_R_MAX / P_MAX = ACRO 모드 풀스틱 각속도[deg/s] (control.max_rate_dps 와 일치).
#   MPC_MAN_TILT_MAX / MPC_TILTMAX_AIR = 최대 기울기[deg]. 크게 기울일수록 옆으로 빨리 감
#     → 빠른 표적 추격용. 35°(기본)→65° 로 상향. 단 너무 크면 양력 손실.
PX4_PARAMS = {
    "COM_DISARM_PRFLT": 0,
    "NAV_DLL_ACT": 0,
    "NAV_RCL_ACT": 0,
    "MC_ACRO_R_MAX": 400,
    "MC_ACRO_P_MAX": 400,
    "MPC_MAN_TILT_MAX": 65,
    "MPC_TILTMAX_AIR": 65,
}

# 종료/재기동 시 정리할 프로세스 패턴 (pkill -f).
#   gz sim 은 PX4 rc 스크립트가 '&' 로 띄우고 떠나서 PX4 가 죽어도 살아남는다.
KILL_PATTERNS = ["px4_sitl_default/bin/px4", "gz sim"]


def _kill_stale():
    for pat in KILL_PATTERNS:
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _link(src: Path, dst: Path):
    """ln -sfn src dst. 기존 파일/링크(구버전 run_arms.sh 의 cp 잔재 포함)는 교체."""
    if dst.is_symlink() or dst.exists():
        if dst.is_dir() and not dst.is_symlink():
            shutil.rmtree(dst)
        else:
            dst.unlink()
    dst.symlink_to(src)


def _setup(context, *args, **kwargs):
    px4_dir = Path(os.path.expanduser(LaunchConfiguration("px4_dir").perform(context)))
    world = LaunchConfiguration("world").perform(context)
    model = LaunchConfiguration("model").perform(context)
    set_params = LaunchConfiguration("set_params").perform(context).lower() in ("true", "1")

    share = Path(get_package_share_directory("arms_sim"))
    assets = share / "assets"
    models_dir = assets / "models"
    world_sdf = assets / "worlds" / f"{world}.sdf"

    build_dir = px4_dir / "build" / "px4_sitl_default"
    px4_bin = build_dir / "bin" / "px4"
    rootfs = build_dir / "rootfs"
    px4_worlds = px4_dir / "Tools" / "simulation" / "gz" / "worlds"
    px4_models = px4_dir / "Tools" / "simulation" / "gz" / "models"

    if not px4_bin.exists():
        raise RuntimeError(
            f"PX4 SITL 바이너리 없음: {px4_bin}\n"
            f"  PX4 경로가 다르면 px4_dir:=<경로> (또는 PX4_DIR env), "
            f"빌드 안 했으면 SW/setup_sim.sh 실행.")
    if not world_sdf.exists():
        raise RuntimeError(f"월드 파일 없음: {world_sdf}")

    # 남아있는 이전 인스턴스 정리 (Ctrl+C 로 안 죽은 gz sim 등)
    _kill_stale()

    # 월드/모델을 PX4 트리에 링크 → PX4 가 PX4_GZ_WORLD 로 찾고, 항상 레포 최신이 반영됨.
    px4_worlds.mkdir(parents=True, exist_ok=True)
    px4_models.mkdir(parents=True, exist_ok=True)
    _link(world_sdf, px4_worlds / world_sdf.name)
    for m in models_dir.iterdir():
        if m.is_dir():
            _link(m, px4_models / m.name)
    rootfs.mkdir(parents=True, exist_ok=True)

    env = dict(os.environ)
    env.update({
        "PX4_SIM_MODEL": model,
        "PX4_GZ_WORLD": world,
        # make 를 안 거치므로 make 가 넣어주던 값을 직접 지정 (rc 스크립트 기본값과 동일).
        "PX4_GZ_MODELS": str(px4_models),
        "PX4_GZ_WORLDS": str(px4_worlds),
        # 레포 모델 폴더도 gz 리소스 경로에 추가 (링크와 이중 안전장치).
        "GZ_SIM_RESOURCE_PATH": ":".join(
            p for p in [str(models_dir), os.environ.get("GZ_SIM_RESOURCE_PATH", "")] if p),
        # Wayland 세션에서 gz sim 3D 뷰 마우스 입력(휠 줌/드래그 회전)이 막히는 문제
        # → Qt 를 X11(xcb)로 강제.
        "QT_QPA_PLATFORM": "xcb",
    })

    actions = [
        LogInfo(msg=f"PX4 SITL: {px4_bin}  model={model}  world={world}"),
        ExecuteProcess(
            cmd=[str(px4_bin), "-d", "-s", "etc/init.d-posix/rcS", str(build_dir / "etc")],
            cwd=str(rootfs), env=env, name="px4", output="screen",
        ),
    ]

    if set_params:
        shell_py = px4_dir / "Tools" / "mavlink_shell.py"
        if shell_py.exists():
            script = "".join(f"param set {k} {v}\\n" for k, v in PX4_PARAMS.items())
            # mavlink_shell.py 는 heartbeat 올 때까지 기다리므로 PX4 부팅 완료를 기다릴 필요는
            # 없지만, 부팅 로그와 섞이지 않게 잠깐 지연. 실패해도 스택은 계속 뜬다.
            actions.append(TimerAction(period=15.0, actions=[
                ExecuteProcess(
                    cmd=["bash", "-c",
                         f"printf '{script}' | python3 '{shell_py}' udp:127.0.0.1:14550 "
                         f">/dev/null 2>&1 && echo 'PX4 param set 완료: "
                         f"{' '.join(f'{k}={v}' for k, v in PX4_PARAMS.items())}'"],
                    name="px4_params", output="screen",
                ),
            ]))
        else:
            actions.append(LogInfo(msg=f"!! {shell_py} 없음 → PX4 파라미터 세팅 생략"))

    # 첫 부팅은 위 PX4가 Gazebo와 기체를 생성한다. 이후 패널 RESET은 이 관리
    # 노드가 Gazebo를 유지한 채 world reset 후 PX4만 standalone으로 재기동한다.
    actions.append(Node(
        package="arms_sim", executable="sitl_reset_manager", name="arms_sitl_reset_manager",
        output="screen",
        parameters=[{
            "px4_dir": str(px4_dir), "world": world, "model": model,
            "drone_model": "arms_drone",
        }],
    ))

    return actions


def _on_shutdown(event, context):
    _kill_stale()


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "px4_dir",
            default_value=EnvironmentVariable("PX4_DIR", default_value="~/PX4-Autopilot"),
            description="PX4-Autopilot 경로"),
        DeclareLaunchArgument("world", default_value="arms_sitl", description="Gazebo 월드"),
        DeclareLaunchArgument("model", default_value="gz_arms_drone", description="PX4 SITL 모델"),
        DeclareLaunchArgument("set_params", default_value="true",
                              description="부팅 후 PX4 param set 실행"),
        OpaqueFunction(function=_setup),
        RegisterEventHandler(OnShutdown(on_shutdown=_on_shutdown)),
    ])
