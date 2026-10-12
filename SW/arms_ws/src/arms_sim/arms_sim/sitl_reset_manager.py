#!/usr/bin/env python3
"""Panel-triggered SITL episode reset without restarting Gazebo.

순서: /arms/reset 알림 → PX4 종료 → 기체를 출발 위치로 되돌림 → PX4 재기동(기존 기체에 attach).

★ 기체 엔티티를 지우지 않는 것이 이 파일의 핵심 제약이다.

예전에는 gz 의 `reset: {all: true}` 로 월드를 초기화했다. 그런데 그건 '초기 SDF 상태'로
되돌리는 것이라 런타임에 spawn 된 엔티티는 삭제된다 — 기체(arms_drone_0)는 PX4 rc
스크립트가 /world/<w>/create 로 만든 것이라 같이 사라졌다. 그러면 두 가지가 깨진다:

  1) PX4 를 attach 모드로 띄워도 붙을 기체가 없다 (기체 없는 월드만 남음).
  2) 더 고약한 쪽 — gz 가 엔티티는 지우면서 **카메라 센서는 파괴하지 않는다.** 몸체가
     사라진 유령 센서가 토픽을 계속 점유한 채 갱신되지 않는 그림을 뿌려서, 기체를 다시
     만들어도 arms_ui 화면이 reset 직전 장면에 멈춰 있었다.
     (실측: reset 직후 model list 에 기체가 없는데도 카메라 publisher 는 살아서 발행 중)

그래서 삭제 대신 set_pose 로 **되돌린다**. 엔티티가 한 번도 안 죽으니 센서도 멀쩡하고,
PX4 는 원래 의도대로 PX4_GZ_STANDALONE + PX4_GZ_MODEL_NAME 으로 기존 기체에 다시 붙는다.
표적(target_ball/target_uav)은 referee 가 /arms/reset 을 받아 직접 되돌리므로 여기서
월드를 건드릴 이유가 없다.
"""

import os
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger


class SitlResetManager(Node):
    """Owns the reset ordering: ROS episode state → PX4 → Gazebo → fresh PX4."""

    def __init__(self):
        super().__init__("arms_sitl_reset_manager")
        self.declare_parameter("px4_dir", "~/PX4-Autopilot")
        self.declare_parameter("world", "arms_sitl")
        self.declare_parameter("model", "gz_arms_drone")
        # PX4 가 spawn 하는 '인스턴스' 이름 (모델명 + _<px4_instance>). attach 대상.
        self.declare_parameter("drone_model", "arms_drone_0")
        # 발사기(launcher 노드)가 없을 때만 쓰는 예전 출발 위치(지면). 발사기가 있으면
        # 기체는 발사기 튜브로 재장전된다 — 최초 스폰도 튜브 끝(launcher_geometry).
        self.declare_parameter("spawn_pose", [0.0, 0.0, 0.24])
        self._busy = threading.Lock()
        self._reset_pub = self.create_publisher(Empty, "sim/reset_event", 10)
        # 견착 발사기(launcher 노드)의 재장전 서비스. 기체를 '지면'이 아니라 발사기
        # 튜브(현재 앙각/방위)로 되돌려 다시 물리고, 실제로 물렸는지 확인해서 답한다.
        #   RESET 서비스 콜백 안에서 다른 서비스를 기다려야 하므로, 응답을 받을 수
        #   있게 콜백 그룹을 나누고 main() 에서 MultiThreadedExecutor 로 돌린다.
        self._reload_cli = self.create_client(
            Trigger, "sim/launcher/reload", callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, "sim/reset", self._reset,
                            callback_group=MutuallyExclusiveCallbackGroup())

    def _reset(self, _request, response):
        if not self._busy.acquire(blocking=False):
            response.success = False
            response.message = "이미 RESET 진행 중"
            return response
        try:
            # ★ PX4 를 건드리기 전에 Gazebo 가 살아있는지 먼저 본다.
            #   gz 서버는 가끔 혼자 죽는다(SIGABRT 관측됨). 그 상태에서 리셋을 누르면
            #   예전 순서로는 PX4 를 먼저 죽인 뒤 set_pose 에서 실패해서, 결국 gz 도
            #   PX4 도 없는 더 나쁜 상태가 됐다. 살릴 수 없는 리셋이면 아무것도
            #   건드리지 말고 그대로 실패를 알린다.
            self._require_gazebo()
            self._stop_px4()
            self._park_drone()
            self._start_px4()
            drone = self._wait_for_px4()
            # PX4가 새 기체에 다시 붙은 뒤에만 episode 경계를 알린다.
            self._reset_pub.publish(Empty())
            response.success = True
            response.message = f"RESET 완료 ({drone} 원위치 + PX4 재attach)"
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
        finally:
            self._busy.release()
        return response

    def _stop_px4(self):
        pattern = "px4_sitl_default/bin/px4"
        subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if subprocess.run(["pgrep", "-f", pattern], stdout=subprocess.DEVNULL).returncode:
                return
            time.sleep(0.1)
        subprocess.run(["pkill", "-KILL", "-f", pattern], check=False)
        time.sleep(0.2)

    def _require_gazebo(self):
        """Gazebo 가 응답하는지 확인. 아니면 아무것도 건드리지 않고 실패."""
        drone = self.get_parameter("drone_model").value
        try:
            if self._drone_exists(drone):
                return
        except RuntimeError as exc:
            raise RuntimeError(
                f"Gazebo 응답 없음 ({exc}) — gz 서버가 죽은 것 같다. "
                "RESET 으로는 복구 못 하니 런치를 다시 띄워야 한다.") from exc
        raise RuntimeError(
            f"Gazebo 에 {drone} 가 없다 — 월드가 죽었거나 기체가 지워졌다. "
            "런치를 다시 띄워야 한다.")

    def _park_drone(self):
        """기체를 출발 위치·수평 자세로 되돌린다 (삭제하지 않는다).

        set_pose 는 속도를 0 으로 만들어주지 않는다. PX4 를 끈 직후 기체는 추력을 잃고
        낙하 중이라 그대로 두면 잔여 속도를 가진 채 재시작된다. 그래서 지면 높이로
        되돌린 뒤 잠깐 기다린다 — 접지 접촉이 속도를 깎아 없앤다.
        """
        # 발사기가 있으면 발사기에 재장전을 맡기고, 확인된 결과를 기다린다.
        #   (월드에 발사기가 있는 한 기체는 튜브에 용접돼 있을 수 있어서, 아래 지면용
        #    set_pose 는 용접 상태에선 조용히 무시된다 → 발사기 노드가 없으면 실패로 알린다)
        if self._reload_cli.wait_for_service(timeout_sec=1.0):
            fut = self._reload_cli.call_async(Trigger.Request())
            deadline = time.monotonic() + 8.0
            while not fut.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not fut.done():
                raise RuntimeError("발사기 재장전 응답 없음(8s) — launcher 노드 로그 확인")
            r = fut.result()
            if not r.success:
                raise RuntimeError(f"발사기 재장전 실패: {r.message}")
            self.get_logger().info(f"RESET: {r.message}")
            return
        self.get_logger().warn(
            "발사기 재장전 서비스 없음(launcher 노드 미기동?) → 예전처럼 지면으로 복귀 시도. "
            "월드에 발사기가 있으면 기체가 튜브에 물린 채라 실패할 수 있다.")

        world = self.get_parameter("world").value
        drone = self.get_parameter("drone_model").value
        x, y, z = (list(self.get_parameter("spawn_pose").value) + [0.0, 0.0, 0.0])[:3]
        # orientation 을 명시하지 않으면 뒤집힌 채로 리셋될 수 있다(추락해 구른 경우).
        req = (f'name: "{drone}", position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{x: 0, y: 0, z: 0, w: 1}}')
        result = subprocess.run([
            "gz", "service", "-s", f"/world/{world}/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", req,
        ], capture_output=True, text=True)
        if result.returncode or "true" not in result.stdout:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(
                f"기체 위치 복귀 실패({drone}): {detail or 'gz service 오류'} — "
                "Gazebo 가 떠 있는지, 기체 이름이 맞는지 확인")
        time.sleep(1.0)   # 접지로 잔여 속도가 사라질 시간

    def _start_px4(self):
        px4_dir = Path(os.path.expanduser(self.get_parameter("px4_dir").value))
        build_dir = px4_dir / "build" / "px4_sitl_default"
        px4_bin = build_dir / "bin" / "px4"
        if not px4_bin.exists():
            raise RuntimeError(f"PX4 바이너리 없음: {px4_bin}")

        env = dict(os.environ)
        env.update({
            "PX4_SIM_MODEL": self.get_parameter("model").value,
            "PX4_GZ_WORLD": self.get_parameter("world").value,
            "PX4_GZ_MODELS": str(px4_dir / "Tools/simulation/gz/models"),
            "PX4_GZ_WORLDS": str(px4_dir / "Tools/simulation/gz/worlds"),
            # 새 gz 를 띄우지 말고(STANDALONE), 살아있는 기체에 다시 붙어라(MODEL_NAME).
            # 이름은 PX4 가 만든 인스턴스명이어야 한다 — 'arms_drone' 이 아니라 'arms_drone_0'.
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_MODEL_NAME": self.get_parameter("drone_model").value,
        })
        subprocess.Popen(
            [str(px4_bin), "-d", "-s", "etc/init.d-posix/rcS", str(build_dir / "etc")],
            cwd=str(build_dir / "rootfs"), env=env, start_new_session=True)
        self.get_logger().info("PX4 재기동: 살아있는 기체에 attach (Gazebo·센서 유지)")

    def _wait_for_px4(self, timeout=20.0):
        """기체가 그대로 있고 PX4 가 붙는 데 성공했는지 확인한다.

        attach 에 실패하면 PX4 는 rcS 에서 exit 하고 조용히 사라진다. 여기서 보지
        않으면 패널은 '완료'라고 띄워놓고 실제로는 제어가 죽어 있는 상태가 된다.
        """
        drone = self.get_parameter("drone_model").value
        pattern = "px4_sitl_default/bin/px4"
        deadline = time.monotonic() + timeout
        alive_since = None
        while time.monotonic() < deadline:
            px4_up = subprocess.run(["pgrep", "-f", pattern],
                                    stdout=subprocess.DEVNULL).returncode == 0
            if not px4_up:
                if alive_since:   # 떴다가 죽었다 = attach 실패
                    raise RuntimeError(
                        f"PX4 가 기동 직후 종료됨 — {drone} attach 실패 가능성 "
                        "(rcS 로그 확인)")
                time.sleep(0.5)
                continue
            alive_since = alive_since or time.monotonic()
            if not self._drone_exists(drone):
                raise RuntimeError(f"{drone} 이 월드에서 사라짐 — 위치 복귀 로직 확인")
            # PX4 가 죽지 않고 몇 초 버티면 attach 성공으로 본다 (실패는 rcS 에서 즉시 exit).
            if time.monotonic() - alive_since > 5.0:
                return drone
            time.sleep(0.5)
        raise RuntimeError(f"PX4 재기동 확인 실패 ({timeout:.0f}s 초과)")

    @staticmethod
    def _drone_exists(drone):
        try:
            r = subprocess.run(["gz", "model", "--list"],
                               capture_output=True, text=True, timeout=5.0)
        except (subprocess.TimeoutExpired, OSError) as exc:
            raise RuntimeError(f"gz model --list 실패: {exc}") from exc
        return any(line.strip().lstrip("- ").strip() == drone
                   for line in r.stdout.splitlines())


def main(args=None):
    rclpy.init(args=args)
    node = SitlResetManager()
    executor = MultiThreadedExecutor(num_threads=2)   # RESET 콜백 중에도 재장전 응답 수신
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
