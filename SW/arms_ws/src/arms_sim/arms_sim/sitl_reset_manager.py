#!/usr/bin/env python3
"""Panel-triggered SITL episode reset without restarting Gazebo."""

import os
import subprocess
import threading
import time
from pathlib import Path

import rclpy
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
        self.declare_parameter("drone_model", "arms_drone")
        self._busy = threading.Lock()
        self._reset_pub = self.create_publisher(Empty, "/arms/reset", 10)
        self.create_service(Trigger, "/arms/sitl/reset", self._reset)

    def _reset(self, _request, response):
        if not self._busy.acquire(blocking=False):
            response.success = False
            response.message = "이미 RESET 진행 중"
            return response
        try:
            # Referee/control 등이 이전 에피소드의 표적 진행 및 명령을 버리게 한다.
            self._reset_pub.publish(Empty())
            self._stop_px4()
            self._reset_world()
            self._start_px4()
            response.success = True
            response.message = "Gazebo world reset 및 PX4 재시작 완료"
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

    def _reset_world(self):
        world = self.get_parameter("world").value
        result = subprocess.run([
            "gz", "service", "-s", f"/world/{world}/control",
            "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", "reset: {all: true}",
        ], capture_output=True, text=True)
        if result.returncode:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"Gazebo world reset 실패: {detail or 'gz service 오류'}")
        # reset 요청이 적용된 다음 PX4 센서/actuator bridge를 붙인다.
        time.sleep(0.3)

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
            # 새 gz-server를 실행하지 않고, reset된 기존 기체에 다시 연결한다.
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_MODEL_NAME": self.get_parameter("drone_model").value,
            "PX4_GZ_MODELS": str(px4_dir / "Tools/simulation/gz/models"),
            "PX4_GZ_WORLDS": str(px4_dir / "Tools/simulation/gz/worlds"),
        })
        subprocess.Popen(
            [str(px4_bin), "-d", "-s", "etc/init.d-posix/rcS", str(build_dir / "etc")],
            cwd=str(build_dir / "rootfs"), env=env, start_new_session=True)
        self.get_logger().info("PX4 재시작: standalone으로 reset된 arms_drone에 연결")


def main(args=None):
    rclpy.init(args=args)
    node = SitlResetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
