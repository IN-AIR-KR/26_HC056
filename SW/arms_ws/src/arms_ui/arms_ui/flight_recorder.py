#!/usr/bin/env python3
"""실기체 비행 로그 녹화기 (auto_save:=true 일 때 arms.launch.py 가 기동).

자동모드에서 SEARCH 진입 시 record_topics(UI 영상 + 검출 raw/필터/진단 + CRSF 송·수신
+ 상태/조종)를 rosbag 으로 녹화하기 시작하고, 조종기의 ARM(=자동모드 search)
스위치를 내릴 때까지 저장한다. 저장 포맷은 sqlite3(항상 사용 가능). mcap 은
젯슨에 플러그인이 없어 쓰지 않는다.
  · kill(buttons[0]) 은 무관 → kill 이 걸려도 ARM 스위치가 위면 계속 녹화.
  · 저장 위치: ~/arms_flight_log/<YYYYmmdd_HHMMSS>/ (비행 시작 일시). 이 폴더 안에
    bag/(rosbag) + record.log(녹화 프로세스 로그)를 함께 담는다.
  · detections_raw = 칼만 적용 전 원시 검출 → 추후 KF 파라미터 오프라인 재현/튜닝용.

/arms/command(Joy) 스위치: buttons[0]=kill, [1]=arm, [2]=mode(0=auto/1=manual), [3]=launch.
"""
import os
import signal
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy

from arms_msgs.msg import MissionState


class FlightRecorder(Node):
    def __init__(self):
        super().__init__("flight_recorder")
        # 녹화할 토픽 목록. 영상(UI) + 검출 raw/필터/진단 + CRSF 송·수신 + 상태/조종.
        #   detections_raw = 칼만 적용 전 원시 측정값(추후 KF 튜닝 재현용).
        #   crsf_tx=송신 채널, crsf_rx=수신 텔레메트리(배터리/링크/자세). 배터리 별도 토픽은 없다.
        #   대부분 구독자 있을 때만 발행되는 토픽이라, 이 녹화기가 구독하면 자동으로 켜진다.
        self.declare_parameter("record_topics", [
            "/arms/ui_image/compressed",
            "/arms/detections_raw",
            "/arms/detections",
            "/arms/detector_status",
            "/arms/crsf_tx",
            "/arms/crsf_rx",
            "/arms/mission_state",
            "/arms/command",
        ])
        self.declare_parameter("save_dir", "~/arms_flight_log")
        # 저장 포맷. 젯슨엔 mcap 플러그인이 없어 sqlite3 로 고정(항상 사용 가능).
        self.declare_parameter("storage", "sqlite3")
        self._topics = list(self.get_parameter("record_topics").value)
        self._save_dir = os.path.expanduser(self.get_parameter("save_dir").value)
        os.makedirs(self._save_dir, exist_ok=True)

        self._proc = None        # 실행 중인 'ros2 bag record' 프로세스
        self._flight_dir = None  # 이번 비행 폴더 (~/arms_flight_log/<일시>/)
        self._bag_dir = None     # 그 안의 rosbag (<일시>/bag)
        self._log_path = None    # 자식 stdout/stderr 로그 파일 경로
        self._log_fh = None
        self._health_timer = None
        self._arm = 0            # 최신 ARM 스위치 (command buttons[1])

        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(MissionState, "/arms/mission_state", self._cb_state, 10)
        self.create_subscription(Joy, "/arms/command", self._cb_cmd, best)
        self.get_logger().info(
            "flight_recorder ready — 자동모드 SEARCH 진입 시 녹화 시작, "
            f"ARM 스위치 내리면 종료. 저장: {self._save_dir}")

    def _recording(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _cb_state(self, msg: MissionState):
        # 자동모드에서 SEARCH 진입 → 녹화 시작.
        if not self._recording() and not msg.manual_mode and msg.state == "SEARCH":
            self._start()

    def _cb_cmd(self, msg: Joy):
        self._arm = msg.buttons[1] if len(msg.buttons) > 1 else 0
        # ARM(search) 스위치를 내리면 종료. kill(buttons[0]) 과는 무관하다.
        if self._recording() and not self._arm:
            self._stop("ARM 스위치 OFF")

    def _start(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # 이번 비행 결과를 한 폴더에 모은다: <일시>/bag(rosbag) + <일시>/record.log.
        self._flight_dir = os.path.join(self._save_dir, ts)
        os.makedirs(self._flight_dir, exist_ok=True)
        self._bag_dir = os.path.join(self._flight_dir, "bag")   # ros2 bag record 가 새로 만든다
        self._log_path = os.path.join(self._flight_dir, "record.log")
        storage = str(self.get_parameter("storage").value).strip().lower() or "sqlite3"
        # ros2 bag record 는 -o 경로가 없어야 새로 만든다. 그룹세션으로 띄워 SIGINT 를 그룹째 보낸다.
        cmd = ["ros2", "bag", "record", "--storage", storage, "-o", self._bag_dir, *self._topics]
        try:
            self._log_fh = open(self._log_path, "wb")
            self._proc = subprocess.Popen(
                cmd, start_new_session=True,
                stdout=self._log_fh, stderr=subprocess.STDOUT)
            self.get_logger().info(
                f"[REC] 녹화 시작 → {self._flight_dir}  (storage={storage}, {len(self._topics)}개 토픽)")
            # 플러그인 부재 등으로 곧바로 죽는지 1초 뒤 1회 점검(조용한 실패 방지).
            self._health_timer = self.create_timer(1.0, self._check_started)
        except Exception as e:
            self.get_logger().error(f"[REC] 녹화 시작 실패: {e}")
            self._close_log()
            self._proc = None

    def _check_started(self):
        if self._health_timer is not None:
            self._health_timer.cancel()
            self._health_timer = None
        if self._proc is not None and self._proc.poll() is not None:
            tail = ""
            try:
                with open(self._log_path, "r", errors="replace") as f:
                    tail = f.read()[-600:].strip()
            except Exception:
                pass
            self.get_logger().error(
                "[REC] 녹화가 즉시 종료됨 — bag record 실패. 저장 안 됨. "
                f"마지막 로그:\n{tail}")
            self._close_log()
            self._proc = None

    def _close_log(self):
        if self._log_fh is not None:
            try:
                self._log_fh.close()
            except Exception:
                pass
            self._log_fh = None

    def _stop(self, reason: str):
        if self._health_timer is not None:
            self._health_timer.cancel()
            self._health_timer = None
        if self._proc is None:
            self._close_log()
            return
        try:
            # ros2 bag record 는 자식 프로세스를 두므로 그룹째 SIGINT (bag 정상 마감/인덱싱).
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            self._proc.wait(timeout=10)
        except Exception as e:
            self.get_logger().warn(f"[REC] 종료 처리 예외({e}) → 강제 종료")
            try:
                self._proc.kill()
            except Exception:
                pass
        self.get_logger().info(f"[REC] 녹화 종료 ({reason}) → {self._flight_dir}")
        self._close_log()
        self._proc = None

    def destroy_node(self):
        if self._recording():
            self._stop("노드 종료")   # 스택 종료 시에도 bag 을 정상 마감
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlightRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
