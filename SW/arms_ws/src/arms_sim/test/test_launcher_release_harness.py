"""발사기 자동 분리(LOCK→TRACK) 비구동 시험.

Gazebo·PX4 없이 실제 launcher 노드만 띄우고, 하네스가 다음을 흉내 낸다.
  - /clock (RTF 0.5), mission/state, vehicle/armed, vehicle/attitude (ROS)
  - DetachableJoint 플러그인 상태 "attached" (gz transport, 격리된 GZ_PARTITION)
분리 '명령'은 gz 토픽 /arms_sim/launcher/detach 로 나가는데, 받을 Gazebo 가 없으므로
아무것도 움직이지 않는다. 하네스는 그 토픽을 구독해 분리 명령이 나갔는지만 본다.

검증
  1. ARM 직후(무장 < release_min_armed_sec) TRACK 진입 → spool-up 대기 후 분리 명령이 나간다
     (회귀: 예전엔 TRACK 진입 순간 한 번만 판정해 영영 분리되지 않았다)
  2. FC 실제 무장이 아니면 TRACK 이어도 분리 명령 없음 (인터록 유지)
  3. PX4 자세가 발사관 기하와 다르면 분리 명령 없음 (인터록 유지)
  4. 대기 중 TRACK 을 벗어나면 이후 분리 명령 없음
"""
import os
import random
import signal
import subprocess
import sys
import threading
import time

import pytest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(150, 230))
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["GZ_PARTITION"] = f"arms_launcher_harness_{os.getpid()}"
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

gz = pytest.importorskip("gz.transport13")
from gz.msgs10.empty_pb2 import Empty as GzEmpty  # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg as GzString  # noqa: E402

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time as RosTime  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from arms_msgs.msg import AttitudeTelemetry, MissionState  # noqa: E402

from arms_sim import launcher_geometry as G  # noqa: E402

RTF = 0.5
ELEV = 50.0
# 하네스가 쓸 launcher 실행 명령. 다른 버전과 비교할 때 LAUNCHER_CMD 로 바꾼다.
LAUNCHER_CMD = os.environ.get("LAUNCHER_CMD", "ros2 run arms_sim launcher").split()


class Harness(Node):
    def __init__(self):
        super().__init__("launcher_release_harness")
        self.lock = threading.Lock()
        self.sim = 100.0
        self._w0 = time.monotonic()
        self.state = "IDLE"
        self.mission_armed = False
        self.fc_armed = False
        self.att = G.expected_px4_euler_deg(ELEV, 0.0)
        self.detach_cmds = []          # (sim, wall)
        self.plugin = "attached"
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_state = self.create_publisher(MissionState, "/arms/mission/state", 10)
        self.pub_armed = self.create_publisher(Bool, "/arms/vehicle/armed", 10)
        self.pub_att = self.create_publisher(AttitudeTelemetry, "/arms/vehicle/attitude", 10)
        self.status = ""
        self.create_subscription(String, "/arms/sim/launcher/status",
                                 lambda m: setattr(self, "status", m.data), 10)
        self.create_timer(0.005, self._tick_clock)
        self.create_timer(0.02, self._tick)
        self.gz = gz.Node()
        self.gz_state = self.gz.advertise("/arms_sim/launcher/state", GzString)
        self.gz.subscribe(GzEmpty, "/arms_sim/launcher/detach", self._on_detach)
        self.create_timer(0.5, self._tick_plugin)

    def now(self):
        with self.lock:
            return self.sim

    def _tick_clock(self):
        with self.lock:
            self.sim = 100.0 + (time.monotonic() - self._w0) * RTF
            t = self.sim
        self.pub_clock.publish(Clock(clock=RosTime(sec=int(t), nanosec=int((t % 1) * 1e9))))

    def _tick(self):
        m = MissionState()
        m.state, m.armed = self.state, self.mission_armed
        self.pub_state.publish(m)
        self.pub_armed.publish(Bool(data=self.fc_armed))
        a = AttitudeTelemetry()
        a.roll_deg, a.pitch_deg, a.yaw_deg = self.att
        self.pub_att.publish(a)

    def _tick_plugin(self):
        msg = GzString()
        msg.data = self.plugin
        self.gz_state.publish(msg)

    def _on_detach(self, _msg):
        with self.lock:
            self.detach_cmds.append((self.sim, time.monotonic()))
        self.plugin = "detached"


def wait_until(pred, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


@pytest.fixture
def h():
    rclpy.init()
    node = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(0.3)
    proc = subprocess.Popen(
        LAUNCHER_CMD + ["--ros-args", "-r", "__ns:=/arms", "-r", "__node:=arms_launcher",
                        "-p", "use_sim_time:=true", "-p", f"elevation_deg:={ELEV}"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = []
    threading.Thread(target=lambda: out.extend(iter(proc.stdout.readline, "")),
                     daemon=True).start()
    node.log = out
    # launcher 가 플러그인 'attached' 를 받아 장착 상태가 될 때까지
    assert wait_until(lambda: any("장착 확인" in l for l in out), 20.0), "".join(out[-20:])
    yield node
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.stdout.write("\n----- launcher log -----\n" + "".join(out[-25:]))


def _arm_then_track(h, armed_before_track_sim):
    """IDLE → (FC·미션 무장) SEARCH → LOCK → TRACK. 무장 후 armed_before_track_sim 초 뒤 TRACK."""
    h.state, h.mission_armed = "SEARCH", True
    t_arm = h.now()
    wait_until(lambda: h.now() - t_arm >= armed_before_track_sim * 0.7, 30.0)
    h.state = "LOCK"
    wait_until(lambda: h.now() - t_arm >= armed_before_track_sim, 30.0)
    h.state = "TRACK"
    return t_arm


def test_1_release_after_spoolup_when_track_entered_early(h):
    h.fc_armed = True
    t_arm = _arm_then_track(h, 0.9)      # 사용자 SITL 기록: 무장 0.37~1.20 s 에 TRACK
    ok = wait_until(lambda: h.detach_cmds, 10.0)
    assert ok, "TRACK 진입 후 분리 명령이 끝내 나가지 않음 (회귀)"
    t_rel = h.detach_cmds[0][0] - t_arm
    print(f"\n[측정] 무장→분리 명령: 시뮬 {t_rel:.2f}s")
    assert t_rel >= 1.5 - 0.05, f"spool-up 인터록보다 일찍 분리: {t_rel:.2f}s"
    assert t_rel <= 2.5, t_rel


def test_2_no_release_without_fc_armed(h):
    h.fc_armed = False                    # control 은 ARM 요청, PX4 는 실제 미무장
    _arm_then_track(h, 2.0)
    assert not wait_until(lambda: h.detach_cmds, 4.0), "FC 미무장인데 분리 명령이 나감"
    assert any("FC 실제 무장 아님" in l for l in h.log), "인터록 판정이 기록되지 않음(TRACK 미수신?)"
    # 패널에 보이는 발사기 상태가 '왜 안 나가는지'를 보여줘야 한다
    assert wait_until(lambda: h.status.startswith("분리 대기") and "FC 무장 아님" in h.status, 3.0), \
        h.status


def test_3_no_release_with_attitude_mismatch(h):
    h.fc_armed = True
    r, p, y = G.expected_px4_euler_deg(ELEV, 0.0)
    h.att = (r, p + 30.0, y)              # EKF 미정렬 상태
    _arm_then_track(h, 2.0)
    assert not wait_until(lambda: h.detach_cmds, 4.0), "자세 불일치인데 분리 명령이 나감"
    assert any("발사관 기하와" in l for l in h.log), "인터록 판정이 기록되지 않음(TRACK 미수신?)"


def test_4_pending_release_cancelled_when_leaving_track(h):
    h.fc_armed = True
    _arm_then_track(h, 0.3)
    # launcher 가 TRACK 을 실제로 받아 '분리 대기'에 들어간 것을 확인한 뒤 이탈한다
    #   (바로 IDLE 로 바꾸면 TRACK 이 한 번도 발행되지 않아 시험이 공허하게 통과한다).
    assert wait_until(lambda: any("자동 분리 대기 시작" in l for l in h.log), 5.0), \
        "launcher 가 TRACK 진입/분리 대기를 기록하지 않음"
    h.state = "IDLE"                      # spool-up 대기 중 TRACK 이탈
    h.mission_armed = False
    h.fc_armed = False
    assert not wait_until(lambda: h.detach_cmds, 4.0), "TRACK 이탈 후 분리 명령이 나감"
    assert any("자동 분리 대기 취소" in l for l in h.log), "대기 취소가 기록되지 않음"
