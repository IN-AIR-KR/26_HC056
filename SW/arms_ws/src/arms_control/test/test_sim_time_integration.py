"""arms_control_node 시뮬 시간(use_sim_time) 통합 시험.

하네스가 /clock 을 RTF 0.25(시뮬 1 s = 벽시계 4 s)로 발행하고, 실제 노드를
use_sim_time:=true + stamp_source=header 로 띄운다. PX4/Gazebo 불필요, 별도 DOMAIN_ID.

검증 항목
  1. 제어 주기가 시뮬 시간 50 Hz (벽시계 ≈ 12.5 Hz) — PX4 lockstep 과 같은 시간축
  2. 측정 신선도는 header.stamp(촬영 시각) 기준: 제때 받아도 촬영이 오래됐으면(0.5 s > gate 0.2 s)
     PN 명령을 내지 않고 수평·호버 유지, 촬영이 최근이면(0.05 s) PN 명령 활성
  3. TRACK 표적 소실 timeout 이 시뮬 2.0 s(벽시계 ≈ 8 s) 에 일어난다
"""
import os
import random
import signal
import subprocess
import threading
import time

import pytest

EXE = os.environ.get("ARMS_CONTROL_NODE_EXE", "")
PARAMS = os.environ.get("ARMS_CONTROL_PARAMS", "")

pytestmark = pytest.mark.skipif(
    not (EXE and os.path.exists(EXE) and PARAMS and os.path.exists(PARAMS)),
    reason="ARMS_CONTROL_NODE_EXE / ARMS_CONTROL_PARAMS 미설정 (colcon test 로 실행)")

os.environ["ROS_DOMAIN_ID"] = str(random.randint(150, 230))
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from arms_msgs.msg import (AttitudeTelemetry, BoundingBox, DetectionArray,  # noqa: E402
                           FlightCommand, IoStatus, MissionState, OperatorInput, PnDebug)

RTF = 0.25
TRACK_LOST = 2.0
HOVER = 0.32


class SimHarness(Node):
    def __init__(self):
        super().__init__("sim_time_harness")
        self.lock = threading.Lock()
        self.sim = 1000.0
        self._wall0 = time.monotonic()
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        ns = "/arms/"
        self.pub_op = self.create_publisher(OperatorInput, ns + "operator/input", 10)
        self.pub_io = self.create_publisher(IoStatus, ns + "io/status", 10)
        self.pub_att = self.create_publisher(AttitudeTelemetry, ns + "vehicle/attitude", 10)
        self.pub_det = self.create_publisher(DetectionArray, ns + "perception/detections", 10)
        self.create_subscription(FlightCommand, ns + "control/flight_command", self._on_cmd, 100)
        self.create_subscription(MissionState, ns + "mission/state", self._on_state, 100)
        self.create_subscription(PnDebug, ns + "control/pn_debug", self._on_pn, 100)
        self.arm = False
        self.launch = False
        self.target = False
        self.capture_lag = 0.05        # 검출 header = 시뮬 현재 - capture_lag
        self.size = 0.04
        self.cmd = None
        self.pn = None
        self.state = None
        self.cmd_wall = []
        self.transitions = []          # (wall, sim, prev, new)
        self.last_target_wall = None
        self.last_target_sim = None
        self.create_timer(0.005, self._tick_clock)
        self.create_timer(0.02, self._tick_fast)
        self.create_timer(0.05, self._tick_det)

    def sim_now(self):
        with self.lock:
            return self.sim

    def _stamp(self, t):
        from builtin_interfaces.msg import Time
        return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

    def _tick_clock(self):
        with self.lock:
            self.sim = 1000.0 + (time.monotonic() - self._wall0) * RTF
            t = self.sim
        self.pub_clock.publish(Clock(clock=self._stamp(t)))

    def _tick_fast(self):
        t = self.sim_now()
        op = OperatorInput()
        op.throttle = -1.0
        op.arm_switch, op.launch_request = self.arm, self.launch
        self.pub_op.publish(op)
        a = AttitudeTelemetry()
        a.header.stamp = self._stamp(t)
        a.yaw_deg = 90.0
        self.pub_att.publish(a)
        s = IoStatus()
        s.transport_connected = s.vehicle_telemetry_connected = True
        self.pub_io.publish(s)

    def _tick_det(self):
        t = self.sim_now()
        d = DetectionArray()
        d.header.stamp = self._stamp(t - self.capture_lag)
        if self.target:
            b = BoundingBox()
            b.x_center, b.y_center = 0.52, 0.48
            self.size = min(self.size * 1.01, 0.3)
            b.width = b.height = self.size
            b.confidence = 0.9
            d.detections.append(b)
            self.last_target_wall, self.last_target_sim = time.monotonic(), t
        self.pub_det.publish(d)

    def _on_cmd(self, m):
        with self.lock:
            self.cmd = m
            self.cmd_wall.append(time.monotonic())

    def _on_state(self, m):
        with self.lock:
            if self.state is not None and m.state != self.state:
                self.transitions.append((time.monotonic(), self.sim, self.state, m.state))
            self.state = m.state

    def _on_pn(self, m):
        self.pn = m


def wait_until(pred, timeout, step=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(step)
    return pred()


@pytest.fixture(scope="module")
def h():
    rclpy.init()
    node = SimHarness()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(0.5)    # /clock 이 먼저 흐르게
    proc = subprocess.Popen(
        [EXE, "--ros-args", "-r", "__ns:=/arms", "-r", "__node:=arms_control_node",
         "--params-file", PARAMS, "-p", "use_sim_time:=true",
         "-p", "control.pn.stamp_source:=header"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = []
    threading.Thread(target=lambda: out.extend(iter(proc.stdout.readline, "")),
                     daemon=True).start()
    assert wait_until(lambda: node.state is not None, 20.0), "mission/state 없음"
    node.log = out
    yield node
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    print("\n----- arms_control_node log (tail) -----")
    print("".join(out[-40:]))


def test_1_control_loop_runs_on_sim_time(h):
    assert any("시뮬 시간(/clock)" in line for line in h.log), "노드가 use_sim_time 이 아니다"
    with h.lock:
        h.cmd_wall.clear()
    time.sleep(4.0)
    with h.lock:
        n = len(h.cmd_wall)
    wall_hz = n / 4.0
    # 시뮬 50 Hz × RTF 0.25 = 벽시계 12.5 Hz. 벽시계 타이머였다면 50 Hz.
    assert 9.0 <= wall_hz <= 16.0, wall_hz


def test_2_freshness_uses_capture_stamp(h):
    h.target = True
    h.capture_lag = 0.05
    h.arm = False
    time.sleep(0.6)
    h.arm = True
    assert wait_until(lambda: h.state == "SEARCH", 4.0), h.state
    assert wait_until(lambda: h.state == "LOCK", 8.0), h.state      # lock 1.0 s(시뮬) ≈ 4 s
    h.launch = True
    assert wait_until(lambda: h.state == "TRACK", 2.0), h.state
    h.launch = False
    assert wait_until(lambda: h.pn is not None and h.pn.command_active, 3.0), \
        "최근 촬영 검출인데 PN 명령이 활성화되지 않음"
    # 같은 주기로 '제때' 도착하지만 촬영 시각이 0.5 s(시뮬) 지난 검출 → gate 0.2 s 초과.
    h.capture_lag = 0.5
    assert wait_until(lambda: not h.pn.command_active, 3.0), \
        "촬영 0.5 s 지난 검출로 PN 명령이 계속 나간다 (수신 시각 기준으로 판단 중?)"
    c = h.cmd
    assert c.arm and c.stabilized_mode and not c.manual_mode
    assert c.throttle == pytest.approx(HOVER, abs=1e-4)
    h.capture_lag = 0.05
    assert wait_until(lambda: h.pn.command_active, 3.0)


def test_3_target_lost_timeout_in_sim_seconds(h):
    assert h.state == "TRACK"
    h.target = False
    t_wall, t_sim = h.last_target_wall, h.last_target_sim
    assert wait_until(lambda: h.state == "IDLE", TRACK_LOST / RTF + 4.0), h.state
    w, s, a, b = [x for x in h.transitions if x[2] == "TRACK" and x[3] == "IDLE"][-1]
    sim_dt, wall_dt = s - t_sim, w - t_wall
    print(f"\n[측정] 표적 소실→IDLE: 시뮬 {sim_dt:.2f}s, 벽시계 {wall_dt:.2f}s (RTF {RTF})")
    assert TRACK_LOST - 0.1 <= sim_dt <= TRACK_LOST + 0.4, sim_dt
    assert wall_dt > TRACK_LOST / RTF * 0.8, wall_dt   # 벽시계 2 s 였다면 여기서 실패
    assert wait_until(lambda: h.cmd.manual_mode and h.cmd.arm, 1.0)
    h.arm = False
    assert wait_until(lambda: not h.cmd.arm, 2.0)
