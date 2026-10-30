"""arms_control_node 통합 시험: 자동 비행 명령 중단 → 수동 제어 전환, 안전 전이.

실제 arms_control_node 바이너리를 띄우고 가짜 입력(operator/input, io/status,
vehicle/attitude, perception/detections)을 넣어 FlightCommand·MissionState 를 본다.
PX4/Gazebo 는 필요 없다. 다른 ROS 그래프(SITL 등)와 섞이지 않게 별도 DOMAIN_ID 로 돈다.

검증 항목
  1. SEARCH → LOCK → TRACK (launch 요청) 정상 진입, TRACK 에서 자동 명령(arm, !manual)
  2. TRACK 중 표적 소실: track_target_lost_timeout 경과 시 IDLE + 수동 인계
       - 무장 유지(공중 disarm 금지), 조종 스틱 그대로 전달, RTL 아님
       - 인계 후 ARM 스위치가 올라가 있어도 자동 미션이 다시 시작되지 않음
  3. ARM 스위치 OFF → 즉시 CH5 disarm (비상 motor cut)
  4. flight I/O 단절(io/status) → RTL + rtl_request
  5. use_ground_truth=true 인데 GT 토픽이 없으면 PN 명령을 만들지 않음(추력 0)
  6. 표적 소실 hold 구간(PN 입력이 오래됨)에서 추력 0(자유낙하)이 아니라 수평 자세·호버 추력 유지

실행: colcon test --packages-select arms_control --pytest-args -s
"""
import json
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

DOMAIN = str(random.randint(150, 230))
os.environ["ROS_DOMAIN_ID"] = DOMAIN
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy  # noqa: E402  (DOMAIN_ID 를 먼저 설정해야 한다)
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from arms_msgs.msg import (AttitudeTelemetry, BoundingBox, DetectionArray,  # noqa: E402
                           FlightCommand, IoStatus, MissionState, OperatorInput, PnDebug)

TRACK_LOST = 2.0     # control_params.yaml mission.track_target_lost_timeout_sec
HOVER = 0.32         # control_params.yaml control.hover_throttle


class Harness(Node):
    def __init__(self):
        super().__init__("handover_test_harness")
        ns = "/arms/"
        self.pub_op = self.create_publisher(OperatorInput, ns + "operator/input", 10)
        self.pub_io = self.create_publisher(IoStatus, ns + "io/status", 10)
        self.pub_att = self.create_publisher(AttitudeTelemetry, ns + "vehicle/attitude", 10)
        self.pub_det = self.create_publisher(DetectionArray, ns + "perception/detections", 10)
        self.create_subscription(FlightCommand, ns + "control/flight_command", self._on_cmd, 50)
        self.create_subscription(MissionState, ns + "mission/state", self._on_state, 50)
        self.create_subscription(PnDebug, ns + "control/pn_debug", self._on_pn, 50)
        self.set_cli = self.create_client(SetParameters, "/arms/arms_control_node/set_parameters")
        self.lock = threading.Lock()
        # 입력 상태 (테스트가 바꾼다)
        self.arm = False
        self.manual = False
        self.launch = False
        self.sticks = (0.2, -0.1, 0.0, 0.05)   # roll, pitch, throttle, yaw
        self.io_ok = True
        self.publish_att = True
        self.target = False                    # True=표적 bbox, False=빈 DetectionArray
        self.size = 0.04
        # 관측
        self.cmd = None
        self.state = None
        self.pn = None
        self.cmd_log = []                      # (t, state, FlightCommand)
        self.transitions = []                  # (t, prev, new)
        self.last_target_t = None
        self.create_timer(0.02, self._tick_fast)
        self.create_timer(0.05, self._tick_det)
        self.create_timer(0.1, self._tick_io)

    def _tick_fast(self):
        op = OperatorInput()
        op.roll, op.pitch, op.throttle, op.yaw = self.sticks
        op.arm_switch, op.manual_mode, op.launch_request = self.arm, self.manual, self.launch
        self.pub_op.publish(op)
        if self.publish_att:
            a = AttitudeTelemetry()
            a.yaw_deg = 90.0
            self.pub_att.publish(a)

    def _tick_det(self):
        d = DetectionArray()
        if self.target:
            b = BoundingBox()
            b.x_center, b.y_center = 0.52, 0.48
            self.size = min(self.size * 1.01, 0.3)
            b.width = b.height = self.size
            b.confidence = 0.9
            d.detections.append(b)
            self.last_target_t = time.monotonic()
        self.pub_det.publish(d)

    def _tick_io(self):
        s = IoStatus()
        s.source = "test"
        s.transport_connected = self.io_ok
        s.vehicle_telemetry_connected = self.io_ok
        self.pub_io.publish(s)

    def _on_cmd(self, m):
        with self.lock:
            self.cmd = m
            self.cmd_log.append((time.monotonic(), self.state, m))

    def _on_state(self, m):
        with self.lock:
            if self.state is not None and m.state != self.state:
                self.transitions.append((time.monotonic(), self.state, m.state))
            self.state = m.state

    def _on_pn(self, m):
        self.pn = m

    def set_param(self, name, value):
        assert self.set_cli.wait_for_service(timeout_sec=5.0)
        pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        fut = self.set_cli.call_async(SetParameters.Request(
            parameters=[ParamMsg(name=name, value=pv)]))
        deadline = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert fut.done() and all(r.successful for r in fut.result().results)


def wait_until(pred, timeout, step=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(step)
    return pred()


@pytest.fixture(scope="module")
def h():
    env = dict(os.environ)
    proc = subprocess.Popen(
        [EXE, "--ros-args", "-r", "__ns:=/arms", "-r", "__node:=arms_control_node",
         "--params-file", PARAMS],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out_lines = []
    reader = threading.Thread(target=lambda: out_lines.extend(iter(proc.stdout.readline, "")),
                              daemon=True)
    reader.start()
    rclpy.init()
    node = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    assert wait_until(lambda: node.state is not None, 15.0), "control node 가 mission/state 를 안 낸다"
    node.node_log = out_lines
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
    print("".join(out_lines[-60:]))


def _to_track(h):
    h.target = True
    h.arm = False
    time.sleep(0.4)          # ARM 재토글 래치 해제
    h.arm = True
    assert wait_until(lambda: h.state == "SEARCH", 3.0), h.state
    assert wait_until(lambda: h.state == "LOCK", 4.0), h.state
    h.launch = True
    assert wait_until(lambda: h.state == "TRACK", 2.0), h.state
    h.launch = False


def test_1_track_entry_and_auto_command(h):
    _to_track(h)
    time.sleep(0.6)
    c = h.cmd
    assert c.arm and not c.manual_mode and not c.rtl_request
    assert c.stabilized_mode            # guidance_mode=1 (PN) → Stabilized
    assert c.throttle > 0.0             # 신선한 검출+자세 → PN 추력


def test_2_target_loss_hands_over_to_manual(h):
    assert h.state == "TRACK"
    h.target = False                    # 검출기는 살아 있고 표적만 사라짐
    t_lost = h.last_target_t
    assert wait_until(lambda: h.state == "IDLE", TRACK_LOST + 1.5), h.state
    t_idle = [t for t, a, b in h.transitions if a == "TRACK" and b == "IDLE"][-1]
    dt = t_idle - t_lost
    assert TRACK_LOST - 0.1 <= dt <= TRACK_LOST + 0.5, dt
    assert wait_until(lambda: h.cmd.manual_mode, 0.5)
    c = h.cmd
    roll, pitch, thr, yaw = h.sticks
    assert c.arm, "표적 소실 인계 중 공중 disarm 이 일어났다"
    assert not c.rtl_request and not c.stabilized_mode
    assert c.roll == pytest.approx(roll, abs=1e-4)
    assert c.pitch == pytest.approx(pitch, abs=1e-4)
    assert c.yaw == pytest.approx(yaw, abs=1e-4)
    assert c.throttle == pytest.approx((thr + 1.0) * 0.5, abs=1e-4)
    # ARM 스위치가 그대로여도 자동 미션이 다시 시작되면 안 된다.
    h.target = True
    time.sleep(1.5)
    assert h.state == "IDLE" and h.cmd.manual_mode and h.cmd.arm

    # 소실~인계 사이 자동 명령: PN 입력이 오래되면 추력 0 이 아니라 수평·호버 유지여야 한다.
    with h.lock:
        hold = [(t, m) for t, s, m in h.cmd_log
                if t_lost + 0.4 <= t <= t_idle - 0.05 and not m.manual_mode]
    assert len(hold) > 20, len(hold)
    zero = [m for _, m in hold if m.throttle <= 1e-3]
    assert not zero, f"hold 구간에 추력 0 명령 {len(zero)}틱 (자유낙하)"
    assert all(m.stabilized_mode and m.arm for _, m in hold)
    assert all(m.throttle == pytest.approx(HOVER, abs=1e-4) for _, m in hold)
    assert all(abs(m.roll) < 1e-4 and abs(m.pitch) < 1e-4 for _, m in hold)
    report = {"track_lost_to_idle_s": round(dt, 3), "hold_ticks": len(hold),
              "hold_throttle": round(hold[-1][1].throttle, 3)}
    print("\n[측정] 표적 소실 hold 구간:", json.dumps(report, ensure_ascii=False))
    h.target = False


def test_3_arm_switch_off_cuts_motors(h):
    h.arm = False
    assert wait_until(lambda: not h.cmd.arm, 0.3)
    assert h.state == "IDLE"


def test_4_io_fault_requests_rtl(h):
    h.target = False
    h.arm = True
    assert wait_until(lambda: h.state == "SEARCH", 3.0), h.state
    assert wait_until(lambda: h.cmd.arm, 1.0)
    h.io_ok = False
    assert wait_until(lambda: h.state == "RTL", 1.0), h.state
    assert wait_until(lambda: h.cmd.rtl_request and h.cmd.manual_mode, 0.5)
    h.io_ok = True
    h.arm = False
    assert wait_until(lambda: h.state == "IDLE" and not h.cmd.arm, 1.0)


def test_5_ground_truth_without_topic_blocks_pn(h):
    h.set_param("control.pn.use_ground_truth", True)
    _to_track(h)
    time.sleep(0.8)
    c = h.cmd
    assert c.arm and not c.manual_mode
    # GT 가 없으면 PN 명령은 만들지 않고(command_active=false) 수평·호버 유지만 한다.
    assert h.pn is not None and not h.pn.command_active, "GT 없이 PN 명령이 생성됐다"
    assert c.throttle == pytest.approx(HOVER, abs=1e-4)
    assert abs(c.roll) < 1e-4 and abs(c.pitch) < 1e-4
    assert any("[GT]" in line and "use_ground_truth=true" in line for line in h.node_log)
    h.arm = False
    h.set_param("control.pn.use_ground_truth", False)
    assert wait_until(lambda: h.state == "IDLE" and not h.cmd.arm, 1.0)
