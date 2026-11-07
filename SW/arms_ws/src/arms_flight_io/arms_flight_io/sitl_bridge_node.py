#!/usr/bin/env python3
"""
sitl_bridge_node — FlightCommand→MAVLink adapter for SITL.

``control/flight_command``를 PX4의 RC override, arm/disarm, 비행 모드와 RTL MAVLink
명령으로 변환한다. PX4 GPS_RAW_INT와 ATTITUDE는 환경 독립 ``vehicle/*`` 토픽으로
정규화해 UI와 rosbag이 실기체와 같은 계약을 사용하게 한다.
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from arms_msgs.msg import AttitudeTelemetry, FlightCommand, GpsTelemetry, IoStatus

try:
    from pymavlink import mavutil
    _PYMAVLINK = True
except ImportError:
    _PYMAVLINK = False

# PX4 mode constants
PX4_BASE_MODE_CUSTOM          = 1
# manual/손제어: 자동수평 + throttle 직결(고도유지 없음). 실기체 FC 의 수동 모드와 동작 일치.
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7  # (CH6 high)
PX4_CUSTOM_MAIN_MODE_ACRO     = 5   # 각속도(rate) 제어 — 자동 요격용 (자동수평 없음)

MAV_MODE_FLAG_SAFETY_ARMED = 128
ARM_RETRY_INTERVAL_SEC = 1.0  # resend arm/disarm while desired != actual armed state
# MAV_CMD_COMPONENT_ARM_DISARM param2 magic value.  PX4 accepts a regular
# disarm only while landed; this explicitly requests an in-flight motor cut.
FORCE_DISARM_MAGIC = 21196.0
# 이 시간 이상 MAVLink 수신이 없으면 PX4 가 내려간 것으로 본다(패널 RESET 이 PX4 만 재기동).
# heartbeat(시뮬 시간 1Hz)만 보면 Gazebo RTF가 0.5 아래로 떨어질 때 벽시계 간격이 2s를
# 넘어 연결이 깜빡이고, control 이 그걸 io fault 로 보고 ARM 직후 RTL 로 빠졌다.
# 그래서 30Hz ATTITUDE 를 포함한 모든 수신으로 판단한다.
#   시간축: ROS clock. SITL 런치는 use_sim_time(/clock) 이라 PX4 lockstep 과 같은 시뮬
#   시간으로 잰다 — RTF 0.1~0.3 에서도 '시뮬 2 s 동안 무수신'일 때만 끊김이다(예전 벽시계
#   2 s 는 RTF 0.2 에서 시뮬 0.4 s 에 해당해 ARM 직후 RTL 을 일으켰다).
HB_TIMEOUT_SEC = 2.0
# /clock 이 멈추면(Gazebo 정지) 시뮬 시간도 멈춘다. 그때도 끊김을 알 수 있게 벽시계 상한을 둔다.
HB_WALL_BACKSTOP_SEC = 15.0
# PX4 Stabilized 모드 추력 곡선 (mc_att_control throttle_curve, MPC_THR_CURVE=2):
#   stick -1/0/+1 → MPC_MANTHR_MIN / MPC_THR_HOVER / MPC_THR_MAX.
# 자동 유도(stabilized_mode)의 FlightCommand.throttle 은 물리 추력(호버=MPC_THR_HOVER)이므로
# 이 곡선의 역함수로 스틱 값을 만든다. 수동(manual_mode)은 조종 스틱 그대로 보낸다.
THR_CURVE_PARAMS = {"MPC_MANTHR_MIN": 0.08, "MPC_THR_HOVER": 0.25, "MPC_THR_MAX": 1.0}
SERIAL_CONTROL_DEV_SHELL = 10
SERIAL_CONTROL_FLAG_RESPOND = 2
SERIAL_CONTROL_FLAG_EXCLUSIVE = 4


def thrust_to_stabilized_stick(thrust, thr_min, hover, thr_max):
    """물리 추력[0,1] → PX4 Stabilized 스틱[-1,1] (throttle_curve 의 역함수)."""
    t = min(max(float(thrust), 0.0), thr_max)
    if t >= hover:
        return min(1.0, (t - hover) / max(thr_max - hover, 1e-6))
    if t <= thr_min:
        return -1.0
    return -1.0 + (t - thr_min) / max(hover - thr_min, 1e-6)
GPS_RAW_INT_MESSAGE_ID = 24
GPS_STREAM_INTERVAL_US = 200_000  # 5 Hz: UI 표시에는 충분하고 MAVLink 부하는 작다.
ATTITUDE_MESSAGE_ID = 30
ATTITUDE_STREAM_INTERVAL_US = 33_333  # 30 Hz: 3D 자세 표현과 rosbag 분석용.


class ArmsSITLCommNode(Node):
    def require_parameter(self, name, parameter_type):
        self.declare_parameter(name, parameter_type)
        value = self.get_parameter(name).value
        if value is None:
            raise RuntimeError(
                f"Required parameter '{name}' is missing for "
                f"{self.get_fully_qualified_name()}")
        return value

    def __init__(self):
        super().__init__("arms_sitl_bridge_node")

        self.require_parameter("connection", Parameter.Type.STRING)
        self.require_parameter("send_rate_hz", Parameter.Type.DOUBLE)

        # 추력 변환이 의존하는 PX4 파라미터를 연결 때마다 PARAM_SET·확인한다. 런치의 일회성
        #   mavlink_shell param set 이 실패/지연돼도 곡선이 어긋나지 않게. 형식 "NAME:i|f:값".
        self.declare_parameter("enforce_params", [""])
        self._enforce = {}
        for item in self.get_parameter("enforce_params").value:
            if item:
                name, typ, val = item.split(":")
                self._enforce[name] = (typ, float(val))
        self._enforced_ok = set()
        self._conn_str   = self.get_parameter("connection").value
        self._send_rate  = self.get_parameter("send_rate_hz").value
        # PID 자동유도는 ACRO rate 명령을, PN 자동유도는 Stabilized attitude 명령을
        # 사용한다. manual_mode와 stabilized_mode는 모두 CH6 high를 뜻하지만,
        # 전자는 스틱 passthrough이고 후자는 자동 목표 자세 명령이다.

        self._command = FlightCommand()
        self._ch5_high  = False        # arm switch, level (not edge) — kept in sync with CH5
        self._force_disarm_pending = False  # arm-switch falling edge; consumed by 50 Hz sender
        self._mode_pending = None      # desired PX4 main mode on CH6 edge
        self._rtl_pending = False      # CH7 rising-edge RTL command; mode 변경 뒤 전송

        self._armed = False            # last known FC armed state, from HEARTBEAT
        self._last_arm_send = 0.0      # throttle for arm/disarm resend

        self._mav       = None
        self._connected = False
        self._lock      = threading.Lock()
        # PX4 재기동(패널 RESET) 감지용. heartbeat 이 끊겼다 돌아오면 그건 '새 PX4'이고,
        # 갓 부팅한 PX4 는 ACRO 가 아니다 — 모드를 다시 넣어주지 않으면 위 주석의
        # '조용히 기울기 명령으로 오해석' 상태로 계속 난다.
        self._last_rx   = None         # 마지막 MAVLink 수신 시각 (ROS clock [s])
        self._last_rx_wall = None      # 같은 수신의 벽시계 시각 (monotonic, 상한 판정용)
        self._thr_curve = dict(THR_CURVE_PARAMS)   # PX4 PARAM_VALUE 로 갱신
        self._reiniting = False        # _initial_setup 재실행 중복 방지
        self._attitude_seen = False    # PX4 ATTITUDE 첫 수신 로그(스트림 진단용)
        self._px4_main_mode = None     # HEARTBEAT 로 받은 PX4 실제 main mode (변경 시 로그)
        # Track A preserves the established telemetry/logging API.
        self._gps_pub = self.create_publisher(GpsTelemetry, "vehicle/gps", 10)
        self._attitude_pub = self.create_publisher(AttitudeTelemetry, "vehicle/attitude", 10)
        self._status_pub = self.create_publisher(IoStatus, "io/status", 10)
        # PX4 가 HEARTBEAT 로 알린 '실제' 무장 상태. FlightCommand.arm(요청)과 다르다 —
        #   PX4 가 ARM 을 거부하면 요청은 ON 인데 실제는 OFF 다. 발사기 분리 조건이 이걸 본다.
        self._armed_pub = self.create_publisher(Bool, "vehicle/armed", 10)
        self.create_subscription(FlightCommand, "control/flight_command", self._on_command, 10)
        # 발사기를 기울인 뒤 EKF 를 현재 자세로 다시 정렬시키는 서비스 (DISARM 상태에서만).
        self.create_service(Trigger, "sitl/reinit_estimator", self._srv_reinit_estimator)

        if not _PYMAVLINK:
            self.get_logger().error("pymavlink not installed: pip3 install pymavlink")
            return

        threading.Thread(target=self._connect_loop, daemon=True).start()
        threading.Thread(target=self._mavlink_rx_loop, daemon=True).start()

        self.create_timer(1.0 / self._send_rate, self._send_override)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f"sitl_bridge_node ready  conn={self._conn_str}")

    # ------------------------------------------------------------------
    # MAVLink connection
    # ------------------------------------------------------------------
    def _connect_loop(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to PX4: {self._conn_str} ...")
                mav = mavutil.mavlink_connection(self._conn_str)
                # HITL에서는 Jetson이 PX4가 있는 다른 호스트의 UDP 수신 포트로
                # ``udpout:`` 연결한다. PX4 UDP server가 이 송신지를 telemetry
                # 반환 peer로 등록하도록 heartbeat를 먼저 보낸다.
                if self._conn_str.startswith("udpout:"):
                    mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0)
                mav.wait_heartbeat(timeout=15)
                self._mav = mav
                self.get_logger().info(
                    f"Connected. system={mav.target_system} "
                    f"component={mav.target_component}")
                self._initial_setup()
                break
            except Exception as e:
                self.get_logger().warn(f"Connection failed: {e}. Retrying in 3s...")
                time.sleep(3.0)

    def _initial_setup(self):
        """Set the default ACRO mode after EKF convergence; arming is via CH5.
        Flight mode thereafter follows FlightCommand (ACRO or Stabilized)."""
        # PX4가 이전 SITL 실행의 저장 파라미터를 복원할 수 있다. 이 경우 airframe의
        # `set-default RC_MAP_KILL_SW 0`만으로는 CH7=kill을 해제하지 못한다.
        # 연결 직후 강제로 지워 RTL 첫 입력부터 motor kill이 되지 않게 한다.
        self._clear_kill_mapping()
        time.sleep(3.0)
        mode_name = "ACRO(각속도)"
        self.get_logger().info(f"Setting autonomous mode: {mode_name} (CH6 default)...")
        self._send_set_mode(PX4_CUSTOM_MAIN_MODE_ACRO)
        self._request_gps_stream()
        self._request_attitude_stream()
        self._request_thr_curve_params()
        time.sleep(0.5)

        self._connected = True
        self.get_logger().info(f"SITL: {mode_name} set. Waiting for arm signal (CH5).")

    def _restore_mode(self):
        """PX4 가 재기동된 뒤 비행 모드를 다시 넣는다 (rx 루프를 막지 않게 별 스레드).

        _initial_setup 은 EKF 수렴을 기다리느라 몇 초 잔다. rx 루프에서 직접 부르면
        그동안 heartbeat 을 못 읽어 또 '끊김'으로 오판한다.
        """
        if self._reiniting:
            return
        self._reiniting = True

        def worker():
            try:
                self.get_logger().info("PX4 재기동 감지 — 비행 모드 재설정")
                self._initial_setup()
            finally:
                self._reiniting = False

        threading.Thread(target=worker, daemon=True).start()

    # ------------------------------------------------------------------
    # MAVLink RX (armed-state tracking + SITL GPS/attitude telemetry)
    # ------------------------------------------------------------------
    def _mavlink_rx_loop(self):
        """HEARTBEAT·GPS_RAW_INT·ATTITUDE를 수신하고 PX4 재기동을 감지한다."""
        while rclpy.ok():
            if self._mav is None:
                time.sleep(0.1)
                continue
            try:
                msg = self._mav.recv_match(
                    type=['HEARTBEAT', 'GPS_RAW_INT', 'ATTITUDE', 'PARAM_VALUE'],
                    blocking=True, timeout=1.0)
                now = self._ros_now()
                wall = time.monotonic()
                if msg is None:
                    # heartbeat 끊김 = PX4 가 내려갔다 (패널 RESET 이거나 죽음).
                    if self._connected and self._rx_gap_exceeded(now, wall):
                        self._connected = False
                        self.get_logger().warn(
                            "PX4 MAVLink 수신 끊김 — 재기동 대기 (RESET?)")
                    continue
                t = msg.get_type()
                # 공백을 두고 돌아온 수신 = 갓 부팅한 새 PX4. 기본 모드는
                # ACRO 가 아니므로 다시 넣어준다. 안 넣으면 자동 출력이 각속도가
                # 아니라 기울기 명령으로 해석돼 '느리게 나는' 상태가 된다.
                #   판정 기준을 _connected 가 아니라 '공백'으로 잡는 이유: 최초 연결
                #   직후엔 _initial_setup 이 EKF 를 기다리며 자느라 _connected 가
                #   아직 False 다. 그걸 재기동으로 오해하면 기동할 때마다 모드를
                #   두 번 넣는다.
                if self._last_rx is not None and self._rx_gap_exceeded(now, wall):
                    self._restore_mode()
                if now is not None:
                    self._last_rx = now
                self._last_rx_wall = wall
                if t == 'HEARTBEAT':
                    self._armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                    self._log_px4_mode(msg)
                elif t == 'GPS_RAW_INT':
                    self._publish_gps(msg)
                elif t == 'ATTITUDE':
                    self._publish_attitude(msg)
                elif t == 'PARAM_VALUE':
                    self._on_param_value(msg)
            except Exception as e:
                self.get_logger().warn(f"MAVLink rx error: {e}", throttle_duration_sec=5.0)
                time.sleep(0.5)

    def _publish_gps(self, msg):
        """PX4 GPS_RAW_INT를 실기체 CRSF GPS와 같은 vehicle/gps 형식으로 변환한다."""
        fix_type = int(getattr(msg, 'fix_type', 1))
        altitude_mm = int(getattr(msg, 'alt', 0))
        velocity_cms = int(getattr(msg, 'vel', 0))
        course_cdeg = int(getattr(msg, 'cog', 0))

        gps = GpsTelemetry()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.latitude_deg = float(getattr(msg, 'lat', 0)) / 1e7
        gps.longitude_deg = float(getattr(msg, 'lon', 0)) / 1e7
        gps.altitude_m = altitude_mm / 1000.0
        # MAVLink의 UINT16_MAX는 unknown sentinel이다. UI에는 0으로 만들지 않고
        # NaN을 넘겨 잘못된 수치를 정상 값처럼 보이지 않게 한다.
        gps.ground_speed_mps = (float('nan') if velocity_cms == 0xFFFF
                                else velocity_cms / 100.0)
        gps.ground_course_deg = (float('nan') if course_cdeg == 0xFFFF
                                 else course_cdeg / 100.0)
        gps.satellites = int(getattr(msg, 'satellites_visible', 0))
        gps.mavlink_fix_type = fix_type
        gps.fix_type_inferred = False
        gps.valid_fix = fix_type >= 2  # MAVLink: 2D FIX 이상
        self._gps_pub.publish(gps)

    def _log_px4_mode(self, msg):
        """PX4 가 '실제로' 들어가 있는 main mode 를 바뀔 때마다 남긴다 (진단 전용).

        SET_MODE/CH6 는 요청일 뿐이라, ACRO 가 안 먹으면 rate 명령이 기울기로
        해석돼 '게인을 바꿔도 거의 안 꺾이는' 상태가 로그에 드러나지 않는다.
        """
        if not (msg.base_mode & PX4_BASE_MODE_CUSTOM):
            return
        main = (int(msg.custom_mode) >> 16) & 0xFF
        if main == self._px4_main_mode:
            return
        self._px4_main_mode = main
        name = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO",
                6: "OFFBOARD", 7: "STABILIZED"}.get(main, f"?{main}")
        self.get_logger().info(f"PX4 실제 모드 = {name}")

    def _publish_attitude(self, msg):
        """PX4 MAVLink ATTITUDE(rad)를 source-independent vehicle/attitude(deg)로 변환한다."""
        att = AttitudeTelemetry()
        att.header.stamp = self.get_clock().now().to_msg()
        att.roll_deg = math.degrees(float(getattr(msg, 'roll', 0.0)))
        att.pitch_deg = math.degrees(float(getattr(msg, 'pitch', 0.0)))
        att.yaw_deg = math.degrees(float(getattr(msg, 'yaw', 0.0)))
        self._attitude_pub.publish(att)
        if not self._attitude_seen:
            self._attitude_seen = True
            self.get_logger().info(
                "PX4 ATTITUDE stream active → vehicle/attitude "
                f"(roll={att.roll_deg:.1f}, pitch={att.pitch_deg:.1f}, yaw={att.yaw_deg:.1f} deg)")

    def _ros_now(self):
        """ROS clock [s]. use_sim_time 인데 /clock 이 아직 없으면 None."""
        ns = self.get_clock().now().nanoseconds
        return ns * 1e-9 if ns > 0 else None

    def _rx_age(self, now, wall):
        if self._last_rx_wall is None:
            return float('inf')
        if now is not None and self._last_rx is not None:
            return max(0.0, now - self._last_rx)
        return wall - self._last_rx_wall

    def _rx_gap_exceeded(self, now, wall):
        if self._last_rx_wall is None:
            return False
        return (self._rx_age(now, wall) > HB_TIMEOUT_SEC
                or wall - self._last_rx_wall > HB_WALL_BACKSTOP_SEC)

    def _publish_status(self):
        now, wall = self._ros_now(), time.monotonic()
        # 부팅 15 s 뒤 런치가 param set 으로 곡선 파라미터를 바꾼다 → 주기적으로 다시 읽는다.
        if self._connected and wall - getattr(self, "_thr_req_wall", 0.0) > 10.0:
            self._thr_req_wall = wall
            self._request_thr_curve_params()
        if self._connected and len(self._enforced_ok) < len(self._enforce) \
                and wall - getattr(self, "_enforce_wall", 0.0) > 3.0:
            self._enforce_wall = wall
            self._send_enforced_params()
        age = self._rx_age(now, wall)
        if self._last_rx_wall is not None and wall - self._last_rx_wall > HB_WALL_BACKSTOP_SEC:
            age = max(age, HB_TIMEOUT_SEC + 1.0)
        status = IoStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.source = "px4"
        status.transport_connected = self._connected and age < HB_TIMEOUT_SEC
        status.vehicle_telemetry_connected = status.transport_connected
        status.last_rx_age_ms = 0xffffffff if not math.isfinite(age) else min(int(age * 1000), 0xffffffff)
        status.last_vehicle_telemetry_age_ms = status.last_rx_age_ms
        # UART 전용 계측값은 SITL/MAVLink에 해당하지 않는다. 0으로 보이는 것을
        # 방지해 로그 분석기에서 이 선을 표시하지 않도록 NaN으로 둔다.
        status.tx_uart_write_hz = float('nan')
        status.rx_uart_valid_frame_hz = float('nan')
        status.rx_uart_byte_rate = float('nan')
        status.rx_uart_crc_error_hz = float('nan')
        status.fault_reason = "" if status.transport_connected else "PX4 heartbeat timeout"
        self._status_pub.publish(status)
        self._armed_pub.publish(Bool(data=bool(self._armed and status.transport_connected)))

    # ------------------------------------------------------------------
    # Vehicle-independent command input
    # ------------------------------------------------------------------
    def _on_command(self, command: FlightCommand):
        """Map semantic command edges to PX4 services; axes go out at the timer rate."""
        with self._lock:
            requested_stabilized = command.manual_mode or command.stabilized_mode
            previous_stabilized = self._command.manual_mode or self._command.stabilized_mode
            if requested_stabilized != previous_stabilized:
                self._mode_pending = (PX4_CUSTOM_MAIN_MODE_STABILIZED
                                      if requested_stabilized else PX4_CUSTOM_MAIN_MODE_ACRO)
            if command.rtl_request and not self._command.rtl_request:
                self._rtl_pending = True
            if self._ch5_high and not command.arm:
                # An ARM-switch fall is the operator's emergency motor-cut
                # request.  A normal PX4 disarm is rejected in flight, so
                # preserve this edge until the sender emits a forced disarm.
                self._force_disarm_pending = True
            self._command = command
            self._ch5_high = command.arm

    # ------------------------------------------------------------------
    # 50 Hz send timer
    # ------------------------------------------------------------------
    def _send_override(self):
        if not self._connected or self._mav is None:
            return

        with self._lock:
            command = self._command
            ch5_high = self._ch5_high
            mode_req = self._mode_pending
            self._mode_pending = None
            rtl_req = self._rtl_pending
            self._rtl_pending = False
            force_disarm = self._force_disarm_pending
            self._force_disarm_pending = False

        # Arm/disarm: keep resending while the FC's actual armed state doesn't match
        # CH5 yet. A single MAV_CMD_COMPONENT_ARM_DISARM can be TEMPORARILY_REJECTED
        # (e.g. EKF/position estimate not converged yet right after boot) — without a
        # retry that one lost attempt meant "denied forever" until CH5 toggled again.
        now = time.time()
        if force_disarm:
            self._last_arm_send = now
            self.get_logger().warn(
                "CH5=OFF edge → sending FORCE DISARM (PX4 in-flight motor cut)")
            self._send_arm(False, force_disarm=True)
        elif ch5_high != self._armed and (now - self._last_arm_send) >= ARM_RETRY_INTERVAL_SEC:
            self._last_arm_send = now
            self.get_logger().info(
                f"CH5={'ON' if ch5_high else 'OFF'}, armed={self._armed} → "
                f"sending {'ARM' if ch5_high else 'DISARM'}")
            self._send_arm(ch5_high)

        # Handle pending flight-mode change (CH6)
        if mode_req is not None:
            name = {PX4_CUSTOM_MAIN_MODE_STABILIZED: "Stabilized",
                    PX4_CUSTOM_MAIN_MODE_ACRO:   "ACRO(각속도)"}.get(mode_req, "?")
            self.get_logger().info(f"CH6 → {name} mode")
            self._send_set_mode(mode_req)

        # RTL은 CH6의 manual/stabilized 전환 *뒤*에 보낸다. 반대 순서면 같은 틱의
        # CH6 mode 명령이 PX4 RTL을 즉시 덮어쓸 수 있다.
        if rtl_req:
            self.get_logger().warn("CH7 RTL request → MAV_CMD_NAV_RETURN_TO_LAUNCH")
            self._send_rtl()

        # RC_CHANNELS_OVERRIDE: semantic normalized command → microseconds.
        def axis(v):
            return int(1500 + max(-1.0, min(1.0, v)) * 500)
        us = [1500] * 8
        if command.stabilized_mode and not command.manual_mode:
            # 자동 Stabilized: 물리 추력 → PX4 추력 곡선 역변환 스틱. 이게 없으면
            #   호버(0.32)가 스틱 0.32(=-0.36)로 나가 실제 추력 ≈0.23 이 됐다.
            c = self._thr_curve
            stick = thrust_to_stabilized_stick(command.throttle, c["MPC_MANTHR_MIN"],
                                               c["MPC_THR_HOVER"], c["MPC_THR_MAX"])
            thr_us = int(1500 + stick * 500)
        else:
            # 수동 스틱 passthrough / ACRO(추력 = (stick+1)/2 로 선형) 는 그대로.
            thr_us = int(1000 + max(0.0, min(1.0, command.throttle)) * 1000)
        us[0], us[1], us[2], us[3] = (axis(command.roll), axis(command.pitch),
                                      thr_us, axis(command.yaw))
        us[4] = 2000 if command.arm else 1000
        us[5] = 2000 if (command.manual_mode or command.stabilized_mode) else 1000
        us[6] = 2000 if command.rtl_request else 1000
        try:
            self._mav.mav.rc_channels_override_send(
                self._mav.target_system,
                self._mav.target_component,
                us[0], us[1], us[2], us[3],
                us[4], us[5], us[6], us[7],
            )
        except Exception as e:
            self.get_logger().warn(
                f"RC override failed: {e}", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # MAVLink helpers
    # ------------------------------------------------------------------
    def _send_arm(self, arm: bool, force_disarm: bool = False):
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            FORCE_DISARM_MAGIC if force_disarm else 0.0,
            0, 0, 0, 0, 0,
        )

    def _send_set_mode(self, main_mode: int, sub_mode: int = 0):
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(PX4_BASE_MODE_CUSTOM),
            float(main_mode),
            float(sub_mode),
            0, 0, 0, 0,
        )

    def _send_rtl(self):
        """CH7 RTL 요청을 PX4 Return mode로 전달한다 (RC kill과 완전히 분리)."""
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )

    def _clear_kill_mapping(self):
        """SITL PX4에 남아 있을 수 있는 CH7 kill 매핑을 즉시 제거한다."""
        if self._mav is None:
            return
        try:
            self._mav.mav.param_set_send(
                self._mav.target_system, self._mav.target_component,
                b"RC_MAP_KILL_SW", 0.0,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
        except Exception as e:
            self.get_logger().warn(
                f"RC_MAP_KILL_SW clear failed: {e}", throttle_duration_sec=5.0)

    def _request_thr_curve_params(self):
        """Stabilized 추력 곡선 파라미터를 PX4 에서 읽어 역변환에 쓴다(기본값은 airframe 값)."""
        if self._mav is None:
            return
        for name in THR_CURVE_PARAMS:
            try:
                self._mav.mav.param_request_read_send(
                    self._mav.target_system, self._mav.target_component,
                    name.encode(), -1)
            except Exception as e:
                self.get_logger().warn(f"{name} 읽기 요청 실패: {e}", throttle_duration_sec=5.0)

    def _send_enforced_params(self):
        for name, (typ, val) in self._enforce.items():
            if name in self._enforced_ok:
                continue
            ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32 if typ == "i"
                     else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            # PX4 는 PARAM_ENCODE_BYTEWISE: INT32 는 float 필드에 비트 그대로 싣는다.
            wire = (struct.unpack("<f", struct.pack("<i", int(val)))[0] if typ == "i"
                    else float(val))
            try:
                self._mav.mav.param_set_send(self._mav.target_system, self._mav.target_component,
                                             name.encode(), wire, ptype)
                self._mav.mav.param_request_read_send(self._mav.target_system,
                                                      self._mav.target_component, name.encode(), -1)
            except Exception as e:
                self.get_logger().warn(f"{name} PARAM_SET 실패: {e}", throttle_duration_sec=5.0)

    def _on_param_value(self, msg):
        name = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) \
            else msg.param_id.decode(errors="replace").rstrip("\x00")
        if name in self._enforce and name not in self._enforced_ok:
            want = self._enforce[name][1]
            got = float(msg.param_value)
            if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                got = float(struct.unpack("<i", struct.pack("<f", msg.param_value))[0])
            if abs(got - want) < 1e-3:
                self._enforced_ok.add(name)
                self.get_logger().info(f"PX4 {name}={want:g} 확인 (추력 변환 전제 파라미터)")
        if name in self._thr_curve:
            value = float(msg.param_value)
            if abs(value - self._thr_curve[name]) > 1e-4:
                self.get_logger().info(
                    f"PX4 {name}={value:.3f} (기본 {self._thr_curve[name]:.3f}) → Stabilized "
                    f"추력 역변환에 반영")
            self._thr_curve[name] = value

    def _shell(self, line):
        """PX4 nsh 셸에 한 줄을 보낸다 (MAVLink SERIAL_CONTROL, mavlink_shell.py 와 같은 방식)."""
        data = list((line + "\n").encode())
        while data:
            chunk, data = data[:70], data[70:]
            n = len(chunk)
            self._mav.mav.serial_control_send(
                SERIAL_CONTROL_DEV_SHELL,
                SERIAL_CONTROL_FLAG_EXCLUSIVE | SERIAL_CONTROL_FLAG_RESPOND,
                0, 0, n, chunk + [0] * (70 - n))

    def _srv_reinit_estimator(self, _req, res):
        """EKF2 를 재시작해 현재(정지) 자세로 다시 정렬시킨다.

        발사기는 set_pose(순간이동)로 기체를 기울인다. 각속도가 없으니 자이로가 회전을
        못 보고, PX4 1.18 EKF2 는 GPS 수평 보정 중 중력 벡터 융합을 끈다(isHorizontalAidingActive).
        그래서 기울인 뒤 PX4 자세가 수직 쪽에 남아 수십 초에 걸쳐 일부만 수렴했다.
        재시작한 EKF 는 정지 상태 가속도로 기울기를, 자력계로 heading 을 새로 잡는다.
        """
        if self._mav is None or not self._connected:
            res.success, res.message = False, "PX4 미연결"
            return res
        if self._armed or self._ch5_high:
            res.success, res.message = False, "ARM 상태에서는 EKF 재정렬 금지"
            return res
        try:
            self._shell("ekf2 stop")
            time.sleep(0.5)
            self._shell("ekf2 start")
            time.sleep(0.2)
            self._mav.mav.serial_control_send(SERIAL_CONTROL_DEV_SHELL, 0, 0, 0, 0, [0] * 70)
        except Exception as e:
            res.success, res.message = False, f"MAVLink shell 실패: {e}"
            return res
        self.get_logger().warn("EKF2 재시작 요청(발사기 자세 변경 후 재정렬) — 수렴까지 수 초")
        res.success, res.message = True, "ekf2 restart 요청 완료"
        return res

    def _request_gps_stream(self):
        """PX4에 GPS_RAW_INT 주기 전송을 요청한다.

        Gazebo NavSat 센서와 PX4 EKF가 정상이어도 MAVLink 기본 stream profile에 따라
        GPS_RAW_INT가 느리거나 빠질 수 있다. SITL UI용으로 5 Hz를 명시 요청한다.
        지원하지 않는 PX4 버전은 command를 무시할 뿐 RC bridge에는 영향이 없다.
        """
        if self._mav is None:
            return
        try:
            self._mav.mav.command_long_send(
                self._mav.target_system, self._mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(GPS_RAW_INT_MESSAGE_ID),
                float(GPS_STREAM_INTERVAL_US),
                0, 0, 0, 0, 0,
            )
        except Exception as e:
            self.get_logger().warn(
                f"GPS_RAW_INT stream request failed: {e}", throttle_duration_sec=5.0)

    def _request_attitude_stream(self):
        """PX4 ATTITUDE를 30 Hz로 요청해 3D 뷰/rosbag 자세를 채운다."""
        if self._mav is None:
            return
        try:
            self._mav.mav.command_long_send(
                self._mav.target_system, self._mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(ATTITUDE_MESSAGE_ID),
                float(ATTITUDE_STREAM_INTERVAL_US),
                0, 0, 0, 0, 0,
            )
        except Exception as e:
            self.get_logger().warn(
                f"ATTITUDE stream request failed: {e}", throttle_duration_sec=5.0)
        # 일부 PX4/MAVLink 조합은 SET_MESSAGE_INTERVAL을 무시하거나 stream profile이
        # 다시 덮어쓴다. EXTRA1은 ATTITUDE가 속한 표준 data stream이므로 함께 요청한다.
        try:
            self._mav.mav.request_data_stream_send(
                self._mav.target_system, self._mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 30, 1)
        except Exception as e:
            self.get_logger().warn(
                f"ATTITUDE EXTRA1 stream request failed: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArmsSITLCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
