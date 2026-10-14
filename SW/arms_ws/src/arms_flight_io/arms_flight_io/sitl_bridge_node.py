#!/usr/bin/env python3
"""
sitl_bridge_node — FlightCommand→MAVLink adapter for SITL.

``control/flight_command``를 PX4의 RC override, arm/disarm, 비행 모드와 RTL MAVLink
명령으로 변환한다. PX4 GPS_RAW_INT와 ATTITUDE는 환경 독립 ``vehicle/*`` 토픽으로
정규화해 UI와 rosbag이 실기체와 같은 계약을 사용하게 한다.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

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
# 이 시간 이상 heartbeat 이 없으면 PX4 가 내려간 것으로 본다(패널 RESET 이 PX4 만 재기동).
# PX4 는 1Hz 로 heartbeat 을 보낸다. 2s는 한 heartbeat 유실은 허용하면서
# 재기동/단절 상태를 기존 3s보다 빠르게 UI·control에 전달한다.
HB_TIMEOUT_SEC = 2.0
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

        self._conn_str   = self.get_parameter("connection").value
        self._send_rate  = self.get_parameter("send_rate_hz").value
        # 자동(CH6 low)은 ACRO 고정이다. 고를 여지가 없다 -- arms_control_node 가
        # 내는 자동 출력은 각속도[deg/s]를 max_rate_dps 로 정규화한 값이고, PX4 가
        # 그걸 각속도로 되돌리는 모드는 ACRO 뿐이다. Manual(각도) 로 넣으면 같은
        # 숫자에 MPC_MAN_TILT_MAX 가 곱해져 '기울기 명령'으로 조용히 오해석된다
        # (안 떨어지고 그냥 다른 제어 법칙으로 난다 -- 그래서 더 나쁘다).

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
        self._last_hb   = 0.0          # 마지막 heartbeat 시각 (monotonic)
        self._reiniting = False        # _initial_setup 재실행 중복 방지
        self._attitude_seen = False    # PX4 ATTITUDE 첫 수신 로그(스트림 진단용)
        # Track A preserves the established telemetry/logging API.
        self._gps_pub = self.create_publisher(GpsTelemetry, "vehicle/gps", 10)
        self._attitude_pub = self.create_publisher(AttitudeTelemetry, "vehicle/attitude", 10)
        self._status_pub = self.create_publisher(IoStatus, "io/status", 10)
        self.create_subscription(FlightCommand, "control/flight_command", self._on_command, 10)

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
        """Set default autonomous mode after EKF convergence; arming is done via CH5.
        Flight mode thereafter follows CH6 (low=ACRO/자동, high=Stabilized/수동)."""
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
                msg = self._mav.recv_match(type=['HEARTBEAT', 'GPS_RAW_INT', 'ATTITUDE'],
                                           blocking=True, timeout=1.0)
                now = time.monotonic()
                if msg is None:
                    # heartbeat 끊김 = PX4 가 내려갔다 (패널 RESET 이거나 죽음).
                    if self._connected and now - self._last_hb > HB_TIMEOUT_SEC:
                        self._connected = False
                        self.get_logger().warn(
                            "PX4 heartbeat 끊김 — 재기동 대기 (RESET?)")
                    continue
                t = msg.get_type()
                if t == 'HEARTBEAT':
                    # 공백을 두고 돌아온 heartbeat = 갓 부팅한 새 PX4. 기본 모드는
                    # ACRO 가 아니므로 다시 넣어준다. 안 넣으면 자동 출력이 각속도가
                    # 아니라 기울기 명령으로 해석돼 '느리게 나는' 상태가 된다.
                    #   판정 기준을 _connected 가 아니라 '공백'으로 잡는 이유: 최초 연결
                    #   직후엔 _initial_setup 이 EKF 를 기다리며 자느라 _connected 가
                    #   아직 False 다. 그걸 재기동으로 오해하면 기동할 때마다 모드를
                    #   두 번 넣는다.
                    if self._last_hb > 0.0 and (now - self._last_hb) > HB_TIMEOUT_SEC:
                        self._restore_mode()
                    self._last_hb = now
                    self._armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == 'GPS_RAW_INT':
                    self._publish_gps(msg)
                elif t == 'ATTITUDE':
                    self._publish_attitude(msg)
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

    def _publish_status(self):
        age = time.monotonic() - self._last_hb if self._last_hb else float('inf')
        status = IoStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.source = "px4"
        status.transport_connected = self._connected and age < HB_TIMEOUT_SEC
        status.vehicle_telemetry_connected = status.transport_connected
        status.last_rx_age_ms = 0xffffffff if not math.isfinite(age) else min(int(age * 1000), 0xffffffff)
        status.last_vehicle_telemetry_age_ms = status.last_rx_age_ms
        status.fault_reason = "" if status.transport_connected else "PX4 heartbeat timeout"
        self._status_pub.publish(status)

    # ------------------------------------------------------------------
    # Vehicle-independent command input
    # ------------------------------------------------------------------
    def _on_command(self, command: FlightCommand):
        """Map semantic command edges to PX4 services; axes go out at the timer rate."""
        with self._lock:
            if command.manual_mode != self._command.manual_mode:
                self._mode_pending = (PX4_CUSTOM_MAIN_MODE_STABILIZED
                                      if command.manual_mode else PX4_CUSTOM_MAIN_MODE_ACRO)
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
        us[0], us[1], us[2], us[3] = (axis(command.roll), axis(command.pitch),
                                      int(1000 + max(0.0, min(1.0, command.throttle)) * 1000),
                                      axis(command.yaw))
        us[4] = 2000 if command.arm else 1000
        us[5] = 2000 if command.manual_mode else 1000
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
