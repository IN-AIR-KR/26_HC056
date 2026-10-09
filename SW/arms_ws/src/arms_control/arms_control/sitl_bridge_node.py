#!/usr/bin/env python3
"""
sitl_bridge_node — CRSF→MAVLink bridge for SITL

Reads CRSF RC frames from a virtual serial port (socat PTY pair) and converts
them to MAVLink RC_CHANNELS_OVERRIDE toward PX4. PX4가 Gazebo NavSat 센서에서
만든 GPS_RAW_INT와 ATTITUDE는 각각 /arms/gps와 /arms/attitude 호환 텔레메트리로
다시 발행해 SITL UI/rosbag도 실기체와 같은 토픽을 쓴다.

Channel mapping:
  CH1: roll     CH2: pitch     CH3: throttle  CH4: yaw
  CH5: arm sw   CH6: mode sw   CH7: RTL request   CH8: (unused)

CH6 flight mode: low=ACRO(자동/영상유도, 젯슨이 계산한 CRSF), high=Stabilized(수동/사람 입력).
  high=Stabilized 는 자동수평 + throttle 직결(고도유지 없음). 실기체 FC 의 수동 모드와 동작을 맞춘 것.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node

from arms_msgs.msg import AttitudeTelemetry, GpsTelemetry

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

CRSF_SYNC   = 0xC8
CRSF_TYPE_RC = 0x16
CRSF_MIN     = 172
CRSF_MAX     = 1811
SWITCH_THRESH = 1500  # > threshold → switch is ON

MAV_MODE_FLAG_SAFETY_ARMED = 128
ARM_RETRY_INTERVAL_SEC = 1.0  # resend arm/disarm while desired != actual armed state
# 이 시간 이상 heartbeat 이 없으면 PX4 가 내려간 것으로 본다(패널 RESET 이 PX4 만 재기동).
# PX4 는 1Hz 로 heartbeat 을 보내므로 3s 면 오탐 없이 충분히 빠르다.
HB_TIMEOUT_SEC = 3.0
GPS_RAW_INT_MESSAGE_ID = 24
GPS_STREAM_INTERVAL_US = 200_000  # 5 Hz: UI 표시에는 충분하고 MAVLink 부하는 작다.
ATTITUDE_MESSAGE_ID = 30
ATTITUDE_STREAM_INTERVAL_US = 33_333  # 30 Hz: 3D 자세 표현과 rosbag 분석용.


def _crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    return crc


def _crsf_to_us(v: int) -> int:
    """CRSF units (172–1811) → microseconds (1000–2000)."""
    return int(1000 + (v - CRSF_MIN) / (CRSF_MAX - CRSF_MIN) * 1000)


def _decode_crsf_frame(fd) -> list | None:
    """
    Read one CRSF RC frame from binary file-like fd.
    Returns list of 16 channel values (CRSF units) or None on error.
    """
    # Scan for sync byte
    while True:
        b = fd.read(1)
        if not b:
            return None
        if b[0] == CRSF_SYNC:
            break

    lb = fd.read(1)
    if not lb:
        return None
    length = lb[0]  # type + payload + crc

    rest = fd.read(length)
    if len(rest) < length:
        return None

    frame_type = rest[0]
    crc_received = rest[-1]
    if frame_type != CRSF_TYPE_RC:
        return None  # skip non-RC frames

    # CRC over type + payload
    expected_crc = _crc8_dvb_s2(rest[:-1])
    if crc_received != expected_crc:
        return None

    payload = rest[1:-1]  # 22 bytes for 16-channel RC frame
    if len(payload) < 22:
        return None

    # Unpack 16 × 11-bit channels (little-endian bit order)
    channels = []
    bit_pos = 0
    for _ in range(16):
        val = 0
        for b in range(11):
            byte_idx = bit_pos // 8
            bit_idx = bit_pos % 8
            if byte_idx < len(payload):
                val |= ((payload[byte_idx] >> bit_idx) & 1) << b
            bit_pos += 1
        channels.append(val)

    return channels


class ArmsSITLCommNode(Node):
    def __init__(self):
        super().__init__("arms_sitl_bridge_node")

        self.declare_parameter("connection", "udpin:0.0.0.0:14540")
        self.declare_parameter("crsf_port", "/tmp/crsf_rx")
        self.declare_parameter("send_rate_hz", 50.0)

        self._conn_str   = self.get_parameter("connection").value
        self._crsf_port  = self.get_parameter("crsf_port").value
        self._send_rate  = self.get_parameter("send_rate_hz").value
        # 자동(CH6 low)은 ACRO 고정이다. 고를 여지가 없다 -- arms_control_node 가
        # 내는 자동 출력은 각속도[deg/s]를 max_rate_dps 로 정규화한 값이고, PX4 가
        # 그걸 각속도로 되돌리는 모드는 ACRO 뿐이다. Manual(각도) 로 넣으면 같은
        # 숫자에 MPC_MAN_TILT_MAX 가 곱해져 '기울기 명령'으로 조용히 오해석된다
        # (안 떨어지고 그냥 다른 제어 법칙으로 난다 -- 그래서 더 나쁘다).

        self._channels  = [CRSF_MIN] * 16
        self._channels[2] = CRSF_MIN   # throttle min
        self._prev_ch6  = CRSF_MIN     # flight-mode switch (for edge detect)
        self._prev_ch7  = CRSF_MIN     # RTL switch (rising edge → MAV_CMD_NAV_RETURN_TO_LAUNCH)
        self._ch5_high  = False        # arm switch, level (not edge) — kept in sync with CH5
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
        # 실기체는 arms_control_node가 ELRS 수신 CRSF GPS를 발행한다. SITL은 ELRS가
        # 없으므로 여기서 PX4 MAVLink GPS_RAW_INT를 같은 인터페이스로 공급한다.
        self._gps_pub = self.create_publisher(GpsTelemetry, "/arms/gps", 10)
        # 실기체는 arms_control_node가 CRSF Attitude(0x1E)를 /arms/attitude로 낸다.
        # SITL은 PX4 MAVLink ATTITUDE(rad)를 같은 roll/pitch/yaw [deg] 인터페이스로 낸다.
        self._attitude_pub = self.create_publisher(AttitudeTelemetry, "/arms/attitude", 10)

        if not _PYMAVLINK:
            self.get_logger().error("pymavlink not installed: pip3 install pymavlink")
            return

        threading.Thread(target=self._connect_loop, daemon=True).start()
        threading.Thread(target=self._crsf_read_loop, daemon=True).start()
        threading.Thread(target=self._mavlink_rx_loop, daemon=True).start()

        self.create_timer(1.0 / self._send_rate, self._send_override)
        self.get_logger().info(
            f"sitl_bridge_node ready  conn={self._conn_str}  crsf={self._crsf_port}")

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
        """PX4 GPS_RAW_INT를 실기체 CRSF GPS와 같은 /arms/gps 형식으로 변환한다."""
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
        """PX4 MAVLink ATTITUDE(rad)를 source-independent /arms/attitude(deg)로 변환한다."""
        att = AttitudeTelemetry()
        att.header.stamp = self.get_clock().now().to_msg()
        att.roll_deg = math.degrees(float(getattr(msg, 'roll', 0.0)))
        att.pitch_deg = math.degrees(float(getattr(msg, 'pitch', 0.0)))
        att.yaw_deg = math.degrees(float(getattr(msg, 'yaw', 0.0)))
        self._attitude_pub.publish(att)
        if not self._attitude_seen:
            self._attitude_seen = True
            self.get_logger().info(
                "PX4 ATTITUDE stream active → /arms/attitude "
                f"(roll={att.roll_deg:.1f}, pitch={att.pitch_deg:.1f}, yaw={att.yaw_deg:.1f} deg)")

    # ------------------------------------------------------------------
    # CRSF reader
    # ------------------------------------------------------------------
    def _crsf_read_loop(self):
        while rclpy.ok():
            try:
                fd = open(self._crsf_port, 'rb', buffering=0)
                self.get_logger().info(f"CRSF port opened: {self._crsf_port}")
                while rclpy.ok():
                    channels = _decode_crsf_frame(fd)
                    if channels:
                        with self._lock:
                            self._channels = channels
                            self._process_switches(channels)
            except Exception as e:
                self.get_logger().warn(f"CRSF read error: {e}. Retrying in 2s...")
                time.sleep(2.0)

    def _process_switches(self, channels):
        """Track CH5 arm, CH6 mode, and CH7 RTL switch edges."""
        self._ch5_high = channels[4] > SWITCH_THRESH

        ch6 = channels[5]
        ch6_high = ch6 > SWITCH_THRESH
        if ch6_high != (self._prev_ch6 > SWITCH_THRESH):
            # CH6 low→ACRO(자동/영상유도), high→Stabilized(수동/사람 입력)
            self._mode_pending = (PX4_CUSTOM_MAIN_MODE_STABILIZED if ch6_high
                                  else PX4_CUSTOM_MAIN_MODE_ACRO)
        self._prev_ch6 = ch6

        ch7 = channels[6]
        if ch7 > SWITCH_THRESH and self._prev_ch7 <= SWITCH_THRESH:
            # CH7은 PX4 RC kill channel이 아니다. RTL은 level mode가 아니라 명령으로
            # 한 번만 보낸다. control 노드가 State::RTL을 유지하므로 버튼을 놓아도
            # 상태/UI와 FC 귀환 모드가 불일치하지 않는다.
            self._rtl_pending = True
        self._prev_ch7 = ch7

    # ------------------------------------------------------------------
    # 50 Hz send timer
    # ------------------------------------------------------------------
    def _send_override(self):
        if not self._connected or self._mav is None:
            return

        with self._lock:
            chs = list(self._channels)
            ch5_high = self._ch5_high
            mode_req = self._mode_pending
            self._mode_pending = None
            rtl_req = self._rtl_pending
            self._rtl_pending = False

        # Arm/disarm: keep resending while the FC's actual armed state doesn't match
        # CH5 yet. A single MAV_CMD_COMPONENT_ARM_DISARM can be TEMPORARILY_REJECTED
        # (e.g. EKF/position estimate not converged yet right after boot) — without a
        # retry that one lost attempt meant "denied forever" until CH5 toggled again.
        now = time.time()
        if ch5_high != self._armed and (now - self._last_arm_send) >= ARM_RETRY_INTERVAL_SEC:
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

        # RC_CHANNELS_OVERRIDE: CRSF → microseconds
        us = [_crsf_to_us(v) for v in chs]
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
    def _send_arm(self, arm: bool):
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0, 0, 0, 0, 0, 0,
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
