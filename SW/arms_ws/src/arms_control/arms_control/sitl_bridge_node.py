#!/usr/bin/env python3
"""
sitl_bridge_node — CRSF→MAVLink bridge for SITL

Reads CRSF RC frames from a virtual serial port (socat PTY pair) and converts
them to MAVLink RC_CHANNELS_OVERRIDE toward PX4.  PX4가 Gazebo NavSat 센서에서
만든 GPS_RAW_INT는 /arms/gps로 다시 발행해 SITL UI도 실기체와 같은 토픽을 쓴다.

Channel mapping:
  CH1: roll     CH2: pitch     CH3: throttle  CH4: yaw
  CH5: arm sw   CH6: mode sw   CH7: kill sw   CH8: (unused)

CH6 flight mode: low=ACRO(자동/영상유도, 젯슨이 계산한 CRSF), high=Stabilized(수동/사람 입력).
  high=Stabilized 는 자동수평 + throttle 직결(고도유지 없음). 실기체 FC 의 수동 모드와 동작을 맞춘 것.
"""

import threading
import time

import rclpy
from rclpy.node import Node

from arms_msgs.msg import GpsTelemetry

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
        self._ch5_high  = False        # arm switch, level (not edge) — kept in sync with CH5
        self._mode_pending = None      # desired PX4 main mode on CH6 edge

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
        # 실기체는 arms_control_node가 ELRS 수신 CRSF GPS를 발행한다. SITL은 ELRS가
        # 없으므로 여기서 PX4 MAVLink GPS_RAW_INT를 같은 인터페이스로 공급한다.
        self._gps_pub = self.create_publisher(GpsTelemetry, "/arms/gps", 10)

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
        time.sleep(3.0)
        mode_name = "ACRO(각속도)"
        self.get_logger().info(f"Setting autonomous mode: {mode_name} (CH6 default)...")
        self._send_set_mode(PX4_CUSTOM_MAIN_MODE_ACRO)
        self._request_gps_stream()
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
    # MAVLink RX (armed-state tracking via HEARTBEAT + SITL GPS telemetry)
    # ------------------------------------------------------------------
    def _mavlink_rx_loop(self):
        """HEARTBEAT와 GPS_RAW_INT를 수신하고 PX4 재기동을 감지한다."""
        while rclpy.ok():
            if self._mav is None:
                time.sleep(0.1)
                continue
            try:
                msg = self._mav.recv_match(type=['HEARTBEAT', 'GPS_RAW_INT'],
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
        """Track CH5 switch level (arm target) and CH6 flight-mode edges."""
        self._ch5_high = channels[4] > SWITCH_THRESH

        ch6 = channels[5]
        ch6_high = ch6 > SWITCH_THRESH
        if ch6_high != (self._prev_ch6 > SWITCH_THRESH):
            # CH6 low→ACRO(자동/영상유도), high→Stabilized(수동/사람 입력)
            self._mode_pending = (PX4_CUSTOM_MAIN_MODE_STABILIZED if ch6_high
                                  else PX4_CUSTOM_MAIN_MODE_ACRO)
        self._prev_ch6 = ch6

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
