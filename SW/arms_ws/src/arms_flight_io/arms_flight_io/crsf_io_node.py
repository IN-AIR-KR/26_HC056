#!/usr/bin/env python3
"""Real-hardware CRSF adapter.

This is the only process that opens the ELRS/CRSF UART.  It maps the
vehicle-independent ``control/flight_command`` to the Betaflight AETR/AUX
layout and normalizes received telemetry into ``vehicle/*``.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int16MultiArray
from arms_msgs.msg import (AttitudeTelemetry, BatteryTelemetry, CrsfTelemetry,
                           FlightCommand, GpsTelemetry, IoStatus)

try:
    import serial
except ImportError:  # reported at runtime so a non-hardware launch can still build
    serial = None

CRSF_MIN, CRSF_CENTER, CRSF_MAX = 172, 992, 1811


def _crc(data: bytes) -> int:
    c = 0
    for value in data:
        c ^= value
        for _ in range(8):
            c = ((c << 1) ^ 0xD5) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def _norm(v: float) -> int:
    v = max(-1.0, min(1.0, v))
    return int(CRSF_CENTER + v * (CRSF_MAX - CRSF_CENTER if v >= 0 else CRSF_CENTER - CRSF_MIN))


def _throttle(v: float) -> int:
    return int(CRSF_MIN + max(0.0, min(1.0, v)) * (CRSF_MAX - CRSF_MIN))


def _packed_channels(channels: list[int]) -> bytes:
    payload = bytearray(22)
    bit = 0
    for value in channels[:16]:
        value = max(CRSF_MIN, min(CRSF_MAX, int(value)))
        for b in range(11):
            if value & (1 << b):
                payload[bit // 8] |= 1 << (bit % 8)
            bit += 1
    body = bytes([0x16]) + bytes(payload)
    return bytes([0xC8, len(body) + 1]) + body + bytes([_crc(body)])


class CrsfIoNode(Node):
    def require_parameter(self, name, parameter_type):
        self.declare_parameter(name, parameter_type)
        value = self.get_parameter(name).value
        if value is None:
            raise RuntimeError(
                f"Required parameter '{name}' is missing for "
                f"{self.get_fully_qualified_name()}")
        return value

    def __init__(self):
        super().__init__("arms_crsf_io_node")
        self.require_parameter("port", Parameter.Type.STRING)
        self.require_parameter("baud", Parameter.Type.INTEGER)
        self.require_parameter("send_rate_hz", Parameter.Type.DOUBLE)
        self.require_parameter("telemetry_timeout_sec", Parameter.Type.DOUBLE)
        self.require_parameter("battery.cell_full_v", Parameter.Type.DOUBLE)
        self.require_parameter("battery.cell_empty_v", Parameter.Type.DOUBLE)
        self._port = self.get_parameter("port").value
        self._baud = int(self.get_parameter("baud").value)
        self._timeout = float(self.get_parameter("telemetry_timeout_sec").value)
        self._ser = None
        self._rx = bytearray()
        self._command = FlightCommand()
        self._last_any = 0.0
        self._last_vehicle = 0.0
        self._cells = 0
        self._telem = CrsfTelemetry()

        self.create_subscription(FlightCommand, "control/flight_command", self._on_command, 10)
        # Track A preserves the established telemetry/logging API.  Only the
        # command boundary and I/O ownership move in this track.
        self._tx_pub = self.create_publisher(Int16MultiArray, "io/crsf/tx", 10)
        self._rx_pub = self.create_publisher(CrsfTelemetry, "io/crsf/rx", 10)
        self._gps_pub = self.create_publisher(GpsTelemetry, "vehicle/gps", 10)
        self._att_pub = self.create_publisher(AttitudeTelemetry, "vehicle/attitude", 10)
        self._bat_pub = self.create_publisher(BatteryTelemetry, "vehicle/battery", 10)
        self._status_pub = self.create_publisher(IoStatus, "io/status", 10)
        self.create_timer(1.0 / float(self.get_parameter("send_rate_hz").value), self._send)
        self.create_timer(0.005, self._poll_rx)
        self.create_timer(0.2, self._publish_status)

    def _open(self):
        if self._ser is not None or serial is None:
            return self._ser is not None
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0, write_timeout=0)
            self._ser.dtr = False
            self._ser.rts = False
            self.get_logger().info(f"CRSF UART opened: {self._port} @ {self._baud}")
        except Exception as exc:
            self._ser = None
            self.get_logger().warn(f"CRSF UART unavailable: {exc}", throttle_duration_sec=5.0)
        return self._ser is not None

    def _on_command(self, msg: FlightCommand):
        self._command = msg

    def _send(self):
        if not self._open():
            return
        cmd = self._command
        channels = [CRSF_MIN] * 16
        channels[0] = _norm(cmd.roll)
        channels[1] = _norm(cmd.pitch)
        channels[2] = _throttle(cmd.throttle)
        channels[3] = _norm(cmd.yaw)
        channels[4] = CRSF_MAX if cmd.arm else CRSF_MIN
        channels[5] = CRSF_MAX if cmd.manual_mode else CRSF_MIN
        channels[6] = CRSF_MAX if cmd.rtl_request else CRSF_MIN
        try:
            self._ser.write(_packed_channels(channels))
            if self._tx_pub.get_subscription_count():
                out = Int16MultiArray()
                out.data = channels
                self._tx_pub.publish(out)
        except Exception as exc:
            self.get_logger().warn(f"CRSF TX failed: {exc}", throttle_duration_sec=2.0)
            self._ser = None

    def _poll_rx(self):
        if not self._open():
            return
        try:
            waiting = self._ser.in_waiting
            if waiting:
                self._rx.extend(self._ser.read(waiting))
        except Exception as exc:
            self.get_logger().warn(f"CRSF RX failed: {exc}", throttle_duration_sec=2.0)
            self._ser = None
            return
        while len(self._rx) >= 2:
            length = self._rx[1]
            if length < 2 or length > 62:
                del self._rx[0]
                continue
            total = length + 2
            if len(self._rx) < total:
                return
            frame = bytes(self._rx[:total])
            del self._rx[:total]
            if _crc(frame[2:-1]) != frame[-1]:
                continue
            self._last_any = time.monotonic()
            self._handle(frame[2], frame[3:-1])

    @staticmethod
    def _i16(data, offset):
        return int.from_bytes(data[offset:offset + 2], "big", signed=True)

    @staticmethod
    def _u16(data, offset):
        return int.from_bytes(data[offset:offset + 2], "big")

    def _handle(self, typ: int, p: bytes):
        now = self.get_clock().now().to_msg()
        self._telem.header.stamp = now
        vehicle = False
        if typ == 0x08 and len(p) >= 8:  # battery
            voltage = self._u16(p, 0) / 10.0
            current = self._u16(p, 2) / 10.0
            capacity = int.from_bytes(p[4:7], "big")
            remaining = p[7]
            if not self._cells and voltage > 1.0:
                self._cells = math.ceil(voltage / 4.3)
            if self._cells:
                empty = self._cells * float(self.get_parameter("battery.cell_empty_v").value)
                full = self._cells * float(self.get_parameter("battery.cell_full_v").value)
                remaining = int(max(0.0, min(1.0, (voltage - empty) / max(0.01, full - empty))) * 100 + .5)
            self._telem.voltage_v, self._telem.current_a = voltage, current
            self._telem.capacity_used_mah, self._telem.battery_remaining_pct = capacity, remaining
            msg = BatteryTelemetry(); msg.header.stamp = now
            msg.voltage_v, msg.current_a = voltage, current
            msg.capacity_used_mah, msg.battery_remaining_pct, msg.cell_count = capacity, remaining, self._cells
            self._bat_pub.publish(msg); vehicle = True
        elif typ == 0x1E and len(p) >= 6:  # attitude: pitch, roll, yaw in rad*10000
            pitch, roll, yaw = (self._i16(p, i) / 10000.0 * 180.0 / math.pi for i in (0, 2, 4))
            self._telem.roll_deg, self._telem.pitch_deg, self._telem.yaw_deg = roll, pitch, yaw
            msg = AttitudeTelemetry(); msg.header.stamp = now
            msg.roll_deg, msg.pitch_deg, msg.yaw_deg = roll, pitch, yaw
            self._att_pub.publish(msg); vehicle = True
        elif typ == 0x02 and len(p) >= 15:  # GPS
            sats = p[14]
            msg = GpsTelemetry(); msg.header.stamp = now
            msg.latitude_deg = int.from_bytes(p[0:4], "big", signed=True) / 1e7
            msg.longitude_deg = int.from_bytes(p[4:8], "big", signed=True) / 1e7
            msg.ground_speed_mps = self._u16(p, 8) / 36.0
            msg.ground_course_deg = self._u16(p, 10) / 100.0
            msg.altitude_m, msg.satellites = self._u16(p, 12) - 1000.0, sats
            msg.mavlink_fix_type = 1 if sats < 3 else 2 if sats < 4 else 3
            msg.fix_type_inferred, msg.valid_fix = True, sats >= 4
            self._gps_pub.publish(msg); vehicle = True
        elif typ == 0x14 and len(p) >= 10:
            self._telem.uplink_rssi_dbm = int.from_bytes(p[0:1], "big", signed=True)
            self._telem.uplink_lq = p[2]
            self._telem.uplink_snr_db = int.from_bytes(p[3:4], "big", signed=True)
        elif typ == 0x21:
            self._telem.flight_mode = p.rstrip(b"\0").decode(errors="replace")
        if vehicle:
            self._last_vehicle = time.monotonic()
        self._rx_pub.publish(self._telem)

    def _publish_status(self):
        age = time.monotonic() - self._last_any if self._last_any else float("inf")
        vage = time.monotonic() - self._last_vehicle if self._last_vehicle else float("inf")
        def ms(value):
            return 0xFFFFFFFF if not math.isfinite(value) else min(int(value * 1000), 0xFFFFFFFF)
        msg = IoStatus(); msg.header.stamp = self.get_clock().now().to_msg(); msg.source = "crsf"
        msg.transport_connected, msg.vehicle_telemetry_connected = age < self._timeout, vage < self._timeout
        msg.last_rx_age_ms = ms(age)
        msg.last_vehicle_telemetry_age_ms = ms(vage)
        msg.fault_reason = "" if msg.transport_connected else "CRSF telemetry timeout"
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CrsfIoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
