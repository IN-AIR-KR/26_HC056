#!/usr/bin/env python3
"""Betaflight MSP raw-IMU adapter.

The CRSF receiver UART and the flight-controller MSP UART are independent
links.  This node owns only the latter and publishes the stack-wide raw IMU
contract: ``vehicle/imu_raw`` as ``sensor_msgs/msg/Imu`` in body FRD/SI.
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

try:
    import serial
except ImportError:
    serial = None


MSP_RAW_IMU = 102
GRAVITY_MPS2 = 9.80665


def _msp_request(command: int) -> bytes:
    """Encode a zero-payload MSP v1 request ($M<)."""
    size = 0
    checksum = size ^ command
    return bytes((ord('$'), ord('M'), ord('<'), size, command, checksum))


class MspImuNode(Node):
    def require_parameter(self, name, parameter_type):
        self.declare_parameter(name, parameter_type)
        value = self.get_parameter(name).value
        if value is None:
            raise RuntimeError(
                f"Required parameter '{name}' is missing for "
                f"{self.get_fully_qualified_name()}")
        return value

    def __init__(self):
        super().__init__("arms_msp_imu_node")
        self.require_parameter("enabled", Parameter.Type.BOOL)
        self.require_parameter("port", Parameter.Type.STRING)
        self.require_parameter("baud", Parameter.Type.INTEGER)
        self.require_parameter("request_rate_hz", Parameter.Type.DOUBLE)
        self.require_parameter("accel_1g_lsb", Parameter.Type.DOUBLE)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._port = str(self.get_parameter("port").value)
        self._baud = int(self.get_parameter("baud").value)
        self._accel_1g_lsb = float(self.get_parameter("accel_1g_lsb").value)
        rate = float(self.get_parameter("request_rate_hz").value)
        if rate <= 0.0 or self._accel_1g_lsb <= 0.0:
            raise RuntimeError("request_rate_hz and accel_1g_lsb must be positive")

        self._serial = None
        self._rx = bytearray()
        self._pub = self.create_publisher(Imu, "vehicle/imu_raw", 50)
        self._timer = self.create_timer(1.0 / rate, self._request_and_poll)
        if self._enabled:
            self.get_logger().info(
                f"MSP raw IMU enabled: {self._port} @ {self._baud}, {rate:.0f} Hz")
        else:
            self.get_logger().info("MSP raw IMU disabled by io_params.yaml")

    def _open(self):
        if self._serial is not None or serial is None:
            return self._serial is not None
        try:
            self._serial = serial.Serial(
                self._port, self._baud, timeout=0, write_timeout=0)
            self._serial.dtr = False
            self._serial.rts = False
            self.get_logger().info(f"MSP UART opened: {self._port} @ {self._baud}")
        except Exception as exc:
            self._serial = None
            self.get_logger().warn(
                f"MSP UART unavailable: {exc}", throttle_duration_sec=5.0)
        return self._serial is not None

    def _request_and_poll(self):
        if not self._enabled or not self._open():
            return
        try:
            self._serial.write(_msp_request(MSP_RAW_IMU))
            waiting = self._serial.in_waiting
            if waiting:
                self._rx.extend(self._serial.read(waiting))
            self._consume_frames()
        except Exception as exc:
            self.get_logger().warn(
                f"MSP IMU UART error: {exc}", throttle_duration_sec=2.0)
            self._serial = None

    def _consume_frames(self):
        """Parse MSP v1 replies ($M>) without accepting malformed frames."""
        while len(self._rx) >= 6:
            start = self._rx.find(b"$M")
            if start < 0:
                self._rx.clear()
                return
            if start:
                del self._rx[:start]
            if len(self._rx) < 6:
                return
            if self._rx[2] != ord('>'):
                del self._rx[0]
                continue
            size = self._rx[3]
            total = size + 6
            if len(self._rx) < total:
                return
            frame = bytes(self._rx[:total])
            del self._rx[:total]
            checksum = 0
            for value in frame[3:-1]:
                checksum ^= value
            if checksum != frame[-1]:
                self.get_logger().warn("MSP IMU checksum mismatch", throttle_duration_sec=2.0)
                continue
            if frame[4] == MSP_RAW_IMU:
                self._publish_raw_imu(frame[5:-1])

    def _publish_raw_imu(self, payload: bytes):
        # Betaflight MSP_RAW_IMU: int16 acc[3], int16 gyro[3], int16 mag[3],
        # all little endian. In current Betaflight gyro values are serialized
        # as gyroRateDps(), while acceleration is accADC (ACC_1G counts/g).
        if len(payload) < 18:
            self.get_logger().warn("MSP_RAW_IMU payload too short", throttle_duration_sec=2.0)
            return
        ax, ay, az, gx_dps, gy_dps, gz_dps, *_ = struct.unpack("<9h", payload[:18])
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "base_frd"
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x = math.radians(gx_dps)
        imu.angular_velocity.y = math.radians(gy_dps)
        imu.angular_velocity.z = math.radians(gz_dps)
        scale = GRAVITY_MPS2 / self._accel_1g_lsb
        imu.linear_acceleration.x = ax * scale
        imu.linear_acceleration.y = ay * scale
        imu.linear_acceleration.z = az * scale
        self._pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = MspImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
