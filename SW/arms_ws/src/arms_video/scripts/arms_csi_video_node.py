#!/usr/bin/env python3
"""Receive host CSI frames over a Unix socket and publish the standard ROS image."""

import socket
import struct
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


MAGIC = b"ARMS"
HEADER = struct.Struct("!4sQQI")
MAX_FRAME_BYTES = 8 * 1024 * 1024


class CsiVideoNode(Node):
    def required(self, name, type_):
        self.declare_parameter(name, type_)
        value = self.get_parameter(name).value
        if value is None:
            raise RuntimeError(f"Required parameter '{name}' is missing")
        return value

    def __init__(self):
        super().__init__("arms_video_csi_node")
        self._socket_path = str(self.required("socket_path", Parameter.Type.STRING))
        self._frame_id = str(self.required("frame_id", Parameter.Type.STRING))
        self._reconnect_ms = int(self.required("reconnect_ms", Parameter.Type.INTEGER))
        if self._reconnect_ms < 1:
            raise RuntimeError("reconnect_ms must be positive")
        self._socket = None
        self._buffer = bytearray()
        self._next_connect_at = 0.0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._image_pub = self.create_publisher(Image, "camera/image_raw", qos)
        self._info_pub = self.create_publisher(CameraInfo, "camera/camera_info", qos)
        self._timer = self.create_timer(0.001, self._poll)
        self.get_logger().info(f"CSI Unix socket input: {self._socket_path}")

    def destroy_node(self):
        self._disconnect()
        return super().destroy_node()

    def _connect(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(0.001)
        try:
            sock.connect(self._socket_path)
        except (FileNotFoundError, ConnectionRefusedError, BlockingIOError, socket.timeout, OSError):
            sock.close()
            self._next_connect_at = time.monotonic() + self._reconnect_ms / 1000.0
            return False
        self._socket = sock
        self._buffer.clear()
        self.get_logger().info("CSI host capture connected")
        return True

    def _disconnect(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self._buffer.clear()
        self._next_connect_at = time.monotonic() + self._reconnect_ms / 1000.0

    def _poll(self):
        if self._socket is None:
            if time.monotonic() >= self._next_connect_at:
                self._connect()
            return
        try:
            data = self._socket.recv(1 << 20)
        except socket.timeout:
            return
        except OSError:
            self._disconnect()
            return
        if not data:
            self.get_logger().warning("CSI host capture disconnected", throttle_duration_sec=2.0)
            self._disconnect()
            return
        self._buffer.extend(data)
        latest = None
        while True:
            if len(self._buffer) < HEADER.size:
                break
            magic, _, stamp_ns, jpeg_size = HEADER.unpack(self._buffer[:HEADER.size])
            if magic != MAGIC or jpeg_size > MAX_FRAME_BYTES:
                self.get_logger().error("Invalid CSI socket frame; reconnecting")
                self._disconnect()
                break
            frame_size = HEADER.size + jpeg_size
            if len(self._buffer) < frame_size:
                return
            jpeg = bytes(self._buffer[HEADER.size:frame_size])
            del self._buffer[:frame_size]
            latest = (jpeg, stamp_ns)
        if latest is not None:
            self._publish(*latest)

    def _publish(self, jpeg, stamp_ns):
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warning("CSI JPEG decode failed", throttle_duration_sec=2.0)
            return
        image = Image()
        image.header.stamp.sec = stamp_ns // 1_000_000_000
        image.header.stamp.nanosec = stamp_ns % 1_000_000_000
        image.header.frame_id = self._frame_id
        image.height, image.width = bgr.shape[:2]
        image.encoding = "bgr8"
        image.step = image.width * 3
        image.data = bgr.tobytes()
        self._image_pub.publish(image)
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = image.width, image.height
        self._info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CsiVideoNode()
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
