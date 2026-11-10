#!/usr/bin/env python3
"""Monitor the normalized ARMS raw-IMU topic without commanding the vehicle.

Run after the normal ARMS launch:
    python3 scripts/imu_test/imu_monitor.py

The output is intentionally source-independent.  It works with PX4 SITL
HIGHRES_IMU and with the SpeedyBee MSP adapter because both publish
sensor_msgs/msg/Imu at /arms/vehicle/imu_raw in body FRD / SI units.
"""

import argparse
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class ImuMonitor(Node):
    def __init__(self, topic: str, window_sec: float, min_hz: float):
        super().__init__("arms_imu_monitor")
        self._topic = topic
        self._window_sec = window_sec
        self._min_hz = min_hz
        self._received = 0
        self._stamps = deque()
        self._last = None
        self._last_receive = 0.0
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, topic, self._on_imu, qos)
        self.create_timer(0.25, self._report)

    def _on_imu(self, msg: Imu):
        now = time.monotonic()
        self._received += 1
        self._stamps.append(now)
        self._last_receive = now
        self._last = msg

    def _rate_hz(self, now: float) -> float:
        while self._stamps and self._stamps[0] < now - self._window_sec:
            self._stamps.popleft()
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0.0 else 0.0

    def healthy(self) -> bool:
        now = time.monotonic()
        if self._last is None or now - self._last_receive > 1.0:
            return False
        values = (
            self._last.angular_velocity.x, self._last.angular_velocity.y,
            self._last.angular_velocity.z, self._last.linear_acceleration.x,
            self._last.linear_acceleration.y, self._last.linear_acceleration.z,
        )
        return all(math.isfinite(value) for value in values) and self._rate_hz(now) >= self._min_hz

    def _report(self):
        now = time.monotonic()
        if self._last is None:
            line = f"{self._topic}: waiting for sensor_msgs/Imu ..."
        else:
            gyro = self._last.angular_velocity
            acc = self._last.linear_acceleration
            g_norm = math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z)
            age_ms = (now - self._last_receive) * 1000.0
            rate = self._rate_hz(now)
            status = "OK" if self.healthy() else "CHECK"
            line = (
                f"[{status}] {self._topic}  {rate:6.1f} Hz  age {age_ms:5.0f} ms\n"
                f"  gyro [rad/s]  x={gyro.x:+8.3f}  y={gyro.y:+8.3f}  z={gyro.z:+8.3f}\n"
                f"  accel[m/s²]  x={acc.x:+8.3f}  y={acc.y:+8.3f}  z={acc.z:+8.3f}"
                f"   |a|={g_norm:6.3f}\n"
                f"  frame={self._last.header.frame_id or '(unset)'}  samples={self._received}"
            )
        if sys.stdout.isatty():
            print("\x1b[H\x1b[J" + line, flush=True)
        else:
            print(line, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/arms/vehicle/imu_raw")
    parser.add_argument("--window", type=float, default=2.0,
                        help="rate calculation window in seconds")
    parser.add_argument("--min-hz", type=float, default=35.0,
                        help="minimum healthy receive rate")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="exit after this many seconds; 0 runs until Ctrl-C")
    args, ros_args = parser.parse_known_args()
    if args.window <= 0.0 or args.min_hz < 0.0 or args.duration < 0.0:
        parser.error("window must be > 0; min-hz and duration must be >= 0")

    rclpy.init(args=ros_args)
    node = ImuMonitor(args.topic, args.window, args.min_hz)
    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration and time.monotonic() - started >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - started
        print(f"\nResult after {elapsed:.1f}s: {'PASS' if node.healthy() else 'CHECK'}")
        print("Move one axis at a time and confirm the matching gyro axis changes. "
              "At rest, acceleration magnitude should be close to 9.81 m/s².")
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.healthy() else 1


if __name__ == "__main__":
    raise SystemExit(main())
