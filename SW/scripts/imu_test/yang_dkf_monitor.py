#!/usr/bin/env python3
"""Observe the Yang delayed-EKF state without sending vehicle commands.

Run the normal stack with control.guidance_mode:=2, acquire a target once,
then run:
    python3 scripts/imu_test/yang_dkf_monitor.py
"""

import argparse
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from arms_msgs.msg import YangDebug


class YangDkfMonitor(Node):
    def __init__(self, topic: str, min_hz: float):
        super().__init__("arms_yang_dkf_monitor")
        self._topic = topic
        self._min_hz = min_hz
        self._stamps = deque()
        self._last = None
        self._last_receive = 0.0
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(YangDebug, topic, self._on_debug, qos)
        self.create_timer(0.25, self._report)

    def _on_debug(self, msg: YangDebug):
        now = time.monotonic()
        self._stamps.append(now)
        self._last_receive = now
        self._last = msg

    def _rate_hz(self, now: float) -> float:
        while self._stamps and self._stamps[0] < now - 2.0:
            self._stamps.popleft()
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0.0 else 0.0

    def healthy(self) -> bool:
        now = time.monotonic()
        if self._last is None or not self._last.valid or now - self._last_receive > 1.0:
            return False
        values = (
            self._last.relative_position_north_m,
            self._last.relative_position_east_m,
            self._last.relative_position_down_m,
            self._last.relative_velocity_north_mps,
            self._last.relative_velocity_east_mps,
            self._last.relative_velocity_down_mps,
            self._last.image_x,
            self._last.image_y,
        )
        return all(math.isfinite(value) for value in values) and self._rate_hz(now) >= self._min_hz

    def _report(self):
        now = time.monotonic()
        if self._last is None:
            line = f"{self._topic}: waiting — set control.guidance_mode=2 and acquire the target once"
        else:
            msg = self._last
            status = "OK" if self.healthy() else "CHECK"
            line = (
                f"[{status}] {self._topic}  {self._rate_hz(now):6.1f} Hz"
                f"  image age {msg.image_age_sec:.3f} s\n"
                f"  p_r NED [m]    {msg.relative_position_north_m:+8.2f}"
                f"  {msg.relative_position_east_m:+8.2f}"
                f"  {msg.relative_position_down_m:+8.2f}\n"
                f"  v_r NED [m/s]  {msg.relative_velocity_north_mps:+8.2f}"
                f"  {msg.relative_velocity_east_mps:+8.2f}"
                f"  {msg.relative_velocity_down_mps:+8.2f}\n"
                f"  image tan      x={msg.image_x:+.4f} y={msg.image_y:+.4f}"
            )
        if sys.stdout.isatty():
            print("\x1b[H\x1b[J" + line, flush=True)
        else:
            print(line, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/arms/control/yang_debug")
    parser.add_argument("--min-hz", type=float, default=35.0)
    parser.add_argument("--duration", type=float, default=0.0)
    args, ros_args = parser.parse_known_args()
    if args.min_hz < 0.0 or args.duration < 0.0:
        parser.error("min-hz and duration must be >= 0")

    rclpy.init(args=ros_args)
    node = YangDkfMonitor(args.topic, args.min_hz)
    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration and time.monotonic() - started >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        passed = node.healthy()
        print(f"\nResult: {'PASS' if passed else 'CHECK'}")
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
