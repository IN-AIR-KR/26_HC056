"""Standalone camera sources; no ROS or flight-controller integration.

The bridge is an atomically replaced latest-frame file, not a growing queue.
JPEG transport is deliberately simple and is not a zero-copy performance test.
"""
import json
import os
import struct
import time
from pathlib import Path


class FrameSource:
    def __init__(self, source, camera=0, width=1280, height=720, fps=30,
                 bridge="shared/latest.frame", timeout=10.0):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.source, self.width, self.height, self.fps = source, width, height, fps
        self.bridge, self.timeout = Path(bridge), timeout
        self.last_sequence = None
        self.sequence = 0
        self.next_frame = time.monotonic()
        self.camera = None
        if source == "picamera2":
            try:
                from picamera2 import Picamera2
            except ImportError as exc:
                raise RuntimeError("Picamera2 is host-only here. Use system python3 on Pi OS, "
                                   "or --source bridge inside Docker.") from exc
            self.camera = Picamera2(camera_num=camera)
            try:
                # Picamera2 RGB888 memory is BGR byte order, suitable for OpenCV.
                config = self.camera.create_video_configuration(
                    main={"size": (width, height), "format": "RGB888"},
                    controls={"FrameRate": fps}, buffer_count=4)
                self.camera.configure(config)
                self.camera.start()
            except BaseException:
                self.camera.close()
                raise

    def read(self):
        if self.source == "picamera2":
            request = self.camera.capture_request()
            try:
                image = request.make_array("main").copy()
                sensor_ns = request.get_metadata().get("SensorTimestamp")
            finally:
                request.release()
            return image, {"sequence": time.monotonic_ns(),
                           "captured_monotonic_ns": time.monotonic_ns(),
                           "sensor_timestamp_ns": sensor_ns}
        if self.source == "synthetic":
            time.sleep(max(0.0, self.next_frame - time.monotonic()))
            self.next_frame = time.monotonic() + 1.0 / self.fps
            self.sequence += 1
            image = self.np.zeros((self.height, self.width, 3), dtype=self.np.uint8)
            x = (self.sequence * 8) % self.width
            self.cv2.rectangle(image, (x, 20),
                               (min(x + 80, self.width - 1), min(100, self.height - 1)),
                               (0, 255, 0), -1)
            return image, {"sequence": self.sequence,
                           "captured_monotonic_ns": time.monotonic_ns(),
                           "sensor_timestamp_ns": None}
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            try:
                data = self.bridge.read_bytes()
            except FileNotFoundError:
                time.sleep(0.005)
                continue
            if len(data) < 4:
                raise RuntimeError("Invalid bridge file: missing header")
            size = struct.unpack("!I", data[:4])[0]
            if size > 65536 or len(data) <= 4 + size:
                raise RuntimeError("Invalid bridge file: header/payload length")
            meta = json.loads(data[4:4 + size])
            age = (time.monotonic_ns() - meta["captured_monotonic_ns"]) / 1e9
            if meta["sequence"] == self.last_sequence or not 0 <= age <= self.timeout:
                time.sleep(0.005)
                continue
            image = self.cv2.imdecode(self.np.frombuffer(data[4 + size:], self.np.uint8),
                                      self.cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError("Invalid bridge JPEG")
            self.last_sequence = meta["sequence"]
            return image, meta
        raise TimeoutError(f"No fresh frame for {self.timeout}s at {self.bridge}. "
                           "Start the host camera publisher first.")

    def close(self):
        if self.camera is not None:
            self.camera.stop()
            self.camera.close()


def publish_frame(path, image, metadata, quality=85):
    import cv2
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    header = json.dumps(metadata).encode()
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        temporary.write_bytes(struct.pack("!I", len(header)) + header + encoded.tobytes())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def add_camera_arguments(parser):
    parser.add_argument("--source", choices=["picamera2", "bridge", "synthetic"],
                        default="picamera2")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30)
    parser.add_argument("--bridge", default="shared/latest.frame")
    parser.add_argument("--timeout", type=float, default=10)
    parser.add_argument("--seconds", type=float, default=30, help="0 = until Ctrl+C")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--fullscreen", action="store_true")


def validate_camera_arguments(args):
    if min(args.width, args.height, args.fps, args.timeout) <= 0 or args.seconds < 0:
        raise ValueError("width, height, fps, timeout must be positive; seconds >= 0")
    if args.camera < 0:
        raise ValueError("camera index must be nonnegative")
