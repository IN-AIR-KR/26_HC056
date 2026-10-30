#!/usr/bin/env python3
"""Pi OS CSI camera adapter for the ARMS container.

Picamera2 remains on the Raspberry Pi OS host.  Each JPEG frame is sent over a
Unix stream socket to one local consumer; when the consumer falls behind, the
connection is dropped rather than building latency.  The ROS publisher lives
inside the container and reconnects automatically.
"""

import argparse
import os
import socket
import struct
import time
from pathlib import Path


MAGIC = b"ARMS"
HEADER = struct.Struct("!4sQQI")  # magic, sequence, wall-clock ns, JPEG bytes


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default="/run/arms-csi/camera.sock")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--jpeg-quality", type=int, default=85)
    return parser.parse_args()


def main():
    args = arguments()
    if args.width < 1 or args.height < 1 or args.fps <= 0:
        raise ValueError("width/height/fps must be positive")
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("jpeg quality must be 1..100")

    import cv2
    from picamera2 import Picamera2

    socket_path = Path(args.socket)
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    socket_path.unlink(missing_ok=True)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    os.chmod(socket_path, 0o660)
    server.listen(1)
    server.setblocking(False)

    camera = Picamera2(camera_num=args.camera)
    config = camera.create_video_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"},
        controls={"FrameRate": args.fps}, buffer_count=4)
    camera.configure(config)
    camera.start()
    client = None
    sequence = 0
    try:
        while True:
            if client is None:
                try:
                    client, _ = server.accept()
                    client.settimeout(0.002)
                    print("CSI consumer connected", flush=True)
                except BlockingIOError:
                    pass

            if client is None:
                time.sleep(0.01)
                continue

            request = camera.capture_request()
            try:
                # Picamera2 RGB888 buffer has BGR byte ordering, matching OpenCV.
                frame = request.make_array("main")
            finally:
                request.release()
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
            if not ok:
                continue
            sequence += 1
            packet = HEADER.pack(MAGIC, sequence, time.time_ns(), len(jpeg)) + jpeg.tobytes()
            try:
                client.sendall(packet)
            except (BrokenPipeError, ConnectionResetError, TimeoutError, socket.timeout):
                client.close()
                client = None
                print("CSI consumer disconnected (slow or closed)", flush=True)
    finally:
        if client is not None:
            client.close()
        camera.stop()
        camera.close()
        server.close()
        socket_path.unlink(missing_ok=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
