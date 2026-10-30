#!/usr/bin/env python3
"""Picamera2 capture, latest-frame bridge publishing, and frame reception test."""
import argparse
import json
import sys
import time
from pathlib import Path

from frames import FrameSource, add_camera_arguments, publish_frame, validate_camera_arguments, preview_image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_camera_arguments(parser)
    parser.add_argument("--list", action="store_true", help="List host CSI cameras")
    parser.add_argument("--publish", metavar="PATH", help="Atomically publish latest JPEG frame")
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--snapshot", help="Save first received frame, e.g. outputs/camera.jpg")
    args = parser.parse_args()
    if args.list:
        from picamera2 import Picamera2
        print(json.dumps(Picamera2.global_camera_info(), indent=2, default=str))
        return
    validate_camera_arguments(args)
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be 1..100")
    if args.source == "bridge" and args.publish:
        parser.error("bridge consumers cannot republish to avoid feedback loops")
    import cv2
    source = FrameSource(args.source, args.camera, args.width, args.height, args.fps,
                         args.bridge, args.timeout)
    start = last_report = time.monotonic()
    count = interval_count = 0
    window = "Pi5 camera test"
    try:
        if args.preview:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, args.preview_width, args.preview_height)
            if args.fullscreen:
                cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        while args.seconds == 0 or time.monotonic() - start < args.seconds:
            image, meta = source.read()
            count += 1
            interval_count += 1
            if args.publish:
                publish_frame(args.publish, image, meta, args.jpeg_quality)
            if args.snapshot and count == 1:
                Path(args.snapshot).parent.mkdir(parents=True, exist_ok=True)
                if not cv2.imwrite(args.snapshot, image):
                    raise RuntimeError(f"Cannot save {args.snapshot}")
            now = time.monotonic()
            if now - last_report >= 1:
                age = (time.monotonic_ns() - meta["captured_monotonic_ns"]) / 1e6
                print(json.dumps({"frames": count, "received_fps": round(interval_count / (now-last_report), 2),
                                  "frame_age_ms": round(age, 2), "shape": list(image.shape)}), flush=True)
                interval_count, last_report = 0, now
            if args.preview:
                cv2.imshow(window, preview_image(image, args.preview_width, args.preview_height))
                if cv2.waitKey(1) & 0xff in (27, ord("q")):
                    break
    finally:
        source.close()
        if args.preview:
            cv2.destroyAllWindows()
    elapsed = time.monotonic() - start
    print(json.dumps({"frames": count, "elapsed_s": round(elapsed, 3),
                      "average_received_fps": round(count / max(elapsed, 1e-9), 2)}))
    if count == 0:
        raise RuntimeError("No frames captured")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
