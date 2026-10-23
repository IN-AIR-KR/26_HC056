#!/usr/bin/env python3
"""Hailo identity and Python single-input RGB HEF inference smoke test.

Reports completed synchronous calls, not camera-request FPS or async peak NPU FPS.
Model-specific decoding is intentionally not assumed for arbitrary HEFs.
"""
import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

from frames import FrameSource, add_camera_arguments, validate_camera_arguments


def describe(value):
    if isinstance(value, (list, tuple)):
        return {"length": len(value), "first": describe(value[0]) if len(value) else None}
    return {"shape": list(value.shape), "dtype": str(value.dtype)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_camera_arguments(parser)
    parser.add_argument("--identify", action="store_true")
    parser.add_argument("--hef", type=Path, help="HAILO8L HEF with one NHWC RGB uint8 input")
    parser.add_argument("--resize", choices=["letterbox", "stretch"], default="letterbox")
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--metrics", type=Path, default=Path("outputs/hailo.json"))
    args = parser.parse_args()
    validate_camera_arguments(args)
    if args.warmup < 0:
        parser.error("--warmup must be nonnegative")
    if args.identify or args.hef is None:
        executable = shutil.which("hailortcli")
        if executable is None:
            raise RuntimeError("hailortcli is missing; install the matching HailoRT first")
        subprocess.run([executable, "fw-control", "identify"], check=True, timeout=30)
    if args.hef is None:
        return
    if not args.hef.is_file():
        raise FileNotFoundError(args.hef)
    import cv2
    import numpy as np
    from hailo_platform import (HEF, VDevice, HailoSchedulingAlgorithm,
                               ConfigureParams, HailoStreamInterface,
                               InputVStreamParams, OutputVStreamParams,
                               FormatType, InferVStreams)
    hef = HEF(str(args.hef))
    inputs = hef.get_input_vstream_infos()
    if len(inputs) != 1 or len(inputs[0].shape) != 3 or inputs[0].shape[2] != 3:
        raise ValueError("This smoke test supports only one HWC three-channel image input")
    height, width, _ = inputs[0].shape
    print(json.dumps({"model_input": inputs[0].name, "shape": list(inputs[0].shape),
                      "preprocessing": args.resize + ", RGB uint8 [0,255]"}), flush=True)
    params = VDevice.create_params()
    params.scheduling_algorithm = HailoSchedulingAlgorithm.NONE
    source = None
    samples = []
    completed = interval_count = 0
    window = "Pi5 Hailo input (tensor test; no detection overlay)"
    try:
        with VDevice(params=params) as device:
            configured = device.configure(hef, ConfigureParams.create_from_hef(
                hef, interface=HailoStreamInterface.PCIe))
            if len(configured) != 1:
                raise ValueError("Only single-network-group HEFs are supported")
            group = configured[0]
            in_params = InputVStreamParams.make(group, format_type=FormatType.UINT8)
            out_params = OutputVStreamParams.make(group, format_type=FormatType.FLOAT32)
            source = FrameSource(args.source, args.camera, args.width, args.height, args.fps,
                                 args.bridge, args.timeout)
            if args.preview:
                cv2.namedWindow(window, cv2.WINDOW_NORMAL)
                if args.fullscreen:
                    cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            with InferVStreams(group, in_params, out_params) as pipeline, group.activate():
                # Warm up separately so model setup/warmup do not inflate steady-state latency.
                for _ in range(args.warmup):
                    pipeline.infer({inputs[0].name: np.zeros((1, height, width, 3), dtype=np.uint8)})
                start = last_report = time.monotonic()
                while args.seconds == 0 or time.monotonic() - start < args.seconds:
                    image, meta = source.read()
                    if args.resize == "stretch":
                        resized = cv2.resize(image, (width, height))
                    else:
                        ratio = min(width / image.shape[1], height / image.shape[0])
                        w, h = max(1, round(image.shape[1] * ratio)), max(1, round(image.shape[0] * ratio))
                        resized = np.full((height, width, 3), 114, dtype=np.uint8)
                        left, top = (width - w) // 2, (height - h) // 2
                        resized[top:top+h, left:left+w] = cv2.resize(image, (w, h))
                    tensor = np.ascontiguousarray(cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)[None])
                    before = time.perf_counter()
                    result = pipeline.infer({inputs[0].name: tensor})
                    latency = (time.perf_counter() - before) * 1000
                    samples.append(latency)
                    completed += 1
                    interval_count += 1
                    if completed == 1:
                        print(json.dumps({"outputs": {k: describe(v) for k, v in result.items()}}), flush=True)
                    now = time.monotonic()
                    if now - last_report >= 1:
                        print(json.dumps({"completed": completed,
                            "pipeline_fps": round(interval_count / (now-last_report), 2),
                            "infer_call_ms": round(latency, 2),
                            "capture_return_to_result_ms": round(
                                (time.monotonic_ns()-meta["captured_monotonic_ns"]) / 1e6, 2)}), flush=True)
                        interval_count, last_report = 0, now
                    if args.preview:
                        cv2.imshow(window, image)
                        if cv2.waitKey(1) & 0xff in (27, ord("q")):
                            break
                elapsed = time.monotonic() - start
    finally:
        if source is not None:
            source.close()
        if args.preview:
            cv2.destroyAllWindows()
    if not samples:
        raise RuntimeError("No inference completed")
    summary = {"hef": str(args.hef), "completed": completed,
               "elapsed_s": round(elapsed, 3),
               "pipeline_fps": round(completed / elapsed, 2),
               "infer_call_ms_mean": round(float(np.mean(samples)), 2),
               "infer_call_ms_p95": round(float(np.percentile(samples, 95)), 2)}
    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    args.metrics.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
