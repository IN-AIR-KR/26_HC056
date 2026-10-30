"""Model-runtime adapters used by :mod:`arms_detection_node`.

The detection pipeline deliberately owns HSV proposals, ROI handling, target
filtering and ROS messages.  A backend only turns BGR images into normalized
bounding boxes, so changing a runtime cannot alter the rest of the pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class InferenceBox:
    """A bounding box normalized to the source image passed to ``infer``."""

    x_center: float
    y_center: float
    width: float
    height: float
    confidence: float
    class_id: int
    class_name: str


class UltralyticsBackend:
    """Ultralytics runtime for SITL and existing PT/TensorRT deployments."""

    name = "ultralytics"

    def __init__(self, model_path: str, confidence: float, iou: float, warmup_size: int):
        from ultralytics import YOLO

        self._confidence = confidence
        self._iou = iou
        self._model = YOLO(model_path, task="detect")
        self._batch_enabled = not model_path.lower().endswith(".engine")
        dummy = np.zeros((warmup_size, warmup_size, 3), np.uint8)
        self._model.predict(dummy, imgsz=warmup_size, verbose=False)

    def infer(self, images: Sequence[np.ndarray], imgsz: int) -> list[InferenceBox | None]:
        if not images:
            return []
        if len(images) > 1 and not self._batch_enabled:
            return [self.infer([image], imgsz)[0] for image in images]

        sources = [np.ascontiguousarray(image) for image in images]
        results = self._model.predict(
            sources, conf=self._confidence, iou=self._iou, imgsz=imgsz, verbose=False)
        output: list[InferenceBox | None] = []
        for image, result in zip(images, results):
            h, w = image.shape[:2]
            if len(result.boxes) == 0:
                output.append(None)
                continue
            best = max(result.boxes, key=lambda box: float(box.conf[0]))
            x1, y1, x2, y2 = best.xyxy[0].tolist()
            class_id = int(best.cls[0])
            output.append(InferenceBox(
                x_center=float((x1 + x2) / (2.0 * w)),
                y_center=float((y1 + y2) / (2.0 * h)),
                width=float((x2 - x1) / w),
                height=float((y2 - y1) / h),
                confidence=float(best.conf[0]),
                class_id=class_id,
                class_name=str(self._model.names[class_id]),
            ))
        output.extend([None] * (len(images) - len(output)))
        return output

    def close(self) -> None:
        pass


class HailoBackend:
    """Synchronous HailoRT backend for a NMS-containing YOLO HEF.

    Raw feature-map HEFs are intentionally unsupported: decoding those tensors
    is model-architecture dependent and silently guessing would be unsafe.
    Configure the exact NMS tensor name/layout after inspecting it with the
    Pi smoke test.
    """

    name = "hailo"

    def __init__(
        self,
        model_path: str,
        confidence: float,
        class_names: Sequence[str],
        input_color: str,
        output_name: str,
        nms_format: str,
    ):
        try:
            from hailo_platform import (
                ConfigureParams, FormatType, HailoSchedulingAlgorithm,
                HailoStreamInterface, HEF, InferVStreams,
                InputVStreamParams, OutputVStreamParams, VDevice,
            )
        except ImportError as exc:
            raise RuntimeError(
                "Hailo backend requires the matching HailoRT Python binding "
                "(hailo_platform) inside this container") from exc

        if input_color.lower() not in {"rgb", "bgr"}:
            raise ValueError("inference.hailo.input_color must be 'rgb' or 'bgr'")
        if nms_format not in {
            "xyxy_score_class", "yxyx_score_class", "classwise_yxyx_score",
        }:
            raise ValueError("unsupported inference.hailo.nms_format")

        self._np = np
        self._confidence = confidence
        self._class_names = list(class_names)
        self._input_color = input_color.lower()
        self._output_name = output_name
        self._nms_format = nms_format

        hef = HEF(model_path)
        inputs = hef.get_input_vstream_infos()
        if len(inputs) != 1 or len(inputs[0].shape) != 3 or inputs[0].shape[2] != 3:
            raise RuntimeError("Hailo HEF must have exactly one HWC three-channel input")
        self._input_name = inputs[0].name
        self._input_h, self._input_w, _ = inputs[0].shape

        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.NONE
        self._device = VDevice(params=params)
        configured = self._device.configure(
            hef, ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe))
        if len(configured) != 1:
            self.close()
            raise RuntimeError("Hailo HEF must contain exactly one network group")
        self._group = configured[0]
        in_params = InputVStreamParams.make(self._group, format_type=FormatType.UINT8)
        out_params = OutputVStreamParams.make(self._group, format_type=FormatType.FLOAT32)
        self._pipeline = InferVStreams(self._group, in_params, out_params)
        self._pipeline.__enter__()
        self._activation = self._group.activate()
        self._activation.__enter__()

        # Model activation and tensor transport failures must surface during
        # node start, rather than pretending the health check is online.
        blank = np.zeros((1, self._input_h, self._input_w, 3), dtype=np.uint8)
        self._pipeline.infer({self._input_name: blank})

    def close(self) -> None:
        for attr in ("_activation", "_pipeline", "_device"):
            resource = getattr(self, attr, None)
            if resource is None:
                continue
            try:
                resource.__exit__(None, None, None)
            except AttributeError:
                try:
                    resource.release()
                except AttributeError:
                    pass
            setattr(self, attr, None)

    def infer(self, images: Sequence[np.ndarray], imgsz: int) -> list[InferenceBox | None]:
        # A fixed-shape HEF accepts one image per invocation.  ``imgsz`` is
        # intentionally ignored; the HEF's declared input shape is authoritative.
        return [self._infer_one(image) for image in images]

    def _infer_one(self, image: np.ndarray) -> InferenceBox | None:
        tensor, scale, pad_x, pad_y = self._letterbox(image)
        result = self._pipeline.infer({self._input_name: tensor[None]})
        rows = self._nms_rows(result)
        best = None
        for x1, y1, x2, y2, score, class_id in rows:
            if score < self._confidence:
                continue
            candidate = self._source_box(
                x1, y1, x2, y2, score, class_id,
                image.shape[1], image.shape[0], scale, pad_x, pad_y)
            if candidate is not None and (best is None or candidate.confidence > best.confidence):
                best = candidate
        return best

    def _letterbox(self, image: np.ndarray):
        h, w = image.shape[:2]
        scale = min(self._input_w / w, self._input_h / h)
        resized_w, resized_h = max(1, round(w * scale)), max(1, round(h * scale))
        pad_x, pad_y = (self._input_w - resized_w) // 2, (self._input_h - resized_h) // 2
        tensor = np.full((self._input_h, self._input_w, 3), 114, dtype=np.uint8)
        tensor[pad_y:pad_y + resized_h, pad_x:pad_x + resized_w] = cv2.resize(
            image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
        if self._input_color == "rgb":
            tensor = cv2.cvtColor(tensor, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(tensor), scale, pad_x, pad_y

    def _nms_rows(self, outputs) -> list[tuple[float, float, float, float, float, int]]:
        if self._output_name:
            if self._output_name not in outputs:
                raise RuntimeError(
                    f"Hailo NMS output '{self._output_name}' not found; "
                    f"available: {list(outputs)}")
            value = outputs[self._output_name]
        elif len(outputs) == 1:
            value = next(iter(outputs.values()))
        else:
            raise RuntimeError(
                "Hailo HEF has multiple outputs; set inference.hailo.output_name")
        data = np.asarray(value)
        # Hailo's standard YOLO NMS shape is [batch, classes, detections, 5].
        # Only batch may be removed here: a one-class model still needs its
        # class axis to remain [1, detections, 5].
        if self._nms_format == "classwise_yxyx_score":
            if data.ndim == 4 and data.shape[0] == 1:
                data = data[0]
        else:
            while data.ndim > 0 and data.shape[0] == 1:
                data = data[0]

        rows: list[tuple[float, float, float, float, float, int]] = []
        if self._nms_format in {"xyxy_score_class", "yxyx_score_class"}:
            if data.ndim != 2 or data.shape[1] < 6:
                raise RuntimeError(
                    "Hailo NMS tensor must be [detections, >=6] for the selected format")
            for row in data:
                a, b, c, d, score, class_id = map(float, row[:6])
                if self._nms_format == "xyxy_score_class":
                    rows.append((a, b, c, d, score, int(class_id)))
                else:
                    rows.append((b, a, d, c, score, int(class_id)))
            return rows

        if data.ndim != 3 or data.shape[2] < 5:
            raise RuntimeError(
                "Hailo classwise NMS tensor must be [classes, detections, >=5]")
        for class_id, class_rows in enumerate(data):
            for row in class_rows:
                y1, x1, y2, x2, score = map(float, row[:5])
                rows.append((x1, y1, x2, y2, score, class_id))
        return rows

    def _source_box(self, x1, y1, x2, y2, score, class_id, source_w, source_h,
                    scale, pad_x, pad_y) -> InferenceBox | None:
        # Hailo NMS variants commonly return normalized coordinates; preserve
        # pixel-coordinate HEFs as well.
        if max(abs(x1), abs(x2), abs(y1), abs(y2)) <= 1.5:
            x1, x2 = x1 * self._input_w, x2 * self._input_w
            y1, y2 = y1 * self._input_h, y2 * self._input_h
        x1, x2 = (x1 - pad_x) / scale, (x2 - pad_x) / scale
        y1, y2 = (y1 - pad_y) / scale, (y2 - pad_y) / scale
        x1, x2 = np.clip([x1, x2], 0, source_w)
        y1, y2 = np.clip([y1, y2], 0, source_h)
        if x2 <= x1 or y2 <= y1:
            return None
        class_name = (self._class_names[class_id]
                      if 0 <= class_id < len(self._class_names)
                      else f"class_{class_id}")
        return InferenceBox(
            x_center=float((x1 + x2) / (2.0 * source_w)),
            y_center=float((y1 + y2) / (2.0 * source_h)),
            width=float((x2 - x1) / source_w),
            height=float((y2 - y1) / source_h),
            confidence=float(score), class_id=class_id, class_name=class_name,
        )
