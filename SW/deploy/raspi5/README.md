# Raspberry Pi 5 phase-1 validation

This compose file runs only the CSI ROS publisher and Hailo detection backend.
It is not the final flight-stack compose yet.

On the Pi OS host, install Picamera2/OpenCV. Create the runtime socket directory
once per boot (Phase 2 replaces this with a systemd `RuntimeDirectory`), then run:

```bash
sudo install -d -m 0770 -o "$USER" -g "$USER" /run/arms-csi
python3 deploy/raspi5/host/arms_csi_capture.py \
  --socket /run/arms-csi/camera.sock --width 1280 --height 720 --fps 30
```

Copy `.env.example` to `.env`, place the Hailo-8L NMS HEF at the configured
model path, then build and run in another terminal:

```bash
docker compose --env-file .env up --build
```

The HEF must contain one HWC 3-channel input and an NMS output. Before running
the detection service, inspect the output tensor with `scripts/raspi5_test` and
set `inference.hailo.output_name` / `nms_format` in
`arms_ws/src/arms_detection/config/detection.yaml` if its layout differs from
`classwise_yxyx_score`.

Validation commands inside the `video` container or from another ROS Humble
container on the same host network:

```bash
ros2 topic hz /arms/camera/image_raw
ros2 topic echo /arms/perception/detections
```
