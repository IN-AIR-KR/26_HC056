# Raspberry Pi 5 이전 계획

## 목표

Raspberry Pi 5에서는 비행 스택 전체를 Ubuntu 22.04 Docker 컨테이너에서 실행한다.
CSI 카메라와 Hailo PCIe 드라이버처럼 Pi OS 커널·ISP에 결합된 부분만 호스트에 남긴다.

```text
Pi OS host
  arms-csi-capture (Picamera2) ─ Unix domain socket ─┐
                                                      │
Ubuntu 22.04 container                                │
  arms_video_csi_node ─ /arms/camera/image_raw ───────┘
  arms_detection_node (Ultralytics or Hailo)
  arms_control / arms_operator / arms_flight_io / arms_ui / arms_logging
```

호스트 카메라 process는 ROS를 실행하지 않는다. 따라서 Pi OS와 컨테이너의 ROS 배포판을
섞지 않으며, 컨테이너 안의 `arms_video_csi_node`가 기존과 동일한 ROS 영상 토픽을 발행한다.

## 배포 파일 구조

```text
SW/deploy/raspi5/
  Dockerfile                 전체 ROS stack + HailoRT 이미지
  compose.yaml               실제 운용 compose
  .env.example               모델·장치·ROS domain 경로 예시
  entrypoint.sh              ROS workspace source 후 bringup 실행
  config/                    Pi 전용 launch/config override
  host/
    arms_csi_capture.py      Pi OS Picamera2 → Unix socket adapter
    arms-csi-capture.service systemd unit
```

`arms_detection/docker/`는 노트북/Jetson처럼 detection만 별도 컨테이너로 쓸 때의
기존 구성으로 유지한다. `scripts/raspi5_test/`는 독립적인 카메라·Hailo smoke test이며
배포 진입점으로 사용하지 않는다.

## 단계

### Phase 1 — CSI 및 Hailo 검증 (이번 구현 범위)

1. Pi OS 호스트의 `arms_csi_capture.py`가 Picamera2 영상을 Unix domain socket으로
   최신 프레임만 전달한다. 소비자가 느리거나 끊기면 누적하지 않고 프레임을 버린다.
2. 컨테이너의 `arms_video_csi_node`가 socket을 재접속하며
   `/arms/camera/image_raw`와 `/arms/camera/camera_info`를 발행한다.
3. `arms_video/video_csi.launch.py`가 CSI node의 표준 launch entry point가 된다.
4. detection의 HSV 후보·ROI·KF·ROS 토픽은 유지하고, 실제 모델 실행만 backend로 분리한다.
   - `ultralytics`: SITL 및 `.pt`/기존 TensorRT engine
   - `hailo`: Hailo-8L용 NMS 포함 `.hef`
5. backend는 자동 확장자 판별이 아니라 `inference.backend`로 명시한다. 모델 경로도
   `inference.model_path`로 명시해 잘못된 조합을 시작 시 실패시킨다.
6. Hailo backend 검증은 HEF input shape, warmup, Hailo device, NMS output 형식을 확인한 뒤
   저장된 프레임과 실제 CSI 프레임에서 bbox를 발행하는 것으로 완료한다.

완료 기준:

- 호스트 카메라 process가 재시작되어도 container video node가 자동 재접속한다.
- 컨테이너에서 `camera/image_raw`의 FPS, 해상도, timestamp가 확인된다.
- `.pt`와 `.hef` backend가 각각 잘못된 런타임을 import하지 않고, 모델 load/warmup 실패 시
  detection node가 종료된다.
- HEF 결과의 bbox가 동일 프레임의 PT 결과와 육안 비교 가능하다.

### Phase 2 — Pi 전체 Docker 이미지와 bringup

1. `deploy/raspi5/Dockerfile`에 ROS Humble workspace와 HailoRT Python binding을 포함한다.
   HailoRT 버전은 호스트 `hailo-all`/드라이버와 동일하게 pin한다.
2. compose는 `network_mode: host`를 사용하고 `/dev/hailo0`, CSI socket 디렉터리,
   모델 디렉터리만 명시적으로 mount한다. privileged mode는 사용하지 않는다.
3. `arms_rpi5.launch.py`를 추가한다. `video_source:=csi`,
   `inference_backend:=hailo`가 기본이며, launch argument가 detection node의
   `inference.backend`/`inference.model_path`에 전달된다.
4. recorder, UI, operator, control을 포함한 컨테이너 단독 bringup을 검증한다.

### Phase 3 — UART 및 운용 I/O

1. Pi UART console을 끄고 FC/ESP32 UART를 설정한다.
2. `/dev/ttyAMA*` 대신 udev alias 또는 안정적인 `/dev/serial/by-id` 경로를 compose에
   전달한다.
3. FC MSP override, ELRS/CRSF, ESP32 operator input을 프로펠러 없이 검증한다.
4. timeout 시 neutral command 연속 송신 및 조종기 takeover를 확인한다.

### Phase 4 — 통합 비행 검증

1. 벤치에서 카메라→검출→control→FC command 전체 지연 및 rate를 기록한다.
2. 모델/카메라/flight I/O 각각을 중단해 health 및 fail-safe를 검증한다.
3. 무추력, tether, 저고도 순서로 실제 비행 범위를 넓힌다.

## 인터페이스 원칙

- 카메라 종류는 `arms_video` 내부에서만 다룬다. downstream은
  `/arms/camera/image_raw`만 사용한다.
- 모델 종류는 `arms_detection` 내부 backend에서만 다룬다. downstream은
  `/arms/perception/detections`만 사용한다.
- Hailo backend는 임의의 raw tensor HEF를 추측해 해석하지 않는다. NMS 포함 YOLO HEF와
  그 출력 형식을 명시적으로 받아야 한다.
- HEF, HailoRT userspace, 호스트 kernel driver/firmware는 같은 호환 계열로 고정한다.
