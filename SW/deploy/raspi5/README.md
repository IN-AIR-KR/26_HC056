# Raspberry Pi 5 Phase 1 검증

이 단계에서는 **CSI 카메라 → ROS 영상 토픽** 경로와 **Hailo 추론 백엔드**만
검증합니다. 비행 스택, GPIO, UART, 조종기 노드는 아직 실행하지 않습니다.

아래 명령은 저장소가 `~/ARMS/SW`에 있다고 가정합니다. 다른 경로에 두었다면
그 부분만 실제 경로로 바꿉니다.

## 0. 호스트 사전 확인

컨테이너를 빌드하기 전에 Pi OS 호스트에서 카메라와 Hailo 런타임을 확인합니다.
Hailo 장치는 `/dev/hailo0`로 보여야 합니다.

```bash
cd ~/ARMS/SW
uname -m
hailortcli --version
hailortcli fw-control identify
test -e /dev/hailo0 && echo "Hailo device ready"

sudo apt update
sudo apt install python3-picamera2 python3-opencv python3-numpy
```

NMS 출력이 포함된 **Hailo-8L용 HEF**를 사용해야 합니다. raw feature map만
출력하는 HEF는 이 Phase 1 노드에서 지원하지 않습니다. 모델 출력 형식이 확실하지
않다면 먼저 기존 `scripts/raspi5_test` 도구로 확인합니다.

## 1. 호스트 디렉터리와 모델 준비

`/run`은 재부팅할 때마다 비워지므로 소켓 디렉터리는 부팅 후 한 번씩 다시
만들어야 합니다. 선택한 HEF는 영구 경로인 모델 디렉터리에 복사합니다.

```bash
cd ~/ARMS/SW
sudo install -d -m 0770 -o "$USER" -g "$USER" /run/arms-csi
sudo install -d -m 0755 -o "$USER" -g "$USER" /opt/arms/models
cp /path/to/balloon-nms.hef /opt/arms/models/balloon.hef

cd deploy/raspi5
cp .env.example .env
```

Hailo 장치·소켓 디렉터리·모델 경로가 다르지 않다면 `.env`의 기본값을 그대로
사용합니다. `HEF_MODEL`은 **컨테이너 내부 경로**이므로 보통
`/models/balloon.hef`로 유지합니다.

## 2. Pi OS 호스트에서 CSI 캡처 시작

터미널 A에서 호스트 어댑터를 실행합니다. 동일 카메라를 사용하는 다른 Picamera2
또는 rpicam 프로그램은 동시에 실행하면 안 됩니다.

```bash
cd ~/ARMS/SW
python3 deploy/raspi5/host/arms_csi_capture.py \
  --socket /run/arms-csi/camera.sock --width 1280 --height 720 --fps 30
```

## 3. 컨테이너 영상 발행부터 검증

터미널 B에서 `video` 서비스만 빌드·실행합니다. 이렇게 하면 CSI/소켓 문제와
Hailo/모델 문제를 분리해서 확인할 수 있습니다.

```bash
cd ~/ARMS/SW/deploy/raspi5
docker compose --env-file .env up --build -d video
docker compose exec video bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic hz /arms/camera/image_raw'
```

두 번째 명령은 요청한 카메라 FPS에 가까운 값을 보여야 합니다. 확인 후 `Ctrl+C`로
종료합니다.

## 4. Hailo 검출 시작 및 검증

```bash
cd ~/ARMS/SW/deploy/raspi5
docker compose --env-file .env up -d detection
docker compose logs -f detection
```

다른 터미널에서 검출 토픽이 발행되는지 확인합니다.

```bash
cd ~/ARMS/SW/deploy/raspi5
docker compose exec detection bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic echo /arms/perception/detections'
```

HEF는 HWC 형식의 3채널 입력 하나와 NMS 출력 하나를 가져야 합니다. 기본 출력
형식은 `classwise_yxyx_score`이며 배열 구조는
`[batch, class, detection, y1, x1, y2, x2, score]`입니다. HEF의 출력 이름이나
배열 형식이 다르면 `arms_ws/src/arms_detection/config/detection.yaml`의
`inference.hailo.output_name` 또는 `inference.hailo.nms_format`을 맞춘 뒤
`detection` 서비스를 재시작합니다.

## 종료

Phase 1 컨테이너를 종료하려면 다음을 실행합니다.

```bash
cd ~/ARMS/SW/deploy/raspi5
docker compose down
```
