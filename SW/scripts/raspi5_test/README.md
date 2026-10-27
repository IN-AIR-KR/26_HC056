# Raspberry Pi 5 카메라 / Hailo-8L 단독 테스트

ROS, 비행 제어, UART, 기존 `arms_ws`와 연결하지 않는 독립 하드웨어 테스트다.
호스트는 현재 Raspberry Pi OS Trixie ARM64, Docker 이미지는 **Ubuntu 22.04 ARM64**다.
HailoRT 기본 버전은 사용자 장치에서 확인한 **4.23.0**이다.

## 구조와 제한

```text
호스트 Picamera2 → Python 캡처/미리보기
                     │ JPEG + 메타데이터, 최신 파일을 원자적으로 교체
                     ▼
              shared/latest.frame
                     │ read-only bind mount
                     ▼
Ubuntu 22.04 Docker → Python 영상 수신 / HailoRT Python 추론 → JSON 측정 결과
```

- Pi 5 CSI/PiSP 카메라는 단순 USB V4L2 장치가 아니다. 최신 호스트의 libcamera,
  IPA 파일, Python 3.13 모듈을 Jammy/Python 3.10에 마운트하지 않는다.
- CSI 캡처는 호스트 Picamera2가 담당한다. Docker 단독으로 CSI를 여는 구성은 아니다.
  파일 브리지는 JPEG 인코딩/디코딩과 복사 비용이 있으므로 최종 저지연 파이프라인의
  성능과 같다고 해석하면 안 된다. 최신 프레임 하나만 보관하며 소비자가 느리면 건너뛴다.
- 커널 Hailo PCIe 드라이버/펌웨어는 **호스트에만** 설치한다. 이미 작동하는
  `hailo-all` 환경을 유지한다. Docker는 대응 HailoRT/CLI/Python 바인딩을 소스 빌드한다.
- `hailo_test.py`는 한 개의 HWC RGB uint8 입력을 가진 HEF를 위한 **텐서 추론 시험**이다.
  FLOAT 입력, 다중 입력/네트워크 그룹, 모델별 특수 전처리는 지원하지 않는다.
  출력 텐서 요약과 완료 FPS/호출 지연을 제공하며, YOLO 박스·pose·segmentation
  후처리/정확도를 검증하거나 표시하지 않는다. 실행 성공이 모델 정확도를 뜻하지 않는다.
- 실제 프레임 전달·NPU 실행은 Pi에서 검증해야 한다. macOS/x86 Docker 에뮬레이션으로
  PCIe 하드웨어 동작을 시험할 수 없다.

## 1. 호스트 확인

Pi에서 이 폴더로 이동한다. 설치 경로는 실제 저장소 위치에 맞춘다.

```bash
cd ~/ARMS/SW/scripts/raspi5_test
uname -m
hailortcli --version
hailortcli fw-control identify
modinfo hailo_pci | head
```

`aarch64`, `HAILO8L`과 버전을 확인한다. **펌웨어 숫자만 보지 말고 호스트 드라이버와
사용자 라이브러리도 확인**한다. 4.23.0과 다르면 무작정 드라이버를 내리지 말고
Dockerfile의 소스 ref/commit과 실제 스택의 호환성을 먼저 맞춘다.

호스트 카메라용 패키지가 없으면 Raspberry Pi OS 저장소에서 설치한다.

```bash
sudo apt update
sudo apt install python3-picamera2 python3-opencv python3-numpy
```

호스트에서는 시스템 `python3`를 사용한다. 별도 venv는 시스템 Picamera2가 보이도록
`--system-site-packages`가 필요할 수 있다. 호스트에 pip OpenCV를 덮어쓰지 않는다.

## 2. Python 카메라 테스트 (호스트)

```bash
python3 camera_test.py --list
python3 camera_test.py --camera 0 --preview --seconds 30 --snapshot outputs/camera0.jpg
python3 camera_test.py --camera 1 --preview --seconds 30
```

목록으로 번호를 확인한다. 다른 rpicam/Picamera2 앱이 같은 카메라를 사용하면 종료한다.
SSH/headless에서는 `--preview`를 생략한다. `--fullscreen`은 `--preview`와 함께 사용한다.
미리보기 종료: `q`, Escape 또는 Ctrl+C. `--seconds 0`은 무제한 실행이다.

IMX477 4K 요청 예:

```bash
python3 camera_test.py --camera 1 --width 3840 --height 2160 --fps 15 --preview
```

실제 프레임 shape/FPS를 출력으로 확인한다. IMX219는 네이티브 4K 센서가 아니다.
카메라 타임아웃이 있다면 먼저 호스트 전원·케이블 문제를 해결한다.

## 3. Ubuntu 22.04 Docker 빌드

Docker Engine 및 Compose 플러그인이 설치된 Pi에서:

```bash
docker compose version
cp .env.example .env
docker compose build camera
```

`build.network: host`를 설정했고 `additional privileges requested`가 나오면
Compose 파일을 그대로 읽는 Buildx Bake로 빌드 권한을 전달한다:

```bash
docker buildx bake -f compose.yaml --allow=network.host --load camera
```

두 서비스는 같은 이미지를 쓴다. 처음에는 HailoRT와 Python 바인딩을 컴파일하므로
시간/메모리/다운로드 공간이 필요하다. 기본 병렬 작업 수는 2, 부족하면 `.env`에서
`BUILD_JOBS=1`로 낮춘다. 이미지 안에 호스트 드라이버를 설치하지 않는다.
HailoRT와 Python 의존성(`netifaces` 네이티브 확장 포함)은 빌드 단계에서 wheel로
준비한다. 실행 이미지에서는 `--no-index --only-binary=:all:`로 로컬 wheel만
설치하고, `pip check`, Python import, CLI 버전을 확인한다.

## 4. Docker에서 카메라 프레임 수신

호스트 터미널 A에서 캡처를 유지한다:

```bash
python3 camera_test.py --camera 0 --seconds 0 --preview --publish shared/latest.frame
```

터미널 B에서:

```bash
docker compose run --rm camera
```

30초간 수신 FPS/프레임 크기를 출력하고 `outputs/camera.jpg`를 저장한다.
기본 `shared/`는 저장장치에 쓰므로 짧은 시험에만 사용한다. 장시간/4K 시험은
`.env`의 `CAMERA_SHARED_DIR=/dev/shm/arms-raspi5-test`로 바꿔 RAM을 사용한다.
호스트에서 `mkdir -p /dev/shm/arms-raspi5-test`를 실행하고 publisher의 출력도
`--publish /dev/shm/arms-raspi5-test/latest.frame`으로 **함께 변경**한다.
RAM 경로는 재부팅 후 다시 만들어야 한다.
파일은 컨테이너를 지워도 남는다. 기본 Docker 시험은 headless다.
Docker 창을 띄우기 위해 X11/Wayland 인증을 우회하거나 `xhost +`를 실행하지 않는다.
현재 단계에서는 호스트 미리보기와 저장 이미지를 사용한다.

`frame_age_ms`는 호스트 capture 반환 시각부터의 시간이며 센서 노출 시작부터의
엄밀한 지연은 아니다. `sensor_timestamp_ns`는 별도 메타데이터로 전달한다.
컨테이너와 호스트는 같은 Linux 커널의 monotonic clock을 공유하는 전제다.

## 5. Hailo 장치 확인 (Python 진입점)

호스트:

```bash
python3 hailo_test.py --identify
```

Docker:

```bash
docker compose run --rm hailo
```

장치 확인 모드는 CLI를 호출한다. 아래 실제 추론 모드는 `hailo_platform` Python API를
사용한다. `hailortcli --version`은 성공해도 하드웨어/드라이버 호환은 이 단계에서
실패할 수 있다. 동시에 다른 Hailo 앱을 실행하지 않는다.

## 6. HEF 선택 및 Python 추론

모델은 자동 다운로드하지 않는다. **Hailo-8L용 HEF**를 `models/`에 복사한다.
기존 예제의 파일 위치는 다음에서 확인할 수 있다:

```bash
cat /usr/share/rpi-camera-assets/hailo_yolov8_inference.json
find /usr/share -type f -name '*.hef' 2>/dev/null
```

JSON에는 Hailo-8과 8L용 경로가 함께 있을 수 있다. 실제 8L 경로를 선택하고,
그 파일을 `models/model.hef`로 복사한다. 확장자 변경은 모델 변환이 아니다.

터미널 A의 카메라 publisher가 실행 중인 상태에서:

```bash
docker compose run --rm hailo python3 hailo_test.py \
  --hef /models/model.hef --source bridge --bridge /shared/latest.frame \
  --seconds 30 --metrics /outputs/hailo.json
```

호스트에 `hailo_platform`이 설치돼 있다면 브리지 없이도 실행 가능하다
(동일 카메라를 사용 중인 publisher를 먼저 종료):

```bash
python3 hailo_test.py --hef models/model.hef --camera 0 --preview --seconds 30
```

입력은 RGB uint8 [0,255], 기본 letterbox이며 배경색 114를 사용한다.
모델 요구가 다르면 이 테스트를 정확도 평가로 사용하지 않는다.
`--resize stretch`로 단순 리사이즈를 선택할 수 있다. 크기가 4K라도 **NPU 입력은 HEF의
입력 크기**로 축소된다. 미리보기에는 입력 원본만 표시한다.

측정값:

| 값 | 의미 |
|---|---|
| `completed` | 실제 동기 추론 호출 완료 횟수 |
| `pipeline_fps` | 캡처/브리지 수신·전처리·추론·선택적 미리보기를 포함한 완료율 |
| `infer_call_ms` | HailoRT 동기 호출 시간. 순수 NPU 실행 시간은 아님 |
| `capture_return_to_result_ms` | 호스트 캡처 반환부터 추론 완료까지. JPEG 비용 포함 |
| `infer_call_ms_p95` | 워밍업을 제외한 추론 호출 지연 95백분위 |

카메라 요청 FPS와 추론 완료 FPS를 혼동하지 않는다. 최신 파일 방식은 느린 소비자의
프레임을 버리므로 무제한 적체되지 않는다. 매 프레임 처리/손실률 검증용 전달 방식은 아니다.

YOLO 박스나 pose/segmentation 시각화는 현재 정상 동작하는 **호스트 공식 데모**로
별도 확인한다:

```bash
rpicam-hello --camera 0 -t 0 \
  --post-process-file /usr/share/rpi-camera-assets/hailo_yolov8_inference.json
```

## 7. 카메라 없이 전달 경로 확인

호스트에서 `python3-opencv`, `python3-numpy`만 있으면:

```bash
python3 camera_test.py --source synthetic --seconds 0 --publish shared/latest.frame
```

다른 터미널에서 `docker compose run --rm camera`로 수신을 확인한다.
이것은 CSI/NPU 검증이 아니다. HEF가 있다면 Hailo 스모크 테스트의 입력원으로도 쓸 수 있다.

## 문제 해결

- `No fresh frame`: 호스트 publisher, `shared` 경로, 파일 권한 확인. 중단된 publisher의
  오래된 파일은 재사용하지 않고 타임아웃된다.
- `hailo_platform` 없음: 호스트 패키지 상태 또는 Docker 빌드 로그 확인.
- 빌드 중 `Could not resolve host`: GitHub/PyPI DNS 조회 실패다. Python 바인딩
  빌드도 CMake를 통해 GitHub의 pybind11 소스를 추가로 받는다.
- `netifaces` 설치 중 `aarch64-linux-gnu-gcc ... No such file or directory`:
  컴파일러 없는 실행 단계에서 소스 패키지를 빌드한 오류다. 의존성 wheel을
  빌드 단계에서 준비하는 수정된 Dockerfile을 사용한다.
- Hailo 드라이버 버전 불일치: `modinfo hailo_pci`, `hailortcli --version`, 호스트 패키지
  버전을 비교. 호스트 `.so`나 Python 패키지를 Jammy 컨테이너에 마운트하지 않는다.
- 카메라 부하에서 재부팅: 소프트웨어보다 전원·배선·열 상태를 먼저 확인.
- `--preview` 오류: 우선 headless로 확인. 컨테이너 기본 구성은 GUI 소켓을 전달하지 않는다.
- `--seconds 0`에서 Ctrl+C 종료 시 요약 파일은 저장되지 않을 수 있다. 측정 기록에는
  유한한 `--seconds`를 사용한다.

## 작성 환경에서 확인한 범위

- Python 문법/`--help`, Docker Compose 설정 검증 통과.
- 별도 프로세스의 합성 영상 → 최신 파일 → 수신 → JPEG 저장 확인
  (320×240, 요청 20FPS, 수신 약 19.6FPS).
- 송신 중단 후 오래된 프레임 타임아웃, 손상된 프레임 거부 확인.
- 초기 작성 환경(Mac)에서는 이미지 빌드를 검증하지 못했다.
- 2026-10-27 ARM64 Docker에서 기존 HailoRT 컴파일 캐시를 재사용한 이미지 빌드로
  `netifaces` wheel 생성, 오프라인 설치, `pip check`, OpenCV/NumPy/HailoRT import,
  HailoRT CLI 4.23.0 확인을 통과했다. CSI·Hailo 장치 실행은 이 검증에 포함하지 않았다.

## 공식 참고 자료

- [Raspberry Pi 카메라 소프트웨어](https://www.raspberrypi.com/documentation/computers/camera_software.html)
- [Picamera2 매뉴얼](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf)
- [Raspberry Pi Hailo 설치](https://www.raspberrypi.com/documentation/computers/ai.html)
- [HailoRT 4.23.0 소스](https://github.com/hailo-ai/hailort/tree/v4.23.0)
