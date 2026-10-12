# A.R.M.S. 설치 및 실행 매뉴얼

두 가지 모드를 다룹니다.

- **[A] 실기체** — Jetson + FPV 수신기 + FC(Flight Controller)
- **[B] SITL** — PX4 + Gazebo Harmonic (시뮬레이션)

## 공통 사전 준비

### OS / ROS2

| 항목   | 버전      |
| ------ | --------- |
| Ubuntu | 22.04 LTS |
| ROS2   | Humble    |

```bash
# ROS2 환경 소싱 (매 터미널마다 또는 bashrc에 추가)
source /opt/ros/humble/setup.bash
```

### 의존성 설치

```bash
# 카메라 드라이버 (실기체 FPV 캡처 — usb_cam 대신 v4l2_camera 사용)
#   아날로그→USB 캡처 동글(MS210x/EasierCAP)은 프레임 간격을 stepwise 로
#   보고해 usb_cam 이 포맷 열거에 실패한다. v4l2_camera(C++)는 정상 동작.
sudo apt install ros-humble-v4l2-camera

# NumPy 1.x 고정 (apt python3-opencv 4.5.4 는 NumPy 1.x 로 빌드됨.
#   pip numpy 2.x 가 있으면 cv2 import 시 크래시)
pip3 install "numpy<2"

# MAVSDK (arms_control C++ 빌드에 필요)
sudo apt install libmavsdk-dev

# GPIO (실 기체 Jetson에서만 필요)
sudo apt install libgpiod-dev

# ros_gz_bridge (SITL에서만 필요 — Harmonic은 B-1 소스 빌드 참고)
sudo apt install ros-humble-ros-gz-bridge

# Docker (detection 컨테이너)
sudo apt install docker.io docker-compose-plugin
sudo usermod -aG docker $USER   # 재로그인 필요
```

### 워크스페이스 빌드

```bash
cd SW/arms_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 이후 모든 터미널에서 반드시 실행:
>
> ```bash
> source /opt/ros/humble/setup.bash
> source SW/arms_ws/install/setup.bash
> ```

## A. 실 기체 실행

### A-1. Detection 컨테이너 빌드 (최초 1회)

```bash
cd SW/arms_ws/src/arms_detection/docker

# 모델 파일 배치 — Jetson은 TensorRT .engine
cp /path/to/best.engine models/

# 이미지 빌드 (최초 or Dockerfile 변경 시에만)
docker compose -f docker-compose.jetson.yml build
```

> 컨테이너 기동은 A-2 의 `arms.launch.py` 가 `docker compose up -d` 로 자동 처리합니다.
> `ROS_DOMAIN_ID` 가 기본값(0)이 아니면 런치 실행 전에 `export ROS_DOMAIN_ID=<n>` 하세요.

### A-2. 전체 시스템 시작

```bash
ros2 launch arms_bringup arms.launch.py
```

노드 스택과 함께 detection 컨테이너(`docker compose up -d`)까지 자동 기동됩니다.

#### 검출 모델(가중치) 지정 — `model:=`

detection 이 로드할 가중치는 `model:=` 로 고릅니다. 경로는 **컨테이너 내부 경로**
(`/models/<파일>`)입니다 — 호스트의 `.../docker/models/` 가 컨테이너에 `/models` 로
마운트되므로 파일명만 `/models/` 뒤에 붙이면 됩니다(호스트 절대경로 아님).

```bash
# 기본 모델 — 그냥 실행
ros2 launch arms_bringup arms.launch.py

# 다른 모델 지정 (아래 ⚠️ — 컨테이너가 이미 떠 있으면 이것만으론 안 바뀝니다)
ros2 launch arms_bringup arms.launch.py model:=/models/drone_small.engine
```

> ⚠️ **`model:=` 는 detection 컨테이너를 "새로 만들 때"만 반영됩니다.** 런치의 `docker compose
> up -d` 는 멱등이라, 이미 다른 모델로 떠 있는 컨테이너는 그대로 두고 모델을 **안 바꿉니다**
> (`restart` 도 마찬가지). 실행 중인 컨테이너의 모델을 바꾸려면 **`ARMS_MODEL` + 재생성
> (`--force-recreate`)** 을 먼저 하고 런치를 띄우세요:
>
> ```bash
> cd SW/arms_ws/src/arms_detection/docker
> ARMS_MODEL=/models/drone_small.engine docker compose -f docker-compose.jetson.yml up -d --force-recreate
> ```
>
> 그 다음(또는 컨테이너가 아직 안 떠 있을 때)만 런치의 `model:=` 이 그대로 적용됩니다.

- 기본 모델은 각 launch의 `model` 기본값을 따릅니다.
- Jetson 은 **`.engine`(TensorRT)** 만 로드합니다. `.pt` 는 먼저 변환하세요:
  `python3 SW/yolo/export_trt.py`(모델 지정: `ARMS_PT=/models/<파일>.pt`). 변환된 `.engine`
  은 `.../docker/models/` 에 생기고 컨테이너 `/models/` 에서 바로 보입니다.
- 현재 로드된 모델 확인:
  ```bash
  docker inspect docker-arms_detection-1 --format '{{range .Config.Env}}{{println .}}{{end}}' | grep ARMS_MODEL
  ```

### A-3. 샘플 영상 replay (카메라·FC 없이 파이프라인 테스트)

저장된 샘플 영상으로 detection → control → UI 파이프라인을 검증합니다. 캡처 동글이나
FC 없이 검출·추적·UI를 그대로 돌려볼 수 있어, 검출 튜닝이나 UI 확인에 유용합니다.
라이브 카메라 대신 `image_publisher` 가 영상 파일을 `/arms/camera/image_raw` 로 스트리밍합니다.

detection 은 `arms.launch.py` 와 동일하게 **GPU 도커 컨테이너로 자동 기동**됩니다
(`docker compose up -d`, 멱등). 즉 실기체와 같은 YOLO 검출을 샘플 영상에 그대로 적용합니다.

조종기는 `controller:=` 인자로 고릅니다(기본 `gui`):

- **`controller:=gui`** (기본) — **SITL 가상 조종기(tkinter GUI)** 가 함께 떠서, 그 창에서
  ARM / 모드 / RTL / LAUNCH를 클릭으로 넣어 상태 전이(SEARCH→LOCK→…)를 테스트합니다.
  실기체 조종기·수신기 없이 파이프라인만 돌려볼 때 편합니다.
- **`controller:=real`** — 실기체 **ESP32 조종기(UART → `operator/input`)** 를 그대로 씁니다.
  실제 조종기 입력으로 replay 영상 검증 시 사용합니다(ESP32 시리얼 연결 필요).

```bash
ros2 launch arms_bringup arms_replay.launch.py                    # gui 가상 조종기(기본)
ros2 launch arms_bringup arms_replay.launch.py controller:=real   # 실기체 ESP32 조종기
```

```bash
# 기본 샘플(sample1.mov) + 기본 모델
ros2 launch arms_bringup arms_replay.launch.py

# 다른 영상 지정 (절대경로 권장)
ros2 launch arms_bringup arms_replay.launch.py \
  video_path:=/path/to/SW/sample_viedo/sample2.mov

# 발행 fps 조정
ros2 launch arms_bringup arms_replay.launch.py publish_rate:=15.0

# 다른 가중치 사용 / detection 컨테이너 끄기
ros2 launch arms_bringup arms_replay.launch.py model:=/models/<파일>.engine
ros2 launch arms_bringup arms_replay.launch.py start_detection:=false
```

| 인자              | 기본값                            | 설명                                      |
| ----------------- | --------------------------------- | ----------------------------------------- |
| `video_path`      | `SW/sample_viedo/sample1.mov`     | 재생할 영상 파일 (절대경로 권장)          |
| `publish_rate`    | `30.0`                            | `/arms/camera/image_raw` 발행 fps                |
| `model`           | launch 기본값                         | detection 컨테이너가 로드할 가중치 (컨테이너 경로 `/models/…`) |
| `start_detection` | `true`                            | detection 도커 컨테이너 자동 기동 여부    |
| `controller`      | `gui`                             | 조종기 소스: `gui`(SITL 가상 조종기) / `real`(실기체 ESP32) |
| `crsf_port`       | `/dev/ttyUSB0`                    | ELRS TX 시리얼 포트 (제어 출력용)         |
| `fullscreen`      | `false` (replay) / `true` (실기체)| UI 전체화면 여부 (A-4)                    |
| `debug`           | `true` (replay) / `false` (실기체)| 메인 화면 디버그 오버레이 (A-4)           |
| `cv_debug`        | `false`                           | 검출 진단 패널(HSV+그래프) (A-4)          |
| `auto_save`      | `false` — **`arms.launch.py`(실기체) / `arms_sitl.launch.py`** | 비행 로그 자동 녹화. ARM 스위치 ON~OFF를 기준으로 `~/arms_flight_logs_ibvs/<일시>/`에 bag(sqlite3)과 manifest를 저장 |

샘플 영상은 `SW/sample_viedo/` 에 `sample1.mov`, `sample2.mov`, `sample3.mov` 가 있습니다.

> `auto_save` 는 `arms.launch.py`(실기체)와 `arms_sitl.launch.py`(SITL)에 있는 인자입니다. 위 표의 나머지 인자는
> `arms_replay.launch.py` 기준이며, 실기체와 공유되는 것(`model`/`start_detection`/`crsf_port`/
> `fullscreen`/`debug`/`cv_debug`)은 `arms.launch.py` 에서도 동일하게 씁니다.

의존성 (영상 스트리밍):

```bash
sudo apt install ros-humble-image-publisher
```

> detection 컨테이너 이미지가 최초라면 A-1 처럼 먼저 `docker compose -f
docker-compose.jetson.yml build` 가 필요합니다.

### A-4. UI 표시 옵션 (`debug` / `cv_debug` / `fullscreen`)

UI 화면 구성은 세 개의 독립 런치 인자로 제어합니다. 실기체(`arms.launch.py`)와
replay(`arms_replay.launch.py`)의 **기본값이 다릅니다**.

| 인자         | 대상 파라미터   | 실기체 기본 | replay 기본 | 켜면 표시되는 것 |
| ------------ | --------------- | ----------- | ----------- | ---------------- |
| `fullscreen` | `ui.fullscreen` | `true`      | `false`     | 전체화면(작업표시줄까지 가림) vs 창모드 |
| `debug`      | `ui.debug`      | `false`     | `true`      | **메인 화면 디버그 오버레이** — err/cmd 숫자 텍스트, YOLO/HSV 상태 텍스트, 오차(ERR)·명령(CMD) 화살표 |
| `cv_debug`   | `ui.cv_debug`   | `false`     | `false`     | **검출 진단 패널** — 화면 오른쪽에 HSV 후보 미리보기 + 실시간 성능 그래프 |

- **`debug` (메인 오버레이).** `false` 면 실기체처럼 메인 화면만 깔끔하게 나옵니다.
  단, **표적 bounding box·confidence 는 `debug` 와 무관하게 항상** 그려집니다(상태 라벨/십자선/
  배터리/락 진행바/전원 버튼도 항상). `true` 면 그 위에 조준 오차·제어 명령 화살표와
  숫자 디버그 텍스트, 검출기(YOLO/HSV) 작동 상태가 추가로 뜹니다.
- **`cv_debug` (진단 패널).** 켜면 UI 가 `/arms/hsv_debug_image`(≈500KB raw)를 구독하고,
  그때만 detection 노드가 그 영상을 발행합니다 → **컨테이너→호스트 UDP 대용량 전송으로
  검출 발행률이 크게 떨어집니다**(측정상 ~30Hz → ~12Hz). 그래서 기본 off 이며, 검출 튜닝·
  성능 확인이 필요할 때만 켭니다. `debug` 와 독립이라 따로 켤 수 있습니다.

```bash
# 실기체를 디버그 오버레이까지 켜서 실행 (기본은 메인 화면만)
ros2 launch arms_bringup arms.launch.py debug:=true

# replay 를 실기체처럼 메인 화면만으로
ros2 launch arms_bringup arms_replay.launch.py debug:=false

# replay 에 검출 진단 패널까지 (검출 발행률 하락 감수)
ros2 launch arms_bringup arms_replay.launch.py cv_debug:=true

# 창모드 replay 를 전체화면으로
ros2 launch arms_bringup arms_replay.launch.py fullscreen:=true
```

### A-5. 검출 튜닝 config 선택 (실외 / 실내)

detection 컨테이너는 `--params-file` 로 검출 파라미터 yaml 을 로드합니다. 파일은
`SW/arms_ws/src/arms_detection/config/` 에 있고, 컨테이너에는 `/params` 로 바인드
마운트됩니다(yaml 만 고치고 `docker compose ... restart` 하면 반영, 재빌드 불필요).

| config 파일             | 역할 | 특징 |
| ----------------------- | ---- | ---- |
| `detection.yaml` | **기본(항상 로드) = 실외** | 검출 노드의 전체 파라미터. 자연광 가정 — 색 게이트를 조여 하늘/반사광 오탐↓, 원거리 작은 표적 허용 |
| `detection.indoor.yaml` | 실내 override | 기본 위에 얹는 **델타만**. 형광/LED 가정 — 채도 낮은 붉은색도 후보로, 근접 큰 표적 허용 |

기본 config(`detection.yaml`)는 **항상 먼저 로드**되고, `ARMS_DET_CONFIG` 로
override 파일을 지정하면 그 위에 덮어씁니다. 지정 안 하면 기본(실외)만 로드합니다:

```bash
cd SW/arms_ws/src/arms_detection/docker

# 기본(실외) — 그냥 기동하면 detection.yaml 만 로드
docker compose -f docker-compose.jetson.yml up -d

# 실내 override 를 얹어 기동
ARMS_DET_CONFIG=detection.indoor.yaml docker compose -f docker-compose.jetson.yml up -d

# 이미 떠 있는 컨테이너에 override 만 바꿔 재기동
ARMS_DET_CONFIG=detection.indoor.yaml docker compose -f docker-compose.jetson.yml up -d --force-recreate
```

- `arms.launch.py` / `arms_replay.launch.py` 가 자동 기동하는 컨테이너도 이 환경변수를
  따릅니다. 런치 실행 전에 `export ARMS_DET_CONFIG=detection.indoor.yaml` 하면 됩니다.
- 새 환경(예: 야간)을 추가하려면 `detection.indoor.yaml` 처럼 **바뀌는 값만** 담은
  override yaml 을 `config/` 에 만들고 `ARMS_DET_CONFIG=<파일명>` 으로 가리키면 됩니다.
- 상시 기본값을 바꾸려면 `detection.yaml` 을 직접 고칩니다(전체 파라미터가 여기 있음).
  예: 부하를 아끼려면 `yolo.full_fallback_interval` 을 `2` 이상으로.
- `detection.yaml` 값은 노드의 `declare_parameter` 기본값과 같게 유지합니다. 따라서
  config 없이 노드만 띄워도 같은 값으로 동작하고, override yaml(예: indoor)에는 바뀌는
  값만 넣으면 됩니다.

## B. SITL 실행

| 항목          | 버전           |
| ------------- | -------------- |
| Gazebo        | Harmonic (8.x) |
| PX4-Autopilot | 최신           |
| Python        | 3.10+          |

### B-1. SITL 환경 구성 (최초 1회)

#### ros_gz_bridge (소스 빌드 필수)

> ⚠️ apt의 `ros-humble-ros-gz-bridge`는 Gazebo Fortress (`ign-msgs 8`, `ign-transport 11`) 기준으로 빌드되어 있습니다. 반면 PX4 SITL은 Gazebo Harmonic (`gz-msgs 10`, `gz-transport 13`)을 사용하므로 버전이 맞지 않아 `Unknown message type [9]` 에러가 발생하고 ROS2 토픽이 브릿지되지 않습니다. 반드시 Harmonic 버전으로 소스 빌드해야 합니다.

```bash
# Harmonic dev 패키지 설치
sudo apt install libgz-msgs10-dev libgz-transport13-dev -y

# 소스 클론 및 빌드
mkdir -p ~/ros_gz_harmonic_ws/src
cd ~/ros_gz_harmonic_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble

cd ~/ros_gz_harmonic_ws
source /opt/ros/humble/setup.bash
GZ_VERSION=harmonic colcon build --packages-select ros_gz_bridge ros_gz_interfaces

source ~/ros_gz_harmonic_ws/install/setup.bash
```

설치 확인:

```bash
ldd ~/ros_gz_harmonic_ws/install/ros_gz_bridge/lib/libros_gz_bridge.so | grep gz-msgs
# libgz-msgs10.so.10 이 보여야 정상
```

#### worlds / models 등록

`make px4_sitl`은 빌드만 하며 커스텀 모델/월드는 자동 등록되지 않습니다.
`SW/setup_sim.sh` 가 심볼릭 링크 + airframe 등록 + PX4/ROS 빌드를 한 번에 처리합니다.

```bash
cd SW
./setup_sim.sh                 # PX4 위치가 다르면: PX4_DIR=<경로> ./setup_sim.sh
```

<details>
<summary>수동으로 링크하려면</summary>

```bash
# 월드 파일 링크
cd ~/PX4-Autopilot/Tools/simulation/gz/worlds/
ln -s /path/to/SW/arms_ws/src/arms_sim/assets/worlds/arms_sitl.sdf .

# 모델 폴더 링크
cd ~/PX4-Autopilot/Tools/simulation/gz/models/
ln -s /path/to/SW/arms_ws/src/arms_sim/assets/models/arms_drone .
```

</details>

### B-2. 실행 — 원클릭 (권장)

```bash
cd SW
source arms_ws/install/setup.bash     # 새 터미널이면 한 번
ros2 launch arms_bringup arms_sitl.launch.py
```

런치 하나로 PX4 SITL(arms_drone) + Gazebo + 전체 ROS 스택 + 패널 GUI 가 다 뜨고,
Ctrl+C 한 번에 PX4/Gazebo 까지 전부 내려갑니다. PX4 쪽은 `arms_sim/launch/px4_sitl.launch.py` 가 담당하며
기동 때마다 월드/모델 링크 갱신 → PX4 바이너리 직접 실행(`-d` 데몬 모드) → 부팅 후 파라미터 세팅(페일세이프 해제 등)을 합니다.

| 인자 | 기본 | 설명 |
| --- | --- | --- |
| `px4:=false` | true | PX4/Gazebo 는 띄우지 않음. 이미 떠 있는 상태에서 ROS 스택만 재시작할 때 |
| `px4_dir:=<경로>` | `$PX4_DIR` 또는 `~/PX4-Autopilot` | PX4-Autopilot 위치 |

YOLO 검출을 쓰려면 Detection 컨테이너를 따로 실행합니다(B-2′ Terminal 3).
런치가 비정상 종료돼 프로세스가 남았으면 `SW/kill_all.sh`.

> PX4 는 `make` 를 거치지 않고 빌드된 바이너리를 바로 실행하므로, airframe(`arms_sim/assets/px4/airframes`) 을
> 고쳤으면 `setup_sim.sh` 로 PX4 를 다시 빌드해야 반영됩니다.

### B-2′. 실행 — 수동 (터미널 3개)

원클릭을 안 쓰고 단계별로 실행하려면 (Terminal 2 는 `px4:=false` 필수):

**Terminal 1 — PX4 SITL + Gazebo**

```bash
cd ~/path/to/~PX4-Autopilot
PX4_GZ_WORLD=arms_sitl make px4_sitl gz_arms_drone
```

Gazebo 창과 PX4 콘솔이 뜨면 정상입니다.

**Terminal 2 — A.R.M.S. 노드**

```bash
source /opt/ros/humble/setup.bash
source /path/to/SW/arms_ws/install/setup.bash
source ~/ros_gz_harmonic_ws/install/setup.bash

ros2 launch arms_bringup arms_sitl.launch.py px4:=false
```

`arms_sitl.launch.py`가 gz 카메라 브리지, control, SITL MAVLink I/O, UI, detection, 패널 GUI를 한 번에 기동합니다.
이 경우 PX4 파라미터(페일세이프 해제 등)는 세팅되지 않으니 필요하면 `Tools/mavlink_shell.py` 로 직접 넣으세요.

#### 3D 시각화 — `rviz:=true`

```bash
ros2 launch arms_bringup arms_sitl.launch.py rviz:=true
```

`arms_sim/sitl_viz` 와 `rviz2`(설정: `arms_bringup/config/sitl.rviz`)가 같이 뜹니다. 화면에는
기체(메쉬)·표적·양쪽 비행 궤적이 그려집니다. 기본값은 `false` 입니다.

- 데이터는 **Gazebo 진실값**(`dynamic_pose/info`)입니다. 검출·추정을 거치지 않으므로
  "제어가 실제로 어디를 날았나"를 보는 용도이며, 실기체에는 없는 정보입니다.
- 궤적은 `/arms/reset`(패널 RESET)에서 비워집니다. 에피소드 경계가 선으로 남지 않습니다.
- 발행 토픽: `/arms/viz/drone_path`, `/arms/viz/target_path`, `/arms/viz/markers`, `/tf`.
  Fixed Frame 은 `map` 입니다.
- 이 스택은 `use_sim_time` 을 쓰지 않으므로(전부 벽시계) `/clock` 브리지나 별도 시간 설정이
  필요 없습니다.

#### 검출 실행 위치 — `detection:=`

| 값 | 동작 |
| --- | --- |
| `host` (기본) | 호스트에서 검출 노드를 직접 실행. `det_model:=` 가중치를 in-process 로 로드하며 호스트 GPU 를 그대로 씁니다. 컨테이너 경계가 없어 SITL 에선 이쪽이 가장 단순합니다. |
| `docker` | 실기체와 같은 검출 컨테이너를 `docker compose up -d` 로 기동하고, 호스트 노드는 띄우지 않습니다. |

```bash
ros2 launch arms_bringup arms_sitl.launch.py                                       # host (기본)
ros2 launch arms_bringup arms_sitl.launch.py det_model:=/models/balloon_small_v4.pt
ros2 launch arms_bringup arms_sitl.launch.py detection:=docker
```

- ⚠️ **`host` 와 `docker` 를 동시에 쓰면 안 됩니다.** 둘 다 `/arms_detection_node` 라는 같은
  이름으로 `/arms/perception/detections` 를 발행해서 서로를 덮어씁니다.
- **가중치 인자는 `det_model:=` 입니다** — 다른 런치처럼 `model:=` 이 아닙니다. 이 런치는
  `px4_sitl.launch.py` 를 include 하는데 거기서 `model:=` 은 **PX4 airframe 이름**
  (`gz_arms_drone`)이고, 부모 런치 인자는 include 된 런치로 그대로 새어 들어가기 때문입니다.
  (`model:=` 에 가중치 경로를 주면 PX4 가 그걸 `PX4_SIM_MODEL` 로 받아 즉시 죽고 Gazebo 도
  안 뜹니다.)
- `det_model:=` 은 다른 런치의 `model:=` 과 같은 **컨테이너 내부 경로**(`/models/<파일>`)
  표기입니다. `host` 모드에선 호스트의 `docker/models/<파일>` 로 자동 치환됩니다.
  SITL 기본 모델은 PyTorch `.pt` 형식입니다.
- **가중치는 git-lfs 로 관리됩니다.** `git lfs install && git lfs pull` 을 하지 않으면 파일이
  200바이트짜리 포인터 텍스트라 로드에 실패합니다 — 런치가 기동 전에 미리 경고합니다.
- **YOLO 없이는 검출이 아예 안 나옵니다.** HSV 는 후보 제안일 뿐 발행 권한이 없어
  (`use_hsv` 는 그 후보 단계만 끕니다), 모델이 없으면 `/arms/perception/detections` 가 계속 빕니다.
- `detection:=docker` 는 SITL gz 카메라가 raw 만 내보내므로 `ARMS_INPUT_COMPRESSED=0` 을
  자동으로 넣어 줍니다 (compose 기본은 압축 입력 `1`).

## 동작 확인

### 토픽 모니터링

```bash
# 상태 머신 상태 확인
ros2 topic echo /arms/mission/state

# detection 확인
ros2 topic echo /arms/perception/detections

# 발사 판정용 looming 확인 (x=τ, y=bbox크기, z=팽창률)
ros2 topic echo /arms/control/looming_debug

# 카메라 영상 확인
ros2 run rqt_image_view rqt_image_view /arms/camera/image_raw
```

### 검출 화면 캡처 (`S` 키)

A.R.M.S. UI 창이 떠 있을 때 **`S`(또는 `s`) 키**를 누르면 현재 화면을 JPG로 저장한다.
검출·HSV 튜닝 자료 수집용. **한 번에 두 장**이 같은 타임스탬프 파일명으로 저장된다:

- `SW/output_image/detection/<YYYYmmdd_HHMMSS_mmm>.jpg` — 오버레이 포함 검출 화면
- `SW/output_image/hsv/<...같은 이름>.jpg` — HSV 마스크·후보 화면

- 저장 폴더는 `ui.capture_dir` 파라미터로 바꿀 수 있다(기본 `SW/output_image`).
- 평소 UI 는 `hsv_debug_image` 를 구독하지 않는다(검출 발행률 보호). `S` 를 누른 **그 순간에만
  잠깐 구독**해 HSV 한 프레임을 받아 저장하고 바로 해제하므로, 저장까지 아주 짧은 지연이 있을 수 있다.
  (`cv_debug` 패널이 이미 켜져 있으면 즉시 저장.)
- **A.R.M.S. 창이 포커스**돼 있어야 키가 먹는다(OpenCV `waitKey` 기반).
- `SW/output_image/` 는 런타임 생성물이라 git 추적 대상이 아니다(`.gitignore`).

### UI 화면 녹화 / 재생 / mp4 변환

UI 노드는 화면을 두 토픽으로 발행한다(둘 다 구독자 있을 때만, lazy):

- `/arms/ui/image` (raw `sensor_msgs/Image`) — rqt 로 바로 보기용.
- `/arms/ui/image/compressed` (JPEG) — 경량 녹화·원격 스트리밍용. **launch 의 image_transport
  `republish` 노드가 만든다**(파이썬 노드는 image_transport 규약 compressed 를 직접 못 냄).

발행 화질/대역폭은 파라미터로 조절: `ui.stream_max_width`(기본 480), `ui.stream_fps`(기본 15),
JPEG 품질은 republish 의 `compressed.jpeg_quality`(launch 에 60).

#### 1) 녹화 (compressed = 경량)

```bash
ros2 bag record -o ui_rec /arms/ui/image/compressed
```

> mcap 으로 저장하려면 `--storage mcap` (플러그인 필요: `sudo apt install ros-humble-rosbag2-storage-mcap`).
> 안 깔려 있으면 옵션 없이 기본 `sqlite3` 로 저장된다.

#### 2) 재생하며 화면으로 보기 (compressed → raw → rqt)

rqt 는 압축 토픽을 직접 표시 못 하므로, 재생하면서 raw 로 풀어서 본다(터미널 3개):

```bash
# ① 재생 (--loop 반복재생)
ros2 bag play ui_rec --loop

# ② compressed → raw 디코딩
ros2 run image_transport republish compressed raw \
  --ros-args -r in/compressed:=/arms/ui/image/compressed -r out:=/arms/ui_view

# ③ raw 보기
ros2 run rqt_image_view rqt_image_view /arms/ui_view
```

> ⚠️ 라이브 스택(UI 노드)이 돌고 있으면 재생과 같은 토픽에 섞인다. 스택을 끄거나, 재생 시
> `--remap /arms/ui/image/compressed:=/pb/ui_image/compressed` 로 토픽명을 바꿔 충돌을 피한다.

#### 3) 비행 자동 저장 모드 (`auto_save:=true`, 실기체 / SITL)

실기체 비행 중 로그(영상, 검출, 정규화 텔레메트리, 제어 상태)를 **자동으로** 녹화한다.
`arms.launch.py` 에 `auto_save:=true` 를 주면 `flight_recorder` 노드가 함께 떠서:

- **시작**: 조종기 **ARM 스위치를 올릴 때**(`operator/input.arm_switch` false→true).
- **종료**: **ARM 스위치를 내릴 때**. 한 번 올림~내림 = 기록 폴더 하나.
  - 모드(auto/manual)와 상태 머신에 독립적이다.
  - 녹화기가 뜰 때 이미 ARM 이 위였다면 한 번 내렸다 올려야 시작된다(상승엣지).
- **SITL**: `arms_sitl.launch.py auto_save:=true` 도 같은 조건(가상 조종기 ARM 스위치).
  폴더명이 `<일시>_sitl` 이다. UI 화면 영상도 실기체와 똑같이 저장된다.
- **저장 위치**: `~/arms_flight_logs_ibvs/<YYYYmmdd_HHMMSS>/` (비행 시작 일시 폴더). 이 폴더 안에
  `bag/`(rosbag) 과 `record.log`(녹화 프로세스 로그)가 함께 담긴다.

```bash
ros2 launch arms_bringup arms.launch.py auto_save:=true
```

기본값은 `false`(녹화 안 함). 저장된 bag은 `ibvs_viewer`에서 열어 영상, 3D 궤적, 텔레메트리와 제어 그래프를 함께 분석한다. 녹화 토픽은 UIㆍ원본 compressed 영상, 검출 결과, `control/*`, `control/flight_command`, `io/status`, `vehicle/*`, `mission/state`, `operator/input`이다. 실제 목록은 `flight_recorder`의 `record_topics` 파라미터와 비행 폴더 `manifest.json`에 남는다.

비행 로그는 웹뷰어에서 분석한다(ROS 불필요, 그래프 호버/줌 연동):

```bash
python3 SW/scripts/ibvs_viewer/server.py --data ~/arms_flight_logs_ibvs
```

자세한 사용법은 `SW/scripts/ibvs_viewer/README.md`.

### 정상 상태 전이 흐름

```
시작         →  IDLE
ARM 요청     →  SEARCH   (검출 대기)
연속 인식    →  LOCK     (잠금 타이머)
LAUNCH       →  TRACK    (유도·추력 제어)
타겟 소실/RTL 조건 → RTL
ARM 해제     →  IDLE
```

### 노드 그래프 확인

```bash
ros2 node list
ros2 topic list
rqt_graph
```

### 카메라 테스트

```bash
ffplay -f v4l2 -framerate 30 -video_size 720x480 -i /dev/video0
```

### PlotJuggler 실시간 그래프 (제어 튜닝용)

오차(`/arms/mission/state`)·유도 출력(`/arms/control/debug`)·발사 τ(`/arms/control/looming_debug`)를
실시간 플롯한다. SITL 튜닝 시 오버슈트·발산을 눈으로 본다.

설치:

```bash
sudo apt install ros-humble-plotjuggler-ros
```

레이아웃 지정 실행:

```bash
ros2 run plotjuggler plotjuggler --buffer_size 60 \
  --layout SW/arms_ws/src/arms_bringup/config/sitl_debug.xml
```

> - 창이 뜨면 상단 **Streaming → Start** → **ROS2 Topic Subscriber** 로 스트리밍을 켜야
>   데이터가 흐른다(토픽 `control_debug`/`debug_looming`/`mission_state`는 레이아웃에 이미 선택됨).
> - 데이터는 `arms_control_node` 가 돌고 있을 때만 나온다(그 노드가 발행). 스택 없이
>   PlotJuggler만 켜면 빈 화면.
