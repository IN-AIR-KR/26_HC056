# A.R.M.S. SW

## 문서

| 문서                                | 설명                                                                             |
| ----------------------------------- | -------------------------------------------------------------------------------- |
| [DESIGN.md](docs/DESIGN.md)         | 소프트웨어 아키텍처 설계 문서. 노드 구조, 상태 머신, PID 제어, 등 전체 설계 구조 |
| [SETUP.md](docs/SETUP.md)           | 설치 및 실행 매뉴얼 (실기체 + SITL 환경 구성/실행 통합)                          |
| [HOW_TO_GIT.md](docs/HOW_TO_GIT.md) | Git/GitHub 팀 사용 가이드. 기본 개념, 자주 쓰는 명령어, 작업 시나리오            |
| [TODO.md](docs/TODO.md)             | 개발 진행 현황 및 테스트 체크리스트                                              |

## 폴더 구조

```
SW/
├── docs/               # 문서
├── arms_ws/            # ROS2 워크스페이스
│   └── src/
│       ├── arms_bringup/       # 런치 파일
│       ├── arms_video/         # 영상 수신
│       ├── arms_detection/     # YOLO 객체 인식 (Docker)
│       ├── arms_control/       # 상태 머신 + PID + CRSF 출력
│       ├── arms_logging/       # rosbag 비행 기록 및 실험 manifest 저장
│       ├── arms_command/       # 조종 입력 (실기체 C++ HW / SITL 가상 조종기)
│       ├── arms_sim/           # SITL 전용: PX4 SITL 런치 + Gazebo 자산 + 표적 심판(referee) + 튜닝 콘솔(panel)
│       │   ├── launch/         #   px4_sitl.launch.py (PX4+Gazebo 기동/정리/파라미터)
│       │   └── assets/         #   시뮬 자산
│       │       ├── models/     #     Gazebo 모델 (arms_drone)
│       │       ├── worlds/     #     Gazebo 월드 (arms_sitl.sdf)
│       │       └── px4/airframes/  # PX4 airframe (4022_gz_arms_drone)
│       ├── arms_ui/            # 오퍼레이터 UI
│       └── arms_msgs/          # 커스텀 메시지
├── scripts/            # 테스트/도구 스크립트 (crsf / joy / detection_analysis / yolo 학습)
├── setup_sim.sh        # 최초 셋업 (한 번)
└── kill_all.sh         # 비상용 전체 종료
```

## 빠른 시작

### 환경

| 항목       | 내용                                      |
| ---------- | ----------------------------------------- |
| OS         | Ubuntu 22.04                              |
| ROS        | ROS2 Humble                               |
| 시뮬레이터 | Gazebo Harmonic                           |
| PX4        | `~/PX4-Autopilot` 에 clone 되어 있어야 함 |

### 설치

```bash
# 1) 최초 한 번 — 모델 링크 + airframe 등록 + PX4 빌드 + ROS 빌드
cd SW
./setup_sim.sh                       # PX4 다른 위치면: PX4_DIR=<경로> ./setup_sim.sh

# 2) 실행 — 매번 이것만
source arms_ws/install/setup.bash    # 새 터미널이면 한 번
ros2 launch arms_bringup arms_sitl.launch.py
```

`setup_sim.sh` 가 드론 모델(arms_drone, 커스텀 x500_base 형태/무게)·월드를 PX4 트리에
자동 심볼릭 링크하고, airframe `4022_gz_arms_drone` 등록 + PX4/ROS 빌드까지 처리한다.
이후 `ros2 launch arms_bringup arms_sitl.launch.py` 하나로 PX4 SITL + Gazebo + socat + 전체
ROS 스택 + 패널 GUI 가 다 뜨고, Ctrl+C 한 번에 전부 내려간다.

- ROS 코드만 고쳐서 스택만 재시작하고 싶으면 (PX4/Gazebo 는 살려둔 채):
  `ros2 launch arms_bringup arms_sitl.launch.py px4:=false`
- PX4 위치가 다르면 `px4_dir:=<경로>`
