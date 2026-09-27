# A.R.M.S. uPlot 비행 로그 뷰어

`auto_save:=true`가 만든 rosbag을 브라우저에서 확대·비교할 수 있는 uPlot 기반 뷰어다.
영상은 기존 `scripts/bag_visualize`가 담당하고, 이 도구는 수치 토픽을 다룬다.

## 녹화

```bash
# 실기체
ros2 launch arms_bringup arms.launch.py auto_save:=true

# PX4/Gazebo SITL
ros2 launch arms_bringup arms_sitl.launch.py auto_save:=true
```

자동모드가 `SEARCH`에 들어가면 `~/arms_flight_log/<일시>/`에 녹화를 시작하고 ARM 스위치를
내리면 종료한다. 현재 기본 녹화 토픽은 다음과 같다.

- 영상: `/arms/ui_image/compressed`
- 검출: `/arms/detections_raw`, `/arms/detections`, `/arms/detector_status`
- PN: `/arms/pn_debug`, `/arms/guidance_ground_truth`, `/arms/control_debug`, `/arms/debug_looming`
- 자세/FC: `/arms/attitude_quaternion`, `/arms/crsf_tx`, `/arms/crsf_rx`
- 상태/입력: `/arms/mission_state`, `/arms/command`

`detections_raw`는 detector KF 전 측정이고 `detections`는 detector KF 후 결과다. PN은 raw만
입력받아 자체 timestamp-aware LOS KF를 한 번 적용한다.

## 실행

ROS 2가 설치되고 workspace가 source된 Ubuntu에서 실행한다.

```bash
cd /path/to/ARMS/SW
python3 scripts/bag_uplot/view_bag.py ~/arms_flight_log/20260923_153000
```

Flask 서버가 `http://127.0.0.1:8765/`를 연다. 입력은 비행 폴더, 그 안의 `bag/`, 또는
기존 CSV 폴더 모두 가능하다. 로그 루트를 열 때는 선택한 기록만 CSV/JSON으로 변환하고,
변경되지 않은 bag은 이전 변환 캐시를 재사용한다.

로그 루트를 넘기면 그 안의 비행 기록들이 최신순으로 왼쪽 사이드바에 표시된다.
사이드바에서 기록을 선택해 그래프를 전환할 수 있고, 상단 메뉴 버튼으로 목록을 접을 수 있다.

```bash
python3 scripts/bag_uplot/view_bag.py ~/arms_flight_log
```

```bash
# 다른 포트
python3 scripts/bag_uplot/view_bag.py <bag> --port 9000
```

한 그래프에 마우스를 올리면 모든 그래프의 같은 시각에 세로 커서와 범례 값이 연동된다.
그래프 위에서 드래그하면 해당 시간축을 확대한다. 범례 아래 체크박스로 series를 켜고 끌 수 있다.
새 PN 로그의 주요 필드는 LOS/LOS-rate, range, closing speed, PN·접근 가속도, 목표/실제
roll·pitch, PX4에 보내는 목표 자세, throttle이다.

## 의존성

- bag 변환: ROS 2의 `rosbag2_py`, `rosidl_runtime_py`, PyYAML
- 웹 서버: Flask (`sudo apt install python3-flask`)
- 브라우저: uPlot을 jsDelivr에서 읽으므로 최초 열 때 인터넷 연결 필요
- 별도 npm 설치는 필요 없다.
