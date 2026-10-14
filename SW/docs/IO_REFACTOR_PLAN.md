# I/O 분리 및 ROS 인터페이스 정리 계획

> 상태: **단계 분할 계획**. 통신 분리와 전체 토픽명 전환을 같은 변경으로 묶지 않는다.
> 각 단계는 독립적으로 SITL 비행 검증을 통과한 뒤 다음 단계로 진행한다.

## 1. 목표

현재 `arms_control_node`는 제어 판단, ESP32 조종 입력 수신, CRSF 프레임 처리,
ELRS 송신, 실기체 telemetry 정규화를 함께 맡고 있다. 이 구조는 SITL과 실기체의
차이를 control 내부에 누적시키고, 비행 로그의 의미도 모호하게 만든다.

리팩터링 후의 원칙은 다음과 같다.

1. `arms_control_node`는 환경을 알지 않는다. 조종 의도·인식·정규화된 기체 상태를
   입력으로 받고, 정규화된 비행 명령과 미션 상태를 출력한다.
2. 실기체의 ESP32/ELRS/CRSF와 SITL의 MAVLink/PX4 차이는 I/O 어댑터에서 끝낸다.
3. UI, recorder, Flight Analyzer는 실기체와 SITL에 공통인 정규화 토픽만 사용한다.
4. 한 토픽에는 한 launch 구성에서 하나의 책임 있는 publisher만 둔다.
5. `io/status`는 link의 관측 결과이고, RTL 판단은 control의 책임이다.

### 작업 분할 원칙

이 계획에는 성격이 다른 두 작업이 있다.

1. **통신 경계 분리**: control에서 ESP32·CRSF·MAVLink를 제거하고, 기체 비종속
   입력/명령 계약으로 I/O 어댑터와 연결한다.
2. **전역 토픽명 전환**: camera, perception, vehicle, mission, sim 등 전체 공개
   ROS API를 상대 이름·도메인 이름으로 바꾼다.

둘을 한 번에 수행하면 비행 이상이 생겼을 때 제어 로직, I/O 변환, namespace/remap,
UI·recorder·viewer 중 어느 쪽이 원인인지 분리할 수 없다. 따라서 **1번을 먼저 끝내고
안정 비행을 확인한 뒤에만 2번에 착수**한다.

통신 분리 단계에서 `operator/input`, `control/flight_command`, `io/status` 세 이름은
예외적으로 먼저 도입한다. 이들은 단순 rename이 아니라 control과 하드웨어 사이의 새
계약 자체이므로, 이를 미루면 I/O 분리가 불가능하다. 그 외의 공개 토픽명은 토픽명
전환 단계까지 기존 이름을 유지한다.

### Namespace 규칙

이 문서의 **to-be 토픽 이름은 모두 상대 이름**이다. 노드 코드에는 선행 `/`나 고정
`arms` 토큰을 넣지 않는다. 최상위 launch가 `namespace` argument(기본값 `arms`)를
`PushRosNamespace`로 적용하며, 기본 실행에서만 `vehicle/gps`가
`/arms/vehicle/gps`로 해석된다.

```text
기본 launch:       namespace:=arms        → /arms/vehicle/gps
다기체 launch:     namespace:=arms/uav_01 → /arms/uav_01/vehicle/gps
노드 소스/계약:     vehicle/gps             (상대 이름)
```

As-is 열은 현재 실제 절대 이름을 보존해 표시한다.

### 호환성 방침

이 전환은 하위 호환을 제공하지 않는 **단일 API 전환**이다.

- old/new 토픽 alias, dual publisher, remap fallback을 넣지 않는다.
- 새 `ARMS Flight Analyzer`는 이전 토픽 이름이 든 rosbag을 열지 않는다.
- 새 recorder manifest에는 `topic_schema_version`을 기록하고, 이 전환 뒤에는 새
  schema만 지원한다.
- 기존 rosbag은 전환 전 버전의 viewer로만 열람한다.

그 대가로 control, UI, recorder, viewer가 하나의 이름·메시지 계약만 유지하게 된다.

### 시간축과 SITL reset 방침

- Flight Analyzer의 재생·영상·3D·그래프 동기화 기준은 **rosbag 저장 timestamp**다.
  recorder가 같은 호스트에서 관측한 단일 시간축이므로 현재 목적에 가장 적합하다.
- ROS header stamp는 측정·수신·생성 시각 및 지연 분석을 위한 보조 정보다. 모든 새
  정규화 메시지는 header stamp를 채우되, viewer의 주 시간축을 header stamp로 바꾸지 않는다.
- 현재 SITL은 `/clock`과 `use_sim_time`을 쓰지 않으며, 이 방침도 유지한다. manifest에는
  `use_sim_time`, launch namespace, `topic_schema_version`을 기록한다.
- `sim/reset` service와 `sim/reset_event` topic은 모두 유지한다. service는 panel의 요청/결과,
  event는 PX4 재기동 성공 뒤 control·referee·UI의 단순 상태 초기화 용도다. event는
  `std_msgs/Empty`, 기본 reliable/volatile QoS로 충분하며 lifecycle·reset ID는 도입하지 않는다.
- GPS Rescue 불가 상황의 fallback은 FC 설정에 맡긴다. `io/status`에는 RTL capability를 추가하지 않는다.

## 2. 목표 아키텍처

```mermaid
flowchart LR
    OP[ESP32 조종 입력 또는 SITL 가상 조종기] --> OI["operator/input\nOperatorInput"]

    CAM[실기체 / Gazebo 카메라] --> IMAGE["camera/*"]
    IMAGE --> DET[arms_detection_node]
    DET --> PER["perception/*"]

    OI --> CTRL[arms_control_node]
    PER --> CTRL
    VEH["vehicle/*"] --> CTRL
    IOSTATUS["io/status"] --> CTRL
    SIM_EVENT["sim/reset_event\nSITL only"] --> CTRL
    HIT["sim/hit\nSITL only"] --> CTRL

    CTRL --> FCMD["control/flight_command\nFlightCommand"]
    CTRL --> MISSION["mission/state"]
    CTRL --> DEBUG["control/*_debug"]

    subgraph REAL[실기체]
        INPUT[arms_operator_hardware_node]
        CRSF[arms_crsf_io_node]
        ELRS[ELRS / CRSF]
        FC1[Betaflight FC]
        INPUT --> OI
        FCMD --> CRSF
        CRSF <--> ELRS
        ELRS <--> FC1
        CRSF --> VEH
        CRSF --> RAW["io/crsf/rx, tx"]
        CRSF --> IOSTATUS
    end

    subgraph SITL[SITL]
        VJOY[arms_operator_gui_node]
        BRIDGE[sitl_bridge_node]
        PX4[PX4 MAVLink]
        GZ[Gazebo]
        VJOY --> OI
        FCMD --> BRIDGE
        BRIDGE <--> PX4
        PX4 <--> GZ
        BRIDGE --> VEH
        BRIDGE --> IOSTATUS
    end

    VEH --> OBS[UI · recorder · Flight Analyzer]
    MISSION --> OBS
    DEBUG --> OBS
    PER --> OBS
    RAW --> OBS
```

`io/status`의 publisher는 실행 환경별로 하나다.

- 실기체 launch: `arms_crsf_io_node`가 CRSF/FC telemetry 연결 상태를 발행한다.
- SITL launch: `sitl_bridge_node`가 PX4 MAVLink heartbeat/telemetry 상태를 발행한다.

`arms_control_node`는 status의 timeout이나 fault를 보고 RTL을 결정한다. I/O 노드는
상태를 관측·보고할 뿐 RTL 정책을 직접 결정하지 않는다.

### 2.1 패키지 경계

비행제어 통신 패키지명은 **`arms_flight_io`**를 권장한다. `telemetry`는 수신만 한다는
인상을 주지만 이 패키지는 RC 채널 송신도 담당하고, 단순 `arms_io`는 향후 카메라·디스크·
네트워크 I/O까지 포괄하는 지나치게 넓은 이름이기 때문이다.

```text
arms_operator       # ESP32/가상 조종기 → OperatorInput
arms_flight_io      # FC와의 통신 경계
  ├── arms_crsf_io_node    # 실기체: ELRS/CRSF 송수신 및 telemetry 정규화
  └── sitl_bridge_node     # SITL: MAVLink RC/telemetry 변환

arms_control         # 유도·상태 머신·failsafe 정책 → FlightCommand
arms_sim             # Gazebo referee, panel, sim_reset_manager
```

`arms_operator`로의 패키지 rename은 이 리팩터링과 동시에 하거나, 위험을 줄이기 위해
토픽 전환이 완료된 뒤 별도 커밋으로 해도 된다. 반면 `arms_crsf_io_node`와
`sitl_bridge_node`는 이번 전환에서 `arms_flight_io`로 옮긴다.

## 3. 토픽 계약: as-is → to-be

### 3.1 Operator, control, mission

| As-is | To-be | 주 publisher | 의미 |
| --- | --- | --- | --- |
| 구형 `/arms/command` (폐기) | `operator/input` | `arms_operator_hardware_node` | 조종자가 의도한 stick, arm, auto/manual, RTL 등 고수준 입력 |
| 없음 | `control/flight_command` | `arms_control_node` | 정규화된 최종 비행 명령. CRSF I/O와 SITL bridge가 기체별 RC/AUX·MAVLink로 변환 |
| `/arms/control_debug` | `control/debug` | `arms_control_node` | 제어 상태와 일반 디버그 값 |
| `/arms/ibvs_debug` | `control/ibvs_debug` | `arms_control_node` | image error, LOS rate, bbox, thrust 등 IBVS 값 |
| `/arms/debug_looming` | `control/looming_debug` | `arms_control_node` | looming 유도 디버그 값 |
| `/arms/mission_state` | `mission/state` | `arms_control_node` | IDLE, SEARCH, LOCK, TRACK, RTL 등 |

`operator/input`의 메시지는 기존 `sensor_msgs/Joy` 대신 목적이 드러나는
`arms_msgs/OperatorInput`으로 교체한다. `control/flight_command`는 `Joy`나 실제 RC
채널 배열이 아닌 `arms_msgs/FlightCommand`를 사용한다. 양자는 이름과 메시지 타입 모두
달라야 한다.

### 3.2 I/O 및 기체 telemetry

| As-is | To-be | 주 publisher | 의미 |
| --- | --- | --- | --- |
| 없음 | `io/status` | 실기체: `arms_crsf_io_node`<br>SITL: `sitl_bridge_node` | CRSF/FC 또는 PX4 telemetry 링크 상태, 마지막 수신 시각, fault 원인 |
| `/arms/crsf_rx` | `io/crsf/rx` | `arms_crsf_io_node` | 실기체 수신 CRSF 원시 프레임 및 기록용 진단 정보 |
| `/arms/crsf_tx` | `io/crsf/tx` | `arms_crsf_io_node` | 실기체 송신 CRSF 원시 프레임 및 기록용 진단 정보 |
| `/arms/gps` | `vehicle/gps` | 실기체: `arms_crsf_io_node`<br>SITL: `sitl_bridge_node` | 위치·속도·고도·GPS fix·위성 수 |
| `/arms/attitude` | `vehicle/attitude` | 실기체: `arms_crsf_io_node`<br>SITL: `sitl_bridge_node` | roll, pitch, yaw |
| `/arms/battery` | `vehicle/battery` | `arms_crsf_io_node` | 전압, 전류, 잔량 등 배터리 telemetry |

`vehicle/*`는 환경에 독립적인 공개 계약이다. 실기체에서는 CRSF telemetry에서,
SITL에서는 PX4 MAVLink에서 같은 의미의 메시지로 변환해 발행한다.

### 3.3 Camera, perception, UI

| As-is | To-be | 의미 |
| --- | --- | --- |
| `/arms/image_raw` | `camera/image_raw` | 원본 카메라 영상 |
| `/arms/camera_info` | `camera/camera_info` | 카메라 intrinsics 등 `CameraInfo` |
| `/arms/image_raw/compressed` | `camera/image_raw/compressed` | 압축 원본 영상 |
| `/arms/image_raw/compressedDepth` | `camera/image_raw/compressedDepth` | image_transport 파생 토픽 |
| `/arms/image_raw/theora` | `camera/image_raw/theora` | image_transport 파생 토픽 |
| `/arms/detections` | `perception/detections` | control이 사용할 확정 검출 결과 |
| `/arms/detections_raw` | `perception/detections_raw` | 필터 전 또는 분석용 검출 결과 |
| `/arms/detector_status` | `perception/status` | detector 준비·성능·오류 상태 |
| `/arms/debug_image` | `perception/debug_image` | 검출 오버레이 영상 |
| `/arms/hsv_debug_image` | `perception/hsv_debug_image` | HSV segmentation 디버그 영상 |
| `/arms/ui_image` | `ui/image` | 오퍼레이터 UI 오버레이 영상 |
| `/arms/ui_image/compressed` | `ui/image/compressed` | 압축 UI 영상 |

### 3.4 SITL 전용 인터페이스

| As-is | To-be | 주체 | 의미 |
| --- | --- | --- | --- |
| `/arms/hit` | `sim/hit` | `arms_sim/referee` | SITL 명중 판정 이벤트 |
| `/arms/sitl/reset` 서비스 | `sim/reset` 서비스 | `sim_reset_manager` | PX4만 재시작하고 Gazebo는 유지하라는 요청 |
| `/arms/reset` 토픽 | `sim/reset_event` | `sim_reset_manager` | PX4 재기동 성공 뒤 control, referee, UI 등에 알리는 초기화 이벤트 |
| `/arms/viz/drone_path` | `sim/viz/drone_path` | SITL bridge | Gazebo truth 기반 기체 궤적 |
| `/arms/viz/target_path` | `sim/viz/target_path` | SITL bridge | Gazebo truth 기반 표적 궤적 |
| `/arms/viz/markers` | `sim/viz/markers` | SITL bridge | Gazebo truth 기반 RViz marker |

실기체에는 `sim/reset`과 `sim/reset_event`를 launch하지 않는다. Reset panel은
`arms_sim` 소속의 서비스 **client**이고, reset을 수행하거나 이벤트를 발행하지 않는다.

```text
arms_sim reset panel
  → sim/reset service
      → sim_reset_manager: PX4 재기동과 완료 확인
          → sim/reset_event topic
```

## 4. 노드 책임 변경

### `arms_operator_hardware_node`

- 실기체: ESP32 serial 패킷을 읽고 `OperatorInput`으로 변환한다.
- SITL: 가상 조종기 UI를 동일한 `OperatorInput`으로 변환한다.
- 링크 끊김 자체를 RTL로 바꾸지 않는다. control이 `OperatorInput`의 마지막 수신 시각을
  기준으로 timeout과 RTL을 판단한다.

### `arms_control_node`

- CRSF serial, ELRS, MAVLink, Gazebo를 직접 알지 않는다.
- `operator/input`, `perception/*`, `vehicle/*`, `io/status`를
  구독한다.
- operator input timeout, I/O fault, operator RTL 요청, target loss에 따라 미션 상태와
  RTL을 판단한다.
- `control/flight_command`만으로 FC 조작 의도를 내보낸다.

### `arms_crsf_io_node` — 실기체 전용

- CRSF/ELRS UART의 유일한 owner다.
- `control/flight_command`를 구독하여 Betaflight의 RC/AUX 매핑에 맞는 CRSF RC frame을 송신한다.
- 수신 CRSF telemetry를 `vehicle/gps`, `vehicle/attitude`,
  `vehicle/battery`로 정규화한다.
- 원시 통신 관찰용 `io/crsf/rx`, `io/crsf/tx`를 발행한다.
- CRSF/FC telemetry freshness를 `io/status`로 발행한다. 이 노드는 통신 상태를
  관측할 뿐 RTL 정책을 갖지 않는다.

### `sitl_bridge_node` — SITL 전용

- `control/flight_command`를 PX4의 MAVLink RC 입력·arm·mode·RTL 명령으로 변환한다.
- PX4 MAVLink telemetry를 `vehicle/gps`, `vehicle/attitude`로 정규화한다.
- PX4 heartbeat/telemetry freshness를 `io/status`로 발행한다.
- PX4 재시작은 직접 판단하지 않고 `sim_reset_manager`의 명령을 수행한다.

## 5. 메시지 초안

정확한 필드와 단위는 구현 전 별도 message review에서 확정한다. 아래는 책임을 드러내는
최소 형태다.

```text
OperatorInput
  header, roll, pitch, throttle, yaw
  arm_switch, auto_switch, rtl_switch, launch_request, input_lost

FlightCommand
  header, roll, pitch, throttle, yaw
  arm, manual_mode, rtl_request
  # -1..1 또는 throttle 0..1의 기체 비종속 명령. RC 채널값은 담지 않는다.

IoStatus
  header
  source                        # CRSF 또는 PX4
  transport_connected
  vehicle_telemetry_connected
  last_rx_age_ms, last_vehicle_telemetry_age_ms
  fault_reason
```

SITL에서 의미 없는 field는 0이나 false로 꾸며내기보다 `valid`/source 표시를 두어
명시적으로 unavailable로 표현한다.

## 6. 단계별 전환 계획

### Phase 0 — 기준선 고정 및 관측 준비

코드를 옮기기 전에 현재 기준 브랜치에서 다음을 보관한다.

- SITL 정상 비행 1회와 arm, manual/auto 전환, RTL, target loss의 기대 동작을 기록한다.
- `ros2 node list`, `ros2 topic list -t`, 핵심 토픽의 `ros2 topic info -v`를 캡처한다.
- control 입력·출력과 CRSF/MAVLink 경계를 표로 확정한다. 이 단계에서는 rename이나
  package 이동을 하지 않는다.

**통과 기준:** 기준 브랜치에서 동일 launch와 파라미터로 재현 가능한 SITL 비행 로그가 있다.

---

### Track A — control 통신 분리

이 track의 목적은 control의 환경 의존성을 없애는 것이다. camera, perception, mission,
debug, telemetry의 기존 토픽명은 바꾸지 않는다. 새 boundary topic 세 개만 사용한다.

#### A1. 메시지와 경계 계약만 추가

- `OperatorInput`, `FlightCommand`, `IoStatus` 메시지를 추가한다.
- `operator/input`, `control/flight_command`, `io/status`의 필드 단위·범위·QoS를 확정한다.
- 아직 기존 노드의 publisher/subscriber나 launch는 바꾸지 않는다.

**검증:** 전 워크스페이스 build, 메시지 interface 확인, 기존 SITL 회귀 없음.

#### A2. operator 입력을 control에서 분리

- ESP32 serial 처리와 가상 조종기 처리를 `arms_operator`로 분리한다.
- 두 입력 노드는 모두 `operator/input`만 발행한다.
- control은 이 토픽을 받아 기존과 같은 arm, manual/auto, RTL, 300 ms input timeout을
  판단한다. ESP32 UART 직접 접근은 이 단계에서 control에서 제거한다.
- control의 기존 기체 출력 경로와 camera/perception/mission/debug 토픽명은 유지한다.

**검증:** SITL GUI와 ESP32 각각에서 스틱·arm·모드 전환·입력 단절 RTL이 기준선과 같은지
확인한다. `operator/input` publisher는 launch당 정확히 하나여야 한다.

#### A3. FlightCommand를 shadow 관측으로 검증

- control이 기존 출력과 동일한 의미의 `control/flight_command`를 추가 발행한다.
- 이 단계의 `FlightCommand`는 **관측 전용**이다. 새 I/O 어댑터를 FC에 연결하지 않으므로
  RC를 두 경로가 동시에 보내지 않는다.
- 기존 command/CRSF 출력과 `FlightCommand`를 rosbag으로 나란히 비교해 축 부호,
  throttle 범위, arm, manual/auto, RTL edge가 일치하는지 확인한다.

**검증:** 실제 FC 또는 PX4에 새 명령을 보내지 않은 상태에서, 비교 로그가 전 구간 일치한다.
불일치가 있으면 A2/A3에서 수정하며 다음 단계로 진행하지 않는다.

#### A4. SITL I/O 어댑터 전환

- `sitl_bridge_node`가 `control/flight_command`만 구독하도록 구현한다.
- MAVLink RC/arm/mode/RTL 변환과 PX4 heartbeat 관측을 이 노드로 옮긴다.
- control의 SITL/MAVLink 직접 코드를 제거한다. 기존 GPS·attitude 토픽 이름은 이 단계에서
  유지하고, `io/status`만 새로 발행한다.
- launch에서는 기존 PX4 제어 publisher를 제거한 뒤 bridge 하나만 활성화한다.

**검증:** SITL에서 auto/manual, arm/disarm, RTL, PX4 재기동 후 reconnect, input timeout을
반복한다. PX4로 향하는 command publisher는 bridge 하나여야 한다.

#### A5. 실기체 CRSF I/O 어댑터 전환

- `arms_flight_io` 패키지에 실기체 CRSF adapter를 둔다.
- adapter는 `FlightCommand → CRSF RC/AUX` 변환, CRSF telemetry 수신, raw CRSF 기록,
  `io/status` 발행만 담당한다. RTL 정책은 넣지 않는다.
- control에서 CRSF parser, serial port, ELRS mapping 코드를 제거한다.
- GPS·attitude·battery의 기존 토픽명은 유지한다. `vehicle/*` rename은 Track B의 일이다.

**검증:** 벤치에서 arm/disarm·manual/auto·RTL AUX, ESP32 300 ms 단절, CRSF telemetry
freshness를 확인한다. 프로펠러를 제거한 상태에서 먼저 수행한다.

#### A6. 통신 분리 안정화와 정리

- SITL과 실기체 모두에서 control source에 UART device, CRSF, MAVLink, Gazebo 관련 코드가
  남지 않았는지 확인한다.
- `arms_operator`, `arms_flight_io`, `arms_control`의 package 경계만 문서화한다.
- 이 시점에는 UI, recorder, Flight Analyzer의 기존 telemetry/camera/perception 이름을
  그대로 유지한다. 기능 변경 없이 안정 비행을 먼저 확보한다.

**Track A 종료 기준:** control은 `operator/input`·기존 perception/telemetry·`io/status`를
입력으로 받고 `control/flight_command`와 기존 mission/debug를 출력한다. SITL과 실기체
비행 검증이 모두 끝난 상태여야 한다.

---

### Track B — 전체 토픽명·namespace 리팩터링

Track A가 안정화된 뒤에만 진행한다. 이 track에서는 메시지 의미나 FC 변환을 바꾸지 않고,
Section 3의 as-is → to-be 이름만 launch 단위로 전환한다.

#### B1. 카메라와 perception 도메인

- `camera/*`, `perception/*`으로 전환한다.
- video, detection, control subscriber, UI의 해당 구독을 같은 변경에서 맞춘다.
- image transport 파생 토픽도 base 이름과 함께 바꾼다.

**검증:** camera frame rate, detection rate, bbox 좌표와 control 입력이 기준선과 같은지
비교한다. 이 단계에서는 유도 수식이나 gain을 수정하지 않는다.

#### B2. control·mission·debug 도메인

- 남아 있는 `control/debug`, `control/ibvs_debug`, `control/looming_debug`,
  `mission/state`를 전환한다. `control/flight_command`는 Track A에서 이미 전환되어 있다.
- UI의 상태 표시와 PlotJuggler 레이아웃을 함께 갱신한다.

**검증:** 상태 머신 전이, UI 상태, debug 값이 이전 로그와 같은 의미·단위인지 확인한다.

#### B3. vehicle·I/O telemetry 도메인

- 기존 GPS, attitude, battery, CRSF raw 토픽을 `vehicle/*`, `io/crsf/*`, `io/status`로
  전환한다.
- 실기체 CRSF adapter와 SITL bridge가 같은 `vehicle/*` 계약을 발행하게 한다.

**검증:** 실기체와 SITL 각각에서 GPS fix, attitude, battery, I/O link 상태의 type·publisher
수·갱신률을 비교한다.

#### B4. SITL 전용 API

- `sim/hit`, `sim/reset`, `sim/reset_event`, `sim/viz/*`로 전환한다.
- panel, referee, launcher, reset manager, RViz를 같은 launch에서 전환한다.

**검증:** Gazebo를 유지한 PX4 reset, reset event 1회 발행, hit → RTL, RViz marker/path를
SITL에서 확인한다. 실기체 launch에는 sim node와 sim topic이 없어야 한다.

#### B5. recorder, Flight Analyzer, 문서 cutover

- recorder의 녹화 목록과 manifest에 최종 토픽 목록·`topic_schema_version`·namespace를
  기록한다.
- `scripts/flight_analyzer`의 bag reader, graph binding, 3D telemetry binding, UI 라벨을 최종
  계약으로 전환한다.
- `DESIGN.md`, SETUP 문서, launch 설명의 토픽 표와 mermaid를 최종 구조로 갱신한다.
- 구형 bag 지원을 넣지 않고, schema version이 맞지 않는 bag은 명확히 거부한다.

**Track B 종료 기준:** Section 3의 to-be 표가 실제 `ros2 topic list -t`와 일치하고,
새 rosbag 하나를 recorder와 Flight Analyzer로 끝까지 열 수 있다.

### 공통 변경 규칙

- old/new 토픽을 장기간 dual publish하지 않는다. 특히 RC/AUX·arm·RTL은 launch당
  publisher 하나만 허용한다.
- 단, A3의 `FlightCommand`는 FC에 연결되지 않은 shadow 관측이므로 안전하다.
- 각 phase는 별도 커밋과 별도 SITL 검증 로그를 남긴다. 제어 gain, 물리량, 모델 변경은
  I/O 또는 rename phase와 같은 커밋에 넣지 않는다.
- 문제 발생 시 마지막으로 통과한 phase로만 되돌린다. 두 track의 변경을 섞어서 원인을
  상쇄시키지 않는다.

## 7. 완료 기준

- Track A 완료 전에는 아래 최종 조건 중 토픽명 관련 조건을 요구하지 않는다. Track A의
  성공 기준은 **통신 경계 분리 후 기준선과 같은 비행 동작**이다.
- control source에 CRSF, UART device, MAVLink, Gazebo 조건 분기가 없다.
- 실기체와 SITL 모두 control이 같은 `FlightCommand` 메시지를 출력한다.
- UI, recorder, Flight Analyzer가 `vehicle/*`, `io/status`, `mission/state`,
  `control/*`만으로 동일한 화면과 로그를 만든다.
- ESP32 입력 단절은 control이 `OperatorInput` timeout으로, FC/PX4 telemetry 단절은
  `io/status`로 감지하여 정의된 시간 기준으로 RTL을 요청한다.
- SITL reset은 Gazebo를 유지한 채 PX4를 재기동하고, 성공했을 때만
  `sim/reset_event`를 한 번 발행한다.
- `DESIGN.md`, recorder manifest, ARMS Flight Analyzer가 모두 같은
  `topic_schema_version`과 상대 토픽 목록을 참조한다.
- 구형 토픽만 들어 있는 rosbag은 새 Flight Analyzer에서 지원 대상이 아니다.
