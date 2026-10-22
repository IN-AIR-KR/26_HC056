# A.R.M.S. PN Guidance

## 목적과 범위

`guidance_mode: 1`은 풍선 bbox의 bearing과 기체 attitude로 관성 좌표계 LOS
(line of sight)와 LOS-rate를 추정하고, true proportional navigation(PN)으로
목표 가속도·자세·추력을 만든다. 기본값 `guidance_mode: 0`은 기존 PID/ACRO
제어를 그대로 유지한다.

```text
perception/detections_raw + vehicle/attitude
  → camera ray → body FRD → world NED LOS
  → constant-rate LOS filter
  → PN lateral acceleration + LOS approach acceleration
  → gravity compensation → desired roll/pitch + throttle
  → FlightCommand(stabilized_mode=true)
  → flight_io → PX4/Betaflight Stabilized
```

이 구현은 상방 카메라와 고정 yaw를 전제로 한다. 화면 중앙 유지(center-holding),
FOV barrier, terminal 전용 제어는 포함하지 않는다.

## 입력과 시간 동기화

PN은 raw IMU/gyro 토픽을 직접 구독하지 않는다. 다만 영상 bearing에서 기체 회전을
제거하려면 **attitude estimate가 필수**다.

- `/arms/perception/detections_raw`: 원시 bbox와 원본 영상 `header.stamp`
- `/arms/vehicle/attitude`: roll/pitch/yaw attitude estimate

SITL은 PX4 MAVLink `ATTITUDE`, 실기체는 CRSF attitude telemetry를 이 공통 토픽으로
발행한다. 제어 노드는 detection의 촬영 시각에 attitude buffer를 보간한다. 해당 시각에서
자세가 `control.pn.max_measurement_age_sec`보다 오래되면 PN 추력과 명령을 0으로 둔다.

즉 새 IMU 하드웨어는 필요 없지만, FC의 attitude telemetry가 정상적으로 들어와야 한다.
실기체 CRSF attitude의 지연·축 정의가 SITL과 다르면 PN 성능과 부호가 달라질 수 있으므로
SITL에서 먼저 확인한다.

## 좌표계

- camera optical `C`: right, down, forward
- PX4 body `B`: FRD (forward, right, down)
- world `N`: NED (north, east, down)

정규화 bbox 중심 `(x, y)`에서 카메라 ray는 다음과 같다.

```text
u = x W, v = y H
l_C = normalize([(u-cx)/fx, (v-cy)/fy, 1])
l_N = R_NB R_BC l_C
```

`R_BC`는 `control.pn.camera.r_bc`에 둔다. 기본값은 상방 카메라의 optical forward가
기체 up (`-body z`)을 향하는 장착을 나타낸다. 이 행렬이나 roll/pitch/yaw 부호가 틀리면
기체 회전 중에도 관성 LOS가 움직여 PN이 잘못된 횡가속도를 낸다.

## PN과 접근 가속도

필터가 추정한 LOS와 그 미분을 각각 `l`, `l_dot`, bbox scale 기반 closing speed를 `Vc`라
하면 횡가속도는 다음과 같다.

```text
a_PN = N Vc l_dot
```

`N`은 `navigation_ratio`이며 3부터 시작한다. PN은 LOS에 수직인 성분만 만들므로,
표적 쪽으로 실제 접근시키는 성분은 별도로 더한다.

```text
a_guidance = approach_accel_mps2 * l + a_PN
```

bbox의 픽셀 크기와 알려진 풍선 지름 `D`로 거리를 근사한다.

```text
range ≈ D / angular_scale
Vc ≈ -d(range)/dt
```

SITL의 풍선 반지름은 0.25 m이므로 기준 `target_diameter_m`은 0.50 m이다. 실기체에서
다른 풍선을 쓰면 반드시 실제 지름으로 override한다.

## 중력 보상과 출력

NED에서 중력은 `[0, 0, +g]`다. 원하는 관성 가속도를 만들 specific thrust는 다음과 같다.

```text
f_des = a_guidance - [0, 0, g]
```

`f_des`의 방향에서 목표 roll/pitch를 복원하고, 크기는 hover throttle 비례로 collective
throttle로 변환한다. PN 자동 모드에서는 `FlightCommand.stabilized_mode=true`를 함께
보내므로 flight I/O가 FC를 Stabilized로 둔다. 이는 수동 모드가 아니며 operator 입력을
passthrough하지 않는다.

PID mode 0은 기존과 같이 ACRO rate 입력을 쓴다.

## 설정과 최초 SITL 절차

`control_params.yaml`의 주요 PN 설정:

| 항목 | 의미 | 초기값 |
| --- | --- | --- |
| `guidance_mode` | `0` PID/ACRO, `1` PN/Stabilized | `0` |
| `pn.navigation_ratio` | PN navigation ratio `N` | `3.0` |
| `pn.approach_accel_mps2` | LOS 방향 접근 가속도 | `5.0` |
| `pn.target_diameter_m` | 실제 풍선 지름 | `0.50` |
| `pn.max_lateral_accel_mps2` | PN 횡가속도 상한 | `12.0` |
| `max_angle_deg` | Stabilized roll/pitch 상한 | `55.0` |

1. PID mode 0에서 `vehicle/attitude`가 지속 발행되는지 확인한다.
2. `guidance_mode: 1`로 바꾸고, 정지·무추력 상태에서 기체를 회전시킨다.
   화면 bbox는 움직여도 관성 LOS-rate가 작아야 한다.
3. 작은 초기 offset과 낮은 `approach_accel_mps2`로 비행한다.
4. `N`은 3, 4, 5 순서로만 올리고, tilt/throttle 포화와 실제 miss를 함께 비교한다.

PN 유효성이 끊겼을 때 PID 출력으로 조용히 전환하지 않는다. Stabilized에서 중립 자세와
zero throttle을 보내므로, telemetry·timestamp·camera 장착 문제를 먼저 해결해야 한다.
