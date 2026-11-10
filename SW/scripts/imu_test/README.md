# Raw IMU 확인

ARMS를 먼저 실행한 뒤, 별도 터미널에서 다음을 실행한다.

```bash
cd ~/ARMS/SW
source /opt/ros/humble/setup.bash
source arms_ws/install/setup.bash
python3 scripts/imu_test/imu_monitor.py
```

10초만 검사하고 성공·실패 코드까지 받고 싶으면 다음과 같이 실행한다.

```bash
python3 scripts/imu_test/imu_monitor.py --duration 10
```

확인 기준:

- `/arms/vehicle/imu_raw`가 수신되고 `frame=base_frd`가 표시된다.
- SITL은 약 50 Hz 이상, 실기체 MSP는 설정한 `request_rate_hz` 부근으로 표시된다.
- 기체를 roll/pitch/yaw 방향으로 하나씩 움직일 때 해당 `gyro` 축이 반응한다.
- 정지 상태에서 `|a|`가 약 `9.81 m/s²`다.

이 스크립트는 읽기 전용이다. FC에 명령을 보내거나 ARM 상태를 바꾸지 않는다.

## Yang delayed-EKF 확인

`control.guidance_mode: 2`로 실행하면 PID/PN 대신 Yang 관측기만 동작하며, 기체에는
추력·자세 명령을 내보내지 않는다. 표적을 한 번 검출한 뒤 다음을 실행한다.

```bash
python3 scripts/imu_test/yang_dkf_monitor.py
```

`/arms/control/yang_debug`가 약 IMU 수신률로 표시되고 `PASS`가 나오면, 영상 지연을
보정하는 DKF 상태 전파와 현재 시각 재전파가 동작한 것이다.
