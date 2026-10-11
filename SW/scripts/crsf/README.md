# CRSF UART 진단 도구

실기체 어댑터 `arms_flight_io/crsf_io_node.py`와 같은 CRSF RC 프레임을 사용해 UART 배선과 baud를 ROS 없이 분리 점검한다. 정상 운용은 이 스크립트가 아니라 `arms_crsf_io_node`가 담당한다.

| 파일 | 용도 |
| --- | --- |
| `crsf.py` | RC 프레임 인코더/디코더 |
| `crsf_send.py` | 안전한 중립ㆍsweepㆍARM 프레임 송신 |
| `crsf_dump.py` | TX/RX 루프백에서 프레임과 CRC 확인 |

현재 ARMS CRSF 링크 기준값은 **400000 baud**이며, 채널은 AETR + AUX다.

```text
CH1 roll   CH2 pitch   CH3 throttle   CH4 yaw
CH5 arm    CH6 auto/manual   CH7 RTL/GPS Rescue   CH8 unused
```

프로펠러를 제거하고 Jetson TX와 RX를 루프백한 뒤 아래 두 명령을 각각 실행한다.

```bash
python3 crsf_dump.py --baud 400000
python3 crsf_send.py --baud 400000 --mode sweep
```

CRC 오류가 없고 roll/pitch 값이 변하면 UART 경로는 정상이다. `--mode arm`은 CH5를 토글하므로 FC가 연결된 상태에서는 사용하지 않는다. 스로틀은 `--allow-throttle` 없이는 항상 최저값이다.
