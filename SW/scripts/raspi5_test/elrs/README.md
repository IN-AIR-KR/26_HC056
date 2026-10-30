# ELRS UART 조종 채널 모니터 (수신 전용)

`elrs_monitor.py`는 UART형 ELRS **수신기**의 CRSF 출력에서 16개 채널을 읽어 표시한다.
카메라·Hailo·ROS는 필요 없다. 무선 송신 모듈, SPI형 수신기, SBUS/inverted/half-duplex,
MAVLink 설정의 출력에는 사용하지 않는다. CRSF sync `0xC8`의 `0x16` 채널 프레임과
`0x14` Link Statistics를 해석한다. CRSFv3 `0x17` subset 채널 프레임은 해석하지 않고
타입 카운트만 표시한다.

이 예제는 시리얼 데이터를 송신하지 않는다. FC/모터/자동 제어와 연결하지 않는다.

## 1. 배선

전원을 끈 상태에서 수신기 TX → Pi RX, GND를 연결한다.

| ELRS 수신기 | Raspberry Pi 5 |
|---|---|
| TX (CRSF 출력) | GPIO15 / **물리 10번**, UART0 RX |
| GND | **물리 6번**, GND |
| 전원 | 수신기 모델 지정 전압으로 별도 확인 |
| RX | **이번 읽기 전용 시험에서는 연결하지 않음** |

Pi TX(물리 8번)는 연결할 필요가 없다. 전원 5V와 UART 신호 3.3V는 별개다.
Pi GPIO에 5V UART 신호를 넣지 않는다. 물리 핀 번호와 GPIO 번호를 혼동하지 않는다.
GPIO14/15를 사용하는 HAT/다른 앱과 동시에 사용할 수 없다.

## 2. Pi UART0 준비

이미 활성화했으면 중복 설정하지 않는다.

```bash
dtoverlay -h uart0-pi5
sudo nano /boot/firmware/config.txt
```

Pi 5용 설정이 모든 모델에 적용되는 `[all]` 구역에 있도록 확인한다:

```ini
[all]
dtoverlay=uart0-pi5
```

`sudo raspi-config` → Interface Options → Serial Port에서 **로그인 셸 No / 시리얼
하드웨어 Yes**를 선택한다. `/boot/firmware/cmdline.txt`의 `console=serial0,...` 또는
`console=ttyAMA0,...`처럼 해당 포트를 콘솔로 점유하는 옵션이 남아 있으면 제거한다.
다른 부팅 옵션은 유지하고 파일은 한 줄로 유지한다. 변경 후:

```bash
sudo reboot
```

재접속 후:

```bash
ls -l /dev/ttyAMA0
pinctrl get 14-15
```

Pi 5의 `/dev/serial0`은 별도 디버그 UART를 가리킬 수 있으므로 `/dev/ttyAMA0`을
명시한다. 사용자 권한 오류가 있으면:

```bash
sudo usermod -aG dialout "$USER"
```

그룹 적용을 위해 완전히 로그아웃 후 다시 로그인한다. UART2를 별도로 활성화해 쓴다면
포트·핀을 해당 설정에 맞게 바꾼다. **호스트에서 UART를 활성화해야 하며 Docker가
핀 설정을 대신하지 않는다.**

## 3. 호스트에서 먼저 실행

Pi에서 ELRS 전용 폴더로 이동한 뒤:

```bash
cd ~/ARMS/SW/scripts/raspi5_test/elrs
```

```bash
sudo apt install python3-serial
python3 elrs_monitor.py --port /dev/ttyAMA0
```

기본 속도는 수신기 CRSF UART의 일반적인 `420000` baud, 8N1이다. 수신기 Web UI에서
출력 프로토콜/속도를 바꿨다면 실제 설정에 맞춰 `--baud`를 지정한다.
조종기와 수신기가 바인딩된 상태에서 스틱·스위치를 움직여 CH01~CH16 변화를 확인한다.
표시는 기본 5Hz이며 수신 처리는 별도로 계속 수행한다. Ctrl+C로 종료한다.

30초 JSONL 시험과 로그 저장:

```bash
python3 elrs_monitor.py --port /dev/ttyAMA0 --seconds 30 \
  --json --log outputs/elrs.jsonl
```

로그는 **표시 주기별 스냅샷**이며 모든 프레임을 기록하는 원시 캡처가 아니다.
스위치가 반드시 특정 AUX에 해당한다고 가정하지 않는다. 채널 이름은 CH01~CH16만
사용한다. `approx_us`는 `1500 + (raw - 992) × 0.625`의 표시용 환산이며 실제 PWM
출력이 아니다. raw 992는 약 1500μs이고 종점은 조종기 설정에 따라 달라진다.

## 4. Docker 실행 (독립 이미지)

Hailo 이미지를 다시 컴파일하지 않고 작은 Ubuntu 22.04 ARM64 이미지로 실행한다:

```bash
docker compose build elrs
docker compose run --rm elrs
```

`.env`에서 필요하면 `ELRS_PORT=/dev/ttyAMA0`을 지정한다. 이미 `.env`가 있다면
`.env.example` 전체를 덮어쓰지 말고 필요한 줄만 추가한다. 컨테이너 내부 이름은
`/dev/elrs`로 고정이다. 기본 화면은 TTY에서 표이며, 리다이렉트 시 JSONL이다.

```bash
docker compose run --rm elrs python3 elrs_monitor.py \
  --port /dev/elrs --seconds 30 --json --log /outputs/elrs.jsonl
```

호스트와 Docker 모니터를 동시에 켜지 않는다. 포트는 독점으로 연다.
카메라 publisher와 GUI 권한 설정은 이 시험에 필요 없다.
이 폴더의 Compose는 ELRS 서비스만 포함한다. 계속 실행하려면
`docker compose up elrs`를 사용할 수 있다.

## 5. 상태 해석

| 표시 | 의미 |
|---|---|
| WAITING | 아직 정상 채널 프레임을 한 번도 받지 못함 |
| RC_STREAM | 최근 500ms 안에 CRC가 맞는 채널 프레임을 받음 |
| STALE | 마지막 채널 프레임 이후 500ms 초과. 채널값을 `----`/null로 표시 |
| rc_hz | 실제 디코딩한 채널 프레임의 초당 수신 수. 화면 갱신률과 다름 |
| crc_errors | 후보 프레임 CRC 불일치. 배선/속도/노이즈 확인 |
| link_age_ms | 표시된 Link Statistics가 얼마나 오래된 정보인지 |

500ms는 **이 모니터의 표시 기준**이며 FC failsafe를 설정하지 않는다.
다른 텔레메트리만 들어와도 채널 수신 타임아웃은 갱신하지 않는다.
**RC_STREAM이 RF 연결의 보장은 아니다.** 수신기 설정이 마지막 채널을 반복하면 UART
프레임은 계속 올 수 있다. 별도로 LQ와 그 데이터의 나이를 보고 조종기 링크를 끊었을 때
관찰한다. Link Statistics가 없으면 RF 상태는 이 도구만으로 확정하지 않는다.
장치 분리·I/O 오류는 오류와 함께 종료한다. 자동 재연결이나 명령 출력은 하지 않는다.

## 문제 해결

- bytes=0: UART 핀/전원/공통 GND/바인딩/포트 선택 확인.
- bytes는 늘지만 CRC/타입만 이상함: CRSF 출력 모드와 baud, 3.3V 신호 레벨 확인.
- `0x17`만 보임: subset 프로토콜은 미지원. 이 예제가 지원하는 출력과 구분한다.
- Permission denied: dialout 그룹과 재로그인, Docker 장치 전달 확인.
- Device busy: 콘솔/getty 또는 다른 모니터가 해당 UART를 점유하는지 확인.

## 오프라인 검증

```bash
python3 -m unittest -v test_elrs_monitor.py
```

CRC 표준 체크값, 분할 프레임, 11비트 경계, 잘못된 CRC/길이에서 복구,
RC 중단 중 텔레메트리만 수신, 채널 재수신을 검증한다. 실제 ELRS/Pi 배선 검증은 아니다.

작성 환경에서 위 8개 테스트와 가상 시리얼(PTY, 115200 baud)의 실제 CLI 수신·로그·
타임아웃·복구 시험이 통과했고 모니터의 송신 바이트는 없었다. Compose 설정 검증도
통과했다. 현재 Docker 데몬에 연결할 수 없어 새 ELRS 이미지 빌드는 미검증이며,
Pi 하드웨어의 420000 baud와 실제 RF 연결도 별도 확인해야 한다.

참고: [CRSF 패킷 정의](https://github.com/crsf-wg/crsf/wiki/Packet-Types),
[ExpressLRS 배선](https://www.expresslrs.org/quick-start/receivers/wiring-up/),
[ExpressLRS Web UI](https://www.expresslrs.org/quick-start/webui/).
