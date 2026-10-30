# Raspberry Pi 5 하드웨어 검증

각 폴더는 별도의 Docker 빌드·Compose·설정·출력 경로를 사용한다.

| 폴더 | 용도 | 실행 안내 |
|---|---|---|
| `camera_hailo/` | CSI 카메라, Hailo-8L 추론, Docker 미리보기 | [카메라/Hailo README](camera_hailo/README.md) |
| `elrs/` | ELRS UART 조종 채널 수신 모니터 | [ELRS README](elrs/README.md) |

## 카메라 / Hailo

```bash
cd ~/ARMS/SW/scripts/raspi5_test/camera_hailo
python3 camera_test.py --camera 0 --seconds 0 --publish shared/latest.frame
```

같은 폴더의 다른 터미널에서:

```bash
docker compose build camera
./gui.sh camera
# 모델이 준비됐다면:
./gui.sh hailo
```

## ELRS

```bash
cd ~/ARMS/SW/scripts/raspi5_test/elrs
python3 elrs_monitor.py --port /dev/ttyAMA0
```

또는 Docker로:

```bash
docker compose build elrs
docker compose run --rm elrs
```

## 기존 경로에서 옮길 때

- 실행 전에 해당 하위 폴더로 이동한다. 최상위에는 Compose 파일이 없다.
- 카메라 `models/`, `shared/`, `outputs/`는 `camera_hailo/` 아래로 이동했다.
- ELRS 로그는 `elrs/outputs/`를 사용한다. 기존 `elrs*.jsonl` 로그도 이 경로로 이동한다.
- Pi의 기존 `.env`는 자동으로 이동하지 않는다. 카메라/Hailo 관련 값은
  `camera_hailo/.env`, `ELRS_PORT`는 `elrs/.env`에 필요한 항목만 복사한다.
- `.env.example`을 복사할 때 기존 `.env`를 덮어쓰지 않는다. 상대 경로로 지정한
  공유 폴더 등은 새 작업 폴더 기준으로 확인한다.
- 이미지 이름은 유지하므로 소스 이동만으로 HailoRT를 다시 컴파일할 필요는 없다.
  빌드가 필요하면 각 폴더에서 실행한다.
