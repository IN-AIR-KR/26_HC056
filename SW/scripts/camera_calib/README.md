# camera_calib — 체커보드 카메라 캘리브레이션

OpenCV 창에 카메라 영상을 띄우고 키로 체커보드를 캡처해 내부파라미터(fx, fy, cx, cy, 왜곡)를
계산한다. 결과는 `arms_control_node` 의 `control.camera.*` 형식 그대로 저장되어
`hw_overrides.yaml` 에 붙여넣으면 IBVS(mode 2)의 픽셀 → tan 좌표 변환에 쓰인다.
불러온 캘리로 **보정 전/후를 키로 바꿔 가며** 비교할 수 있다. ROS 불필요.

## 실행

```bash
pip install -r SW/scripts/camera_calib/requirements.txt   # 젯슨은 시스템 python3-opencv 로 충분
python3 SW/scripts/camera_calib/calibrate.py
```

- 기본 카메라는 실기체 캡처 동글(`/dev/v4l/by-id/usb-MACROSIL_AV_TO_USB2.0-video-index0`), 720x480.
  **스택(arms_video)이 카메라를 쓰고 있으면 안 열린다** — 스택을 내리고 실행.
- 다른 카메라: `--device 0`. 해상도: `--width/--height` (실기체 검출 해상도와 같게 둘 것).
- 체커보드: `--cols/--rows` = **안쪽 코너 수**(칸이 10x7 이면 9x6, 기본), `--square` 칸 크기[m].

| 옵션 | 용도 |
|---|---|
| `--model fisheye` | 왜곡이 강한 광각용 equidistant 모델 (기본 `plumb_bob`) |
| `--load <yaml>` | 시작부터 그 캘리로 보정해서 보기. 결과 파일 / `hw_overrides.yaml` 둘 다 됨 |
| `--images <폴더>` | 저장해 둔 캡처 사진으로 바로 재계산 (모델을 바꿔 다시 돌려볼 때) |
| `--out <폴더>` | 저장 위치 (기본 `SW/scripts/camera_calib/output/`, git 에 안 올라감) |

## 키

| 키 | 동작 |
|---|---|
| `SPACE` | 캡처 (체커보드가 검출된 프레임만 저장) |
| `d` | 마지막 캡처 삭제 |
| `k` | 캘리 계산 + 저장 (최소 10장) |
| `u` | 보정 ON/OFF |
| `s` | 나란히 보기 (원본 \| 보정) ON/OFF |
| `a` | 보정 화면 범위: 유효 영역 크롭 ↔ 원본 전체(검은 테두리) |
| `g` | 격자선 ON/OFF — 보정 후 화면 속 직선이 격자와 나란히 펴지는지 확인 |
| `l` | 캘리 파일 다시 불러오기 (`--load` 파일 또는 방금 계산한 결과) |
| `q` / `ESC` | 종료 |

화면 표시: 체커보드가 잡히면 코너가 그려지고 상태줄이 초록색이 된다. 이미 캡처한 코너는 파란 점으로
남아서 **화면 어디를 아직 안 덮었는지** 보인다. 자세한 메시지는 터미널에 나온다.

## 잘 찍는 법

- 15~25장. 보드를 화면 **가장자리·모서리**까지 고루 보내고(왜곡은 가장자리에서 크다),
  기울기(앞뒤/좌우 30~45°)와 거리를 다양하게.
- 흔들림·반사·보드 일부 가림은 피한다. 아날로그 영상은 노이즈가 많으니 보드를 잠깐 멈춘 뒤 캡처.
- 계산 후 터미널의 **오차 큰 캡처** 목록을 보고, 튀는 장이 있으면 지우고 다시 계산한다.
  RMS 는 대략 0.5px 이하면 양호(아날로그 캡처는 ~1px 까지도 흔함).

## 결과와 적용

`SW/scripts/camera_calib/output/<일시>/` 에 `camera.yaml` 과 캡처 사진(`images/`)이 저장된다
(`output/` 은 `.gitignore` 에 등록 — 적용할 값은 `hw_overrides.yaml` 에 옮겨서 커밋).

```yaml
arms_control_node:
  ros__parameters:
    control:
      camera:
        calibrated: true
        image_width: 720
        image_height: 480
        fx: 300.0022
        ...
        distortion_model: plumb_bob     # fisheye 모델이면 equidistant
        distortion: [k1, k2, p1, p2, k3]   # equidistant 면 [k1, k2, k3, k4]
```

`camera:` 블록을 `SW/arms_ws/src/arms_control/config/hw_overrides.yaml` 의 같은 위치에 덮어쓰고
노드를 재기동한다(캘리는 기동 시 1회만 읽음). 기동 로그에 `카메라 캘리 적용: ...` 이 찍히면 적용된 것.

## ⚠ 노드 호환 검사

계산/불러오기 때마다 `arms_control_node` 의 왜곡 역보정 알고리즘을 그대로 옮겨 **화면 전체에서
수렴하는지** 검사하고, 결과를 화면 상태줄과 터미널에 보여준다.

- plumb_bob 모델에 왜곡이 강하면(광각 FPV) 노드의 반복 계산이 **화면 가장자리에서 발산**한다.
  그 위치에선 노드가 계산하는 x_n, y_n(→ z1, 배리어)이 엉뚱한 값이 된다.
- 경고가 뜨면 `--model fisheye` 로 다시 계산해 본다(같은 사진으로: `--images <폴더> --model fisheye`).
  fisheye 쪽은 노드가 뉴턴법으로 풀어서 넓은 화각에서도 안정적이다.
