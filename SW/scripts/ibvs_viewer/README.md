# ibvs_viewer — IBVS 비행 로그 웹뷰어

`auto_save:=true` 로 녹화한 `~/arms_flight_logs_ibvs/<일시>/` 기록의 비행 텔레메트리와
`/arms/ibvs_debug`를 브라우저에서 uPlot 그래프로 본다.
**ROS 불필요** — sqlite3 로 bag 을 직접 열고 `rosbags`(순수 파이썬)로 역직렬화한다.
젯슨에서 기록 폴더만 노트북으로 복사해 와서 봐도 된다.

## 실행

```bash
pip install -r SW/scripts/ibvs_viewer/requirements.txt
python3 SW/scripts/ibvs_viewer/server.py
```

- 브라우저가 열리면 **데이터 폴더를 선택**한다(기본 제안 `~/arms_flight_logs_ibvs`).
  `--data <폴더>` 로 주면 선택 화면을 건너뛴다.
- 젯슨에서 띄우고 노트북으로 볼 때: `--host 0.0.0.0` → `http://<젯슨IP>:5050/`
  (SSH 세션이면 브라우저 자동 열기는 생략된다).
- 포트 변경 `--port`, 자동 열기 끄기 `--no-browser`.

## 화면

- **사이드바**: 데이터 폴더 안의 기록 목록(최신순). 시작시각 · 길이 · 크기 · ibvs_debug 개수.
  `ibvs_debug 없음` 은 IbvsDebug 추가 전 녹화분. "변경"으로 폴더를 바꾼다. 상단 `☰` 버튼으로
  접거나 펼칠 수 있으며, 선택 상태는 브라우저에 저장된다.
- **그래프**: 먼저 검출 수/신뢰도·검출 지연, 미션/표적 오차, 최종 제어 출력, GPS 위치·속도,
  배터리, 기체 자세·CRSF 링크, 조종 입력, FC 송신 채널을 표시한다. 그 아래에는 `IbvsDebug.msg`의 계산 단계
  (①오차 → ②LOS각속도 → ③z1/배리어 → ④항별 기여 → ⑤스케줄링 → ⑥명령 vs 자세 → ⑦추력)
  패널이 나온다. 녹화에 없는 토픽의 패널은 숨긴다. 배경색 = 상태(LOCK/TRACK/…).
  - 마우스 호버: 모든 그래프의 커서/범례 값이 같은 시각으로 연동된다. 상단에 t 와 상태 표시.
  - 드래그 = 구간 확대(전 그래프 동시), 더블클릭 = 전체 구간으로 복귀.
  - 범례 클릭 = 시리즈 켜기/끄기.
  - 범례 = 필드명. 칩에 필드별 수식이 나온다(상단 "수식 숨기기"로 수식만 끔). 범례 이름에
    마우스를 올려도 수식이 뜬다. 같은 표가 `SW/docs/IBVS_CONTROL.md` 11절에 있다.
  - 패널 위 **필터 칩**([체크박스][필드명][수식])을 눌러 선을 켜고 끈다(범례 클릭과 연동, 패널별로 브라우저에 기억).
  - `gain_barrier`, `aim_gain`, `aim_term_*`, `*_share_*` 는 기록된 필드로 뷰어가 계산한 중간값
    (게인 곱한 값 · 항별 비중). 구 기록에도 나온다.
- **화면 영상 + 3D 뷰**: 우상단 `Graphs` / `Video + Graphs` 로 전환. `Video + Graphs` 는 왼쪽에
  녹화된 영상과 3D 비행 뷰, 오른쪽에 그래프가 나온다. 기본 영상은 UI overlay이고,
  UI overlay와 원본(`/arms/ui/image/compressed`, `/arms/camera/image_raw/compressed`) 중
  기록된 것을 고를 수 있다. Stream rate의 video는 원본 카메라, UI overlay는 합성 화면의
  각각의 기록 수신율이다.
  3D 뷰는 첫 GPS 위치를 원점으로 한 ENU 좌표(X=동, Y=상, Z=북)에서 GPS 경로와
  `/arms/attitude`의 roll/pitch/yaw 자세를 함께 표시한다. 그래프 호버와 영상 재생 시각에 동기화되며,
  3D 화면을 마우스로 드래그하면 시점을 회전하고, 휠/트랙패드 스크롤로 확대·축소할 수 있다.
  **회전/줌 중심** 토글로 GPS 원점 또는 해당 시각의 드론을 중심으로 선택한다.
  - 3D 뷰 아래에 풍선 위도·경도·고도를 입력하고 **풍선 위치 저장**을 누르면 빨간 풍선이 표시되며,
    해당 비행 폴더의 `manifest.json`에 `balloon` 값으로 저장된다.
  - `Play`: 영상 재생(▶/Space, 시크바, 0.25~4×). 재생 위치가 모든 그래프의 커서로 표시되고,
    그래프를 클릭하면 그 시각으로 이동한다.
  - `Follow cursor`: 그래프에 마우스를 올린 시각의 화면이 바로 나온다.
  - **MP4 다운로드**: 현재 선택한 compressed 영상 전체를 요청 시 H.264 MP4로 변환해 내려받는다.
    변환은 뷰어를 실행한 컴퓨터에서 이뤄지며 `ffmpeg`가 필요하다.
  - 영상 토픽이 없는 기록(SITL 구버전 녹화 등)은 영상 칸에 안내만 뜬다.
- **패널 추가**(맨 아래): `토픽 필드` 를 쉼표로 여러 개 입력하면 새 패널이 생긴다
  (예: `/arms/ibvs_debug barrier, /arms/crsf_rx roll_deg`). 토픽을 생략하면 ibvs_debug.

## 참고

- 커스텀 메시지는 레포의 `SW/arms_ws/src/arms_msgs/msg/*.msg` 로 해석한다. 녹화 이후 .msg
  필드가 바뀌었다면 그 토픽은 "역직렬화 실패"로 표시되고 건너뛴다.
- 영상은 프레임을 필요할 때 한 장씩 읽는다. MP4 다운로드는 클릭 시에만 `ffmpeg`로 변환한다.
- uPlot과 3D 렌더러(Three.js)는 CDN(jsdelivr)에서 받는다 — 처음 열 때 인터넷이 필요하다.
