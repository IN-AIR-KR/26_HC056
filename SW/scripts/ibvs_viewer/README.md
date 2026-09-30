# ibvs_viewer — IBVS 비행 로그 웹뷰어

`auto_save:=true` 로 녹화한 `~/arms_flight_logs_ibvs/<일시>/` 기록의 `/arms/ibvs_debug`
(+ `/arms/crsf_rx`, `/arms/mission_state` 등)를 브라우저에서 uPlot 그래프로 본다.
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
  `ibvs_debug 없음` 은 IbvsDebug 추가 전 녹화분. "변경"으로 폴더를 바꾼다.
- **그래프**: `IbvsDebug.msg` 의 계산 단계(①오차 → ②LOS각속도 → ③z1/배리어 → ④항별 기여
  → ⑤스케줄링 → ⑥명령 vs 자세 → ⑦추력)대로 패널이 나온다. 배경색 = 상태(LOCK/TRACK/…).
  - 마우스 호버: 모든 그래프의 커서/범례 값이 같은 시각으로 연동된다. 상단에 t 와 상태 표시.
  - 드래그 = 구간 확대(전 그래프 동시), 더블클릭 / "전체 보기" = 원래대로.
  - 범례 클릭 = 시리즈 켜기/끄기. 패널 제목 클릭 = 접기(브라우저에 기억).
- **패널 추가**(맨 아래): `토픽 필드` 를 쉼표로 여러 개 입력하면 새 패널이 생긴다
  (예: `/arms/ibvs_debug barrier, /arms/crsf_rx roll_deg`). 토픽을 생략하면 ibvs_debug.

## 참고

- 커스텀 메시지는 레포의 `SW/arms_ws/src/arms_msgs/msg/*.msg` 로 해석한다. 녹화 이후 .msg
  필드가 바뀌었다면 그 토픽은 "역직렬화 실패"로 표시되고 건너뛴다.
- 영상 토픽은 읽지 않는다(영상은 `bag_visualize/bag_to_mp4.py`).
- uPlot 은 CDN(jsdelivr)에서 받는다 — 처음 열 때 인터넷이 필요하다.
