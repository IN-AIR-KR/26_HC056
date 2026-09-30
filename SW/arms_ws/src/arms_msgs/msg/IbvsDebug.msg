# IBVS(guidance_mode 2) 튜닝/디버깅용 스냅샷 — arms_control_node 가 제어 틱마다 발행(/arms/ibvs_debug).
#   구독자 있을 때만 발행(flight_recorder 가 구독하면 자동 on).
#   각 항은 control_loop() 계산 순서대로다. 적용되지 않은 단계(IDLE/SEARCH 등)는 0/기본값.
#   *_x = roll 축, *_y = pitch 축. 명령 단위는 각속도[deg/s] (ACRO, 각도 아님).
std_msgs/Header header
string  state               # IDLE | SEARCH | LOCK | TRACK | FIRE | RTL
int32   guidance_mode       # 0=PID 추적 / 1=PN / 2=IBVS
bool    filter_active       # LOCK/TRACK/FIRE: 필터(LPF/alpha-beta) 갱신 중
bool    cmd_active          # TRACK/FIRE: 실제 회전/추력 명령 생성 중
bool    manual_out          # 수동/RTL 핸드오버(스틱 passthrough → 아래 명령은 미사용)
float64 dt                  # 제어 틱 간격[s]

# ── ① 오차 (정규화 픽셀, ±0.5) ────────────────────────────────
float64 raw_err_x           # 상태머신 원시 오차 (bbox 중심 - 0.5)
float64 raw_err_y
float64 filt_err_x          # error_lpf_alpha LPF = los_x (IBVS 입력)
float64 filt_err_y
bool    detection_held      # 검출 끊김 → 마지막 값 유지 중

# ── ② LOS각속도 추정 ──────────────────────────────────────────
float64 err_dot_x           # 단순 미분+LPF+clamp (mode 0 리드용)
float64 err_dot_y
float64 losf_x              # alpha-beta 평활 LOS
float64 losf_y
float64 losf_x_dot          # alpha-beta LOS각속도 [1/s] (±3 clamp)
float64 losf_y_dot
float64 los_rate_x          # pn_los_clamp 적용 후 = IBVS LOS각속도항 입력
float64 los_rate_y

# ── ③ IBVS 기하 ───────────────────────────────────────────────
float64 x_n                 # tan 단위 이미지좌표 (캘리/hfov 환산)
float64 y_n
float64 theta_deg           # 광축-표적 이탈각 [deg]
float64 z1_raw              # 1 - cos(theta) (클램프 전)
float64 z1                  # kb*0.995 로 클램프 후
bool    z1_clamped          # z1_raw 가 배리어 한계에 걸림
float64 kb                  # control.ibvs_kb
float64 barrier             # z1 / (kb^2 - z1^2)
float64 dir_x               # 회전축 방향 x_n/N
float64 dir_y

# ── ④ IBVS 항별 기여 [deg/s] (부호·스케줄링 전) ────────────────
float64 barrier_term_x      # ibvs_gain * barrier * dir
float64 barrier_term_y
float64 center_term_x       # ibvs_center_gain * dir
float64 center_term_y
float64 los_rate_term_x     # ibvs_los_rate_gain * los_rate
float64 los_rate_term_y
float64 guidance_x          # 유도 출력 합 (mode 무관, 부호 전)
float64 guidance_y

# ── ⑤ 근접도/τ 스케줄링 ────────────────────────────────────────
float64 loom_size           # bbox 크기 EMA (정규화)
float64 loom_rate           # 크기변화율 ṡ EMA
float64 tau                 # 충돌까지 시간[s] (미접근/무한대 = 9.99 캡)
float64 closeness           # near_size 스케줄 진행도 0~1
float64 range_gain_scale    # 회전 게인 배율
float64 range_throttle_scale

# ── ⑥ 최종 회전 명령 [deg/s] ─────────────────────────────────────
float64 roll_rate_cmd       # sign·range_gain_scale 적용 후, clamp 전
float64 pitch_rate_cmd
float64 roll_rate_out       # max_cmd_rate_dps clamp 후 = 실제 송신값
float64 pitch_rate_out
bool    rate_saturated      # 둘 중 하나라도 max_cmd_rate_dps 에 걸림

# ── ⑦ 추력 ─────────────────────────────────────────────────────
float64 emag                # |filt_err|
float64 center_q            # 정렬 품질 0~1 (pursuit_center_boost=false 면 1)
float64 throttle_up         # 근접 스케줄 적용된 최대 상승 추력
float64 tau_ease            # τ 추력 감속 배율
float64 thrust              # 최종 추력 0~1

# ── 게인 스냅샷 (런타임 param 변경 추적용) ──────────────────────
float64 ibvs_gain
float64 ibvs_center_gain
float64 ibvs_los_rate_gain
