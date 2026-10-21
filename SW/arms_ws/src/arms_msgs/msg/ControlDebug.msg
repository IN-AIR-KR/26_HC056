# PID 제어기의 상세 스냅샷. arms_control_node가 제어 틱마다 발행한다.
# 적용되지 않은 단계(IDLE/SEARCH 등)의 값은 0/기본값이다.
std_msgs/Header header
string  state               # IDLE | SEARCH | LOCK | TRACK | RTL
bool    filter_active       # LOCK/TRACK: 오차 필터 갱신 중
bool    cmd_active          # TRACK: 자동 제어 명령 생성 중
bool    manual_out          # 수동/RTL 핸드오버(스틱 passthrough)
float64 dt                  # 제어 틱 간격[s]

# 오차와 시간기반 lead
float64 raw_err_x           # 원시 화면 정규화 오차 (bbox 중심 - 0.5)
float64 raw_err_y
float64 filt_err_x          # LPF 후 tan 카메라 좌표
float64 filt_err_y
bool    detection_held      # 검출 끊김 → 마지막 오차 유지 중
float64 err_dot_x           # LPF/클램프된 filt_err_x 미분 [tan/s]
float64 err_dot_y
float64 guidance_x          # PID roll 출력 (부호 전, 근접도 스케줄 전) [deg/s]
float64 guidance_y          # PID pitch 출력 (부호 전, 근접도 스케줄 전) [deg/s]

# 거리·접근 스케줄링
float64 loom_size           # bbox 크기 EMA (정규화)
float64 loom_rate           # bbox 크기 변화율 EMA
float64 tau                 # 충돌까지 시간[s] (미접근/무한대 = 9.99)
float64 closeness           # near-size 스케줄 진행도 0~1
float64 range_gain_scale    # 회전 게인 배율
float64 range_throttle_scale

# 최종 명령과 추력
float64 roll_rate_cmd       # sign·range_gain_scale 적용 후, clamp 전 [deg/s]
float64 pitch_rate_cmd
float64 roll_rate_out       # max_cmd_rate_dps clamp 후 실제 송신값 [deg/s]
float64 pitch_rate_out
bool    rate_saturated
float64 emag                # 조준 오프셋·deadzone 적용 후 오차 크기
float64 center_q            # 추력 정렬 품질 0~1
float64 throttle_up         # 근접도 스케줄 적용 최대 상승 추력
float64 tau_ease            # τ 기반 추력 감속 배율
float64 thrust              # 최종 추력 0~1
