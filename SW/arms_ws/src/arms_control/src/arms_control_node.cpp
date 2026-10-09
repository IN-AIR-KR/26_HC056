#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "arms_control/crsf_output.hpp"
#include "arms_control/pid_controller.hpp"
#include "arms_control/servo_lock.hpp"
#include "arms_control/state_machine.hpp"
#include "arms_msgs/msg/crsf_telemetry.hpp"
#include "arms_msgs/msg/gps_telemetry.hpp"
#include "arms_msgs/msg/detection_array.hpp"
#include "arms_msgs/msg/ibvs_debug.hpp"
#include "arms_msgs/msg/mission_state.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

using namespace std::chrono_literals;

namespace arms_control {

class ArmsControlNode : public rclcpp::Node {
 public:
  ArmsControlNode() : Node("arms_control_node") {
    // ----------------------------------------------------------------
    // Declare & load parameters
    // ----------------------------------------------------------------
    declare_parameter("mission.detection_confidence_threshold", 0.32);
    declare_parameter("mission.lock_duration_sec", 1.0);
    declare_parameter("mission.detection_timeout_sec", 1.0);
    declare_parameter("mission.intercept_align_tol", 0.2);  // 종말 RTL 정렬 허용오차
    // 비전 looming(τ) 기반 종말 RTL — bbox 팽창률로 충돌 임박을 판정한다.
    declare_parameter("mission.tau_rtl_sec", 0.3);       // 충돌까지 시간(τ) 임계 [s]
    declare_parameter("mission.loom_s_min", 0.1);        // 종말 RTL 최소 bbox 크기(정규화)
    declare_parameter("mission.loom_size_alpha", 0.3);   // bbox 크기 EMA (지터 억제)
    declare_parameter("mission.loom_rate_alpha", 0.3);   // 크기변화율(ṡ) EMA

    declare_parameter("control.roll_pid.kp", 455.0);
    declare_parameter("control.roll_pid.ki", 0.0);
    declare_parameter("control.roll_pid.kd", 3.5);
    declare_parameter("control.pitch_pid.kp", 455.0);
    declare_parameter("control.pitch_pid.ki", 0.0);
    declare_parameter("control.pitch_pid.kd", 3.5);
    declare_parameter("control.track_throttle", 0.66);
    declare_parameter("control.lead_gain", 0.0);
    declare_parameter("control.roll_sign", 1.0);
    declare_parameter("control.pitch_sign", 1.0);
    // Mode 0 중력 처짐 보상: 양수면 카메라 중심을 표적보다 화면 위쪽으로 조준한다.
    declare_parameter("control.mode0_aim_above_offset", 0.03);
    declare_parameter("control.error_lpf_alpha", 0.25);
    declare_parameter("control.control_rate_hz", 50.0);
    declare_parameter("control.deadzone", 0.04);
    declare_parameter("control.deriv_lpf_alpha", 0.25);

    declare_parameter("mission.sitl_auto_launch", true);
    // 종말 RTL 소스 — 서로 독립적인 두 스위치. 둘 다 환경 무관하게 동작한다.
    //   hit_rtl_via_referee: 심판 /arms/hit(지상진실 접촉)를 받으면 RTL.
    //       SITL 전용(Gazebo 접촉 심판이 /arms/hit 발행). 실기체엔 발행자가 없어 false.
    //   looming_rtl_enabled: 비전 τ(bbox 팽창률)로 충돌 임박을 판정해 RTL.
    //   조합: SITL=(true,false) / 실기체 현재=(false,false) / 자동 종말=(false,true).
    declare_parameter("mission.hit_rtl_via_referee", true);
    declare_parameter("mission.looming_rtl_enabled", false);
    declare_parameter("mission.auto_launch_delay_sec", 0.5);

    // 자동 모드 CH5(FC arm)를 켜는 상태 목록. 자동 모드에선 arm 을 스위치와 분리해
    //   컨트롤 노드가 상태머신 상태로 결정한다. 기본=발사(TRACK)부터 무장.
    //   나중에 SEARCH 등에서 arm 하도록 바꾸려면 이 목록만 수정하면 된다.
    declare_parameter("control.auto_arm_states", std::vector<std::string>{"SEARCH", "LOCK", "TRACK", "RTL"});

    // 수동 arm pre-arm 안전 확인: arm 순간 스틱이 idle(throttle 최저 + roll/pitch/yaw 중앙)
    //   이어야만 실제 arm 되게 한다(급상승/오발 방지). 한 번 arm 되면 비행 중엔 무관.
    declare_parameter("control.prearm_check", true);
    declare_parameter("control.prearm_throttle_max", -0.85);  // throttle(axes[1]) 이 이하(최저)
    declare_parameter("control.prearm_stick_tol", 0.15);      // roll/pitch/yaw 중앙 허용오차

    // 발사 잠금장치 서보 (Jetson 하드웨어 PWM, sysfs). SITL 은 enabled=false.
    declare_parameter("servo.enabled", false);
    declare_parameter("servo.chip_path", std::string("/sys/class/pwm/pwmchip0"));
    declare_parameter("servo.channel", 0);
    declare_parameter("servo.period_ns", 20000000);    // 50Hz = 20ms
    declare_parameter("servo.lock_duty_ns", 1500000);  // 90°  = 1.5ms (LOCK 기본)
    declare_parameter("servo.open_duty_ns", 2500000);  // 180° = 2.5ms (OPEN 기본)

    declare_parameter("crsf.port", std::string("/tmp/crsf_tx"));
    declare_parameter("crsf.baud", 400000);
    // 이 시간[s] 내 CRSF 텔레메트리가 안 오면 ELRS 링크 끊김으로 판정(UI 표시/효과음).
    declare_parameter("crsf.telemetry_timeout_sec", 2.0);

    // ── 배터리 잔량(%) 계산 ────────────────────────────────────────────
    // 셀 수는 전압으로 자동 감지(cells=ceil(V/4.3), 1회 감지 후 래치). 잔량은:
    //   pct = (전압 - cells·empty_v) / (cells·(full_v - empty_v)), 0~100 clamp.
    declare_parameter("battery.cell_full_v", 4.2);       // 만충 시 셀당 전압[V]
    declare_parameter("battery.cell_empty_v", 3.5);      // 방전(0%) 셀당 전압[V]
    // 자동요격은 ACRO(각속도) 고정이다. crsf.max_angle_deg / control.acro_mode 와
    // Stabilized(각도) 출력 분기는 삭제됐다 -- 유도 출력이 각속도[deg/s]가 된 이상
    // 그 분기는 각속도를 각도로 잘못 해석할 뿐이다. 브리지 쪽 짝이던
    // autonomous_acro 도 같이 없앴다(CH6 low = ACRO 고정).
    declare_parameter("control.max_rate_dps", 400.0);  // 풀스틱 각속도[deg/s], PX4 MC_ACRO_*_MAX 와 일치
    // 추격 궤적(Pursuit): 정렬 기다리지 않고 조준하며 처음부터 상승.
    declare_parameter("control.hover_throttle", 0.32);  // 겨냥 안 될 때 최소 상승(호버)
    declare_parameter("control.pursuit_gate", 0.25);    // 오차 이 이하면 full 상승(공쪽으로 돌진)
    // pursuit_center_boost: true=중앙 정렬될수록 추력↑(center_q 스케일, 기존동작).
    //                       false=정렬 무관 상수 추력(track_throttle 고정) → "정렬시 추력증가" 비활성.
    declare_parameter("control.pursuit_center_boost", true);
    // 표적 속도추정(err_dot): LOS 미분+LPF. guidance_mode 0 의 리드(lead_gain)와 gain_shaping 이 공유.
    //   PN(mode 1)은 이 값을 안 쓰고 alpha-beta(pn_alpha/beta) 로 리드를 따로 만든다.
    declare_parameter("control.err_dot_alpha", 0.2);    // 표적 속도추정 LPF(클수록 민첩/노이즈↑)
    declare_parameter("control.err_dot_clamp", 0.6);    // 속도추정 스파이크 제한[1/s]
    // 각도 단계는 없다. PID/PN 이 곧바로 각속도[deg/s] 명령을 낸다.
    //   예전에는 PID -> '목표기울기[deg]' -> clamp(max_tilt) -> x att_p -> 각속도 였는데,
    //   자세 피드백이 없어서 그 '각도'는 이름뿐이었고 att_p 는 kp 와 직렬로 곱해지는
    //   두 번째 게인일 뿐이었다. att_p 는 kp/kd/pn_* 게인에 흡수했고(3.5배),
    //   max_tilt_deg 클램프는 아래 max_cmd_rate_dps 로 대체했다.
    declare_parameter("control.max_cmd_rate_dps", 227.5);
    // 중심 근처 게인 감쇠(gain_scale / pitch_scale) 를 켤지. 기본 false = 순수 P.
    //   true 면 아래 두 가지가 켜진다:
    //     · 느린 표적(|LOS각속도| < 0.2) 이 중심 근처일 때 게인 감쇠 (0.08→x0.5, 0.20→x0.75)
    //     · roll 오차가 크면 pitch 게인 감쇠 (0.12→x0.8, 0.06→x0.9) = 좌우 정렬 우선
    //   이 숫자들은 구 물리량 시절 지터를 막으려고 손으로 넣은 값이고 현재 기체에서
    //   재검증된 적이 없다. deadzone 과 함께 "중심 근처에서 일부러 약해지는" 장치라,
    //   표적이 중앙에 안 붙는 증상을 만들 수 있다. 먼저 끄고 순수 P 를 본 뒤,
    //   지터가 실제로 보이면 그때 켠다.
    declare_parameter("control.gain_shaping", false);  // 유도 출력 각속도 상한[deg/s]
    // 유도 방식: 0=기본 추적(현위치 추종 PID), 1=비례항법(PN, 시선각속도 제거/빠른표적),
    //            2=IBVS 공선유도(배리어 Lyapunov). 현재 기본값.
    declare_parameter("control.guidance_mode", 2);
    declare_parameter("control.pn_nav_gain", 175.0);    // PN 항법이득 N (LOS각속도→기울기). 못따라가면↑ 떨리면↓
    declare_parameter("control.pn_center_gain", 52.5);  // 표적 중심유지(화면 이탈 방지)
    declare_parameter("control.pn_alpha", 0.35);         // alpha-beta 위치이득 (표적 상태추정)
    declare_parameter("control.pn_beta", 0.02);         // alpha-beta 속도이득 (LOS각속도 추정)
    declare_parameter("control.pn_los_clamp", 1.5);     // PN 시선각속도 제한(0=무제한)

    // ── IBVS 공선유도 [mode 2 전용] ─────────────────────────────────────
    // 논문: Yang, Bai, She, Quan, "High-Speed Interception Multicopter Control
    //       by Image-based Visual Servoing" (arXiv:2404.08296) 식 (12),(13).
    //
    // 논문은 전방 카메라라 설계 LOS n_td(=광축)와 추력축 n_f 가 90° 어긋나 있고,
    // 그래서 n_td 정렬 명령이 yaw 성분을 갖는다(논문은 R_tilt 로 따로 제거).
    // 우리 기체는 카메라가 기체 +Z(=추력축)를 본다 → n_td = n_c = n_f 다. 결과:
    //   · z1 = 1 - n_f^T n_t 를 풀면 자세 R_b^e 가 양변에서 상쇄되어 사라진다.
    //       => z1 = 1 - cos(theta),  theta = 표적이 광축에서 벗어난 각도.
    //          픽셀과 화각만으로 계산된다. 자세/거리/속도/DKF 전부 불필요.
    //   · 회전축 e3 x (표적방향) 은 기체 z 에 수직 => yaw 성분이 구조적으로 0.
    //          (지금 yaw 를 중앙 고정하는 것과 일치. 억지로 뺄 필요가 없다.)
    //
    // 픽셀오차(정규화 ±0.5) → tan 단위 이미지좌표 변환에 화각이 필요하다.
    //   x_n = 2*tan(hfov/2) * error_x,   y_n = 2*tan(hfov/2)*aspect * error_y
    // SITL 기본값은 arms_sim 기체 SDF 의 upward_camera 스펙(hfov 0.7610rad = Phoenix 2 SE V2 + 8mm, 720x480).
    // ★ 실기체는 hw_overrides.yaml 의 control.camera.* (체커보드 캘리)가 이 근사를 대체한다.
    declare_parameter("control.ibvs_hfov_deg", 43.6028);   // 수평 화각[deg] (=0.7610 rad)
    declare_parameter("control.ibvs_aspect", 0.6666667);   // 세로/가로 = 480/720
    // 카메라 내부파라미터(체커보드 캘리). calibrated=true 면 위 hfov/aspect 근사 대신
    //   fx,fy,cx,cy + 왜곡 역보정으로 x_n,y_n 을 구한다. 실기체 전용 — hw_overrides.yaml.
    //   값은 image_width x image_height 해상도 기준 픽셀 단위(ROS camera_calibration 출력 그대로).
    //   기본값은 SITL upward_camera 와 동일한 이상적 핀홀(fx = 360/tan(0.3805) = 360/0.4 = 900, 왜곡 0).
    //   ※ 기동 시 1회만 읽는다(런타임 변경 미지원).
    declare_parameter("control.camera.calibrated", false);
    declare_parameter("control.camera.image_width", 720);
    declare_parameter("control.camera.image_height", 480);
    declare_parameter("control.camera.fx", 900.0);
    declare_parameter("control.camera.fy", 900.0);
    declare_parameter("control.camera.cx", 360.0);
    declare_parameter("control.camera.cy", 240.0);
    // "plumb_bob"(k1,k2,p1,p2[,k3]) | "equidistant"(fisheye k1,k2,k3,k4)
    declare_parameter("control.camera.distortion_model", std::string("plumb_bob"));
    declare_parameter("control.camera.distortion",
                      std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    // 배리어 kb: z1 이 이 값에 닿으면 게인이 발산한다 = "여기는 못 넘어간다".
    //   z1 = 1 - cos(theta) 이므로 kb 는 허용 최대 이탈각으로 환산된다.
    //   화면 모서리의 z1 = 0.099 (hfov 43.6°, 3:2) 이므로 그보다 약간 크게 잡는다.
    //   낮추면 표적을 더 중앙에 붙잡아두려 하고(공격적), 높이면 가장자리까지 허용.
    declare_parameter("control.ibvs_kb", 0.11);
    // 배리어항 계수[deg/s]. 유도 전체의 세기. 못 따라가면 ↑, 과조향이면 ↓.
    declare_parameter("control.ibvs_gain", 200.0);
    // 선형항 계수[deg/s]. 순수 배리어는 중앙 근처에서 theta^3 라 거의 0 이다
    //   (논문은 그 구간을 자세루프 omega2 가 메우는데 지금은 omega1 만 쓴다).
    //   0 으로 두면 논문 그대로의 순수 배리어 거동을 볼 수 있다.
    declare_parameter("control.ibvs_center_gain", 100.0);
    // LOS각속도항 계수 — ★횡단 표적 요격의 핵심★
    //   alpha-beta 가 추정한 시선각속도(losf_*_dot_)에 곱한다. 한 항이 두 일을 한다:
    //     · 표적이 화면을 가로지를 때  → 예측 조준(리드). PN 과 같은 역할.
    //     · 내가 과조향해 오차가 빨리 줄 때 → 브레이크(감쇠). PID 의 kd 와 같은 역할.
    //   배리어항만으로는 '표적이 지금 있는 곳'만 겨냥한다(pure pursuit). 횡단
    //   표적은 거리가 가까워질수록 LOS각속도가 1/거리로 치솟아서(3.6m/s 표적
    //   기준 10m 에서 21deg/s, 3m 에서 69deg/s) 게인을 아무리 올려도 종말에
    //   반드시 빗나간다 — 리드가 없으면 구조적으로 못 맞힌다.
    //   크기의 의미: 이 값 / (kx * 57.3) = 항법계수 N = 명령각속도 / LOS각속도.
    //     175 → N=1.0  "시선과 같은 속도로 돈다" = 카메라만 붙어 있고 리드 없음
    //     220 → N=1.2  약한 리드 (기본값 — 아래 ★주의★ 참고)
    //     350 → N=2.0  강한 리드. SITL 에서 심한 진동을 유발해 낮췄다.
    //
    //   ★주의★ 이 항은 losf_x_dot_(alpha-beta 로 추정한 '화면이 움직이는 속도')
    //   에 곱하는데, 그 속도에는 "표적이 움직여서"뿐 아니라 "내가 회전해서
    //   카메라가 같이 돌아가서"도 섞여 있다(los_x = filt_err_x_ 는 자세 보정이
    //   없는 순수 픽셀오차라서). 배리어항(z1)은 정적 정렬 오차라 자세가 기하학적
    //   으로 상쇄되지만, 이 시간미분항은 그 상쇄가 안 된다 — 내 회전을 표적
    //   움직임으로 착각해 더 돌고, 그게 더 큰 착각을 만드는 되먹임이 생길 수
    //   있다. 게인을 낮추면 이 되먹임의 증폭이 줄어 완화되지만 원인 자체가
    //   없어지진 않는다. 근본 해결은 자이로 각속도로 자기회전분을 빼는 것인데
    //   (=논문의 R_b^e 보정과 같은 역할, 지금은 미구현) 그건 더 큰 변경이다.
    //   흔들리면 ↓ (175까지, 그래도 흔들리면 pn_alpha/pn_beta 도 원인일 수 있음)
    //   종말에 뒤처지면(못 맞추면) ↑ (350 이상은 SITL 에서 진동 확인됨, 주의)
    declare_parameter("control.ibvs_los_rate_gain", 220.0);

    // ── 근접도 기반 게인/스로틀 스케줄링 ────────────────────────────────
    //   화면 속 표적 크기(loom_s_ema_, 클수록 가까움)를 기준으로 가까워질수록
    //   회전 게인과 최대 추력을 같이 낮춘다. 원근 때문에 같은 실제 오차라도
    //   가까울수록 화면 오차가 커져 PID/PN이 과민반응(오버슈트)하기 쉽고,
    //   접근 속도가 빠르면 그 과민반응을 보정할 시간도 부족해진다 — 그래서
    //   회전(각속도)과 병진(추력) 둘 다 같이 스케줄링해야 의미가 있다.
    declare_parameter("control.near_size_start", 0.15);   // 이 크기부터 스케줄링 시작
    declare_parameter("control.near_size_full", 0.35);    // 이 크기에서 최대(min_*)로 적용됨
    declare_parameter("control.min_gain_scale", 0.4);     // 가장 가까울 때 회전 게인 배율(하한)
    declare_parameter("control.min_throttle_scale", 0.6); // 가장 가까울 때 추력 상한 배율(하한)

    // ── τ(충돌까지 시간) 기반 추력 감속 ─────────────────────────────────
    // 위 near_size 스케줄링은 "화면 속 크기"만 보고 "접근 속도"는 모른다 — 같은
    // 크기라도 천천히 다가오면 여유, 빠르게 다가오면 임박인데 구분을 못 한다.
    // τ = 크기/팽창률 = 몇 초 뒤 충돌(update_looming() 에서 항상 계산됨, 스케일
    // 불변이라 표적 실제크기 불필요)은 접근 속도까지 반영하는 더 나은 신호다.
    //
    // 겨냥(guidance_mode 2)이 좋아질수록 접근이 빨라지는데, 추력은 여전히
    // "정렬 품질"만 보고 최대로 밀어붙인다(center_q, 위 pursuit 블록) → 정렬이
    // 좋아도 관성 때문에 표적을 살짝 지나칠 수 있다. τ 가 작아질수록(=곧 충돌)
    // 정렬 품질과 무관하게 추력을 hover 쪽으로 눌러, 지나치기 전에 속도를 줄이고
    // 정밀 정렬에 시간을 벌어준다.
    //   tau_ease_start 보다 멀면(τ 큼) 영향 없음(=지금과 동일).
    //   tau_ease_full  에서 최소 배율까지 눌린다.
    // 표적을 아예 못 볼 때/안 다가올 때는 τ=+inf 이므로 자동으로 영향 없음(1.0).
    declare_parameter("control.tau_ease_start", 1.0);   // 이 초 전부터 서서히 감속 시작[s]
    declare_parameter("control.tau_ease_full", 0.3);    // 이 초에서 최소 배율 적용[s] (tau_rtl_sec과 비슷한 크기가 자연스럽다)
    declare_parameter("control.tau_ease_min_scale", 0.3); // τ 임박시 추력 배율 하한 (0=완전 hover, 1=끔)

    // ----------------------------------------------------------------
    // State machine
    // ----------------------------------------------------------------
    SMParams sm_params;
    sm_params.confidence_threshold =
        get_parameter("mission.detection_confidence_threshold").as_double();
    sm_params.lock_duration_sec =
        get_parameter("mission.lock_duration_sec").as_double();
    sm_params.detection_timeout_sec =
        get_parameter("mission.detection_timeout_sec").as_double();
    sm_params.intercept_align_tol =
        get_parameter("mission.intercept_align_tol").as_double();
    sm_params.tau_rtl_sec =
        get_parameter("mission.tau_rtl_sec").as_double();
    sm_params.loom_s_min =
        get_parameter("mission.loom_s_min").as_double();
    loom_size_alpha_ = get_parameter("mission.loom_size_alpha").as_double();
    loom_rate_alpha_ = get_parameter("mission.loom_rate_alpha").as_double();

    auto log_fn = [this](const std::string& msg) {
      RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    };
    sm_ = std::make_unique<StateMachine>(sm_params, log_fn);

    // ----------------------------------------------------------------
    // PID controllers
    // ----------------------------------------------------------------
    // PID 의 출력 한계 = control.max_cmd_rate_dps. 각속도 천장은 이 하나뿐이다.
    //   예전에는 roll_pid.output_limit / pitch_pid.output_limit 가 따로 있었고
    //   값도 달랐다(157.5 vs 227.5). 순서상 PID 쪽이 항상 먼저 물려서 뒤의
    //   max_cmd_rate_dps 는 기본 추적 경로에서 죽은 값이었다 — 같은 일을 하는
    //   한계가 둘이면 어느 쪽이 실제로 걸리는지 아무도 모른다.
    max_cmd_rate_dps_ = get_parameter("control.max_cmd_rate_dps").as_double();
    gain_shaping_ = get_parameter("control.gain_shaping").as_bool();
    pid_roll_ = std::make_unique<PIDController>(
        get_parameter("control.roll_pid.kp").as_double(),
        get_parameter("control.roll_pid.ki").as_double(),
        get_parameter("control.roll_pid.kd").as_double(),
        max_cmd_rate_dps_);

    pid_pitch_ = std::make_unique<PIDController>(
        get_parameter("control.pitch_pid.kp").as_double(),
        get_parameter("control.pitch_pid.ki").as_double(),
        get_parameter("control.pitch_pid.kd").as_double(),
        max_cmd_rate_dps_);

    rkp_ = get_parameter("control.roll_pid.kp").as_double();
    rki_ = get_parameter("control.roll_pid.ki").as_double();
    rkd_ = get_parameter("control.roll_pid.kd").as_double();
    pkp_ = get_parameter("control.pitch_pid.kp").as_double();
    pki_ = get_parameter("control.pitch_pid.ki").as_double();
    pkd_ = get_parameter("control.pitch_pid.kd").as_double();

    track_throttle_ = get_parameter("control.track_throttle").as_double();
    lead_gain_ = get_parameter("control.lead_gain").as_double();
    roll_sign_ = get_parameter("control.roll_sign").as_double();
    pitch_sign_ = get_parameter("control.pitch_sign").as_double();
    mode0_aim_above_offset_ = std::clamp(
        get_parameter("control.mode0_aim_above_offset").as_double(), -0.5, 0.5);
    error_lpf_alpha_ = get_parameter("control.error_lpf_alpha").as_double();
    control_rate_hz_ = get_parameter("control.control_rate_hz").as_double();
    deadzone_ = get_parameter("control.deadzone").as_double();
    deriv_lpf_alpha_ = get_parameter("control.deriv_lpf_alpha").as_double();
    pid_roll_->set_deriv_alpha(deriv_lpf_alpha_);
    pid_pitch_->set_deriv_alpha(deriv_lpf_alpha_);

    sitl_auto_launch_ = get_parameter("mission.sitl_auto_launch").as_bool();
    hit_rtl_via_referee_ = get_parameter("mission.hit_rtl_via_referee").as_bool();
    looming_rtl_enabled_ = get_parameter("mission.looming_rtl_enabled").as_bool();
    auto_launch_delay_sec_ =
        get_parameter("mission.auto_launch_delay_sec").as_double();

    {
      const auto v = get_parameter("control.auto_arm_states").as_string_array();
      auto_arm_states_ = std::set<std::string>(v.begin(), v.end());
    }
    prearm_check_ = get_parameter("control.prearm_check").as_bool();
    prearm_throttle_max_ = get_parameter("control.prearm_throttle_max").as_double();
    prearm_stick_tol_ = get_parameter("control.prearm_stick_tol").as_double();

    max_rate_dps_ = get_parameter("control.max_rate_dps").as_double();
    hover_throttle_ = get_parameter("control.hover_throttle").as_double();
    pursuit_gate_ = get_parameter("control.pursuit_gate").as_double();
    pursuit_center_boost_ = get_parameter("control.pursuit_center_boost").as_bool();
    err_dot_alpha_ = get_parameter("control.err_dot_alpha").as_double();
    err_dot_clamp_ = get_parameter("control.err_dot_clamp").as_double();
    max_cmd_rate_dps_ = get_parameter("control.max_cmd_rate_dps").as_double();
    guidance_mode_ = static_cast<int>(get_parameter("control.guidance_mode").as_int());
    pn_nav_gain_ = get_parameter("control.pn_nav_gain").as_double();
    pn_center_gain_ = get_parameter("control.pn_center_gain").as_double();
    pn_alpha_ = get_parameter("control.pn_alpha").as_double();
    pn_beta_ = get_parameter("control.pn_beta").as_double();
    pn_los_clamp_ = get_parameter("control.pn_los_clamp").as_double();
    ibvs_hfov_deg_ = get_parameter("control.ibvs_hfov_deg").as_double();
    ibvs_aspect_ = get_parameter("control.ibvs_aspect").as_double();
    ibvs_kb_ = get_parameter("control.ibvs_kb").as_double();
    ibvs_gain_ = get_parameter("control.ibvs_gain").as_double();
    ibvs_center_gain_ = get_parameter("control.ibvs_center_gain").as_double();
    ibvs_los_rate_gain_ = get_parameter("control.ibvs_los_rate_gain").as_double();
    load_camera_params();
    update_ibvs_scale();
    near_size_start_ = get_parameter("control.near_size_start").as_double();
    near_size_full_ = get_parameter("control.near_size_full").as_double();
    min_gain_scale_ = get_parameter("control.min_gain_scale").as_double();
    min_throttle_scale_ = get_parameter("control.min_throttle_scale").as_double();
    tau_ease_start_ = get_parameter("control.tau_ease_start").as_double();
    tau_ease_full_ = get_parameter("control.tau_ease_full").as_double();
    tau_ease_min_scale_ = get_parameter("control.tau_ease_min_scale").as_double();
    crsf_out_ = std::make_unique<CrsfOutput>(
        get_parameter("crsf.port").as_string(),
        static_cast<int>(get_parameter("crsf.baud").as_int()));

    // ---- 발사 잠금장치 서보 ----
    ServoLock::Params servo_params;
    servo_params.enabled     = get_parameter("servo.enabled").as_bool();
    servo_params.chip_path   = get_parameter("servo.chip_path").as_string();
    servo_params.channel     = static_cast<int>(get_parameter("servo.channel").as_int());
    servo_params.period_ns   = static_cast<long>(get_parameter("servo.period_ns").as_int());
    servo_params.lock_duty_ns = static_cast<long>(get_parameter("servo.lock_duty_ns").as_int());
    servo_params.open_duty_ns = static_cast<long>(get_parameter("servo.open_duty_ns").as_int());
    servo_ = std::make_unique<ServoLock>(servo_params, log_fn);
    servo_->init();

    // ---- 배터리 잔량 계산 설정 ----
    telemetry_timeout_sec_ = get_parameter("crsf.telemetry_timeout_sec").as_double();
    battery_cell_full_v_  = get_parameter("battery.cell_full_v").as_double();
    battery_cell_empty_v_ = get_parameter("battery.cell_empty_v").as_double();

    // 런타임 파라미터 변경 콜백
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;
          bool pid_changed = false;
          for (const auto& p : params) {
            const std::string& n = p.get_name();
            if (n == "control.roll_sign")
              roll_sign_ = p.as_double();
            else if (n == "control.pitch_sign")
              pitch_sign_ = p.as_double();
            else if (n == "control.track_throttle")
              track_throttle_ = p.as_double();
            else if (n == "control.lead_gain")
              lead_gain_ = p.as_double();
            else if (n == "control.mode0_aim_above_offset")
              mode0_aim_above_offset_ = std::clamp(p.as_double(), -0.5, 0.5);
            else if (n == "control.error_lpf_alpha")
              error_lpf_alpha_ = p.as_double();
            else if (n == "control.deadzone")
              deadzone_ = p.as_double();
            else if (n == "control.max_rate_dps")
              max_rate_dps_ = p.as_double();
            else if (n == "control.hover_throttle")
              hover_throttle_ = p.as_double();
            else if (n == "control.pursuit_gate")
              pursuit_gate_ = p.as_double();
            else if (n == "control.pursuit_center_boost")
              pursuit_center_boost_ = p.as_bool();
            else if (n == "mission.hit_rtl_via_referee")
              hit_rtl_via_referee_ = p.as_bool();
            else if (n == "mission.looming_rtl_enabled")
              looming_rtl_enabled_ = p.as_bool();
            else if (n == "control.err_dot_alpha")
              err_dot_alpha_ = p.as_double();
            else if (n == "control.err_dot_clamp")
              err_dot_clamp_ = p.as_double();
            else if (n == "control.max_cmd_rate_dps") {
              max_cmd_rate_dps_ = p.as_double();
              pid_changed = true;   // PID 출력 한계도 이 값이라 같이 반영해야 한다
            }
            else if (n == "control.gain_shaping")
              gain_shaping_ = p.as_bool();
            else if (n == "control.guidance_mode")
              guidance_mode_ = static_cast<int>(p.as_int());
            else if (n == "control.pn_nav_gain")
              pn_nav_gain_ = p.as_double();
            else if (n == "control.pn_center_gain")
              pn_center_gain_ = p.as_double();
            else if (n == "control.pn_alpha")
              pn_alpha_ = p.as_double();
            else if (n == "control.pn_beta")
              pn_beta_ = p.as_double();
            else if (n == "control.pn_los_clamp")
              pn_los_clamp_ = p.as_double();
            else if (n == "control.ibvs_hfov_deg") {
              ibvs_hfov_deg_ = p.as_double();
              update_ibvs_scale();
            } else if (n == "control.ibvs_aspect") {
              ibvs_aspect_ = p.as_double();
              update_ibvs_scale();
            } else if (n == "control.ibvs_kb")
              ibvs_kb_ = p.as_double();
            else if (n == "control.ibvs_gain")
              ibvs_gain_ = p.as_double();
            else if (n == "control.ibvs_center_gain")
              ibvs_center_gain_ = p.as_double();
            else if (n == "control.ibvs_los_rate_gain")
              ibvs_los_rate_gain_ = p.as_double();
            else if (n == "mission.tau_rtl_sec")
              sm_->set_tau_rtl(p.as_double());
            else if (n == "mission.loom_s_min")
              sm_->set_loom_s_min(p.as_double());
            else if (n == "mission.loom_size_alpha")
              loom_size_alpha_ = p.as_double();
            else if (n == "mission.loom_rate_alpha")
              loom_rate_alpha_ = p.as_double();
            else if (n == "control.deriv_lpf_alpha") {
              deriv_lpf_alpha_ = p.as_double();
              pid_roll_->set_deriv_alpha(deriv_lpf_alpha_);
              pid_pitch_->set_deriv_alpha(deriv_lpf_alpha_);
            } else if (n == "control.roll_pid.kp") {
              rkp_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.roll_pid.ki") {
              rki_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.roll_pid.kd") {
              rkd_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.pitch_pid.kp") {
              pkp_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.pitch_pid.ki") {
              pki_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.pitch_pid.kd") {
              pkd_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.near_size_start") {
              near_size_start_ = p.as_double();
            } else if (n == "control.near_size_full") {
              near_size_full_ = p.as_double();
            } else if (n == "control.min_gain_scale") {
              min_gain_scale_ = p.as_double();
            } else if (n == "control.min_throttle_scale") {
              min_throttle_scale_ = p.as_double();
            } else if (n == "control.tau_ease_start") {
              tau_ease_start_ = p.as_double();
            } else if (n == "control.tau_ease_full") {
              tau_ease_full_ = p.as_double();
            } else if (n == "control.tau_ease_min_scale") {
              tau_ease_min_scale_ = p.as_double();
            }
          }
          if (pid_changed) {
            pid_roll_->set_gains(rkp_, rki_, rkd_, max_cmd_rate_dps_);
            pid_pitch_->set_gains(pkp_, pki_, pkd_, max_cmd_rate_dps_);
          }
          RCLCPP_INFO(get_logger(),
                      "param 적용: roll_sign=%.0f pitch_sign=%.0f rkp=%.1f "
                      "rkd=%.1f track_thr=%.2f",
                      roll_sign_, pitch_sign_, rkp_, rkd_, track_throttle_);
          return res;
        });

    // ----------------------------------------------------------------
    // ROS publishers
    // ----------------------------------------------------------------
    pub_state_ = create_publisher<arms_msgs::msg::MissionState>("/arms/mission_state", 10);
    pub_dbg_  = create_publisher<geometry_msgs::msg::Vector3>("/arms/control_debug", 10);
    // looming 튜닝용: x=τ[s](미접근/무한대는 9.99로 캡), y=bbox크기(EMA), z=크기변화율 ṡ
    pub_loom_ = create_publisher<geometry_msgs::msg::Vector3>("/arms/debug_looming", 10);
    // IBVS 튜닝/디버깅: 유도 각 항(오차→LOS각속도→z1/배리어→항별 기여→스케줄링→명령/추력)을
    //   한 메시지로 매 틱 발행(구독자 있을 때만). auto_save 녹화 대상.
    pub_ibvs_dbg_ = create_publisher<arms_msgs::msg::IbvsDebug>("/arms/ibvs_debug", 10);
    // CRSF 배터리 텔레메트리(0x08)를 UI 등에서 쓰도록 표준 메시지로 재발행한다.
    // CRSF 수신(RX) 텔레메트리 통합 발행: 배터리/자세(RPY)/수직속도/비행모드/링크통계 → /arms/crsf_rx.
    //   (배터리는 별도 /arms/battery 없이 이 토픽으로만 나간다. UI 도 이걸 구독.)
    pub_crsf_rx_ = create_publisher<arms_msgs::msg::CrsfTelemetry>("/arms/crsf_rx", 10);
    // CRSF 송신(TX) 채널 로그용(구독자 있을 때만).
    // [roll,pitch,thr,yaw,arm,mode,rtl,ch8] CRSF 단위.
    pub_crsf_tx_ = create_publisher<std_msgs::msg::Int16MultiArray>("/arms/crsf_tx", 10);
    // FC → ELRS → companion 경로의 GPS frame(0x02)을 별도 발행한다.
    pub_gps_ = create_publisher<arms_msgs::msg::GpsTelemetry>("/arms/gps", 10);

    // ----------------------------------------------------------------
    // ROS subscribers
    // ----------------------------------------------------------------
    auto best_effort_qos = rclcpp::QoS(1).best_effort();

    sub_detections_ = create_subscription<arms_msgs::msg::DetectionArray>(
        "/arms/detections", best_effort_qos,
        [this](arms_msgs::msg::DetectionArray::SharedPtr msg) {
          sm_->on_detection(msg->detections);
          update_looming(msg);
        });

    // BEST_EFFORT 로 구독: uart 노드(SensorDataQoS=best_effort)와 GUI(reliable)
    // 양쪽 발행자 모두와 호환된다. (reliable 구독은 best_effort 발행을 못 받음)
    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
        "/arms/command", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Joy::SharedPtr msg) {
          for (size_t i = 0; i < 4 && i < msg->axes.size(); ++i)
            joy_axes_[i] = msg->axes[i];

          auto btn = [&](int i) -> int {
            return (static_cast<size_t>(i) < msg->buttons.size()) ? msg->buttons[i] : 0;
          };
          int arm = btn(1), mode = btn(2), launch = btn(3), rtl = btn(4);
          int command_lost = btn(5);

          // MODE: 레벨 스위치. 0=auto(영상유도, ACRO/각속도), 1=manual(손제어, 자동수평)
          bool new_manual = static_cast<bool>(mode);
          if (new_manual != joy_manual_mode_) {
            joy_manual_mode_ = new_manual;
            // ---- ARM 재토글 안전장치 ----
            //   모드 전환 순간 ARM 스위치가 올라가 있어도 즉시 arm 되면 위험하다
            //   (예: auto+SEARCH=ARM 상태에서 manual 로 바꾸면 CH5 가 곧바로 arm).
            //   모드가 바뀌면 래치를 걸어, DISARM 으로 내렸다가 다시 올려야만 arm 되게 한다.
            //
            //   ★ 예외: TRACK(자동 요격 비행 중)에서 수동으로 넘길 땐 "날던 채로 인계"다.
            //     래치·pre-arm idle 검사를 건너뛰고 무장을 그대로 유지한다(공중 disarm=낙하 방지).
            //     TRACK 은 ARM 스위치가 이미 위(auto 에서 내리면 SEARCH OFF→IDLE)라 joy_arm_=true 보장.
            if (new_manual && sm_->state() == State::TRACK) {
              manual_armed_ = true;
              require_arm_reset_ = false;
              RCLCPP_INFO(get_logger(), "Mode: MANUAL (Angle) — TRACK 중 인계, 무장 유지");
            } else {
              require_arm_reset_ = true;
              RCLCPP_INFO(get_logger(), "Mode: %s (ARM 재토글 필요)",
                          joy_manual_mode_ ? "MANUAL (Angle)" : "AUTO (Acro)");
            }
          }
          if (launch && !prev_btn_[3]) {
            sm_->on_launch_button();
          }

          joy_arm_  = static_cast<bool>(arm);
          joy_rtl_  = static_cast<bool>(rtl);
          joy_command_lost_ = static_cast<bool>(command_lost);
          // ARM 스위치를 기본 위치(DISARM)로 내리면 재토글 래치 해제.
          if (!joy_arm_) require_arm_reset_ = false;
          prev_btn_[1] = arm;
          prev_btn_[2] = mode; prev_btn_[3] = launch;
          prev_btn_[4] = rtl;
        });

    // 심판(referee)의 직격 통지. SITL에서 실제 접촉이 확인되면 TRACK을 RTL로 전이한다.
    sub_hit_ = create_subscription<std_msgs::msg::Empty>(
        "/arms/hit", 10, [this](std_msgs::msg::Empty::SharedPtr) {
          if (hit_rtl_via_referee_) {
            const auto prev = to_string(sm_->state());
            sm_->on_hit();   // 심판 직격 명중 → RTL (SITL 지상진실 판정)
            RCLCPP_INFO(get_logger(),
                "[심판] 직격 명중 → RTL (referee-hit 모드, %s→%s)",
                prev.c_str(), to_string(sm_->state()).c_str());
          } else {
            RCLCPP_INFO(get_logger(),
                "[심판] 직격 명중 (미션 상태에는 영향 없음, 현재 %s)",
                to_string(sm_->state()).c_str());
          }
        });

    // SITL 패널 RESET: PX4가 다시 붙는 동안 이전 에피소드의 TRACK/RTL 상태나
    // arm 스위치가 새 기체로 이어지지 않도록 제어 상태를 안전한 IDLE로 비운다.
    sub_reset_ = create_subscription<std_msgs::msg::Empty>(
        "/arms/reset", 10, [this](std_msgs::msg::Empty::SharedPtr) {
          sm_->disarm();
          pid_roll_->reset();
          pid_pitch_->reset();
          filt_err_x_ = filt_err_y_ = 0.0;
          prev_err_x_ = prev_err_y_ = 0.0;
          err_dot_x_ = err_dot_y_ = 0.0;
          loom_s_ema_ = loom_rate_ema_ = 0.0;
          loom_init_ = false;
          pn_init_ = false;
          align_locked_ = false;
          auto_armed_ = manual_armed_ = false;
          lock_timer_started_ = auto_launched_ = false;
          rtl_sent_ = false;
          joy_arm_ = false;
          joy_rtl_ = false;
          joy_command_lost_ = false;
          require_arm_reset_ = true;
          RCLCPP_INFO(get_logger(), "실험 RESET: 제어 상태 IDLE, ARM 재토글 필요");
        });


    // ----------------------------------------------------------------
    // Control loop timer
    // ----------------------------------------------------------------
    last_tick_ = now();
    auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ArmsControlNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "arms_control_node started.");
  }

 private:
  // ----------------------------------------------------------------
  // Helpers
  // ----------------------------------------------------------------
  // 비전 looming: bbox 크기의 팽창률로 충돌까지 시간 τ 를 추정해 종말 RTL 판정에 넘긴다.
  //   τ = s/ṡ = d/v_closing. 초점거리·표적 실제크기가 소거되므로 카메라 스펙·표적
  //   크기 모두에 무관하다.
  void update_looming(const arms_msgs::msg::DetectionArray::SharedPtr & msg) {
    if (msg->detections.empty()) {
      loom_init_ = false;
      tau_sec_ = std::numeric_limits<double>::infinity();  // 표적 소실 → 급함 없음(추력 제약 해제)
      return;
    }
    // 검출 노드는 표적 1개만 발행. sphere/compact 표적이라 지름∝크기, 한 축 clip 대비 max.
    const auto & b = msg->detections.front();
    double s = std::max(b.width, b.height);
    rclcpp::Time t = now();
    if (!loom_init_) {                       // 첫 프레임: 초기화만 (rate 계산 불가)
      loom_s_ema_ = s;
      loom_rate_ema_ = 0.0;
      loom_last_time_ = t;
      loom_init_ = true;
      return;
    }
    double dt = (t - loom_last_time_).seconds();
    loom_last_time_ = t;
    if (dt <= 1e-3) return;                   // 같은 프레임/역행 시각 방어
    double s_prev = loom_s_ema_;
    loom_s_ema_ = loom_size_alpha_ * s + (1.0 - loom_size_alpha_) * loom_s_ema_;
    double rate = (loom_s_ema_ - s_prev) / dt;  // ṡ (정규화크기/초)
    loom_rate_ema_ = loom_rate_alpha_ * rate + (1.0 - loom_rate_alpha_) * loom_rate_ema_;
    // 닫히는 중(ṡ>0)일 때만 τ 유효. 아니면 무한대로 둔다.
    const double tau = (loom_rate_ema_ > 1e-4)
                   ? (loom_s_ema_ / loom_rate_ema_)
                   : std::numeric_limits<double>::infinity();
    tau_sec_ = tau;   // control_loop() 의 추력 스케줄링이 재사용
    if (looming_rtl_enabled_) {
      sm_->on_intercept(tau, loom_s_ema_);
    }

    geometry_msgs::msg::Vector3 lm;
    lm.x = std::isfinite(tau) ? std::min(tau, 9.99) : 9.99;  // 무한대(미접근)는 9.99로 표시
    lm.y = loom_s_ema_;
    lm.z = loom_rate_ema_;
    if (pub_loom_->get_subscription_count() > 0)  // 튜닝용: 구독자 있을 때만 발행
      pub_loom_->publish(lm);
  }

  /* 화각/종횡비 → 정규화오차(±0.5) 를 tan 단위 이미지좌표로 바꾸는 배율.
     error=±0.5(화면 가장자리)에서 x_n = tan(hfov/2) 가 되도록 맞춘다.
     세로는 정사각 픽셀 가정으로 tan(vfov/2) = tan(hfov/2) * (세로/가로). */
  void update_ibvs_scale() {
    const double half = 0.5 * ibvs_hfov_deg_ * M_PI / 180.0;
    const double t = std::tan(std::clamp(half, 1e-3, 1.5533));  // <89°로 제한(tan 발산 방지)
    ibvs_kx_ = 2.0 * t;
    ibvs_ky_ = 2.0 * t * ibvs_aspect_;
    double cxn = 0.0, cyn = 0.0;
    error_to_tan(0.5, 0.5, cxn, cyn);   // 화면 모서리
    RCLCPP_INFO(get_logger(),
                "IBVS 스케일: %s hfov=%.1fdeg aspect=%.3f → kx=%.3f ky=%.3f "
                "(화면 모서리 z1=%.3f, kb=%.3f)",
                cam_calibrated_ ? "[캘리 사용 — hfov/aspect 무시]" : "[hfov 근사]",
                ibvs_hfov_deg_, ibvs_aspect_, ibvs_kx_, ibvs_ky_,
                1.0 - 1.0 / std::sqrt(1.0 + cxn * cxn + cyn * cyn), ibvs_kb_);
  }

  /* 카메라 캘리 파라미터 로드 + 검증. 값이 이상하면 hfov 근사로 되돌린다
     (틀린 캘리로 나는 것보다 근사로 나는 게 안전). */
  void load_camera_params() {
    cam_calibrated_ = get_parameter("control.camera.calibrated").as_bool();
    cam_width_  = static_cast<double>(get_parameter("control.camera.image_width").as_int());
    cam_height_ = static_cast<double>(get_parameter("control.camera.image_height").as_int());
    cam_fx_ = get_parameter("control.camera.fx").as_double();
    cam_fy_ = get_parameter("control.camera.fy").as_double();
    cam_cx_ = get_parameter("control.camera.cx").as_double();
    cam_cy_ = get_parameter("control.camera.cy").as_double();
    const std::string model = get_parameter("control.camera.distortion_model").as_string();
    const auto d = get_parameter("control.camera.distortion").as_double_array();
    if (!cam_calibrated_) return;

    cam_fisheye_ = (model == "equidistant");
    const std::size_t need_max = cam_fisheye_ ? 4 : 5;
    std::string why;
    if (model != "plumb_bob" && model != "equidistant")
      why = "distortion_model 은 plumb_bob|equidistant 여야 함: " + model;
    else if (d.size() < 4 || d.size() > need_max)
      why = "distortion 계수 개수 " + std::to_string(d.size()) + " (" + model +
            (cam_fisheye_ ? " 는 4개)" : " 는 4~5개)");
    else if (cam_fx_ <= 0.0 || cam_fy_ <= 0.0 || cam_width_ <= 0.0 || cam_height_ <= 0.0)
      why = "fx/fy/image_width/image_height 는 양수여야 함";
    if (!why.empty()) {
      RCLCPP_ERROR(get_logger(), "카메라 캘리 무효 → hfov 근사로 대체: %s", why.c_str());
      cam_calibrated_ = false;
      return;
    }
    cam_dist_.fill(0.0);
    std::copy(d.begin(), d.end(), cam_dist_.begin());
    RCLCPP_INFO(get_logger(),
                "카메라 캘리 적용: %.0fx%.0f fx=%.2f fy=%.2f cx=%.2f cy=%.2f %s "
                "D=[%.4f %.4f %.4f %.4f %.4f]",
                cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_, model.c_str(),
                cam_dist_[0], cam_dist_[1], cam_dist_[2], cam_dist_[3], cam_dist_[4]);
  }

  /* 정규화 픽셀오차(error = bbox중심/화면 - 0.5, ±0.5) → tan 단위 이미지좌표 (x_n, y_n).
       calibrated=false: 이상적 핀홀 + 화면중앙 = 광축 (hfov/aspect 근사, SITL).
       calibrated=true : 픽셀 복원 → (u-cx)/fx → 렌즈 왜곡 역보정.
     detection 좌표는 해상도 무관 정규화값이라 캘리 해상도와 종횡비만 같으면 된다
     (크롭/레터박스가 들어가면 안 됨). */
  void error_to_tan(double ex, double ey, double& xn, double& yn) const {
    if (!cam_calibrated_) {
      xn = ibvs_kx_ * ex;
      yn = ibvs_ky_ * ey;
      return;
    }
    const double xd = ((ex + 0.5) * cam_width_  - cam_cx_) / cam_fx_;
    const double yd = ((ey + 0.5) * cam_height_ - cam_cy_) / cam_fy_;
    undistort_normalized(xd, yd, xn, yn);
  }

  /* 왜곡된 정규화좌표 (xd, yd) → 이상적 핀홀 좌표 (x, y). OpenCV undistortPoints 와 같은 모델. */
  void undistort_normalized(double xd, double yd, double& x, double& y) const {
    const auto& D = cam_dist_;
    if (cam_fisheye_) {
      // equidistant: theta_d = theta*(1 + k1 th^2 + k2 th^4 + k3 th^6 + k4 th^8) 를 뉴턴법으로 푼다.
      const double rd = std::hypot(xd, yd);
      if (rd < 1e-9) { x = xd; y = yd; return; }
      double th = rd;
      for (int i = 0; i < 10; ++i) {
        const double t2 = th * th, t4 = t2 * t2, t6 = t4 * t2, t8 = t4 * t4;
        const double f  = th * (1.0 + D[0] * t2 + D[1] * t4 + D[2] * t6 + D[3] * t8) - rd;
        const double df = 1.0 + 3.0 * D[0] * t2 + 5.0 * D[1] * t4 + 7.0 * D[2] * t6 + 9.0 * D[3] * t8;
        if (std::abs(df) < 1e-9) break;
        th -= f / df;
      }
      th = std::clamp(th, 0.0, 1.5533);   // <89° (tan 발산 방지)
      const double s = std::tan(th) / rd;
      x = xd * s;
      y = yd * s;
    } else {
      // plumb_bob (k1,k2,p1,p2,k3): 고정점 반복.
      x = xd;
      y = yd;
      for (int i = 0; i < 20; ++i) {
        const double r2 = x * x + y * y;
        const double rad = 1.0 + ((D[4] * r2 + D[1]) * r2 + D[0]) * r2;
        if (rad < 0.1) break;   // 캘리 유효범위 밖(강한 음의 k 로 모델이 꺾임) — 직전 값 유지
        const double dx = 2.0 * D[2] * x * y + D[3] * (r2 + 2.0 * x * x);
        const double dy = D[2] * (r2 + 2.0 * y * y) + 2.0 * D[3] * x * y;
        x = (xd - dx) / rad;
        y = (yd - dy) / rad;
      }
    }
  }

  static double apply_deadzone(double e, double dz) {
    if (dz <= 0.0) return e;
    if (e > dz) return e - dz;
    if (e < -dz) return e + dz;
    return 0.0;
  }

  /* 수신된 CRSF 텔레메트리와 통신 오류를 시험용 로그로 출력한다. */
  void handle_crsf_input() {
    const auto result = crsf_out_->receive();                     // UART에 쌓인 응답을 모두 가져온다.

    crsf_rx_bytes_ += result.bytes_read;                          // 누적 수신량을 갱신한다.
    crsf_rx_echoes_ += result.echo_frames;                        // 자체 송신 에코 수를 갱신한다.
    crsf_rx_crc_errors_ += result.crc_errors;                     // CRC 오류 수를 갱신한다.
    crsf_rx_framing_errors_ += result.framing_errors;             // 프레임 길이 오류 수를 갱신한다.
    crsf_rx_valid_frames_ += result.frames.size();                // 정상 텔레메트리 수를 갱신한다.

    if (result.bytes_read > 0 || result.crc_errors > 0 ||
        result.framing_errors > 0)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CRSF RX: bytes=%zu valid=%zu echo=%zu crc_err=%zu frame_err=%zu",
        crsf_rx_bytes_, crsf_rx_valid_frames_, crsf_rx_echoes_,
        crsf_rx_crc_errors_, crsf_rx_framing_errors_);            // 2초마다 누적 통계를 보여준다.
    }

    for (const auto & frame : result.frames) {
      if (!crsf_seen_types_[frame.type]) {
        crsf_seen_types_[frame.type] = true;                      // 타입별 첫 수신 여부를 기록한다.
        RCLCPP_INFO(
          get_logger(), "CRSF telemetry detected: addr=0x%02X type=0x%02X payload=%zu",
          static_cast<unsigned int>(frame.address),
          static_cast<unsigned int>(frame.type), frame.payload.size());   // 새 텔레메트리 타입을 한 번 알린다.
      }

      if (frame.type == 0x14 && frame.payload.size() >= 10) {
        const int up_rssi_1 = static_cast<int>(static_cast<int8_t>(frame.payload[0]));   // 상향 안테나 1 RSSI를 int8 부호로 복원해 dBm으로 읽는다.
        const int up_rssi_2 = static_cast<int>(static_cast<int8_t>(frame.payload[1]));   // 상향 안테나 2 RSSI를 int8 부호로 복원해 dBm으로 읽는다.
        const int up_lq = static_cast<int>(frame.payload[2]);              // 상향 링크 품질을 백분율로 읽는다.
        const int up_snr = static_cast<int>(static_cast<int8_t>(frame.payload[3]));   // 상향 SNR의 부호를 복원한다.
        const int down_rssi = static_cast<int>(static_cast<int8_t>(frame.payload[7]));   // 하향 RSSI를 int8 부호로 복원해 dBm으로 읽는다.
        const int down_lq = static_cast<int>(frame.payload[8]);            // 하향 링크 품질을 백분율로 읽는다.
        const int down_snr = static_cast<int>(static_cast<int8_t>(frame.payload[9]));   // 하향 SNR의 부호를 복원한다.

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ELRS link: up_rssi=(%d,%d)dBm up_lq=%d%% up_snr=%ddB "
          "down_rssi=%ddBm down_lq=%d%% down_snr=%ddB",
          up_rssi_1, up_rssi_2, up_lq, up_snr,
          down_rssi, down_lq, down_snr);                          // 링크 상태를 1초마다 보여준다.
        last_up_lq_ = up_lq;                                      // uplink LQ 저장(0이면 RF 링크 끊김으로 본다).
        last_lq_time_ = now();
        crsf_telem_.uplink_rssi_dbm = up_rssi_1;                  // /arms/crsf_rx 통합 필드 갱신
        crsf_telem_.uplink_lq = up_lq;
        crsf_telem_.uplink_snr_db = up_snr;
      } else if ((frame.type == 0x1C || frame.type == 0x1D) &&
                 frame.payload.size() >= 5)
      {
        const int rssi = static_cast<int>(static_cast<int8_t>(frame.payload[0]));   // 단일 링크 RSSI를 int8 부호로 복원해 dBm으로 읽는다.
        const int rssi_percent = static_cast<int>(frame.payload[1]);       // RSSI 백분율을 읽는다.
        const int link_quality = static_cast<int>(frame.payload[2]);       // 링크 품질을 백분율로 읽는다.
        const int snr = static_cast<int>(static_cast<int8_t>(frame.payload[3]));   // SNR의 부호를 복원한다.

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ELRS %s link: rssi=%ddBm rssi_pct=%d%% lq=%d%% snr=%ddB",
          frame.type == 0x1C ? "RX" : "TX",
          rssi, rssi_percent, link_quality, snr);                 // 분리형 링크 상태를 1초마다 보여준다.
      } else if (frame.type == 0x08 && frame.payload.size() >= 8) {
        const int voltage_dv =
          (static_cast<int>(frame.payload[0]) << 8) | frame.payload[1];   // 전압을 0.1V 단위(big-endian)로 읽는다.
        const int current_da =
          (static_cast<int>(frame.payload[2]) << 8) | frame.payload[3];   // 전류를 0.1A 단위(big-endian)로 읽는다.
        const int capacity_mah =
          (static_cast<int>(frame.payload[4]) << 16) |
          (static_cast<int>(frame.payload[5]) << 8) | frame.payload[6];    // 사용 용량을 mAh(3바이트 big-endian)로 읽는다.
        const int remaining_pct = static_cast<int>(frame.payload[7]);      // 남은 배터리 백분율을 읽는다.

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "CRSF battery: %.1fV %.1fA used=%dmAh remain=%d%%",
          voltage_dv / 10.0, current_da / 10.0,
          capacity_mah, remaining_pct);                            // 배터리 텔레메트리를 1초마다 보여준다.

        const double voltage_v = voltage_dv / 10.0;               // 전압을 V 단위로 환산한다.
        // 셀 수는 전압으로 자동 감지한다. cells = ceil(V / 4.3): 셀당 최대≈4.3V 로 나눠 이 전압을
        //   낼 수 있는 최소 셀 수를 구한다. 배터리 연결 시점엔 대개 충전돼 있어 정확하며, 첫
        //   유효 전압에서 1회 감지 후 래치한다(방전으로 값이 흔들리지 않게).
        if (battery_detected_cells_ <= 0 && voltage_v > 1.0) {
          battery_detected_cells_ = static_cast<int>(std::ceil(voltage_v / 4.3));
          RCLCPP_INFO(get_logger(), "배터리 셀 수 자동감지: %dS (%.1fV)",
                      battery_detected_cells_, voltage_v);
        }
        const int cells = battery_detected_cells_;
        double pct_ratio = remaining_pct / 100.0;                  // 감지 전 fallback: CRSF 잔량값.
        if (cells > 0) {                                           // 셀 수를 알면 전압으로 직접 계산.
          const double empty_v = cells * battery_cell_empty_v_;    // 0% 기준 전압.
          const double full_v  = cells * battery_cell_full_v_;     // 100% 기준 전압.
          const double span = full_v - empty_v;                   // 만충-방전 전압 폭.
          pct_ratio = (span > 1e-6) ? (voltage_v - empty_v) / span : 0.0;   // 선형 보간한다.
          pct_ratio = std::clamp(pct_ratio, 0.0, 1.0);            // 0~1 범위로 자른다.
        }

        // 배터리는 /arms/crsf_rx 통합 필드로만 발행(별도 /arms/battery 폐지 — UI 도 /arms/crsf_rx 구독).
        //   잔량%: cell_count 설정 시 전압 기반 계산값, 아니면 FC(CRSF) 원값(pct_ratio 가 이미 반영).
        crsf_telem_.voltage_v = static_cast<float>(voltage_v);
        crsf_telem_.current_a = static_cast<float>(current_da) / 10.0f;
        crsf_telem_.capacity_used_mah = capacity_mah;
        crsf_telem_.battery_remaining_pct =
            static_cast<int>(pct_ratio * 100.0 + 0.5);
      } else if (frame.type == 0x1E && frame.payload.size() >= 6) {
        // 자세(Attitude): pitch, roll, yaw 순서의 int16 big-endian, 값 = radian × 10000.
        auto be16 = [&](size_t i) {
          return static_cast<int16_t>(
            (static_cast<uint16_t>(frame.payload[i]) << 8) | frame.payload[i + 1]);
        };
        constexpr double R2D = 57.29577951308232;                  // rad→deg
        crsf_telem_.pitch_deg = static_cast<float>(be16(0) / 10000.0 * R2D);
        crsf_telem_.roll_deg  = static_cast<float>(be16(2) / 10000.0 * R2D);
        crsf_telem_.yaw_deg   = static_cast<float>(be16(4) / 10000.0 * R2D);
      } else if (frame.type == 0x02 && frame.payload.size() >= 15) {
        // GPS: lat/lon E7, speed 0.1 km/h, course 0.01 deg, altitude+1000 m, sats.
        auto be32 = [&](size_t i) {
          return static_cast<int32_t>(
            (static_cast<uint32_t>(frame.payload[i]) << 24) |
            (static_cast<uint32_t>(frame.payload[i + 1]) << 16) |
            (static_cast<uint32_t>(frame.payload[i + 2]) << 8) |
             static_cast<uint32_t>(frame.payload[i + 3]));
        };
        auto beu16 = [&](size_t i) {
          return static_cast<uint16_t>(
            (static_cast<uint16_t>(frame.payload[i]) << 8) | frame.payload[i + 1]);
        };
        arms_msgs::msg::GpsTelemetry gps;
        gps.header.stamp = now();
        gps.latitude_deg = static_cast<double>(be32(0)) / 1e7;
        gps.longitude_deg = static_cast<double>(be32(4)) / 1e7;
        gps.ground_speed_mps = static_cast<float>(beu16(8)) / 36.0f;
        gps.ground_course_deg = static_cast<float>(beu16(10)) / 100.0f;
        gps.altitude_m = static_cast<float>(beu16(12)) - 1000.0f;
        gps.satellites = frame.payload[14];
        // CRSF GPS에는 MAVLink GPS_RAW_INT의 fix_type이 없다. MAVLink 명칭을
        // UI/로그에서 일관되게 쓰기 위해 위성 수로만 보수적으로 추정한다.
        // 0=NO_GPS는 "GPS frame 자체가 전혀 없음"에만 쓰고, 수신 frame에서
        // sats<3은 1=NO_FIX, 3은 2=2D_FIX, 4 이상은 3=3D_FIX로 기록한다.
        gps.mavlink_fix_type = gps.satellites < 3 ? 1
                             : gps.satellites < 4 ? 2
                                                   : 3;
        gps.fix_type_inferred = true;
        gps.valid_fix = gps.mavlink_fix_type >= 3;
        pub_gps_->publish(gps);
      } else if (frame.type == 0x07 && frame.payload.size() >= 2) {
        // 수직속도(Vario): int16 big-endian, 단위 cm/s → m/s.
        const int16_t vspeed_cms = static_cast<int16_t>(
          (static_cast<uint16_t>(frame.payload[0]) << 8) | frame.payload[1]);
        crsf_telem_.vertical_speed_mps = static_cast<float>(vspeed_cms) / 100.0f;
      } else if (frame.type == 0x21 && !frame.payload.empty()) {
        // 비행 모드(Flight mode): null-종료 ASCII 문자열.
        const char * s = reinterpret_cast<const char *>(frame.payload.data());
        size_t len = 0;
        while (len < frame.payload.size() && s[len] != '\0') ++len;   // 널/끝까지만
        crsf_telem_.flight_mode.assign(s, len);
      }
    }

    // 이번에 CRSF 프레임을 하나라도 받았으면 통합 텔레메트리를 발행한다(최신 스냅샷).
    if (!result.frames.empty()) {
      crsf_telem_.header.stamp = now();
      pub_crsf_rx_->publish(crsf_telem_);
    }
  }

  // ----------------------------------------------------------------
  // 30 Hz control loop
  // ----------------------------------------------------------------
  void control_loop() {
    handle_crsf_input();                                         // 직전 송신 이후 도착한 텔레메트리를 처리한다.

    auto now_t = now();
    double dt = (now_t - last_tick_).seconds();
    last_tick_ = now_t;

    State state = sm_->state();

    // 유도 출력 = 각속도 명령[deg/s]. 각도가 아니다 (ACRO 고정, 자세 피드백 없음).
    double roll_rate_cmd = 0.0;
    double pitch_rate_cmd = 0.0;
    float thrust = 0.f;

    // IBVS 디버그 스냅샷 — 아래 계산 단계마다 채우고 틱 끝에 발행한다.
    arms_msgs::msg::IbvsDebug idbg;
    idbg.header.stamp = now_t;
    idbg.state = to_string(state);
    idbg.guidance_mode = guidance_mode_;
    idbg.dt = dt;
    idbg.kb = ibvs_kb_;
    idbg.ibvs_gain = ibvs_gain_;
    idbg.ibvs_center_gain = ibvs_center_gain_;
    idbg.ibvs_los_rate_gain = ibvs_los_rate_gain_;
    idbg.range_gain_scale = 1.0;
    idbg.range_throttle_scale = 1.0;
    idbg.tau_ease = 1.0;

    // LOCK 단계부터 필터(LOS/오차미분/alpha-beta 추정)를 미리 갱신해둔다.
    // 실제 회전/추력 명령은 TRACK에서만 나가지만(아래 안쪽 if),
    // TRACK 진입 순간 filt_err_x_/y_ 등이 0부터 다시 시작하는 콜드스타트를 없애기 위함.
    if (state == State::LOCK || state == State::TRACK) {
      double raw_ex = sm_->current_error_x();
      double raw_ey = sm_->current_error_y();
      filt_err_x_ =
          error_lpf_alpha_ * raw_ex + (1.0 - error_lpf_alpha_) * filt_err_x_;
      filt_err_y_ =
          error_lpf_alpha_ * raw_ey + (1.0 - error_lpf_alpha_) * filt_err_y_;

      // ---- LOS = 픽셀 오차 ----
      //   (예전엔 실제 자세로 카메라-기울기 성분을 보정했으나, acro 제어 전환으로 attitude
      //    의존을 제거함. FC 가 ACRO 로 각속도를 직접 처리하므로 젯슨 자세 루프 불필요.)
      double los_x = filt_err_x_;
      double los_y = filt_err_y_;

      // ---- 중앙 데드존 (보정된 LOS 기준) ----
      double ex = apply_deadzone(los_x, deadzone_);
      // Mode 0에서 양의 offset은 풍선이 화면 중심보다 아래(raw_err_y>0)에
      // 머물도록 만든다. 즉 카메라 중심/추력축은 풍선보다 화면 위를 조준한다.
      // 상수항은 미분 전에 넣지 않아 err_dot/PN/IBVS 추정을 오염시키지 않는다.
      const double mode0_y_reference =
          guidance_mode_ == 0 ? mode0_aim_above_offset_ : 0.0;
      double ey = apply_deadzone(los_y - mode0_y_reference, deadzone_);
      if (dt > 1e-6) {
        // 표적 시선각속도(LOS rate) 추정 = 예측조준의 핵심 신호 (보정 LOS 의 미분).
        const double DOT_A = err_dot_alpha_;
        double raw_dx = (los_x - prev_err_x_) / dt;
        double raw_dy = (los_y - prev_err_y_) / dt;
        err_dot_x_ = DOT_A * raw_dx + (1.0 - DOT_A) * err_dot_x_;
        err_dot_y_ = DOT_A * raw_dy + (1.0 - DOT_A) * err_dot_y_;
        err_dot_x_ = std::clamp(err_dot_x_, -err_dot_clamp_, err_dot_clamp_);
        err_dot_y_ = std::clamp(err_dot_y_, -err_dot_clamp_, err_dot_clamp_);
      }
      prev_err_x_ = los_x;
      prev_err_y_ = los_y;

      bool held = sm_->is_detection_held();
      idbg.filter_active = true;
      idbg.raw_err_x = raw_ex;
      idbg.raw_err_y = raw_ey;
      idbg.filt_err_x = filt_err_x_;
      idbg.filt_err_y = filt_err_y_;
      idbg.detection_held = held;
      idbg.err_dot_x = err_dot_x_;
      idbg.err_dot_y = err_dot_y_;
      pid_roll_->set_integral_frozen(held);
      pid_pitch_->set_integral_frozen(held);

      // ---- 표적 상태추정: alpha-beta 필터 (매끈한 LOS + LOS각속도) ----
      //   거친 미분(+클램프) 대신 예측-보정 필터로 시선각속도를 깔끔히 추정한다.
      //   빠른 표적 추적의 핵심 신호 → PN 유도가 이 losf_*_dot_ 을 쓴다.
      if (!pn_init_) {
        losf_x_ = los_x; losf_y_ = los_y;
        losf_x_dot_ = 0.0; losf_y_dot_ = 0.0;
        pn_init_ = true;
      } else if (dt > 1e-6) {
        double xp = losf_x_ + losf_x_dot_ * dt;      // 예측
        double rx = los_x - xp;                       // 잔차(측정−예측)
        losf_x_ = xp + pn_alpha_ * rx;
        losf_x_dot_ += (pn_beta_ / dt) * rx;
        double yp = losf_y_ + losf_y_dot_ * dt;
        double ry = los_y - yp;
        losf_y_ = yp + pn_alpha_ * ry;
        losf_y_dot_ += (pn_beta_ / dt) * ry;
        // 검출이 튀면(표적 프레임 이탈/재획득) 추정치가 ±10 로 폭주해 드론이 날뛴다 → 안전 상한.
        losf_x_dot_ = std::clamp(losf_x_dot_, -3.0, 3.0);
        losf_y_dot_ = std::clamp(losf_y_dot_, -3.0, 3.0);
      }
      idbg.losf_x = losf_x_;
      idbg.losf_y = losf_y_;
      idbg.losf_x_dot = losf_x_dot_;
      idbg.losf_y_dot = losf_y_dot_;

      // ---- LOCK 프리웜 디버그: filt_err_x/y_ 가 실제로 수렴해가는지 눈으로 확인용 ----
      //   TRACK 진입 순간 이 값이 이미 실제 오차 근처여야 콜드스타트 수정이 제대로 먹은 것.
      if (state == State::LOCK && (++dbg_count_ % 6 == 0)) {
        RCLCPP_INFO(get_logger(), "LOCK 프리웜: filt_err=(%.3f,%.3f) raw=(%.3f,%.3f)",
                    filt_err_x_, filt_err_y_, raw_ex, raw_ey);
      }

      // ---- 여기부터 실제 회전/추력 명령을 만든다: TRACK에서만 ----
      if (state == State::TRACK) {
      idbg.cmd_active = true;
      if (guidance_mode_ == 2) {
        // ===== IBVS 공선유도 (배리어 Lyapunov) =====
        //   논문 식 (12),(13) 을 우리 기하(광축 = 추력축 = 기체 +Z)에 적용한 것.
        //   목표: 추력축을 시선에 정렬 = "표적을 겨누면 곧 표적으로 가속한다".
        //   자세/거리/속도 없이 픽셀과 화각만으로 계산된다 (자세 R_b^e 가 상쇄됨).

        // ① 픽셀오차 → tan 단위 정규화 이미지좌표. error=±0.5 가 화면 가장자리.
        //    실기체는 캘리(fx,fy,cx,cy,왜곡)로, SITL 은 hfov 근사로 환산한다.
        double x_n = 0.0, y_n = 0.0;
        error_to_tan(los_x, los_y, x_n, y_n);
        const double Nn  = std::sqrt(1.0 + x_n * x_n + y_n * y_n);

        // ② 공선 오차 z1 = 1 - cos(theta). theta = 광축에서 벗어난 각도.
        double z1 = 1.0 - 1.0 / Nn;
        idbg.x_n = x_n;
        idbg.y_n = y_n;
        idbg.theta_deg = std::acos(1.0 / Nn) * 180.0 / M_PI;
        idbg.z1_raw = z1;
        // 배리어 안쪽으로 강제한다. kb 를 화면보다 작게 잡았거나 검출이 튀어
        //   z1 >= kb 가 되면 분모가 음수로 넘어가 명령 부호가 뒤집힌다 — 그러면
        //   드론이 표적 반대로 돌아버린다. 여기서 반드시 막아야 한다.
        const double z1_max = ibvs_kb_ * 0.995;
        if (z1 > z1_max) z1 = z1_max;
        const double denom   = std::max(ibvs_kb_ * ibvs_kb_ - z1 * z1, 1e-6);
        const double barrier = z1 / denom;   // 중앙에선 ~0, 가장자리에서 발산
        ibvs_z1_dbg_ = z1;
        idbg.z1 = z1;
        idbg.z1_clamped = (idbg.z1_raw > z1_max);
        idbg.barrier = barrier;

        // ③ 회전축 = e3 x (표적 방향). 기체좌표계에서 (x_n, y_n, 0)/N 이 된다.
        //    크기는 sin(theta), yaw(z) 성분은 구조적으로 0 — 상방 카메라 기하 덕분.
        const double dir_x = x_n / Nn;
        const double dir_y = y_n / Nn;

        // ④ LOS각속도(리드/감쇠) 항. 배리어항만으론 '표적의 현재 위치'만 겨냥하는
        //    pure pursuit 이라 횡단 표적을 종말에 놓친다 — 이 항이 리드를 만든다.
        //    근거리 폭주 방지 클램프는 PN(mode 1)과 같은 pn_los_clamp 를 쓴다
        //    (같은 신호에 한계가 둘이면 어느 쪽이 걸리는지 알 수 없다).
        double ldx = losf_x_dot_, ldy = losf_y_dot_;
        if (pn_los_clamp_ > 0.0) {
          ldx = std::clamp(ldx, -pn_los_clamp_, pn_los_clamp_);
          ldy = std::clamp(ldy, -pn_los_clamp_, pn_los_clamp_);
        }

        // ⑤ 배리어항(FOV 유지) + 선형항(중앙 권한) + LOS각속도항(리드/감쇠).
        const double rc = (ibvs_gain_ * barrier + ibvs_center_gain_) * dir_x
                          + ibvs_los_rate_gain_ * ldx;
        const double pc = (ibvs_gain_ * barrier + ibvs_center_gain_) * dir_y
                          + ibvs_los_rate_gain_ * ldy;
        idbg.dir_x = dir_x;
        idbg.dir_y = dir_y;
        idbg.los_rate_x = ldx;
        idbg.los_rate_y = ldy;
        idbg.barrier_term_x = ibvs_gain_ * barrier * dir_x;
        idbg.barrier_term_y = ibvs_gain_ * barrier * dir_y;
        idbg.center_term_x = ibvs_center_gain_ * dir_x;
        idbg.center_term_y = ibvs_center_gain_ * dir_y;
        idbg.los_rate_term_x = ibvs_los_rate_gain_ * ldx;
        idbg.los_rate_term_y = ibvs_los_rate_gain_ * ldy;
        idbg.guidance_x = rc;
        idbg.guidance_y = pc;
        roll_rate_cmd  = roll_sign_  * rc;
        pitch_rate_cmd = pitch_sign_ * pc;
      } else if (guidance_mode_ == 1) {
        // ===== 비례항법 (Proportional Navigation) =====
        //   기본 추적(현위치 추종)과 달리 '시선각속도(losf_dot)를 0으로' 만들도록 조종한다.
        //   → 자동으로 미래 충돌점을 앞질러 겨냥 = 가로지르는 빠른 표적에 훨씬 유리.
        //   횡가속(=기울기) 명령 = N×LOS각속도 + 약한 중심유지(표적 화면이탈 방지).
        //   LOS각속도 클램프로 노이즈 폭주 방지.
        double ldx = losf_x_dot_, ldy = losf_y_dot_;
        if (pn_los_clamp_ > 0.0) {
          ldx = std::clamp(ldx, -pn_los_clamp_, pn_los_clamp_);
          ldy = std::clamp(ldy, -pn_los_clamp_, pn_los_clamp_);
        }
        double rc = pn_nav_gain_ * ldx + pn_center_gain_ * los_x;
        double pc = pn_nav_gain_ * ldy + pn_center_gain_ * los_y;
        idbg.los_rate_x = ldx;
        idbg.los_rate_y = ldy;
        idbg.guidance_x = rc;
        idbg.guidance_y = pc;
        roll_rate_cmd  = roll_sign_  * rc;
        pitch_rate_cmd = pitch_sign_ * pc;
      } else {
        // ===== 기본 추적: PID + 시간기반 리드 =====
        // 게인 감쇠는 control.gain_shaping 이 true 일 때만. 기본은 순수 P 다.
        double gain_scale = 1.0;
        if (gain_shaping_) {
          double emag_g = std::hypot(ex, ey);
          double tgt_spd = std::hypot(err_dot_x_, err_dot_y_);   // 표적 각속도
          // 중앙 근처 감쇠는 '느린/정지' 표적에서만(지터 방지). 빠른 표적은 full 게인 → 뒤처짐 방지.
          if (tgt_spd < 0.2) {
            if (emag_g < 0.08)      gain_scale = 0.5;
            else if (emag_g < 0.20) gain_scale = 0.75;
          }
        }
        double t_lead = lead_gain_;   // 시간기반 리드만 쓴다 (거리비례 리드는 3D 거리와 함께 삭제됨)
        double ex_lead = ex + t_lead * err_dot_x_;
        double ey_lead = ey + t_lead * err_dot_y_;
        roll_rate_cmd = roll_sign_ * gain_scale * pid_roll_->compute(ex_lead, dt);
        double pitch_scale = 1.0;
        if (gain_shaping_) {          // roll 오차가 크면 pitch 를 눌러 좌우 정렬 우선
          if (fabs(ex) > 0.12)      pitch_scale = 0.8;
          else if (fabs(ex) > 0.06) pitch_scale = 0.9;
        }
        pitch_rate_cmd = pitch_sign_ * gain_scale * pitch_scale * pid_pitch_->compute(ey_lead, dt);
        idbg.guidance_x = roll_sign_ * roll_rate_cmd;    // 부호 전 값으로 되돌려 기록
        idbg.guidance_y = pitch_sign_ * pitch_rate_cmd;
      }

      // ---- 근접도(closeness) 기반 회전 게인 스케줄링 ----
      //   loom_s_ema_(화면 속 표적 크기)가 near_size_start_를 넘으면서부터 서서히,
      //   near_size_full_에서 min_gain_scale_까지 낮춘다. guidance_mode(PID/PN)
      //   양쪽 다 여기서 공통으로 적용 — 원근 때문에 가까울수록 같은 실제 오차도
      //   화면상 커 보여서 과민반응(오버슈트)하기 쉬운 걸 완화한다.
      {
        double closeness = std::clamp(
            (loom_s_ema_ - near_size_start_) /
                std::max(near_size_full_ - near_size_start_, 1e-3),
            0.0, 1.0);
        double range_gain_scale = 1.0 - closeness * (1.0 - min_gain_scale_);
        roll_rate_cmd  *= range_gain_scale;
        pitch_rate_cmd *= range_gain_scale;
        idbg.closeness = closeness;
        idbg.range_gain_scale = range_gain_scale;
      }

      // ---- 추격 궤적(Pursuit): 조준하며 처음부터 상승 ----
      //   정렬 완료를 기다리지 않는다. 추력은 기체가 기울어진 방향(=공을 겨눈 방향)으로
      //   나가므로, "공쪽으로 기울이기(추적)" + "상승" 을 동시에 하면 드론이 공을 향해
      //   대각선으로 날아간다(추격). 공을 잘 겨눌수록(center_q↑) 그쪽으로 더 세게 상승하고,
      //   빗나가면 상승을 줄여(hover) 재조준을 우선한다. → 수직으로 떴다 늦게 꺾는 문제 해결.
      {
        // 근접도 기반 스로틀 스케줄링: 가까울수록 최대 추력 상한을 낮춰서
        // 회전 게인을 부드럽게 만든 만큼 반응할 시간도 같이 벌어준다
        // (속도를 안 줄이면 게인만 낮춰봤자 스쳐 지나가는 시간이 그대로라 무의미함).
        double closeness_thr = std::clamp(
            (loom_s_ema_ - near_size_start_) /
                std::max(near_size_full_ - near_size_start_, 1e-3),
            0.0, 1.0);
        double range_throttle_scale = 1.0 - closeness_thr * (1.0 - min_throttle_scale_);
        const double up = hover_throttle_ +
            (track_throttle_ - hover_throttle_) * range_throttle_scale;  // 잘 겨눴을 때 최대 상승(=공쪽 돌진, 근접시 하향)
        const double hover = hover_throttle_;   // 빗나갔을 때 최소(재조준)
        // Mode 0은 중앙이 아니라 aim offset 위치가 목표이므로, 추력 보간과
        // 정렬 완료 판정도 같은 보정 오차를 사용한다. Mode 1/2는 기존 중앙 기준.
        double emag = guidance_mode_ == 0
                          ? std::hypot(ex, ey)
                          : std::hypot(filt_err_x_, filt_err_y_);

        // ---- τ(충돌까지 시간) 기반 추력 감속 ----
        //   위 near_size 스케줄링은 "크기"만 보고 접근속도는 모른다. τ 는 크기와
        //   팽창률을 같이 보므로 "곧 부딪힌다"를 더 일찍/정확히 안다. 정렬 품질
        //   (center_q)과 무관하게 곱해져서, 겨냥이 완벽해도 임박하면 추력을 눌러
        //   관성으로 지나치는 것을 막고 정밀 정렬에 시간을 번다.
        //   τ=+inf(표적 없음/안 다가오는 중)면 tau_ease=1.0 → 지금과 동일하게 동작.
        const double tau_ease = std::clamp(
            (tau_sec_ - tau_ease_full_) /
                std::max(tau_ease_start_ - tau_ease_full_, 1e-3),
            tau_ease_min_scale_, 1.0);
        tau_ease_dbg_ = tau_ease;   // 로그용

        if (pursuit_center_boost_) {
          // 중앙 정렬될수록(center_q↑) 그쪽으로 더 세게 상승, 빗나가면 hover 로 낮춰 재조준 우선.
          double center_q =
              std::clamp(1.0 - emag / std::max(pursuit_gate_, 1e-3), 0.0, 1.0);
          thrust = static_cast<float>(hover + (up - hover) * center_q * tau_ease);
          idbg.center_q = center_q;
        } else {
          // 비활성화: 정렬 정도와 무관하게 상수 추력(track_throttle) 유지 (τ 감속만 적용).
          thrust = static_cast<float>(hover + (up - hover) * tau_ease);
          idbg.center_q = 1.0;
        }
        idbg.range_throttle_scale = range_throttle_scale;
        idbg.throttle_up = up;
        idbg.emag = emag;
        idbg.tau_ease = tau_ease;

        if (!align_locked_ && emag < 0.15) {
          align_locked_ = true;
          RCLCPP_INFO(get_logger(), "추격 시작 (오차 %.2f) → 공 향해 대각선 상승", emag);
        }
      }

      if (++dbg_count_ % 6 == 0) {
        const char* gm = (guidance_mode_ == 2) ? "IBVS"
                       : (guidance_mode_ == 1) ? "PN" : "PUR";
        RCLCPP_INFO(get_logger(),
                    "TRACK[%s] err=(%.2f,%.2f) losdot=(%.2f,%.2f) z1=%.3f/%.2f bbox=%.2f "
                    "tau=%.2fs ease=%.2f thr=%.2f rate=(%.0f,%.0f)deg/s",
                    gm,
                    filt_err_x_, filt_err_y_,
                    guidance_mode_ >= 1 ? losf_x_dot_ : err_dot_x_,
                    guidance_mode_ >= 1 ? losf_y_dot_ : err_dot_y_,
                    ibvs_z1_dbg_, ibvs_kb_,
                    loom_s_ema_,
                    std::isfinite(tau_sec_) ? tau_sec_ : 9.99, tau_ease_dbg_, thrust,
                    roll_rate_cmd,
                    pitch_rate_cmd);
      }
      }
    } else {
      // IDLE / SEARCH / RTL (표적을 아예 못 보거나 교전 밖인 진짜 리셋 상황)
      pid_roll_->reset();
      pid_pitch_->reset();
      filt_err_x_ = 0.0;
      filt_err_y_ = 0.0;
      thrust = 0.f;
      align_locked_ = false;
      pn_init_ = false;   // alpha-beta 추정기 재초기화 (다음 TRACK 진입 시)
      losf_x_ = losf_y_ = losf_x_dot_ = losf_y_dot_ = 0.0;
      ibvs_z1_dbg_ = 0.0;
      tau_ease_dbg_ = 1.0;
    }

    // ---- 유효 스위치 = 상태 스위치 && 재토글 래치 해제 ----
    //   모드 전환/부팅 직후엔 스위치가 올라가 있어도 즉시 적용 안 됨 → 재토글(DOWN→UP) 필요.
    //   · 자동: idle/search 게이트에 적용 (기본 IDLE, SEARCH 재토글 안전장치)
    //   · 수동: CH5 arm 에 적용 (기본 DISARM, arm 재토글 안전장치)
    //   ※ 자동 CH5 arm 자체는 스위치와 분리되어 상태머신 상태(auto_arm_states_)로 결정한다.
    // command UART가 끊긴 경우에는 직전 틱에서 실제로 arm 상태였던 기체만 무장을
    // 유지해 FC GPS Rescue로 넘긴다. 미무장 상태의 통신 단절이 새 arm을 만들면 안 된다.
    const bool effective_arm =
        (joy_arm_ && !require_arm_reset_) ||
        (joy_command_lost_ && last_ch5_armed_);

    // ---- 상태 스위치 = idle/search 게이트 (auto 모드) ----
    //   자동 모드에서 이 스위치는 arm 과 분리되어 미션 상태(IDLE↔SEARCH)만 제어한다.
    //   재토글 안전장치 적용: 모드 전환/부팅 직후 스위치가 SEARCH(위)여도 기본 IDLE 이고,
    //   DISARM 으로 내렸다 다시 올려야 SEARCH 진입 (수동 arm 과 동일 매커니즘).
    if (!joy_manual_mode_) {
      if (effective_arm && state == State::IDLE && !auto_armed_) {
        auto_armed_ = true;
        sm_->arm();
        RCLCPP_INFO(get_logger(), "SEARCH ON (IDLE → SEARCH)");
      } else if (!effective_arm) {
        if (state != State::IDLE) {
          sm_->disarm();
          RCLCPP_INFO(get_logger(), "SEARCH OFF → IDLE");
        }
        auto_armed_ = false;
      }
    } else {
      // ---- 수동 모드: 자동 상태머신을 IDLE 로 고정 ----
      //   수동에서는 미션 상태가 의미 없으므로 IDLE 로 둔다(서보도 열림). 영상/방아쇠로
      //   미션이 멋대로 진행되는 것도 막는다. 자동으로 넘어오면 상태 스위치가 곧바로
      //   idle/search 를 제어한다(자동에선 스위치가 arm 과 분리되어 재토글 불필요).
      if (state != State::IDLE) {
        sm_->disarm();
        RCLCPP_INFO(get_logger(), "MANUAL → 자동 상태머신 IDLE 고정");
      }
      auto_armed_ = false;
    }

    // ---- SITL auto-launch ----
    if (sitl_auto_launch_) {
      if (state == State::LOCK) {
        if (!lock_timer_started_) {
          lock_enter_time_ = now_t;
          lock_timer_started_ = true;
        } else if (!auto_launched_ && (now_t - lock_enter_time_).seconds() >=
                                          auto_launch_delay_sec_) {
          auto_launched_ = true;
          RCLCPP_INFO(get_logger(), "SITL auto-launch: LOCK -> TRACK");
          sm_->on_launch_button();
        }
      } else if (state == State::IDLE || state == State::SEARCH) {
        lock_timer_started_ = false;
        auto_launched_ = false;
      }
    }

    // ---- 수동 arm pre-arm 안전 확인 (스틱 idle) ----
    //   arm 하는 순간 스틱이 idle(throttle 최저 + roll/pitch/yaw 중앙)이어야만 실제 arm.
    //   arm 순간에만 확인하고, 한 번 arm(manual_armed_)되면 비행 중 스틱을 움직여도 유지된다.
    //   arm 스위치를 내리면(!effective_arm) 해제. 자동 모드에선 미사용.
    if (joy_manual_mode_) {
      const bool sticks_idle =
          (joy_axes_[1] <= prearm_throttle_max_) &&         // throttle 최저
          (std::abs(joy_axes_[0]) <= prearm_stick_tol_) &&  // yaw 중앙
          (std::abs(joy_axes_[2]) <= prearm_stick_tol_) &&  // roll 중앙
          (std::abs(joy_axes_[3]) <= prearm_stick_tol_);    // pitch 중앙
      if (!effective_arm) {
        manual_armed_ = false;
        prearm_blocked_ = false;
      } else if (!manual_armed_) {   // arm 시도 중 (아직 무장 전)
        if (!prearm_check_ || sticks_idle) {
          manual_armed_ = true;
          prearm_blocked_ = false;
          RCLCPP_INFO(get_logger(), "MANUAL ARM (스틱 idle 확인)");
        } else {
          prearm_blocked_ = true;   // arm 스위치 올렸지만 스틱 안 idle → 차단 (UI 경고)
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "ARM 차단: 스틱을 idle 로 두세요 (throttle 최저 + roll/pitch/yaw 중앙)");
        }
      } else {
        prearm_blocked_ = false;   // 이미 무장됨
      }
    } else {
      manual_armed_ = false;
      prearm_blocked_ = false;
    }

    // AUTO 처리에서 상태가 바뀌었을 수 있으므로 최신 상태를 다시 읽는다.
    state = sm_->state();

    // ---- CH5(FC arm) 결정 ----
    // ARM 스위치를 내리면 수동/자동/RTL 모두 CH5를 내린다. 따라서 RTL 중 ARM OFF는
    // 귀환 취소가 아니라 비상 motor-cut이다. GPS Rescue를 쓸 때 ARM은 계속 ON이어야 한다.
    const bool base_ch5_armed =
        joy_manual_mode_ ? manual_armed_
                         : (effective_arm && auto_armed_ &&
                            auto_arm_states_.count(to_string(state)) > 0);

    // RTL은 "이미 ARM 된 기체"에서만 받아들인다. 자동/수동 어느 쪽이든 상태머신을
    // RTL로 옮겨 UI에 명확히 표시하고, 실제 귀환은 CH7의 FC GPS Rescue 매핑이 수행한다.
    const bool rtl_active = (joy_rtl_ || joy_command_lost_) && base_ch5_armed;
    if (rtl_active && state != State::RTL) {
      sm_->request_rtl();
      state = sm_->state();
      RCLCPP_WARN(get_logger(), "RTL requested — FC GPS Rescue AUX asserted");
    }
    const bool rtl_handover = (state == State::RTL);
    const bool manual_out = joy_manual_mode_ || rtl_handover;
    const bool ch5_armed = base_ch5_armed;

    // ---- CRSF output ----
    {
      CrsfOutput::Channels crsf{};
      crsf.fill(CrsfOutput::CRSF_MIN);

      if (manual_out) {
        // 수동/RTL 핸드오버: 스틱 → AETR 채널 passthrough.
        //   L-stick: X=axes[0]=yaw,      Y=axes[1]=throttle
        //   R-stick: X=axes[2]=roll,     Y=axes[3]=pitch
        crsf[0] = CrsfOutput::norm_to_crsf(joy_axes_[2]);  // CH1 roll  = R-stick X
        crsf[1] = CrsfOutput::norm_to_crsf(joy_axes_[3]);  // CH2 pitch = R-stick Y
        crsf[2] = CrsfOutput::norm_to_crsf(joy_axes_[1]);  // CH3 thr   = L-stick Y (스프링, 중앙=50%)
        crsf[3] = CrsfOutput::norm_to_crsf(joy_axes_[0]);  // CH4 yaw   = L-stick X
      } else {
        // AUTO(영상유도) 출력 — ACRO 고정.
        //   유도(PID/PN/IBVS)가 낸 각속도 명령을 max_cmd_rate_dps 로 제한하고,
        //   풀스틱 기준 max_rate_dps 로 정규화한다. PX4 ACRO 가 그 각속도로 직접
        //   회전한다(자동수평 없음). 중간의 '목표 기울기' 단계는 없다.
        double rr = std::clamp(roll_rate_cmd,  -max_cmd_rate_dps_, max_cmd_rate_dps_);
        double pr = std::clamp(pitch_rate_cmd, -max_cmd_rate_dps_, max_cmd_rate_dps_);
        crsf[0] = CrsfOutput::norm_to_crsf(rr / max_rate_dps_);
        crsf[1] = CrsfOutput::norm_to_crsf(pr / max_rate_dps_);
        crsf[2] = CrsfOutput::thr_to_crsf(static_cast<double>(thrust));
        crsf[3] = CrsfOutput::CRSF_CENTER;  // yaw 중앙 = yaw rate 0(헤딩 유지)
      }

      // CH5: arm — 수동은 스위치(재토글 안전장치), 자동은 상태머신 상태 기반.
      //   자동: 기본 발사(TRACK)부터 arm (control.auto_arm_states 로 조정).
      crsf[4] = ch5_armed ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

      // CH6: flight mode — auto=ACRO/각속도(172, low), manual|RTL=자동수평(1811, high).
      //   FC 를 CH6 high=자동수평(self-level), low=ACRO 로 매핑해 둔다.
      //   RTL 이면 자동으로 자동수평(수동)으로 넘어가 조종사가 이어받아 착륙.
      crsf[5] = manual_out ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

      // CH7: RTL/GPS Rescue. Betaflight의 GPS RESCUE mode에 이 AUX를 매핑한다.
      // 조종기 요청뿐 아니라 target loss/종말 판정/심판 hit로 상태가 RTL이 된 경우도
      // 동일하게 FC Rescue를 활성화해야 실제 귀환이 수행된다.
      crsf[6] = rtl_handover ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

      // CH8: 미사용 (crsf.fill(CRSF_MIN)으로 172 고정)

      if (!crsf_out_->send(crsf)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "CRSF TX failed: check UART port and baud rate.");      // 송신 실패를 과도한 반복 없이 알린다.
      }

      // 송신 CRSF 채널 로그(구독자 있을 때만; bag record 가 구독하면 자동 on).
      if (pub_crsf_tx_->get_subscription_count() > 0) {
        std_msgs::msg::Int16MultiArray ch;
        ch.data = {                                                // [roll,pitch,thr,yaw,arm,mode,rtl,ch8]
          static_cast<int16_t>(crsf[0]), static_cast<int16_t>(crsf[1]),
          static_cast<int16_t>(crsf[2]), static_cast<int16_t>(crsf[3]),
          static_cast<int16_t>(crsf[4]), static_cast<int16_t>(crsf[5]),
          static_cast<int16_t>(crsf[6]), static_cast<int16_t>(crsf[7])};
        pub_crsf_tx_->publish(ch);
      }
    }

    last_ch5_armed_ = ch5_armed;

    // ---- IBVS 디버그 발행 — 구독자 있을 때만 (auto_save 녹화기가 구독) ----
    if (pub_ibvs_dbg_->get_subscription_count() > 0) {
      idbg.manual_out = manual_out;
      idbg.loom_size = loom_s_ema_;
      idbg.loom_rate = loom_rate_ema_;
      idbg.tau = std::isfinite(tau_sec_) ? std::min(tau_sec_, 9.99) : 9.99;
      idbg.roll_rate_cmd = roll_rate_cmd;
      idbg.pitch_rate_cmd = pitch_rate_cmd;
      idbg.roll_rate_out = std::clamp(roll_rate_cmd, -max_cmd_rate_dps_, max_cmd_rate_dps_);
      idbg.pitch_rate_out = std::clamp(pitch_rate_cmd, -max_cmd_rate_dps_, max_cmd_rate_dps_);
      idbg.rate_saturated = std::abs(roll_rate_cmd) > max_cmd_rate_dps_ ||
                            std::abs(pitch_rate_cmd) > max_cmd_rate_dps_;
      idbg.thrust = thrust;
      pub_ibvs_dbg_->publish(idbg);
    }

    // ---- Debug publish (UI 화살표용) — 구독자 있을 때만 ----
    if (pub_dbg_->get_subscription_count() > 0) {
      geometry_msgs::msg::Vector3 dbg;
      dbg.x = roll_rate_cmd;   // deg/s (각도 아님)
      dbg.y = pitch_rate_cmd;
      dbg.z = thrust;
      pub_dbg_->publish(dbg);
    }

    // ---- RTL one-shot ----
    // NOTE: land/RTL 전용 CRSF 채널은 제거됨(CH6는 flight mode). 자동 착륙은
    // 별도 경로(예: manual 모드 Altitude 전환 후 수동 착륙)로 처리 필요.
    if (state == State::RTL && !rtl_sent_) {
      rtl_sent_ = true;
      RCLCPP_INFO(get_logger(), "RTL state — no dedicated land channel (see NOTE).");
    }

    if (state == State::IDLE) {
      rtl_sent_ = false;
    }

    // ---- Publish mission state ----
    arms_msgs::msg::MissionState msg;
    msg.header.stamp = now_t;
    msg.state = to_string(state);
    msg.lock_elapsed_sec = static_cast<float>(sm_->lock_elapsed_sec());
    msg.lock_duration_sec = static_cast<float>(sm_->lock_duration_sec());
    msg.error_x = static_cast<float>(sm_->current_error_x());
    msg.error_y = static_cast<float>(sm_->current_error_y());
    msg.target_locked = sm_->target_locked();
    msg.armed = ch5_armed;                // UI 효과음/표시용 (실제 CH5 arm 상태)
    msg.manual_mode = joy_manual_mode_;   // UI 효과음/표시용 (모드)
    msg.prearm_blocked = prearm_blocked_; // UI 경고용 (스틱 미idle 로 arm 차단)
    // ELRS 링크 판정 = **바인딩+RF 링크가 실제로 살아있는가**.
    //   TX 모듈이 젯슨에 꽂혀 있으면 바인딩 안 돼도 CRSF 프레임(0x3A RadioSync 등)은
    //   계속 오므로, "프레임 수신"만으론 연결로 보면 안 된다. 실제 링크의 지표는
    //   LinkStatistics(0x14)의 **uplink LQ**(우리 명령이 RX 까지 도달하는 비율):
    //     · 바인딩 안 됨/범위 밖 → LQ=0 (또는 0x14 자체가 안 옴) → LOST
    //     · 바인딩+연결        → LQ>0 → OK
    const bool elrs_connected =
        (last_up_lq_ > 0) &&
        ((now_t - last_lq_time_).seconds() < telemetry_timeout_sec_);
    msg.elrs_connected = elrs_connected;  // UI 표시/효과음용 (연결/끊김)
    pub_state_->publish(msg);

    // ---- 발사 잠금장치 서보 ----
    //   상태 전이 엣지로만 판정한다(방아쇠 버튼이 아니라 LOCK→TRACK 전이가 열림 트리거).
    //     · IDLE 진입            → OPEN (무조건 열림, 기체 장착/탈거)
    //     · IDLE→SEARCH (auto)   → LOCK (상승엣지, 발사기 고정)
    //     · LOCK→TRACK (발사)    → OPEN (기체 놓아줌)
    //   in-tick 전이(arm/disarm 등) 이후의 최신 상태로 판정한다.
    State servo_state = sm_->state();
    if (!servo_init_) {
      servo_init_ = true;
      servo_->open();  // 시작 상태 IDLE → OPEN
    } else if (servo_state != servo_prev_state_) {
      if (servo_state == State::IDLE) {
        servo_->open();
      } else if (servo_prev_state_ == State::IDLE &&
                 servo_state == State::SEARCH && !joy_manual_mode_) {
        servo_->lock();
      } else if (servo_prev_state_ == State::LOCK &&
                 servo_state == State::TRACK) {
        servo_->open();
      }
    }
    servo_prev_state_ = servo_state;
  }

  // ----------------------------------------------------------------
  // Members
  // ----------------------------------------------------------------
  std::unique_ptr<StateMachine> sm_;
  std::unique_ptr<PIDController> pid_roll_;
  std::unique_ptr<PIDController> pid_pitch_;
  std::unique_ptr<ServoLock> servo_;

  // 서보 전이감지용 직전 상태 (제어 루프 상태와 별개로 서보 전이만 추적).
  State servo_prev_state_{State::IDLE};
  bool  servo_init_{false};

  double track_throttle_{0.66};
  double lead_gain_{0.0};
  double roll_sign_{1.0};
  double pitch_sign_{1.0};
  double mode0_aim_above_offset_{0.03};  // 정규화 영상 높이, 양수=표적보다 화면 위 조준
  double error_lpf_alpha_{0.25};
  double filt_err_x_{0.0};
  double filt_err_y_{0.0};
  double prev_err_x_{0.0};
  double prev_err_y_{0.0};
  double err_dot_x_{0.0};
  double err_dot_y_{0.0};
  double deadzone_{0.04};
  double deriv_lpf_alpha_{0.25};

  // 비전 looming(τ) 상태 — bbox 크기 EMA 와 팽창률 EMA 로 충돌까지 시간 추정
  double loom_size_alpha_{0.3};
  double loom_rate_alpha_{0.3};
  double loom_s_ema_{0.0};
  double loom_rate_ema_{0.0};
  bool   loom_init_{false};
  rclcpp::Time loom_last_time_;
  // τ(충돌까지 시간) — update_looming() 이 매번 갱신, control_loop() 의 추력 스케줄링이 재사용.
  //   기본값은 반드시 +inf(급함 없음). 0.0 이면 첫 검출 전에 추력이 hover 에 묶인다.
  double tau_sec_{std::numeric_limits<double>::infinity()};

  int dbg_count_{0};
  double control_rate_hz_{50.0};

  bool align_locked_{false};

  bool sitl_auto_launch_{false};
  bool hit_rtl_via_referee_{true};    // 심판 /arms/hit → RTL. SITL 지상진실 판정(실기체는 발행자 없음).
  bool looming_rtl_enabled_{false};   // 비전 τ → RTL 종말 경로(현재 비활성).
  double auto_launch_delay_sec_{0.5};
  bool lock_timer_started_{false};
  bool auto_launched_{false};
  rclcpp::Time lock_enter_time_;

  // Auto arm (IDLE→SEARCH 즉시 1회)
  bool auto_armed_{false};

  // 자동 모드에서 CH5(FC arm)를 켜는 상태 집합 (스위치와 분리, 컨트롤 노드가 결정)
  std::set<std::string> auto_arm_states_;

  // 수동 arm 래치 + pre-arm 스틱 idle 확인
  bool   manual_armed_{false};
  bool   prearm_blocked_{false};   // arm 스위치 올림 + 스틱 미idle → 차단 (UI 경고)
  bool   prearm_check_{true};
  double prearm_throttle_max_{-0.85};
  double prearm_stick_tol_{0.15};

  // Joy state
  std::array<float, 4> joy_axes_{};
  bool joy_arm_{false};
  bool joy_rtl_{false};
  bool joy_command_lost_{false};
  bool last_ch5_armed_{false};
  bool joy_manual_mode_{false};
  std::array<int, 5> prev_btn_{};
  // ARM 재토글 안전장치: true 면 ARM 스위치가 올라가 있어도 arm 안 됨.
  //   부팅 시 true(스위치 이미 올라가 있어도 즉시 arm 방지), 모드 전환 시 true,
  //   ARM 스위치를 DISARM 으로 내리면 false. → 재토글해야만 arm.
  bool require_arm_reset_{true};

  // CRSF 송수신 상태
  std::unique_ptr<CrsfOutput> crsf_out_;                          // CRSF UART 송수신기를 소유한다.
  std::array<bool, 256> crsf_seen_types_{};                       // 처음 발견한 텔레메트리 타입을 구분한다.
  arms_msgs::msg::CrsfTelemetry crsf_telem_;                      // /arms/crsf_rx 통합 발행용 최신 스냅샷
  // ELRS 링크(바인딩+RF) 판정: LinkStatistics(0x14)의 uplink LQ 로만 본다.
  int          last_up_lq_{-1};         // 마지막 0x14 uplink LQ[%] (-1=미수신)
  rclcpp::Time last_lq_time_;           // 마지막 0x14 수신 시각
  double       telemetry_timeout_sec_{2.0};  // 이 시간 내 0x14(LQ) 없으면 끊김
  std::size_t crsf_rx_bytes_{0};                                 // 누적 UART 수신량을 센다.
  std::size_t crsf_rx_valid_frames_{0};                          // 정상 텔레메트리 프레임 수를 센다.
  std::size_t crsf_rx_echoes_{0};                                // 자체 송신 에코 프레임 수를 센다.
  std::size_t crsf_rx_crc_errors_{0};                            // CRC 오류 프레임 수를 센다.
  std::size_t crsf_rx_framing_errors_{0};                        // 길이 오류 프레임 수를 센다.
  // 자동요격 제어(ACRO 각속도 + 유도). 각도 단계 없음.
  double max_rate_dps_{400.0};
  double hover_throttle_{0.32};
  double pursuit_gate_{0.25};
  bool   pursuit_center_boost_{true};   // 중앙 정렬시 추력증가 on/off (config)
  double err_dot_alpha_{0.2};
  double err_dot_clamp_{0.6};
  double max_cmd_rate_dps_{227.5};   // 유도 출력 각속도 상한[deg/s]
  bool   gain_shaping_{false};       // 중심 근처 게인 감쇠 (기본 off = 순수 P)
  // 유도(Guidance): 기본 추적(0) / 비례항법 PN(1) / IBVS 공선유도(2)
  int    guidance_mode_{2};
  // IBVS 공선유도 [mode 2] — 배리어 Lyapunov. 광축=추력축 기하라 자세 불필요.
  double ibvs_hfov_deg_{43.6028};    // 수평 화각[deg]
  double ibvs_aspect_{0.6666667};    // 세로/가로
  double ibvs_kx_{0.8000};             // = 2*tan(hfov/2)         (update_ibvs_scale 가 계산)
  double ibvs_ky_{0.5333};             // = 2*tan(hfov/2)*aspect
  // 카메라 캘리 (control.camera.*) — calibrated=true 면 위 kx/ky 대신 사용
  bool   cam_calibrated_{false};
  bool   cam_fisheye_{false};        // distortion_model == "equidistant"
  double cam_width_{720.0}, cam_height_{480.0};
  double cam_fx_{900.0}, cam_fy_{900.0};
  double cam_cx_{360.0}, cam_cy_{240.0};
  std::array<double, 5> cam_dist_{};  // plumb_bob: k1,k2,p1,p2,k3 / equidistant: k1..k4
  double ibvs_kb_{0.55};             // 배리어 한계 (z1 = 1-cos(theta) 기준)
  double ibvs_gain_{200.0};          // 배리어항 계수[deg/s]
  double ibvs_center_gain_{100.0};   // 선형항 계수[deg/s] (중앙 근처 권한 보강)
  double ibvs_los_rate_gain_{220.0}; // LOS각속도항 (횡단표적 리드) — 자기회전 오염 주의, 흔들리면 ↓
  double ibvs_z1_dbg_{0.0};          // 로그용 현재 z1
  double tau_ease_dbg_{1.0};         // 로그용 현재 τ 추력 감속 배율
  double pn_nav_gain_{175.0};
  double pn_center_gain_{52.5};
  double pn_alpha_{0.35};
  double pn_beta_{0.02};
  double pn_los_clamp_{1.5};
  // 근접도(화면 속 표적 크기) 기반 게인/스로틀 스케줄링
  double near_size_start_{0.15};
  double near_size_full_{0.35};
  double min_gain_scale_{0.4};
  double min_throttle_scale_{0.6};
  // τ 기반 추력 감속 (near_size 와 별개 — 크기가 아니라 접근속도를 반영)
  double tau_ease_start_{1.0};
  double tau_ease_full_{0.3};
  double tau_ease_min_scale_{0.3};
  // alpha-beta 표적 상태추정 (매끈한 LOS + LOS각속도)
  double losf_x_{0.0}, losf_y_{0.0};
  double losf_x_dot_{0.0}, losf_y_dot_{0.0};
  bool   pn_init_{false};

  rclcpp::Subscription<arms_msgs::msg::DetectionArray>::SharedPtr sub_detections_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_hit_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_reset_;
  rclcpp::Publisher<arms_msgs::msg::MissionState>::SharedPtr pub_state_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_dbg_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_loom_;
  rclcpp::Publisher<arms_msgs::msg::IbvsDebug>::SharedPtr pub_ibvs_dbg_;      // IBVS 디버그 → /arms/ibvs_debug
  rclcpp::Publisher<arms_msgs::msg::CrsfTelemetry>::SharedPtr pub_crsf_rx_;   // CRSF 수신 텔레메트리 → /arms/crsf_rx
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_crsf_tx_;  // CRSF 송신 채널 로그 → /arms/crsf_tx
  rclcpp::Publisher<arms_msgs::msg::GpsTelemetry>::SharedPtr pub_gps_;        // CRSF GPS → /arms/gps
  int battery_detected_cells_{0};    // 전압으로 자동 감지한 셀 수(첫 유효 전압에서 1회 감지 후 래치)
  double battery_cell_full_v_{4.2};  // 만충 셀당 전압[V]
  double battery_cell_empty_v_{3.5}; // 방전(0%) 셀당 전압[V]
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  double rkp_{455.0}, rki_{0}, rkd_{0};
  double pkp_{455.0}, pki_{0}, pkd_{0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_;

  bool rtl_sent_{false};
};

}  // namespace arms_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arms_control::ArmsControlNode>());
  rclcpp::shutdown();
  return 0;
}
