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
#include "arms_control/pn_guidance.hpp"
#include "arms_control/servo_lock.hpp"
#include "arms_control/state_machine.hpp"
#include "arms_msgs/msg/crsf_telemetry.hpp"
#include "arms_msgs/msg/detection_array.hpp"
#include "arms_msgs/msg/guidance_ground_truth.hpp"
#include "arms_msgs/msg/mission_state.hpp"
#include "arms_msgs/msg/pn_debug.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
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
    declare_parameter("mission.fire_align_tol", 0.2);  // FIRE 정렬 허용오차(가짜명중 방지)
    // 비전 looming(τ) 기반 FIRE — 거리센서 없이 bbox 팽창률로 충돌 임박을 판정.
    //   (3D 거리는 실기체에서 못 쓰므로 sim 에서도 제거. FIRE 는 τ 로만.)
    declare_parameter("mission.tau_fire_sec", 0.3);      // 충돌까지 시간(τ) 임계 [s]
    declare_parameter("mission.loom_s_min", 0.1);        // FIRE 최소 bbox 크기(정규화)
    declare_parameter("mission.loom_size_alpha", 0.3);   // bbox 크기 EMA (지터 억제)
    declare_parameter("mission.loom_rate_alpha", 0.3);   // 크기변화율(ṡ) EMA

    declare_parameter("control.roll_pid.kp", 130.0);
    declare_parameter("control.roll_pid.ki", 0.0);
    declare_parameter("control.roll_pid.kd", 1.0);
    declare_parameter("control.pitch_pid.kp", 130.0);
    declare_parameter("control.pitch_pid.ki", 0.0);
    declare_parameter("control.pitch_pid.kd", 1.0);
    declare_parameter("control.track_throttle", 0.70);
    declare_parameter("control.lead_gain", 0.0);
    declare_parameter("control.roll_sign", 1.0);
    declare_parameter("control.pitch_sign", 1.0);
    declare_parameter("control.error_lpf_alpha", 0.25);
    declare_parameter("control.control_rate_hz", 50.0);
    declare_parameter("control.deadzone", 0.04);
    declare_parameter("control.deriv_lpf_alpha", 0.25);

    declare_parameter("mission.sitl_auto_launch", true);
    // 충돌 판정(FIRE/RTL) 소스 — 서로 독립적인 두 스위치. 둘 다 환경 무관하게 동작한다.
    //   hit_rtl_via_referee: 심판 /arms/hit(지상진실 접촉)를 받으면 RTL.
    //       SITL 전용(Gazebo 접촉 심판이 /arms/hit 발행). 실기체엔 발행자가 없어 false.
    //   looming_fire_enabled: 비전 τ(bbox 팽창률)로 충돌 임박을 판정해 FIRE.
    //       실기체의 자동 요격 경로. 지금은 비활성(false), 실기체 검증되면 true 로 켠다.
    //   조합: SITL=(true,false) / 실기체 현재=(false,false) / 실기체 요격 켤 때=(false,true).
    declare_parameter("mission.hit_rtl_via_referee", true);
    declare_parameter("mission.looming_fire_enabled", false);
    declare_parameter("mission.auto_launch_delay_sec", 0.5);

    // 자동 모드 CH5(FC arm)를 켜는 상태 목록. 자동 모드에선 arm 을 스위치와 분리해
    //   컨트롤 노드가 상태머신 상태로 결정한다. 기본=발사(TRACK)부터 무장.
    //   나중에 SEARCH 등에서 arm 하도록 바꾸려면 이 목록만 수정하면 된다.
    declare_parameter("control.auto_arm_states", std::vector<std::string>{"SEARCH", "LOCK", "TRACK", "FIRE", "RTL"});

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
    declare_parameter("control.max_angle_deg", 55.0);  // PX4 Stabilized 풀스틱 목표 기울기
    // 추격 궤적(Pursuit): 정렬 기다리지 않고 조준하며 처음부터 상승.
    declare_parameter("control.hover_throttle", 0.42);  // 1.05kg, 모터 1.5kgf×4 이론 호버
    declare_parameter("control.pursuit_gate", 0.25);    // 오차 이 이하면 full 상승(공쪽으로 돌진)
    // pursuit_center_boost: true=중앙 정렬될수록 추력↑(center_q 스케일, 기존동작).
    //                       false=정렬 무관 상수 추력(track_throttle 고정) → "정렬시 추력증가" 비활성.
    declare_parameter("control.pursuit_center_boost", true);
    // 표적 속도추정(err_dot): mode 0의 리드와 gain_shaping 전용.
    // PN(mode 1)은 raw detection의 관성 LOS constant-rate KF를 별도로 쓴다.
    declare_parameter("control.err_dot_alpha", 0.2);    // 표적 속도추정 LPF(클수록 민첩/노이즈↑)
    declare_parameter("control.err_dot_clamp", 0.6);    // 속도추정 스파이크 제한[1/s]
    // 중심 근처 게인 감쇠(gain_scale / pitch_scale) 를 켤지. 기본 false = 순수 P.
    //   true 면 아래 두 가지가 켜진다:
    //     · 느린 표적(|LOS각속도| < 0.2) 이 중심 근처일 때 게인 감쇠 (0.08→x0.5, 0.20→x0.75)
    //     · roll 오차가 크면 pitch 게인 감쇠 (0.12→x0.8, 0.06→x0.9) = 좌우 정렬 우선
    //   이 숫자들은 구 물리량 시절 지터를 막으려고 손으로 넣은 값이고 현재 기체에서
    //   재검증된 적이 없다. deadzone 과 함께 "중심 근처에서 일부러 약해지는" 장치라,
    //   표적이 중앙에 안 붙는 증상을 만들 수 있다. 먼저 끄고 순수 P 를 본 뒤,
    //   지터가 실제로 보이면 그때 켠다.
    declare_parameter("control.gain_shaping", false);  // 유도 출력 각속도 상한[deg/s]
    // 유도 방식: 0=기본 추적, 1=관성 LOS-rate + looming Vc 비례항법.
    declare_parameter("control.guidance_mode", 0);
    declare_parameter("control.pn.navigation_ratio", 3.0);
    declare_parameter("control.pn.approach_accel_mps2", 5.0);
    declare_parameter("control.pn.target_diameter_m", 0.5);
    declare_parameter("control.pn.max_closing_speed_mps", 20.0);
    declare_parameter("control.pn.range_kf_accel_noise_mps2", 8.0);
    declare_parameter("control.pn.range_kf_measurement_noise_ratio", 0.04);
    declare_parameter("control.pn.range_kf_min_measurement_noise_m", 0.15);
    declare_parameter("control.pn.range_kf_innovation_gate_sigma", 4.0);
    declare_parameter("control.pn.kf_accel_noise", 0.8);
    declare_parameter("control.pn.kf_measurement_noise", 0.0025);
    declare_parameter("control.pn.max_lateral_accel_mps2", 12.0);
    declare_parameter("control.pn.max_measurement_age_sec", 0.20);
    declare_parameter("control.pn.use_ground_truth_los_rate", false);
    declare_parameter("control.pn.use_ground_truth_closing_speed", false);
    declare_parameter("control.pn.ground_truth_timeout_sec", 0.20);
    declare_parameter("control.pn.camera.fx", 231.16);
    declare_parameter("control.pn.camera.fy", 231.16);
    declare_parameter("control.pn.camera.cx", 360.0);
    declare_parameter("control.pn.camera.cy", 240.0);
    declare_parameter("control.pn.camera.width", 720.0);
    declare_parameter("control.pn.camera.height", 480.0);
    declare_parameter("control.pn.camera.r_bc", std::vector<double>{
      0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0});

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
    sm_params.fire_align_tol =
        get_parameter("mission.fire_align_tol").as_double();
    sm_params.tau_fire_sec =
        get_parameter("mission.tau_fire_sec").as_double();
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
    // 기본 추적 PID 출력도 PN과 같은 목표 기울기[deg]로 해석한다.
    max_angle_deg_ = get_parameter("control.max_angle_deg").as_double();
    gain_shaping_ = get_parameter("control.gain_shaping").as_bool();
    pid_roll_ = std::make_unique<PIDController>(
        get_parameter("control.roll_pid.kp").as_double(),
        get_parameter("control.roll_pid.ki").as_double(),
        get_parameter("control.roll_pid.kd").as_double(),
        max_angle_deg_);

    pid_pitch_ = std::make_unique<PIDController>(
        get_parameter("control.pitch_pid.kp").as_double(),
        get_parameter("control.pitch_pid.ki").as_double(),
        get_parameter("control.pitch_pid.kd").as_double(),
        max_angle_deg_);

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
    error_lpf_alpha_ = get_parameter("control.error_lpf_alpha").as_double();
    control_rate_hz_ = get_parameter("control.control_rate_hz").as_double();
    deadzone_ = get_parameter("control.deadzone").as_double();
    deriv_lpf_alpha_ = get_parameter("control.deriv_lpf_alpha").as_double();
    pid_roll_->set_deriv_alpha(deriv_lpf_alpha_);
    pid_pitch_->set_deriv_alpha(deriv_lpf_alpha_);

    sitl_auto_launch_ = get_parameter("mission.sitl_auto_launch").as_bool();
    hit_rtl_via_referee_ = get_parameter("mission.hit_rtl_via_referee").as_bool();
    looming_fire_enabled_ = get_parameter("mission.looming_fire_enabled").as_bool();
    auto_launch_delay_sec_ =
        get_parameter("mission.auto_launch_delay_sec").as_double();

    {
      const auto v = get_parameter("control.auto_arm_states").as_string_array();
      auto_arm_states_ = std::set<std::string>(v.begin(), v.end());
    }
    prearm_check_ = get_parameter("control.prearm_check").as_bool();
    prearm_throttle_max_ = get_parameter("control.prearm_throttle_max").as_double();
    prearm_stick_tol_ = get_parameter("control.prearm_stick_tol").as_double();

    hover_throttle_ = get_parameter("control.hover_throttle").as_double();
    pursuit_gate_ = get_parameter("control.pursuit_gate").as_double();
    pursuit_center_boost_ = get_parameter("control.pursuit_center_boost").as_bool();
    err_dot_alpha_ = get_parameter("control.err_dot_alpha").as_double();
    err_dot_clamp_ = get_parameter("control.err_dot_clamp").as_double();
    guidance_mode_ = static_cast<int>(get_parameter("control.guidance_mode").as_int());
    pn_params_.navigation_ratio = get_parameter("control.pn.navigation_ratio").as_double();
    pn_params_.approach_accel_mps2 = get_parameter("control.pn.approach_accel_mps2").as_double();
    pn_params_.target_diameter_m = get_parameter("control.pn.target_diameter_m").as_double();
    pn_params_.max_closing_speed_mps =
        get_parameter("control.pn.max_closing_speed_mps").as_double();
    pn_params_.range_kf_accel_noise_mps2 =
        get_parameter("control.pn.range_kf_accel_noise_mps2").as_double();
    pn_params_.range_kf_measurement_noise_ratio =
        get_parameter("control.pn.range_kf_measurement_noise_ratio").as_double();
    pn_params_.range_kf_min_measurement_noise_m =
        get_parameter("control.pn.range_kf_min_measurement_noise_m").as_double();
    pn_params_.range_kf_innovation_gate_sigma =
        get_parameter("control.pn.range_kf_innovation_gate_sigma").as_double();
    pn_params_.kf_accel_noise = get_parameter("control.pn.kf_accel_noise").as_double();
    pn_params_.kf_measurement_noise = get_parameter("control.pn.kf_measurement_noise").as_double();
    pn_params_.max_lateral_accel_mps2 =
        get_parameter("control.pn.max_lateral_accel_mps2").as_double();
    pn_params_.max_measurement_age_sec =
        get_parameter("control.pn.max_measurement_age_sec").as_double();
    use_gt_los_rate_ =
        get_parameter("control.pn.use_ground_truth_los_rate").as_bool();
    use_gt_closing_speed_ =
        get_parameter("control.pn.use_ground_truth_closing_speed").as_bool();
    gt_timeout_sec_ =
        get_parameter("control.pn.ground_truth_timeout_sec").as_double();
    pn_params_.hover_throttle = hover_throttle_;
    pn_params_.max_throttle = track_throttle_;
    pn_camera_.fx = get_parameter("control.pn.camera.fx").as_double();
    pn_camera_.fy = get_parameter("control.pn.camera.fy").as_double();
    pn_camera_.cx = get_parameter("control.pn.camera.cx").as_double();
    pn_camera_.cy = get_parameter("control.pn.camera.cy").as_double();
    pn_camera_.width = get_parameter("control.pn.camera.width").as_double();
    pn_camera_.height = get_parameter("control.pn.camera.height").as_double();
    const auto r_bc = get_parameter("control.pn.camera.r_bc").as_double_array();
    if (r_bc.size() == 9) std::copy(r_bc.begin(), r_bc.end(), pn_camera_.r_bc.v.begin());
    pn_guidance_.set_parameters(pn_params_);
    pn_guidance_.set_camera(pn_camera_);
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
          bool pn_changed = false;
          for (const auto& p : params) {
            const std::string& n = p.get_name();
            if (n == "control.roll_sign")
              roll_sign_ = p.as_double();
            else if (n == "control.pitch_sign")
              pitch_sign_ = p.as_double();
            else if (n == "control.track_throttle") {
              track_throttle_ = p.as_double();
              pn_params_.max_throttle = track_throttle_;
              pn_changed = true;
            }
            else if (n == "control.lead_gain")
              lead_gain_ = p.as_double();
            else if (n == "control.error_lpf_alpha")
              error_lpf_alpha_ = p.as_double();
            else if (n == "control.deadzone")
              deadzone_ = p.as_double();
            else if (n == "control.max_angle_deg") {
              max_angle_deg_ = p.as_double();
              pid_changed = true;
            }
            else if (n == "control.hover_throttle") {
              hover_throttle_ = p.as_double();
              pn_params_.hover_throttle = hover_throttle_;
              pn_changed = true;
            }
            else if (n == "control.pursuit_gate")
              pursuit_gate_ = p.as_double();
            else if (n == "control.pursuit_center_boost")
              pursuit_center_boost_ = p.as_bool();
            else if (n == "mission.hit_rtl_via_referee")
              hit_rtl_via_referee_ = p.as_bool();
            else if (n == "mission.looming_fire_enabled")
              looming_fire_enabled_ = p.as_bool();
            else if (n == "control.err_dot_alpha")
              err_dot_alpha_ = p.as_double();
            else if (n == "control.err_dot_clamp")
              err_dot_clamp_ = p.as_double();
            else if (n == "control.gain_shaping")
              gain_shaping_ = p.as_bool();
            else if (n == "control.guidance_mode")
              guidance_mode_ = static_cast<int>(p.as_int());
            else if (n == "control.pn.navigation_ratio") {
              pn_params_.navigation_ratio = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.approach_accel_mps2") {
              pn_params_.approach_accel_mps2 = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.target_diameter_m") {
              pn_params_.target_diameter_m = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.max_closing_speed_mps") {
              pn_params_.max_closing_speed_mps = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.range_kf_accel_noise_mps2") {
              pn_params_.range_kf_accel_noise_mps2 = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.range_kf_measurement_noise_ratio") {
              pn_params_.range_kf_measurement_noise_ratio = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.range_kf_min_measurement_noise_m") {
              pn_params_.range_kf_min_measurement_noise_m = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.range_kf_innovation_gate_sigma") {
              pn_params_.range_kf_innovation_gate_sigma = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.kf_accel_noise") {
              pn_params_.kf_accel_noise = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.kf_measurement_noise") {
              pn_params_.kf_measurement_noise = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.max_lateral_accel_mps2") {
              pn_params_.max_lateral_accel_mps2 = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.max_measurement_age_sec") {
              pn_params_.max_measurement_age_sec = p.as_double(); pn_changed = true;
            } else if (n == "control.pn.use_ground_truth_los_rate") {
              use_gt_los_rate_ = p.as_bool();
            } else if (n == "control.pn.use_ground_truth_closing_speed") {
              use_gt_closing_speed_ = p.as_bool();
            } else if (n == "control.pn.ground_truth_timeout_sec") {
              gt_timeout_sec_ = p.as_double();
            }
            else if (n == "mission.tau_fire_sec")
              sm_->set_tau_fire(p.as_double());
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
            }
          }
          if (pid_changed) {
            pid_roll_->set_gains(rkp_, rki_, rkd_, max_angle_deg_);
            pid_pitch_->set_gains(pkp_, pki_, pkd_, max_angle_deg_);
          }
          if (pn_changed) pn_guidance_.set_parameters(pn_params_);
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
    pub_pn_dbg_ = create_publisher<arms_msgs::msg::PnDebug>("/arms/pn_debug", 10);
    // looming 튜닝용: x=τ[s](미접근/무한대는 9.99로 캡), y=bbox크기(EMA), z=크기변화율 ṡ
    pub_loom_ = create_publisher<geometry_msgs::msg::Vector3>("/arms/debug_looming", 10);
    // CRSF 배터리 텔레메트리(0x08)를 UI 등에서 쓰도록 표준 메시지로 재발행한다.
    // CRSF 수신(RX) 텔레메트리 통합 발행: 배터리/자세(RPY)/수직속도/비행모드/링크통계 → /arms/crsf_rx.
    //   (배터리는 별도 /arms/battery 없이 이 토픽으로만 나간다. UI 도 이걸 구독.)
    pub_crsf_rx_ = create_publisher<arms_msgs::msg::CrsfTelemetry>("/arms/crsf_rx", 10);
    // CRSF 송신(TX) 채널 로그용(구독자 있을 때만). [roll,pitch,thr,yaw,arm,mode,kill,ch8] CRSF 단위.
    pub_crsf_tx_ = create_publisher<std_msgs::msg::Int16MultiArray>("/arms/crsf_tx", 10);

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

    // PN 경로는 detection 후처리와 독립적으로 raw bbox를 받아,
    // 영상 timestamp에 보간한 자세로 관성 LOS를 계산한 뒤 KF를 한 번만 적용한다.
    sub_detections_raw_ = create_subscription<arms_msgs::msg::DetectionArray>(
        "/arms/detections_raw", best_effort_qos,
        [this](arms_msgs::msg::DetectionArray::SharedPtr msg) {
          if (msg->detections.empty()) return;
          rclcpp::Time receipt = now();
          rclcpp::Time sample(msg->header.stamp, get_clock()->get_clock_type());
          if (sample.nanoseconds() == 0 || std::abs((receipt - sample).seconds()) > 2.0) {
            sample = receipt;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "raw detection timestamp가 ROS clock과 맞지 않아 수신 시각을 사용합니다");
          }
          pn::Quaternion q_nb;
          if (!pn_attitudes_.interpolate(sample.seconds(),
                                          pn_params_.max_measurement_age_sec, q_nb)) return;
          const bool need_gt = use_gt_los_rate_ || use_gt_closing_speed_;
          if (need_gt && !ground_truth_fresh(receipt)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "PN ground truth 옵션이 켜졌지만 /arms/guidance_ground_truth가 없거나 오래됨");
            return;
          }
          pn::Vec3 gt_los_rate;
          double gt_closing_speed = 0.0;
          const pn::Vec3 *los_rate_override = nullptr;
          const double *closing_speed_override = nullptr;
          if (use_gt_los_rate_) {
            gt_los_rate = {latest_gt_.los_rate_x, latest_gt_.los_rate_y,
                           latest_gt_.los_rate_z};
            los_rate_override = &gt_los_rate;
          }
          if (use_gt_closing_speed_) {
            gt_closing_speed = latest_gt_.closing_speed_mps;
            closing_speed_override = &gt_closing_speed;
          }
          const auto & b = msg->detections.front();
          pn_guidance_.update_detection(sample.seconds(), b.x_center, b.y_center,
                                        b.width, b.height, q_nb,
                                        los_rate_override, closing_speed_override);
        });

    sub_guidance_gt_ = create_subscription<arms_msgs::msg::GuidanceGroundTruth>(
        "/arms/guidance_ground_truth", rclcpp::SensorDataQoS(),
        [this](arms_msgs::msg::GuidanceGroundTruth::SharedPtr msg) {
          // referee는 두 pose가 아직 준비되지 않은 순간 valid=false인 0 메시지도
          // 발행한다. 그걸 저장하면 pn_debug GT 곡선이 중간중간 0으로 떨어진다.
          // 마지막 유효 샘플만 유지하고 freshness는 수신 시각으로 별도 판정한다.
          if (!msg->valid) return;
          latest_gt_ = *msg;
          gt_receipt_time_ = now();
          gt_received_ = true;
        });

    sub_attitude_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/arms/attitude_quaternion", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
          rclcpp::Time t(msg->header.stamp, get_clock()->get_clock_type());
          if (t.nanoseconds() == 0) t = now();
          pn_attitudes_.push(t.seconds(), {msg->quaternion.w, msg->quaternion.x,
                                          msg->quaternion.y, msg->quaternion.z});
        });

    sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/arms/camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          if (msg->k[0] <= 1.0 || msg->k[4] <= 1.0) return;
          pn_camera_.fx = msg->k[0]; pn_camera_.fy = msg->k[4];
          pn_camera_.cx = msg->k[2]; pn_camera_.cy = msg->k[5];
          pn_camera_.width = msg->width; pn_camera_.height = msg->height;
          pn_guidance_.set_camera(pn_camera_);
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
          int kill = btn(0), arm = btn(1), mode = btn(2), launch = btn(3);

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

          const bool kill_edge = kill && !prev_btn_[0];   // kill 상승엣지
          joy_kill_ = static_cast<bool>(kill);
          joy_arm_  = static_cast<bool>(arm);
          // ARM 스위치를 기본 위치(DISARM)로 내리면 재토글 래치 해제.
          if (!joy_arm_) require_arm_reset_ = false;
          // KILL 이 걸리면 재무장 래치를 세운다(위 해제보다 우선) → kill 을 내려도
          //   arm 스위치를 재토글(DOWN→UP)해야만 다시 무장. 실수로 즉시 재무장 방지.
          if (kill_edge) {
            require_arm_reset_ = true;
            RCLCPP_WARN(get_logger(), "KILL — 강제 disarm (재무장하려면 ARM 재토글)");
          }
          prev_btn_[0] = kill; prev_btn_[1] = arm;
          prev_btn_[2] = mode; prev_btn_[3] = launch;
        });

    // 심판(referee)의 직격 통지 — 로그만 남기고 상태머신은 건드리지 않는다.
    //
    // 예전에는 이게 on_external_hit() 로 들어가 TRACK/LOCK 을 곧장 FIRE 로
    // 밀었다. 그러면 영상 파이프라인이 한 번도 발사 판정을 못 해도 미션이
    // 성공으로 끝나서, 검출/τ 판정이 고장난 걸 심판이 덮어버린다. 채점자가
    // 응시자 답을 대신 써 주는 배선이라 성능 측정이 오염된다.
    //
    // FIRE 는 이제 on_vision_tick(τ, bbox) 한 경로로만 들어간다. 심판은
    // 그것과 독립적으로 "실제로 닿았는가"만 보고한다 — 둘이 어긋나는 것
    // 자체가 정보다(닿았는데 FIRE 없음 = 판정 실패, 그 반대 = 오발).
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
          pn_guidance_.reset();
          pn_attitudes_.clear();
          align_locked_ = false;
          auto_armed_ = manual_armed_ = false;
          lock_timer_started_ = auto_launched_ = false;
          fire_sent_ = rtl_sent_ = false;
          joy_kill_ = false;
          joy_arm_ = false;
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
  // 비전 looming: bbox 크기의 팽창률로 충돌까지 시간 τ 를 추정해 FIRE 판정에 넘긴다.
  //   τ = s/ṡ = d/v_closing. 초점거리·표적 실제크기가 소거되므로 카메라 스펙·표적
  //   크기 모두에 무관하다(작은 x500도, 큰 풍선도 같은 로직으로 접촉 직전 발사).
  void update_looming(const arms_msgs::msg::DetectionArray::SharedPtr & msg) {
    if (msg->detections.empty()) { loom_init_ = false; return; }  // 표적 소실 → 추정 리셋
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
    // 닫히는 중(ṡ>0)일 때만 τ 유효. 아니면 무한대로 두어 FIRE 안 되게.
    double tau = (loom_rate_ema_ > 1e-4)
                   ? (loom_s_ema_ / loom_rate_ema_)
                   : std::numeric_limits<double>::infinity();
    // 비전 τ FIRE 는 looming_fire_enabled 일 때만. 꺼져 있으면(현재 실기체·SITL)
    //   τ 로 FIRE 하지 않는다 — TRACK 유지하며 추격만 하고 FIRE 상태엔 진입 안 함.
    //   SITL 은 대신 심판 /arms/hit 로 RTL, 실기체는 지금 자동 요격을 아직 안 켬.
    if (looming_fire_enabled_) {
      sm_->on_looming(tau, loom_s_ema_);
    }

    geometry_msgs::msg::Vector3 lm;
    lm.x = std::isfinite(tau) ? std::min(tau, 9.99) : 9.99;  // 무한대(미접근)는 9.99로 표시
    lm.y = loom_s_ema_;
    lm.z = loom_rate_ema_;
    if (pub_loom_->get_subscription_count() > 0)  // 튜닝용: 구독자 있을 때만 발행
      pub_loom_->publish(lm);
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
        const double pitch = be16(0) / 10000.0;
        const double roll = be16(2) / 10000.0;
        const double yaw = be16(4) / 10000.0;
        crsf_telem_.pitch_deg = static_cast<float>(pitch * pn::kRadToDeg);
        crsf_telem_.roll_deg  = static_cast<float>(roll * pn::kRadToDeg);
        crsf_telem_.yaw_deg   = static_cast<float>(yaw * pn::kRadToDeg);
        // 실기체에서는 CRSF ATTITUDE가 PN의 자세 타임라인이다.
        pn_attitudes_.push(now().seconds(), pn::Quaternion::from_euler(roll, pitch, yaw));
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

  bool ground_truth_fresh(const rclcpp::Time &t) const {
    if (!gt_received_ || !latest_gt_.valid) return false;
    const double age = (t - gt_receipt_time_).seconds();
    return age >= 0.0 && age <= gt_timeout_sec_;
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

    // 유도 출력 = PX4 Stabilized attitude controller에 보낼 목표 각도[deg].
    double roll_cmd_deg = 0.0;
    double pitch_cmd_deg = 0.0;
    float thrust = 0.f;

    if (state == State::TRACK || state == State::FIRE) {
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
      double ey = apply_deadzone(los_y, deadzone_);
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
      pid_roll_->set_integral_frozen(held);
      pid_pitch_->set_integral_frozen(held);

      if (guidance_mode_ == 1) {
        // ===== 관성 LOS-rate PN → desired attitude → PX4 attitude controller =====
        // center-holding은 의도적으로 없다. yaw는 CH4=0 rate로 초기 방향을 유지한다.
        const auto & out = pn_guidance_.output();
        const bool fresh = out.valid &&
            (now_t.seconds() - out.stamp_sec) >= 0.0 &&
            (now_t.seconds() - out.stamp_sec) <= pn_params_.max_measurement_age_sec;
        if (fresh) {
          const auto & rd = out.attitude_desired_nb;
          roll_cmd_deg = std::atan2(rd(2, 1), rd(2, 2)) * pn::kRadToDeg;
          pitch_cmd_deg =
              std::asin(std::clamp(-rd(2, 0), -1.0, 1.0)) * pn::kRadToDeg;
          thrust = static_cast<float>(out.throttle);
        } else {
          // 새 측정이 없으면 예측만으로 진격하지 않고 level + hover로 유지.
          roll_cmd_deg = pitch_cmd_deg = 0.0;
          thrust = static_cast<float>(hover_throttle_);
        }
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
        roll_cmd_deg = roll_sign_ * gain_scale * pid_roll_->compute(ex_lead, dt);
        double pitch_scale = 1.0;
        if (gain_shaping_) {          // roll 오차가 크면 pitch 를 눌러 좌우 정렬 우선
          if (fabs(ex) > 0.12)      pitch_scale = 0.8;
          else if (fabs(ex) > 0.06) pitch_scale = 0.9;
        }
        // image down(+ey)=body forward이므로 목표 Euler pitch는 기수 하강(-)이다.
        // 아래 CRSF 변환에서 PX4 stick convention에 맞게 다시 반전되므로 mode 0의
        // 실제 채널 방향은 기존과 동일하다.
        pitch_cmd_deg = -pitch_sign_ * gain_scale * pitch_scale *
                        pid_pitch_->compute(ey_lead, dt);
      }

      // ---- 추격 궤적(Pursuit): 조준하며 처음부터 상승 ----
      //   정렬 완료를 기다리지 않는다. 추력은 기체가 기울어진 방향(=공을 겨눈 방향)으로
      //   나가므로, "공쪽으로 기울이기(추적)" + "상승" 을 동시에 하면 드론이 공을 향해
      //   대각선으로 날아간다(추격). 공을 잘 겨눌수록(center_q↑) 그쪽으로 더 세게 상승하고,
      //   빗나가면 상승을 줄여(hover) 재조준을 우선한다. → 수직으로 떴다 늦게 꺾는 문제 해결.
      if (guidance_mode_ == 0) {
        const double up = track_throttle_;      // 잘 겨눴을 때 최대 상승(=공쪽 돌진)
        const double hover = hover_throttle_;   // 빗나갔을 때 최소(재조준)
        double emag = std::hypot(filt_err_x_, filt_err_y_);
        if (pursuit_center_boost_) {
          // 중앙 정렬될수록(center_q↑) 그쪽으로 더 세게 상승, 빗나가면 hover 로 낮춰 재조준 우선.
          double center_q =
              std::clamp(1.0 - emag / std::max(pursuit_gate_, 1e-3), 0.0, 1.0);
          thrust = static_cast<float>(hover + (up - hover) * center_q);
        } else {
          // 비활성화: 정렬 정도와 무관하게 상수 추력(track_throttle) 유지.
          thrust = static_cast<float>(up);
        }

        if (!align_locked_ && emag < 0.15) {
          align_locked_ = true;
          RCLCPP_INFO(get_logger(), "추격 시작 (오차 %.2f) → 공 향해 대각선 상승", emag);
        }
      }

      if (++dbg_count_ % 6 == 0) {
        const auto & pn_out = pn_guidance_.output();
        RCLCPP_INFO(get_logger(),
                    "TRACK[%s] err=(%.2f,%.2f) losdot=(%.2f,%.2f) R=%.1fm Vc=%.1fm/s attitude=(%.1f,%.1f)deg thr=%.2f",
                    guidance_mode_ == 1 ? "PN" : "PUR",
                    filt_err_x_, filt_err_y_,
                    guidance_mode_ == 1 ? pn_out.los_rate_ned.x : err_dot_x_,
                    guidance_mode_ == 1 ? pn_out.los_rate_ned.y : err_dot_y_,
                    guidance_mode_ == 1 ? pn_out.range_m : 0.0,
                    guidance_mode_ == 1 ? pn_out.closing_speed_mps : 0.0,
                    roll_cmd_deg, pitch_cmd_deg, thrust);
      }
    } else {
      // IDLE / SEARCH / LOCK / RTL
      pid_roll_->reset();
      pid_pitch_->reset();
      filt_err_x_ = 0.0;
      filt_err_y_ = 0.0;
      thrust = 0.f;
      align_locked_ = false;
      pn_guidance_.reset();
    }

    // ---- 유효 스위치 = 상태 스위치 && 재토글 래치 해제 ----
    //   모드 전환/부팅 직후엔 스위치가 올라가 있어도 즉시 적용 안 됨 → 재토글(DOWN→UP) 필요.
    //   · 자동: idle/search 게이트에 적용 (기본 IDLE, SEARCH 재토글 안전장치)
    //   · 수동: CH5 arm 에 적용 (기본 DISARM, arm 재토글 안전장치)
    //   ※ 자동 CH5 arm 자체는 스위치와 분리되어 상태머신 상태(auto_arm_states_)로 결정한다.
    const bool effective_arm = joy_arm_ && !require_arm_reset_;

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

    // ---- RTL 핸드오버 ----
    //   FIRE 후 RTL 에서는 조종사가 수동 스위치를 안 올려도 자동으로 자동수평 모드(CH6 high)
    //   + 스틱 passthrough 로 넘어간다. 자동수평은 고도유지가 없으므로(throttle 직결)
    //   조종사가 throttle·스틱으로 직접 몰고 와 착륙시킨다.
    const bool rtl_handover = (state == State::RTL);
    const bool manual_out = joy_manual_mode_ || rtl_handover;

    // ---- CH5(FC arm) 결정 ----
    //   KILL: 최우선 — 눌려 있으면 무조건 disarm(CH5 low). 즉시 모터 정지.
    //   RTL: 이미 비행 중이므로 무조건 armed 유지 (착지 후 auto-disarm).
    //   수동: manual_armed_ (재토글 안전장치 + pre-arm 스틱 idle 확인).
    //   자동: 스위치와 분리 — 상태머신 상태 기반(auto_arm_states_)으로 컨트롤 노드가 결정.
    const bool ch5_armed =
        joy_kill_          ? false
        : rtl_handover     ? true
        : joy_manual_mode_ ? manual_armed_
                           : (auto_arm_states_.count(to_string(state)) > 0);

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
        // AUTO(영상유도): 목표 roll/pitch 각도를 풀스틱 각도로 정규화한다.
        // PX4 Stabilized가 이 입력을 attitude setpoint로 해석하고 내부 자세/각속도
        // 루프를 모두 수행한다.
        const double angle_limit = std::max(max_angle_deg_, 1.0);
        const double roll = std::clamp(roll_cmd_deg, -angle_limit, angle_limit);
        const double pitch = std::clamp(pitch_cmd_deg, -angle_limit, angle_limit);
        crsf[0] = CrsfOutput::norm_to_crsf(roll / angle_limit);
        // Euler pitch(NED/FRD)는 음수=기수 하강이지만 PX4 manual pitch stick은
        // 양수=스틱 전진=기수 하강이다. 따라서 desired Euler pitch를 반전한다.
        crsf[1] = CrsfOutput::norm_to_crsf(-pitch / angle_limit);
        crsf[2] = CrsfOutput::thr_to_crsf(static_cast<double>(thrust));
        crsf[3] = CrsfOutput::CRSF_CENTER;  // yaw 중앙 = yaw rate 0(헤딩 유지)
      }

      // CH5: arm — 수동은 스위치(재토글 안전장치), 자동은 상태머신 상태 기반.
      //   자동: 기본 발사(TRACK)부터 arm (control.auto_arm_states 로 조정).
      crsf[4] = ch5_armed ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

      // CH6: flight mode — auto=ACRO/각속도(172, low), manual|RTL=자동수평(1811, high).
      //   FC 를 CH6 high=자동수평(self-level), low=ACRO 로 매핑해 둔다.
      //   RTL 이면 자동으로 자동수평(수동)으로 넘어가 조종사가 이어받아 착륙.
      crsf[5] = CrsfOutput::CRSF_MAX;  // 자동/수동 모두 PX4 Stabilized 자세 제어

      // CH7: kill 지시(참고용). 실제 kill 은 CH5 를 강제 disarm 해 처리하므로 FC 매핑 불필요.
      crsf[6] = joy_kill_ ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

      // CH8: 미사용 (crsf.fill(CRSF_MIN)으로 172 고정)

      if (!crsf_out_->send(crsf)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "CRSF TX failed: check UART port and baud rate.");      // 송신 실패를 과도한 반복 없이 알린다.
      }

      // 송신 CRSF 채널 로그(구독자 있을 때만; bag record 가 구독하면 자동 on).
      if (pub_crsf_tx_->get_subscription_count() > 0) {
        std_msgs::msg::Int16MultiArray ch;
        ch.data = {                                                // [roll,pitch,thr,yaw,arm,mode,kill,ch8]
          static_cast<int16_t>(crsf[0]), static_cast<int16_t>(crsf[1]),
          static_cast<int16_t>(crsf[2]), static_cast<int16_t>(crsf[3]),
          static_cast<int16_t>(crsf[4]), static_cast<int16_t>(crsf[5]),
          static_cast<int16_t>(crsf[6]), static_cast<int16_t>(crsf[7])};
        pub_crsf_tx_->publish(ch);
      }
    }

    // ---- Debug publish (UI 화살표용) — 구독자 있을 때만 ----
    if (pub_dbg_->get_subscription_count() > 0) {
      geometry_msgs::msg::Vector3 dbg;
      dbg.x = roll_cmd_deg;   // desired attitude [deg]
      dbg.y = pitch_cmd_deg;
      dbg.z = thrust;
      pub_dbg_->publish(dbg);
    }

    // PN 추정·유도·자세 외부루프를 한 토픽으로 녹화해 오프라인에서 비교한다.
    if (pub_pn_dbg_->get_subscription_count() > 0) {
      const auto & out = pn_guidance_.output();
      arms_msgs::msg::PnDebug dbg;
      dbg.header.stamp = now_t;
      dbg.valid = out.valid;
      dbg.los_x = out.los_ned.x; dbg.los_y = out.los_ned.y; dbg.los_z = out.los_ned.z;
      dbg.los_rate_x = out.los_rate_ned.x;
      dbg.los_rate_y = out.los_rate_ned.y;
      dbg.los_rate_z = out.los_rate_ned.z;
      dbg.bbox_scale = out.bbox_scale;
      dbg.range_raw_m = out.range_raw_m;
      dbg.range_m = out.range_m;
      dbg.closing_speed_raw_mps = out.closing_speed_raw_mps;
      dbg.closing_speed_mps = out.closing_speed_mps;
      dbg.range_innovation_m = out.range_innovation_m;
      dbg.range_measurement_accepted = out.range_measurement_accepted;
      dbg.pn_accel_x = out.acceleration_pn_ned.x;
      dbg.pn_accel_y = out.acceleration_pn_ned.y;
      dbg.pn_accel_z = out.acceleration_pn_ned.z;
      dbg.guidance_accel_x = out.acceleration_guidance_ned.x;
      dbg.guidance_accel_y = out.acceleration_guidance_ned.y;
      dbg.guidance_accel_z = out.acceleration_guidance_ned.z;
      const auto & rd = out.attitude_desired_nb;
      dbg.desired_roll_deg = std::atan2(rd(2, 1), rd(2, 2)) * pn::kRadToDeg;
      dbg.desired_pitch_deg =
          std::asin(std::clamp(-rd(2, 0), -1.0, 1.0)) * pn::kRadToDeg;
      pn::AttitudeSample actual;
      if (pn_attitudes_.latest(actual)) {
        const auto ra = actual.q_nb.rotation_matrix();
        dbg.actual_roll_deg = std::atan2(ra(2, 1), ra(2, 2)) * pn::kRadToDeg;
        dbg.actual_pitch_deg =
            std::asin(std::clamp(-ra(2, 0), -1.0, 1.0)) * pn::kRadToDeg;
      }
      dbg.roll_cmd_deg = roll_cmd_deg;
      dbg.pitch_cmd_deg = pitch_cmd_deg;
      dbg.throttle = thrust;
      const bool gt_fresh = ground_truth_fresh(now_t);
      dbg.ground_truth_valid = gt_fresh;
      dbg.use_ground_truth_los_rate = use_gt_los_rate_;
      dbg.use_ground_truth_closing_speed = use_gt_closing_speed_;
      // fresh가 아니어도 마지막 유효 GT 값은 그래프 연속성을 위해 유지한다.
      // ground_truth_valid가 false이므로 제어에는 사용되지 않고 stale 여부도 구분된다.
      if (gt_received_) {
        dbg.ground_truth_los_x = latest_gt_.los_x;
        dbg.ground_truth_los_y = latest_gt_.los_y;
        dbg.ground_truth_los_z = latest_gt_.los_z;
        dbg.ground_truth_los_rate_x = latest_gt_.los_rate_x;
        dbg.ground_truth_los_rate_y = latest_gt_.los_rate_y;
        dbg.ground_truth_los_rate_z = latest_gt_.los_rate_z;
        dbg.ground_truth_range_m = latest_gt_.range_m;
        dbg.ground_truth_closing_speed_mps = latest_gt_.closing_speed_mps;
      }
      pub_pn_dbg_->publish(dbg);
    }

    // ---- FIRE one-shot ----
    // FIRE 는 looming_fire_enabled=true 일 때만 진입한다(꺼져 있으면 on_looming 게이트에서 차단).
    if (state == State::FIRE && !fire_sent_) {
      fire_sent_ = true;
      RCLCPP_INFO(get_logger(), "FIRE state — payload trigger signaled via mission_state.");
      sm_->on_fire_complete();
    }

    // ---- RTL one-shot ----
    // NOTE: land/RTL 전용 CRSF 채널은 제거됨(CH6는 flight mode). 자동 착륙은
    // 별도 경로(예: manual 모드 Altitude 전환 후 수동 착륙)로 처리 필요.
    if (state == State::RTL && !rtl_sent_) {
      rtl_sent_ = true;
      RCLCPP_INFO(get_logger(), "RTL state — no dedicated land channel (see NOTE).");
    }

    if (state == State::IDLE) {
      fire_sent_ = false;
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

  double track_throttle_{0.70};
  double lead_gain_{0.0};
  double roll_sign_{1.0};
  double pitch_sign_{1.0};
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

  int dbg_count_{0};
  double control_rate_hz_{50.0};

  bool align_locked_{false};

  bool sitl_auto_launch_{false};
  bool hit_rtl_via_referee_{true};    // 심판 /arms/hit → RTL. SITL 지상진실 판정(실기체는 발행자 없음).
  bool looming_fire_enabled_{false};  // 비전 τ → FIRE. 실기체 자동 요격 경로(현재 비활성).
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
  bool joy_kill_{false};
  bool joy_arm_{false};
  bool joy_manual_mode_{false};
  std::array<int, 4> prev_btn_{};
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
  // 자동요격 목표 자세 출력(PX4 Stabilized attitude controller).
  double max_angle_deg_{55.0};
  double hover_throttle_{0.42};
  double pursuit_gate_{0.25};
  bool   pursuit_center_boost_{true};   // 중앙 정렬시 추력증가 on/off (config)
  double err_dot_alpha_{0.2};
  double err_dot_clamp_{0.6};
  bool   gain_shaping_{false};       // 중심 근처 게인 감쇠 (기본 off = 순수 P)
  // 유도(Guidance): 기본 추적(0) / 비례항법 PN(1)
  int    guidance_mode_{0};
  pn::PnParameters pn_params_{};
  pn::CameraModel pn_camera_{};
  pn::PnGuidance pn_guidance_{};
  pn::AttitudeBuffer pn_attitudes_{};
  bool use_gt_los_rate_{false};
  bool use_gt_closing_speed_{false};
  double gt_timeout_sec_{0.20};
  bool gt_received_{false};
  rclcpp::Time gt_receipt_time_{};
  arms_msgs::msg::GuidanceGroundTruth latest_gt_{};

  rclcpp::Subscription<arms_msgs::msg::DetectionArray>::SharedPtr sub_detections_;
  rclcpp::Subscription<arms_msgs::msg::DetectionArray>::SharedPtr sub_detections_raw_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_attitude_;
  rclcpp::Subscription<arms_msgs::msg::GuidanceGroundTruth>::SharedPtr sub_guidance_gt_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_hit_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_reset_;
  rclcpp::Publisher<arms_msgs::msg::MissionState>::SharedPtr pub_state_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_dbg_;
  rclcpp::Publisher<arms_msgs::msg::PnDebug>::SharedPtr pub_pn_dbg_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_loom_;
  rclcpp::Publisher<arms_msgs::msg::CrsfTelemetry>::SharedPtr pub_crsf_rx_;   // CRSF 수신 텔레메트리 → /arms/crsf_rx
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_crsf_tx_;  // CRSF 송신 채널 로그 → /arms/crsf_tx
  int battery_detected_cells_{0};    // 전압으로 자동 감지한 셀 수(첫 유효 전압에서 1회 감지 후 래치)
  double battery_cell_full_v_{4.2};  // 만충 셀당 전압[V]
  double battery_cell_empty_v_{3.5}; // 방전(0%) 셀당 전압[V]
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  double rkp_{130.0}, rki_{0}, rkd_{1.0};
  double pkp_{130.0}, pki_{0}, pkd_{1.0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_;

  bool fire_sent_{false};
  bool rtl_sent_{false};
};

}  // namespace arms_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arms_control::ArmsControlNode>());
  rclcpp::shutdown();
  return 0;
}
