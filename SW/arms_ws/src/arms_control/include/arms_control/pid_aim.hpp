#pragma once

#include "arms_control/pn_guidance.hpp"

namespace arms_control::pid_aim {

/**
 * PID 관성 조준점 (control.pid_aim.mode = inertial).
 *
 * 예전 PID 는 "표적을 화면 중앙 + 고정 오프셋(aim_above_offset, 중력 보상)"에 두는
 * 자세 추종(attitude pursuit)이었다. 그 구조는 추력 '방향'만 맞출 뿐 LOS 에 수직인
 * 속도(발사 직후 수직 상승분, 중력 보상 부족분, 추종 지연)를 없애는 항이 없어서,
 * 화면 오차를 아무리 잘 0 으로 만들어도 공 옆을 지나간다. 게인을 바꿔도 궤적이
 * 거의 같은 이유다. 또 재투영 오차의 미분(err_dot)은 '관성 LOS 회전 − 기체 회전'이라
 * 기체가 표적을 잘 따라갈수록 0 이 되어, 빗나감의 원인인 관성 LOS 회전을 보지 못한다.
 *
 * 여기서는 조준점을 매 틱 관성 유도 법칙으로 계산한다. 목표 비추력(specific thrust)
 *     f = A·l̂ + N·Vc·λ̇ − g          (NED, g = +9.81 ẑ)
 * 의 방향 d̂ 가 곧 카메라 광축(= 추력축 body −z)이 향해야 할 방향이다.
 *   · A·l̂      : LOS 방향 접근 가속 (근접·τ 감속은 이 항에만 곱한다)
 *   · N·Vc·λ̇  : 비례항법 횡가속 — 관성 LOS 회전을 0 으로 만든다(빗나감 보정의 핵심)
 *   · −g       : 중력 상쇄 — 예전 g_off·aim_above_offset 를 대체하는 물리식
 * PID 입력은 "d̂ 의 화면 위치"(tan 좌표)다. 이것이 0 이면 광축이 d̂ 를 향한다.
 * PID 는 이제 안쪽 '광축 → d̂' 추종 루프일 뿐이고, 궤적은 바깥 유도 법칙이 정한다.
 * 추가로 d̂ 가 관성 공간에서 도는 속도(≈ LOS 회전)를 각속도 피드포워드로 내서
 * P 만으로 생기던 정상상태 추종 지연(오차 = LOS 회전 / kp)을 없앤다.
 */
struct Parameters {
  double navigation_ratio{3.0};
  double approach_accel_mps2{6.0};
  // bbox 크기로 추정한 접근속도는 발사 직후 0 에 가깝다. 그 동안에도 LOS 회전을
  // 죽이도록 PN 항에 쓰는 Vc 의 하한을 둔다.
  double min_closing_speed_mps{3.0};
  double max_lateral_accel_mps2{12.0};
  // 광축이 LOS 에서 벗어날 수 있는 최대 각 [rad]. 0 = 제약 없음.
  double max_look_angle_rad{0.0};
  double hover_throttle{0.25};
};

struct Input {
  pn::Vec3 los_c{0.0, 0.0, 1.0};  // 현재 자세 기준 optical 좌표 표적 방향 (정규화 불필요)
  pn::Mat3 r_nb{};                // body FRD → world NED (현재 자세)
  pn::Mat3 r_bc{};                // optical → body FRD
  bool los_rate_valid{false};
  pn::Vec3 los_rate_ned{};        // dl̂/dt [1/s], 관성
  double closing_speed_mps{0.0};
  double approach_scale{1.0};     // 0~1, 접근 가속에만 곱한다(근접·τ 감속)
};

struct Output {
  bool valid{false};
  double err_x{0.0}, err_y{0.0};  // d̂ 의 화면 위치 [tan] = PID 입력
  double aim_x{0.0}, aim_y{0.0};  // 표적이 놓여야 할 화면 위치(= 표적 − err, 표시용 등가 조준점)
  double ff_x{0.0}, ff_y{0.0};    // 관성 LOS 회전에 의한 화면 이동속도 [tan/s ≈ rad/s]
  pn::Vec3 accel_pn_ned{};
  double closing_speed_used_mps{0.0};
  // 실제 광축 방향 성분으로 f 를 냈을 때의 물리 추력 (hover_throttle 기준, 상한 없음)
  double throttle{0.0};
  bool look_limited{false};
  double look_angle_deg{0.0};     // 최종 d̂ 와 LOS 사이 각
};

Output compute(const Parameters &params, const Input &input);

}  // namespace arms_control::pid_aim
