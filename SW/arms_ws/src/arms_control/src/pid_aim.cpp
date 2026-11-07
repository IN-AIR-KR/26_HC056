#include "arms_control/pid_aim.hpp"

#include <algorithm>

namespace arms_control::pid_aim {

namespace {

// m^T · p (회전행렬 역변환)
pn::Vec3 mul_t(const pn::Mat3 &m, const pn::Vec3 &p) {
  return {m(0, 0) * p.x + m(1, 0) * p.y + m(2, 0) * p.z,
          m(0, 1) * p.x + m(1, 1) * p.y + m(2, 1) * p.z,
          m(0, 2) * p.x + m(1, 2) * p.y + m(2, 2) * p.z};
}

// 광축 뒤쪽/옆으로 넘어간 방향도 화면 좌표가 발산하지 않게 z 하한을 둔다.
constexpr double kMinProjZ = 0.2;

}  // namespace

Output compute(const Parameters &params, const Input &input) {
  Output out;
  const pn::Vec3 l_c = pn::normalized(input.los_c);
  if (!pn::finite(l_c) || l_c.z <= 0.05) return out;

  const auto to_ned = [&](const pn::Vec3 &v) { return input.r_nb * (input.r_bc * v); };
  const auto to_cam = [&](const pn::Vec3 &v) { return mul_t(input.r_bc, mul_t(input.r_nb, v)); };
  const pn::Vec3 l_ned = to_ned(l_c);

  // ---- 비례항법 횡가속 ----
  pn::Vec3 a_pn{};
  if (input.los_rate_valid && pn::finite(input.los_rate_ned)) {
    const double vc = std::max(input.closing_speed_mps, params.min_closing_speed_mps);
    // 필터 출력은 이미 LOS 수직이지만, 지금 LOS(재투영)와 필터 LOS 가 약간 다를 수 있다.
    const pn::Vec3 rate = input.los_rate_ned - l_ned * pn::dot(input.los_rate_ned, l_ned);
    a_pn = params.navigation_ratio * vc * rate;
    const double lateral = pn::norm(a_pn);
    if (params.max_lateral_accel_mps2 > 0.0 && lateral > params.max_lateral_accel_mps2) {
      a_pn = a_pn * (params.max_lateral_accel_mps2 / lateral);
    }
    out.closing_speed_used_mps = vc;

    // ---- 피드포워드: 기체가 가만히 있을 때 LOS 가 화면에서 움직이는 속도 ----
    const pn::Vec3 ld_c = to_cam(rate);
    out.ff_x = (ld_c.x * l_c.z - l_c.x * ld_c.z) / (l_c.z * l_c.z);
    out.ff_y = (ld_c.y * l_c.z - l_c.y * ld_c.z) / (l_c.z * l_c.z);
  }
  out.accel_pn_ned = a_pn;

  // ---- 목표 비추력 f = A·l̂ + a_pn − g ----
  const double approach = params.approach_accel_mps2 * std::clamp(input.approach_scale, 0.0, 1.0);
  pn::Vec3 f = approach * l_ned + a_pn - pn::Vec3{0.0, 0.0, pn::kGravity};

  // ---- 시야각 제약 (PN 과 같은 방식) ----
  //   d̂ 를 LOS 에서 θmax 이내로 당기고, 크기는 LOS 수직 성분(횡가속 + 중력 상쇄)을
  //   그대로 내도록 정한다. 남는 추력은 LOS 방향 접근 가속이 된다.
  const double max_look = params.max_look_angle_rad;
  const pn::Vec3 axis = pn::normalized(f);
  const double cos_look = std::clamp(pn::dot(axis, l_ned), -1.0, 1.0);
  if (max_look > 0.0 && std::acos(cos_look) > max_look) {
    const pn::Vec3 u = pn::normalized(axis - cos_look * l_ned);
    if (pn::norm(u) > 0.5) {
      const pn::Vec3 d = std::cos(max_look) * l_ned + std::sin(max_look) * u;
      const double t = pn::dot(f, u) / std::sin(max_look);
      f = std::max(t, 0.0) * d;
      out.look_limited = true;
    }
  }
  if (pn::norm(f) < 1e-6) return out;
  out.look_angle_deg =
      std::acos(std::clamp(pn::dot(pn::normalized(f), l_ned), -1.0, 1.0)) * pn::kRadToDeg;

  // ---- 화면 좌표로 ----
  const pn::Vec3 f_c = to_cam(f);
  const double fz = std::max(f_c.z, kMinProjZ * pn::norm(f_c));
  out.err_x = f_c.x / fz;
  out.err_y = f_c.y / fz;
  out.aim_x = l_c.x / l_c.z - out.err_x;
  out.aim_y = l_c.y / l_c.z - out.err_y;

  // 추력은 실제 추력축(광축 = optical +z) 방향으로만 나간다. 그 성분이 f 와 같도록.
  out.throttle = params.hover_throttle * std::max(f_c.z, 0.0) / pn::kGravity;
  out.valid = std::isfinite(out.err_x) && std::isfinite(out.err_y);
  return out;
}

}  // namespace arms_control::pid_aim
