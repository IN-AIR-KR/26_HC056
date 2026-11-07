#include "arms_control/pn_guidance.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

using arms_control::pn::CameraModel;
using arms_control::pn::Guidance;
using arms_control::pn::Parameters;
using arms_control::pn::Quaternion;
using arms_control::pn::norm;
using arms_control::pn::kDegToRad;
using arms_control::pn::kRadToDeg;

int main() {
  Guidance guidance;
  CameraModel camera;
  camera.fx = camera.fy = 200.0;
  camera.cx = camera.cy = 100.0;
  camera.width = camera.height = 200.0;
  guidance.set_camera(camera);

  Parameters params;
  params.target_diameter_m = 0.5;
  params.closing_speed_alpha = 1.0;
  guidance.set_parameters(params);

  const Quaternion identity;
  assert(guidance.update_detection(1.00, 0.50, 0.50, 0.10, 0.10, identity));
  assert(guidance.update_detection(1.05, 0.50, 0.50, 0.11, 0.11, identity));
  assert(guidance.output().valid);
  assert(guidance.output().closing_speed_mps > 0.0);
  assert(norm(guidance.output().los_rate_ned) < 1e-6);

  assert(guidance.update_detection(1.10, 0.53, 0.50, 0.12, 0.12, identity));
  assert(norm(guidance.output().los_rate_ned) > 1e-3);
  assert(norm(guidance.output().acceleration_pn_ned) > 1e-3);
  assert(!guidance.output().look_angle_limited);

  // 시야각 제약: 상방 카메라로 앙각 60° 공을 보며 발사(pitch -30°). 제약이 없으면
  // 추력축이 LOS 보다 20° 가까이 서서 시야(이 카메라 ±26.6°의 0.4배)를 벗어난다.
  Guidance limited;
  limited.set_camera(camera);
  params.look_angle_fov_ratio = 0.4;
  params.max_throttle = 1.0;
  limited.set_parameters(params);
  const Quaternion launch = Quaternion::from_euler(0.0, -30.0 * kDegToRad, 0.0);
  assert(limited.update_detection(2.00, 0.50, 0.50, 0.10, 0.10, launch));
  assert(limited.update_detection(2.05, 0.50, 0.50, 0.10, 0.10, launch));
  const auto &out = limited.output();
  const double max_look_deg = 0.4 * std::atan(0.5) * kRadToDeg;
  assert(out.look_angle_limited);
  assert(std::abs(out.look_angle_deg - max_look_deg) < 0.1);
  // 추력축(body -z)의 기울기 = 30° - max_look, 그리고 제약 없는 경우보다 추력이 크다.
  const double tilt_deg =
      std::acos(std::clamp(out.attitude_desired_nb(2, 2), -1.0, 1.0)) * kRadToDeg;
  assert(std::abs(tilt_deg - (30.0 - max_look_deg)) < 0.2);
  assert(out.throttle > params.hover_throttle);

  // 제약을 끄면 원래 법칙(LOS 방향 5 m/s² + 중력 상쇄 → 기울기 약 10°)이다.
  params.look_angle_fov_ratio = 0.0;
  limited.set_parameters(params);
  assert(limited.update_detection(2.10, 0.50, 0.50, 0.10, 0.10, launch));
  assert(!limited.output().look_angle_limited);
  const double free_tilt_deg =
      std::acos(std::clamp(limited.output().attitude_desired_nb(2, 2), -1.0, 1.0)) * kRadToDeg;
  assert(free_tilt_deg < 12.0);
  return 0;
}
