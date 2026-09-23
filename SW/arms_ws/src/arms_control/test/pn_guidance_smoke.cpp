#include "arms_control/pn_guidance.hpp"

#include <cassert>
#include <cmath>

using arms_control::pn::AttitudeBuffer;
using arms_control::pn::CameraModel;
using arms_control::pn::Mat3;
using arms_control::pn::norm;
using arms_control::pn::PnGuidance;
using arms_control::pn::PnParameters;
using arms_control::pn::Quaternion;

int main() {
  AttitudeBuffer attitudes;
  attitudes.push(0.0, Quaternion::from_euler(0.0, 0.0, 0.0));
  attitudes.push(1.0, Quaternion::from_euler(0.0, 0.0, 0.4));
  Quaternion interpolated;
  assert(attitudes.interpolate(0.5, 0.6, interpolated));
  const Mat3 r = interpolated.rotation_matrix();
  assert(std::abs(std::atan2(r(1, 0), r(0, 0)) - 0.2) < 1e-6);

  PnGuidance guidance;
  CameraModel camera;
  camera.fx = camera.fy = 200.0;
  camera.cx = camera.cy = 100.0;
  camera.width = camera.height = 200.0;
  guidance.set_camera(camera);

  PnParameters params;
  params.target_diameter_m = 2.0;
  params.closing_speed_alpha = 1.0;
  params.max_closing_speed_mps = 20.0;
  guidance.set_parameters(params);

  const Quaternion identity;
  assert(guidance.update_detection(1.00, 0.50, 0.50, 0.10, 0.10, identity));
  assert(guidance.update_detection(1.05, 0.50, 0.50, 0.11, 0.11, identity));
  const auto &straight = guidance.output();
  assert(straight.valid);
  assert(straight.closing_speed_mps > 0.0);
  assert(norm(straight.los_rate_ned) < 1e-6);
  assert(straight.throttle > params.hover_throttle);

  assert(guidance.update_detection(1.10, 0.53, 0.50, 0.12, 0.12, identity));
  const auto &crossing = guidance.output();
  assert(norm(crossing.los_rate_ned) > 1e-3);
  assert(norm(crossing.acceleration_pn_ned) > 1e-3);

  // 한 프레임에서 bbox가 비정상적으로 커져도 Vc가 PN을 폭주시켜선 안 된다.
  assert(guidance.update_detection(1.15, 0.53, 0.50, 0.80, 0.80, identity));
  assert(guidance.output().closing_speed_mps <= params.max_closing_speed_mps);
  return 0;
}
