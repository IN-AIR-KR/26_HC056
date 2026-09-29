#include "arms_control/pn_guidance.hpp"

#include <cassert>
#include <cmath>

using arms_control::pn::AttitudeBuffer;
using arms_control::pn::CameraModel;
using arms_control::pn::ConstantRateRangeKalman;
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
  params.max_closing_speed_mps = 20.0;
  params.range_kf_accel_noise_mps2 = 8.0;
  params.range_kf_measurement_noise_ratio = 0.02;
  params.range_kf_min_measurement_noise_m = 0.05;
  params.range_kf_innovation_gate_sigma = 4.0;
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
  assert(!guidance.output().range_measurement_accepted);

  const arms_control::pn::Vec3 gt_rate{0.1, -0.2, 0.05};
  const double gt_vc = 7.0;
  assert(guidance.update_detection(1.20, 0.53, 0.50, 0.13, 0.13, identity,
                                   &gt_rate, &gt_vc));
  assert(std::abs(guidance.output().los_rate_ned.x - gt_rate.x) < 1e-9);
  assert(std::abs(guidance.output().los_rate_ned.y - gt_rate.y) < 1e-9);
  assert(std::abs(guidance.output().closing_speed_mps - gt_vc) < 1e-9);

  // 거리 KF는 일정하게 감소하는 noisy range에서 양의 closing speed를 추정한다.
  ConstantRateRangeKalman range_filter;
  for (int i = 0; i < 40; ++i) {
    const double t = 0.05 * i;
    const double noise = (i % 2 == 0) ? 0.03 : -0.03;
    assert(range_filter.update(20.0 - 5.0 * t + noise, t, 4.0, 0.01,
                               0.05, 4.0));
  }
  assert(std::abs(-range_filter.range_rate_mps() - 5.0) < 0.5);
  return 0;
}
