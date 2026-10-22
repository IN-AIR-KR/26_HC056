#include "arms_control/pn_guidance.hpp"

#include <cassert>

using arms_control::pn::CameraModel;
using arms_control::pn::Guidance;
using arms_control::pn::Parameters;
using arms_control::pn::Quaternion;
using arms_control::pn::norm;

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
  return 0;
}
