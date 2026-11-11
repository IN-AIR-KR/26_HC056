#include "arms_control/yang_guidance.hpp"

#include <cassert>
#include <cmath>

int main() {
  arms_control::yang::GuidanceParameters params;
  params.mass_kg = 1.04;
  params.thrust_axis_body = {0.0, 0.0, -1.0};
  params.k1 = 1.0;
  params.k2 = 1.0;
  params.barrier_kb = 0.9;
  params.max_rate_radps = 6.981317;
  params.max_thrust_n = 58.86;

  arms_control::yang::Guidance guidance;
  guidance.set_parameters(params);
  arms_control::yang::State state;
  state.q_nb = Eigen::Quaterniond::Identity();
  state.p_r = {0.0, 0.0, 10.0};  // target is directly along body -Z.
  state.v_r = Eigen::Vector3d::Zero();

  const auto output = guidance.compute(state);
  assert(output.valid);
  assert(std::abs(output.z1) < 1e-12);
  assert(output.omega_body_radps.norm() < 1e-12);
  assert(output.desired_thrust_n > 0.0);
  return 0;
}
