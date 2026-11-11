#pragma once

#include "arms_control/yang_dkf.hpp"

#include <Eigen/Dense>

namespace arms_control::yang {

// Yang et al., §III, Eqs. (13), (19), (21)--(23), (26), and (28).
// The output stays in physical SI units; FlightCommand conversion belongs to
// the following integration stages.
struct GuidanceParameters {
  double mass_kg{1.0};
  Eigen::Vector3d drag_coeff_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d thrust_axis_body{0.0, 0.0, -1.0};
  Eigen::Vector3d target_accel_ned{Eigen::Vector3d::Zero()};
  double k1{1.0};
  double k2{1.0};
  double barrier_kb{0.9};
  double max_rate_radps{1.0};
  double max_thrust_n{1.0};
};

struct GuidanceOutput {
  bool valid{false};
  Eigen::Vector3d target_unit_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d designed_los_ned{Eigen::Vector3d::Zero()};
  double z1{0.0};
  Eigen::Vector3d z2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_accel_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega1_body_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega2_body_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_body_radps{Eigen::Vector3d::Zero()};
  double desired_thrust_n{0.0};
};

class Guidance {
 public:
  void set_parameters(const GuidanceParameters &params) { params_ = params; }
  GuidanceOutput compute(const State &state) const;

 private:
  static Eigen::Vector3d vex(const Eigen::Matrix3d &skew);
  static Eigen::Matrix3d rotation_between(const Eigen::Vector3d &from,
                                          const Eigen::Vector3d &to);
  static Eigen::Vector3d saturate_norm(const Eigen::Vector3d &value, double limit);

  GuidanceParameters params_{};
};

}  // namespace arms_control::yang
