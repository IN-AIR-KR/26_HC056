#include "arms_control/yang_guidance.hpp"

#include <algorithm>
#include <cmath>

namespace arms_control::yang {
namespace {
constexpr double kGravity = 9.80665;
constexpr double kEpsilon = 1e-8;
constexpr double kPi = 3.14159265358979323846;
}  // namespace

Eigen::Vector3d Guidance::vex(const Eigen::Matrix3d &skew) {
  return {skew(2, 1), skew(0, 2), skew(1, 0)};
}

Eigen::Vector3d Guidance::saturate_norm(const Eigen::Vector3d &value, double limit) {
  const double norm = value.norm();
  return norm > limit && norm > kEpsilon ? value * (limit / norm) : value;
}

Eigen::Matrix3d Guidance::rotation_between(const Eigen::Vector3d &from,
                                            const Eigen::Vector3d &to) {
  const Eigen::Vector3d a = from.normalized();
  const Eigen::Vector3d b = to.normalized();
  const double cosine = std::clamp(a.dot(b), -1.0, 1.0);
  const Eigen::Vector3d cross = a.cross(b);
  const double sine = cross.norm();
  if (sine > kEpsilon) {
    return Eigen::AngleAxisd(std::atan2(sine, cosine), cross / sine).toRotationMatrix();
  }
  if (cosine >= 0.0) return Eigen::Matrix3d::Identity();

  // 180° has no unique axis. Pick a deterministic perpendicular axis.
  Eigen::Vector3d axis = a.unitOrthogonal();
  return Eigen::AngleAxisd(kPi, axis).toRotationMatrix();
}

GuidanceOutput Guidance::compute(const State &state) const {
  GuidanceOutput out;
  if (!(params_.mass_kg > 0.0) || !(params_.barrier_kb > 0.0) ||
      !(params_.max_rate_radps > 0.0) || !(params_.max_thrust_n > 0.0) ||
      !state.p_r.allFinite() || !state.v_r.allFinite() ||
      !state.q_nb.coeffs().allFinite()) {
    return out;
  }

  const double range = state.p_r.norm();
  const double thrust_axis_norm = params_.thrust_axis_body.norm();
  if (!(range > kEpsilon) || !(thrust_axis_norm > kEpsilon)) return out;

  const Eigen::Matrix3d r_nb = state.q_nb.normalized().toRotationMatrix();
  // Eq. (4): p_r points interceptor -> target with the opposite sign.
  out.target_unit_ned = -state.p_r / range;
  // §II-A-4: designed LOS is chosen to approximate the current force vector.
  const Eigen::Vector3d thrust_axis_body = params_.thrust_axis_body / thrust_axis_norm;
  const Eigen::Vector3d force_direction_ned = r_nb * thrust_axis_body;
  out.designed_los_ned = force_direction_ned;
  out.z1 = 1.0 - out.designed_los_ned.dot(out.target_unit_ned);
  const double denominator = params_.barrier_kb * params_.barrier_kb - out.z1 * out.z1;
  if (!(denominator > kEpsilon)) return out;

  // Eq. (13): the collinear LOS-rate component.
  out.omega1_body_radps =
      (out.z1 / denominator) * r_nb.transpose() *
      out.designed_los_ned.cross(out.target_unit_ned);

  // Eqs. (16), (19). target_accel_ned is an explicit configuration input;
  // it is zero for the stationary-balloon experiment where a_t is absent.
  out.z2 = state.v_r + params_.k1 * state.p_r;
  const Eigen::Matrix3d transverse =
      -Eigen::Matrix3d::Identity() + out.target_unit_ned * out.target_unit_ned.transpose();
  out.desired_accel_ned =
      -params_.k1 * state.v_r - params_.k2 * out.z2 - state.p_r +
      (out.z1 / denominator) * (params_.mass_kg / range) * transverse *
          out.designed_los_ned +
      params_.target_accel_ned;

  // Eq. (1) drag model. For the current stationary-target experiment,
  // v_r equals vehicle velocity; target acceleration is explicitly zero above.
  const Eigen::Vector3d velocity_body = r_nb.transpose() * state.v_r;
  const Eigen::Vector3d drag_ned =
      -r_nb * params_.drag_coeff_body.asDiagonal() * velocity_body;
  const Eigen::Vector3d gravity_ned{0.0, 0.0, kGravity};
  const Eigen::Vector3d force_vector =
      params_.mass_kg * (out.desired_accel_ned - gravity_ned) - drag_ned;
  if (!(force_vector.norm() > kEpsilon)) return out;

  // Eqs. (21)--(22): rotate only the current thrust direction into the
  // desired force direction, retaining the current yaw convention.
  const Eigen::Vector3d desired_force_direction = force_vector.normalized();
  const Eigen::Matrix3d r_desired =
      rotation_between(force_direction_ned, desired_force_direction) * r_nb;

  // Eq. (23): physical desired thrust. It remains in N in this stage.
  out.desired_thrust_n = std::clamp(
      force_direction_ned.dot(force_vector), 0.0, params_.max_thrust_n);

  // Eq. (26), then Eq. (28)'s vector-norm saturation.
  const Eigen::Matrix3d attitude_error = r_desired.transpose() * r_nb -
      r_nb.transpose() * r_desired;
  out.omega2_body_radps = -vex(attitude_error);
  out.omega_body_radps = saturate_norm(
      out.omega1_body_radps + out.omega2_body_radps, params_.max_rate_radps);
  out.valid = out.desired_accel_ned.allFinite() && out.omega_body_radps.allFinite() &&
      std::isfinite(out.desired_thrust_n);
  return out;
}

}  // namespace arms_control::yang
