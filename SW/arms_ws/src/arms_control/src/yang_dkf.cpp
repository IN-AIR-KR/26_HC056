#include "arms_control/yang_dkf.hpp"

#include <algorithm>
#include <cmath>

namespace arms_control::yang {
namespace {
constexpr double kGravity = 9.80665;
constexpr int kImageOffset = 10;

bool finite(const Eigen::VectorXd &value) {
  return value.allFinite();
}
}  // namespace

void DelayedKalmanFilter::reset() {
  initialized_ = false;
  state_ = {};
  covariance_.setIdentity();
  last_stamp_sec_ = 0.0;
  last_image_stamp_sec_ = 0.0;
  history_.clear();
  output_ = {};
}

Eigen::Vector3d DelayedKalmanFilter::normalized_ray(const Eigen::Vector2d &image) {
  return Eigen::Vector3d{image.x(), image.y(), 1.0}.normalized();
}

DelayedKalmanFilter::Vector18 DelayedKalmanFilter::pack(const State &state) {
  Vector18 vector;
  const auto q = state.q_nb.normalized();
  vector << q.w(), q.x(), q.y(), q.z(), state.p_r, state.v_r, state.image,
      state.bias_gyr, state.bias_acc;
  return vector;
}

State DelayedKalmanFilter::unpack(const Vector18 &vector) {
  State state;
  state.q_nb = Eigen::Quaterniond(vector(0), vector(1), vector(2), vector(3)).normalized();
  state.p_r = vector.segment<3>(4);
  state.v_r = vector.segment<3>(7);
  state.image = vector.segment<2>(10);
  state.bias_gyr = vector.segment<3>(12);
  state.bias_acc = vector.segment<3>(15);
  return state;
}

bool DelayedKalmanFilter::initialize(double stamp_sec, const Eigen::Quaterniond &q_nb,
                                     const Eigen::Vector2d &image) {
  if (!std::isfinite(stamp_sec) || !q_nb.coeffs().allFinite() || !image.allFinite() ||
      !(params_.initial_range_m > 0.0)) {
    return false;
  }
  const Eigen::Vector3d ray_world =
      (q_nb.normalized().toRotationMatrix() * params_.r_cb * normalized_ray(image)).normalized();
  state_ = {};
  state_.q_nb = q_nb.normalized();
  state_.p_r = -params_.initial_range_m * ray_world;
  state_.image = image;
  covariance_.setZero();
  covariance_.diagonal() << 0.01, 0.01, 0.01, 0.01,
      25.0, 25.0, 25.0, 4.0, 4.0, 4.0,
      params_.image_noise * params_.image_noise, params_.image_noise * params_.image_noise,
      0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
  initialized_ = true;
  last_stamp_sec_ = last_image_stamp_sec_ = stamp_sec;
  history_.clear();
  history_.push_back({stamp_sec, state_, covariance_, {stamp_sec, {}, {}}});
  refresh_output();
  return true;
}

State DelayedKalmanFilter::propagate_state(const State &state, const ImuInput &input,
                                            double dt) const {
  State next = state;
  const Eigen::Vector3d omega = input.gyro - state.bias_gyr;
  const Eigen::Vector3d accel_body = input.accel - state.bias_acc;
  const Eigen::Quaterniond delta(Eigen::AngleAxisd(omega.norm() * dt,
      omega.norm() > 1e-10 ? omega.normalized() : Eigen::Vector3d::UnitX()));
  next.q_nb = (state.q_nb * delta).normalized();  // Eq. (44), body rate right-multiplication
  const Eigen::Matrix3d r_nb = state.q_nb.toRotationMatrix();
  const Eigen::Vector3d v_next = state.v_r +
      (r_nb * accel_body + Eigen::Vector3d{0.0, 0.0, kGravity}) * dt;  // Eq. (48)
  next.p_r = state.p_r + 0.5 * (state.v_r + v_next) * dt;              // Eq. (46)
  next.v_r = v_next;

  // Eq. (51): use the target point expressed in camera coordinates only to
  // obtain p_zc. The state itself stores p_r in the earth frame.
  const Eigen::Vector3d target_camera =
      -params_.r_cb.transpose() * r_nb.transpose() * state.p_r;
  const double p_zc = std::max(target_camera.z(), 0.05);
  const double x = state.image.x();
  const double y = state.image.y();
  Eigen::Matrix<double, 2, 3> l_trans;
  l_trans << -1.0 / p_zc, 0.0, x / p_zc,
      0.0, -1.0 / p_zc, y / p_zc;
  Eigen::Matrix<double, 2, 3> l_rot;
  l_rot << x * y, -(1.0 + x * x), y,
      1.0 + y * y, -x * y, -x;
  const Eigen::Vector3d v_camera = params_.r_cb.transpose() * r_nb.transpose() * state.v_r;
  const Eigen::Vector3d omega_camera = params_.r_cb.transpose() * omega;
  next.image = state.image + (l_trans * v_camera + l_rot * omega_camera) * dt;
  return next;
}

DelayedKalmanFilter::Matrix18 DelayedKalmanFilter::transition_jacobian(
    const State &state, const ImuInput &input, double dt) const {
  // F in Eq. (31) is the linearization of the paper's recurrence
  // (44), (46), (48), and (51). A central numerical derivative keeps the
  // implementation aligned with those nonlinear recurrences while avoiding a
  // second, hand-maintained set of lengthy quaternion Jacobian expressions.
  Matrix18 f;
  const Vector18 base = pack(state);
  for (int col = 0; col < kStateSize; ++col) {
    const double epsilon = col < 4 ? 1e-6 : 1e-5;
    Vector18 plus = base;
    Vector18 minus = base;
    plus(col) += epsilon;
    minus(col) -= epsilon;
    const Vector18 p = pack(propagate_state(unpack(plus), input, dt));
    const Vector18 m = pack(propagate_state(unpack(minus), input, dt));
    f.col(col) = (p - m) / (2.0 * epsilon);
  }
  return f;
}

DelayedKalmanFilter::Matrix18 DelayedKalmanFilter::process_covariance(double dt) const {
  Matrix18 q = Matrix18::Zero();
  const double gyro_var = params_.gyro_noise_radps * params_.gyro_noise_radps;
  const double accel_var = params_.accel_noise_mps2 * params_.accel_noise_mps2;
  const double gyro_bias_var = params_.gyro_bias_rw_radps2 * params_.gyro_bias_rw_radps2;
  const double accel_bias_var = params_.accel_bias_rw_mps3 * params_.accel_bias_rw_mps3;
  q.block<4, 4>(0, 0).diagonal().setConstant(gyro_var * dt * dt);
  q.block<3, 3>(4, 4).diagonal().setConstant(0.25 * accel_var * dt * dt * dt * dt);
  q.block<3, 3>(7, 7).diagonal().setConstant(accel_var * dt * dt);
  q.block<2, 2>(10, 10).diagonal().setConstant(params_.image_noise * params_.image_noise * dt);
  q.block<3, 3>(12, 12).diagonal().setConstant(gyro_bias_var * dt);
  q.block<3, 3>(15, 15).diagonal().setConstant(accel_bias_var * dt);
  return q;
}

bool DelayedKalmanFilter::predict(double stamp_sec, const Eigen::Vector3d &gyro_radps,
                                  const Eigen::Vector3d &accel_mps2) {
  if (!initialized_ || !std::isfinite(stamp_sec) || !gyro_radps.allFinite() ||
      !accel_mps2.allFinite()) {
    return false;
  }
  const double dt = stamp_sec - last_stamp_sec_;
  if (!(dt > 1e-4 && dt <= 0.1)) return false;
  const ImuInput input{stamp_sec, gyro_radps, accel_mps2};
  const Matrix18 f = transition_jacobian(state_, input, dt);
  state_ = propagate_state(state_, input, dt);
  covariance_ = f * covariance_ * f.transpose() + process_covariance(dt);
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  last_stamp_sec_ = stamp_sec;
  history_.push_back({stamp_sec, state_, covariance_, input});
  trim_history();
  refresh_output();
  return true;
}

bool DelayedKalmanFilter::correct_delayed(double image_stamp_sec,
                                          const Eigen::Vector2d &image) {
  if (!initialized_ || !std::isfinite(image_stamp_sec) || !image.allFinite() ||
      history_.empty() || image_stamp_sec < history_.front().stamp_sec ||
      image_stamp_sec > last_stamp_sec_ + 0.02) {
    return false;
  }
  std::size_t index = 0;
  for (std::size_t i = 1; i < history_.size(); ++i) {
    if (history_[i].stamp_sec > image_stamp_sec) break;
    index = i;
  }
  auto &past = history_[index];
  Eigen::Matrix<double, 2, kStateSize> h = Eigen::Matrix<double, 2, kStateSize>::Zero();
  h(0, kImageOffset) = 1.0;
  h(1, kImageOffset + 1) = 1.0;
  const Eigen::Matrix2d r = Eigen::Matrix2d::Identity() *
      params_.image_noise * params_.image_noise;
  const Eigen::Matrix2d s = h * past.covariance * h.transpose() + r;
  const Eigen::Matrix<double, kStateSize, 2> k = past.covariance * h.transpose() * s.inverse();
  Vector18 corrected = pack(past.state) + k * (image - past.state.image);
  past.state = unpack(corrected);
  const Matrix18 i = Matrix18::Identity();
  past.covariance = (i - k * h) * past.covariance * (i - k * h).transpose() + k * r * k.transpose();

  // Eq. (34)--(36): correct the matching historical state, then replay every
  // stored IMU sample forward to the present.
  for (std::size_t n = index + 1; n < history_.size(); ++n) {
    auto &previous = history_[n - 1];
    auto &current = history_[n];
    const double dt = current.stamp_sec - previous.stamp_sec;
    const Matrix18 f = transition_jacobian(previous.state, current.input, dt);
    current.state = propagate_state(previous.state, current.input, dt);
    current.covariance = f * previous.covariance * f.transpose() + process_covariance(dt);
    current.covariance = 0.5 * (current.covariance + current.covariance.transpose());
  }
  state_ = history_.back().state;
  covariance_ = history_.back().covariance;
  last_image_stamp_sec_ = image_stamp_sec;
  refresh_output();
  return true;
}

void DelayedKalmanFilter::trim_history() {
  const double cutoff = last_stamp_sec_ - std::max(params_.history_sec, 0.1);
  while (history_.size() > 1 && history_[1].stamp_sec < cutoff) history_.pop_front();
}

void DelayedKalmanFilter::refresh_output() {
  output_.valid = initialized_ && finite(pack(state_));
  output_.stamp_sec = last_stamp_sec_;
  output_.state = state_;
  output_.last_image_age_sec = std::max(0.0, last_stamp_sec_ - last_image_stamp_sec_);
}

}  // namespace arms_control::yang
