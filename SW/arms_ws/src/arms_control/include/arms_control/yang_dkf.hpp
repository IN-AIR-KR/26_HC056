#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <deque>

namespace arms_control::yang {

// Delayed EKF state from Yang et al., Eq. (30)--(36):
// [q(4), p_r(3), v_r(3), image(2), b_gyr(3), b_acc(3)].
struct Parameters {
  Eigen::Matrix3d r_cb{Eigen::Matrix3d::Identity()};  // camera -> body FRD
  double initial_range_m{30.0};
  double gyro_noise_radps{0.02};
  double accel_noise_mps2{0.3};
  double gyro_bias_rw_radps2{0.002};
  double accel_bias_rw_mps3{0.03};
  double image_noise{0.004};
  double history_sec{1.0};
};

struct State {
  Eigen::Quaterniond q_nb{Eigen::Quaterniond::Identity()};  // body FRD -> world NED
  Eigen::Vector3d p_r{Eigen::Vector3d::Zero()};              // interceptor - target
  Eigen::Vector3d v_r{Eigen::Vector3d::Zero()};
  Eigen::Vector2d image{Eigen::Vector2d::Zero()};            // normalized tan coordinates
  Eigen::Vector3d bias_gyr{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_acc{Eigen::Vector3d::Zero()};
};

struct Output {
  bool valid{false};
  double stamp_sec{0.0};
  State state{};
  double last_image_age_sec{0.0};
};

class DelayedKalmanFilter {
 public:
  void reset();
  void set_parameters(const Parameters &params) { params_ = params; }

  // The paper assumes x(0) is known. We initialize position from an explicit
  // range parameter and the first image ray; attitude comes from the FC only
  // for this initial condition, then raw IMU propagates q.
  bool initialize(double stamp_sec, const Eigen::Quaterniond &q_nb,
                  const Eigen::Vector2d &image);
  bool predict(double stamp_sec, const Eigen::Vector3d &gyro_radps,
               const Eigen::Vector3d &accel_mps2);
  bool correct_delayed(double image_stamp_sec, const Eigen::Vector2d &image);

  const Output &output() const { return output_; }
  bool initialized() const { return initialized_; }

 private:
  static constexpr int kStateSize = 18;
  using Matrix18 = Eigen::Matrix<double, kStateSize, kStateSize>;
  using Vector18 = Eigen::Matrix<double, kStateSize, 1>;

  struct ImuInput {
    double stamp_sec{0.0};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
  };
  struct Snapshot {
    double stamp_sec{0.0};
    State state{};
    Matrix18 covariance{Matrix18::Identity()};
    ImuInput input{};  // input applied from the previous snapshot to this one
  };

  State propagate_state(const State &state, const ImuInput &input, double dt) const;
  Matrix18 transition_jacobian(const State &state, const ImuInput &input, double dt) const;
  Matrix18 process_covariance(double dt) const;
  static Vector18 pack(const State &state);
  static State unpack(const Vector18 &vector);
  static Eigen::Vector3d normalized_ray(const Eigen::Vector2d &image);
  void trim_history();
  void refresh_output();

  Parameters params_{};
  bool initialized_{false};
  State state_{};
  Matrix18 covariance_{Matrix18::Identity()};
  double last_stamp_sec_{0.0};
  double last_image_stamp_sec_{0.0};
  std::deque<Snapshot> history_;
  Output output_{};
};

}  // namespace arms_control::yang
