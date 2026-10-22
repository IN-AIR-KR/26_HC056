#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <deque>

namespace arms_control::pn {

constexpr double kGravity = 9.80665;
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kDegToRad = 0.017453292519943295;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3 operator+(const Vec3 &other) const { return {x + other.x, y + other.y, z + other.z}; }
  Vec3 operator-(const Vec3 &other) const { return {x - other.x, y - other.y, z - other.z}; }
  Vec3 operator*(double scale) const { return {x * scale, y * scale, z * scale}; }
  Vec3 operator/(double scale) const { return {x / scale, y / scale, z / scale}; }
};

inline Vec3 operator*(double scale, const Vec3 &value) { return value * scale; }
inline double dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3 &value) { return std::sqrt(dot(value, value)); }
inline Vec3 normalized(const Vec3 &value) {
  const double length = norm(value);
  return length > 1e-12 ? value / length : Vec3{};
}
inline bool finite(const Vec3 &value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

struct Mat3 {
  std::array<double, 9> v{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  double &operator()(std::size_t row, std::size_t col) { return v[row * 3 + col]; }
  double operator()(std::size_t row, std::size_t col) const { return v[row * 3 + col]; }
  Vec3 operator*(const Vec3 &point) const {
    return {v[0] * point.x + v[1] * point.y + v[2] * point.z,
            v[3] * point.x + v[4] * point.y + v[5] * point.z,
            v[6] * point.x + v[7] * point.y + v[8] * point.z};
  }
};

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Quaternion normalized_copy() const;
  Mat3 rotation_matrix() const;
  static Quaternion from_euler(double roll, double pitch, double yaw);
  static Quaternion slerp(const Quaternion &a, const Quaternion &b, double ratio);
};

struct AttitudeSample {
  double stamp_sec{0.0};
  Quaternion q_nb;  // PX4 body FRD -> world NED
};

class AttitudeBuffer {
 public:
  void clear();
  void push(double stamp_sec, const Quaternion &q_nb);
  bool interpolate(double stamp_sec, double max_gap_sec, Quaternion &out) const;

 private:
  std::deque<AttitudeSample> samples_;
  static constexpr std::size_t kMaxSamples = 256;
};

// Three independent constant-rate filters, followed by unit-vector projection.
class ConstantRateLosFilter {
 public:
  void reset();
  bool update(const Vec3 &measurement, double stamp_sec, double accel_noise,
              double measurement_noise);
  bool initialized() const { return initialized_; }
  const Vec3 &los() const { return los_; }
  const Vec3 &los_rate() const { return los_rate_; }

 private:
  struct Axis {
    double position{0.0};
    double velocity{0.0};
    double p00{1.0};
    double p01{0.0};
    double p10{0.0};
    double p11{1.0};
  };

  void predict_axis(Axis &axis, double dt, double noise);
  void update_axis(Axis &axis, double measurement, double noise);
  void refresh_vectors();

  std::array<Axis, 3> axes_{};
  bool initialized_{false};
  double last_stamp_sec_{0.0};
  Vec3 los_{};
  Vec3 los_rate_{};
};

struct Parameters {
  double navigation_ratio{3.0};
  double approach_accel_mps2{5.0};
  double target_diameter_m{2.0};
  double closing_speed_alpha{0.25};
  double kf_accel_noise{0.8};
  double kf_measurement_noise{0.0025};
  double hover_throttle{0.42};
  double max_throttle{0.70};
  double max_lateral_accel_mps2{12.0};
};

struct CameraModel {
  double fx{900.0};
  double fy{900.0};
  double cx{360.0};
  double cy{240.0};
  double width{720.0};
  double height{480.0};
  // OpenCV optical (right, down, forward) -> PX4 body FRD.
  Mat3 r_bc{{0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0}};
};

struct Output {
  bool valid{false};
  double stamp_sec{0.0};
  Vec3 los_ned{};
  Vec3 los_rate_ned{};
  double range_m{0.0};
  double closing_speed_mps{0.0};
  Vec3 acceleration_pn_ned{};
  Mat3 attitude_desired_nb{};
  double throttle{0.0};
};

class Guidance {
 public:
  void reset();
  void set_parameters(const Parameters &params) { params_ = params; }
  void set_camera(const CameraModel &camera) { camera_ = camera; }
  bool update_detection(double stamp_sec, double x_center, double y_center,
                        double width, double height, const Quaternion &q_nb,
                        const Vec3 *los_rate_override_ned = nullptr,
                        const double *closing_speed_override_mps = nullptr);
  const Output &output() const { return output_; }

 private:
  Mat3 desired_attitude(const Vec3 &specific_thrust_ned) const;

  Parameters params_{};
  CameraModel camera_{};
  ConstantRateLosFilter los_filter_{};
  bool scale_initialized_{false};
  double previous_range_m_{0.0};
  double previous_scale_stamp_sec_{0.0};
  double closing_speed_mps_{0.0};
  bool yaw_initialized_{false};
  double fixed_yaw_rad_{0.0};
  Output output_{};
};

}  // namespace arms_control::pn
