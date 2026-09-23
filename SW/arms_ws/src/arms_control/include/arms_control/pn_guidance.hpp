#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

namespace arms_control::pn {

constexpr double kGravity = 9.80665;
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kDegToRad = 0.017453292519943295;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
  Vec3 &operator+=(const Vec3 &o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
};

inline Vec3 operator*(double s, const Vec3 &v) { return v * s; }
inline double dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3 &v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(const Vec3 &v) {
  const double n = norm(v);
  return n > 1e-12 ? v / n : Vec3{};
}
inline bool finite(const Vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

struct Mat3 {
  std::array<double, 9> v{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  double &operator()(std::size_t r, std::size_t c) { return v[r * 3 + c]; }
  double operator()(std::size_t r, std::size_t c) const { return v[r * 3 + c]; }

  Vec3 operator*(const Vec3 &p) const {
    return {v[0] * p.x + v[1] * p.y + v[2] * p.z,
            v[3] * p.x + v[4] * p.y + v[5] * p.z,
            v[6] * p.x + v[7] * p.y + v[8] * p.z};
  }
};

inline Mat3 transpose(const Mat3 &a) {
  Mat3 out;
  for (std::size_t r = 0; r < 3; ++r)
    for (std::size_t c = 0; c < 3; ++c)
      out(r, c) = a(c, r);
  return out;
}

inline Mat3 operator*(const Mat3 &a, const Mat3 &b) {
  Mat3 out;
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t c = 0; c < 3; ++c) {
      out(r, c) = 0.0;
      for (std::size_t k = 0; k < 3; ++k)
        out(r, c) += a(r, k) * b(k, c);
    }
  }
  return out;
}

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Quaternion normalized_copy() const;
  Mat3 rotation_matrix() const;
  static Quaternion from_euler(double roll, double pitch, double yaw);
  static Quaternion slerp(const Quaternion &a, const Quaternion &b, double u);
};

struct AttitudeSample {
  double stamp_sec{0.0};
  Quaternion q_nb; // body FRD -> world NED
};

class AttitudeBuffer {
public:
  void clear();
  void push(double stamp_sec, const Quaternion &q_nb);
  bool interpolate(double stamp_sec, double max_gap_sec, Quaternion &out) const;
  bool latest(AttitudeSample &out) const;

private:
  std::deque<AttitudeSample> samples_;
  static constexpr std::size_t kMaxSamples = 256;
};

// 세 축에 같은 2-state constant-rate KF를 적용한다.
// 상태는 [LOS component, LOS component rate]. 갱신 후 LOS를 단위벡터로 만들고
// rate의 LOS 평행 성분을 제거해 구면의 접평면 속도만 남긴다.
class ConstantRateLosKalman {
public:
  void reset();
  bool update(const Vec3 &los_measurement, double stamp_sec, double accel_noise,
              double measurement_noise);
  bool predict(double stamp_sec, double accel_noise);
  bool initialized() const { return initialized_; }
  const Vec3 &los() const { return los_; }
  const Vec3 &los_rate() const { return los_rate_; }

private:
  struct Axis {
    double p{0.0};
    double v{0.0};
    double P00{1.0};
    double P01{0.0};
    double P10{0.0};
    double P11{1.0};
  };

  void predict_axis(Axis &a, double dt, double q);
  void update_axis(Axis &a, double z, double r);
  void refresh_vectors();

  std::array<Axis, 3> axes_{};
  bool initialized_{false};
  double last_stamp_sec_{0.0};
  Vec3 los_{};
  Vec3 los_rate_{};
};

struct PnParameters {
  double navigation_ratio{3.0};
  double approach_accel_mps2{5.0};
  double target_diameter_m{2.0};
  double closing_speed_alpha{0.25};
  double kf_accel_noise{0.8};
  double kf_measurement_noise{0.0025};
  double hover_throttle{0.42};
  double max_throttle{0.70};
  double max_lateral_accel_mps2{12.0};
  double max_measurement_age_sec{0.20};
};

struct CameraModel {
  double fx{231.16};
  double fy{231.16};
  double cx{360.0};
  double cy{240.0};
  double width{720.0};
  double height{480.0};
  // OpenCV optical (right, down, forward) -> PX4 body FRD.
  // 상방 카메라 기본값: image right=body right, image down=body forward,
  // optical forward=body up(-z).
  Mat3 r_bc{{0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0}};
};

struct PnOutput {
  bool valid{false};
  double stamp_sec{0.0};
  Vec3 los_ned{};
  Vec3 los_rate_ned{};
  Vec3 omega_los_ned{};
  double range_m{0.0};
  double closing_speed_mps{0.0};
  Vec3 acceleration_pn_ned{};
  Vec3 acceleration_guidance_ned{};
  Vec3 specific_thrust_ned{};
  Mat3 attitude_desired_nb{};
  double throttle{0.0};
};

class PnGuidance {
public:
  void reset();
  void set_parameters(const PnParameters &params) { params_ = params; }
  void set_camera(const CameraModel &camera) { camera_ = camera; }
  const PnParameters &parameters() const { return params_; }
  const CameraModel &camera() const { return camera_; }

  // raw bbox 중심/크기를 한 번만 필터링한다. stamp는 원본 영상 촬영시각이다.
  bool update_detection(double stamp_sec, double x_center, double y_center,
                        double width, double height, const Quaternion &q_nb);

  const PnOutput &output() const { return output_; }

private:
  Mat3 desired_attitude(const Vec3 &specific_thrust_ned) const;

  PnParameters params_{};
  CameraModel camera_{};
  ConstantRateLosKalman los_filter_{};
  bool scale_initialized_{false};
  double previous_range_m_{0.0};
  double previous_scale_stamp_sec_{0.0};
  double closing_speed_mps_{0.0};
  bool yaw_initialized_{false};
  double fixed_yaw_rad_{0.0};
  PnOutput output_{};
};

} // namespace arms_control::pn
