#include "arms_control/pn_guidance.hpp"

#include <algorithm>

namespace arms_control::pn {

Quaternion Quaternion::normalized_copy() const {
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  if (n <= 1e-12)
    return {};
  return {w / n, x / n, y / n, z / n};
}

Mat3 Quaternion::rotation_matrix() const {
  const auto q = normalized_copy();
  Mat3 r;
  r.v = {1.0 - 2.0 * (q.y * q.y + q.z * q.z), 2.0 * (q.x * q.y - q.z * q.w),
         2.0 * (q.x * q.z + q.y * q.w),       2.0 * (q.x * q.y + q.z * q.w),
         1.0 - 2.0 * (q.x * q.x + q.z * q.z), 2.0 * (q.y * q.z - q.x * q.w),
         2.0 * (q.x * q.z - q.y * q.w),       2.0 * (q.y * q.z + q.x * q.w),
         1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
  return r;
}

Quaternion Quaternion::from_euler(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

Quaternion Quaternion::slerp(const Quaternion &qa, const Quaternion &qb,
                             double u) {
  Quaternion a = qa.normalized_copy();
  Quaternion b = qb.normalized_copy();
  double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (d < 0.0) {
    d = -d;
    b = {-b.w, -b.x, -b.y, -b.z};
  }
  d = std::clamp(d, -1.0, 1.0);
  if (d > 0.9995) {
    Quaternion q{a.w + u * (b.w - a.w), a.x + u * (b.x - a.x),
                 a.y + u * (b.y - a.y), a.z + u * (b.z - a.z)};
    return q.normalized_copy();
  }
  const double theta = std::acos(d);
  const double s = std::sin(theta);
  const double wa = std::sin((1.0 - u) * theta) / s;
  const double wb = std::sin(u * theta) / s;
  return {wa * a.w + wb * b.w, wa * a.x + wb * b.x, wa * a.y + wb * b.y,
          wa * a.z + wb * b.z};
}

void AttitudeBuffer::clear() { samples_.clear(); }

void AttitudeBuffer::push(double stamp_sec, const Quaternion &q_nb) {
  if (!std::isfinite(stamp_sec))
    return;
  if (!samples_.empty() && stamp_sec < samples_.back().stamp_sec)
    return;
  samples_.push_back({stamp_sec, q_nb.normalized_copy()});
  while (samples_.size() > kMaxSamples)
    samples_.pop_front();
}

bool AttitudeBuffer::interpolate(double stamp_sec, double max_gap_sec,
                                 Quaternion &out) const {
  if (samples_.empty())
    return false;
  if (stamp_sec <= samples_.front().stamp_sec) {
    if (samples_.front().stamp_sec - stamp_sec > max_gap_sec)
      return false;
    out = samples_.front().q_nb;
    return true;
  }
  if (stamp_sec >= samples_.back().stamp_sec) {
    if (stamp_sec - samples_.back().stamp_sec > max_gap_sec)
      return false;
    out = samples_.back().q_nb;
    return true;
  }
  for (std::size_t i = 1; i < samples_.size(); ++i) {
    if (samples_[i].stamp_sec < stamp_sec)
      continue;
    const auto &a = samples_[i - 1];
    const auto &b = samples_[i];
    if ((stamp_sec - a.stamp_sec) > max_gap_sec ||
        (b.stamp_sec - stamp_sec) > max_gap_sec)
      return false;
    const double span = b.stamp_sec - a.stamp_sec;
    const double u = span > 1e-9 ? (stamp_sec - a.stamp_sec) / span : 0.0;
    out = Quaternion::slerp(a.q_nb, b.q_nb, u);
    return true;
  }
  return false;
}

bool AttitudeBuffer::latest(AttitudeSample &out) const {
  if (samples_.empty())
    return false;
  out = samples_.back();
  return true;
}

void ConstantRateLosKalman::reset() {
  axes_ = {};
  initialized_ = false;
  last_stamp_sec_ = 0.0;
  los_ = {};
  los_rate_ = {};
}

void ConstantRateLosKalman::predict_axis(Axis &a, double dt, double q) {
  a.p += a.v * dt;
  const double p00 = a.P00 + dt * (a.P10 + a.P01) + dt * dt * a.P11;
  const double p01 = a.P01 + dt * a.P11;
  const double p10 = a.P10 + dt * a.P11;
  const double p11 = a.P11;
  const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
  a.P00 = p00 + q * dt4 * 0.25;
  a.P01 = p01 + q * dt3 * 0.5;
  a.P10 = p10 + q * dt3 * 0.5;
  a.P11 = p11 + q * dt2;
}

void ConstantRateLosKalman::update_axis(Axis &a, double z, double r) {
  const double innovation = z - a.p;
  const double s = a.P00 + std::max(r, 1e-9);
  const double k0 = a.P00 / s;
  const double k1 = a.P10 / s;
  const double p00 = a.P00, p01 = a.P01;
  a.p += k0 * innovation;
  a.v += k1 * innovation;
  a.P00 -= k0 * p00;
  a.P01 -= k0 * p01;
  a.P10 -= k1 * p00;
  a.P11 -= k1 * p01;
}

void ConstantRateLosKalman::refresh_vectors() {
  Vec3 raw{axes_[0].p, axes_[1].p, axes_[2].p};
  los_ = normalized(raw);
  Vec3 rate{axes_[0].v, axes_[1].v, axes_[2].v};
  // 단위벡터의 변화율은 LOS에 수직이어야 한다.
  los_rate_ = rate - dot(rate, los_) * los_;
}

bool ConstantRateLosKalman::predict(double stamp_sec, double accel_noise) {
  if (!initialized_)
    return false;
  const double dt = stamp_sec - last_stamp_sec_;
  if (!(dt > 1e-4 && dt < 0.5))
    return false;
  for (auto &a : axes_)
    predict_axis(a, dt, accel_noise * accel_noise);
  last_stamp_sec_ = stamp_sec;
  refresh_vectors();
  return true;
}

bool ConstantRateLosKalman::update(const Vec3 &measurement, double stamp_sec,
                                   double accel_noise,
                                   double measurement_noise) {
  const Vec3 z = normalized(measurement);
  if (!finite(z) || norm(z) < 0.5 || !std::isfinite(stamp_sec))
    return false;
  if (!initialized_) {
    axes_[0].p = z.x;
    axes_[1].p = z.y;
    axes_[2].p = z.z;
    for (auto &a : axes_) {
      a.v = 0.0;
      a.P00 = measurement_noise * measurement_noise;
      a.P01 = a.P10 = 0.0;
      a.P11 = 1.0;
    }
    initialized_ = true;
    last_stamp_sec_ = stamp_sec;
    refresh_vectors();
    return true;
  }
  const double dt = stamp_sec - last_stamp_sec_;
  if (!(dt > 1e-4 && dt < 0.5)) {
    if (dt <= 0.0)
      return false;
    reset();
    return update(z, stamp_sec, accel_noise, measurement_noise);
  }
  const double q = accel_noise * accel_noise;
  const double r = measurement_noise * measurement_noise;
  for (auto &a : axes_)
    predict_axis(a, dt, q);
  update_axis(axes_[0], z.x, r);
  update_axis(axes_[1], z.y, r);
  update_axis(axes_[2], z.z, r);
  last_stamp_sec_ = stamp_sec;
  refresh_vectors();
  return true;
}

void PnGuidance::reset() {
  los_filter_.reset();
  scale_initialized_ = false;
  previous_range_m_ = 0.0;
  previous_scale_stamp_sec_ = 0.0;
  closing_speed_mps_ = 0.0;
  yaw_initialized_ = false;
  fixed_yaw_rad_ = 0.0;
  output_ = {};
}

Mat3 PnGuidance::desired_attitude(const Vec3 &specific_thrust_ned) const {
  // PX4 body FRD의 +z는 아래, 프로펠러 추력은 -z 방향이다.
  const Vec3 b3 = -1.0 * normalized(specific_thrust_ned);
  const Vec3 heading{std::cos(fixed_yaw_rad_), std::sin(fixed_yaw_rad_), 0.0};
  Vec3 b2 = normalized(cross(b3, heading));
  if (norm(b2) < 1e-6)
    b2 = {-std::sin(fixed_yaw_rad_), std::cos(fixed_yaw_rad_), 0.0};
  const Vec3 b1 = normalized(cross(b2, b3));
  Mat3 r;
  r.v = {b1.x, b2.x, b3.x, b1.y, b2.y, b3.y, b1.z, b2.z, b3.z};
  return r;
}

bool PnGuidance::update_detection(double stamp_sec, double x_center,
                                  double y_center, double width, double height,
                                  const Quaternion &q_nb,
                                  const Vec3 *los_rate_override_ned,
                                  const double *closing_speed_override_mps) {
  if (!(camera_.fx > 1.0 && camera_.fy > 1.0 && camera_.width > 1.0 &&
        camera_.height > 1.0))
    return false;
  const double u = x_center * camera_.width;
  const double v = y_center * camera_.height;
  const Vec3 ray_c = normalized(
      {(u - camera_.cx) / camera_.fx, (v - camera_.cy) / camera_.fy, 1.0});
  const Mat3 r_nb = q_nb.rotation_matrix();
  const Vec3 ray_n = normalized(r_nb * (camera_.r_bc * ray_c));
  if (!los_filter_.update(ray_n, stamp_sec, params_.kf_accel_noise,
                          params_.kf_measurement_noise))
    return false;

  if (!yaw_initialized_) {
    fixed_yaw_rad_ = std::atan2(r_nb(1, 0), r_nb(0, 0));
    yaw_initialized_ = true;
  }

  // 구형 표적의 알려진 지름 D와 angular scale d/f로 R≈D/(d/f)를 얻는다.
  // 이것이 looming의 scale 변화에 절대 단위(m)를 부여해 Vc[m/s]를 만든다.
  const double scale_x = width * camera_.width / camera_.fx;
  const double scale_y = height * camera_.height / camera_.fy;
  const double angular_scale = std::max(scale_x, scale_y);
  double range_m = 0.0;
  if (angular_scale > 1e-4 && params_.target_diameter_m > 0.0) {
    range_m = params_.target_diameter_m / angular_scale;
    if (scale_initialized_) {
      const double dt = stamp_sec - previous_scale_stamp_sec_;
      if (dt > 1e-3 && dt < 0.5) {
        // bbox 크기는 프레임마다 흔들리므로 미분값에 큰 스파이크가 생긴다.
        // 물리적으로 가능한 접근속도 범위에서 먼저 자른 뒤 LPF에 넣어야
        // 한 프레임의 잘못된 bbox가 이후 여러 프레임의 PN을 포화시키지 않는다.
        const double vc_limit = std::max(0.0, params_.max_closing_speed_mps);
        const double vc_raw = std::clamp(
            (previous_range_m_ - range_m) / dt, -vc_limit, vc_limit);
        const double a = std::clamp(params_.closing_speed_alpha, 0.0, 1.0);
        closing_speed_mps_ = a * vc_raw + (1.0 - a) * closing_speed_mps_;
      }
    } else {
      scale_initialized_ = true;
    }
    previous_range_m_ = range_m;
    previous_scale_stamp_sec_ = stamp_sec;
  }

  const Vec3 los = los_filter_.los();
  const Vec3 los_rate = los_rate_override_ned != nullptr
                            ? *los_rate_override_ned
                            : los_filter_.los_rate();
  const Vec3 omega = cross(los, los_rate);
  const double vc_source = closing_speed_override_mps != nullptr
                               ? *closing_speed_override_mps
                               : closing_speed_mps_;
  const double vc = std::clamp(vc_source, 0.0,
                               std::max(0.0, params_.max_closing_speed_mps));
  Vec3 a_pn = params_.navigation_ratio * vc * los_rate;
  const double lateral = norm(a_pn);
  if (params_.max_lateral_accel_mps2 > 0.0 &&
      lateral > params_.max_lateral_accel_mps2) {
    a_pn = a_pn * (params_.max_lateral_accel_mps2 / lateral);
  }
  const Vec3 a_guidance = params_.approach_accel_mps2 * los + a_pn;
  const Vec3 gravity_ned{0.0, 0.0, kGravity};
  const Vec3 specific_thrust = a_guidance - gravity_ned;
  const double throttle_raw =
      params_.hover_throttle * norm(specific_thrust) / kGravity;

  // Vc=0이어도 LOS 방향 접근 가속도는 유효하다. 정지 상태에서
  // looming을 기다리며 hover하면 스케일이 변하지 않아 출발할 수 없다.
  output_.valid =
      los_filter_.initialized() && scale_initialized_ && range_m > 0.0;
  output_.stamp_sec = stamp_sec;
  output_.los_ned = los;
  output_.los_rate_ned = los_rate;
  output_.omega_los_ned = omega;
  output_.range_m = range_m;
  output_.closing_speed_mps = vc;
  output_.acceleration_pn_ned = a_pn;
  output_.acceleration_guidance_ned = a_guidance;
  output_.specific_thrust_ned = specific_thrust;
  output_.attitude_desired_nb = desired_attitude(specific_thrust);
  output_.throttle = std::clamp(throttle_raw, 0.0, params_.max_throttle);
  return true;
}

} // namespace arms_control::pn
