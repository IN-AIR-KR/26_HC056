#include "arms_control/pn_guidance.hpp"

#include <algorithm>

namespace arms_control::pn {

Quaternion Quaternion::normalized_copy() const {
  const double length = std::sqrt(w * w + x * x + y * y + z * z);
  return length > 1e-12 ? Quaternion{w / length, x / length, y / length, z / length}
                        : Quaternion{};
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

Quaternion Quaternion::slerp(const Quaternion &qa, const Quaternion &qb, double ratio) {
  auto a = qa.normalized_copy();
  auto b = qb.normalized_copy();
  double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (d < 0.0) {
    d = -d;
    b = {-b.w, -b.x, -b.y, -b.z};
  }
  d = std::clamp(d, -1.0, 1.0);
  if (d > 0.9995) {
    return {a.w + ratio * (b.w - a.w), a.x + ratio * (b.x - a.x),
            a.y + ratio * (b.y - a.y), a.z + ratio * (b.z - a.z)};
  }
  const double theta = std::acos(d);
  const double sin_theta = std::sin(theta);
  const double wa = std::sin((1.0 - ratio) * theta) / sin_theta;
  const double wb = std::sin(ratio * theta) / sin_theta;
  return {wa * a.w + wb * b.w, wa * a.x + wb * b.x, wa * a.y + wb * b.y,
          wa * a.z + wb * b.z};
}

void AttitudeBuffer::clear() { samples_.clear(); }

void AttitudeBuffer::push(double stamp_sec, const Quaternion &q_nb) {
  if (!std::isfinite(stamp_sec) || (!samples_.empty() && stamp_sec < samples_.back().stamp_sec)) return;
  samples_.push_back({stamp_sec, q_nb.normalized_copy()});
  while (samples_.size() > kMaxSamples) samples_.pop_front();
}

bool AttitudeBuffer::interpolate(double stamp_sec, double max_gap_sec, Quaternion &out) const {
  if (samples_.empty()) return false;
  if (stamp_sec <= samples_.front().stamp_sec) {
    if (samples_.front().stamp_sec - stamp_sec > max_gap_sec) return false;
    out = samples_.front().q_nb;
    return true;
  }
  if (stamp_sec >= samples_.back().stamp_sec) {
    if (stamp_sec - samples_.back().stamp_sec > max_gap_sec) return false;
    out = samples_.back().q_nb;
    return true;
  }
  for (std::size_t i = 1; i < samples_.size(); ++i) {
    if (samples_[i].stamp_sec < stamp_sec) continue;
    const auto &a = samples_[i - 1];
    const auto &b = samples_[i];
    if (stamp_sec - a.stamp_sec > max_gap_sec || b.stamp_sec - stamp_sec > max_gap_sec) return false;
    const double span = b.stamp_sec - a.stamp_sec;
    out = Quaternion::slerp(a.q_nb, b.q_nb, span > 1e-9 ? (stamp_sec - a.stamp_sec) / span : 0.0);
    return true;
  }
  return false;
}

void ConstantRateLosFilter::reset() {
  axes_ = {};
  initialized_ = false;
  last_stamp_sec_ = 0.0;
  los_ = {};
  los_rate_ = {};
}

void ConstantRateLosFilter::predict_axis(Axis &axis, double dt, double noise) {
  axis.position += axis.velocity * dt;
  const double p00 = axis.p00 + dt * (axis.p10 + axis.p01) + dt * dt * axis.p11;
  const double p01 = axis.p01 + dt * axis.p11;
  const double p10 = axis.p10 + dt * axis.p11;
  const double dt2 = dt * dt;
  axis.p00 = p00 + noise * dt2 * dt2 * 0.25;
  axis.p01 = p01 + noise * dt2 * dt * 0.5;
  axis.p10 = p10 + noise * dt2 * dt * 0.5;
  axis.p11 += noise * dt2;
}

void ConstantRateLosFilter::update_axis(Axis &axis, double measurement, double noise) {
  const double innovation = measurement - axis.position;
  const double variance = axis.p00 + std::max(noise, 1e-9);
  const double k0 = axis.p00 / variance;
  const double k1 = axis.p10 / variance;
  const double p00 = axis.p00;
  const double p01 = axis.p01;
  axis.position += k0 * innovation;
  axis.velocity += k1 * innovation;
  axis.p00 -= k0 * p00;
  axis.p01 -= k0 * p01;
  axis.p10 -= k1 * p00;
  axis.p11 -= k1 * p01;
}

void ConstantRateLosFilter::refresh_vectors() {
  los_ = normalized({axes_[0].position, axes_[1].position, axes_[2].position});
  const Vec3 rate{axes_[0].velocity, axes_[1].velocity, axes_[2].velocity};
  los_rate_ = rate - dot(rate, los_) * los_;
}

bool ConstantRateLosFilter::update(const Vec3 &measurement, double stamp_sec,
                                   double accel_noise, double measurement_noise) {
  const Vec3 z = normalized(measurement);
  if (!finite(z) || norm(z) < 0.5 || !std::isfinite(stamp_sec)) return false;
  if (!initialized_) {
    for (std::size_t i = 0; i < axes_.size(); ++i) {
      axes_[i].position = i == 0 ? z.x : (i == 1 ? z.y : z.z);
      axes_[i].velocity = 0.0;
      axes_[i].p00 = measurement_noise * measurement_noise;
      axes_[i].p01 = axes_[i].p10 = 0.0;
      axes_[i].p11 = 1.0;
    }
    initialized_ = true;
    last_stamp_sec_ = stamp_sec;
    refresh_vectors();
    return true;
  }
  const double dt = stamp_sec - last_stamp_sec_;
  if (!(dt > 1e-4 && dt < 0.5)) {
    if (dt <= 0.0) return false;
    reset();
    return update(z, stamp_sec, accel_noise, measurement_noise);
  }
  const double process_noise = accel_noise * accel_noise;
  const double sensor_noise = measurement_noise * measurement_noise;
  for (auto &axis : axes_) predict_axis(axis, dt, process_noise);
  update_axis(axes_[0], z.x, sensor_noise);
  update_axis(axes_[1], z.y, sensor_noise);
  update_axis(axes_[2], z.z, sensor_noise);
  last_stamp_sec_ = stamp_sec;
  refresh_vectors();
  return true;
}

void Guidance::reset() {
  los_filter_.reset();
  scale_initialized_ = false;
  previous_range_m_ = previous_scale_stamp_sec_ = closing_speed_mps_ = 0.0;
  yaw_initialized_ = false;
  fixed_yaw_rad_ = 0.0;
  output_ = {};
}

Mat3 Guidance::desired_attitude(const Vec3 &specific_thrust_ned) const {
  const Vec3 b3 = -1.0 * normalized(specific_thrust_ned);  // FRD +z is down.
  const Vec3 heading{std::cos(fixed_yaw_rad_), std::sin(fixed_yaw_rad_), 0.0};
  Vec3 b2 = normalized(cross(b3, heading));
  if (norm(b2) < 1e-6) b2 = {-std::sin(fixed_yaw_rad_), std::cos(fixed_yaw_rad_), 0.0};
  const Vec3 b1 = normalized(cross(b2, b3));
  Mat3 result;
  result.v = {b1.x, b2.x, b3.x, b1.y, b2.y, b3.y, b1.z, b2.z, b3.z};
  return result;
}

bool Guidance::update_detection(double stamp_sec, double x_center, double y_center,
                                double width, double height, const Quaternion &q_nb,
                                const Vec3 *los_rate_override_ned,
                                const double *closing_speed_override_mps) {
  if (!(camera_.fx > 1.0 && camera_.fy > 1.0 && camera_.width > 1.0 && camera_.height > 1.0)) return false;
  const Vec3 ray_camera = normalized({(x_center * camera_.width - camera_.cx) / camera_.fx,
                                      (y_center * camera_.height - camera_.cy) / camera_.fy, 1.0});
  const Mat3 r_nb = q_nb.rotation_matrix();
  const Vec3 ray_ned = normalized(r_nb * (camera_.r_bc * ray_camera));
  if (!los_filter_.update(ray_ned, stamp_sec, params_.kf_accel_noise, params_.kf_measurement_noise)) return false;
  if (!yaw_initialized_) {
    fixed_yaw_rad_ = std::atan2(r_nb(1, 0), r_nb(0, 0));
    yaw_initialized_ = true;
  }

  const double angular_scale = std::max(width * camera_.width / camera_.fx,
                                        height * camera_.height / camera_.fy);
  double range_m = 0.0;
  if (angular_scale > 1e-4 && params_.target_diameter_m > 0.0) {
    range_m = params_.target_diameter_m / angular_scale;
    if (scale_initialized_) {
      const double dt = stamp_sec - previous_scale_stamp_sec_;
      if (dt > 1e-3 && dt < 0.5) {
        const double raw_vc = (previous_range_m_ - range_m) / dt;
        const double alpha = std::clamp(params_.closing_speed_alpha, 0.0, 1.0);
        closing_speed_mps_ = alpha * raw_vc + (1.0 - alpha) * closing_speed_mps_;
      }
    } else {
      scale_initialized_ = true;
    }
    previous_range_m_ = range_m;
    previous_scale_stamp_sec_ = stamp_sec;
  }

  const Vec3 los = los_filter_.los();
  const Vec3 los_rate = los_rate_override_ned != nullptr
      ? *los_rate_override_ned : los_filter_.los_rate();
  const double closing_speed = closing_speed_override_mps != nullptr
      ? std::max(0.0, *closing_speed_override_mps)
      : std::max(0.0, closing_speed_mps_);
  Vec3 a_pn = params_.navigation_ratio * closing_speed * los_rate;
  const double lateral = norm(a_pn);
  if (params_.max_lateral_accel_mps2 > 0.0 && lateral > params_.max_lateral_accel_mps2) {
    a_pn = a_pn * (params_.max_lateral_accel_mps2 / lateral);
  }
  const Vec3 guidance_accel = params_.approach_accel_mps2 * los + a_pn;
  const Vec3 specific_thrust = guidance_accel - Vec3{0.0, 0.0, kGravity};
  const double throttle = params_.hover_throttle * norm(specific_thrust) / kGravity;

  output_.valid = los_filter_.initialized() && scale_initialized_ && range_m > 0.0;
  output_.stamp_sec = stamp_sec;
  output_.los_ned = los;
  output_.los_rate_ned = los_rate;
  output_.range_m = range_m;
  output_.closing_speed_mps = closing_speed;
  output_.acceleration_pn_ned = a_pn;
  output_.attitude_desired_nb = desired_attitude(specific_thrust);
  output_.throttle = std::clamp(throttle, 0.0, params_.max_throttle);
  return true;
}

}  // namespace arms_control::pn
