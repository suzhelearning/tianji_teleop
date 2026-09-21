#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pico_bridge::fusion {
using Quaternion = std::array<double, 4>;  // x, y, z, w
using Vector3 = std::array<double, 3>;

inline bool try_normalize(Quaternion q, Quaternion &out) {
  double n2 = 0.0;
  for (double value : q) {
    if (!std::isfinite(value)) return false;
    n2 += value * value;
  }
  if (n2 <= 1e-18) return false;
  const double n = std::sqrt(n2);
  for (double &value : q) value /= n;
  if (q[3] < 0.0) for (double &value : q) value = -value;
  out = q;
  return true;
}

inline Quaternion normalize(Quaternion q) {
  Quaternion out{};
  if (!try_normalize(q, out)) throw std::invalid_argument("invalid quaternion");
  return out;
}

inline Quaternion multiply(Quaternion a, Quaternion b) {
  a = normalize(a); b = normalize(b);
  return normalize({a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
                    a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
                    a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
                    a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2]});
}

inline Quaternion inverse(Quaternion q) {
  q = normalize(q);
  return {-q[0], -q[1], -q[2], q[3]};
}

inline Vector3 rotate(Quaternion q, Vector3 v) {
  q = normalize(q);
  const double tx = 2.0 * (q[1]*v[2] - q[2]*v[1]);
  const double ty = 2.0 * (q[2]*v[0] - q[0]*v[2]);
  const double tz = 2.0 * (q[0]*v[1] - q[1]*v[0]);
  return {v[0] + q[3]*tx + q[1]*tz - q[2]*ty,
          v[1] + q[3]*ty + q[2]*tx - q[0]*tz,
          v[2] + q[3]*tz + q[0]*ty - q[1]*tx};
}

inline Quaternion relative_motion(Quaternion parent, Quaternion child,
                                  Quaternion parent0, Quaternion child0) {
  return multiply(multiply(multiply(inverse(parent), child), inverse(child0)), parent0);
}

inline Quaternion aligned_foot_orientation(Quaternion parent, Quaternion child,
                                           Quaternion parent0, Quaternion child0,
                                           Quaternion foot0) {
  return multiply(multiply(multiply(parent, relative_motion(parent, child, parent0, child0)),
                           inverse(parent0)), foot0);
}

inline Quaternion relative_ankle_orientation(Quaternion shank, Quaternion foot) {
  return multiply(inverse(shank), foot);
}

inline double angular_distance(Quaternion a, Quaternion b) {
  a = normalize(a); b = normalize(b);
  const double dot = std::abs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
  return 2.0 * std::acos(std::min(1.0, dot));
}

inline bool calibration_step_is_stable(
  Quaternion previous_pelvis, Quaternion current_pelvis,
  Quaternion previous_left_foot, Quaternion current_left_foot,
  Quaternion previous_right_foot, Quaternion current_right_foot,
  Quaternion previous_left_imu, Quaternion current_left_imu,
  Quaternion previous_right_imu, Quaternion current_right_imu,
  double max_step_rad) {
  return angular_distance(previous_pelvis, current_pelvis) <= max_step_rad &&
         angular_distance(previous_left_foot, current_left_foot) <= max_step_rad &&
         angular_distance(previous_right_foot, current_right_foot) <= max_step_rad &&
         angular_distance(previous_left_imu, current_left_imu) <= max_step_rad &&
         angular_distance(previous_right_imu, current_right_imu) <= max_step_rad;
}

inline Vector3 reconstruct_foot_position(Vector3 ankle, Quaternion foot, Vector3 local_offset) {
  const auto offset = rotate(foot, local_offset);
  return {ankle[0] + offset[0], ankle[1] + offset[1], ankle[2] + offset[2]};
}
}  // namespace pico_bridge::fusion
