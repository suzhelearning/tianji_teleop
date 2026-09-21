#include "imu_ros2/orientation_reference.hpp"

#include <cmath>

#include "imu_ros2/im948_protocol.hpp"

namespace imu_ros2
{

namespace
{

constexpr double kInvalidQuaternionTolerance = 1e-12;
constexpr double kMinimumQuaternionNorm = 1e-9;

bool is_invalid_im948_quaternion_sentinel(const Quaternion & quaternion)
{
  return std::abs(quaternion.x + kScaleQuat) < kInvalidQuaternionTolerance &&
         std::abs(quaternion.y + kScaleQuat) < kInvalidQuaternionTolerance &&
         std::abs(quaternion.z + kScaleQuat) < kInvalidQuaternionTolerance &&
         std::abs(quaternion.w + kScaleQuat) < kInvalidQuaternionTolerance;
}

double squared_norm(const Quaternion & quaternion)
{
  return quaternion.x * quaternion.x + quaternion.y * quaternion.y +
         quaternion.z * quaternion.z + quaternion.w * quaternion.w;
}

}  // namespace

OrientationReference::OrientationReference(bool force_positive_w, bool use_quaternion_continuity)
: force_positive_w_(force_positive_w),
  use_quaternion_continuity_(use_quaternion_continuity)
{
}

void OrientationReference::configure(bool force_positive_w, bool use_quaternion_continuity)
{
  force_positive_w_ = force_positive_w;
  use_quaternion_continuity_ = use_quaternion_continuity;
  has_last_orientation_ = false;
  last_orientation_ = Quaternion{};
}

bool is_valid_quaternion(const Quaternion & quaternion)
{
  if (
    !std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
    !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w))
  {
    return false;
  }

  if (is_invalid_im948_quaternion_sentinel(quaternion)) {
    return false;
  }

  return squared_norm(quaternion) > kMinimumQuaternionNorm * kMinimumQuaternionNorm;
}

Quaternion normalize(const Quaternion & quaternion)
{
  const double norm = std::sqrt(squared_norm(quaternion));
  if (norm <= kMinimumQuaternionNorm) {
    return Quaternion{};
  }

  return Quaternion{
    quaternion.x / norm,
    quaternion.y / norm,
    quaternion.z / norm,
    quaternion.w / norm};
}

void align_hemisphere_in_place(Quaternion & current, const Quaternion & reference)
{
  const double dot =
    current.x * reference.x + current.y * reference.y + current.z * reference.z +
    current.w * reference.w;
  if (dot < 0.0) {
    current.x = -current.x;
    current.y = -current.y;
    current.z = -current.z;
    current.w = -current.w;
  }
}

bool OrientationReference::apply(
  const Quaternion & raw_orientation,
  Quaternion & output_orientation)
{
  if (!is_valid_quaternion(raw_orientation)) {
    return false;
  }

  output_orientation = normalize(raw_orientation);
  if (use_quaternion_continuity_ && has_last_orientation_) {
    const double dot =
      output_orientation.x * last_orientation_.x +
      output_orientation.y * last_orientation_.y +
      output_orientation.z * last_orientation_.z +
      output_orientation.w * last_orientation_.w;
    if (dot < 0.0) {
      output_orientation.x = -output_orientation.x;
      output_orientation.y = -output_orientation.y;
      output_orientation.z = -output_orientation.z;
      output_orientation.w = -output_orientation.w;
    }
  } else if (force_positive_w_ && output_orientation.w < 0.0) {
    output_orientation.x = -output_orientation.x;
    output_orientation.y = -output_orientation.y;
    output_orientation.z = -output_orientation.z;
    output_orientation.w = -output_orientation.w;
  }
  last_orientation_ = output_orientation;
  has_last_orientation_ = true;
  return true;
}

}  // namespace imu_ros2
