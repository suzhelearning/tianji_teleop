#include "tianji_qp_ik/cartesian_servo.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <algorithm>

namespace tianji_qp_ik {
namespace {

Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, double maximum_norm) {
  const double norm = value.norm();
  if (norm <= maximum_norm || norm == 0.0) {
    return value;
  }
  return (maximum_norm / norm) * value;
}

double smoothstep(double value) {
  const double clamped = std::clamp(value, 0.0, 1.0);
  return clamped * clamped * (3.0 - 2.0 * clamped);
}

Eigen::Vector3d scheduledGain(const Eigen::Vector3d& near_gain,
                              const Eigen::Vector3d& far_gain,
                              double error_norm, double transition_start,
                              double transition_end) {
  const double phase = smoothstep(
      (error_norm - transition_start) / (transition_end - transition_start));
  return near_gain + phase * (far_gain - near_gain);
}

}  // namespace

Vec6 cartesianServoTwist(const CartesianServoConfig& config, const Pose& desired,
                         const Pose& current, const Vec6& target_twist) {
  const Vec6 error = poseErrorWorld(desired, current);
  Vec6 twist;
  const Eigen::Vector3d linear =
      config.kp_position.cwiseProduct(error.head<3>()) +
      config.kff_linear * target_twist.head<3>();
  const Eigen::Vector3d angular =
      config.kp_orientation.cwiseProduct(error.tail<3>()) +
      config.kff_angular * target_twist.tail<3>();
  twist.head<3>() = clampNorm(linear, config.max_linear_velocity);
  twist.tail<3>() = clampNorm(angular, config.max_angular_velocity);
  return twist;
}

Vec6 cartesianReferenceServoTwist(const CartesianServoConfig& config,
                                  const CartesianReference& reference,
                                  const Pose& measured) {
  const Vec6 error = poseErrorWorld(reference.pose, measured);
  const Eigen::Vector3d position_gain =
      config.adaptive_gain_enabled
          ? scheduledGain(config.kp_position_near, config.kp_position,
                          error.head<3>().norm(),
                          config.position_gain_transition_start_m,
                          config.position_gain_transition_end_m)
          : config.kp_position;
  const Eigen::Vector3d orientation_gain =
      config.adaptive_gain_enabled
          ? scheduledGain(config.kp_orientation_near,
                          config.kp_orientation, error.tail<3>().norm(),
                          config.orientation_gain_transition_start_rad,
                          config.orientation_gain_transition_end_rad)
          : config.kp_orientation;
  Vec6 twist;
  twist.head<3>() = clampNorm(
      reference.twist.head<3>() +
          position_gain.cwiseProduct(error.head<3>()),
      config.max_linear_velocity);
  twist.tail<3>() = clampNorm(
      reference.twist.tail<3>() +
          orientation_gain.cwiseProduct(error.tail<3>()),
      config.max_angular_velocity);
  return twist;
}

}  // namespace tianji_qp_ik
