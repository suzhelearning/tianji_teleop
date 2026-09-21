#include "tianji_v131/cartesian_servo.hpp"

#include "tianji_v131/so3.hpp"

#include <algorithm>

namespace tianji_v131 {
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
  return cartesianReferenceServoBreakdown(config, reference, measured).command;
}

CartesianServoBreakdown cartesianReferenceServoBreakdown(
    const CartesianServoConfig& config, const CartesianReference& reference,
    const Pose& measured) {
  CartesianServoBreakdown result;
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
  result.feedforward = reference.twist;
  result.feedback.head<3>() =
      position_gain.cwiseProduct(error.head<3>());
  result.feedback.tail<3>() =
      orientation_gain.cwiseProduct(error.tail<3>());
  result.demand = result.feedforward + result.feedback;
  result.linear_saturated =
      result.demand.head<3>().norm() > config.max_linear_velocity;
  result.angular_saturated =
      result.demand.tail<3>().norm() > config.max_angular_velocity;
  result.command.head<3>() =
      clampNorm(result.demand.head<3>(), config.max_linear_velocity);
  result.command.tail<3>() =
      clampNorm(result.demand.tail<3>(), config.max_angular_velocity);
  result.valid = result.feedforward.allFinite() &&
                 result.feedback.allFinite() && result.demand.allFinite() &&
                 result.command.allFinite();
  return result;
}

}  // namespace tianji_v131
