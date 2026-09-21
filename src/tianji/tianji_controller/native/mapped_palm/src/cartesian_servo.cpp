#include "tianji_mapped_palm/cartesian_servo.hpp"

#include "tianji_mapped_palm/so3.hpp"

#include <algorithm>

namespace tianji_mapped_palm {
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
  const Vec6 feedback =
      cartesianReferenceFeedbackTwist(config, reference.pose, measured);
  return clampCartesianTwist(config, feedback + reference.twist);
}

Vec6 cartesianReferenceFeedbackTwist(const CartesianServoConfig& config,
                                     const Pose& desired,
                                     const Pose& measured) {
  return cartesianReferenceFeedbackTwist(config, desired, measured, 0.0);
}

Vec6 cartesianReferenceFeedbackTwist(const CartesianServoConfig& config,
                                     const Pose& desired,
                                     const Pose& measured,
                                     double motion_phase) {
  const Vec6 error = poseErrorWorld(desired, measured);
  const CartesianFeedbackGainSchedule gains =
      cartesianFeedbackGainSchedule(config, desired, measured, motion_phase);
  Vec6 feedback;
  feedback.head<3>() = gains.position.cwiseProduct(error.head<3>());
  feedback.tail<3>() = gains.orientation.cwiseProduct(error.tail<3>());
  return feedback;
}

CartesianFeedbackGainSchedule cartesianFeedbackGainSchedule(
    const CartesianServoConfig& config, const Pose& desired,
    const Pose& measured, double motion_phase) {
  return cartesianFeedbackGainSchedule(config, desired, measured,
                                       motion_phase, motion_phase);
}

CartesianFeedbackGainSchedule cartesianFeedbackGainSchedule(
    const CartesianServoConfig& config, const Pose& desired,
    const Pose& measured, double linear_motion_phase,
    double angular_motion_phase) {
  const Vec6 error = poseErrorWorld(desired, measured);
  const double trusted_linear = config.motion_gain_enabled
                                    ? std::clamp(linear_motion_phase, 0.0, 1.0)
                                    : 0.0;
  const double trusted_angular = config.motion_gain_enabled
                                     ? std::clamp(angular_motion_phase, 0.0, 1.0)
                                     : 0.0;
  const auto combined_phase = [](double trusted_motion, double error_norm,
                                                double start, double end,
                                                double near_gain,
                                                double motion_gain,
                                                double far_gain) {
    const double error_phase = smoothstep((error_norm - start) / (end - start));
    const double motion_range = far_gain > near_gain
                                    ? (motion_gain - near_gain) /
                                          (far_gain - near_gain)
                                    : 0.0;
    return 1.0 - (1.0 - error_phase) *
                     (1.0 - trusted_motion * motion_range);
  };
  CartesianFeedbackGainSchedule result;
  if (!config.adaptive_gain_enabled) {
    result.position = config.kp_position;
    result.orientation = config.kp_orientation;
    return result;
  }
  for (int axis = 0; axis < 3; ++axis) {
    result.position[axis] = config.kp_position_near[axis] +
        combined_phase(trusted_linear, error.head<3>().norm(),
                       config.position_gain_transition_start_m,
                       config.position_gain_transition_end_m,
                       config.kp_position_near[axis],
                       config.kp_position_motion[axis],
                       config.kp_position[axis]) *
            (config.kp_position[axis] - config.kp_position_near[axis]);
    result.orientation[axis] = config.kp_orientation_near[axis] +
        combined_phase(trusted_angular, error.tail<3>().norm(),
                       config.orientation_gain_transition_start_rad,
                       config.orientation_gain_transition_end_rad,
                       config.kp_orientation_near[axis],
                       config.kp_orientation_motion[axis],
                       config.kp_orientation[axis]) *
            (config.kp_orientation[axis] -
             config.kp_orientation_near[axis]);
  }
  return result;
}

Vec6 clampCartesianTwist(const CartesianServoConfig& config,
                         const Vec6& twist) {
  Vec6 limited;
  limited.head<3>() =
      clampNorm(twist.head<3>(), config.max_linear_velocity);
  limited.tail<3>() =
      clampNorm(twist.tail<3>(), config.max_angular_velocity);
  return limited;
}

}  // namespace tianji_mapped_palm
