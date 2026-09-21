#include "tianji_qp_ik/cartesian_acceleration_servo.hpp"

#include "tianji_qp_ik/so3.hpp"

namespace tianji_qp_ik {
namespace {

Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, double maximum) {
  const double norm = value.norm();
  return norm <= maximum || norm == 0.0 ? value
                                        : value * (maximum / norm);
}

}  // namespace

Vec6 cartesianAccelerationCommand(
    const CartesianAccelerationServoConfig& config,
    const CartesianReference& reference, const Pose& measured_pose,
    const Vec6& measured_twist) {
  const Vec6 error = poseErrorWorld(reference.pose, measured_pose);
  Vec6 command;
  command.head<3>() = clampNorm(
      reference.acceleration.head<3>() +
          config.kd_position *
              (reference.twist.head<3>() - measured_twist.head<3>()) +
          config.kp_position * error.head<3>(),
      config.linear_limit);
  command.tail<3>() = clampNorm(
      reference.acceleration.tail<3>() +
          config.kd_orientation *
              (reference.twist.tail<3>() - measured_twist.tail<3>()) +
          config.kp_orientation * error.tail<3>(),
      config.angular_limit);
  return command;
}

}  // namespace tianji_qp_ik
