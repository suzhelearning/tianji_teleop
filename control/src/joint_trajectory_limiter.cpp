#include "tianji_qp_ik/joint_trajectory_limiter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace tianji_qp_ik {
namespace {

std::array<double, kArmDof> toArray(const Vec7& value) {
  std::array<double, kArmDof> result{};
  for (int index = 0; index < kArmDof; ++index) {
    result[static_cast<std::size_t>(index)] = value[index];
  }
  return result;
}

Vec7 toEigen(const std::array<double, kArmDof>& value) {
  Vec7 result;
  for (int index = 0; index < kArmDof; ++index) {
    result[index] = value[static_cast<std::size_t>(index)];
  }
  return result;
}

double maximumRatio(const Vec7& value, const Vec7& limit) {
  return (value.cwiseAbs().array() / limit.array()).maxCoeff();
}

}  // namespace

JointTrajectoryLimiter7::JointTrajectoryLimiter7(
    DlsPostureRuckigConfig config, ArmLimits limits, double initial_dt)
    : config_(std::move(config)),
      limits_(std::move(limits)),
      otg_(initial_dt) {
  input_.control_interface = ruckig::ControlInterface::Position;
  input_.synchronization = ruckig::Synchronization::Time;
}

bool JointTrajectoryLimiter7::validState(
    const ArmMotionState& state) const noexcept {
  if (!state.q.allFinite() || !state.qdot.allFinite() ||
      !state.qddot.allFinite()) {
    return false;
  }
  const Vec7 velocity_limit =
      (config_.velocity_scale * limits_.velocity)
          .cwiseMin(config_.max_velocity_rad_s);
  const double tolerance = config_.validation_tolerance;
  return (state.q.array() >= limits_.lower_position.array() - tolerance).all() &&
         (state.q.array() <= limits_.upper_position.array() + tolerance).all() &&
         (state.qdot.cwiseAbs().array() <=
          velocity_limit.array() + tolerance).all() &&
         (state.qddot.cwiseAbs().array() <=
          config_.max_acceleration_rad_s2.array() + tolerance).all();
}

bool JointTrajectoryLimiter7::reset(const ArmMotionState& state) noexcept {
  if (!validState(state)) {
    return false;
  }
  state_ = state;
  initialized_ = true;
  otg_.reset();
  return true;
}

JointTrajectoryResult JointTrajectoryLimiter7::update(const Vec7& target,
                                                       double dt) {
  JointTrajectoryResult result;
  result.state = state_;
  if (!initialized_ || !target.allFinite() || !std::isfinite(dt) ||
      dt <= 0.0 ||
      (target.array() < limits_.lower_position.array()).any() ||
      (target.array() > limits_.upper_position.array()).any()) {
    result.detail = "invalid_joint_trajectory_input";
    return result;
  }

  const Vec7 velocity_limit =
      (config_.velocity_scale * limits_.velocity)
          .cwiseMin(config_.max_velocity_rad_s);
  otg_.delta_time = dt;
  input_.current_position = toArray(state_.q);
  input_.current_velocity = toArray(state_.qdot);
  input_.current_acceleration = toArray(state_.qddot);
  input_.target_position = toArray(target);
  input_.target_velocity = toArray(Vec7::Zero());
  input_.target_acceleration = toArray(Vec7::Zero());
  input_.max_velocity = toArray(velocity_limit);
  input_.max_acceleration = toArray(config_.max_acceleration_rad_s2);
  input_.max_jerk = toArray(config_.max_jerk_rad_s3);
  input_.synchronization = ruckig::Synchronization::Time;

  const ruckig::Result update_result = otg_.update(input_, output_);
  if (static_cast<int>(update_result) < 0) {
    result.detail = "joint_trajectory_ruckig_failed";
    return result;
  }

  ArmMotionState candidate;
  candidate.q = toEigen(output_.new_position);
  candidate.qdot = toEigen(output_.new_velocity);
  candidate.qddot = toEigen(output_.new_acceleration);
  result.jerk = (candidate.qddot - state_.qddot) / dt;
  result.velocity_ratio = maximumRatio(candidate.qdot, velocity_limit);
  result.acceleration_ratio = maximumRatio(
      candidate.qddot, config_.max_acceleration_rad_s2);
  result.jerk_ratio = maximumRatio(result.jerk, config_.max_jerk_rad_s3);
  const double maximum_ratio = std::max(
      {result.velocity_ratio, result.acceleration_ratio, result.jerk_ratio});
  if (!validState(candidate) || !result.jerk.allFinite() ||
      maximum_ratio > 1.0 + config_.validation_tolerance) {
    result.detail = "joint_trajectory_output_rejected";
    return result;
  }

  state_ = candidate;
  result.state = state_;
  result.accepted = true;
  result.detail = update_result == ruckig::Result::Finished
                      ? "joint_trajectory_finished"
                      : "joint_trajectory_working";
  return result;
}

}  // namespace tianji_qp_ik
