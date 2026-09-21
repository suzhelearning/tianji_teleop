#include "tianji_v131/dexhand_target_conditioner.hpp"

#include "tianji_v131/so3.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace tianji_v131 {
namespace {

Eigen::Vector3d limitedNorm(const Eigen::Vector3d& vector, double maximum,
                            bool* limited) {
  const double norm = vector.norm();
  if (norm <= maximum || norm < 1.0e-12) {
    *limited = false;
    return vector;
  }
  *limited = true;
  return vector * (maximum / norm);
}

Eigen::Matrix3d expMap(const Eigen::Vector3d& vector) {
  const double norm = vector.norm();
  if (norm < 1.0e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(norm, vector / norm).toRotationMatrix();
}

void requireFinitePositive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

}  // namespace

DexhandTargetConditioner::DexhandTargetConditioner(
    const Pose& origin, const DexhandPreIkConditioningConfig& config)
    : origin_(origin), config_(config), position_state_(origin) {
  validate(origin, config);
  reset();
}

void DexhandTargetConditioner::validate(
    const Pose& pose, const DexhandPreIkConditioningConfig& config) {
  if (!pose.position.allFinite() || !isProperRotation(pose.rotation)) {
    throw std::invalid_argument("Dexhand target conditioner received invalid pose");
  }
  requireFinitePositive(config.rate_hz, "dexhand pre-IK rate_hz");
  if (!config.translation_gain.allFinite() ||
      (config.translation_gain.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "dexhand pre-IK translation_gain must be finite and positive");
  }
  if (!std::isfinite(config.rotation_gain) || config.rotation_gain <= 0.0 ||
      config.rotation_gain > 1.0) {
    throw std::invalid_argument(
        "dexhand pre-IK rotation_gain must be in (0, 1]");
  }
  if (!config.workspace_relative_radii_m.allFinite() ||
      (config.workspace_relative_radii_m.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "dexhand pre-IK workspace radii must be finite and positive");
  }
  if (!std::isfinite(config.workspace_soft_zone_ratio) ||
      config.workspace_soft_zone_ratio <= 0.0 ||
      config.workspace_soft_zone_ratio >= 1.0) {
    throw std::invalid_argument(
        "dexhand pre-IK workspace_soft_zone_ratio must be in (0, 1)");
  }
  requireFinitePositive(config.maximum_linear_speed_m_s,
                        "dexhand pre-IK maximum_linear_speed_m_s");
  requireFinitePositive(config.maximum_angular_speed_rad_s,
                        "dexhand pre-IK maximum_angular_speed_rad_s");
  requireFinitePositive(config.maximum_linear_acceleration_m_s2,
                        "dexhand pre-IK maximum_linear_acceleration_m_s2");
  requireFinitePositive(config.maximum_angular_acceleration_rad_s2,
                        "dexhand pre-IK maximum_angular_acceleration_rad_s2");
}

void DexhandTargetConditioner::reset() noexcept {
  position_state_ = origin_;
  linear_velocity_.setZero();
  angular_velocity_.setZero();
  diagnostics_ = DexhandTargetConditioningDiagnostics{};
}

void DexhandTargetConditioner::synchronize(const Pose& pose) {
  if (!pose.position.allFinite() || !isProperRotation(pose.rotation)) {
    throw std::invalid_argument(
        "Dexhand target conditioner cannot synchronize invalid pose");
  }
  position_state_ = pose;
  linear_velocity_.setZero();
  angular_velocity_.setZero();
  diagnostics_ = DexhandTargetConditioningDiagnostics{};
}

Pose DexhandTargetConditioner::condition(const Pose& requested) {
  if (!requested.position.allFinite() || !isProperRotation(requested.rotation)) {
    throw std::invalid_argument(
        "Dexhand target conditioner received invalid target pose");
  }
  if (!config_.enabled) {
    diagnostics_ = DexhandTargetConditioningDiagnostics{};
    return requested;
  }

  const double rate = config_.rate_hz;
  Eigen::Vector3d relative =
      (requested.position - origin_.position).cwiseProduct(
          config_.translation_gain);
  double utilization =
      (relative.cwiseQuotient(config_.workspace_relative_radii_m)).norm();
  const double requested_utilization = utilization;
  const bool workspace_limited =
      utilization > config_.workspace_soft_zone_ratio;
  if (workspace_limited && utilization > 1.0e-12) {
    const double soft = config_.workspace_soft_zone_ratio;
    const double mapped_utilization =
        soft + (1.0 - soft) *
                   (1.0 - std::exp(-(utilization - soft) / (1.0 - soft)));
    relative *= mapped_utilization / utilization;
    utilization = mapped_utilization;
  }
  Pose desired;
  desired.position = origin_.position + relative;
  const Eigen::Vector3d requested_rotation_vector = so3Log(
      origin_.rotation.transpose() * requested.rotation);
  desired.rotation = origin_.rotation *
                     expMap(requested_rotation_vector * config_.rotation_gain);

  const Eigen::Vector3d requested_linear_velocity =
      (desired.position - position_state_.position) * rate;
  bool linear_speed_limited = false;
  const Eigen::Vector3d desired_linear_velocity = limitedNorm(
      requested_linear_velocity, config_.maximum_linear_speed_m_s,
      &linear_speed_limited);
  const Eigen::Vector3d linear_acceleration =
      (desired_linear_velocity - linear_velocity_) * rate;
  bool linear_acceleration_limited = false;
  const Eigen::Vector3d limited_linear_acceleration = limitedNorm(
      linear_acceleration, config_.maximum_linear_acceleration_m_s2,
      &linear_acceleration_limited);
  linear_velocity_ += limited_linear_acceleration / rate;
  Eigen::Vector3d linear_step = linear_velocity_ / rate;
  const Eigen::Vector3d remaining_position =
      desired.position - position_state_.position;
  if (linear_step.norm() >= remaining_position.norm()) {
    linear_step = remaining_position;
    linear_velocity_ = linear_step * rate;
  }
  position_state_.position += linear_step;

  const Eigen::Vector3d rotation_delta_vector = so3Log(
      position_state_.rotation.transpose() * desired.rotation);
  const Eigen::Vector3d requested_angular_velocity =
      rotation_delta_vector * rate;
  bool angular_speed_limited = false;
  const Eigen::Vector3d desired_angular_velocity = limitedNorm(
      requested_angular_velocity, config_.maximum_angular_speed_rad_s,
      &angular_speed_limited);
  const Eigen::Vector3d angular_acceleration =
      (desired_angular_velocity - angular_velocity_) * rate;
  bool angular_acceleration_limited = false;
  const Eigen::Vector3d limited_angular_acceleration = limitedNorm(
      angular_acceleration, config_.maximum_angular_acceleration_rad_s2,
      &angular_acceleration_limited);
  angular_velocity_ += limited_angular_acceleration / rate;
  Eigen::Vector3d angular_step = angular_velocity_ / rate;
  if (angular_step.norm() >= rotation_delta_vector.norm()) {
    angular_step = rotation_delta_vector;
    angular_velocity_ = angular_step * rate;
  }
  position_state_.rotation = position_state_.rotation * expMap(angular_step);

  diagnostics_.requested_workspace_utilization = requested_utilization;
  diagnostics_.workspace_utilization = utilization;
  diagnostics_.workspace_soft_limited = workspace_limited;
  diagnostics_.requested_linear_speed_m_s = requested_linear_velocity.norm();
  diagnostics_.applied_linear_speed_m_s = linear_velocity_.norm();
  diagnostics_.linear_speed_limited = linear_speed_limited;
  diagnostics_.linear_acceleration_limited = linear_acceleration_limited;
  diagnostics_.requested_angular_speed_rad_s =
      requested_angular_velocity.norm();
  diagnostics_.applied_angular_speed_rad_s = angular_velocity_.norm();
  diagnostics_.angular_speed_limited = angular_speed_limited;
  diagnostics_.angular_acceleration_limited = angular_acceleration_limited;
  return position_state_;
}

}  // namespace tianji_v131
