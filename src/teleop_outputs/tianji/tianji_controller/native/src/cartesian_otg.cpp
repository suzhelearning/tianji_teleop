#include "tianji_qp_ik/cartesian_otg.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, double maximum_norm) {
  const double norm = value.norm();
  if (norm <= maximum_norm || norm == 0.0) {
    return value;
  }
  return value * (maximum_norm / norm);
}

Eigen::Matrix3d expSo3(const Eigen::Vector3d& rotation_vector) {
  const double angle = rotation_vector.norm();
  if (angle < 1e-12) {
    Eigen::Matrix3d skew;
    skew << 0.0, -rotation_vector.z(), rotation_vector.y(),
        rotation_vector.z(), 0.0, -rotation_vector.x(),
        -rotation_vector.y(), rotation_vector.x(), 0.0;
    return Eigen::Matrix3d::Identity() + skew;
  }
  return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

bool finitePose(const Pose& pose) {
  return pose.position.allFinite() && isProperRotation(pose.rotation);
}

std::array<double, 3> toArray(const Eigen::Vector3d& vector) {
  return {vector.x(), vector.y(), vector.z()};
}

Eigen::Vector3d toEigen(const std::array<double, 3>& values) {
  return {values[0], values[1], values[2]};
}

std::array<double, 3> directionalLimits(const Eigen::Vector3d& direction,
                                        double maximum_norm) {
  constexpr double kMinimumFraction = 1e-9;
  Eigen::Vector3d factors = direction.cwiseAbs();
  if (factors.norm() > kMinimumFraction) {
    factors.normalize();
  } else {
    factors = Eigen::Vector3d::Constant(1.0 / std::sqrt(3.0));
  }
  for (int index = 0; index < 3; ++index) {
    factors[index] = std::max(factors[index], kMinimumFraction);
  }
  factors.normalize();
  return toArray(maximum_norm * factors);
}

std::array<double, 3> isotropicLimits(double maximum_norm) {
  const double per_axis = maximum_norm / std::sqrt(3.0);
  return {per_axis, per_axis, per_axis};
}

}  // namespace

CartesianReferenceGenerator::CartesianReferenceGenerator(
    CartesianOtgConfig config, double dt_seconds)
    : config_(config),
      translation_otg_(dt_seconds),
      orientation_otg_(dt_seconds) {
  if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
    throw std::invalid_argument("Cartesian OTG period must be finite and positive");
  }
  state_.valid = false;
}

void CartesianReferenceGenerator::reconfigure(CartesianOtgConfig config) {
  config_ = config;
  if (!orientation_limit_initialized_) {
    return;
  }
  orientation_input_.max_velocity =
      orientation_limits_isotropic_
          ? isotropicLimits(config_.angular_velocity_max)
          : directionalLimits(orientation_limit_direction_,
                              config_.angular_velocity_max);
  orientation_input_.max_acceleration =
      orientation_limits_isotropic_
          ? isotropicLimits(config_.angular_acceleration_max)
          : directionalLimits(orientation_limit_direction_,
                              config_.angular_acceleration_max);
  orientation_input_.max_jerk =
      orientation_limits_isotropic_
          ? isotropicLimits(config_.angular_jerk_max)
          : directionalLimits(orientation_limit_direction_,
                              config_.angular_jerk_max);
}

void CartesianReferenceGenerator::reset(const Pose& measured) {
  if (!finitePose(measured)) {
    throw std::invalid_argument("Cartesian OTG reset pose is invalid");
  }
  state_ = {};
  state_.pose = measured;
  state_.valid = true;
  initialized_ = true;
  translation_otg_.reset();
  orientation_otg_.reset();
  orientation_position_.setZero();
  orientation_target_position_.setZero();
  orientation_limit_target_ = measured.rotation;
  orientation_limit_direction_ = Eigen::Vector3d::UnitX();
  orientation_limit_initialized_ = false;
  orientation_limits_isotropic_ = false;
  translation_hold_active_ = false;
  translation_quiet_seconds_ = 0.0;
  translation_hold_target_ = measured.position;
  orientation_hold_active_ = false;
  orientation_quiet_seconds_ = 0.0;
  orientation_hold_target_ = measured.rotation;
}

CartesianReference CartesianReferenceGenerator::update(
    const Pose& target, const Vec6& target_twist, bool stale,
    double dt_seconds) {
  if (!initialized_ || !finitePose(target) || !target_twist.allFinite() ||
      !std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
    CartesianReference rejected = state_;
    rejected.valid = false;
    rejected.stale = stale;
    return rejected;
  }

  if (!config_.stationary_hold_enabled) {
    translation_hold_active_ = false;
    translation_quiet_seconds_ = 0.0;
    orientation_hold_active_ = false;
    orientation_quiet_seconds_ = 0.0;
  } else if (!stale) {
    const double target_linear_speed = target_twist.head<3>().norm();
    if (translation_hold_active_) {
      const bool intentional_translation =
          target_linear_speed > config_.translation_hold_exit_velocity_m_s ||
          (target.position - translation_hold_target_).norm() >
              config_.translation_hold_exit_position_error_m;
      if (intentional_translation) {
        translation_hold_active_ = false;
        translation_quiet_seconds_ = 0.0;
      }
    } else if (target_linear_speed <=
               config_.translation_hold_enter_velocity_m_s) {
      translation_quiet_seconds_ += dt_seconds;
      if (translation_quiet_seconds_ >=
          config_.stationary_hold_dwell_seconds) {
        translation_hold_active_ = true;
        translation_hold_target_ = target.position;
        translation_quiet_seconds_ = 0.0;
      }
    } else {
      translation_quiet_seconds_ = 0.0;
    }

    const double target_angular_speed = target_twist.tail<3>().norm();
    if (orientation_hold_active_) {
      const bool intentional_rotation =
          target_angular_speed >
              config_.orientation_hold_exit_velocity_rad_s ||
          rotationDistance(orientation_hold_target_, target.rotation) >
              config_.orientation_hold_exit_error_rad;
      if (intentional_rotation) {
        orientation_hold_active_ = false;
        orientation_quiet_seconds_ = 0.0;
      }
    } else if (target_angular_speed <=
               config_.orientation_hold_enter_velocity_rad_s) {
      orientation_quiet_seconds_ += dt_seconds;
      if (orientation_quiet_seconds_ >=
          config_.stationary_hold_dwell_seconds) {
        orientation_hold_active_ = true;
        orientation_hold_target_ = target.rotation;
        orientation_quiet_seconds_ = 0.0;
      }
    } else {
      orientation_quiet_seconds_ = 0.0;
    }
  }

  const Eigen::Vector3d effective_target_position =
      translation_hold_active_ ? translation_hold_target_ : target.position;
  const Eigen::Matrix3d effective_target_rotation =
      orientation_hold_active_ ? orientation_hold_target_ : target.rotation;

  CartesianReference candidate = state_;
  const bool translation_position_mode =
      config_.translation_position_mode;
  const bool translation_tracking_active =
      !translation_position_mode && config_.translation_tracking_enabled &&
      !translation_hold_active_ && !stale &&
      target_twist.head<3>().norm() >
          config_.translation_stationary_velocity_threshold;
  translation_otg_.delta_time = dt_seconds;
  translation_input_.control_interface =
      !translation_position_mode && translation_tracking_active
          ? ruckig::ControlInterface::Velocity
          : ruckig::ControlInterface::Position;
  translation_input_.synchronization = ruckig::Synchronization::Time;
  translation_input_.current_position = toArray(state_.pose.position);
  translation_input_.current_velocity = toArray(state_.twist.head<3>());
  translation_input_.current_acceleration =
      toArray(state_.acceleration.head<3>());
  Eigen::Vector3d translation_feedforward = Eigen::Vector3d::Zero();
  if (!translation_position_mode && !translation_hold_active_ && !stale &&
      (!config_.translation_tracking_enabled || translation_tracking_active)) {
    translation_feedforward = target_twist.head<3>();
  }
  Eigen::Vector3d translation_correction = Eigen::Vector3d::Zero();
  if (!translation_position_mode && translation_tracking_active) {
    translation_correction = config_.translation_tracking_gain *
                             (effective_target_position - state_.pose.position);
  }
  const Eigen::Vector3d translation_velocity =
      translation_position_mode
          ? Eigen::Vector3d::Zero()
          : clampNorm(translation_feedforward + translation_correction,
                      config_.translation_velocity_max);
  translation_input_.target_position = toArray(effective_target_position);
  translation_input_.target_velocity = toArray(translation_velocity);
  translation_input_.target_acceleration = {0.0, 0.0, 0.0};
  translation_input_.max_velocity = {
      config_.translation_velocity_max, config_.translation_velocity_max,
      config_.translation_velocity_max};
  translation_input_.max_acceleration = {
      config_.translation_acceleration_max,
      config_.translation_acceleration_max,
      config_.translation_acceleration_max};
  translation_input_.max_jerk = {
      config_.translation_jerk_max, config_.translation_jerk_max,
      config_.translation_jerk_max};

  const ruckig::Result translation_result =
      translation_otg_.update(translation_input_, translation_output_);
  if (static_cast<int>(translation_result) < 0) {
    CartesianReference rejected = state_;
    rejected.valid = false;
    rejected.stale = stale;
    translation_otg_.reset();
    return rejected;
  }

  candidate.pose.position = toEigen(translation_output_.new_position);
  candidate.twist.head<3>() = toEigen(translation_output_.new_velocity);
  candidate.acceleration.head<3>() =
      toEigen(translation_output_.new_acceleration);

  const Eigen::Vector3d orientation_error =
      so3Log(effective_target_rotation * state_.pose.rotation.transpose());
  orientation_otg_.delta_time = dt_seconds;
  orientation_input_.control_interface = ruckig::ControlInterface::Position;
  orientation_input_.synchronization = ruckig::Synchronization::Time;
  orientation_input_.current_position = toArray(orientation_position_);
  orientation_input_.current_velocity = toArray(state_.twist.tail<3>());
  orientation_input_.current_acceleration =
      toArray(state_.acceleration.tail<3>());
  const Eigen::Vector3d angular_target_velocity =
      (stale || orientation_hold_active_)
          ? Eigen::Vector3d::Zero()
            : clampNorm(target_twist.tail<3>(),
                        config_.angular_velocity_max);
  orientation_input_.target_acceleration = {0.0, 0.0, 0.0};
  const bool orientation_path_exhausted_with_residual =
      state_.twist.tail<3>().norm() <=
          config_.angular_settle_velocity_rad_s &&
      state_.acceleration.tail<3>().norm() <=
          config_.angular_settle_acceleration_rad_s2 &&
      orientation_error.norm() > config_.orientation_settle_error_rad;
  const bool orientation_target_changed =
      !orientation_limit_initialized_ ||
      rotationDistance(orientation_limit_target_, effective_target_rotation) >
          1e-10 ||
      orientation_path_exhausted_with_residual;
  if (orientation_target_changed) {
    const bool had_orientation_limits = orientation_limit_initialized_;
    Eigen::Vector3d new_limit_direction = orientation_error;
    if (new_limit_direction.norm() <= 1e-12) {
      new_limit_direction = state_.twist.tail<3>();
    }
    if (had_orientation_limits && !orientation_limits_isotropic_ &&
        new_limit_direction.norm() > 1e-12 &&
        orientation_limit_direction_.norm() > 1e-12) {
      const double axis_alignment = std::abs(
          new_limit_direction.normalized().dot(
              orientation_limit_direction_.normalized()));
      if (axis_alignment < 1.0 - 1e-6) {
        orientation_limits_isotropic_ = true;
      }
    }
    orientation_limit_target_ = effective_target_rotation;
    orientation_target_position_ = orientation_position_ + orientation_error;
    orientation_limit_direction_ = new_limit_direction;
    orientation_limit_initialized_ = true;
    if (!had_orientation_limits || orientation_limits_isotropic_) {
      orientation_input_.max_velocity = orientation_limits_isotropic_
          ? isotropicLimits(config_.angular_velocity_max)
          : directionalLimits(orientation_limit_direction_,
                              config_.angular_velocity_max);
      orientation_input_.max_acceleration = orientation_limits_isotropic_
          ? isotropicLimits(config_.angular_acceleration_max)
          : directionalLimits(orientation_limit_direction_,
                              config_.angular_acceleration_max);
      orientation_input_.max_jerk = orientation_limits_isotropic_
          ? isotropicLimits(config_.angular_jerk_max)
          : directionalLimits(orientation_limit_direction_,
                              config_.angular_jerk_max);
    }
  }
  orientation_input_.target_position = toArray(orientation_target_position_);
  Eigen::Vector3d feasible_target_velocity = angular_target_velocity;
  for (int index = 0; index < 3; ++index) {
    feasible_target_velocity[index] = std::clamp(
        feasible_target_velocity[index],
        -orientation_input_.max_velocity[static_cast<std::size_t>(index)],
        orientation_input_.max_velocity[static_cast<std::size_t>(index)]);
  }
  orientation_input_.target_velocity = toArray(feasible_target_velocity);

  const ruckig::Result orientation_result =
      orientation_otg_.update(orientation_input_, orientation_output_);
  if (static_cast<int>(orientation_result) < 0) {
    CartesianReference rejected = state_;
    rejected.valid = false;
    rejected.stale = stale;
    orientation_otg_.reset();
    return rejected;
  }

  const Eigen::Vector3d next_orientation_position =
      toEigen(orientation_output_.new_position);
  candidate.pose.rotation =
      expSo3(next_orientation_position - orientation_position_) *
      state_.pose.rotation;
  candidate.twist.tail<3>() =
      toEigen(orientation_output_.new_velocity);
  candidate.acceleration.tail<3>() =
      toEigen(orientation_output_.new_acceleration);
  const double remaining_orientation_error =
      rotationDistance(candidate.pose.rotation, effective_target_rotation);
  const bool orientation_settled =
      angular_target_velocity.norm() <=
          config_.angular_settle_velocity_rad_s &&
      remaining_orientation_error <= config_.orientation_settle_error_rad &&
      candidate.twist.tail<3>().norm() <=
          config_.angular_settle_velocity_rad_s &&
      candidate.acceleration.tail<3>().norm() <=
          config_.angular_settle_acceleration_rad_s2;
  if (orientation_settled) {
    candidate.pose.rotation = effective_target_rotation;
    candidate.twist.tail<3>().setZero();
    candidate.acceleration.tail<3>().setZero();
  }
  candidate.stale = stale;
  candidate.valid = finitePose(candidate.pose) && candidate.twist.allFinite() &&
                    candidate.acceleration.allFinite();
  if (!candidate.valid) {
    CartesianReference rejected = state_;
    rejected.valid = false;
    rejected.stale = stale;
    return rejected;
  }

  if (orientation_settled) {
    orientation_otg_.reset();
    orientation_position_.setZero();
    orientation_target_position_.setZero();
    orientation_limit_initialized_ = false;
    orientation_limits_isotropic_ = false;
  } else {
    orientation_position_ = next_orientation_position;
  }
  state_ = candidate;
  return state_;
}

}  // namespace tianji_qp_ik
