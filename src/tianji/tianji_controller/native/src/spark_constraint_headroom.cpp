#include "tianji_qp_ik/spark_constraint_headroom.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tianji_qp_ik {
namespace {

double headroomFromUsage(double usage) noexcept {
  return std::clamp(1.0 - usage, 0.0, 1.0);
}

double maximumRatio(const Vec7& value, const Vec7& limit) noexcept {
  double result = 0.0;
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (!(limit[joint] > 0.0) || !std::isfinite(limit[joint])) {
      return std::numeric_limits<double>::infinity();
    }
    result = std::max(result, std::abs(value[joint]) / limit[joint]);
  }
  return result;
}

}  // namespace

SparkConstraintHeadroomGovernor::SparkConstraintHeadroomGovernor(
    SparkHeadroomFeedforwardVelocityQpConfig config, ArmLimits limits,
    JointAccelerationLimitConfig dynamic_limits)
    : config_(std::move(config)),
      limits_(std::move(limits)),
      dynamic_limits_(std::move(dynamic_limits)) {
  reset();
}

void SparkConstraintHeadroomGovernor::reset() noexcept {
  previous_acceleration_valid_ = false;
  previous_acceleration_.setZero();
  filtered_jerk_usage_ = 0.0;
  result_ = {};
  result_.state = SparkHeadroomState::kInactive;
}

double SparkConstraintHeadroomGovernor::smoothstep5(double value) noexcept {
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

double SparkConstraintHeadroomGovernor::normalizedTaskHeadroom(
    double scale, double minimum) noexcept {
  return std::clamp((scale - minimum) / (1.0 - minimum), 0.0, 1.0);
}

SparkConstraintHeadroomResult SparkConstraintHeadroomGovernor::update(
    const SparkConstraintHeadroomFeedback& feedback, double dt) noexcept {
  const bool finite = feedback.qdot.allFinite() && feedback.qddot.allFinite() &&
                      std::isfinite(feedback.task_scale_position) &&
                      std::isfinite(feedback.task_scale_orientation) &&
                      std::isfinite(dt) && dt > 0.0;
  if (!feedback.accepted || !finite || !config_.enabled) {
    previous_acceleration_valid_ = false;
    result_.valid = false;
    result_.derivative_history_valid = false;
    result_.raw_headroom = 0.0;
    result_.dominant_source = finite ? SparkHeadroomSource::kNone
                                     : SparkHeadroomSource::kInvalid;
    result_.state = finite ? SparkHeadroomState::kReducing
                           : SparkHeadroomState::kInvalid;
  } else {
    result_.velocity_headroom = headroomFromUsage(
        maximumRatio(feedback.qdot, limits_.velocity));
    result_.acceleration_headroom = headroomFromUsage(maximumRatio(
        feedback.qddot, dynamic_limits_.max_acceleration_rad_s2));
    result_.task_headroom = std::min(
        normalizedTaskHeadroom(feedback.task_scale_position,
                               config_.task_scaling_min_position),
        normalizedTaskHeadroom(feedback.task_scale_orientation,
                               config_.task_scaling_min_orientation));
    result_.derivative_history_valid = previous_acceleration_valid_;
    if (previous_acceleration_valid_) {
      const double instantaneous_jerk_usage = std::clamp(
          maximumRatio((feedback.qddot - previous_acceleration_) / dt,
                       dynamic_limits_.max_jerk_rad_s3),
          0.0, 1.0);
      const bool attacking = instantaneous_jerk_usage > filtered_jerk_usage_;
      const double jerk_time_constant =
          attacking ? config_.jerk_usage_attack_seconds
                    : config_.jerk_usage_release_seconds;
      const double jerk_alpha = 1.0 - std::exp(-dt / jerk_time_constant);
      filtered_jerk_usage_ +=
          jerk_alpha * (instantaneous_jerk_usage - filtered_jerk_usage_);
      result_.jerk_headroom = headroomFromUsage(filtered_jerk_usage_);
    } else {
      result_.jerk_headroom = 0.0;
    }
    previous_acceleration_ = feedback.qddot;
    previous_acceleration_valid_ = true;

    result_.raw_headroom = result_.velocity_headroom;
    result_.dominant_source = SparkHeadroomSource::kVelocity;
    const auto select = [this](double value, SparkHeadroomSource source) {
      if (value < result_.raw_headroom) {
        result_.raw_headroom = value;
        result_.dominant_source = source;
      }
    };
    select(result_.acceleration_headroom,
           SparkHeadroomSource::kAcceleration);
    select(result_.jerk_headroom, SparkHeadroomSource::kJerk);
    select(result_.task_headroom, SparkHeadroomSource::kTaskScaling);
    result_.valid = result_.derivative_history_valid;
    if (!result_.derivative_history_valid) {
      result_.raw_headroom = 0.0;
      result_.dominant_source = SparkHeadroomSource::kJerk;
    }
  }

  const bool reducing = result_.raw_headroom < result_.filtered_headroom;
  const double time_constant = reducing ? config_.reduction_time_seconds
                                        : config_.recovery_time_seconds;
  const double alpha = 1.0 - std::exp(-dt / time_constant);
  result_.filtered_headroom +=
      alpha * (result_.raw_headroom - result_.filtered_headroom);
  const double normalized =
      (result_.filtered_headroom - config_.low_headroom) /
      (config_.full_headroom - config_.low_headroom);
  result_.scale = smoothstep5(normalized);
  if (result_.state != SparkHeadroomState::kInvalid) {
    result_.state = reducing ? SparkHeadroomState::kReducing
                    : result_.scale >= 1.0 - 1.0e-9
                        ? SparkHeadroomState::kTracking
                        : SparkHeadroomState::kRecovering;
  }
  return result_;
}

std::string_view toString(SparkHeadroomSource source) noexcept {
  switch (source) {
    case SparkHeadroomSource::kNone: return "none";
    case SparkHeadroomSource::kVelocity: return "velocity";
    case SparkHeadroomSource::kAcceleration: return "acceleration";
    case SparkHeadroomSource::kJerk: return "jerk";
    case SparkHeadroomSource::kTaskScaling: return "task_scaling";
    case SparkHeadroomSource::kInvalid: return "invalid";
  }
  return "invalid";
}

std::string_view toString(SparkHeadroomState state) noexcept {
  switch (state) {
    case SparkHeadroomState::kInactive: return "inactive";
    case SparkHeadroomState::kRecovering: return "recovering";
    case SparkHeadroomState::kTracking: return "tracking";
    case SparkHeadroomState::kReducing: return "reducing";
    case SparkHeadroomState::kInvalid: return "invalid";
  }
  return "invalid";
}

}  // namespace tianji_qp_ik
