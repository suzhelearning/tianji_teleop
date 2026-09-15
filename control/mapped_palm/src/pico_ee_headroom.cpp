#include "tianji_mapped_palm/pico_ee_headroom.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tianji_mapped_palm {
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

PicoEeFeedforwardAllocation allocatePicoEeFeedforward(
    const Vec6& low_frequency_twist, const Vec6& high_frequency_twist,
    double headroom_scale, double low_frequency_min_scale,
    double high_frequency_min_scale) noexcept {
  PicoEeFeedforwardAllocation result;
  if (!low_frequency_twist.allFinite() ||
      !high_frequency_twist.allFinite() || !std::isfinite(headroom_scale) ||
      !std::isfinite(low_frequency_min_scale) ||
      !std::isfinite(high_frequency_min_scale) ||
      low_frequency_min_scale < 0.0 || low_frequency_min_scale > 1.0 ||
      high_frequency_min_scale < 0.0 || high_frequency_min_scale > 1.0) {
    return result;
  }
  const double bounded_headroom = std::clamp(headroom_scale, 0.0, 1.0);
  result.high_frequency_scale =
      std::max(bounded_headroom, high_frequency_min_scale);
  result.low_frequency_scale =
      std::max(bounded_headroom, low_frequency_min_scale);
  result.twist =
      result.low_frequency_scale * low_frequency_twist +
      result.high_frequency_scale * high_frequency_twist;
  result.valid = result.twist.allFinite();
  if (!result.valid) {
    result = {};
  }
  return result;
}

double picoEeRedundancyAuthority(
    const PicoEeHeadroomConfig& config,
    const PicoEeHeadroomResult& headroom) noexcept {
  if (!config.redundancy_authority_enabled || !headroom.valid ||
      !std::isfinite(headroom.filtered_headroom)) {
    return 1.0;
  }
  const double phase = std::clamp(
      (headroom.filtered_headroom - config.redundancy_zero_headroom) /
          (config.redundancy_full_headroom -
           config.redundancy_zero_headroom),
      0.0, 1.0);
  return phase * phase * phase *
         (10.0 + phase * (-15.0 + 6.0 * phase));
}

PicoEeHeadroomGovernor::PicoEeHeadroomGovernor(
    SparkHeadroomFeedforwardVelocityQpConfig config, ArmLimits limits,
    JointAccelerationLimitConfig dynamic_limits)
    : config_(std::move(config)),
      limits_(std::move(limits)),
      dynamic_limits_(std::move(dynamic_limits)) {
  reset();
}

void PicoEeHeadroomGovernor::reset() noexcept {
  previous_acceleration_valid_ = false;
  previous_acceleration_.setZero();
  filtered_jerk_usage_ = 0.0;
  result_ = {};
}

double PicoEeHeadroomGovernor::smoothstep5(double value) noexcept {
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

double PicoEeHeadroomGovernor::normalizedTaskHeadroom(
    double scale, double minimum) noexcept {
  return std::clamp((scale - minimum) / (1.0 - minimum), 0.0, 1.0);
}

PicoEeHeadroomResult PicoEeHeadroomGovernor::update(
    const PicoEeHeadroomFeedback& feedback, double dt) noexcept {
  const bool finite = feedback.qdot.allFinite() && feedback.qddot.allFinite() &&
                      std::isfinite(feedback.task_scale_position) &&
                      std::isfinite(feedback.task_scale_orientation) &&
                      std::isfinite(dt) && dt > 0.0;
  if (!feedback.accepted || !finite || !config_.enabled) {
    previous_acceleration_valid_ = false;
    result_.valid = false;
    result_.derivative_history_valid = false;
    result_.raw_headroom = 0.0;
    result_.dominant_source = finite ? PicoEeHeadroomSource::kNone
                                     : PicoEeHeadroomSource::kInvalid;
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
      const double time_constant =
          attacking ? config_.jerk_usage_attack_seconds
                    : config_.jerk_usage_release_seconds;
      const double alpha = 1.0 - std::exp(-dt / time_constant);
      filtered_jerk_usage_ +=
          alpha * (instantaneous_jerk_usage - filtered_jerk_usage_);
      result_.jerk_headroom = headroomFromUsage(filtered_jerk_usage_);
    } else {
      result_.jerk_headroom = 0.0;
    }
    previous_acceleration_ = feedback.qddot;
    previous_acceleration_valid_ = true;
    result_.raw_headroom = result_.velocity_headroom;
    result_.dominant_source = PicoEeHeadroomSource::kVelocity;
    const auto select = [this](double value, PicoEeHeadroomSource source) {
      if (value < result_.raw_headroom) {
        result_.raw_headroom = value;
        result_.dominant_source = source;
      }
    };
    select(result_.acceleration_headroom,
           PicoEeHeadroomSource::kAcceleration);
    select(result_.jerk_headroom, PicoEeHeadroomSource::kJerk);
    select(result_.task_headroom, PicoEeHeadroomSource::kTaskScaling);
    result_.valid = result_.derivative_history_valid;
    if (!result_.derivative_history_valid) {
      result_.raw_headroom = 0.0;
      result_.dominant_source = PicoEeHeadroomSource::kJerk;
    }
  }

  const bool reducing = result_.raw_headroom < result_.filtered_headroom;
  const double time_constant =
      reducing ? config_.reduction_time_seconds : config_.recovery_time_seconds;
  if (!std::isfinite(time_constant) || time_constant <= 0.0 ||
      !std::isfinite(dt) || dt <= 0.0) {
    result_.scale = 0.0;
    result_.dominant_source = PicoEeHeadroomSource::kInvalid;
    return result_;
  }
  const double alpha = 1.0 - std::exp(-dt / time_constant);
  result_.filtered_headroom +=
      alpha * (result_.raw_headroom - result_.filtered_headroom);
  const double normalized =
      (result_.filtered_headroom - config_.low_headroom) /
      (config_.full_headroom - config_.low_headroom);
  result_.scale = smoothstep5(normalized);
  return result_;
}

std::string_view toString(PicoEeHeadroomSource source) noexcept {
  switch (source) {
    case PicoEeHeadroomSource::kNone:
      return "none";
    case PicoEeHeadroomSource::kVelocity:
      return "velocity";
    case PicoEeHeadroomSource::kAcceleration:
      return "acceleration";
    case PicoEeHeadroomSource::kJerk:
      return "jerk";
    case PicoEeHeadroomSource::kTaskScaling:
      return "task_scaling";
    case PicoEeHeadroomSource::kInvalid:
      return "invalid";
  }
  return "invalid";
}

}  // namespace tianji_mapped_palm
