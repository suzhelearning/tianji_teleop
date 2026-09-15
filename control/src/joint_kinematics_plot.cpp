#include "tianji_qp_ik/joint_kinematics_plot.hpp"

#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

const Vec7& metricValue(const JointKinematicsState& state,
                        PlotMetric metric) noexcept {
  switch (metric) {
    case PlotMetric::kPosition:
      return state.position;
    case PlotMetric::kVelocity:
      return state.velocity;
    case PlotMetric::kAcceleration:
      return state.acceleration;
    case PlotMetric::kJerk:
      return state.jerk;
  }
  return state.position;
}

const Vec7& lowerBound(const JointKinematicsBounds& bounds,
                       PlotMetric metric) noexcept {
  switch (metric) {
    case PlotMetric::kPosition:
      return bounds.position_lower;
    case PlotMetric::kVelocity:
      return bounds.velocity_lower;
    case PlotMetric::kAcceleration:
      return bounds.acceleration_lower;
    case PlotMetric::kJerk:
      return bounds.jerk_lower;
  }
  return bounds.position_lower;
}

const Vec7& upperBound(const JointKinematicsBounds& bounds,
                       PlotMetric metric) noexcept {
  switch (metric) {
    case PlotMetric::kPosition:
      return bounds.position_upper;
    case PlotMetric::kVelocity:
      return bounds.velocity_upper;
    case PlotMetric::kAcceleration:
      return bounds.acceleration_upper;
    case PlotMetric::kJerk:
      return bounds.jerk_upper;
  }
  return bounds.position_upper;
}

}  // namespace

JointKinematicsDerivatives JointKinematicsDifferentiator::update(
    const Vec7& reference_velocity,
    const Vec7& direct_reference_acceleration,
    ReferenceAccelerationSource source, const Vec7& actual_velocity,
    double fixed_dt, bool reset_requested) noexcept {
  JointKinematicsDerivatives output;
  const bool input_valid = reference_velocity.allFinite() &&
                           direct_reference_acceleration.allFinite() &&
                           actual_velocity.allFinite() &&
                           std::isfinite(fixed_dt) && fixed_dt > 0.0;
  const bool source_changed = source_initialized_ && source != previous_source_;
  if (reset_requested || source_changed || !input_valid) {
    reset();
  }
  if (!input_valid) {
    return output;
  }

  if (source == ReferenceAccelerationSource::kDirectQpOutput) {
    output.reference_acceleration = direct_reference_acceleration;
    output.reference_acceleration_valid = true;
  } else if (reference_velocity_initialized_) {
    output.reference_acceleration =
        (reference_velocity - previous_reference_velocity_) / fixed_dt;
    output.reference_acceleration_valid =
        output.reference_acceleration.allFinite();
  }

  if (output.reference_acceleration_valid &&
      reference_acceleration_initialized_) {
    output.reference_jerk =
        (output.reference_acceleration - previous_reference_acceleration_) /
        fixed_dt;
    output.reference_jerk_valid = output.reference_jerk.allFinite();
  }

  if (actual_velocity_initialized_) {
    output.actual_acceleration =
        (actual_velocity - previous_actual_velocity_) / fixed_dt;
    output.actual_acceleration_valid = output.actual_acceleration.allFinite();
  }
  if (output.actual_acceleration_valid && actual_acceleration_initialized_) {
    output.actual_jerk =
        (output.actual_acceleration - previous_actual_acceleration_) / fixed_dt;
    output.actual_jerk_valid = output.actual_jerk.allFinite();
  }

  source_initialized_ = true;
  previous_source_ = source;
  reference_velocity_initialized_ = true;
  previous_reference_velocity_ = reference_velocity;
  actual_velocity_initialized_ = true;
  previous_actual_velocity_ = actual_velocity;
  if (output.reference_acceleration_valid) {
    reference_acceleration_initialized_ = true;
    previous_reference_acceleration_ = output.reference_acceleration;
  }
  if (output.actual_acceleration_valid) {
    actual_acceleration_initialized_ = true;
    previous_actual_acceleration_ = output.actual_acceleration;
  }
  return output;
}

void JointKinematicsDifferentiator::reset() noexcept {
  source_initialized_ = false;
  reference_velocity_initialized_ = false;
  reference_acceleration_initialized_ = false;
  actual_velocity_initialized_ = false;
  actual_acceleration_initialized_ = false;
  previous_reference_velocity_.setZero();
  previous_reference_acceleration_.setZero();
  previous_actual_velocity_.setZero();
  previous_actual_acceleration_.setZero();
}

JointKinematicsHistory::JointKinematicsHistory(std::size_t capacity)
    : samples_(capacity) {
  if (capacity == 0U) {
    throw std::invalid_argument("joint plot history capacity must be positive");
  }
}

void JointKinematicsHistory::push(const JointKinematicsSample& sample) noexcept {
  if (size_ < samples_.size()) {
    samples_[(begin_ + size_) % samples_.size()] = sample;
    ++size_;
    return;
  }
  samples_[begin_] = sample;
  begin_ = (begin_ + 1U) % samples_.size();
}

void JointKinematicsHistory::clear() noexcept {
  begin_ = 0U;
  size_ = 0U;
}

const JointKinematicsSample& JointKinematicsHistory::chronological(
    std::size_t index) const noexcept {
  return samples_[(begin_ + index) % samples_.size()];
}

JointPlotSeries JointKinematicsHistory::series(ArmSide side, PlotMetric metric,
                                               int joint) const {
  if (joint < 0 || joint >= kArmDof) {
    throw std::out_of_range("joint plot index out of range");
  }
  JointPlotSeries output;
  output.time.reserve(size_);
  output.reference.reserve(size_);
  output.actual.reserve(size_);
  output.lower.reserve(size_);
  output.upper.reserve(size_);
  output.reference_valid.reserve(size_);
  output.actual_valid.reserve(size_);
  for (std::size_t index = 0U; index < size_; ++index) {
    const JointKinematicsSample& sample = chronological(index);
    const ArmJointKinematicsSample& arm =
        side == ArmSide::kLeft ? sample.left : sample.right;
    output.time.push_back(sample.time_seconds);
    output.reference.push_back(metricValue(arm.reference, metric)[joint]);
    output.actual.push_back(metricValue(arm.actual, metric)[joint]);
    output.lower.push_back(lowerBound(arm.bounds, metric)[joint]);
    output.upper.push_back(upperBound(arm.bounds, metric)[joint]);

    bool reference_valid = true;
    bool actual_valid = true;
    if (metric == PlotMetric::kAcceleration) {
      reference_valid = arm.reference_acceleration_valid;
      actual_valid = arm.actual_acceleration_valid;
    } else if (metric == PlotMetric::kJerk) {
      reference_valid = arm.reference_jerk_valid;
      actual_valid = arm.actual_jerk_valid;
    }
    output.reference_valid.push_back(reference_valid);
    output.actual_valid.push_back(actual_valid);
  }
  return output;
}

const char* plotMetricName(PlotMetric metric) noexcept {
  switch (metric) {
    case PlotMetric::kPosition:
      return "q";
    case PlotMetric::kVelocity:
      return "dq";
    case PlotMetric::kAcceleration:
      return "ddq";
    case PlotMetric::kJerk:
      return "jerk";
  }
  return "q";
}

const char* plotMetricUnit(PlotMetric metric) noexcept {
  switch (metric) {
    case PlotMetric::kPosition:
      return "rad";
    case PlotMetric::kVelocity:
      return "rad/s";
    case PlotMetric::kAcceleration:
      return "rad/s^2";
    case PlotMetric::kJerk:
      return "rad/s^3";
  }
  return "rad";
}

PlotMetric nextPlotMetric(PlotMetric metric) noexcept {
  switch (metric) {
    case PlotMetric::kPosition:
      return PlotMetric::kVelocity;
    case PlotMetric::kVelocity:
      return PlotMetric::kAcceleration;
    case PlotMetric::kAcceleration:
      return PlotMetric::kJerk;
    case PlotMetric::kJerk:
      return PlotMetric::kPosition;
  }
  return PlotMetric::kPosition;
}

}  // namespace tianji_qp_ik
