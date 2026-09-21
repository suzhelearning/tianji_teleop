#include "tianji_qp_ik/spark_feedforward_reference.hpp"

#include "tianji_qp_ik/velocity_ik.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace tianji_qp_ik {
namespace {

Vec7 shortestAngularDifference(const Vec7& current,
                               const Vec7& previous) noexcept {
  Vec7 result;
  for (int joint = 0; joint < kArmDof; ++joint) {
    const double difference = current[joint] - previous[joint];
    result[joint] = std::atan2(std::sin(difference), std::cos(difference));
  }
  return result;
}

bool validVector(const Vec7& value) noexcept { return value.allFinite(); }

}  // namespace

SparkFeedforwardReference7::SparkFeedforwardReference7(
    SparkFeedforwardVelocityQpConfig config, ArmLimits limits,
    JointLimitConfig joint_limits, double initial_dt)
    : config_(std::move(config)),
      limits_(std::move(limits)),
      joint_limits_(std::move(joint_limits)),
      nominal_dt_(initial_dt) {
  joint_limits_.velocity_scale *= config_.reference_velocity_scale;
  joint_limits_.max_acceleration_rad_s2 *=
      config_.reference_acceleration_scale;
  joint_limits_.max_jerk_rad_s3 *= config_.reference_jerk_scale;
  maximum_acceleration_ = joint_limits_.max_acceleration_rad_s2;
  maximum_jerk_ = joint_limits_.max_jerk_rad_s3;
}

void SparkFeedforwardReference7::reset(
    const ArmMotionState& model_state,
    std::uint64_t tracking_epoch) noexcept {
  source_initialized_ = false;
  tracking_epoch_ = tracking_epoch;
  last_sequence_ = 0U;
  last_source_timestamp_ns_ = 0;
  valid_source_dts_.clear();
  previous_raw_target_ = model_state.q;
  raw_continuous_target_ = model_state.q;
  continuous_target_ = model_state.q;
  estimated_q_ = model_state.q;
  estimated_qdot_.setZero();
  feedforward_confidence_.setZero();
  activation_phase_ = 0.0;
  result_ = {};
  result_.valid = validVector(model_state.q);
  result_.q = model_state.q;
  result_.state = SparkFeedforwardState::kHold;
  result_.detail = "feedforward_reset";
}

double SparkFeedforwardReference7::medianSourceDt() const noexcept {
  if (valid_source_dts_.empty()) {
    return 0.0;
  }
  std::vector<double> sorted(valid_source_dts_.begin(),
                             valid_source_dts_.end());
  const std::size_t middle = sorted.size() / 2U;
  std::nth_element(sorted.begin(), sorted.begin() +
                                       static_cast<std::ptrdiff_t>(middle),
                   sorted.end());
  return sorted[middle];
}

SparkFeedforwardTargetDecision SparkFeedforwardReference7::acceptTarget(
    const Vec7& q_ik, std::uint64_t sequence,
    std::int64_t source_timestamp_ns, std::uint64_t tracking_epoch,
    const ArmMotionState& model_state) noexcept {
  SparkFeedforwardTargetDecision decision;
  if (!validVector(q_ik) || sequence == 0U || source_timestamp_ns <= 0) {
    decision.detail = "feedforward_invalid_target";
    return decision;
  }

  if (tracking_epoch_ != tracking_epoch) {
    reset(model_state, tracking_epoch);
    decision.epoch_reset = true;
  }

  if (source_initialized_ &&
      (sequence <= last_sequence_ ||
       source_timestamp_ns <= last_source_timestamp_ns_)) {
    decision.detail = "feedforward_duplicate_source";
    decision.continuous_target = continuous_target_;
    decision.estimated_velocity = estimated_qdot_;
    decision.feedforward_confidence = feedforward_confidence_;
    return decision;
  }

  if (!source_initialized_) {
    source_initialized_ = true;
    last_sequence_ = sequence;
    last_source_timestamp_ns_ = source_timestamp_ns;
    previous_raw_target_ = q_ik;
    raw_continuous_target_ = q_ik;
    continuous_target_ = q_ik;
    estimated_q_ = q_ik;
    estimated_qdot_.setZero();
    // The first target starts from rest, so unit confidence retains the
    // critically damped acquisition response without injecting velocity.
    feedforward_confidence_.setOnes();
    result_.state = SparkFeedforwardState::kActive;
    decision.accepted = true;
    decision.continuous_target = continuous_target_;
    decision.estimated_velocity = estimated_qdot_;
    decision.feedforward_confidence = feedforward_confidence_;
    decision.detail = decision.epoch_reset ? "feedforward_epoch_started"
                                           : "feedforward_started";
    return decision;
  }

  const Vec7 delta = shortestAngularDifference(q_ik, previous_raw_target_);
  const double source_dt =
      static_cast<double>(source_timestamp_ns - last_source_timestamp_ns_) *
      1.0e-9;
  decision.source_dt_seconds = source_dt;
  decision.median_dt_seconds = medianSourceDt();

  last_sequence_ = sequence;
  last_source_timestamp_ns_ = source_timestamp_ns;
  previous_raw_target_ = q_ik;
  raw_continuous_target_ += delta;
  const Vec7 pending = raw_continuous_target_ - continuous_target_;
  decision.jump_rejected =
      pending.cwiseAbs().maxCoeff() > config_.maximum_joint_jump_rad;
  continuous_target_ +=
      pending.cwiseMax(Vec7::Constant(-config_.maximum_joint_jump_rad))
          .cwiseMin(Vec7::Constant(config_.maximum_joint_jump_rad));
  const bool enough_timing_history = valid_source_dts_.size() >= 3U;
  const double median_dt = medianSourceDt();
  const bool finite_positive_dt = std::isfinite(source_dt) && source_dt > 0.0;
  const bool dt_in_range =
      !enough_timing_history ||
      (source_dt >= config_.dt_min_ratio * median_dt &&
       source_dt <= config_.dt_max_ratio * median_dt);
  decision.dt_valid = finite_positive_dt && dt_in_range;

  if (decision.dt_valid) {
    valid_source_dts_.push_back(source_dt);
    while (valid_source_dts_.size() >
           static_cast<std::size_t>(config_.dt_median_window)) {
      valid_source_dts_.pop_front();
    }
    const Vec7 source_velocity = delta / source_dt;
    const Vec7 previous_estimated_velocity = estimated_qdot_;
    for (int joint = 0; joint < kArmDof; ++joint) {
      const bool stationary =
          std::abs(source_velocity[joint]) <=
          config_.source_stationary_velocity_rad_s;
      const bool reversing =
          !stationary &&
          source_velocity[joint] * previous_estimated_velocity[joint] < 0.0;
      if (stationary) {
        estimated_qdot_[joint] *=
            std::sqrt(config_.velocity_stationary_decay);
        feedforward_confidence_[joint] =
            config_.velocity_stationary_decay;
      } else if (reversing) {
        estimated_qdot_[joint] *=
            std::sqrt(config_.velocity_reversal_decay);
        feedforward_confidence_[joint] = config_.velocity_reversal_decay;
      } else {
        feedforward_confidence_[joint] = 1.0;
      }
    }
    const Vec7 predicted = estimated_q_ + estimated_qdot_ * source_dt;
    const Vec7 residual = continuous_target_ - predicted;
    estimated_q_ = predicted + config_.alpha * residual;
    estimated_qdot_ += config_.beta * residual / source_dt;
    for (int joint = 0; joint < kArmDof; ++joint) {
      const bool stationary =
          std::abs(source_velocity[joint]) <=
          config_.source_stationary_velocity_rad_s;
      const bool reversing =
          !stationary &&
          source_velocity[joint] * previous_estimated_velocity[joint] < 0.0;
      if (stationary) {
        estimated_qdot_[joint] *=
            std::sqrt(config_.velocity_stationary_decay);
      } else if (reversing) {
        estimated_qdot_[joint] *=
            std::sqrt(config_.velocity_reversal_decay);
      }
    }
    estimated_qdot_ = estimated_qdot_.cwiseMax(
        -config_.reference_velocity_scale * limits_.velocity);
    estimated_qdot_ = estimated_qdot_.cwiseMin(
        config_.reference_velocity_scale * limits_.velocity);
    decision.detail = decision.jump_rejected
                          ? "feedforward_joint_jump_rate_limited"
                          : "feedforward_target_accepted";
  } else {
    estimated_q_ += config_.alpha * (continuous_target_ - estimated_q_);
    decision.detail = decision.jump_rejected
                          ? "feedforward_joint_jump_rate_limited"
                          : "feedforward_source_dt_rejected";
  }

  result_.state = SparkFeedforwardState::kActive;
  decision.accepted = true;
  decision.median_dt_seconds = medianSourceDt();
  decision.continuous_target = continuous_target_;
  decision.estimated_velocity = estimated_qdot_;
  decision.feedforward_confidence = feedforward_confidence_;
  return decision;
}

double SparkFeedforwardReference7::smoothstep5(double value) noexcept {
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

SparkFeedforwardReferenceResult SparkFeedforwardReference7::step(
    const ArmMotionState& model_state, double dt, bool target_live) noexcept {
  if (!result_.valid || !validVector(model_state.q) || !std::isfinite(dt) ||
      dt <= 0.0) {
    result_.valid = false;
    result_.detail = "feedforward_invalid_step";
    return result_;
  }

  if (target_live && source_initialized_) {
    result_.state = SparkFeedforwardState::kActive;
    activation_phase_ = std::min(
        1.0, activation_phase_ + dt / config_.attack_seconds);
  } else if (result_.state != SparkFeedforwardState::kHold) {
    result_.state = SparkFeedforwardState::kStopping;
    activation_phase_ = std::max(
        0.0, activation_phase_ - dt / config_.release_seconds);
  }

  const double tracking_activation = result_.activation;
  Vec7 desired_velocity = Vec7::Zero();
  Vec7 target_position_error = Vec7::Zero();
  if (result_.state == SparkFeedforwardState::kActive) {
    // A critically damped second-order tracker: the target velocity combines
    // source feedforward with half the natural-frequency position term, then
    // the acceleration loop contributes the matching 2*w damping below.
    // Unlike pure source-velocity integration this converges to a static q_ik.
    target_position_error = estimated_q_ - result_.q;
    const Vec7 braking_acceleration =
        config_.target_braking_acceleration_scale * maximum_acceleration_;
    for (int joint = 0; joint < kArmDof; ++joint) {
      const double braking_speed = std::sqrt(
          2.0 * braking_acceleration[joint] *
          std::abs(target_position_error[joint]));
      const double position_correction = std::clamp(
          0.5 * config_.reference_position_gain *
              target_position_error[joint],
          -braking_speed, braking_speed);
      desired_velocity[joint] =
          estimated_qdot_[joint] + position_correction;
    }
  }
  const Vec7 velocity_limit =
      config_.reference_velocity_scale * limits_.velocity;
  for (int joint = 0; joint < kArmDof; ++joint) {
    const double positive_acceleration = std::max(0.0, result_.qddot[joint]);
    const double negative_acceleration = std::max(0.0, -result_.qddot[joint]);
    const double safe_upper =
        velocity_limit[joint] -
        positive_acceleration * positive_acceleration /
            (2.0 * maximum_jerk_[joint]) -
        0.5 * positive_acceleration * dt;
    const double safe_lower =
        -velocity_limit[joint] +
        negative_acceleration * negative_acceleration /
            (2.0 * maximum_jerk_[joint]) +
        0.5 * negative_acceleration * dt;
    desired_velocity[joint] =
        std::clamp(desired_velocity[joint], safe_lower, safe_upper);
  }
  Vec7 requested_acceleration;
  if (result_.state == SparkFeedforwardState::kActive) {
    requested_acceleration =
        tracking_activation * 2.0 *
        config_.reference_position_gain *
        (desired_velocity - result_.qdot);
  } else {
    requested_acceleration =
        (desired_velocity - result_.qdot) /
        std::max(nominal_dt_, config_.stale_velocity_decay_seconds);
  }

  const JointVelocityBounds bounds = computeJointVelocityBounds(
      result_.q, result_.qdot, result_.qddot, limits_, joint_limits_, dt);
  if (!bounds.lower.allFinite() || !bounds.upper.allFinite() ||
      (bounds.lower.array() > bounds.upper.array()).any()) {
    result_.valid = false;
    result_.detail = "feedforward_infeasible_joint_bounds";
    return result_;
  }
  const Vec7 previous_acceleration = result_.qddot;
  const Vec7 previous_velocity = result_.qdot;
  result_.qdot = (result_.qdot + requested_acceleration * dt)
                     .cwiseMax(bounds.lower)
                     .cwiseMin(bounds.upper);
  result_.qddot = (result_.qdot - previous_velocity) / dt;
  result_.jerk = (result_.qddot - previous_acceleration) / dt;
  result_.q += result_.qdot * dt;
  result_.activation = smoothstep5(activation_phase_);

  if (result_.state == SparkFeedforwardState::kStopping &&
      result_.qdot.cwiseAbs().maxCoeff() < 1.0e-7 &&
      result_.qddot.cwiseAbs().maxCoeff() < 1.0e-5 &&
      activation_phase_ <= 0.0) {
    result_.qdot.setZero();
    result_.qddot.setZero();
    result_.jerk.setZero();
    result_.state = SparkFeedforwardState::kHold;
  }

  result_.valid = result_.q.allFinite() && result_.qdot.allFinite() &&
                  result_.qddot.allFinite() && result_.jerk.allFinite();
  result_.detail = result_.state == SparkFeedforwardState::kActive
                       ? "feedforward_active"
                   : result_.state == SparkFeedforwardState::kStopping
                       ? "feedforward_stopping"
                       : "feedforward_hold";
  return result_;
}

std::string_view toString(SparkFeedforwardState state) noexcept {
  switch (state) {
    case SparkFeedforwardState::kHold:
      return "hold";
    case SparkFeedforwardState::kActive:
      return "active";
    case SparkFeedforwardState::kStopping:
      return "stopping";
  }
  return "hold";
}

}  // namespace tianji_qp_ik
