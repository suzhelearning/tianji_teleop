#include "tianji_mapped_palm/pico_ee_reference.hpp"

#include "tianji_mapped_palm/so3.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_mapped_palm {
namespace {

bool finitePose(const Pose& pose) noexcept {
  return pose.position.allFinite() && isProperRotation(pose.rotation);
}

SparkPalmTwistEstimatorConfig makeTwistConfig(
    const SparkFeedforwardVelocityQpConfig& feedforward_config,
    const CartesianServoConfig& servo_config) noexcept {
  SparkPalmTwistEstimatorConfig result;
  result.filter_alpha = feedforward_config.palm_twist_filter_alpha;
  result.dt_median_window = feedforward_config.dt_median_window;
  result.dt_min_ratio = feedforward_config.dt_min_ratio;
  result.dt_max_ratio = feedforward_config.dt_max_ratio;
  result.stationary_decay =
      feedforward_config.velocity_stationary_decay;
  result.reversal_decay = feedforward_config.velocity_reversal_decay;
  result.maximum_linear_velocity_m_s = servo_config.max_linear_velocity;
  result.maximum_angular_velocity_rad_s = servo_config.max_angular_velocity;
  result.lowpass_cutoff_hz =
      feedforward_config.palm_twist_lowpass_cutoff_hz;
  return result;
}

SparkPalmTwistEstimatorConfig legacyTwistConfig(double cutoff_hz) {
  if (!std::isfinite(cutoff_hz) || cutoff_hz <= 0.0) {
    throw std::invalid_argument(
        "PICO EE reference cutoff must be finite and positive");
  }
  SparkPalmTwistEstimatorConfig result;
  result.filter_alpha = 1.0;
  result.lowpass_cutoff_hz = cutoff_hz;
  return result;
}

void limitDecomposition(double maximum_norm, Eigen::Vector3d& low_frequency,
                        Eigen::Vector3d& high_frequency) noexcept {
  const double norm = (low_frequency + high_frequency).norm();
  if (std::isfinite(maximum_norm) && maximum_norm > 0.0 &&
      norm > maximum_norm) {
    const double scale = maximum_norm / norm;
    low_frequency *= scale;
    high_frequency *= scale;
  }
}

void limitFeedforward(const SparkPalmTwistEstimatorConfig& config,
                      Vec6& low_frequency, Vec6& high_frequency) noexcept {
  Eigen::Vector3d low_linear = low_frequency.head<3>();
  Eigen::Vector3d high_linear = high_frequency.head<3>();
  Eigen::Vector3d low_angular = low_frequency.tail<3>();
  Eigen::Vector3d high_angular = high_frequency.tail<3>();
  limitDecomposition(config.maximum_linear_velocity_m_s, low_linear,
                     high_linear);
  limitDecomposition(config.maximum_angular_velocity_rad_s, low_angular,
                     high_angular);
  low_frequency.head<3>() = low_linear;
  low_frequency.tail<3>() = low_angular;
  high_frequency.head<3>() = high_linear;
  high_frequency.tail<3>() = high_angular;
}

void applyFeedforwardGains(
    double position_gain, double orientation_gain,
    double position_high_frequency_gain,
    double orientation_high_frequency_gain, const Vec6& unscaled_low,
    const Vec6& unscaled_high, Vec6& scaled_low,
    Vec6& scaled_high) noexcept {
  scaled_low.head<3>() = position_gain * unscaled_low.head<3>();
  scaled_low.tail<3>() = orientation_gain * unscaled_low.tail<3>();
  scaled_high.head<3>() =
      (position_gain + position_high_frequency_gain) *
      unscaled_high.head<3>();
  scaled_high.tail<3>() =
      (orientation_gain + orientation_high_frequency_gain) *
      unscaled_high.tail<3>();
}

}  // namespace

PicoEeReferenceGenerator::PicoEeReferenceGenerator(double cutoff_hz)
    : twist_config_(legacyTwistConfig(cutoff_hz)),
      twist_estimator_(twist_config_),
      causal_config_(),
      causal_estimator_(causal_config_),
      cutoff_hz_(cutoff_hz) {}

PicoEeReferenceGenerator::PicoEeReferenceGenerator(
    const SparkFeedforwardVelocityQpConfig& feedforward_config,
    const CartesianServoConfig& servo_config)
    : PicoEeReferenceGenerator(feedforward_config, servo_config,
                               CausalSe3TwistEstimatorConfig{}) {}

PicoEeReferenceGenerator::PicoEeReferenceGenerator(
    const SparkFeedforwardVelocityQpConfig& feedforward_config,
    const CartesianServoConfig& servo_config,
    const CausalSe3TwistEstimatorConfig& estimator_config)
    : twist_config_(makeTwistConfig(feedforward_config, servo_config)),
      twist_estimator_(twist_config_),
      causal_config_(estimator_config),
      causal_estimator_(causal_config_),
      cutoff_hz_(feedforward_config.palm_twist_lowpass_cutoff_hz),
      position_feedforward_gain_(
          feedforward_config.position_feedforward_gain),
      orientation_feedforward_gain_(
          feedforward_config.orientation_feedforward_gain),
      position_high_frequency_feedforward_gain_(
          feedforward_config.position_high_frequency_feedforward_gain),
      orientation_high_frequency_feedforward_gain_(
          feedforward_config.orientation_high_frequency_feedforward_gain) {}

void PicoEeReferenceGenerator::reconfigure(double cutoff_hz) noexcept {
  if (!std::isfinite(cutoff_hz) || cutoff_hz <= 0.0) {
    return;
  }
  twist_config_.lowpass_cutoff_hz = cutoff_hz;
  cutoff_hz_ = cutoff_hz;
  twist_estimator_ = SparkPalmTwistEstimator(twist_config_);
  causal_estimator_.reset();
  causal_lowpass_twist_.setZero();
  causal_previous_timestamp_ns_ = 0;
  initialized_ = false;
  previous_epoch_ = 0U;
}

void PicoEeReferenceGenerator::reset() noexcept {
  twist_estimator_.reset();
  causal_estimator_.reset();
  causal_lowpass_twist_.setZero();
  causal_previous_timestamp_ns_ = 0;
  initialized_ = false;
  previous_epoch_ = 0U;
}

PicoEeReferenceSample PicoEeReferenceGenerator::update(
    const Pose& mapped_pose, std::uint64_t sequence,
    std::uint64_t tracking_epoch, std::int64_t source_timestamp_ns,
    bool stream_discontinuity, double dt_seconds,
    double observation_age_s) noexcept {
  PicoEeReferenceSample result;
  result.estimator_mode = causal_config_.mode;
  result.pose = mapped_pose;
  result.pose_valid = finitePose(mapped_pose);
  if (!result.pose_valid || !std::isfinite(dt_seconds) ||
      dt_seconds <= 0.0 || sequence == 0U || tracking_epoch == 0U ||
      source_timestamp_ns <= 0) {
    result.stale = true;
    return result;
  }


  CausalSe3TwistInput causal_input;
  causal_input.pose = mapped_pose;
  causal_input.sequence = sequence;
  causal_input.tracking_epoch = tracking_epoch;
  causal_input.source_timestamp_ns = source_timestamp_ns;
  causal_input.stream_discontinuity = stream_discontinuity;
  causal_input.observation_age_s = observation_age_s;
  causal_input.control_dt_s = dt_seconds;
  result.causal = causal_estimator_.update(causal_input);

  if (result.causal.valid) {
    double source_dt_s = dt_seconds;
    if (causal_previous_timestamp_ns_ > 0 &&
        source_timestamp_ns > causal_previous_timestamp_ns_) {
      source_dt_s = static_cast<double>(source_timestamp_ns -
                                        causal_previous_timestamp_ns_) *
                    1.0e-9;
    }
    const double alpha =
        1.0 - std::exp(-2.0 * 3.14159265358979323846 * cutoff_hz_ *
                       source_dt_s);
    causal_lowpass_twist_ +=
        std::clamp(alpha, 0.0, 1.0) *
        (result.causal.aligned_twist - causal_lowpass_twist_);
    const Vec6 causal_high =
        result.causal.aligned_twist - causal_lowpass_twist_;
    applyFeedforwardGains(
        position_feedforward_gain_, orientation_feedforward_gain_,
        position_high_frequency_feedforward_gain_,
        orientation_high_frequency_feedforward_gain_, causal_lowpass_twist_,
        causal_high, result.causal_low_frequency_feedforward_twist,
        result.causal_high_frequency_feedforward_twist);
    limitFeedforward(twist_config_,
                     result.causal_low_frequency_feedforward_twist,
                     result.causal_high_frequency_feedforward_twist);
  } else if (result.causal.failure == CausalSe3TwistFailure::kEpochReset ||
             result.causal.failure ==
                 CausalSe3TwistFailure::kStreamDiscontinuity ||
             result.causal.failure ==
                 CausalSe3TwistFailure::kNonMonotonicSequence ||
             result.causal.failure ==
                 CausalSe3TwistFailure::kNonMonotonicTimestamp ||
             result.causal.failure ==
                 CausalSe3TwistFailure::kSourceDtOutlier) {
    causal_lowpass_twist_.setZero();
  }
  causal_previous_timestamp_ns_ = source_timestamp_ns;

  const bool was_initialized = initialized_;
  const bool epoch_changed = was_initialized && previous_epoch_ != tracking_epoch;
  const SparkPalmTwistDecision decision = twist_estimator_.update(
      mapped_pose, sequence, source_timestamp_ns, tracking_epoch,
      stream_discontinuity);
  if (!decision.accepted) {
    result.stale = true;
    return result;
  }

  initialized_ = true;
  previous_epoch_ = tracking_epoch;
  result.stale = decision.reset &&
                 (stream_discontinuity || epoch_changed);
  applyFeedforwardGains(
      position_feedforward_gain_, orientation_feedforward_gain_,
      position_high_frequency_feedforward_gain_,
      orientation_high_frequency_feedforward_gain_,
      decision.low_frequency_twist, decision.high_frequency_twist,
      result.legacy_low_frequency_feedforward_twist,
      result.legacy_high_frequency_feedforward_twist);
  limitFeedforward(twist_config_,
                   result.legacy_low_frequency_feedforward_twist,
                   result.legacy_high_frequency_feedforward_twist);

  result.low_frequency_feedforward_twist =
      result.legacy_low_frequency_feedforward_twist;
  result.high_frequency_feedforward_twist =
      result.legacy_high_frequency_feedforward_twist;
  if (causal_config_.mode == PicoEeTwistEstimatorMode::kActive &&
      result.causal.valid) {
    const double confidence = std::clamp(result.causal.confidence, 0.0, 1.0);
    result.low_frequency_feedforward_twist =
        (1.0 - confidence) *
            result.legacy_low_frequency_feedforward_twist +
        confidence * result.causal_low_frequency_feedforward_twist;
    result.high_frequency_feedforward_twist =
        (1.0 - confidence) *
            result.legacy_high_frequency_feedforward_twist +
        confidence * result.causal_high_frequency_feedforward_twist;
    result.causal_selected = confidence > 0.0;
  }
  limitFeedforward(twist_config_, result.low_frequency_feedforward_twist,
                   result.high_frequency_feedforward_twist);
  result.twist = result.low_frequency_feedforward_twist +
                 result.high_frequency_feedforward_twist;
  result.valid = result.twist.allFinite();
  if (!result.valid) {
    result.low_frequency_feedforward_twist.setZero();
    result.high_frequency_feedforward_twist.setZero();
    result.twist.setZero();
    result.stale = true;
  }
  return result;
}

}  // namespace tianji_mapped_palm
