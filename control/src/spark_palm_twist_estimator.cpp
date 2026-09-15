#include "tianji_qp_ik/spark_palm_twist_estimator.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace tianji_qp_ik {
namespace {

bool validPose(const Pose& pose) noexcept {
  return pose.position.allFinite() && isProperRotation(pose.rotation);
}

}  // namespace

SparkPalmTwistEstimator::SparkPalmTwistEstimator(
    SparkPalmTwistEstimatorConfig config)
    : config_(std::move(config)) {}

void SparkPalmTwistEstimator::reset() noexcept {
  initialized_ = false;
  previous_pose_ = {};
  previous_sequence_ = 0U;
  previous_timestamp_ns_ = 0;
  tracking_epoch_ = 0U;
  valid_source_dts_.clear();
  twist_.setZero();
  low_frequency_twist_.setZero();
}

double SparkPalmTwistEstimator::medianSourceDt() const noexcept {
  if (valid_source_dts_.empty()) {
    return 0.0;
  }
  std::vector<double> sorted(valid_source_dts_.begin(),
                             valid_source_dts_.end());
  const std::size_t middle = sorted.size() / 2U;
  std::nth_element(sorted.begin(),
                   sorted.begin() + static_cast<std::ptrdiff_t>(middle),
                   sorted.end());
  return sorted[middle];
}

void SparkPalmTwistEstimator::filterComponent(
    Eigen::Vector3d raw, double stationary_threshold, double filter_alpha,
    double stationary_decay, double reversal_decay,
    Eigen::Vector3d& filtered) noexcept {
  const double raw_norm = raw.norm();
  if (raw_norm <= stationary_threshold) {
    filtered *= stationary_decay;
    return;
  }

  double alpha = filter_alpha;
  if (filtered.squaredNorm() > 1.0e-18 && filtered.dot(raw) < 0.0) {
    filtered *= reversal_decay;
    alpha *= std::sqrt(reversal_decay);
  }
  filtered += alpha * (raw - filtered);
}

void SparkPalmTwistEstimator::limitNorm(double maximum_norm,
                                        Eigen::Vector3d& value) noexcept {
  const double norm = value.norm();
  if (std::isfinite(maximum_norm) && maximum_norm > 0.0 &&
      norm > maximum_norm) {
    value *= maximum_norm / norm;
  }
}

SparkPalmTwistDecision SparkPalmTwistEstimator::update(
    const Pose& pose, std::uint64_t sequence,
    std::int64_t source_timestamp_ns, std::uint64_t tracking_epoch,
    bool stream_discontinuity) noexcept {
  SparkPalmTwistDecision decision;
  decision.twist = twist_;
  decision.low_frequency_twist = low_frequency_twist_;
  decision.high_frequency_twist = highFrequencyTwist();
  if (!validPose(pose) || sequence == 0U || source_timestamp_ns <= 0) {
    decision.detail = "palm_twist_invalid_source";
    return decision;
  }

  const bool epoch_changed = initialized_ && tracking_epoch_ != tracking_epoch;
  if (!initialized_ || epoch_changed || stream_discontinuity) {
    initialized_ = true;
    previous_pose_ = pose;
    previous_sequence_ = sequence;
    previous_timestamp_ns_ = source_timestamp_ns;
    tracking_epoch_ = tracking_epoch;
    valid_source_dts_.clear();
    twist_.setZero();
    low_frequency_twist_.setZero();
    decision.accepted = true;
    decision.reset = true;
    decision.twist = twist_;
    decision.low_frequency_twist = low_frequency_twist_;
    decision.high_frequency_twist = highFrequencyTwist();
    decision.detail = epoch_changed || stream_discontinuity
                          ? "palm_twist_resynchronized"
                          : "palm_twist_started";
    return decision;
  }

  if (sequence <= previous_sequence_ ||
      source_timestamp_ns <= previous_timestamp_ns_) {
    decision.detail = "palm_twist_duplicate_source";
    return decision;
  }

  const double source_dt =
      static_cast<double>(source_timestamp_ns - previous_timestamp_ns_) *
      1.0e-9;
  const double median_dt = medianSourceDt();
  const bool enough_timing_history = valid_source_dts_.size() >= 3U;
  const bool dt_in_range =
      std::isfinite(source_dt) && source_dt > 0.0 &&
      (!enough_timing_history ||
       (source_dt >= config_.dt_min_ratio * median_dt &&
        source_dt <= config_.dt_max_ratio * median_dt));

  decision.accepted = true;
  decision.source_dt_seconds = source_dt;
  decision.median_dt_seconds = median_dt;
  decision.dt_valid = dt_in_range;

  const Pose baseline = previous_pose_;
  previous_pose_ = pose;
  previous_sequence_ = sequence;
  previous_timestamp_ns_ = source_timestamp_ns;
  tracking_epoch_ = tracking_epoch;

  if (!dt_in_range) {
    decision.twist = twist_;
    decision.low_frequency_twist = low_frequency_twist_;
    decision.high_frequency_twist = highFrequencyTwist();
    decision.detail = "palm_twist_source_dt_rejected";
    return decision;
  }

  valid_source_dts_.push_back(source_dt);
  while (valid_source_dts_.size() >
         static_cast<std::size_t>(config_.dt_median_window)) {
    valid_source_dts_.pop_front();
  }

  const Vec6 raw_twist = poseErrorWorld(pose, baseline) / source_dt;
  Eigen::Vector3d linear = twist_.head<3>();
  Eigen::Vector3d angular = twist_.tail<3>();
  filterComponent(raw_twist.head<3>(),
                  config_.linear_stationary_threshold_m_s,
                  config_.filter_alpha, config_.stationary_decay,
                  config_.reversal_decay, linear);
  filterComponent(raw_twist.tail<3>(),
                  config_.angular_stationary_threshold_rad_s,
                  config_.filter_alpha, config_.stationary_decay,
                  config_.reversal_decay, angular);
  limitNorm(config_.maximum_linear_velocity_m_s, linear);
  limitNorm(config_.maximum_angular_velocity_rad_s, angular);
  twist_.head<3>() = linear;
  twist_.tail<3>() = angular;
  const double lowpass_alpha = std::clamp(
      1.0 - std::exp(-2.0 * 3.14159265358979323846 *
                     config_.lowpass_cutoff_hz * source_dt),
      0.0, 1.0);
  low_frequency_twist_ +=
      lowpass_alpha * (twist_ - low_frequency_twist_);

  decision.median_dt_seconds = medianSourceDt();
  decision.twist = twist_;
  decision.low_frequency_twist = low_frequency_twist_;
  decision.high_frequency_twist = highFrequencyTwist();
  decision.detail = "palm_twist_accepted";
  return decision;
}

}  // namespace tianji_qp_ik
