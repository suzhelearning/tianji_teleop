#include "tianji_mapped_palm/pico_ee_motion_confidence.hpp"

#include <algorithm>
#include <cmath>

namespace tianji_mapped_palm {
namespace {

double smoothstepRange(double value, double low, double high) noexcept {
  const double phase = std::clamp((value - low) / (high - low), 0.0, 1.0);
  return phase * phase * (3.0 - 2.0 * phase);
}

}  // namespace

PicoEeMotionConfidence::PicoEeMotionConfidence(
    PicoEeMotionConfidenceConfig config)
    : config_(std::move(config)) {}

PicoEeMotionEvidence PicoEeMotionConfidence::update(
    const PicoEeMotionEvidenceInput& input) noexcept {
  PicoEeMotionEvidence result;
  if (!config_.enabled) {
    result.reason = MotionConfidenceReason::kDisabled;
    return result;
  }
  if (input.reset) {
    reset();
    result.reason = MotionConfidenceReason::kReset;
    return result;
  }
  if (input.stale) {
    reset();
    result.reason = MotionConfidenceReason::kStale;
    return result;
  }
  if (!input.low_frequency_twist.allFinite() ||
      !std::isfinite(input.freshness) ||
      !std::isfinite(input.estimator_confidence) ||
      !std::isfinite(input.dt) || input.dt <= 0.0) {
    reset();
    result.reason = MotionConfidenceReason::kInvalidInput;
    return result;
  }

  const double trust = std::clamp(input.freshness, 0.0, 1.0) *
                       std::clamp(input.estimator_confidence, 0.0, 1.0);
  result.linear_raw =
      smoothstepRange(input.low_frequency_twist.head<3>().norm(),
                      config_.linear_quiet_m_s,
                      config_.linear_tracking_m_s) * trust;
  result.angular_raw =
      smoothstepRange(input.low_frequency_twist.tail<3>().norm(),
                      config_.angular_quiet_rad_s,
                      config_.angular_tracking_rad_s) * trust;
  const auto filter = [this, &input](double raw, double& filtered) {
    const double time_constant = raw > filtered
                                     ? config_.attack_seconds
                                     : config_.release_seconds;
    const double alpha = 1.0 - std::exp(-input.dt / time_constant);
    filtered += alpha * (raw - filtered);
    filtered = std::clamp(filtered, 0.0, 1.0);
  };
  filter(result.linear_raw, linear_filtered_);
  filter(result.angular_raw, angular_filtered_);
  result.raw = std::max(result.linear_raw, result.angular_raw);
  result.linear_filtered = linear_filtered_;
  result.angular_filtered = angular_filtered_;
  result.filtered = std::max(linear_filtered_, angular_filtered_);
  result.valid = true;
  result.reason = MotionConfidenceReason::kNone;
  return result;
}

void PicoEeMotionConfidence::reset() noexcept {
  linear_filtered_ = 0.0;
  angular_filtered_ = 0.0;
}

std::string_view toString(MotionConfidenceReason reason) noexcept {
  switch (reason) {
    case MotionConfidenceReason::kNone: return "none";
    case MotionConfidenceReason::kDisabled: return "disabled";
    case MotionConfidenceReason::kInvalidInput: return "invalid_input";
    case MotionConfidenceReason::kStale: return "stale";
    case MotionConfidenceReason::kReset: return "reset";
  }
  return "unknown";
}

}  // namespace tianji_mapped_palm
