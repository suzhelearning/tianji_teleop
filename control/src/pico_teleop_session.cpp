#include "tianji_qp_ik/pico_teleop_session.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {

double monotonicTimestampSeconds(std::int64_t timestamp_ns) noexcept {
  return timestamp_ns > 0
             ? static_cast<double>(timestamp_ns) * 1.0e-9
             : 0.0;
}

double selectTargetTimeSeconds(bool use_monotonic_clock,
                               double fixed_control_time_seconds,
                               std::int64_t monotonic_now_ns) noexcept {
  if (use_monotonic_clock && monotonic_now_ns > 0) {
    return monotonicTimestampSeconds(monotonic_now_ns);
  }
  return fixed_control_time_seconds;
}

PicoTeleopSession::PicoTeleopSession(double timeout_seconds)
    : timeout_seconds_(timeout_seconds) {
  if (!std::isfinite(timeout_seconds_) || timeout_seconds_ <= 0.0) {
    throw std::invalid_argument(
        "PICO teleop session timeout must be finite and positive");
  }
}

void PicoTeleopSession::setEnabled(bool enabled) noexcept {
  if (enabled == enabled_) {
    return;
  }
  if (enabled) {
    // Re-enabling PICO is an explicit operator takeover. Do not reuse the
    // previous stream reference, even when the tracker stayed in the same
    // epoch while teleoperation was disabled.
    has_applied_frame_ = false;
    applied_epoch_ = 0U;
    applied_sequence_ = 0U;
    applied_resynchronization_generation_ = 0U;
    applied_receive_monotonic_ns_ = 0;
  }
  enabled_ = enabled;
  awaiting_frame_after_enable_ = enabled;
}

PicoTeleopButtonAction PicoTeleopSession::observeButton(
    const PicoTeleopFrame& frame) noexcept {
  const bool stream_identity_changed =
      !button_state_initialized_ ||
      frame.tracking_epoch != button_tracking_epoch_ ||
      frame.resynchronization_generation !=
          button_resynchronization_generation_;
  button_tracking_epoch_ = frame.tracking_epoch;
  button_resynchronization_generation_ =
      frame.resynchronization_generation;
  if (stream_identity_changed) {
    button_state_initialized_ = true;
    previous_button_state_ = frame.user_button_pressed;
    return PicoTeleopButtonAction::kNone;
  }
  const bool changed = previous_button_state_ != frame.user_button_pressed;
  previous_button_state_ = frame.user_button_pressed;
  if (!changed) {
    return PicoTeleopButtonAction::kNone;
  }
  if (enabled_) {
    setEnabled(false);
    return PicoTeleopButtonAction::kPause;
  }
  setEnabled(true);
  return PicoTeleopButtonAction::kResume;
}

PicoTeleopClassification PicoTeleopSession::classify(
    const PicoTeleopFrame& frame, std::int64_t now_monotonic_ns) const {
  const double age_seconds =
      ageSeconds(frame.receive_monotonic_ns, now_monotonic_ns);
  if (!enabled_) {
    return {PicoTeleopAction::kIgnoreDisabled, age_seconds};
  }
  if (frame.receive_monotonic_ns <= 0 || age_seconds >= timeout_seconds_) {
    return {PicoTeleopAction::kIgnoreStale, age_seconds};
  }
  if (has_applied_frame_ &&
      (frame.tracking_epoch < applied_epoch_ ||
       (frame.tracking_epoch == applied_epoch_ &&
        frame.sequence <= applied_sequence_))) {
    return {PicoTeleopAction::kIgnoreAlreadyApplied, age_seconds};
  }

  const bool epoch_changed =
      !has_applied_frame_ || frame.tracking_epoch != applied_epoch_;
  const bool resynchronized =
      frame.stream_discontinuity ||
      frame.resynchronization_generation >
          applied_resynchronization_generation_;
  return {(epoch_changed || resynchronized)
              ? PicoTeleopAction::kResetEpochAndApply
              : PicoTeleopAction::kApply,
          age_seconds};
}

void PicoTeleopSession::commitApplied(
    const PicoTeleopFrame& frame) noexcept {
  has_applied_frame_ = true;
  applied_epoch_ = frame.tracking_epoch;
  applied_sequence_ = frame.sequence;
  applied_resynchronization_generation_ = std::max(
      applied_resynchronization_generation_,
      frame.resynchronization_generation);
  applied_receive_monotonic_ns_ = frame.receive_monotonic_ns;
  awaiting_frame_after_enable_ = false;
}

PicoTeleopFreshness PicoTeleopSession::freshness(
    std::int64_t now_monotonic_ns) const noexcept {
  PicoTeleopFreshness result;
  result.has_applied_frame = has_applied_frame_;
  if (has_applied_frame_) {
    result.frame_age_seconds =
        ageSeconds(applied_receive_monotonic_ns_, now_monotonic_ns);
  }
  if (!enabled_ || !has_applied_frame_) {
    return result;
  }
  result.stale = result.frame_age_seconds >= timeout_seconds_;
  result.live = !awaiting_frame_after_enable_ && !result.stale;
  return result;
}

double PicoTeleopSession::ageSeconds(
    std::int64_t receive_monotonic_ns,
    std::int64_t now_monotonic_ns) const noexcept {
  if (receive_monotonic_ns <= 0 || now_monotonic_ns <= 0) {
    return 0.0;
  }
  const double age = static_cast<double>(
      now_monotonic_ns - receive_monotonic_ns) * 1.0e-9;
  return std::max(0.0, age);
}

}  // namespace tianji_qp_ik
