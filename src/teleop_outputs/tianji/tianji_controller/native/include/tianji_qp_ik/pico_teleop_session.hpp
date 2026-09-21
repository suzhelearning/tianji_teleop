#pragma once

#include "tianji_qp_ik/pico_teleop_protocol.hpp"

#include <cstdint>

namespace tianji_qp_ik {

double monotonicTimestampSeconds(std::int64_t timestamp_ns) noexcept;
double selectTargetTimeSeconds(bool use_monotonic_clock,
                               double fixed_control_time_seconds,
                               std::int64_t monotonic_now_ns) noexcept;

enum class PicoTeleopAction {
  kIgnoreDisabled,
  kIgnoreStale,
  kIgnoreAlreadyApplied,
  kApply,
  kResetEpochAndApply,
};

struct PicoTeleopClassification {
  PicoTeleopAction action{PicoTeleopAction::kIgnoreDisabled};
  double frame_age_seconds{0.0};
};

struct PicoTeleopFreshness {
  bool has_applied_frame{false};
  bool live{false};
  bool stale{false};
  double frame_age_seconds{0.0};
};

enum class PicoTeleopButtonAction {
  kNone,
  kPause,
  kResume,
};

class PicoTeleopSession {
 public:
  explicit PicoTeleopSession(double timeout_seconds);
  void setEnabled(bool enabled) noexcept;
  PicoTeleopButtonAction observeButton(const PicoTeleopFrame& frame) noexcept;
  bool enabled() const noexcept { return enabled_; }
  PicoTeleopClassification classify(const PicoTeleopFrame& frame,
                                     std::int64_t now_monotonic_ns) const;
  void commitApplied(const PicoTeleopFrame& frame) noexcept;
  PicoTeleopFreshness freshness(
      std::int64_t now_monotonic_ns) const noexcept;

 private:
  double ageSeconds(std::int64_t receive_monotonic_ns,
                    std::int64_t now_monotonic_ns) const noexcept;

  double timeout_seconds_{0.0};
  bool enabled_{false};
  bool awaiting_frame_after_enable_{false};
  bool has_applied_frame_{false};
  std::uint64_t applied_epoch_{0U};
  std::uint64_t applied_sequence_{0U};
  std::uint64_t applied_resynchronization_generation_{0U};
  std::int64_t applied_receive_monotonic_ns_{0};
  bool button_state_initialized_{false};
  bool previous_button_state_{false};
  std::uint64_t button_tracking_epoch_{0U};
  std::uint64_t button_resynchronization_generation_{0U};
};

}  // namespace tianji_qp_ik
