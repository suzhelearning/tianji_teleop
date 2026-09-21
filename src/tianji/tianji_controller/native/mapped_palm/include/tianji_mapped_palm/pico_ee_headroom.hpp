#pragma once

#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/types.hpp"

#include <string_view>

namespace tianji_mapped_palm {

enum class PicoEeHeadroomSource {
  kNone,
  kVelocity,
  kAcceleration,
  kJerk,
  kTaskScaling,
  kInvalid,
};

struct PicoEeHeadroomFeedback {
  bool accepted{false};
  Vec7 qdot{Vec7::Zero()};
  Vec7 qddot{Vec7::Zero()};
  double task_scale_position{1.0};
  double task_scale_orientation{1.0};
};

struct PicoEeHeadroomResult {
  bool valid{false};
  bool derivative_history_valid{false};
  double velocity_headroom{0.0};
  double acceleration_headroom{0.0};
  double jerk_headroom{0.0};
  double task_headroom{0.0};
  double raw_headroom{0.0};
  double filtered_headroom{0.0};
  double scale{0.0};
  PicoEeHeadroomSource dominant_source{PicoEeHeadroomSource::kNone};
};

struct PicoEeFeedforwardAllocation {
  bool valid{false};
  Vec6 twist{Vec6::Zero()};
  double low_frequency_scale{0.0};
  double high_frequency_scale{0.0};
};

PicoEeFeedforwardAllocation allocatePicoEeFeedforward(
    const Vec6& low_frequency_twist, const Vec6& high_frequency_twist,
    double headroom_scale, double low_frequency_min_scale,
    double high_frequency_min_scale) noexcept;

double picoEeRedundancyAuthority(const PicoEeHeadroomConfig& config,
                                 const PicoEeHeadroomResult& headroom) noexcept;

// Generic headroom calculation for the mapped PICO EE path.  The existing
// Spark governor remains untouched; this class shares only the configuration
// semantics and never owns Spark guidance state.
class PicoEeHeadroomGovernor {
 public:
  PicoEeHeadroomGovernor(SparkHeadroomFeedforwardVelocityQpConfig config,
                         ArmLimits limits,
                         JointAccelerationLimitConfig dynamic_limits);

  void reset() noexcept;
  PicoEeHeadroomResult update(const PicoEeHeadroomFeedback& feedback,
                              double dt) noexcept;
  const PicoEeHeadroomResult& state() const noexcept { return result_; }

 private:
  static double smoothstep5(double value) noexcept;
  static double normalizedTaskHeadroom(double scale, double minimum) noexcept;

  SparkHeadroomFeedforwardVelocityQpConfig config_;
  ArmLimits limits_;
  JointAccelerationLimitConfig dynamic_limits_;
  bool previous_acceleration_valid_{false};
  Vec7 previous_acceleration_{Vec7::Zero()};
  double filtered_jerk_usage_{0.0};
  PicoEeHeadroomResult result_;
};

std::string_view toString(PicoEeHeadroomSource source) noexcept;

}  // namespace tianji_mapped_palm
