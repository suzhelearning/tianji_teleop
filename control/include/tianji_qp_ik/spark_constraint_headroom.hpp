#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/types.hpp"

#include <string_view>

namespace tianji_qp_ik {

enum class SparkHeadroomSource {
  kNone,
  kVelocity,
  kAcceleration,
  kJerk,
  kTaskScaling,
  kInvalid
};

enum class SparkHeadroomState {
  kInactive,
  kRecovering,
  kTracking,
  kReducing,
  kInvalid
};

struct SparkConstraintHeadroomFeedback {
  bool accepted{false};
  Vec7 qdot{Vec7::Zero()};
  Vec7 qddot{Vec7::Zero()};
  double task_scale_position{1.0};
  double task_scale_orientation{1.0};
};

struct SparkConstraintHeadroomResult {
  bool valid{false};
  bool derivative_history_valid{false};
  double velocity_headroom{0.0};
  double acceleration_headroom{0.0};
  double jerk_headroom{0.0};
  double task_headroom{0.0};
  double raw_headroom{0.0};
  double filtered_headroom{0.0};
  double scale{0.0};
  SparkHeadroomSource dominant_source{SparkHeadroomSource::kNone};
  SparkHeadroomState state{SparkHeadroomState::kInactive};
};

class SparkConstraintHeadroomGovernor {
 public:
  SparkConstraintHeadroomGovernor(
      SparkHeadroomFeedforwardVelocityQpConfig config, ArmLimits limits,
      JointAccelerationLimitConfig dynamic_limits);

  void reset() noexcept;
  SparkConstraintHeadroomResult update(
      const SparkConstraintHeadroomFeedback& feedback, double dt) noexcept;
  const SparkConstraintHeadroomResult& state() const noexcept {
    return result_;
  }

 private:
  static double smoothstep5(double value) noexcept;
  static double normalizedTaskHeadroom(double scale,
                                       double minimum) noexcept;

  SparkHeadroomFeedforwardVelocityQpConfig config_;
  ArmLimits limits_;
  JointAccelerationLimitConfig dynamic_limits_;
  bool previous_acceleration_valid_{false};
  Vec7 previous_acceleration_{Vec7::Zero()};
  double filtered_jerk_usage_{0.0};
  SparkConstraintHeadroomResult result_;
};

std::string_view toString(SparkHeadroomSource source) noexcept;
std::string_view toString(SparkHeadroomState state) noexcept;

}  // namespace tianji_qp_ik
