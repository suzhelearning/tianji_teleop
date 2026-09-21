#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/types.hpp"

#include <cstdint>
#include <deque>
#include <string_view>

namespace tianji_qp_ik {

enum class SparkFeedforwardState { kHold, kActive, kStopping };

struct SparkFeedforwardTargetDecision {
  bool accepted{false};
  bool dt_valid{false};
  bool jump_rejected{false};
  bool epoch_reset{false};
  double source_dt_seconds{0.0};
  double median_dt_seconds{0.0};
  Vec7 continuous_target{Vec7::Zero()};
  Vec7 estimated_velocity{Vec7::Zero()};
  Vec7 feedforward_confidence{Vec7::Zero()};
  std::string_view detail{"not_updated"};
};

struct SparkFeedforwardReferenceResult {
  bool valid{false};
  Vec7 q{Vec7::Zero()};
  Vec7 qdot{Vec7::Zero()};
  Vec7 qddot{Vec7::Zero()};
  Vec7 jerk{Vec7::Zero()};
  double activation{0.0};
  SparkFeedforwardState state{SparkFeedforwardState::kHold};
  std::string_view detail{"not_updated"};
};

class SparkFeedforwardReference7 {
 public:
  SparkFeedforwardReference7(
      SparkFeedforwardVelocityQpConfig config, ArmLimits limits,
      JointLimitConfig joint_limits, double initial_dt);

  void reset(const ArmMotionState& model_state,
             std::uint64_t tracking_epoch) noexcept;
  SparkFeedforwardTargetDecision acceptTarget(
      const Vec7& q_ik, std::uint64_t sequence,
      std::int64_t source_timestamp_ns, std::uint64_t tracking_epoch,
      const ArmMotionState& model_state) noexcept;
  SparkFeedforwardReferenceResult step(const ArmMotionState& model_state,
                                       double dt,
                                       bool target_live) noexcept;

  const SparkFeedforwardReferenceResult& state() const noexcept {
    return result_;
  }

 private:
  double medianSourceDt() const noexcept;
  static double smoothstep5(double value) noexcept;

  SparkFeedforwardVelocityQpConfig config_;
  ArmLimits limits_;
  JointLimitConfig joint_limits_;
  Vec7 maximum_acceleration_{Vec7::Zero()};
  Vec7 maximum_jerk_{Vec7::Zero()};
  double nominal_dt_{0.005};
  std::deque<double> valid_source_dts_;
  bool source_initialized_{false};
  std::uint64_t tracking_epoch_{0U};
  std::uint64_t last_sequence_{0U};
  std::int64_t last_source_timestamp_ns_{0};
  Vec7 previous_raw_target_{Vec7::Zero()};
  Vec7 raw_continuous_target_{Vec7::Zero()};
  Vec7 continuous_target_{Vec7::Zero()};
  Vec7 estimated_q_{Vec7::Zero()};
  Vec7 estimated_qdot_{Vec7::Zero()};
  Vec7 feedforward_confidence_{Vec7::Zero()};
  double activation_phase_{0.0};
  SparkFeedforwardReferenceResult result_;
};

std::string_view toString(SparkFeedforwardState state) noexcept;

}  // namespace tianji_qp_ik
