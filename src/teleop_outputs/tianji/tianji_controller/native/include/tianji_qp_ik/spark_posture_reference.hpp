#pragma once

#include "tianji_qp_ik/joint_trajectory_limiter.hpp"

#include <string_view>

namespace tianji_qp_ik {

struct SparkPostureReferenceResult {
  bool accepted{false};
  ArmMotionState state;
  Vec7 jerk{Vec7::Zero()};
  Vec7 posture_velocity{Vec7::Zero()};
  double velocity_ratio{0.0};
  double acceleration_ratio{0.0};
  double jerk_ratio{0.0};
  std::string_view detail{"not_updated"};
};

class SparkPostureReference7 {
 public:
  SparkPostureReference7(DlsPostureRuckigConfig config, ArmLimits limits,
                         double initial_dt, double posture_position_gain);

  bool reset(const ArmMotionState& model_state) noexcept;
  SparkPostureReferenceResult update(const Vec7& q_spark,
                                     const Vec7& q_model, double dt);
  SparkPostureReferenceResult hold(const Vec7& q_model, double dt);
  const ArmMotionState& state() const noexcept { return limiter_.state(); }

 private:
  SparkPostureReferenceResult advance(const Vec7& target,
                                      const Vec7& q_model, double dt);

  DlsPostureRuckigConfig config_;
  ArmLimits limits_;
  double posture_position_gain_{0.0};
  JointTrajectoryLimiter7 limiter_;
  bool initialized_{false};
};

}  // namespace tianji_qp_ik
