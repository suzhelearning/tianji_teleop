#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/types.hpp"

#include <ruckig/ruckig.hpp>

#include <string_view>

namespace tianji_qp_ik {

struct JointTrajectoryResult {
  bool accepted{false};
  ArmMotionState state;
  Vec7 jerk{Vec7::Zero()};
  double velocity_ratio{0.0};
  double acceleration_ratio{0.0};
  double jerk_ratio{0.0};
  std::string_view detail{"not_updated"};
};

class JointTrajectoryLimiter7 {
 public:
  JointTrajectoryLimiter7(DlsPostureRuckigConfig config, ArmLimits limits,
                          double initial_dt);

  bool reset(const ArmMotionState& state) noexcept;
  JointTrajectoryResult update(const Vec7& target, double dt);
  const ArmMotionState& state() const noexcept { return state_; }

 private:
  bool validState(const ArmMotionState& state) const noexcept;

  DlsPostureRuckigConfig config_;
  ArmLimits limits_;
  ruckig::Ruckig<kArmDof> otg_;
  ruckig::InputParameter<kArmDof> input_;
  ruckig::OutputParameter<kArmDof> output_;
  ArmMotionState state_;
  bool initialized_{false};
};

}  // namespace tianji_qp_ik
