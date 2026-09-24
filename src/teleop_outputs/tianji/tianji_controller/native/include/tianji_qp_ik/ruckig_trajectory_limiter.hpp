#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/types.hpp"

#include <ruckig/ruckig.hpp>

#include <string_view>
#include <cmath>

namespace tianji_qp_ik {

struct SimulationSoftStartLimits {
  double velocity{.35}, acceleration{.5}, jerk{2.};
  bool valid() const noexcept {
    return std::isfinite(velocity) && velocity > 0 &&
           std::isfinite(acceleration) && acceleration > 0 &&
           std::isfinite(jerk) && jerk > 0;
  }
};

struct RuckigTrajectoryResult {
  bool accepted{false};
  ArmMotionState state;
  Vec7 jerk{Vec7::Zero()};
  double velocity_ratio{0.0};
  double acceleration_ratio{0.0};
  double jerk_ratio{0.0};
  std::string_view detail{"not_updated"};
};

class RuckigTrajectoryLimiter7 {
 public:
  RuckigTrajectoryLimiter7(DlsPostureRuckigConfig config, ArmLimits limits,
                          double initial_dt);

  bool canReset(const ArmMotionState& state) const noexcept;
  bool reset(const ArmMotionState& state) noexcept;
  bool beginSoftStart(SimulationSoftStartLimits limits = {}) noexcept;
  bool softStarting() const noexcept { return soft_start_; }
  RuckigTrajectoryResult update(const Vec7& target, double dt, bool allow_soft_start_ramp = true);
  const ArmMotionState& state() const noexcept { return state_; }
  const DlsPostureRuckigConfig& sampledLimits() const noexcept { return sampled_limits_; }
  // Relative real control must never project q/qdot independently of derivatives.
  void preserveConstraints() noexcept { preserve_constraints_ = true; }

 private:
  bool validState(const ArmMotionState& state) const noexcept;

  DlsPostureRuckigConfig config_;
  DlsPostureRuckigConfig nominal_config_;
  DlsPostureRuckigConfig sampled_limits_;
  bool soft_start_{false};
  SimulationSoftStartLimits soft_start_limits_;
  double close_seconds_{0.0}, ramp_seconds_{-1.0};
  ArmLimits limits_;
  ruckig::Ruckig<kArmDof> otg_;
  ruckig::InputParameter<kArmDof> input_;
  ruckig::OutputParameter<kArmDof> output_;
  ArmMotionState state_;
  bool initialized_{false};
  bool preserve_constraints_{false};
};

}  // namespace tianji_qp_ik
