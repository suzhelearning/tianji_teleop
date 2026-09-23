#pragma once
#include "tianji_qp_ik/franka_iterative_pose_dls.hpp"
#include "tianji_qp_ik/dls_ik_types.hpp"
namespace tianji_qp_ik {
class PicoEeFrankaDlsIk7 {
 public:
  PicoEeFrankaDlsIk7(IterativeDlsConfig dls_config,
                     PicoEeFrankaDlsConfig config,
                     double joint_margin_rad);
  PicoEeFrankaDlsResult solve(const PicoEeFrankaDlsInput& input);
  void reset(const ArmMotionState& state) noexcept;
  const Vec7& goal() const noexcept { return goal_; }
 private:
  bool initialize(const PicoEeFrankaDlsInput& input) noexcept;
  bool validInput(const PicoEeFrankaDlsInput& input) const noexcept;
  double effectiveVelocityLimit(const PicoEeFrankaDlsInput& input,
                                int joint) const noexcept;
  FrankaIterativePoseDlsIk7 dls_;
  PicoEeFrankaDlsConfig config_;
  double joint_margin_rad_{0.0};
  Vec7 safe_lower_{Vec7::Zero()};
  Vec7 safe_upper_{Vec7::Zero()};
  Vec7 goal_{Vec7::Zero()};
  ArmMotionState pending_reset_;
  bool pending_reset_valid_{false};
  bool initialized_{false};
};
}  // namespace tianji_qp_ik
