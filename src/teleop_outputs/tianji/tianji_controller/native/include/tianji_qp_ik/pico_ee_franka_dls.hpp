#pragma once

#include "tianji_qp_ik/franka_iterative_pose_dls.hpp"
#include "tianji_qp_ik/ceres_ik_types.hpp"
#include "tianji_qp_ik/velocity_ik.hpp"

#include <array>
#include <string_view>

namespace tianji_qp_ik {

// Fourth-order position planner used by the Franka-style EE DLS route.  Each
// arm owns seven independent instances.  The state is position, velocity,
// acceleration, and jerk; the fourth-order error feedback computes a bounded
// snap-like update for the jerk state.
class RealTimeConstrainedPlanner {
 public:
  RealTimeConstrainedPlanner() noexcept = default;
  RealTimeConstrainedPlanner(double omega_c, double sample_time,
                             double max_velocity, double max_acceleration,
                             double max_jerk, double lower_position,
                             double upper_position) noexcept;

  bool configure(double omega_c, double sample_time, double max_velocity,
                 double max_acceleration, double max_jerk,
                 double lower_position, double upper_position) noexcept;
  bool reset(double position, double velocity = 0.0,
             double acceleration = 0.0) noexcept;
  void setTarget(double target) noexcept;
  bool update(double dt) noexcept;

  void getState(double& position, double& velocity, double& acceleration,
                double& jerk) const noexcept;
  double position() const noexcept { return position_; }
  double velocity() const noexcept { return velocity_; }
  double acceleration() const noexcept { return acceleration_; }
  double jerk() const noexcept { return jerk_; }
  double target() const noexcept { return target_; }

 private:
  void recalculateCoefficients() noexcept;

  double omega_c_{0.0};
  double sample_time_{0.0};
  double max_velocity_{0.0};
  double max_acceleration_{0.0};
  double max_jerk_{0.0};
  double lower_position_{0.0};
  double upper_position_{0.0};
  double k0_{0.0};
  double k1_{0.0};
  double k2_{0.0};
  double k3_{0.0};
  double position_{0.0};
  double velocity_{0.0};
  double acceleration_{0.0};
  double jerk_{0.0};
  double target_{0.0};
  bool configured_{false};
};

class PicoEeFrankaDlsIk7 {
 public:
  PicoEeFrankaDlsIk7(IterativeDlsConfig dls_config,
                     PicoEeFrankaDlsConfig planner_config,
                     double joint_margin_rad);

  PicoEeFrankaDlsResult solve(const PicoEeFrankaDlsInput& input);
  void reset(const ArmMotionState& state) noexcept;
  const Vec7& goal() const noexcept { return goal_; }
  const Vec7& plannerTarget() const noexcept { return planner_target_; }

 private:
  bool initialize(const PicoEeFrankaDlsInput& input) noexcept;
  bool validInput(const PicoEeFrankaDlsInput& input) const noexcept;
  double effectiveVelocityLimit(const PicoEeFrankaDlsInput& input,
                                int joint) const noexcept;

  FrankaIterativePoseDlsIk7 dls_;
  PicoEeFrankaDlsConfig planner_config_;
  double joint_margin_rad_{0.0};
  std::array<RealTimeConstrainedPlanner, kArmDof> planners_;
  Vec7 safe_lower_{Vec7::Zero()};
  Vec7 safe_upper_{Vec7::Zero()};
  Vec7 goal_{Vec7::Zero()};
  Vec7 planner_target_{Vec7::Zero()};
  ArmMotionState pending_reset_;
  bool pending_reset_valid_{false};
  bool initialized_{false};
};

}  // namespace tianji_qp_ik
