// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#pragma once

#include "tianji_v131/config.hpp"
#include "tianji_v131/pico_ee_v131_jacobian_svd.hpp"
#include "tianji_v131/pinocchio_arm_kinematics.hpp"

#include <limits>

namespace tianji_v131 {

struct PicoEeV131SingularityState {
  double sigma_min{0.0};
  double activation{0.0};
  Vec7 gradient{Vec7::Zero()};
  bool gradient_valid{false};
  bool gradient_refreshed{false};
  double gradient_age_seconds{0.0};
  bool gradient_sweep_active{false};
  int gradient_sweep_joint{-1};
  double gradient_source_distance_rad{0.0};
  double requested_sigma_dot{0.0};
  bool escape_active{false};
};

double picoEeV131SingularityActivation(double sigma_min,
                                       double threshold,
                                       double critical) noexcept;

class PicoEeV131SingularityEscape7 final {
 public:
  PicoEeV131SingularityEscape7(PicoEeV131VelocityQpConfig config,
                               PinocchioArmKinematics& kinematics,
                               ArmSide side);

  PicoEeV131SingularityState update(const Vec7& q,
                                    const PicoEeV131JacobianSvdResult& svd,
                                    double now_seconds);
  void reset() noexcept;
  const PicoEeV131SingularityState& state() const noexcept { return state_; }

 private:
  PicoEeV131VelocityQpConfig config_;
  PinocchioArmKinematics& kinematics_;
  ArmSide side_;
  PicoEeV131SingularityState state_;
  double last_gradient_update_seconds_{
      -std::numeric_limits<double>::infinity()};
  double gradient_timestamp_seconds_{
      -std::numeric_limits<double>::infinity()};
  bool gradient_sweep_active_{false};
  Vec7 gradient_sweep_q_{Vec7::Zero()};
  Vec7 gradient_sweep_components_{Vec7::Zero()};
  int gradient_sweep_next_joint_{0};
};

}  // namespace tianji_v131
