#pragma once
#include "tianji_qp_ik/types.hpp"
#include <string_view>
namespace tianji_qp_ik {
struct JointVelocityBounds {
  Vec7 lower{Vec7::Zero()};
  Vec7 upper{Vec7::Zero()};
};
struct ArmIkResult {
  SolverStatus status{SolverStatus::kInvalidInput};
  Vec7 qdot{Vec7::Zero()};
  int iterations{0};
  double solve_time_us{0.0};
  std::string_view detail{"not_solved"};
};
}  // namespace tianji_qp_ik
