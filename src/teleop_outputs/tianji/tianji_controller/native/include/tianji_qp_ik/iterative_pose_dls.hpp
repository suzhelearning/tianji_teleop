#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"

#include <functional>
#include <string_view>

namespace tianji_qp_ik {

enum class PoseDlsStatus { kConverged, kImproved, kRejected, kNotConverged };

using KinematicsEvaluator = std::function<ArmKinematicSample(const Vec7&)>;
using KinematicsFeasibility =
    std::function<bool(const ArmKinematicSample&)>;


struct PoseDlsResult {
  PoseDlsStatus status{PoseDlsStatus::kRejected};
  Vec7 q{Vec7::Zero()};
  int iterations{0};
  int joint_projection_count{0};
  double initial_position_error_m{0.0};
  double initial_orientation_error_rad{0.0};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  double minimum_singular_value{0.0};
  double damping{0.0};
  double solve_time_us{0.0};
  std::string_view detail{"not_solved"};
};


}  // namespace tianji_qp_ik
