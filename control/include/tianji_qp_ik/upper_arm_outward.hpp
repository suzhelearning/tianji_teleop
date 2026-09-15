#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/types.hpp"

#include <Eigen/Core>

namespace tianji_qp_ik {

struct UpperArmOutwardState {
  bool valid{false};
  double distance_m{0.0};
  Vec7 jacobian{Vec7::Zero()};
};

struct UpperArmOutwardDiagnostics {
  UpperArmOutwardState state;
  bool constraint_active{false};
  double requested_lower{0.0};
  double effective_lower{0.0};
  double achieved{0.0};
  double residual{0.0};
  bool feasibility_clipped{false};
};

Eigen::Vector3d upperArmOutwardDirection(ArmSide side) noexcept;

UpperArmOutwardState computeUpperArmOutwardState(
    ArmSide side, const Eigen::Vector3d& shoulder_position,
    const Eigen::Vector3d& elbow_position,
    const Mat37& shoulder_position_jacobian,
    const Mat37& elbow_position_jacobian,
    double minimum_outward_distance_m) noexcept;

LinearJointConstraint makeVelocityOutwardConstraint(
    const UpperArmOutwardState& state, const Vec7& lower,
    const Vec7& upper, const UpperArmOutwardConfig& config,
    double dt) noexcept;

LinearJointConstraint makeAccelerationOutwardConstraint(
    const UpperArmOutwardState& state, const Vec7& qdot,
    double jdot_qdot, const Vec7& lower, const Vec7& upper,
    const UpperArmOutwardConfig& config, double dt) noexcept;

}  // namespace tianji_qp_ik
