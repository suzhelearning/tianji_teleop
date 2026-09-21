#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/target_manager.hpp"

namespace tianji_qp_ik {

Vec6 cartesianAccelerationCommand(
    const CartesianAccelerationServoConfig& config,
    const CartesianReference& reference, const Pose& measured_pose,
    const Vec6& measured_twist);

}  // namespace tianji_qp_ik
