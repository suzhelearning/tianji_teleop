#pragma once

#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/target_manager.hpp"

namespace tianji_mapped_palm {

Vec6 cartesianAccelerationCommand(
    const CartesianAccelerationServoConfig& config,
    const CartesianReference& reference, const Pose& measured_pose,
    const Vec6& measured_twist);

}  // namespace tianji_mapped_palm
