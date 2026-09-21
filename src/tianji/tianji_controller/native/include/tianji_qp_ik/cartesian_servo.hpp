#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/target_manager.hpp"
#include "tianji_qp_ik/types.hpp"

namespace tianji_qp_ik {

Vec6 cartesianServoTwist(const CartesianServoConfig& config, const Pose& desired,
                         const Pose& current,
                         const Vec6& target_twist = Vec6::Zero());

Vec6 cartesianReferenceServoTwist(const CartesianServoConfig& config,
                                  const CartesianReference& reference,
                                  const Pose& measured);

}  // namespace tianji_qp_ik
