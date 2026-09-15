#pragma once

#include "tianji_qp_ik/acceleration_ik.hpp"
#include "tianji_qp_ik/config.hpp"

namespace tianji_qp_ik {

JointAccelerationBounds computeJointAccelerationBounds(
    const Vec7& q_model, const Vec7& qdot_model,
    const Vec7& qddot_previous, const ArmLimits& limits,
    const JointAccelerationLimitConfig& config, double dt);

Vec7 seedPreviousAccelerationForFeasibility(
    const Vec7& q_model, const Vec7& qdot_model, const ArmLimits& limits,
    const JointAccelerationLimitConfig& config, double dt);

}  // namespace tianji_qp_ik
