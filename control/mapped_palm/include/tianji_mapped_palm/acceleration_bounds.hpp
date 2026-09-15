#pragma once

#include "tianji_mapped_palm/acceleration_ik.hpp"
#include "tianji_mapped_palm/config.hpp"

namespace tianji_mapped_palm {

JointAccelerationBounds computeJointAccelerationBounds(
    const Vec7& q_model, const Vec7& qdot_model,
    const Vec7& qddot_previous, const ArmLimits& limits,
    const JointAccelerationLimitConfig& config, double dt);

Vec7 seedPreviousAccelerationForFeasibility(
    const Vec7& q_model, const Vec7& qdot_model, const ArmLimits& limits,
    const JointAccelerationLimitConfig& config, double dt);

}  // namespace tianji_mapped_palm
