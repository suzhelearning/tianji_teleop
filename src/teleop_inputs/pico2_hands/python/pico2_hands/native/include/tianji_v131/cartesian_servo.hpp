#pragma once

#include "tianji_v131/config.hpp"
#include "tianji_v131/target_manager.hpp"
#include "tianji_v131/types.hpp"

namespace tianji_v131 {

struct CartesianServoBreakdown {
  Vec6 feedforward{Vec6::Zero()};
  Vec6 feedback{Vec6::Zero()};
  Vec6 demand{Vec6::Zero()};
  Vec6 command{Vec6::Zero()};
  bool linear_saturated{false};
  bool angular_saturated{false};
  bool valid{false};
};

Vec6 cartesianServoTwist(const CartesianServoConfig& config, const Pose& desired,
                         const Pose& current,
                         const Vec6& target_twist = Vec6::Zero());

Vec6 cartesianReferenceServoTwist(const CartesianServoConfig& config,
                                  const CartesianReference& reference,
                                  const Pose& measured);

CartesianServoBreakdown cartesianReferenceServoBreakdown(
    const CartesianServoConfig& config, const CartesianReference& reference,
    const Pose& measured);

}  // namespace tianji_v131
