#pragma once

#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/target_manager.hpp"
#include "tianji_mapped_palm/types.hpp"

namespace tianji_mapped_palm {

struct CartesianFeedbackGainSchedule {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation{Eigen::Vector3d::Zero()};
};

Vec6 cartesianServoTwist(const CartesianServoConfig& config, const Pose& desired,
                         const Pose& current,
                         const Vec6& target_twist = Vec6::Zero());

Vec6 cartesianReferenceServoTwist(const CartesianServoConfig& config,
                                  const CartesianReference& reference,
                                  const Pose& measured);

Vec6 cartesianReferenceFeedbackTwist(const CartesianServoConfig& config,
                                     const Pose& desired,
                                     const Pose& measured);

Vec6 cartesianReferenceFeedbackTwist(const CartesianServoConfig& config,
                                     const Pose& desired,
                                     const Pose& measured,
                                     double motion_phase);

CartesianFeedbackGainSchedule cartesianFeedbackGainSchedule(
    const CartesianServoConfig& config, const Pose& desired,
    const Pose& measured, double motion_phase = 0.0);

CartesianFeedbackGainSchedule cartesianFeedbackGainSchedule(
    const CartesianServoConfig& config, const Pose& desired,
    const Pose& measured, double linear_motion_phase,
    double angular_motion_phase);

Vec6 clampCartesianTwist(const CartesianServoConfig& config,
                         const Vec6& twist);

}  // namespace tianji_mapped_palm
