#pragma once

#include "tianji_qp_ik/types.hpp"

#include <Eigen/Core>

namespace tianji_qp_ik {

bool isProperRotation(const Eigen::Matrix3d& rotation, double tolerance = 1e-6) noexcept;
Eigen::Vector3d so3Log(const Eigen::Matrix3d& rotation);
Vec6 poseErrorWorld(const Pose& desired, const Pose& current);
double rotationDistance(const Eigen::Matrix3d& first, const Eigen::Matrix3d& second);

}  // namespace tianji_qp_ik
