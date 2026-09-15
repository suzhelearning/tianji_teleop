#include "tianji_mapped_palm/pico_mapped_corrected_palm.hpp"

#include "tianji_mapped_palm/so3.hpp"

#include <Eigen/Geometry>

#include <cmath>

namespace tianji_mapped_palm {
namespace {

constexpr double kQuaternionNormTolerance = 1.0e-6;

Eigen::Matrix3d leftEeBasis() noexcept {
  Eigen::Matrix3d basis;
  basis.col(0) = -Eigen::Vector3d::UnitY();
  basis.col(1) = -Eigen::Vector3d::UnitZ();
  basis.col(2) = Eigen::Vector3d::UnitX();
  return basis;
}

Eigen::Matrix3d rightEeBasis() noexcept {
  Eigen::Matrix3d basis;
  basis.col(0) = Eigen::Vector3d::UnitY();
  basis.col(1) = Eigen::Vector3d::UnitZ();
  basis.col(2) = Eigen::Vector3d::UnitX();
  return basis;
}

bool validQuaternion(const Eigen::Quaterniond& quaternion) noexcept {
  const double norm = quaternion.norm();
  return quaternion.coeffs().allFinite() && std::isfinite(norm) &&
         std::abs(norm - 1.0) <= kQuaternionNormTolerance;
}

bool validSkeleton(const PicoUpperLimbSkeleton& skeleton) noexcept {
  if (!skeleton.valid || !skeleton.rotations_valid) {
    return false;
  }
  for (const Eigen::Vector3d& point : skeleton.points) {
    if (!point.allFinite()) {
      return false;
    }
  }
  for (const Eigen::Quaterniond& rotation : skeleton.rotations) {
    if (!validQuaternion(rotation)) {
      return false;
    }
  }
  return true;
}

Pose mappedPose(const PicoUpperLimbSkeleton& skeleton, std::size_t point_index,
                const Eigen::Matrix3d& basis) noexcept {
  Pose pose;
  pose.position = skeleton.points[point_index];
  pose.rotation = skeleton.rotations[point_index].normalized().toRotationMatrix() *
                  basis;
  if (!isProperRotation(pose.rotation)) {
    return Pose{};
  }
  return pose;
}

}  // namespace

PicoMappedCorrectedPalmResult selectMappedCorrectedPalm(
    const PicoTeleopFrame& frame) noexcept {
  PicoMappedCorrectedPalmResult result;
  if (!validSkeleton(frame.upper_limb_skeleton)) {
    return result;
  }

  result.left = mappedPose(frame.upper_limb_skeleton, kPicoLeftHandPoint,
                           leftEeBasis());
  result.right = mappedPose(frame.upper_limb_skeleton, kPicoRightHandPoint,
                            rightEeBasis());
  result.valid = result.left.position.allFinite() &&
                 result.right.position.allFinite() &&
                 isProperRotation(result.left.rotation) &&
                 isProperRotation(result.right.rotation);
  if (!result.valid) {
    result = PicoMappedCorrectedPalmResult{};
  }
  return result;
}

}  // namespace tianji_mapped_palm
