#include "tianji_mapped_palm/pico_skeleton_arm_angle.hpp"

#include <cmath>

namespace tianji_mapped_palm {
namespace {

constexpr double kAxisEpsilon = 1.0e-9;
constexpr double kRadialEpsilon = 1.0e-3;

ArmDirectionReference oneSide(const PicoUpperLimbSkeleton& skeleton,
                              std::size_t shoulder_index,
                              std::size_t elbow_index,
                              std::size_t wrist_index) noexcept {
  ArmDirectionReference result;
  if (!skeleton.valid) {
    return result;
  }
  const Eigen::Vector3d& shoulder = skeleton.points[shoulder_index];
  const Eigen::Vector3d& elbow = skeleton.points[elbow_index];
  const Eigen::Vector3d& wrist = skeleton.points[wrist_index];
  if (!shoulder.allFinite() || !elbow.allFinite() || !wrist.allFinite()) {
    return result;
  }

  const Eigen::Vector3d shoulder_to_wrist = wrist - shoulder;
  const double axis_norm = shoulder_to_wrist.norm();
  if (!std::isfinite(axis_norm) || axis_norm <= kAxisEpsilon) {
    return result;
  }
  const Eigen::Vector3d axis = shoulder_to_wrist / axis_norm;
  const Eigen::Vector3d shoulder_to_elbow = elbow - shoulder;
  Eigen::Vector3d radial = shoulder_to_elbow -
                           axis * axis.dot(shoulder_to_elbow);
  const double radial_norm = radial.norm();
  if (!radial.allFinite() || !std::isfinite(radial_norm) ||
      radial_norm <= kRadialEpsilon) {
    return result;
  }
  radial /= radial_norm;

  result.direction = radial;
  result.source = ArmDirectionReferenceSource::kPico;
  result.shoulder_to_wrist_axis_valid = axis.allFinite();
  result.shoulder_to_wrist_axis = axis;
  result.valid = result.shoulder_to_wrist_axis_valid &&
                 result.direction.allFinite() &&
                 std::abs(result.direction.norm() - 1.0) <= 1.0e-9;
  return result;
}

}  // namespace

DualArmDirectionReferences selectMappedSkeletonArmDirections(
    const PicoUpperLimbSkeleton& skeleton) noexcept {
  DualArmDirectionReferences result;
  result.left = oneSide(skeleton, kPicoLeftShoulderPoint, kPicoLeftElbowPoint,
                        kPicoLeftWristPoint);
  result.right = oneSide(skeleton, kPicoRightShoulderPoint,
                         kPicoRightElbowPoint, kPicoRightWristPoint);
  return result;
}

}  // namespace tianji_mapped_palm
