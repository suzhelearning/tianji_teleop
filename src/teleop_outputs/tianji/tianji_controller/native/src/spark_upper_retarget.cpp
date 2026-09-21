#include "tianji_qp_ik/spark_upper_retarget.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tianji_qp_ik {
namespace {

constexpr double kMinimumSegmentLength = 0.05;
constexpr double kMinimumAlignmentSegmentLength = 0.005;

enum PointIndex : std::size_t {
  kLeftShoulder = 0,
  kLeftElbow = 1,
  kLeftWrist = 2,
  kLeftHand = 3,
  kRightShoulder = 4,
  kRightElbow = 5,
  kRightWrist = 6,
  kRightHand = 7,
};

bool finiteVector(const Eigen::Vector3d& vector) {
  const double norm = vector.norm();
  return vector.allFinite() && std::isfinite(norm) &&
         norm >= kMinimumSegmentLength;
}

bool finiteGeometry(const SparkUpperRobotGeometry& geometry) {
  return geometry.left_shoulder.allFinite() &&
         geometry.right_shoulder.allFinite() &&
         finiteVector(geometry.left_upper_arm_local) &&
         finiteVector(geometry.left_forearm_local) &&
         finiteVector(geometry.left_wrist_to_palm_local) &&
         finiteVector(geometry.right_upper_arm_local) &&
         finiteVector(geometry.right_forearm_local) &&
         finiteVector(geometry.right_wrist_to_palm_local);
}

bool finitePose(const Pose& pose) {
  return pose.position.allFinite() && isProperRotation(pose.rotation);
}

bool finiteQuaternion(const Eigen::Quaterniond& quaternion) {
  const double norm = quaternion.norm();
  return quaternion.coeffs().allFinite() && std::isfinite(norm) &&
         norm >= 1.0e-9;
}

double quaternionDistance(const Eigen::Quaterniond& first,
                          const Eigen::Quaterniond& second) {
  const double dot = std::clamp(
      std::abs(first.normalized().dot(second.normalized())), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

struct SegmentSelection {
  bool valid{false};
  Eigen::Vector3d vector{Eigen::Vector3d::Zero()};
  SparkSegmentSource source{SparkSegmentSource::kInvalid};
  bool accepted_rotation{false};
  bool accepted_frame_alignment{false};
  Eigen::Vector3d source_local_direction{Eigen::Vector3d::Zero()};
};

SegmentSelection selectSegment(
    const PicoUpperLimbSkeleton& skeleton, std::size_t parent_index,
    std::size_t child_index, const Eigen::Vector3d& robot_local,
    const std::array<Eigen::Quaterniond, kPicoUpperLimbPointCount>& previous,
    const std::array<bool, kPicoUpperLimbPointCount>& previous_valid,
    const std::array<Eigen::Vector3d, kPicoUpperLimbPointCount>&
        source_local_directions,
    const std::array<bool, kPicoUpperLimbPointCount>&
        source_local_directions_valid,
    double maximum_rotation_jump) {
  SegmentSelection result;
  const Eigen::Vector3d human_segment =
      skeleton.points[child_index] - skeleton.points[parent_index];
  const double human_length = human_segment.norm();
  const bool human_direction_observable =
      human_segment.allFinite() && std::isfinite(human_length) &&
      human_length >= kMinimumAlignmentSegmentLength;
  const bool human_fallback_valid =
      human_direction_observable && human_length >= kMinimumSegmentLength;
  if (skeleton.rotations_valid &&
      finiteQuaternion(skeleton.rotations[parent_index])) {
    const Eigen::Quaterniond current =
        skeleton.rotations[parent_index].normalized();
    const bool continuous = !previous_valid[parent_index] ||
        quaternionDistance(current, previous[parent_index]) <=
            maximum_rotation_jump;
    if (continuous) {
      Eigen::Vector3d source_local_direction = Eigen::Vector3d::Zero();
      if (source_local_directions_valid[parent_index]) {
        source_local_direction = source_local_directions[parent_index];
      } else if (human_direction_observable) {
        source_local_direction =
            current.conjugate() * (human_segment / human_length);
        source_local_direction.normalize();
        result.accepted_frame_alignment = true;
        result.source_local_direction = source_local_direction;
      }
      result.vector =
          robot_local.norm() * (current * source_local_direction);
      if (finiteVector(result.vector)) {
        result.valid = true;
        result.source = SparkSegmentSource::kRotation;
        result.accepted_rotation = true;
        return result;
      }
    }
  }

  if (!human_fallback_valid) {
    return result;
  }
  result.vector = robot_local.norm() * human_segment / human_length;
  result.valid = result.vector.allFinite();
  result.source = result.valid ? SparkSegmentSource::kPositionFallback
                               : SparkSegmentSource::kInvalid;
  return result;
}

}  // namespace

std::string_view toString(SparkSegmentSource source) noexcept {
  switch (source) {
    case SparkSegmentSource::kInvalid:
      return "invalid";
    case SparkSegmentSource::kRotation:
      return "rotation";
    case SparkSegmentSource::kPositionFallback:
      return "position_fallback";
  }
  return "invalid";
}

UpperSparkSkeletonScaler::UpperSparkSkeletonScaler(
    SparkUpperRobotGeometry geometry, SparkUpperQpoasesConfig config)
    : geometry_(std::move(geometry)), config_(std::move(config)) {
  if (!finiteGeometry(geometry_)) {
    throw std::invalid_argument("Spark upper robot geometry is invalid");
  }
  if (!std::isfinite(config_.maximum_joint_rotation_jump_rad) ||
      config_.maximum_joint_rotation_jump_rad <= 0.0 ||
      config_.maximum_joint_rotation_jump_rad > M_PI) {
    throw std::invalid_argument("Spark upper rotation jump limit is invalid");
  }
  reset();
}

void UpperSparkSkeletonScaler::reset() noexcept {
  previous_rotations_valid_.fill(false);
  source_local_directions_valid_.fill(false);
  for (Eigen::Quaterniond& rotation : previous_rotations_) {
    rotation.setIdentity();
  }
  for (Eigen::Vector3d& direction : source_local_directions_) {
    direction.setZero();
  }
}

bool UpperSparkSkeletonScaler::validInput(
    const PicoUpperLimbSkeleton& skeleton, const Pose& left_palm,
    const Pose& right_palm) const noexcept {
  if (!skeleton.valid || !finitePose(left_palm) || !finitePose(right_palm)) {
    return false;
  }
  for (const Eigen::Vector3d& point : skeleton.points) {
    if (!point.allFinite()) {
      return false;
    }
  }
  return true;
}

UpperSparkSkeletonScaler::ArmBuildResult UpperSparkSkeletonScaler::makeTarget(
    ArmSide side, const PicoUpperLimbSkeleton& skeleton,
    const Pose& palm) const {
  const bool left = side == ArmSide::kLeft;
  const std::size_t shoulder_index = left ? kLeftShoulder : kRightShoulder;
  const std::size_t elbow_index = left ? kLeftElbow : kRightElbow;
  const std::size_t wrist_index = left ? kLeftWrist : kRightWrist;
  const std::size_t hand_index = left ? kLeftHand : kRightHand;
  const Eigen::Vector3d& shoulder =
      left ? geometry_.left_shoulder : geometry_.right_shoulder;
  const Eigen::Vector3d& upper_local =
      left ? geometry_.left_upper_arm_local : geometry_.right_upper_arm_local;
  const Eigen::Vector3d& forearm_local =
      left ? geometry_.left_forearm_local : geometry_.right_forearm_local;
  const Eigen::Vector3d& hand_local =
      left ? geometry_.left_wrist_to_palm_local
           : geometry_.right_wrist_to_palm_local;

  const SegmentSelection upper = selectSegment(
      skeleton, shoulder_index, elbow_index, upper_local,
      previous_rotations_, previous_rotations_valid_, source_local_directions_,
      source_local_directions_valid_, config_.maximum_joint_rotation_jump_rad);
  ArmBuildResult result;
  if (!upper.valid) {
    result.detail =
        left ? "left_upper_segment_invalid" : "right_upper_segment_invalid";
    return result;
  }
  const SegmentSelection forearm = selectSegment(
      skeleton, elbow_index, wrist_index, forearm_local,
      previous_rotations_, previous_rotations_valid_, source_local_directions_,
      source_local_directions_valid_, config_.maximum_joint_rotation_jump_rad);
  if (!forearm.valid) {
    result.detail = left ? "left_forearm_segment_invalid"
                         : "right_forearm_segment_invalid";
    return result;
  }
  const SegmentSelection hand = selectSegment(
      skeleton, wrist_index, hand_index, hand_local,
      previous_rotations_, previous_rotations_valid_, source_local_directions_,
      source_local_directions_valid_, config_.maximum_joint_rotation_jump_rad);
  if (!hand.valid) {
    result.detail =
        left ? "left_hand_segment_invalid" : "right_hand_segment_invalid";
    return result;
  }

  result.target.shoulder = shoulder;
  result.target.elbow = shoulder + upper.vector;
  result.target.wrist = result.target.elbow + forearm.vector;
  result.target.hand = result.target.wrist + hand.vector;
  result.target.palm = palm;
  result.target.palm.position = result.target.hand;
  result.target.upper_scale = 1.0;
  result.target.forearm_scale = 1.0;
  result.target.upper_source = upper.source;
  result.target.forearm_source = forearm.source;
  result.target.hand_source = hand.source;
  result.accepted_rotations = {{upper.accepted_rotation,
                                forearm.accepted_rotation,
                                hand.accepted_rotation}};
  result.accepted_frame_alignments = {{upper.accepted_frame_alignment,
                                       forearm.accepted_frame_alignment,
                                       hand.accepted_frame_alignment}};
  result.source_local_directions = {{upper.source_local_direction,
                                     forearm.source_local_direction,
                                     hand.source_local_direction}};
  result.valid = true;
  result.detail = "spark_upper_valid";
  return result;
}

void UpperSparkSkeletonScaler::commitRotations(
    ArmSide side, const PicoUpperLimbSkeleton& skeleton,
    const std::array<bool, 3>& accepted_rotations,
    const std::array<bool, 3>& accepted_frame_alignments,
    const std::array<Eigen::Vector3d, 3>& source_local_directions) noexcept {
  const std::size_t base =
      side == ArmSide::kLeft ? kLeftShoulder : kRightShoulder;
  for (std::size_t segment = 0; segment < accepted_rotations.size(); ++segment) {
    const std::size_t index = base + segment;
    if (accepted_rotations[segment]) {
      previous_rotations_[index] = skeleton.rotations[index].normalized();
      previous_rotations_valid_[index] = true;
    }
    if (accepted_frame_alignments[segment]) {
      source_local_directions_[index] =
          source_local_directions[segment].normalized();
      source_local_directions_valid_[index] = true;
    }
  }
}

SparkUpperTargets UpperSparkSkeletonScaler::update(
    const PicoUpperLimbSkeleton& skeleton, const Pose& left_palm,
    const Pose& right_palm) {
  SparkUpperTargets result;
  result.calibrated = true;
  result.calibration_samples = 0;
  if (!validInput(skeleton, left_palm, right_palm)) {
    result.detail = "invalid_skeleton";
    return result;
  }

  const ArmBuildResult left = makeTarget(ArmSide::kLeft, skeleton, left_palm);
  if (!left.valid) {
    result.detail = left.detail;
    return result;
  }
  const ArmBuildResult right =
      makeTarget(ArmSide::kRight, skeleton, right_palm);
  if (!right.valid) {
    result.detail = right.detail;
    return result;
  }

  commitRotations(ArmSide::kLeft, skeleton, left.accepted_rotations,
                  left.accepted_frame_alignments,
                  left.source_local_directions);
  commitRotations(ArmSide::kRight, skeleton, right.accepted_rotations,
                  right.accepted_frame_alignments,
                  right.source_local_directions);
  result.left = left.target;
  result.right = right.target;
  result.valid = true;
  result.detail = "spark_upper_valid";
  return result;
}

}  // namespace tianji_qp_ik
