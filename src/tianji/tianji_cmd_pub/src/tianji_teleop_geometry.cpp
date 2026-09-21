#include "pico_bridge/tianji_teleop_geometry.hpp"

#include <cmath>

namespace pico_bridge {
namespace {

constexpr double kMinimumQuaternionNorm = 1.0e-9;
constexpr double kMinimumShoulderSeparation = 0.10;
constexpr double kMaximumShoulderSeparation = 0.60;
constexpr double kMinimumProjectedSpineNorm = 0.05;
constexpr double kMinimumElbowProjectionNorm = 0.02;
constexpr double kMinimumArmSegmentNorm = 0.05;
constexpr double kRotationTolerance = 1.0e-9;
constexpr double kTianjiShoulderHalfSeparationM = 0.2115;
constexpr double kTianjiUpperArmLengthM = 0.28756390594092296;
constexpr double kTianjiForearmLengthM = 0.31451550041293674;
constexpr double kTianjiWristToTcpLengthM = 0.095;
constexpr double kTianjiShoulderMidpointHeightM = 1.121;

bool poseFinite(const PicoSkeletonPose & pose)
{
  return pose.position.allFinite() && pose.orientation.coeffs().allFinite();
}

bool normalizedArmSegment(
  const Eigen::Vector3d & segment, Eigen::Vector3d & direction)
{
  const double norm = segment.norm();
  if (!segment.allFinite() || !std::isfinite(norm) ||
    norm < kMinimumArmSegmentNorm)
  {
    return false;
  }
  direction = segment / norm;
  return direction.allFinite();
}

Eigen::Matrix3d leftEndEffectorBasis()
{
  Eigen::Matrix3d basis;
  basis.col(0) = -Eigen::Vector3d::UnitY();
  basis.col(1) = -Eigen::Vector3d::UnitZ();
  basis.col(2) = Eigen::Vector3d::UnitX();
  return basis;
}

Eigen::Matrix3d rightEndEffectorBasis()
{
  Eigen::Matrix3d basis;
  basis.col(0) = Eigen::Vector3d::UnitY();
  basis.col(1) = Eigen::Vector3d::UnitZ();
  basis.col(2) = Eigen::Vector3d::UnitX();
  return basis;
}

Eigen::Isometry3d picoHandTransform(
  const PicoSkeletonPose & pose, const Eigen::Matrix3d & end_effector_basis,
  double pico_world_x_offset_m)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = pose.position +
    pico_world_x_offset_m * Eigen::Vector3d::UnitX();
  transform.linear() =
    pose.orientation.normalized().toRotationMatrix() * end_effector_basis;
  return transform;
}

Eigen::Isometry3d robotSegmentTarget(
  const PicoSkeletonPose & hand_pose,
  const Eigen::Matrix3d & end_effector_basis,
  const Eigen::Vector3d & robot_shoulder,
  const Eigen::Vector3d & upper_direction_pico,
  const Eigen::Vector3d & forearm_direction_pico,
  const Eigen::Matrix3d & pico_to_robot,
  double reach_scale)
{
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear() = pico_to_robot *
    hand_pose.orientation.normalized().toRotationMatrix() *
    end_effector_basis;
  target.translation() = robot_shoulder +
    reach_scale * kTianjiUpperArmLengthM *
    (pico_to_robot * upper_direction_pico) +
    reach_scale * kTianjiForearmLengthM *
    (pico_to_robot * forearm_direction_pico) +
    kTianjiWristToTcpLengthM * target.linear().col(2);
  return target;
}

PicoArmDirection armDirection(
  const Eigen::Vector3d & shoulder, const Eigen::Vector3d & elbow,
  const Eigen::Vector3d & wrist, const Eigen::Matrix3d & pico_to_robot)
{
  PicoArmDirection result;
  Eigen::Vector3d upper_direction_pico;
  Eigen::Vector3d forearm_direction_pico;
  if (!normalizedArmSegment(
      elbow - shoulder, upper_direction_pico) ||
    !normalizedArmSegment(wrist - elbow, forearm_direction_pico))
  {
    return result;
  }

  // Keep the original observability gate: when the human elbow is nearly
  // collinear with the shoulder-to-wrist line, the elbow-side direction is
  // not observable and must not be used as a noisy redundancy reference.
  const Eigen::Vector3d human_axis = wrist - shoulder;
  const double human_axis_norm = human_axis.norm();
  if (!std::isfinite(human_axis_norm) ||
    human_axis_norm <= kMinimumQuaternionNorm)
  {
    return result;
  }
  const Eigen::Vector3d human_axis_unit = human_axis / human_axis_norm;
  const Eigen::Vector3d human_radial =
    (elbow - shoulder) - human_axis_unit *
    human_axis_unit.dot(elbow - shoulder);
  if (!std::isfinite(human_radial.norm()) ||
    human_radial.norm() < kMinimumElbowProjectionNorm)
  {
    return result;
  }

  // The position target is built from fixed Tianji segment lengths.  Use the
  // same retargeted shoulder-to-wrist axis for the redundancy direction;
  // projecting against the human shoulder-to-wrist axis would create a
  // geometry mismatch whenever the human and robot segment proportions differ.
  const Eigen::Vector3d upper_direction_robot =
    pico_to_robot * upper_direction_pico;
  const Eigen::Vector3d forearm_direction_robot =
    pico_to_robot * forearm_direction_pico;
  const Eigen::Vector3d retargeted_axis =
    kTianjiUpperArmLengthM * upper_direction_robot +
    kTianjiForearmLengthM * forearm_direction_robot;
  const double axis_norm = retargeted_axis.norm();
  if (!std::isfinite(axis_norm) || axis_norm <= kMinimumQuaternionNorm) {
    return result;
  }
  const Eigen::Vector3d axis_unit = retargeted_axis / axis_norm;
  const Eigen::Vector3d radial =
    upper_direction_robot -
    axis_unit * axis_unit.dot(upper_direction_robot);
  const double radial_norm = radial.norm();
  if (!std::isfinite(radial_norm) ||
    radial_norm < kMinimumElbowProjectionNorm)
  {
    return result;
  }
  // `radial` is already expressed in robot coordinates above.  Applying
  // pico_to_robot again would rotate the redundancy reference twice whenever
  // the operator's torso frame is not aligned with the robot frame.
  result.direction = radial / radial_norm;
  const double direction_norm = result.direction.norm();
  if (!result.direction.allFinite() ||
    !std::isfinite(direction_norm) || direction_norm <= kMinimumQuaternionNorm)
  {
    result.direction.setZero();
    return result;
  }
  result.direction /= direction_norm;
  result.valid = true;
  return result;
}

}  // namespace

PicoShoulderMapResult map_pico_palms_to_tianji(
  const PicoSkeletonFrame & skeleton,
  const PicoPositionRetargetingConfig & config)
{
  PicoShoulderMapResult result;
  if (!std::isfinite(config.robot_arm_reach_scale) ||
    config.robot_arm_reach_scale <= 0.0 ||
    config.robot_arm_reach_scale > 1.0)
  {
    result.rejection_reason = "robot_arm_reach_scale_out_of_range";
    return result;
  }
  if (!std::isfinite(config.pico_world_x_offset_m)) {
    result.rejection_reason = "pico_world_x_offset_non_finite";
    return result;
  }
  switch (config.mode) {
    case PicoPositionRetargetingMode::kRobotArmSegments:
    case PicoPositionRetargetingMode::kPicoPalm:
      break;
    default:
      result.rejection_reason = "position_retargeting_mode_invalid";
      return result;
  }
  for (const auto & pose : skeleton) {
    if (!poseFinite(pose)) {
      result.rejection_reason = "skeleton_pose_non_finite";
      return result;
    }
    const double quaternion_norm = pose.orientation.norm();
    if (!std::isfinite(quaternion_norm) || quaternion_norm < kMinimumQuaternionNorm) {
      result.rejection_reason = "skeleton_quaternion_invalid";
      return result;
    }
  }

  const Eigen::Vector3d & left_shoulder =
    skeleton[kPicoLeftShoulder].position;
  const Eigen::Vector3d & right_shoulder =
    skeleton[kPicoRightShoulder].position;
  const Eigen::Vector3d & spine2 = skeleton[kPicoSpine2].position;

  const Eigen::Vector3d shoulder_vector = left_shoulder - right_shoulder;
  const double shoulder_separation = shoulder_vector.norm();
  if (shoulder_separation < kMinimumShoulderSeparation ||
    shoulder_separation > kMaximumShoulderSeparation)
  {
    result.rejection_reason = "shoulder_separation_out_of_range";
    return result;
  }

  const Eigen::Vector3d origin = 0.5 * (left_shoulder + right_shoulder);
  const Eigen::Vector3d y_axis = shoulder_vector / shoulder_separation;
  const Eigen::Vector3d z_raw = origin - spine2;
  Eigen::Vector3d z_axis = z_raw - y_axis * y_axis.dot(z_raw);
  const double projected_spine_norm = z_axis.norm();
  if (projected_spine_norm < kMinimumProjectedSpineNorm) {
    result.rejection_reason = "spine_direction_degenerate";
    return result;
  }
  z_axis /= projected_spine_norm;
  Eigen::Vector3d x_axis = y_axis.cross(z_axis);
  const double x_norm = x_axis.norm();
  if (!std::isfinite(x_norm) || x_norm < kRotationTolerance) {
    result.rejection_reason = "shoulder_rotation_invalid";
    return result;
  }
  x_axis /= x_norm;
  z_axis = x_axis.cross(y_axis).normalized();

  Eigen::Isometry3d pico_shoulder_frame = Eigen::Isometry3d::Identity();
  pico_shoulder_frame.linear().col(0) = x_axis;
  pico_shoulder_frame.linear().col(1) = y_axis;
  pico_shoulder_frame.linear().col(2) = z_axis;
  pico_shoulder_frame.translation() = origin;

  const Eigen::Matrix3d & rotation = pico_shoulder_frame.linear();
  const double orthogonality_error =
    (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();
  if (!rotation.allFinite() || rotation.determinant() <= 0.0 ||
    orthogonality_error > kRotationTolerance)
  {
    result.rejection_reason = "shoulder_rotation_invalid";
    return result;
  }

  Eigen::Isometry3d robot_midpoint_frame = Eigen::Isometry3d::Identity();
  robot_midpoint_frame.translation() =
    Eigen::Vector3d(0.0, 0.0, kTianjiShoulderMidpointHeightM);

  result.pico_shoulder_frame = pico_shoulder_frame;
  const Eigen::Isometry3d pico_to_robot_rigid =
    robot_midpoint_frame * pico_shoulder_frame.inverse();
  const std::array<std::size_t, kUpperLimbPointCount> upper_limb_indices{{
    kPicoLeftShoulder, kPicoLeftElbow, kPicoLeftWrist, kPicoLeftHand,
    kPicoRightShoulder, kPicoRightElbow, kPicoRightWrist, kPicoRightHand,
  }};
  for (std::size_t index = 0; index < upper_limb_indices.size(); ++index) {
    result.upper_limb_skeleton.points[index] = pico_to_robot_rigid *
      skeleton[upper_limb_indices[index]].position;
  }
  result.upper_limb_skeleton.valid = true;
  const Eigen::Matrix3d pico_to_robot =
    robot_midpoint_frame.linear() * pico_shoulder_frame.linear().transpose();
  for (std::size_t index = 0; index < upper_limb_indices.size(); ++index) {
    const Eigen::Matrix3d mapped_rotation = pico_to_robot *
      skeleton[upper_limb_indices[index]].orientation.normalized()
      .toRotationMatrix();
    result.upper_limb_skeleton.rotations[index] =
      Eigen::Quaterniond(mapped_rotation).normalized();
  }
  result.upper_limb_skeleton.rotations_valid = true;
  if (config.mode == PicoPositionRetargetingMode::kPicoPalm) {
    result.left_target = pico_to_robot_rigid *
      picoHandTransform(
      skeleton[kPicoLeftHand], leftEndEffectorBasis(),
      config.pico_world_x_offset_m);
    result.right_target = pico_to_robot_rigid *
      picoHandTransform(
      skeleton[kPicoRightHand], rightEndEffectorBasis(),
      config.pico_world_x_offset_m);
  } else {
    const Eigen::Vector3d left_upper =
      skeleton[kPicoLeftElbow].position -
      skeleton[kPicoLeftShoulder].position;
    const Eigen::Vector3d left_forearm =
      skeleton[kPicoLeftWrist].position -
      skeleton[kPicoLeftElbow].position;
    const Eigen::Vector3d right_upper =
      skeleton[kPicoRightElbow].position -
      skeleton[kPicoRightShoulder].position;
    const Eigen::Vector3d right_forearm =
      skeleton[kPicoRightWrist].position -
      skeleton[kPicoRightElbow].position;
    Eigen::Vector3d left_upper_direction;
    Eigen::Vector3d left_forearm_direction;
    Eigen::Vector3d right_upper_direction;
    Eigen::Vector3d right_forearm_direction;
    if (!normalizedArmSegment(left_upper, left_upper_direction)) {
      result.rejection_reason = "left_upper_arm_segment_degenerate";
      return result;
    }
    if (!normalizedArmSegment(left_forearm, left_forearm_direction)) {
      result.rejection_reason = "left_forearm_segment_degenerate";
      return result;
    }
    if (!normalizedArmSegment(right_upper, right_upper_direction)) {
      result.rejection_reason = "right_upper_arm_segment_degenerate";
      return result;
    }
    if (!normalizedArmSegment(right_forearm, right_forearm_direction)) {
      result.rejection_reason = "right_forearm_segment_degenerate";
      return result;
    }
    result.left_target = robotSegmentTarget(
      skeleton[kPicoLeftHand], leftEndEffectorBasis(),
      Eigen::Vector3d(
        0.0, kTianjiShoulderHalfSeparationM,
        kTianjiShoulderMidpointHeightM),
      left_upper_direction, left_forearm_direction, pico_to_robot,
      config.robot_arm_reach_scale);
    result.right_target = robotSegmentTarget(
      skeleton[kPicoRightHand], rightEndEffectorBasis(),
      Eigen::Vector3d(
        0.0, -kTianjiShoulderHalfSeparationM,
        kTianjiShoulderMidpointHeightM),
      right_upper_direction, right_forearm_direction, pico_to_robot,
      config.robot_arm_reach_scale);
  }
  result.left_arm_direction = armDirection(
    skeleton[kPicoLeftShoulder].position,
    skeleton[kPicoLeftElbow].position,
    skeleton[kPicoLeftWrist].position,
    pico_to_robot);
  result.right_arm_direction = armDirection(
    skeleton[kPicoRightShoulder].position,
    skeleton[kPicoRightElbow].position,
    skeleton[kPicoRightWrist].position,
    pico_to_robot);
  result.valid = true;
  return result;
}

}  // namespace pico_bridge
