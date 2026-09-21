#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "pico_bridge/tianji_teleop_geometry.hpp"

namespace pb = pico_bridge;

namespace {

pb::PicoSkeletonFrame neutralSkeleton()
{
  pb::PicoSkeletonFrame frame{};
  for (auto & pose : frame) {
    pose.orientation = Eigen::Quaterniond::Identity();
  }
  frame[pb::kPicoSpine2].position = {0.0, 0.0, 1.0};
  frame[pb::kPicoLeftShoulder].position = {0.0, 0.2, 1.5};
  frame[pb::kPicoRightShoulder].position = {0.0, -0.2, 1.5};
  frame[pb::kPicoLeftElbow].position = {0.25, 0.2, 1.2};
  frame[pb::kPicoRightElbow].position = {0.25, -0.2, 1.2};
  frame[pb::kPicoLeftWrist].position = {0.5, 0.2, 1.5};
  frame[pb::kPicoRightWrist].position = {0.5, -0.2, 1.5};
  frame[pb::kPicoLeftHand].position = {0.5, 0.4, 1.4};
  frame[pb::kPicoRightHand].position = {0.5, -0.4, 1.4};
  return frame;
}

pb::PicoSkeletonFrame skeletonWithBentElbows()
{
  auto frame = neutralSkeleton();
  frame[pb::kPicoLeftElbow].position = {0.25, 0.2, 1.2};
  frame[pb::kPicoRightElbow].position = {0.25, -0.2, 1.2};
  frame[pb::kPicoLeftWrist].position = {0.5, 0.2, 1.5};
  frame[pb::kPicoRightWrist].position = {0.5, -0.2, 1.5};
  return frame;
}

Eigen::Matrix3d expectedLeftEndEffectorBasis()
{
  Eigen::Matrix3d basis;
  basis.col(0) = -Eigen::Vector3d::UnitY();
  basis.col(1) = -Eigen::Vector3d::UnitZ();
  basis.col(2) = Eigen::Vector3d::UnitX();
  return basis;
}

Eigen::Matrix3d expectedRightEndEffectorBasis()
{
  Eigen::Matrix3d basis;
  basis.col(0) = Eigen::Vector3d::UnitY();
  basis.col(1) = Eigen::Vector3d::UnitZ();
  basis.col(2) = Eigen::Vector3d::UnitX();
  return basis;
}

void applyRigidTransform(
  pb::PicoSkeletonFrame & frame,
  const Eigen::Matrix3d & rotation,
  const Eigen::Vector3d & translation)
{
  for (auto & pose : frame) {
    pose.position = rotation * pose.position + translation;
    pose.orientation = Eigen::Quaterniond(rotation) * pose.orientation;
  }
}

Eigen::Vector3d expectedRetargetedArmDirection(
  const pb::PicoSkeletonFrame & skeleton,
  std::size_t shoulder_index, std::size_t elbow_index, std::size_t wrist_index)
{
  const double upper_length = 0.28756390594092296;
  const double forearm_length = 0.31451550041293674;
  const Eigen::Vector3d upper =
    (skeleton[elbow_index].position - skeleton[shoulder_index].position).normalized();
  const Eigen::Vector3d forearm =
    (skeleton[wrist_index].position - skeleton[elbow_index].position).normalized();
  const Eigen::Vector3d axis =
    (upper_length * upper + forearm_length * forearm).normalized();
  return (upper - axis * axis.dot(upper)).normalized();
}

Eigen::Vector3d expectedRetargetedArmDirectionInRobot(
  const pb::PicoSkeletonFrame & skeleton,
  const pb::PicoShoulderMapResult & result,
  std::size_t shoulder_index, std::size_t elbow_index, std::size_t wrist_index)
{
  const double upper_length = 0.28756390594092296;
  const double forearm_length = 0.31451550041293674;
  const Eigen::Matrix3d pico_to_robot =
    result.pico_shoulder_frame.linear().transpose();
  const Eigen::Vector3d upper = pico_to_robot *
    (skeleton[elbow_index].position - skeleton[shoulder_index].position).normalized();
  const Eigen::Vector3d forearm = pico_to_robot *
    (skeleton[wrist_index].position - skeleton[elbow_index].position).normalized();
  const Eigen::Vector3d axis =
    (upper_length * upper + forearm_length * forearm).normalized();
  return (upper - axis * axis.dot(upper)).normalized();
}

}  // namespace

TEST(TianjiTeleopGeometry, RetargetsBothArmsUsingNinetyFivePercentRobotSegmentsByDefault)
{
  const auto skeleton = neutralSkeleton();
  const auto result = pb::map_pico_palms_to_tianji(skeleton);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  const double upper_length = 0.95 * std::hypot(0.287, 0.018);
  const double forearm_length = 0.95 * std::hypot(0.314, 0.018);
  const Eigen::Vector3d left_upper_direction =
    (skeleton[pb::kPicoLeftElbow].position -
    skeleton[pb::kPicoLeftShoulder].position).normalized();
  const Eigen::Vector3d left_forearm_direction =
    (skeleton[pb::kPicoLeftWrist].position -
    skeleton[pb::kPicoLeftElbow].position).normalized();
  const Eigen::Vector3d right_upper_direction =
    (skeleton[pb::kPicoRightElbow].position -
    skeleton[pb::kPicoRightShoulder].position).normalized();
  const Eigen::Vector3d right_forearm_direction =
    (skeleton[pb::kPicoRightWrist].position -
    skeleton[pb::kPicoRightElbow].position).normalized();
  const Eigen::Vector3d expected_left =
    Eigen::Vector3d(0.0, 0.2115, 1.121) +
    upper_length * left_upper_direction +
    forearm_length * left_forearm_direction +
    0.095 * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d expected_right =
    Eigen::Vector3d(0.0, -0.2115, 1.121) +
    upper_length * right_upper_direction +
    forearm_length * right_forearm_direction +
    0.095 * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(result.left_target.linear().isApprox(
    expectedLeftEndEffectorBasis(), 1e-12));
  EXPECT_TRUE(result.right_target.linear().isApprox(
    expectedRightEndEffectorBasis(), 1e-12));
  EXPECT_TRUE(result.left_target.translation().isApprox(expected_left, 1e-12));
  EXPECT_TRUE(result.right_target.translation().isApprox(expected_right, 1e-12));
}

TEST(TianjiTeleopGeometry, RigidlyMountsExactCorrectedUpperLimbSkeleton)
{
  const auto source = neutralSkeleton();
  const auto result = pb::map_pico_palms_to_tianji(source);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  ASSERT_TRUE(result.upper_limb_skeleton.valid);
  const auto & points = result.upper_limb_skeleton.points;
  const Eigen::Vector3d shoulder_midpoint =
    0.5 * (points[pb::kUpperLimbLeftShoulder] +
    points[pb::kUpperLimbRightShoulder]);
  EXPECT_TRUE(shoulder_midpoint.isApprox(
    Eigen::Vector3d(0.0, 0.0, 1.121), 1e-12));

  const std::array<std::pair<std::size_t, std::size_t>, 6> segments{{
    {pb::kUpperLimbLeftShoulder, pb::kUpperLimbLeftElbow},
    {pb::kUpperLimbLeftElbow, pb::kUpperLimbLeftWrist},
    {pb::kUpperLimbLeftWrist, pb::kUpperLimbLeftHand},
    {pb::kUpperLimbRightShoulder, pb::kUpperLimbRightElbow},
    {pb::kUpperLimbRightElbow, pb::kUpperLimbRightWrist},
    {pb::kUpperLimbRightWrist, pb::kUpperLimbRightHand},
  }};
  const std::array<std::size_t, 8> source_indices{{
    pb::kPicoLeftShoulder, pb::kPicoLeftElbow, pb::kPicoLeftWrist,
    pb::kPicoLeftHand, pb::kPicoRightShoulder, pb::kPicoRightElbow,
    pb::kPicoRightWrist, pb::kPicoRightHand,
  }};
  for (const auto & [begin, end] : segments) {
    const double source_length =
      (source[source_indices[end]].position -
      source[source_indices[begin]].position).norm();
    EXPECT_NEAR((points[end] - points[begin]).norm(), source_length, 1e-12);
  }
}

TEST(TianjiTeleopGeometry, RigidlyMapsAllCorrectedUpperLimbRotations)
{
  auto source = neutralSkeleton();
  const std::array<std::size_t, 8> source_indices{{
    pb::kPicoLeftShoulder, pb::kPicoLeftElbow, pb::kPicoLeftWrist,
    pb::kPicoLeftHand, pb::kPicoRightShoulder, pb::kPicoRightElbow,
    pb::kPicoRightWrist, pb::kPicoRightHand,
  }};
  for (std::size_t index = 0; index < source_indices.size(); ++index) {
    const Eigen::Vector3d axis =
      index % 3U == 0U ? Eigen::Vector3d::UnitX() :
      index % 3U == 1U ? Eigen::Vector3d::UnitY() :
                         Eigen::Vector3d::UnitZ();
    source[source_indices[index]].orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.05 * static_cast<double>(index + 1U), axis));
  }

  const auto result = pb::map_pico_palms_to_tianji(source);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  ASSERT_TRUE(result.upper_limb_skeleton.rotations_valid);
  const Eigen::Matrix3d pico_to_robot =
    result.pico_shoulder_frame.linear().transpose();
  for (std::size_t index = 0; index < source_indices.size(); ++index) {
    const Eigen::Matrix3d expected = pico_to_robot *
      source[source_indices[index]].orientation.normalized().toRotationMatrix();
    EXPECT_TRUE(result.upper_limb_skeleton.rotations[index]
      .toRotationMatrix().isApprox(expected, 1e-12)) << "index=" << index;
  }
}

TEST(TianjiTeleopGeometry, AppliesConfiguredPicoWorldXOffset)
{
  pb::PicoPositionRetargetingConfig config;
  config.mode = pb::PicoPositionRetargetingMode::kPicoPalm;
  config.pico_world_x_offset_m = 0.25;
  const auto result = pb::map_pico_palms_to_tianji(neutralSkeleton(), config);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_TRUE(result.left_target.translation().isApprox(
    Eigen::Vector3d(0.75, 0.4, 1.021), 1e-12));
  EXPECT_TRUE(result.right_target.translation().isApprox(
    Eigen::Vector3d(0.75, -0.4, 1.021), 1e-12));
  EXPECT_TRUE(result.left_target.linear().isApprox(
    expectedLeftEndEffectorBasis(), 1e-12));
  EXPECT_TRUE(result.right_target.linear().isApprox(
    expectedRightEndEffectorBasis(), 1e-12));
}

TEST(TianjiTeleopGeometry, MapsCorrectedElbowRadialDirectionsIntoRobotWorld)
{
  const auto result = pb::map_pico_palms_to_tianji(skeletonWithBentElbows());

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  ASSERT_TRUE(result.left_arm_direction.valid);
  ASSERT_TRUE(result.right_arm_direction.valid);
  EXPECT_TRUE(result.left_arm_direction.direction.isApprox(
    expectedRetargetedArmDirection(
      skeletonWithBentElbows(), pb::kPicoLeftShoulder,
      pb::kPicoLeftElbow, pb::kPicoLeftWrist), 1e-12));
  EXPECT_TRUE(result.right_arm_direction.direction.isApprox(
    expectedRetargetedArmDirection(
      skeletonWithBentElbows(), pb::kPicoRightShoulder,
      pb::kPicoRightElbow, pb::kPicoRightWrist), 1e-12));
  EXPECT_NEAR(result.left_arm_direction.direction.norm(), 1.0, 1e-12);
  EXPECT_NEAR(result.right_arm_direction.direction.norm(), 1.0, 1e-12);
}

TEST(TianjiTeleopGeometry, PalmWorldXOffsetDoesNotChangeArmDirections)
{
  pb::PicoPositionRetargetingConfig without_offset_config;
  without_offset_config.mode = pb::PicoPositionRetargetingMode::kPicoPalm;
  without_offset_config.pico_world_x_offset_m = 0.0;
  pb::PicoPositionRetargetingConfig with_offset_config = without_offset_config;
  with_offset_config.pico_world_x_offset_m = 0.25;
  const auto without_offset = pb::map_pico_palms_to_tianji(
    skeletonWithBentElbows(), without_offset_config);
  const auto with_offset = pb::map_pico_palms_to_tianji(
    skeletonWithBentElbows(), with_offset_config);

  ASSERT_TRUE(without_offset.valid);
  ASSERT_TRUE(with_offset.valid);
  EXPECT_TRUE(without_offset.left_arm_direction.direction.isApprox(
    with_offset.left_arm_direction.direction, 1e-12));
  EXPECT_TRUE(without_offset.right_arm_direction.direction.isApprox(
    with_offset.right_arm_direction.direction, 1e-12));
}

TEST(TianjiTeleopGeometry, AppliesBodyRotationToRetargetedArmDirectionExactlyOnce)
{
  auto skeleton = skeletonWithBentElbows();
  const Eigen::Matrix3d rotation =
    (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY())).toRotationMatrix();
  applyRigidTransform(skeleton, rotation, Eigen::Vector3d(1.2, -0.7, 0.4));

  const auto result = pb::map_pico_palms_to_tianji(skeleton);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  ASSERT_TRUE(result.left_arm_direction.valid);
  ASSERT_TRUE(result.right_arm_direction.valid);
  EXPECT_TRUE(result.left_arm_direction.direction.isApprox(
    expectedRetargetedArmDirectionInRobot(
      skeleton, result, pb::kPicoLeftShoulder,
      pb::kPicoLeftElbow, pb::kPicoLeftWrist), 1e-12));
  EXPECT_TRUE(result.right_arm_direction.direction.isApprox(
    expectedRetargetedArmDirectionInRobot(
      skeleton, result, pb::kPicoRightShoulder,
      pb::kPicoRightElbow, pb::kPicoRightWrist), 1e-12));
}

TEST(TianjiTeleopGeometry, DegenerateElbowProjectionInvalidatesOnlyThatSide)
{
  auto frame = skeletonWithBentElbows();
  frame[pb::kPicoLeftElbow].position = {0.25, 0.2, 1.5};

  const auto result = pb::map_pico_palms_to_tianji(frame);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_FALSE(result.left_arm_direction.valid);
  EXPECT_TRUE(result.right_arm_direction.valid);
  EXPECT_TRUE(result.left_target.translation().allFinite());
  EXPECT_TRUE(result.right_target.translation().allFinite());
}

TEST(TianjiTeleopGeometry, NearStraightElbowInvalidatesOnlyArmDirection)
{
  auto frame = skeletonWithBentElbows();
  frame[pb::kPicoLeftElbow].position = {0.25, 0.21, 1.5};

  const auto result = pb::map_pico_palms_to_tianji(frame);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_FALSE(result.left_arm_direction.valid);
  EXPECT_TRUE(result.right_arm_direction.valid);
  EXPECT_TRUE(result.left_target.matrix().allFinite());
  EXPECT_TRUE(result.right_target.matrix().allFinite());
}

TEST(TianjiTeleopGeometry, MapsLeftRobotAxesFromLeftPalmLocalAxes)
{
  auto frame = neutralSkeleton();
  frame[pb::kPicoLeftHand].orientation = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()));

  const auto result = pb::map_pico_palms_to_tianji(frame);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  const Eigen::Matrix3d palm =
    frame[pb::kPicoLeftHand].orientation.toRotationMatrix();
  EXPECT_TRUE(result.left_target.linear().col(0).isApprox(-palm.col(1), 1e-12));
  EXPECT_TRUE(result.left_target.linear().col(1).isApprox(-palm.col(2), 1e-12));
  EXPECT_TRUE(result.left_target.linear().col(2).isApprox(palm.col(0), 1e-12));
}

TEST(TianjiTeleopGeometry, MapsRightRobotAxesFromRightPalmLocalAxes)
{
  auto frame = neutralSkeleton();
  frame[pb::kPicoRightHand].orientation = Eigen::Quaterniond(
    Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ()));

  const auto result = pb::map_pico_palms_to_tianji(frame);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  const Eigen::Matrix3d palm =
    frame[pb::kPicoRightHand].orientation.toRotationMatrix();
  EXPECT_TRUE(result.right_target.linear().col(0).isApprox(palm.col(1), 1e-12));
  EXPECT_TRUE(result.right_target.linear().col(1).isApprox(palm.col(2), 1e-12));
  EXPECT_TRUE(result.right_target.linear().col(2).isApprox(palm.col(0), 1e-12));
}

TEST(TianjiTeleopGeometry, KeepsWorldXOffsetFixedWhenPicoBodyRotates)
{
  pb::PicoPositionRetargetingConfig config;
  config.mode = pb::PicoPositionRetargetingMode::kPicoPalm;
  const auto baseline = pb::map_pico_palms_to_tianji(neutralSkeleton(), config);
  ASSERT_TRUE(baseline.valid);
  auto transformed = neutralSkeleton();
  const Eigen::Matrix3d rotation =
    (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY())).toRotationMatrix();
  applyRigidTransform(transformed, rotation, Eigen::Vector3d(1.2, -0.7, 0.4));

  const auto result = pb::map_pico_palms_to_tianji(transformed, config);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  const Eigen::Vector3d pico_world_offset(0.10, 0.0, 0.0);
  const Eigen::Vector3d baseline_robot_offset =
    baseline.pico_shoulder_frame.linear().transpose() * pico_world_offset;
  const Eigen::Vector3d transformed_robot_offset =
    result.pico_shoulder_frame.linear().transpose() * pico_world_offset;
  EXPECT_TRUE((result.left_target.translation() - transformed_robot_offset).isApprox(
    baseline.left_target.translation() - baseline_robot_offset, 1e-12));
  EXPECT_TRUE((result.right_target.translation() - transformed_robot_offset).isApprox(
    baseline.right_target.translation() - baseline_robot_offset, 1e-12));
  EXPECT_TRUE(result.left_target.linear().isApprox(
    baseline.left_target.linear(), 1e-12));
  EXPECT_TRUE(result.right_target.linear().isApprox(
    baseline.right_target.linear(), 1e-12));
}

TEST(TianjiTeleopGeometry, FullRobotSegmentScaleIsAvailableForComparison)
{
  auto config = pb::PicoPositionRetargetingConfig{};
  config.robot_arm_reach_scale = 1.0;
  const auto skeleton = neutralSkeleton();
  const auto result = pb::map_pico_palms_to_tianji(skeleton, config);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  const Eigen::Vector3d upper_direction =
    (skeleton[pb::kPicoLeftElbow].position -
    skeleton[pb::kPicoLeftShoulder].position).normalized();
  const Eigen::Vector3d forearm_direction =
    (skeleton[pb::kPicoLeftWrist].position -
    skeleton[pb::kPicoLeftElbow].position).normalized();
  const Eigen::Vector3d expected =
    Eigen::Vector3d(0.0, 0.2115, 1.121) +
    std::hypot(0.287, 0.018) * upper_direction +
    std::hypot(0.314, 0.018) * forearm_direction +
    0.095 * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(result.left_target.translation().isApprox(expected, 1e-12));
}

TEST(TianjiTeleopGeometry, RejectsInvalidRobotArmReachScale)
{
  for (const double scale : {0.0, 1.01,
      std::numeric_limits<double>::quiet_NaN()})
  {
    auto config = pb::PicoPositionRetargetingConfig{};
    config.robot_arm_reach_scale = scale;
    const auto result = pb::map_pico_palms_to_tianji(neutralSkeleton(), config);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.rejection_reason, "robot_arm_reach_scale_out_of_range");
  }
}

TEST(TianjiTeleopGeometry, RejectsInvalidInactiveConfigurationFields)
{
  auto legacy = pb::PicoPositionRetargetingConfig{};
  legacy.mode = pb::PicoPositionRetargetingMode::kPicoPalm;
  legacy.robot_arm_reach_scale =
    std::numeric_limits<double>::quiet_NaN();
  auto result = pb::map_pico_palms_to_tianji(neutralSkeleton(), legacy);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "robot_arm_reach_scale_out_of_range");

  auto robot_segments = pb::PicoPositionRetargetingConfig{};
  robot_segments.pico_world_x_offset_m =
    std::numeric_limits<double>::quiet_NaN();
  result = pb::map_pico_palms_to_tianji(
    neutralSkeleton(), robot_segments);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "pico_world_x_offset_non_finite");
}

TEST(TianjiTeleopGeometry, RejectsUnknownPositionRetargetingMode)
{
  auto config = pb::PicoPositionRetargetingConfig{};
  config.mode = static_cast<pb::PicoPositionRetargetingMode>(99);

  const auto result = pb::map_pico_palms_to_tianji(neutralSkeleton(), config);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "position_retargeting_mode_invalid");
}

TEST(TianjiTeleopGeometry, RejectsDegenerateArmSegmentsInRobotSegmentMode)
{
  auto frame = neutralSkeleton();
  frame[pb::kPicoLeftElbow].position =
    frame[pb::kPicoLeftShoulder].position;

  const auto result = pb::map_pico_palms_to_tianji(frame);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "left_upper_arm_segment_degenerate");
}

TEST(TianjiTeleopGeometry, RejectsNearZeroAndOverflowedArmSegments)
{
  auto near_zero = neutralSkeleton();
  near_zero[pb::kPicoLeftElbow].position =
    near_zero[pb::kPicoLeftShoulder].position +
    1.0e-6 * Eigen::Vector3d::UnitX();
  auto result = pb::map_pico_palms_to_tianji(near_zero);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "left_upper_arm_segment_degenerate");

  auto overflowed = neutralSkeleton();
  overflowed[pb::kPicoLeftElbow].position.x() =
    std::numeric_limits<double>::max();
  result = pb::map_pico_palms_to_tianji(overflowed);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "left_upper_arm_segment_degenerate");
}

TEST(TianjiTeleopGeometry, IgnoresHumanSegmentMagnitudesInRobotSegmentMode)
{
  const auto baseline = pb::map_pico_palms_to_tianji(neutralSkeleton());
  ASSERT_TRUE(baseline.valid) << baseline.rejection_reason;
  auto rescaled = neutralSkeleton();
  for (const auto indices : {
      std::array<std::size_t, 3>{
        pb::kPicoLeftShoulder, pb::kPicoLeftElbow, pb::kPicoLeftWrist},
      std::array<std::size_t, 3>{
        pb::kPicoRightShoulder, pb::kPicoRightElbow, pb::kPicoRightWrist}})
  {
    const Eigen::Vector3d shoulder = rescaled[indices[0]].position;
    const Eigen::Vector3d upper =
      rescaled[indices[1]].position - shoulder;
    const Eigen::Vector3d forearm =
      rescaled[indices[2]].position - rescaled[indices[1]].position;
    rescaled[indices[1]].position = shoulder + 0.55 * upper;
    rescaled[indices[2]].position =
      rescaled[indices[1]].position + 1.65 * forearm;
  }

  const auto result = pb::map_pico_palms_to_tianji(rescaled);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_TRUE(result.left_target.isApprox(baseline.left_target, 1e-12));
  EXPECT_TRUE(result.right_target.isApprox(baseline.right_target, 1e-12));
}

TEST(TianjiTeleopGeometry, RobotSegmentTargetsAreInvariantToPicoBodyTransform)
{
  const auto baseline = pb::map_pico_palms_to_tianji(neutralSkeleton());
  ASSERT_TRUE(baseline.valid) << baseline.rejection_reason;
  auto transformed = neutralSkeleton();
  const Eigen::Matrix3d rotation =
    (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY())).toRotationMatrix();
  applyRigidTransform(transformed, rotation, Eigen::Vector3d(1.2, -0.7, 0.4));

  const auto result = pb::map_pico_palms_to_tianji(transformed);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_TRUE(result.left_target.isApprox(baseline.left_target, 1e-12));
  EXPECT_TRUE(result.right_target.isApprox(baseline.right_target, 1e-12));
}

TEST(TianjiTeleopGeometry, LegacyPicoPalmModeDoesNotRequireArmSegments)
{
  auto frame = neutralSkeleton();
  frame[pb::kPicoLeftElbow].position =
    frame[pb::kPicoLeftShoulder].position;
  pb::PicoPositionRetargetingConfig config;
  config.mode = pb::PicoPositionRetargetingMode::kPicoPalm;

  const auto result = pb::map_pico_palms_to_tianji(frame, config);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_TRUE(result.left_target.translation().isApprox(
    Eigen::Vector3d(0.6, 0.4, 1.021), 1e-12));
}

TEST(TianjiTeleopGeometry, RejectsShoulderSeparationOutsideContract)
{
  auto too_close = neutralSkeleton();
  too_close[pb::kPicoLeftShoulder].position.y() = 0.049;
  too_close[pb::kPicoRightShoulder].position.y() = -0.049;
  auto result = pb::map_pico_palms_to_tianji(too_close);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "shoulder_separation_out_of_range");

  auto too_far = neutralSkeleton();
  too_far[pb::kPicoLeftShoulder].position.y() = 0.301;
  too_far[pb::kPicoRightShoulder].position.y() = -0.301;
  result = pb::map_pico_palms_to_tianji(too_far);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "shoulder_separation_out_of_range");
}

TEST(TianjiTeleopGeometry, RejectsDegenerateSpineDirection)
{
  auto frame = neutralSkeleton();
  frame[pb::kPicoSpine2].position = {0.0, 0.0, 1.49};

  const auto result = pb::map_pico_palms_to_tianji(frame);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "spine_direction_degenerate");
}

TEST(TianjiTeleopGeometry, RejectsNonFinitePoseAndInvalidQuaternion)
{
  auto non_finite = neutralSkeleton();
  non_finite[3].position.x() = std::numeric_limits<double>::quiet_NaN();
  auto result = pb::map_pico_palms_to_tianji(non_finite);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "skeleton_pose_non_finite");

  auto invalid_quaternion = neutralSkeleton();
  invalid_quaternion[pb::kPicoLeftHand].orientation.coeffs().setZero();
  result = pb::map_pico_palms_to_tianji(invalid_quaternion);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection_reason, "skeleton_quaternion_invalid");
}
