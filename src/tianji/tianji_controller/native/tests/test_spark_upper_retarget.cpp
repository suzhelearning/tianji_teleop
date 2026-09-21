#include "tianji_qp_ik/spark_upper_retarget.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>

namespace tianji_qp_ik {
namespace {

SparkUpperQpoasesConfig testConfig() {
  SparkUpperQpoasesConfig config;
  config.maximum_joint_rotation_jump_rad = 0.5;
  return config;
}

SparkUpperRobotGeometry testGeometry() {
  SparkUpperRobotGeometry geometry;
  geometry.left_shoulder = Eigen::Vector3d(0.0, 0.3, 1.1);
  geometry.right_shoulder = Eigen::Vector3d(0.0, -0.3, 1.1);
  geometry.left_upper_arm_local = Eigen::Vector3d(0.3, 0.0, 0.0);
  geometry.left_forearm_local = Eigen::Vector3d(0.4, 0.0, 0.0);
  geometry.left_wrist_to_palm_local = Eigen::Vector3d(0.1, 0.0, 0.0);
  geometry.right_upper_arm_local = Eigen::Vector3d(0.3, 0.0, 0.0);
  geometry.right_forearm_local = Eigen::Vector3d(0.4, 0.0, 0.0);
  geometry.right_wrist_to_palm_local = Eigen::Vector3d(0.1, 0.0, 0.0);
  return geometry;
}

Pose makePalm(const Eigen::Vector3d& position, double angle) {
  Pose palm;
  palm.position = position;
  palm.rotation =
      Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return palm;
}

PicoUpperLimbSkeleton makeSkeleton() {
  PicoUpperLimbSkeleton skeleton;
  skeleton.valid = true;
  skeleton.rotations_valid = true;
  skeleton.points[0] = Eigen::Vector3d(1.0, 0.2, 1.0);
  skeleton.points[1] = skeleton.points[0] + Eigen::Vector3d(0.6, 0.0, 0.0);
  skeleton.points[2] = skeleton.points[1] + Eigen::Vector3d(0.8, 0.0, 0.0);
  skeleton.points[3] = skeleton.points[2] + Eigen::Vector3d(0.2, 0.0, 0.0);
  skeleton.points[4] = Eigen::Vector3d(1.0, -0.2, 1.0);
  skeleton.points[5] = skeleton.points[4] + Eigen::Vector3d(0.6, 0.0, 0.0);
  skeleton.points[6] = skeleton.points[5] + Eigen::Vector3d(0.8, 0.0, 0.0);
  skeleton.points[7] = skeleton.points[6] + Eigen::Vector3d(0.2, 0.0, 0.0);
  return skeleton;
}

TEST(UpperSparkSkeletonScaler, ReconstructsRobotRestSkeletonOnFirstFrame) {
  const SparkUpperRobotGeometry geometry = testGeometry();
  UpperSparkSkeletonScaler scaler(geometry, testConfig());
  const Pose left_palm = makePalm(Eigen::Vector3d(4.0, 4.0, 4.0), 0.3);
  const Pose right_palm = makePalm(Eigen::Vector3d(5.0, 5.0, 5.0), -0.2);

  const SparkUpperTargets result =
      scaler.update(makeSkeleton(), left_palm, right_palm);

  ASSERT_TRUE(result.valid) << result.detail;
  EXPECT_TRUE(result.calibrated);
  EXPECT_EQ(result.calibration_samples, 0);
  EXPECT_TRUE(result.left.shoulder.isApprox(geometry.left_shoulder));
  EXPECT_NEAR((result.left.elbow - result.left.shoulder).norm(), 0.3, 1e-12);
  EXPECT_NEAR((result.left.wrist - result.left.elbow).norm(), 0.4, 1e-12);
  EXPECT_NEAR((result.left.hand - result.left.wrist).norm(), 0.1, 1e-12);
  EXPECT_EQ(result.left.upper_source, SparkSegmentSource::kRotation);
  EXPECT_EQ(result.left.forearm_source, SparkSegmentSource::kRotation);
  EXPECT_EQ(result.left.hand_source, SparkSegmentSource::kRotation);
  EXPECT_TRUE(result.left.palm.rotation.isApprox(left_palm.rotation));
  EXPECT_TRUE(result.right.palm.rotation.isApprox(right_palm.rotation));
}

TEST(UpperSparkSkeletonScaler, AlignsPicoAndRobotJointFrameBasesImmediately) {
  SparkUpperRobotGeometry geometry = testGeometry();
  geometry.left_upper_arm_local = Eigen::Vector3d(0.0, 0.0, 0.3);
  UpperSparkSkeletonScaler scaler(geometry, testConfig());
  auto skeleton = makeSkeleton();
  const Pose palm = makePalm(Eigen::Vector3d::Zero(), 0.0);

  const SparkUpperTargets first = scaler.update(skeleton, palm, palm);
  ASSERT_TRUE(first.valid) << first.detail;
  EXPECT_TRUE((first.left.elbow - first.left.shoulder).isApprox(
      Eigen::Vector3d(0.3, 0.0, 0.0), 1e-12));

  skeleton.rotations[0] = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  const SparkUpperTargets second = scaler.update(skeleton, palm, palm);
  ASSERT_TRUE(second.valid) << second.detail;
  EXPECT_TRUE((second.left.elbow - second.left.shoulder).isApprox(
      0.3 * Eigen::Vector3d(std::cos(0.4), std::sin(0.4), 0.0), 1e-12));
}

TEST(UpperSparkSkeletonScaler, FallsBackPerSegmentToPositionDirection) {
  UpperSparkSkeletonScaler scaler(testGeometry(), testConfig());
  auto skeleton = makeSkeleton();
  skeleton.rotations_valid = false;
  const Pose palm = makePalm(Eigen::Vector3d::Zero(), 0.0);

  const SparkUpperTargets result = scaler.update(skeleton, palm, palm);

  ASSERT_TRUE(result.valid) << result.detail;
  EXPECT_EQ(result.left.upper_source, SparkSegmentSource::kPositionFallback);
  EXPECT_EQ(result.left.forearm_source,
            SparkSegmentSource::kPositionFallback);
  EXPECT_EQ(result.left.hand_source, SparkSegmentSource::kPositionFallback);
}

TEST(UpperSparkSkeletonScaler, AlignsShortWristToPalmWhenRotationIsValid) {
  UpperSparkSkeletonScaler scaler(testGeometry(), testConfig());
  auto skeleton = makeSkeleton();
  skeleton.points[3] =
      skeleton.points[2] + Eigen::Vector3d(0.03, 0.0, 0.0);
  const Pose palm = makePalm(Eigen::Vector3d::Zero(), 0.0);

  const SparkUpperTargets result = scaler.update(skeleton, palm, palm);

  ASSERT_TRUE(result.valid) << result.detail;
  EXPECT_EQ(result.left.hand_source, SparkSegmentSource::kRotation);
  EXPECT_TRUE((result.left.hand - result.left.wrist).isApprox(
      Eigen::Vector3d(0.1, 0.0, 0.0), 1e-12));
}

TEST(UpperSparkSkeletonScaler, RejectsWhenRotationAndPositionAreBothInvalid) {
  UpperSparkSkeletonScaler scaler(testGeometry(), testConfig());
  auto skeleton = makeSkeleton();
  skeleton.rotations_valid = false;
  skeleton.points[1] = skeleton.points[0];
  const Pose palm = makePalm(Eigen::Vector3d::Zero(), 0.0);

  const SparkUpperTargets result = scaler.update(skeleton, palm, palm);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.detail, "left_upper_segment_invalid");
}

TEST(UpperSparkSkeletonScaler, CommitsFrameAlignmentBilaterally) {
  UpperSparkSkeletonScaler scaler(testGeometry(), testConfig());
  const Pose palm = makePalm(Eigen::Vector3d::Zero(), 0.0);
  auto invalid = makeSkeleton();
  invalid.points[5] = invalid.points[4];
  EXPECT_FALSE(scaler.update(invalid, palm, palm).valid);

  auto valid = makeSkeleton();
  valid.rotations[0] = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  const SparkUpperTargets result = scaler.update(valid, palm, palm);
  ASSERT_TRUE(result.valid) << result.detail;
  EXPECT_TRUE((result.left.elbow - result.left.shoulder).isApprox(
      Eigen::Vector3d(0.3, 0.0, 0.0), 1e-12));
}

}  // namespace
}  // namespace tianji_qp_ik
