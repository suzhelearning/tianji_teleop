#include "tianji_qp_ik/target_manager.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

namespace tianji_qp_ik {
namespace {

TEST(TargetManager, DirectReferencesPreservePoseTwistAndStaleState) {
  DualArmTargets targets;
  targets.left.position << 0.4, 0.3, 1.2;
  targets.right.position << 0.5, -0.2, 1.1;
  targets.left_twist << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  targets.right_twist << -1.0, -2.0, -3.0, -4.0, -5.0, -6.0;
  targets.left_stale = true;
  targets.right_stale = false;

  const DualArmReferences references = directReferences(targets);

  EXPECT_TRUE(references.left.pose.position.isApprox(targets.left.position));
  EXPECT_TRUE(references.right.pose.position.isApprox(targets.right.position));
  EXPECT_TRUE(references.left.twist.isApprox(targets.left_twist));
  EXPECT_TRUE(references.right.twist.isApprox(targets.right_twist));
  EXPECT_TRUE(references.left.acceleration.isZero(1e-12));
  EXPECT_TRUE(references.right.acceleration.isZero(1e-12));
  EXPECT_TRUE(references.left.stale);
  EXPECT_FALSE(references.right.stale);
  EXPECT_TRUE(references.left.valid);
  EXPECT_TRUE(references.right.valid);
}

QpIkConfig config() {
  QpIkConfig result;
  result.trajectories.circle_radius = 0.06;
  result.trajectories.figure_eight_width = 0.08;
  result.trajectories.figure_eight_height = 0.05;
  result.trajectories.angular_amplitude = 0.35;
  result.trajectories.frequency_hz = 0.10;
  result.safety.max_target_position_step = 0.05;
  result.safety.max_target_orientation_step = 0.20;
  return result;
}

DualArmTargets initialTargets() {
  DualArmTargets targets;
  targets.left.position = {0.3, 0.2, 1.2};
  targets.right.position = {0.3, -0.2, 1.2};
  targets.left.rotation = Eigen::Matrix3d::Identity();
  targets.right.rotation = Eigen::Matrix3d::Identity();
  return targets;
}

TEST(TargetManager, HoldReturnsCapturedTargets) {
  TargetManager manager(config(), initialTargets());
  manager.setMode(TargetMode::kHold, 4.0);
  const DualArmTargets targets = manager.sample(123.0);
  EXPECT_TRUE(targets.left.position.isApprox(initialTargets().left.position));
  EXPECT_TRUE(targets.right.rotation.isApprox(initialTargets().right.rotation));
}

TEST(TargetManager, CircleAndFigureEightUseControlTime) {
  QpIkConfig trajectory_config = config();
  trajectory_config.safety.max_target_position_step = 1.0;
  trajectory_config.safety.max_target_orientation_step = 1.0;
  TargetManager manager(trajectory_config, initialTargets());
  manager.setMode(TargetMode::kCircle, 10.0);
  const DualArmTargets circle = manager.sample(12.5);
  EXPECT_NEAR(circle.left.position.x(), initialTargets().left.position.x() - 0.06, 1e-12);
  EXPECT_NEAR(circle.left.position.z(), initialTargets().left.position.z() + 0.06, 1e-12);

  TargetManager eight_manager(trajectory_config, initialTargets());
  eight_manager.setMode(TargetMode::kFigureEight, 10.0);
  const DualArmTargets eight = eight_manager.sample(11.25);
  const double phase = 0.25 * 3.14159265358979323846;
  EXPECT_NEAR(eight.left.position.x(),
              initialTargets().left.position.x() + 0.08 * std::sin(phase), 1e-12);
  EXPECT_NEAR(eight.left.position.z(),
              initialTargets().left.position.z() + 0.05 * std::sin(2.0 * phase), 1e-12);
}

TEST(TargetManager, OrientationModePreservesPosition) {
  QpIkConfig trajectory_config = config();
  trajectory_config.safety.max_target_orientation_step = 1.0;
  TargetManager manager(trajectory_config, initialTargets());
  manager.setMode(TargetMode::kOrientationOnly, 0.0);
  const DualArmTargets targets = manager.sample(2.5);
  EXPECT_TRUE(targets.left.position.isApprox(initialTargets().left.position));
  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(targets.left.rotation.isApprox(expected, 1e-12));
}

TEST(TargetManager, ManualTargetRejectsLargePoseJumpByLimitingIncrement) {
  TargetManager manager(config(), initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);
  Pose requested = initialTargets().left;
  requested.position.x() += 1.0;
  requested.rotation = Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  manager.setManualTarget(ArmSide::kLeft, requested);
  const Pose limited = manager.sample(0.0).left;
  EXPECT_NEAR((limited.position - initialTargets().left.position).norm(), 0.05, 1e-12);
  EXPECT_NEAR(rotationDistance(limited.rotation, initialTargets().left.rotation), 0.20, 1e-12);
}

TEST(TargetManager, UsesSourceTimestampsAndOnlyUpdatesTwistOnNewFrames) {
  QpIkConfig target_config = config();
  target_config.cartesian_servo.feedforward_filter_cutoff_hz = 15.0;
  target_config.cartesian_servo.kff_linear = 0.8;
  target_config.cartesian_servo.kff_angular = 0.8;
  target_config.cartesian_servo.prediction_horizon_seconds = 0.015;
  target_config.cartesian_servo.target_timeout_seconds = 0.075;
  target_config.safety.max_target_position_step = 1.0;
  target_config.safety.max_target_orientation_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);

  Pose first = initialTargets().left;
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, first, 10.0, 0.0));
  EXPECT_TRUE(manager.sample(0.0).left_twist.isZero(1e-12));

  Pose second = first;
  second.position.x() += 0.01;
  second.rotation =
      Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY()).toRotationMatrix();
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, second, 10.01, 0.01));
  const double alpha =
      1.0 - std::exp(-2.0 * 3.14159265358979323846 * 15.0 * 0.01);
  const Vec6 expected =
      (Vec6() << alpha * 1.0, 0.0, 0.0, 0.0, alpha * 2.0, 0.0).finished();

  const DualArmTargets on_frame = manager.sample(0.01);
  EXPECT_TRUE(on_frame.left_twist.isApprox(expected, 1e-12));

  const DualArmTargets between_frames = manager.sample(0.015);
  EXPECT_TRUE(between_frames.left_twist.isApprox(expected, 1e-12));
  EXPECT_TRUE(between_frames.left.position.isApprox(
      second.position + expected.head<3>() * 0.005, 1e-12));

  const DualArmTargets beyond_prediction = manager.sample(0.030);
  EXPECT_TRUE(beyond_prediction.left_twist.isApprox(expected, 1e-12));
  EXPECT_TRUE(beyond_prediction.left.position.isApprox(
      second.position + expected.head<3>() * 0.015, 1e-12));

  EXPECT_FALSE(manager.setManualTarget(ArmSide::kLeft, first, 10.005, 0.031));
  EXPECT_TRUE(manager.sample(0.031).left.position.isApprox(
      beyond_prediction.left.position, 1e-12));
}

TEST(TargetManager, BilateralManualTargetsCommitAtomically) {
  QpIkConfig target_config = config();
  target_config.cartesian_otg.enabled = true;
  target_config.safety.max_target_position_step = 1.0;
  target_config.safety.max_target_orientation_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);

  DualArmTargets requested = initialTargets();
  requested.left.position.x() += 0.01;
  requested.right.position.z() -= 0.02;
  ASSERT_TRUE(manager.setManualTargets(requested.left, requested.right,
                                       10.0, 0.0));
  const DualArmTargets accepted = manager.sample(0.0);
  EXPECT_TRUE(accepted.left.position.isApprox(requested.left.position));
  EXPECT_TRUE(accepted.right.position.isApprox(requested.right.position));

  Pose newer_left = requested.left;
  newer_left.position.y() += 0.03;
  Pose invalid_right = requested.right;
  invalid_right.rotation(0, 0) = 2.0;
  EXPECT_FALSE(manager.setManualTargets(newer_left, invalid_right,
                                        10.014, 0.014));
  const DualArmTargets after_invalid = manager.sample(0.014);
  EXPECT_TRUE(after_invalid.left.position.isApprox(accepted.left.position));
  EXPECT_TRUE(after_invalid.right.position.isApprox(accepted.right.position));

  Pose newer_right = requested.right;
  newer_right.position.y() -= 0.03;
  EXPECT_FALSE(manager.setManualTargets(newer_left, newer_right, 10.0, 0.015));
  const DualArmTargets after_duplicate = manager.sample(0.015);
  EXPECT_TRUE(after_duplicate.left.position.isApprox(accepted.left.position));
  EXPECT_TRUE(after_duplicate.right.position.isApprox(accepted.right.position));
}

TEST(TargetManager, BilateralTwistUsesCommonSourceFrameDelta) {
  QpIkConfig target_config = config();
  target_config.cartesian_otg.enabled = true;
  target_config.cartesian_servo.feedforward_filter_cutoff_hz = 1.0e6;
  target_config.safety.max_target_position_step = 1.0;
  target_config.safety.max_target_orientation_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);

  const DualArmTargets first = initialTargets();
  ASSERT_TRUE(manager.setManualTargets(first.left, first.right, 20.0, 0.0));
  DualArmTargets second = first;
  second.left.position.x() += 0.014;
  second.right.position.y() -= 0.028;
  second.left.rotation =
      Eigen::AngleAxisd(0.007, Eigen::Vector3d::UnitY()).toRotationMatrix();
  second.right.rotation =
      Eigen::AngleAxisd(0.014, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  ASSERT_TRUE(manager.setManualTargets(second.left, second.right,
                                       20.014, 0.014));

  const DualArmTargets output = manager.sample(0.014);
  EXPECT_NEAR(output.left_twist.x(), 1.0, 1e-9);
  EXPECT_NEAR(output.left_twist(4), 0.5, 1e-9);
  EXPECT_NEAR(output.right_twist.y(), -2.0, 1e-9);
  EXPECT_NEAR(output.right_twist(5), 1.0, 1e-9);
}

TEST(TargetManager, BilateralPredictionUsesAccelerationFromNewSourceFrames) {
  QpIkConfig target_config = config();
  target_config.cartesian_otg.enabled = true;
  target_config.cartesian_otg.translation_prediction_enabled = true;
  target_config.cartesian_servo.feedforward_filter_cutoff_hz = 1.0e6;
  target_config.safety.max_target_position_step = 1.0;
  target_config.safety.max_target_orientation_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);

  const DualArmTargets first = initialTargets();
  ASSERT_TRUE(manager.setManualTargets(first.left, first.right, 25.0, 0.0));
  DualArmTargets second = first;
  second.left.position.x() += 0.00005;
  second.right.position.y() -= 0.0001;
  ASSERT_TRUE(manager.setManualTargets(second.left, second.right,
                                       25.01, 0.01));

  const DualArmTargets between_frames = manager.sample(0.015);
  EXPECT_NEAR(between_frames.left.position.x(),
              second.left.position.x() + 0.005 * 0.005 +
                  0.5 * 0.5 * 0.005 * 0.005,
              1e-10);
  EXPECT_NEAR(between_frames.right.position.y(),
              second.right.position.y() - 0.01 * 0.005 -
                  0.5 * 1.0 * 0.005 * 0.005,
              1e-10);
  EXPECT_NEAR(between_frames.left_twist.x(), 0.0075, 1e-8);
  EXPECT_NEAR(between_frames.right_twist.y(), -0.015, 1e-8);

  const DualArmTargets after_prediction_horizon = manager.sample(0.040);
  EXPECT_NEAR(after_prediction_horizon.left.position.x(),
              second.left.position.x() + 0.005 * 0.015 +
                  0.5 * 0.5 * 0.015 * 0.015,
              1e-10);
  EXPECT_NEAR(after_prediction_horizon.right.position.y(),
              second.right.position.y() - 0.01 * 0.015 -
                  0.5 * 1.0 * 0.015 * 0.015,
              1e-10);
  EXPECT_NEAR(after_prediction_horizon.left_twist.x(), 0.0125, 1e-8);
  EXPECT_NEAR(after_prediction_horizon.right_twist.y(), -0.025, 1e-8);
}

TEST(TargetManager, BilateralTargetsBecomeStaleAtTimeoutBoundary) {
  QpIkConfig target_config = config();
  target_config.cartesian_otg.enabled = true;
  target_config.cartesian_servo.feedforward_filter_cutoff_hz = 1.0e6;
  target_config.cartesian_servo.target_timeout_seconds = 0.050;
  target_config.safety.max_target_position_step = 1.0;
  target_config.safety.max_target_orientation_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);

  DualArmTargets first = initialTargets();
  ASSERT_TRUE(manager.setManualTargets(first.left, first.right,
                                       30.0, -0.014));
  DualArmTargets latest = first;
  latest.left.position.x() += 0.014;
  latest.right.position.x() += 0.014;
  ASSERT_TRUE(manager.setManualTargets(latest.left, latest.right,
                                       30.014, 0.0));

  const DualArmTargets boundary = manager.sample(0.050);
  EXPECT_TRUE(boundary.left.position.isApprox(latest.left.position));
  EXPECT_TRUE(boundary.right.position.isApprox(latest.right.position));
  EXPECT_TRUE(boundary.left_twist.isZero(1e-12));
  EXPECT_TRUE(boundary.right_twist.isZero(1e-12));
  EXPECT_TRUE(boundary.left_stale);
  EXPECT_TRUE(boundary.right_stale);
}

TEST(TargetManager, StaleManualTargetsStopAdvancingTowardUnpublishedJump) {
  QpIkConfig target_config = config();
  target_config.cartesian_otg.enabled = true;
  target_config.cartesian_servo.target_timeout_seconds = 0.050;
  target_config.safety.max_target_position_step = 0.05;
  target_config.safety.max_target_orientation_step = 0.20;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);

  DualArmTargets requested = initialTargets();
  requested.left.position.x() += 0.50;
  requested.right.position.x() += 0.50;
  requested.left.rotation =
      Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  requested.right.rotation = requested.left.rotation;
  ASSERT_TRUE(manager.setManualTargets(requested.left, requested.right,
                                       40.0, 0.0));

  const DualArmTargets first = manager.sample(0.0);
  ASSERT_FALSE(first.left_stale);
  ASSERT_NEAR((first.left.position - initialTargets().left.position).norm(),
              0.05, 1e-12);
  ASSERT_NEAR(rotationDistance(first.left.rotation,
                               initialTargets().left.rotation),
              0.20, 1e-12);

  const DualArmTargets stale = manager.sample(0.050);
  EXPECT_TRUE(stale.left_stale);
  EXPECT_TRUE(stale.right_stale);
  EXPECT_TRUE(stale.left.position.isApprox(first.left.position, 1e-12));
  EXPECT_TRUE(stale.right.position.isApprox(first.right.position, 1e-12));
  EXPECT_TRUE(stale.left.rotation.isApprox(first.left.rotation, 1e-12));
  EXPECT_TRUE(stale.right.rotation.isApprox(first.right.rotation, 1e-12));
  EXPECT_TRUE(stale.left_twist.isZero(1e-12));
  EXPECT_TRUE(stale.right_twist.isZero(1e-12));
}

TEST(TargetManager, TimesOutStaleManualTargetVelocity) {
  QpIkConfig target_config = config();
  target_config.cartesian_servo.feedforward_filter_cutoff_hz = 1.0e6;
  target_config.cartesian_servo.kff_linear = 0.8;
  target_config.cartesian_servo.kff_angular = 0.8;
  target_config.cartesian_servo.prediction_horizon_seconds = 0.015;
  target_config.cartesian_servo.target_timeout_seconds = 0.075;
  target_config.safety.max_target_position_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, initialTargets().left,
                                      20.0, 0.0));
  Pose moved = initialTargets().left;
  moved.position.x() += 0.01;
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, moved, 20.01, 0.01));

  const DualArmTargets stale = manager.sample(0.09);
  EXPECT_TRUE(stale.left_twist.isZero(1e-12));
  EXPECT_TRUE(stale.left_stale);
  EXPECT_FALSE(stale.right_stale);
  EXPECT_TRUE(stale.left.position.isApprox(moved.position, 1e-12));

  manager.setMode(TargetMode::kHold, 0.09);
  const DualArmTargets held = manager.sample(0.095);
  EXPECT_TRUE(held.left_twist.isZero(1e-12));
  EXPECT_TRUE(held.right_twist.isZero(1e-12));
}

TEST(TargetManager, ZeroFeedforwardKeepsManualTargetUnpredicted) {
  QpIkConfig target_config = config();
  target_config.cartesian_servo.kff_linear = 0.0;
  target_config.cartesian_servo.kff_angular = 0.0;
  target_config.safety.max_target_position_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, initialTargets().left,
                                      30.0, 0.0));
  Pose moved = initialTargets().left;
  moved.position.x() += 0.01;
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, moved, 30.01, 0.01));

  const DualArmTargets target = manager.sample(0.015);
  EXPECT_TRUE(target.left.position.isApprox(moved.position, 1e-12));
  EXPECT_TRUE(target.left_twist.isZero(1e-12));
}

TEST(TargetManager, OtgReceivesTimestampedTwistWithoutTargetPrediction) {
  QpIkConfig target_config = config();
  target_config.cartesian_otg.enabled = true;
  target_config.cartesian_servo.kff_linear = 0.0;
  target_config.cartesian_servo.kff_angular = 0.0;
  target_config.cartesian_servo.feedforward_filter_cutoff_hz = 1.0e6;
  target_config.safety.max_target_position_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, initialTargets().left,
                                      40.0, 0.0));
  Pose moved = initialTargets().left;
  moved.position.x() += 0.01;
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, moved, 40.01, 0.01));

  const DualArmTargets target = manager.sample(0.015);
  EXPECT_TRUE(target.left.position.isApprox(moved.position, 1e-12));
  EXPECT_TRUE(target.left_twist.head<3>().isApprox(
      Eigen::Vector3d(1.0, 0.0, 0.0), 1e-8));
}

TEST(TargetManager, TranslationPositionModeUsesRawPoseAndPreservesIntentTwist) {
  QpIkConfig target_config = config();
  target_config.cartesian_otg.enabled = true;
  target_config.cartesian_otg.translation_position_mode = true;
  target_config.cartesian_otg.translation_prediction_enabled = true;
  target_config.cartesian_servo.feedforward_filter_cutoff_hz = 1.0e6;
  target_config.safety.max_target_position_step = 1.0;
  target_config.safety.max_target_orientation_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);

  const Pose first = initialTargets().left;
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, first, 45.0, 0.0));
  Pose second = first;
  second.position.x() += 0.01;
  second.rotation =
      Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY()).toRotationMatrix();
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, second, 45.01, 0.01));

  const DualArmTargets between_frames = manager.sample(0.015);
  EXPECT_TRUE(between_frames.left.position.isApprox(second.position, 1e-12));
  EXPECT_NEAR(between_frames.left_twist.x(), 1.0, 1e-8);
  EXPECT_NEAR(between_frames.left_twist(4), 2.0, 1e-8);
}

TEST(TargetManager, EstimatesAccelerationOnlyOnNewSourceFrames) {
  QpIkConfig target_config = config();
  target_config.cartesian_otg.enabled = true;
  target_config.cartesian_otg.translation_prediction_enabled = true;
  target_config.cartesian_servo.feedforward_filter_cutoff_hz = 1.0e6;
  target_config.safety.max_target_position_step = 1.0;
  TargetManager manager(target_config, initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);

  Pose frame = initialTargets().left;
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, frame, 50.0, 0.0));
  EXPECT_TRUE(manager.sample(0.0).left_twist.isZero(1e-12));

  frame.position.x() += 0.00005;
  ASSERT_TRUE(manager.setManualTarget(ArmSide::kLeft, frame, 50.01, 0.01));
  const DualArmTargets on_frame = manager.sample(0.01);
  EXPECT_NEAR(on_frame.left_twist.x(), 0.005, 1e-10);

  const DualArmTargets between_frames = manager.sample(0.015);
  EXPECT_NEAR(between_frames.left.position.x(),
              frame.position.x() + 0.005 * 0.005 +
                  0.5 * 0.5 * 0.005 * 0.005,
              1e-10);
  EXPECT_NEAR(between_frames.left_twist.x(), 0.0075, 1e-8);
}

TEST(TargetManager, LimitsScriptedJumpsAndHoldCapturesLastPublishedTarget) {
  TargetManager manager(config(), initialTargets());
  manager.setMode(TargetMode::kCombined, 0.0);
  const DualArmTargets limited = manager.sample(2.5);
  EXPECT_LE((limited.left.position - initialTargets().left.position).norm(), 0.05 + 1e-12);
  EXPECT_LE(rotationDistance(limited.left.rotation, initialTargets().left.rotation),
            0.20 + 1e-12);

  manager.setMode(TargetMode::kHold, 2.5);
  const DualArmTargets held = manager.sample(100.0);
  EXPECT_TRUE(held.left.position.isApprox(limited.left.position, 1e-12));
  EXPECT_TRUE(held.left.rotation.isApprox(limited.left.rotation, 1e-12));
  EXPECT_TRUE(held.right.position.isApprox(limited.right.position, 1e-12));
  EXPECT_TRUE(held.right.rotation.isApprox(limited.right.rotation, 1e-12));
}

TEST(TargetManager, InvalidManualTargetReachesSafetyPathAndValidCommandRecovers) {
  TargetManager manager(config(), initialTargets());
  manager.setMode(TargetMode::kManual, 0.0);
  Pose invalid = initialTargets().left;
  invalid.rotation(0, 0) = 2.0;
  EXPECT_NO_THROW(manager.setManualTarget(ArmSide::kLeft, invalid));
  const DualArmTargets rejected = manager.sample(0.001);
  EXPECT_FALSE(isProperRotation(rejected.left.rotation));

  Pose recovered = initialTargets().left;
  recovered.position.x() += 0.01;
  EXPECT_NO_THROW(manager.setManualTarget(ArmSide::kLeft, recovered));
  const DualArmTargets accepted = manager.sample(0.002);
  EXPECT_TRUE(isProperRotation(accepted.left.rotation));
  EXPECT_TRUE(accepted.left.position.isApprox(recovered.position, 1e-12));
}

}  // namespace
}  // namespace tianji_qp_ik
