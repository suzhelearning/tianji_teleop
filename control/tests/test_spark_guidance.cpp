#include "tianji_qp_ik/spark_guidance.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>

namespace tianji_qp_ik {
namespace {

constexpr const char* kModelPath =
    TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_qp_test.xml";
constexpr const char* kWuji2ModelPath =
    TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_wuji2.xml";
constexpr const char* kUrdfPath =
    TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_s_ccs_696_v4_local.urdf";

ArmMotionState modelState(MujocoRobot& robot, ArmSide side) {
  ArmMotionState state;
  state.q = robot.armPosition(side);
  state.qdot.setZero();
  state.qddot.setZero();
  return state;
}

PicoTeleopFrame frameFromRobot(MujocoRobot& robot, const Vec7& left_q,
                               const Vec7& right_q) {
  const ArmKinematicSample left =
      robot.armKinematicsAt(ArmSide::kLeft, left_q);
  const ArmKinematicSample right =
      robot.armKinematicsAt(ArmSide::kRight, right_q);
  PicoTeleopFrame frame;
  frame.sequence = 1;
  frame.tracking_epoch = 1;
  frame.upper_limb_skeleton.valid = true;
  frame.upper_limb_skeleton.points = {
      left.shoulder_position, left.elbow_position, left.wrist_position,
      left.tcp_pose.position, right.shoulder_position, right.elbow_position,
      right.wrist_position, right.tcp_pose.position};
  frame.left = left.tcp_pose;
  frame.right = right.tcp_pose;
  return frame;
}

SparkUpperArmTarget sparkTargetFromSample(const ArmKinematicSample& sample) {
  SparkUpperArmTarget target;
  target.palm = sample.tcp_pose;
  target.shoulder = sample.shoulder_position;
  target.elbow = sample.elbow_position;
  target.wrist = sample.wrist_position;
  target.hand = sample.tcp_pose.position;
  return target;
}

TEST(SparkGuidance, ProducesCartesianTargetsAndSoftPostureWithoutCommandingRobot) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q = 0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
                            robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q = 0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
                             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  const ArmKinematicSample left = robot.armKinematicsAt(ArmSide::kLeft, left_q);
  const ArmKinematicSample right = robot.armKinematicsAt(ArmSide::kRight, right_q);

  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  for (int joint = 0; joint < kArmDof; ++joint) {
    ASSERT_GE(left_q[joint],
              robot.mapping(ArmSide::kLeft).limits.lower_position[joint] +
                  config.spark_upper_qpoases.joint_limit_margin_rad)
        << "left joint " << joint;
    ASSERT_LE(left_q[joint],
              robot.mapping(ArmSide::kLeft).limits.upper_position[joint] -
                  config.spark_upper_qpoases.joint_limit_margin_rad)
        << "left joint " << joint;
    ASSERT_GE(right_q[joint],
              robot.mapping(ArmSide::kRight).limits.lower_position[joint] +
                  config.spark_upper_qpoases.joint_limit_margin_rad)
        << "right joint " << joint;
    ASSERT_LE(right_q[joint],
              robot.mapping(ArmSide::kRight).limits.upper_position[joint] -
                  config.spark_upper_qpoases.joint_limit_margin_rad)
        << "right joint " << joint;
  }
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  DualArmSparkGuidance guidance(robot, config, kUrdfPath);
  PicoTeleopFrame frame;
  frame.sequence = 1;
  frame.tracking_epoch = 1;
  frame.upper_limb_skeleton.valid = true;
  frame.upper_limb_skeleton.points = {
      left.shoulder_position, left.elbow_position, left.wrist_position,
      left.tcp_pose.position, right.shoulder_position, right.elbow_position,
      right.wrist_position, right.tcp_pose.position};
  frame.left = left.tcp_pose;
  frame.right = right.tcp_pose;
  ASSERT_TRUE(guidance.updatePicoFrame(frame).valid);

  const SparkGuidanceDiagnostics result = guidance.step(
      modelState(robot, ArmSide::kLeft),
      modelState(robot, ArmSide::kRight), 0.005);
  EXPECT_GT((result.left.target.elbow - result.left.target.shoulder).norm(),
            0.05);
  EXPECT_GT((result.left.target.wrist - result.left.target.elbow).norm(),
            0.05);
  EXPECT_GT((result.right.target.elbow - result.right.target.shoulder).norm(),
            0.05);
  EXPECT_GT((result.right.target.wrist - result.right.target.elbow).norm(),
            0.05);
  ASSERT_TRUE(result.accepted) << result.detail << " left="
                               << result.left.ik.detail << " right="
                               << result.right.ik.detail;
  EXPECT_TRUE(result.target_valid);
  EXPECT_TRUE(result.posture_tasks.left.active);
  EXPECT_TRUE(result.posture_tasks.right.active);
  EXPECT_EQ(result.posture_tasks.left.source,
            JointVelocityPostureSource::kSparkSoftQp);
  EXPECT_DOUBLE_EQ(result.posture_tasks.left.weight,
                   config.spark_upper_qpoases.posture_weight);
  EXPECT_TRUE(result.cartesian_targets.left.position.isApprox(
      result.left.target.hand));
  EXPECT_TRUE(result.cartesian_targets.right.position.isApprox(
      result.right.target.hand));
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left_q));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_q));
}

TEST(SparkGuidance, UsesSelectedHandTcpForHand2PalmTarget) {
  MujocoRobot robot(kWuji2ModelPath);
  const Vec7 left_q = Vec7::Zero();
  const Vec7 right_q = Vec7::Zero();

  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  DualArmSparkGuidance guidance(robot, config, kUrdfPath,
                                SparkPostureGuideMode::kDisabled);

  const SparkUpperTargets targets =
      guidance.updatePicoFrame(frameFromRobot(robot, left_q, right_q));
  ASSERT_TRUE(targets.valid) << targets.detail;

  const ArmKinematicSample left =
      robot.armKinematicsAt(ArmSide::kLeft, left_q);
  const ArmKinematicSample right =
      robot.armKinematicsAt(ArmSide::kRight, right_q);
  constexpr double kTolerance = 1.0e-5;
  EXPECT_TRUE(targets.left.hand.isApprox(left.tcp_pose.position, kTolerance));
  EXPECT_TRUE(targets.right.hand.isApprox(right.tcp_pose.position,
                                          kTolerance));
  EXPECT_TRUE(targets.left.palm.position.isApprox(left.tcp_pose.position,
                                                  kTolerance));
  EXPECT_TRUE(targets.right.palm.position.isApprox(right.tcp_pose.position,
                                                   kTolerance));
  EXPECT_TRUE(targets.left.palm.rotation.isApprox(left.tcp_pose.rotation,
                                                  kTolerance));
  EXPECT_TRUE(targets.right.palm.rotation.isApprox(right.tcp_pose.rotation,
                                                   kTolerance));
}

TEST(SparkGuidance, JointSpaceTakeoverUsesBoundedRuckigReference) {
  MujocoRobot robot(kModelPath);
  const ArmLimits& left_limits = robot.mapping(ArmSide::kLeft).limits;
  const ArmLimits& right_limits = robot.mapping(ArmSide::kRight).limits;
  const Vec7 left_q =
      0.5 * (left_limits.lower_position + left_limits.upper_position);
  const Vec7 right_q =
      0.5 * (right_limits.lower_position + right_limits.upper_position);
  Vec7 left_goal = left_q;
  Vec7 right_goal = right_q;
  left_goal[1] += 0.20;
  right_goal[1] -= 0.20;

  const ArmKinematicSample left_goal_sample =
      robot.armKinematicsAt(ArmSide::kLeft, left_goal);
  const ArmKinematicSample right_goal_sample =
      robot.armKinematicsAt(ArmSide::kRight, right_goal);
  SparkUpperTargets target;
  target.valid = true;
  target.left = sparkTargetFromSample(left_goal_sample);
  target.right = sparkTargetFromSample(right_goal_sample);

  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  DualArmSparkGuidance guidance(robot, config, kUrdfPath);
  ArmMotionState left_model;
  left_model.q = left_q;
  ArmMotionState right_model;
  right_model.q = right_q;

  ASSERT_TRUE(
      guidance.startJointSpaceTakeover(target, left_model, right_model));
  const SparkGuidanceDiagnostics first =
      guidance.stepJointSpaceTakeover(left_model, right_model, 0.005);

  ASSERT_TRUE(first.accepted) << first.detail;
  EXPECT_TRUE(first.joint_takeover_active);
  EXPECT_TRUE(first.left.ik.accepted);
  EXPECT_TRUE(first.right.ik.accepted);
  EXPECT_TRUE(first.cartesian_references_valid);
  EXPECT_EQ(first.posture_tasks.left.source,
            JointVelocityPostureSource::kSparkJointReference);
  EXPECT_EQ(first.posture_tasks.right.source,
            JointVelocityPostureSource::kSparkJointReference);
  EXPECT_TRUE(first.left.reference.state.q.allFinite());
  EXPECT_TRUE(first.right.reference.state.q.allFinite());
  EXPECT_LT((first.left.reference.state.q - left_goal).norm(),
            (left_q - left_goal).norm());
  EXPECT_LT((first.right.reference.state.q - right_goal).norm(),
            (right_q - right_goal).norm());
}

TEST(SparkGuidance, DirectModeBypassesRuckigAndEmitsBoundedSoftPosture) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q = 0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
                            robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q = 0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
                             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath, SparkPostureGuideMode::kDirect);
  ASSERT_TRUE(guidance.updatePicoFrame(
      frameFromRobot(robot, left_q, right_q)).valid);

  const SparkGuidanceDiagnostics result = guidance.step(
      modelState(robot, ArmSide::kLeft),
      modelState(robot, ArmSide::kRight), 0.005);

  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_TRUE(result.left.ik.accepted);
  EXPECT_TRUE(result.right.ik.accepted);
  EXPECT_FALSE(result.left.reference.accepted);
  EXPECT_FALSE(result.right.reference.accepted);
  EXPECT_TRUE(result.posture_tasks.left.active);
  EXPECT_TRUE(result.posture_tasks.right.active);
  EXPECT_EQ(result.posture_tasks.left.source,
            JointVelocityPostureSource::kSparkSoftQp);
  EXPECT_TRUE(result.posture_tasks.left.target.allFinite());
  EXPECT_TRUE(result.posture_tasks.right.target.allFinite());
  EXPECT_TRUE((result.posture_tasks.left.target.cwiseAbs().array() <=
               robot.mapping(ArmSide::kLeft).limits.velocity.array()).all());
  EXPECT_TRUE((result.posture_tasks.right.target.cwiseAbs().array() <=
               robot.mapping(ArmSide::kRight).limits.velocity.array()).all());
  EXPECT_DOUBLE_EQ(result.posture_tasks.left.weight,
                   config.spark_upper_qpoases.posture_weight);
}

TEST(SparkGuidance,
     JointReferenceVelocityModeSmoothlyAttacksAndReleasesLastAcceptedIk) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q = 0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
                            robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q = 0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
                             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_upper_qpoases.joint_reference_attack_seconds = 0.020;
  config.spark_upper_qpoases.joint_reference_release_seconds = 0.020;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kJointReferenceVelocity);
  ASSERT_TRUE(guidance.updatePicoFrame(
      frameFromRobot(robot, left_q, right_q)).valid);

  const ArmMotionState left = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right = modelState(robot, ArmSide::kRight);
  const SparkGuidanceDiagnostics attack = guidance.step(left, right, 0.005);
  ASSERT_TRUE(attack.accepted) << attack.detail;
  EXPECT_TRUE(attack.posture_tasks.left.active);
  EXPECT_GT(attack.posture_tasks.left.activation, 0.0);
  EXPECT_LT(attack.posture_tasks.left.activation, 1.0);
  EXPECT_EQ(attack.posture_tasks.left.source,
            JointVelocityPostureSource::kSparkJointReference);
  EXPECT_DOUBLE_EQ(
      attack.posture_tasks.left.smoothness_weight,
      config.spark_upper_qpoases.joint_reference_smoothness_weight);
  EXPECT_NEAR(attack.cartesian_targets.left_twist.norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(attack.cartesian_targets.right_twist.norm(), 0.0, 1.0e-12);

  SparkGuidanceDiagnostics last_active = attack;
  for (int cycle = 0; cycle < 3; ++cycle) {
    last_active = guidance.step(left, right, 0.005);
    ASSERT_TRUE(last_active.accepted);
  }
  guidance.invalidateTarget("test_stale");
  const SparkGuidanceDiagnostics release = guidance.step(left, right, 0.005);
  ASSERT_TRUE(release.accepted) << release.detail;
  EXPECT_FALSE(release.target_valid);
  EXPECT_TRUE(release.posture_tasks.left.active);
  EXPECT_GT(release.posture_tasks.left.activation, 0.0);
  EXPECT_LT(release.posture_tasks.left.activation, 1.0);
  EXPECT_TRUE(release.cartesian_targets.left.position.allFinite());
  EXPECT_TRUE(release.left.q_ik.isApprox(last_active.left.q_ik));
  EXPECT_TRUE(release.right.q_ik.isApprox(last_active.right.q_ik));

  for (int cycle = 0; cycle < 3; ++cycle) {
    (void)guidance.step(left, right, 0.005);
  }
  const SparkGuidanceDiagnostics released = guidance.step(left, right, 0.005);
  EXPECT_FALSE(released.accepted);
  EXPECT_FALSE(released.posture_tasks.left.active);
}

TEST(SparkGuidance,
     OtgConsistentModeEmitsOneReferenceSharedByPositionIkAndVelocityQp) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_upper_qpoases.ik_cycle_budget_seconds = 0.1;
  config.spark_upper_qpoases.joint_reference_attack_seconds = 0.020;
  config.spark_upper_qpoases.joint_reference_release_seconds = 0.020;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kOtgConsistentJointReferenceVelocity);

  Vec7 left_goal = left_q;
  Vec7 right_goal = right_q;
  left_goal[1] += 0.15;
  right_goal[1] -= 0.15;
  ASSERT_TRUE(
      guidance.updatePicoFrame(frameFromRobot(robot, left_goal, right_goal))
          .valid);
  const SparkGuidanceDiagnostics result = guidance.step(
      modelState(robot, ArmSide::kLeft),
      modelState(robot, ArmSide::kRight), 0.005);

  ASSERT_TRUE(result.accepted) << result.detail << " left="
                               << result.left.ik.detail << " right="
                               << result.right.ik.detail;
  ASSERT_TRUE(result.cartesian_references_valid);
  EXPECT_TRUE(result.cartesian_references.left.valid);
  EXPECT_TRUE(result.cartesian_references.right.valid);
  EXPECT_TRUE(result.posture_tasks.left.active);
  EXPECT_EQ(result.posture_tasks.left.source,
            JointVelocityPostureSource::kSparkJointReference);
  const ArmKinematicSample left_ik =
      robot.armKinematicsAt(ArmSide::kLeft, result.left.q_ik);
  const ArmKinematicSample right_ik =
      robot.armKinematicsAt(ArmSide::kRight, result.right.q_ik);
  const Vec6 left_error = poseErrorWorld(
      result.cartesian_references.left.pose, left_ik.tcp_pose);
  const Vec6 right_error = poseErrorWorld(
      result.cartesian_references.right.pose, right_ik.tcp_pose);
  EXPECT_LT(left_error.head<3>().norm(),
            config.spark_upper_qpoases.otg_position_tolerance_m);
  EXPECT_LT(left_error.tail<3>().norm(),
            config.spark_upper_qpoases.otg_orientation_tolerance_rad);
  EXPECT_LT(right_error.head<3>().norm(),
            config.spark_upper_qpoases.otg_position_tolerance_m);
  EXPECT_LT(right_error.tail<3>().norm(),
            config.spark_upper_qpoases.otg_orientation_tolerance_rad);

  ASSERT_TRUE(guidance.step(modelState(robot, ArmSide::kLeft),
                            modelState(robot, ArmSide::kRight), 0.005)
                  .accepted);
  guidance.invalidateTarget("test_stale");
  const SparkGuidanceDiagnostics stopping = guidance.step(
      modelState(robot, ArmSide::kLeft),
      modelState(robot, ArmSide::kRight), 0.005);
  ASSERT_TRUE(stopping.accepted) << stopping.detail;
  EXPECT_TRUE(stopping.cartesian_references_valid);
  EXPECT_TRUE(stopping.cartesian_references.left.stale);
  EXPECT_TRUE(stopping.cartesian_references.right.stale);
}

TEST(SparkGuidance, PoseModeSkipsPositionIkAndEmitsNoPostureTask) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q = 0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
                            robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q = 0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
                             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath, SparkPostureGuideMode::kDisabled);
  ASSERT_TRUE(guidance.updatePicoFrame(
      frameFromRobot(robot, left_q, right_q)).valid);

  const SparkGuidanceDiagnostics result = guidance.step(
      modelState(robot, ArmSide::kLeft),
      modelState(robot, ArmSide::kRight), 0.005);

  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_TRUE(result.target_valid);
  EXPECT_FALSE(result.left.ik.accepted);
  EXPECT_FALSE(result.right.ik.accepted);
  EXPECT_FALSE(result.posture_tasks.left.active);
  EXPECT_FALSE(result.posture_tasks.right.active);
  EXPECT_TRUE(result.cartesian_targets.left.position.allFinite());
  EXPECT_TRUE(result.cartesian_targets.right.position.allFinite());
}

TEST(SparkGuidance,
     FeedforwardModeTracksLatestSparkPalmWithSameSourceTwistAndPosture) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();

  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_upper_qpoases.ik_cycle_budget_seconds = 0.1;
  config.spark_feedforward_velocity_qp.attack_seconds = 0.020;
  config.spark_feedforward_velocity_qp.release_seconds = 0.020;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kFeedforwardJointReferenceVelocity);

  PicoTeleopFrame first = frameFromRobot(robot, left_q, right_q);
  first.source_timestamp_ns = 1'000'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(first).valid);
  const ArmMotionState left_model = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right_model = modelState(robot, ArmSide::kRight);
  ASSERT_TRUE(guidance.step(left_model, right_model, 0.005).accepted);

  Vec7 left_goal = left_q;
  Vec7 right_goal = right_q;
  left_goal[1] += 0.08;
  right_goal[1] -= 0.08;
  PicoTeleopFrame second = frameFromRobot(robot, left_goal, right_goal);
  second.sequence = 2;
  second.source_timestamp_ns = 1'011'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(second).valid);
  const SparkGuidanceDiagnostics result =
      guidance.step(left_model, right_model, 0.005);

  ASSERT_TRUE(result.accepted) << result.detail << " left="
                               << result.left.ik.detail << " right="
                               << result.right.ik.detail;
  ASSERT_TRUE(result.feedforward_references_valid);
  ASSERT_TRUE(result.left.feedforward.valid);
  ASSERT_TRUE(result.right.feedforward.valid);
  EXPECT_GT(result.cartesian_targets.left_twist.norm(), 0.0);
  EXPECT_GT(result.cartesian_targets.right_twist.norm(), 0.0);
  EXPECT_TRUE(result.posture_tasks.left.active);
  EXPECT_EQ(result.posture_tasks.left.source,
            JointVelocityPostureSource::kSparkFeedforwardJointReference);
  EXPECT_DOUBLE_EQ(result.posture_tasks.left.weight,
                   config.spark_upper_qpoases.joint_reference_weight);
  EXPECT_DOUBLE_EQ(
      result.posture_tasks.left.smoothness_weight,
      config.spark_upper_qpoases.joint_reference_smoothness_weight);
  EXPECT_DOUBLE_EQ(
      result.posture_tasks.left.jerk_smoothness_weight,
      0.0);

  EXPECT_TRUE(result.cartesian_targets.left.position.isApprox(
      result.left.target.palm.position, 1.0e-9));
  EXPECT_TRUE(result.cartesian_targets.right.position.isApprox(
      result.right.target.palm.position, 1.0e-9));
  EXPECT_TRUE(result.cartesian_targets.left.rotation.isApprox(
      result.left.target.palm.rotation, 1.0e-9));
  EXPECT_TRUE(result.cartesian_targets.right.rotation.isApprox(
      result.right.target.palm.rotation, 1.0e-9));
  EXPECT_GT(result.cartesian_targets.left_twist.head<3>().dot(
                result.left.target.palm.position - first.left.position),
            0.0);
  EXPECT_GT(result.cartesian_targets.right_twist.head<3>().dot(
                result.right.target.palm.position - first.right.position),
            0.0);

  const SparkGuidanceDiagnostics held_twist =
      guidance.step(left_model, right_model, 0.005);
  ASSERT_TRUE(held_twist.accepted) << held_twist.detail;
  EXPECT_TRUE(held_twist.left.palm_twist.twist.isApprox(
      result.left.palm_twist.twist, 1.0e-12));
  EXPECT_TRUE(held_twist.right.palm_twist.twist.isApprox(
      result.right.palm_twist.twist, 1.0e-12));
  EXPECT_TRUE(held_twist.cartesian_targets.left.position.isApprox(
      result.left.target.palm.position, 1.0e-12));

  const Vec7 expected_left_posture =
      result.left.feedforward.qdot +
      config.spark_feedforward_velocity_qp.joint_position_gain *
          (result.left.feedforward.q - left_model.q);
  const Vec7 expected_left_bounded =
      expected_left_posture
          .cwiseMax(-robot.mapping(ArmSide::kLeft).limits.velocity)
          .cwiseMin(robot.mapping(ArmSide::kLeft).limits.velocity);
  EXPECT_LT((result.posture_tasks.left.target - expected_left_bounded).norm(),
            1.0e-9)
      << "actual=" << result.posture_tasks.left.target.transpose()
      << " expected=" << expected_left_bounded.transpose();

  guidance.invalidateTarget("test_stale");
  const SparkGuidanceDiagnostics stopping =
      guidance.step(left_model, right_model, 0.005);
  ASSERT_TRUE(stopping.accepted) << stopping.detail;
  EXPECT_EQ(stopping.left.feedforward.state,
            SparkFeedforwardState::kStopping);
  EXPECT_TRUE(stopping.cartesian_references.left.stale);
  EXPECT_LT(stopping.posture_tasks.left.activation,
            held_twist.posture_tasks.left.activation);
}

TEST(SparkGuidance,
     FeedforwardModeHoldsLastAcceptedReferenceOnInvalidSourceFrame) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();

  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_upper_qpoases.ik_cycle_budget_seconds = 0.1;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kFeedforwardJointReferenceVelocity);
  const ArmMotionState left_model = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right_model = modelState(robot, ArmSide::kRight);

  PicoTeleopFrame first = frameFromRobot(robot, left_q, right_q);
  first.source_timestamp_ns = 1'000'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(first).valid);
  const SparkGuidanceDiagnostics accepted =
      guidance.step(left_model, right_model, 0.005);
  ASSERT_TRUE(accepted.accepted) << accepted.detail;

  Vec7 moved_left = left_q;
  Vec7 moved_right = right_q;
  moved_left[1] += 0.05;
  moved_right[1] -= 0.05;
  PicoTeleopFrame invalid = frameFromRobot(robot, moved_left, moved_right);
  invalid.sequence = 2;
  invalid.source_timestamp_ns = 0;
  ASSERT_TRUE(guidance.updatePicoFrame(invalid).valid);
  const SparkGuidanceDiagnostics held =
      guidance.step(left_model, right_model, 0.005);

  ASSERT_TRUE(held.accepted) << held.detail;
  EXPECT_FALSE(held.left.feedforward_target.accepted);
  EXPECT_FALSE(held.right.feedforward_target.accepted);
  EXPECT_EQ(held.left.feedforward_target.detail,
            "feedforward_invalid_target");
  EXPECT_TRUE(held.left.q_ik.isApprox(accepted.left.q_ik));
  EXPECT_TRUE(held.right.q_ik.isApprox(accepted.right.q_ik));
  EXPECT_TRUE(held.feedforward_references_valid);
}

TEST(SparkGuidance,
     HeadroomFeedforwardStartsClosedAndKeepsPostureTaskActive) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_upper_qpoases.ik_cycle_budget_seconds = 0.1;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity);

  const ArmMotionState left_model = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right_model = modelState(robot, ArmSide::kRight);
  PicoTeleopFrame first = frameFromRobot(robot, left_q, right_q);
  first.source_timestamp_ns = 1'000'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(first).valid);
  ASSERT_TRUE(guidance.step(left_model, right_model, 0.005).accepted);

  Vec7 moved_left = left_q;
  Vec7 moved_right = right_q;
  moved_left[1] += 0.08;
  moved_right[1] -= 0.08;
  PicoTeleopFrame second = frameFromRobot(robot, moved_left, moved_right);
  second.sequence = 2;
  second.source_timestamp_ns = 1'011'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(second).valid);
  const auto result = guidance.step(left_model, right_model, 0.005);

  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_DOUBLE_EQ(result.left.headroom.scale, 0.0);
  EXPECT_DOUBLE_EQ(result.right.headroom.scale, 0.0);
  EXPECT_TRUE(result.posture_tasks.left.active);
  EXPECT_TRUE(result.posture_tasks.right.active);
  EXPECT_EQ(result.posture_tasks.left.source,
            JointVelocityPostureSource::kSparkFeedforwardJointReference);
  EXPECT_DOUBLE_EQ(result.posture_tasks.left.weight,
                   config.spark_upper_qpoases.joint_reference_weight);
  EXPECT_DOUBLE_EQ(
      result.posture_tasks.left.smoothness_weight,
      config.spark_headroom_feedforward_velocity_qp
          .joint_reference_smoothness_weight);
  EXPECT_DOUBLE_EQ(
      result.posture_tasks.left.jerk_smoothness_weight,
      config.spark_headroom_feedforward_velocity_qp
          .joint_reference_jerk_smoothness_weight);
}

TEST(SparkGuidance,
     HeadroomFeedforwardHoldsJointReferenceForStationaryPalmSkeletonJitter) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_upper_qpoases.ik_cycle_budget_seconds = 0.1;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity);
  const ArmMotionState left_model = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right_model = modelState(robot, ArmSide::kRight);

  PicoTeleopFrame first = frameFromRobot(robot, left_q, right_q);
  first.source_timestamp_ns = 1'000'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(first).valid);
  const auto initial = guidance.step(left_model, right_model, 0.005);
  ASSERT_TRUE(initial.accepted) << initial.detail;

  PicoTeleopFrame jittered = first;
  jittered.sequence = 2;
  jittered.source_timestamp_ns = 1'011'000'000LL;
  jittered.upper_limb_skeleton.points[1] +=
      Eigen::Vector3d(0.0, 0.05, 0.04);
  jittered.upper_limb_skeleton.points[5] +=
      Eigen::Vector3d(0.0, -0.05, 0.04);
  ASSERT_TRUE(guidance.updatePicoFrame(jittered).valid);
  const auto held = guidance.step(left_model, right_model, 0.005);
  ASSERT_TRUE(held.accepted) << held.detail;
  ASSERT_GT((held.left.ik.q - initial.left.q_ik).norm(), 1.0e-4);
  ASSERT_GT((held.right.ik.q - initial.right.q_ik).norm(), 1.0e-4);
  EXPECT_TRUE(held.left.stationary_joint_reference_held);
  EXPECT_TRUE(held.right.stationary_joint_reference_held);
  EXPECT_TRUE(held.left.q_ik.isApprox(initial.left.q_ik, 1.0e-12));
  EXPECT_TRUE(held.right.q_ik.isApprox(initial.right.q_ik, 1.0e-12));

  Vec7 moved_left = left_q;
  Vec7 moved_right = right_q;
  moved_left[1] += 0.08;
  moved_right[1] -= 0.08;
  PicoTeleopFrame moving = frameFromRobot(robot, moved_left, moved_right);
  moving.sequence = 3;
  moving.source_timestamp_ns = 1'022'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(moving).valid);
  const auto released = guidance.step(left_model, right_model, 0.005);
  ASSERT_TRUE(released.accepted) << released.detail;
  EXPECT_FALSE(released.left.stationary_joint_reference_held);
  EXPECT_FALSE(released.right.stationary_joint_reference_held);
  EXPECT_GT((released.left.q_ik - initial.left.q_ik).norm(), 1.0e-4);
  EXPECT_GT((released.right.q_ik - initial.right.q_ik).norm(), 1.0e-4);
}

TEST(SparkGuidance,
     HeadroomStationaryHoldIgnoresRawPalmPositionSpikeOutsideSparkTarget) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_upper_qpoases.ik_cycle_budget_seconds = 0.1;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity);
  const ArmMotionState left_model = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right_model = modelState(robot, ArmSide::kRight);

  PicoTeleopFrame first = frameFromRobot(robot, left_q, right_q);
  first.source_timestamp_ns = 1'000'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(first).valid);
  ASSERT_TRUE(guidance.step(left_model, right_model, 0.005).accepted);

  PicoTeleopFrame raw_spike = first;
  raw_spike.sequence = 2;
  raw_spike.source_timestamp_ns = 1'011'000'000LL;
  raw_spike.left.position += Eigen::Vector3d(0.10, 0.0, 0.0);
  raw_spike.right.position += Eigen::Vector3d(-0.10, 0.0, 0.0);
  ASSERT_TRUE(guidance.updatePicoFrame(raw_spike).valid);
  const auto held = guidance.step(left_model, right_model, 0.005);

  ASSERT_TRUE(held.accepted) << held.detail;
  EXPECT_GT(held.left.motion_intent_twist.twist.head<3>().norm(), 0.01);
  EXPECT_GT(held.right.motion_intent_twist.twist.head<3>().norm(), 0.01);
  EXPECT_LT(held.left.palm_twist.twist.head<3>().norm(), 1.0e-12);
  EXPECT_LT(held.right.palm_twist.twist.head<3>().norm(), 1.0e-12);
  EXPECT_TRUE(held.left.stationary_joint_reference_held);
  EXPECT_TRUE(held.right.stationary_joint_reference_held);
}

TEST(SparkGuidance,
     FixedFeedforwardStillAcceptsStationaryPalmSkeletonPostureUpdates) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_upper_qpoases.ik_cycle_budget_seconds = 0.1;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kFeedforwardJointReferenceVelocity);
  const ArmMotionState left_model = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right_model = modelState(robot, ArmSide::kRight);
  PicoTeleopFrame first = frameFromRobot(robot, left_q, right_q);
  first.source_timestamp_ns = 1'000'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(first).valid);
  const auto initial = guidance.step(left_model, right_model, 0.005);
  ASSERT_TRUE(initial.accepted) << initial.detail;

  PicoTeleopFrame jittered = first;
  jittered.sequence = 2;
  jittered.source_timestamp_ns = 1'011'000'000LL;
  jittered.upper_limb_skeleton.points[1] +=
      Eigen::Vector3d(0.0, 0.05, 0.04);
  jittered.upper_limb_skeleton.points[5] +=
      Eigen::Vector3d(0.0, -0.05, 0.04);
  ASSERT_TRUE(guidance.updatePicoFrame(jittered).valid);
  const auto updated = guidance.step(left_model, right_model, 0.005);
  ASSERT_TRUE(updated.accepted) << updated.detail;
  EXPECT_FALSE(updated.left.stationary_joint_reference_held);
  EXPECT_FALSE(updated.right.stationary_joint_reference_held);
  EXPECT_GT((updated.left.q_ik - initial.left.q_ik).norm(), 1.0e-4);
  EXPECT_GT((updated.right.q_ik - initial.right.q_ik).norm(), 1.0e-4);
}

TEST(SparkGuidance, HeadroomFeedbackRecoversAndKeepsArmStateIndependent) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_upper_qpoases.ik_cycle_budget_seconds = 0.1;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity);

  SparkConstraintHeadroomFeedback high;
  high.accepted = true;
  high.task_scale_position = 1.0;
  high.task_scale_orientation = 1.0;
  for (int cycle = 0; cycle < 200; ++cycle) {
    guidance.updateHeadroomFeedback(high, high, 0.005);
  }

  PicoTeleopFrame first = frameFromRobot(robot, left_q, right_q);
  first.source_timestamp_ns = 1'000'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(first).valid);
  ASSERT_TRUE(guidance.step(modelState(robot, ArmSide::kLeft),
                            modelState(robot, ArmSide::kRight), 0.005)
                  .accepted);
  Vec7 left_goal = left_q;
  Vec7 right_goal = right_q;
  left_goal[1] += 0.08;
  right_goal[1] -= 0.08;
  PicoTeleopFrame second = frameFromRobot(robot, left_goal, right_goal);
  second.sequence = 2;
  second.source_timestamp_ns = 1'011'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(second).valid);
  const auto recovered = guidance.step(
      modelState(robot, ArmSide::kLeft),
      modelState(robot, ArmSide::kRight), 0.005);
  ASSERT_TRUE(recovered.accepted) << recovered.detail;
  EXPECT_GT(recovered.left.headroom.scale, 0.99);
  EXPECT_GT(recovered.cartesian_targets.left_twist.norm(), 0.0);
  EXPECT_EQ(recovered.posture_tasks.left.source,
            JointVelocityPostureSource::kSparkFeedforwardJointReference);
  EXPECT_DOUBLE_EQ(recovered.posture_tasks.left.weight,
                   config.spark_upper_qpoases.joint_reference_weight);
  EXPECT_DOUBLE_EQ(
      recovered.posture_tasks.left.smoothness_weight,
      config.spark_headroom_feedforward_velocity_qp
          .joint_reference_smoothness_weight);
  EXPECT_DOUBLE_EQ(
      recovered.posture_tasks.left.jerk_smoothness_weight,
      config.spark_headroom_feedforward_velocity_qp
          .joint_reference_jerk_smoothness_weight);

  auto constrained = high;
  constrained.task_scale_position = 0.75;
  for (int cycle = 0; cycle < 10; ++cycle) {
    guidance.updateHeadroomFeedback(constrained, high, 0.005);
  }
  const auto split = guidance.step(modelState(robot, ArmSide::kLeft),
                                   modelState(robot, ArmSide::kRight), 0.005);
  ASSERT_TRUE(split.accepted) << split.detail;
  EXPECT_LT(split.left.headroom.scale, split.right.headroom.scale);
  EXPECT_LT(split.cartesian_targets.left_twist.norm(),
            split.cartesian_targets.right_twist.norm() * 1.5);
}

TEST(SparkGuidance, HeadroomSettledHoldEntersImmediatelyWhenTargetIsStale) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity);
  const ArmMotionState left_model = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right_model = modelState(robot, ArmSide::kRight);
  PicoTeleopFrame frame = frameFromRobot(robot, left_q, right_q);
  frame.source_timestamp_ns = 1'000'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(frame).valid);
  ASSERT_TRUE(guidance.step(left_model, right_model, 0.005).accepted);

  guidance.invalidateTarget("test_stale");
  const auto held = guidance.step(left_model, right_model, 0.005);

  ASSERT_TRUE(held.accepted) << held.detail;
  EXPECT_TRUE(held.left.settled_hold_active);
  EXPECT_TRUE(held.right.settled_hold_active);
  EXPECT_EQ(held.left.settled_hold_reason, SparkSettledHoldReason::kStale);
  EXPECT_EQ(held.right.settled_hold_reason, SparkSettledHoldReason::kStale);
  const Pose left_tcp =
      robot.armKinematicsAt(ArmSide::kLeft, left_model.q).tcp_pose;
  const Pose right_tcp =
      robot.armKinematicsAt(ArmSide::kRight, right_model.q).tcp_pose;
  EXPECT_TRUE(held.cartesian_references.left.pose.position.isApprox(
      left_tcp.position, 1.0e-12));
  EXPECT_TRUE(held.cartesian_references.left.pose.rotation.isApprox(
      left_tcp.rotation, 1.0e-12));
  EXPECT_TRUE(held.cartesian_references.right.pose.position.isApprox(
      right_tcp.position, 1.0e-12));
  EXPECT_TRUE(held.cartesian_references.right.pose.rotation.isApprox(
      right_tcp.rotation, 1.0e-12));
  EXPECT_TRUE(held.cartesian_references.left.twist.isZero(1.0e-12));
  EXPECT_TRUE(held.cartesian_references.right.twist.isZero(1.0e-12));
  EXPECT_TRUE(held.posture_tasks.left.target.isZero(1.0e-12));
  EXPECT_TRUE(held.posture_tasks.right.target.isZero(1.0e-12));
}

TEST(SparkGuidance,
     HeadroomSettledHoldRequiresStationaryExhaustedDwellPerArm) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  config.spark_headroom_feedforward_velocity_qp.settled_hold_dwell_seconds =
      0.010;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity);
  const ArmMotionState left_model = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right_model = modelState(robot, ArmSide::kRight);

  SparkConstraintHeadroomFeedback exhausted;
  exhausted.accepted = true;
  exhausted.task_scale_position = 0.75;
  exhausted.task_scale_orientation = 0.75;
  SparkConstraintHeadroomFeedback healthy;
  healthy.accepted = true;
  healthy.task_scale_position = 1.0;
  healthy.task_scale_orientation = 1.0;
  for (int cycle = 0; cycle < 200; ++cycle) {
    guidance.updateHeadroomFeedback(exhausted, healthy, 0.005);
  }

  PicoTeleopFrame frame = frameFromRobot(robot, left_q, right_q);
  for (int sample = 1; sample <= 3; ++sample) {
    frame.sequence = static_cast<std::uint64_t>(sample);
    frame.source_timestamp_ns = 1'000'000'000LL + sample * 11'000'000LL;
    ASSERT_TRUE(guidance.updatePicoFrame(frame).valid);
    const auto result = guidance.step(left_model, right_model, 0.005);
    ASSERT_TRUE(result.accepted) << result.detail;
    if (sample < 3) {
      EXPECT_FALSE(result.left.settled_hold_active);
    } else {
      EXPECT_TRUE(result.left.settled_hold_active);
      EXPECT_EQ(result.left.settled_hold_reason,
                SparkSettledHoldReason::kHeadroomExhausted);
    }
    EXPECT_FALSE(result.right.settled_hold_active);
  }
}

TEST(SparkGuidance, HeadroomSettledHoldReleasesOnlyOnCorroboratedMotion) {
  MujocoRobot robot(kModelPath);
  const Vec7 left_q =
      0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
             robot.mapping(ArmSide::kLeft).limits.upper_position);
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmPosition(ArmSide::kLeft, left_q);
  robot.setArmPosition(ArmSide::kRight, right_q);
  robot.forward();
  QpIkConfig config = loadConfig(
      TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_teleop.yaml");
  config.spark_upper_qpoases.target_blend_seconds = 0.005;
  DualArmSparkGuidance guidance(
      robot, config, kUrdfPath,
      SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity);
  const ArmMotionState left_model = modelState(robot, ArmSide::kLeft);
  const ArmMotionState right_model = modelState(robot, ArmSide::kRight);
  PicoTeleopFrame first = frameFromRobot(robot, left_q, right_q);
  first.source_timestamp_ns = 1'000'000'000LL;
  ASSERT_TRUE(guidance.updatePicoFrame(first).valid);
  ASSERT_TRUE(guidance.step(left_model, right_model, 0.005).accepted);
  guidance.invalidateTarget("test_stale");
  ASSERT_TRUE(guidance.step(left_model, right_model, 0.005)
                  .left.settled_hold_active);

  PicoTeleopFrame raw_only = first;
  raw_only.sequence = 2;
  raw_only.source_timestamp_ns = 1'011'000'000LL;
  raw_only.left.position.x() += 0.10;
  raw_only.right.position.x() += 0.10;
  ASSERT_TRUE(guidance.updatePicoFrame(raw_only).valid);
  const auto still_held = guidance.step(left_model, right_model, 0.005);
  EXPECT_TRUE(still_held.left.settled_hold_active);
  EXPECT_TRUE(still_held.right.settled_hold_active);

  Vec7 moved_left = left_q;
  Vec7 moved_right = right_q;
  moved_left[1] += 0.08;
  moved_right[1] -= 0.08;
  PicoTeleopFrame corroborated =
      frameFromRobot(robot, moved_left, moved_right);
  corroborated.sequence = 3;
  corroborated.source_timestamp_ns = 1'022'000'000LL;
  corroborated.left.position.x() += 0.10;
  corroborated.right.position.x() += 0.10;
  ASSERT_TRUE(guidance.updatePicoFrame(corroborated).valid);
  const auto released = guidance.step(left_model, right_model, 0.005);
  EXPECT_FALSE(released.left.settled_hold_active);
  EXPECT_FALSE(released.right.settled_hold_active);
}

}  // namespace
}  // namespace tianji_qp_ik
