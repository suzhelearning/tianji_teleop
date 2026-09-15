#include "tianji_qp_ik/cartesian_servo.hpp"
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
#include <utility>

namespace tianji_qp_ik {
namespace {

std::string modelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_test.xml")
      .string();
}

QpIkConfig testConfig() {
  return loadConfig((std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) /
                     "config" / "qp_ik_hierarchical.yaml")
                        .string());
}

void initializeAtMidpoint(MujocoRobot& robot) {
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    robot.setArmPosition(
        side, 0.5 * (limits.lower_position + limits.upper_position));
  }
  robot.forward();
}

DualArmTargets currentTargets(MujocoRobot& robot) {
  robot.forward();
  return {robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)};
}

class SequenceIk final : public IArmVelocityIk {
 public:
  SequenceIk(Vec7 velocity, SolverStatus status = SolverStatus::kSolved)
      : velocity_(std::move(velocity)), status_(status) {}

  ArmIkResult solve(const ArmIkInput& input) override {
    ++solve_count;
    last_input = input;
    ArmIkResult result;
    result.status = status_;
    result.qdot = velocity_;
    result.slack = input.desired_twist - input.jacobian * result.qdot;
    result.detail = status_ == SolverStatus::kSolved ? "sequence_solved" : "sequence_failed";
    return result;
  }

  void reset() override { ++reset_count; }
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kHierarchicalQp;
  }

  ArmIkInput last_input;
  int reset_count{0};
  int solve_count{0};

 private:
  Vec7 velocity_;
  SolverStatus status_;
};

class FailAfterFirstIk final : public IArmVelocityIk {
 public:
  ArmIkResult solve(const ArmIkInput& input) override {
    ++solve_count;
    ArmIkResult result;
    if (solve_count == 1) {
      result.status = SolverStatus::kSolved;
      result.qdot = Vec7::Constant(0.1);
      result.slack = input.desired_twist - input.jacobian * result.qdot;
      return result;
    }
    result.status = SolverStatus::kMaxIterations;
    result.detail = "forced_failure_after_first";
    return result;
  }
  void reset() override { ++reset_count; }
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kHierarchicalQp;
  }

  int solve_count{0};
  int reset_count{0};
};

TEST(HierarchicalController, FreezesOnlyArmWhoseReferenceTrackingIsLost) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  QpIkConfig config = testConfig();
  config.safety.reference_tracking_warn_rad = 0.0002;
  config.safety.reference_tracking_stop_rad = 0.0004;
  auto left = std::make_unique<SequenceIk>(Vec7::Constant(0.1));
  auto right = std::make_unique<SequenceIk>(Vec7::Constant(0.05));
  SequenceIk* left_observer = left.get();
  SequenceIk* right_observer = right.get();
  DualArmController controller(robot, config, IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  const Vec7 left_start = robot.armPosition(ArmSide::kLeft);
  const DualArmTargets targets = currentTargets(robot);

  ASSERT_TRUE(controller.step(targets, 0.005).accepted);
  const Vec7 frozen_reference = controller.reference(ArmSide::kLeft);
  robot.setArmPosition(ArmSide::kLeft, left_start);
  const ControllerDiagnostics held = controller.step(targets, 0.005);

  EXPECT_FALSE(held.accepted);
  EXPECT_EQ(held.hold_reason, HoldReason::kReferenceTrackingError);
  EXPECT_FALSE(held.left.accepted);
  EXPECT_TRUE(held.left.reference_frozen);
  EXPECT_TRUE(held.right.accepted);
  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(
      frozen_reference, 1e-12));
  EXPECT_EQ(left_observer->solve_count, 1);
  EXPECT_EQ(right_observer->solve_count, 2);
}

TEST(HierarchicalController, ModelStateOnlyIgnoresActualTrackingMismatch) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  QpIkConfig config = testConfig();
  config.controller.model_state_only = true;
  config.safety.reference_tracking_warn_rad = 0.0002;
  config.safety.reference_tracking_stop_rad = 0.0004;
  auto left = std::make_unique<SequenceIk>(Vec7::Constant(0.1));
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  SequenceIk* left_observer = left.get();
  DualArmController controller(robot, config, IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  const Vec7 left_actual_start = robot.armPosition(ArmSide::kLeft);
  const DualArmTargets targets = currentTargets(robot);

  ASSERT_TRUE(controller.step(targets, 0.005).accepted);
  const Vec7 first_reference = controller.reference(ArmSide::kLeft);
  robot.setArmPosition(ArmSide::kLeft, left_actual_start);
  const ControllerDiagnostics advanced = controller.step(targets, 0.005);

  EXPECT_TRUE(advanced.left.accepted);
  EXPECT_FALSE(advanced.left.reference_frozen);
  EXPECT_DOUBLE_EQ(advanced.left.reference_scale, 1.0);
  EXPECT_NEAR(advanced.left.reference_error_max_abs, 0.0005, 1e-12);
  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(
      first_reference + Vec7::Constant(0.0005), 1e-12));
  EXPECT_EQ(left_observer->solve_count, 2);
}

TEST(HierarchicalController, ScalesBetweenTrackingWarningAndStop) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  QpIkConfig config = testConfig();
  config.safety.reference_tracking_warn_rad = 0.0002;
  config.safety.reference_tracking_stop_rad = 0.0006;
  auto left = std::make_unique<SequenceIk>(Vec7::Constant(0.1));
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  SequenceIk* left_observer = left.get();
  DualArmController controller(robot, config, IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  const Vec7 left_start = robot.armPosition(ArmSide::kLeft);
  const DualArmTargets targets = currentTargets(robot);

  ASSERT_TRUE(controller.step(targets, 0.005).accepted);
  robot.setArmPosition(ArmSide::kLeft, left_start);
  const ControllerDiagnostics scaled = controller.step(targets, 0.005);

  ASSERT_TRUE(scaled.left.accepted);
  EXPECT_NEAR(scaled.left.reference_error_max_abs, 0.0005, 1e-12);
  EXPECT_NEAR(scaled.left.reference_scale, 0.25, 1e-12);
  EXPECT_FALSE(scaled.left.reference_frozen);
  EXPECT_EQ(left_observer->solve_count, 2);
}

class InconsistentIk final : public IArmVelocityIk {
 public:
  ArmIkResult solve(const ArmIkInput&) override {
    ArmIkResult result;
    result.status = SolverStatus::kSolved;
    result.qdot.setZero();
    result.slack.setZero();
    result.equality_residual = 0.0;
    result.detail = "inconsistent_solved_result";
    return result;
  }

  void reset() override {}
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kHierarchicalQp;
  }
};

class ScaledZeroIk final : public IArmVelocityIk {
 public:
  ArmIkResult solve(const ArmIkInput& input) override {
    ArmIkResult result;
    result.status = SolverStatus::kSolved;
    result.qdot.setZero();
    result.task_scale_position = 0.8;
    result.task_scale_orientation = 0.9;
    result.slack.head<3>() =
        result.task_scale_position * input.desired_twist.head<3>();
    result.slack.tail<3>() =
        result.task_scale_orientation * input.desired_twist.tail<3>();
    result.equality_residual = 0.0;
    return result;
  }
  void reset() override {}
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kHierarchicalQp;
  }
};

TEST(HierarchicalController, AcceptsAConsistentTaskScaledVelocitySolution) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  QpIkConfig config = testConfig();
  config.hierarchical_qp.task_scaling_enabled = true;
  config.arm_angle.enabled = false;
  auto left = std::make_unique<ScaledZeroIk>();
  auto right = std::make_unique<ScaledZeroIk>();
  DualArmController controller(robot, config, IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  DualArmTargets targets = currentTargets(robot);
  targets.left.position.x() += 0.01;
  targets.right.position.y() += 0.01;

  const ControllerDiagnostics result = controller.step(targets, 0.005);

  EXPECT_TRUE(result.accepted) << toString(result.hold_reason);
  EXPECT_TRUE(result.left.accepted) << toString(result.left.hold_reason);
  EXPECT_TRUE(result.right.accepted) << toString(result.right.hold_reason);
}

TEST(HierarchicalController, IntegratesReferenceRatherThanReconstructingIt) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  const Vec7 velocity = Vec7::Constant(0.1);
  auto left = std::make_unique<SequenceIk>(velocity);
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  SequenceIk* left_observer = left.get();
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  const Vec7 start = robot.armPosition(ArmSide::kLeft);
  const DualArmTargets targets = currentTargets(robot);

  ASSERT_TRUE(controller.step(targets, 0.005).accepted);
  EXPECT_TRUE(left_observer->last_input.q_ref.isApprox(start, 1e-12));
  const Vec7 command_model_before_second_step =
      controller.reference(ArmSide::kLeft);
  robot.setArmPosition(ArmSide::kLeft, start);
  robot.forward();
  const ArmKinematicSample expected_model = robot.armKinematicsAt(
      ArmSide::kLeft, command_model_before_second_step);
  const Pose expected_actual = robot.tcpPose(ArmSide::kLeft);
  const ControllerDiagnostics second = controller.step(targets, 0.005);
  ASSERT_TRUE(second.accepted);

  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(
      start + Vec7::Constant(0.001), 1e-12));
  EXPECT_TRUE(left_observer->last_input.q_ref.isApprox(
      start + Vec7::Constant(0.0005), 1e-12));
  EXPECT_TRUE(left_observer->last_input.q_measured.isApprox(
      command_model_before_second_step, 1e-12));
  EXPECT_TRUE(left_observer->last_input.jacobian.isApprox(
      expected_model.tcp_jacobian, 1e-12));
  EXPECT_TRUE(second.left.current.position.isApprox(
      expected_model.tcp_pose.position, 1e-12));
  EXPECT_TRUE(second.left.current.rotation.isApprox(
      expected_model.tcp_pose.rotation, 1e-12));
  EXPECT_TRUE(second.left.q_actual.isApprox(start, 1e-12));
  EXPECT_TRUE(second.left.tcp_actual.position.isApprox(
      expected_actual.position, 1e-12));
  EXPECT_TRUE(second.left.tcp_actual.rotation.isApprox(
      expected_actual.rotation, 1e-12));
  EXPECT_TRUE(second.left.actual_pose_error.isApprox(
      poseErrorWorld(targets.left, expected_actual), 1e-12));
  const JointVelocityBounds expected_bounds = computeJointVelocityBounds(
      command_model_before_second_step, Vec7::Constant(0.1),
      Vec7::Zero(), robot.mapping(ArmSide::kLeft).limits,
      testConfig().joint_limits, 0.005);
  EXPECT_TRUE(left_observer->last_input.bounds.lower.isApprox(
      expected_bounds.lower, 1e-12));
  EXPECT_TRUE(left_observer->last_input.bounds.upper.isApprox(
      expected_bounds.upper, 1e-12));
}

TEST(HierarchicalController, ActualPhysicalLimitViolationHoldsAffectedArm) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<SequenceIk>(Vec7::Constant(0.1));
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  SequenceIk* left_observer = left.get();
  SequenceIk* right_observer = right.get();
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  const Vec7 left_reference = controller.reference(ArmSide::kLeft);
  Vec7 invalid_actual = left_reference;
  invalid_actual[0] =
      robot.mapping(ArmSide::kLeft).limits.upper_position[0] + 0.01;
  robot.setArmPosition(ArmSide::kLeft, invalid_actual);
  const DualArmTargets targets = currentTargets(robot);

  const ControllerDiagnostics result = controller.step(targets, 0.005);

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.left.accepted);
  EXPECT_EQ(result.left.hold_reason, HoldReason::kBoundViolation);
  EXPECT_TRUE(result.right.accepted);
  EXPECT_EQ(left_observer->solve_count, 0);
  EXPECT_EQ(right_observer->solve_count, 1);
  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(
      left_reference, 1e-12));
}

TEST(HierarchicalController, AlgorithmSwitchPreservesMotionHistory) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<SequenceIk>(Vec7::Constant(0.1));
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  ASSERT_TRUE(controller.step(currentTargets(robot), 0.005).accepted);
  const Vec7 before = controller.reference(ArmSide::kLeft);
  const Vec7 velocity_before =
      controller.previousVelocity(ArmSide::kLeft);
  const Vec7 acceleration_before =
      controller.previousAcceleration(ArmSide::kLeft);
  ASSERT_FALSE(velocity_before.isZero());

  controller.setAlgorithm(IkAlgorithm::kNullspaceDls);

  EXPECT_EQ(controller.algorithm(), IkAlgorithm::kNullspaceDls);
  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(before));
  EXPECT_TRUE(controller.previousVelocity(ArmSide::kLeft).isApprox(
      velocity_before, 1e-12));
  EXPECT_TRUE(controller.previousAcceleration(ArmSide::kLeft).isApprox(
      acceleration_before, 1e-12));
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(before));
}

TEST(HierarchicalController, ImportsAndExportsCompleteMotionState) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<SequenceIk>(Vec7::Zero());
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  ArmMotionState requested;
  requested.q = controller.reference(ArmSide::kLeft);
  requested.qdot = Vec7::Constant(0.2);
  requested.qddot = Vec7::Constant(-0.3);

  ASSERT_TRUE(controller.setReferenceState(ArmSide::kLeft, requested));

  const ArmMotionState exported =
      controller.referenceState(ArmSide::kLeft);
  EXPECT_TRUE(exported.q.isApprox(requested.q, 1e-12));
  EXPECT_TRUE(exported.qdot.isApprox(requested.qdot, 1e-12));
  EXPECT_TRUE(exported.qddot.isApprox(requested.qddot, 1e-12));
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(requested.q, 1e-12));
  EXPECT_TRUE(robot.armVelocity(ArmSide::kLeft).isApprox(
      requested.qdot, 1e-12));

  ArmMotionState unsafe = requested;
  unsafe.qddot[0] =
      testConfig().joint_limits.max_acceleration_rad_s2[0] + 1.0;
  EXPECT_FALSE(controller.setReferenceState(ArmSide::kLeft, unsafe));
  EXPECT_TRUE(controller.referenceState(ArmSide::kLeft).qddot.isApprox(
      requested.qddot, 1e-12));
}

TEST(HierarchicalController, ExplicitResynchronizationUsesActualFeedbackOnce) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<SequenceIk>(Vec7::Constant(0.1));
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  ASSERT_TRUE(controller.step(currentTargets(robot), 0.005).accepted);
  Vec7 actual = robot.armPosition(ArmSide::kLeft);
  actual[0] -= 0.02;
  robot.setArmPosition(ArmSide::kLeft, actual);
  ASSERT_FALSE(robot.armVelocity(ArmSide::kLeft).isZero());

  ASSERT_TRUE(controller.synchronizeReferencesToActual());

  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(actual, 1e-12));
  EXPECT_TRUE(controller.previousVelocity(ArmSide::kLeft).isZero());
  EXPECT_TRUE(robot.armVelocity(ArmSide::kLeft).isZero());
}

TEST(HierarchicalController,
     ModelStateOnlySynchronizationPreservesModelReference) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  QpIkConfig config = testConfig();
  config.controller.model_state_only = true;
  auto left = std::make_unique<SequenceIk>(Vec7::Constant(0.1));
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  DualArmController controller(robot, config, IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  ASSERT_TRUE(controller.step(currentTargets(robot), 0.005).accepted);
  const Vec7 model_reference = controller.reference(ArmSide::kLeft);
  Vec7 actual = model_reference;
  actual[0] -= 0.02;
  robot.setArmPosition(ArmSide::kLeft, actual);

  ASSERT_TRUE(controller.synchronizeReferencesToActual());

  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(
      model_reference, 1e-12));
  EXPECT_TRUE(controller.previousVelocity(ArmSide::kLeft).isZero());
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(
      model_reference, 1e-12));
  EXPECT_TRUE(robot.armVelocity(ArmSide::kLeft).isZero());
}

TEST(HierarchicalController, OneArmFailureKeepsOtherArmRunning) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<SequenceIk>(Vec7::Constant(0.1));
  auto right = std::make_unique<SequenceIk>(
      Vec7::Constant(0.2), SolverStatus::kNumericalError);
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  const Vec7 left_before = controller.reference(ArmSide::kLeft);
  const Vec7 right_before = controller.reference(ArmSide::kRight);

  const ControllerDiagnostics result =
      controller.step(currentTargets(robot), 0.005);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.hold_reason, HoldReason::kSolverFailure);
  EXPECT_TRUE(result.left.accepted);
  EXPECT_FALSE(result.right.accepted);
  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(
      left_before + Vec7::Constant(0.0005), 1e-12));
  EXPECT_TRUE(controller.reference(ArmSide::kRight).isApprox(right_before));
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(
      left_before + Vec7::Constant(0.0005), 1e-12));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_before));
  EXPECT_TRUE(controller.previousVelocity(ArmSide::kLeft).isApprox(
      Vec7::Constant(0.1), 1e-12));
  EXPECT_TRUE(controller.previousVelocity(ArmSide::kRight).isZero());
}

TEST(HierarchicalController, FailedSolveUsesContinuousBoundedFallback) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<FailAfterFirstIk>();
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  const DualArmTargets targets = currentTargets(robot);
  ASSERT_TRUE(controller.step(targets, 0.005).accepted);
  const ArmMotionState before = controller.referenceState(ArmSide::kLeft);

  const ControllerDiagnostics failed = controller.step(targets, 0.005);

  EXPECT_FALSE(failed.left.accepted);
  EXPECT_TRUE(failed.left.fallback_applied);
  EXPECT_EQ(failed.left.hold_reason, HoldReason::kSolverFailure);
  const ArmMotionState after = controller.referenceState(ArmSide::kLeft);
  EXPECT_TRUE(after.qdot.isApprox(Vec7::Constant(0.2), 1e-12));
  EXPECT_TRUE(after.qddot.isApprox(before.qddot, 1e-12));
  EXPECT_TRUE(after.q.isApprox(before.q + after.qdot * 0.005, 1e-12));
}

TEST(HierarchicalController, RecomputesEqualityResidualInsteadOfTrustingBackend) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<InconsistentIk>();
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  DualArmTargets targets = currentTargets(robot);
  targets.left.position.x() += 0.02;

  const ControllerDiagnostics result = controller.step(targets, 0.005);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.hold_reason, HoldReason::kEqualityViolation);
}

}  // namespace
}  // namespace tianji_qp_ik
