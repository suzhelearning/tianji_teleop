#include "tianji_qp_ik/benchmark_dataset.hpp"
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace tianji_qp_ik {
namespace {

constexpr double kDt = 0.005;

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

Vec7 midpoint(const ArmLimits& limits) {
  return 0.5 * (limits.lower_position + limits.upper_position);
}

Vec7 redundantPose(const ArmLimits& limits) {
  Vec7 result = midpoint(limits);
  for (int joint = 0; joint < kArmDof; ++joint) {
    result[joint] +=
        0.15 * (limits.upper_position[joint] - limits.lower_position[joint]) *
        std::sin(static_cast<double>(joint + 1));
  }
  return result;
}

DualArmTargets currentTargets(MujocoRobot& robot) {
  robot.forward();
  return {robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)};
}

class FailingIk final : public IArmVelocityIk {
 public:
  ArmIkResult solve(const ArmIkInput&) override {
    ArmIkResult result;
    result.status = SolverStatus::kNumericalError;
    result.qdot = Vec7::Constant(0.4);
    return result;
  }
  void reset() override {}
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kHierarchicalQp;
  }
};

class RecordingZeroIk final : public IArmVelocityIk {
 public:
  ArmIkResult solve(const ArmIkInput& input) override {
    inputs.push_back(input);
    ArmIkResult result;
    result.status = SolverStatus::kSolved;
    result.slack = input.desired_twist;
    return result;
  }
  void reset() override {}
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kHierarchicalQp;
  }
  std::vector<ArmIkInput> inputs;
};

class OutOfBoundsLegacySolver final : public IQpSolver7 {
 public:
  bool initialize(const QpProblem7&) override { return true; }
  SolverResult7 solve(const QpProblem7&) override {
    SolverResult7 result;
    result.status = SolverStatus::kSolved;
    result.qdot = Vec7::Constant(100.0);
    result.detail = "fake_out_of_bounds_solution";
    return result;
  }
  void reset() override {}
  std::string_view name() const noexcept override { return "out_of_bounds_fake"; }
};

TEST(DualArmController, HoldsAtNominalPoseWithZeroError) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft, midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight, midpoint(robot.mapping(ArmSide::kRight).limits));
  QpIkConfig config = testConfig();
  config.arm_angle.enabled = false;
  DualArmController controller(robot, config);
  const ControllerDiagnostics result = controller.step(currentTargets(robot), kDt);
  ASSERT_TRUE(result.accepted) << toString(result.hold_reason);
  EXPECT_LT(result.left.ik.qdot.norm(), 1e-7);
  EXPECT_LT(result.right.ik.qdot.norm(), 1e-7);
  EXPECT_FALSE(result.left.arm_angle_task_active);
  EXPECT_FALSE(result.right.arm_angle_task_active);
}

TEST(DualArmController, ReportsTheEffectiveVelocityBoundsUsedByEachArm) {
  MujocoRobot robot(modelPath());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmPosition(side, midpoint(robot.mapping(side).limits));
  }
  QpIkConfig config = testConfig();
  config.arm_angle.enabled = false;
  DualArmController controller(robot, config);

  const ControllerDiagnostics result =
      controller.step(currentTargets(robot), kDt);

  ASSERT_TRUE(result.accepted) << toString(result.hold_reason);
  for (const ArmControllerDiagnostics* arm : {&result.left, &result.right}) {
    EXPECT_TRUE(arm->bounds.lower.allFinite());
    EXPECT_TRUE(arm->bounds.upper.allFinite());
    EXPECT_TRUE(
        (arm->bounds.lower.array() <= arm->bounds.upper.array()).all());
    EXPECT_TRUE((arm->ik.qdot.array() >= arm->bounds.lower.array() - 1e-12)
                    .all());
    EXPECT_TRUE((arm->ik.qdot.array() <= arm->bounds.upper.array() + 1e-12)
                    .all());
  }
}

TEST(DualArmController, OutwardOnlyAddsContinuityTaskAndBarrier) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft,
                       midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight,
                       midpoint(robot.mapping(ArmSide::kRight).limits));
  auto left = std::make_unique<RecordingZeroIk>();
  auto right = std::make_unique<RecordingZeroIk>();
  RecordingZeroIk* left_observer = left.get();
  RecordingZeroIk* right_observer = right.get();
  QpIkConfig config = testConfig();
  DualArmController controller(robot, config,
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  controller.setArmAngleReferenceMode(ArmAngleReferenceMode::kOutwardOnly);

  const ControllerDiagnostics result =
      controller.step(currentTargets(robot), kDt);

  ASSERT_FALSE(left_observer->inputs.empty());
  ASSERT_FALSE(right_observer->inputs.empty());
  EXPECT_TRUE(left_observer->inputs.back().arm_angle_task.active);
  EXPECT_TRUE(right_observer->inputs.back().arm_angle_task.active);
  EXPECT_NEAR(left_observer->inputs.back().arm_angle_task.target, 0.0, 1e-12);
  EXPECT_NEAR(right_observer->inputs.back().arm_angle_task.target, 0.0, 1e-12);
  EXPECT_TRUE(left_observer->inputs.back().linear_constraint.active);
  EXPECT_TRUE(right_observer->inputs.back().linear_constraint.active);
  EXPECT_TRUE(result.left.upper_arm_outward.constraint_active);
  EXPECT_TRUE(result.right.upper_arm_outward.constraint_active);
}

TEST(DualArmController, PicoOutwardUsesPicoReferenceWithWeakTaskAndBarrier) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft,
                       midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight,
                       midpoint(robot.mapping(ArmSide::kRight).limits));
  auto left = std::make_unique<RecordingZeroIk>();
  auto right = std::make_unique<RecordingZeroIk>();
  RecordingZeroIk* left_observer = left.get();
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  controller.setArmAngleReferenceMode(ArmAngleReferenceMode::kPicoOutward);
  DualArmDirectionReferences pico;
  pico.left = {true, Eigen::Vector3d::UnitX(),
               ArmDirectionReferenceSource::kPico};
  pico.right = pico.left;

  const ControllerDiagnostics result =
      controller.step(currentTargets(robot), pico, kDt);

  ASSERT_FALSE(left_observer->inputs.empty());
  EXPECT_EQ(result.left.arm_angle.reference_source,
            ArmDirectionReferenceSource::kPico);
  EXPECT_TRUE(left_observer->inputs.back().arm_angle_task.active);
  EXPECT_LE(std::abs(left_observer->inputs.back().arm_angle_task.target),
            testConfig().arm_angle.continuity_max_velocity_rad_s);
  EXPECT_TRUE(result.left.upper_arm_outward.constraint_active ||
              result.left.upper_arm_outward.feasibility_clipped);
}

TEST(DualArmController,
     PicoOutwardCanUseSmoothDlsPostureAsTheVelocityQpRedundancyTask) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft,
                       midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight,
                       midpoint(robot.mapping(ArmSide::kRight).limits));
  auto left = std::make_unique<RecordingZeroIk>();
  auto right = std::make_unique<RecordingZeroIk>();
  RecordingZeroIk* left_observer = left.get();
  RecordingZeroIk* right_observer = right.get();
  QpIkConfig config = testConfig();
  config.iterative_dls.posture_reference_enabled = true;
  config.dls_posture_ruckig.enabled = false;
  DualArmController controller(robot, config, IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  controller.setArmAngleReferenceMode(ArmAngleReferenceMode::kPicoOutward);
  DualArmDirectionReferences pico;
  pico.left = {true, Eigen::Vector3d::UnitX(),
               ArmDirectionReferenceSource::kPico};
  pico.right = pico.left;

  DualArmTargets targets = currentTargets(robot);
  targets.left.position.x() += 0.01;
  targets.right.position.x() += 0.01;
  const ControllerDiagnostics result = controller.step(targets, pico, kDt);

  ASSERT_FALSE(left_observer->inputs.empty());
  ASSERT_FALSE(right_observer->inputs.empty());
  EXPECT_TRUE(left_observer->inputs.back().posture_task.active);
  EXPECT_TRUE(right_observer->inputs.back().posture_task.active);
  EXPECT_TRUE(left_observer->inputs.back().posture_task.target.allFinite());
  EXPECT_TRUE(right_observer->inputs.back().posture_task.target.allFinite());
  EXPECT_TRUE(result.left.dls_posture_reference.isApprox(
      result.left.dls_posture_goal));
  EXPECT_TRUE(result.right.dls_posture_reference.isApprox(
      result.right.dls_posture_goal));
  EXPECT_GT((result.left.dls_posture_goal - result.left.q_ref).norm(),
            1.0e-6);
  EXPECT_GT((result.right.dls_posture_goal - result.right.q_ref).norm(),
            1.0e-6);
  EXPECT_TRUE(result.left.dls_posture_velocity_reference.isZero(1.0e-12));
  EXPECT_TRUE(result.right.dls_posture_velocity_reference.isZero(1.0e-12));
  EXPECT_TRUE(left_observer->inputs.back().posture_task.target.isApprox(
      config.hierarchical_qp.nominal_gain *
          (result.left.dls_posture_goal - result.left.q_ref)));
  EXPECT_TRUE(right_observer->inputs.back().posture_task.target.isApprox(
      config.hierarchical_qp.nominal_gain *
          (result.right.dls_posture_goal - result.right.q_ref)));
  EXPECT_FALSE(result.left.dls_posture_ruckig_accepted);
  EXPECT_FALSE(result.right.dls_posture_ruckig_accepted);
  EXPECT_FALSE(left_observer->inputs.back().arm_angle_task.active);
  EXPECT_FALSE(right_observer->inputs.back().arm_angle_task.active);
  EXPECT_TRUE(result.left.upper_arm_outward.constraint_active ||
              result.left.upper_arm_outward.feasibility_clipped);
  EXPECT_TRUE(result.right.upper_arm_outward.constraint_active ||
              result.right.upper_arm_outward.feasibility_clipped);
}

TEST(DualArmController,
     ExternalSparkPostureReachesBothVelocityQpsAndKeepsOutwardBarrier) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft,
                       midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight,
                       midpoint(robot.mapping(ArmSide::kRight).limits));
  auto left = std::make_unique<RecordingZeroIk>();
  auto right = std::make_unique<RecordingZeroIk>();
  RecordingZeroIk* left_observer = left.get();
  RecordingZeroIk* right_observer = right.get();
  QpIkConfig config = testConfig();
  DualArmController controller(robot, config, IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  controller.setArmAngleReferenceMode(ArmAngleReferenceMode::kPicoOutward);

  DualArmJointVelocityPostureTasks posture;
  posture.left.active = true;
  posture.left.source = JointVelocityPostureSource::kSparkJointReference;
  posture.left.target = Vec7::Constant(0.12);
  posture.left.activation = 0.8;
  posture.left.weight = 0.03;
  posture.right = posture.left;
  posture.right.target = Vec7::Constant(-0.14);

  DualArmReferences references;
  DualArmTargets targets = currentTargets(robot);
  targets.left.position.x() += 0.01;
  targets.right.position.x() += 0.01;
  references.left.pose = targets.left;
  references.right.pose = targets.right;
  const ControllerDiagnostics result = controller.step(
      references, defaultArmDirectionReferences(), posture, kDt);

  ASSERT_FALSE(left_observer->inputs.empty());
  ASSERT_FALSE(right_observer->inputs.empty());
  const ArmIkInput& left_input = left_observer->inputs.back();
  const ArmIkInput& right_input = right_observer->inputs.back();
  EXPECT_EQ(left_input.posture_task.source,
            JointVelocityPostureSource::kSparkJointReference);
  EXPECT_TRUE(left_input.posture_task.target.isApprox(posture.left.target));
  EXPECT_TRUE(right_input.posture_task.target.isApprox(posture.right.target));
  EXPECT_DOUBLE_EQ(left_input.posture_task.weight, 0.03);
  Vec6 left_expected_twist = left_input.jacobian * posture.left.target;
  Vec6 right_expected_twist = right_input.jacobian * posture.right.target;
  left_expected_twist.x() += 0.01 * config.cartesian_servo.kp_position.x();
  right_expected_twist.x() += 0.01 * config.cartesian_servo.kp_position.x();
  EXPECT_TRUE(left_input.desired_twist.isApprox(left_expected_twist));
  EXPECT_TRUE(right_input.desired_twist.isApprox(right_expected_twist));
  EXPECT_FALSE(left_input.arm_angle_task.active);
  EXPECT_FALSE(right_input.arm_angle_task.active);
  EXPECT_TRUE(result.left.upper_arm_outward.constraint_active ||
              result.left.upper_arm_outward.feasibility_clipped);
  EXPECT_TRUE(result.right.upper_arm_outward.constraint_active ||
              result.right.upper_arm_outward.feasibility_clipped);
}

TEST(DualArmController,
     SparkFeedforwardPostureDoesNotDuplicateCartesianFeedforward) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft,
                       midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight,
                       midpoint(robot.mapping(ArmSide::kRight).limits));
  auto left = std::make_unique<RecordingZeroIk>();
  auto right = std::make_unique<RecordingZeroIk>();
  RecordingZeroIk* left_observer = left.get();
  RecordingZeroIk* right_observer = right.get();
  QpIkConfig config = testConfig();
  DualArmController controller(robot, config, IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));

  DualArmJointVelocityPostureTasks posture;
  posture.left.active = true;
  posture.left.source =
      JointVelocityPostureSource::kSparkFeedforwardJointReference;
  posture.left.target = Vec7::Constant(0.12);
  posture.left.weight = 0.03;
  posture.right = posture.left;
  posture.right.target = Vec7::Constant(-0.14);

  DualArmReferences references;
  const DualArmTargets targets = currentTargets(robot);
  references.left.pose = targets.left;
  references.right.pose = targets.right;
  references.left.twist = Vec6::LinSpaced(0.1, 0.6);
  references.right.twist = Vec6::LinSpaced(-0.6, -0.1);
  (void)controller.step(references, defaultArmDirectionReferences(), posture,
                        kDt);

  ASSERT_FALSE(left_observer->inputs.empty());
  ASSERT_FALSE(right_observer->inputs.empty());
  EXPECT_TRUE(left_observer->inputs.back().desired_twist.isApprox(
      references.left.twist));
  EXPECT_TRUE(right_observer->inputs.back().desired_twist.isApprox(
      references.right.twist));
}

TEST(DualArmController, NullspaceDlsReportsArmAngleTaskInactive) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft,
                       midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight,
                       midpoint(robot.mapping(ArmSide::kRight).limits));
  QpIkConfig config = testConfig();
  config.ik_algorithm = IkAlgorithm::kNullspaceDls;
  DualArmController controller(robot, config);

  const ControllerDiagnostics result =
      controller.step(currentTargets(robot), defaultArmDirectionReferences(),
                      kDt);

  ASSERT_TRUE(result.accepted) << toString(result.hold_reason);
  EXPECT_TRUE(result.left.arm_angle.active);
  EXPECT_TRUE(result.right.arm_angle.active);
  EXPECT_FALSE(result.left.arm_angle_task_active);
  EXPECT_FALSE(result.right.arm_angle_task_active);
  EXPECT_DOUBLE_EQ(result.left.arm_angle_requested_rate, 0.0);
  EXPECT_DOUBLE_EQ(result.right.arm_angle_requested_rate, 0.0);
}

TEST(DualArmController,
     StandaloneElbowDownReferenceYieldsToJointEnvelopeWithoutLosingPose) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft,
                       redundantPose(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight,
                       redundantPose(robot.mapping(ArmSide::kRight).limits));
  robot.forward();
  const DualArmTargets targets = currentTargets(robot);
  DualArmController controller(robot, testConfig());
  ControllerDiagnostics result;
  for (int cycle = 0; cycle < 200; ++cycle) {
    result = controller.step(targets, defaultArmDirectionReferences(), kDt);
    ASSERT_TRUE(result.accepted) << "cycle=" << cycle;
  }
  EXPECT_TRUE(std::isfinite(result.left.arm_angle.error_rad));
  EXPECT_TRUE(std::isfinite(result.right.arm_angle.error_rad));
  EXPECT_LT(result.left.pose_error.head<3>().norm(), 1e-3);
  EXPECT_LT(result.right.pose_error.head<3>().norm(), 1e-3);
}

TEST(DualArmController, ExplicitPicoArmDirectionsConvergeWithinOneSecond) {
  MujocoRobot robot(modelPath());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmPosition(side, redundantPose(robot.mapping(side).limits));
  }
  robot.forward();
  const DualArmTargets targets = currentTargets(robot);
  DualArmDirectionReferences requested;
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmKinematicSample model =
        robot.armKinematicsAt(side, robot.armPosition(side));
    const Eigen::Vector3d axis =
        (model.wrist_position - model.shoulder_position).normalized();
    Eigen::Vector3d radial = model.elbow_position - model.shoulder_position;
    radial -= axis * axis.dot(radial);
    ArmDirectionReference reference;
    reference.valid = true;
    reference.direction =
        Eigen::AngleAxisd(side == ArmSide::kLeft ? 0.45 : -0.45, axis) *
        radial.normalized();
    reference.source = ArmDirectionReferenceSource::kPico;
    (side == ArmSide::kLeft ? requested.left : requested.right) = reference;
  }

  DualArmController controller(robot, testConfig());
  ControllerDiagnostics result;
  for (int cycle = 0; cycle < 200; ++cycle) {
    result = controller.step(targets, requested, kDt);
    ASSERT_TRUE(result.accepted) << "cycle=" << cycle;
  }
  EXPECT_EQ(result.left.arm_angle.reference_source,
            ArmDirectionReferenceSource::kPico);
  EXPECT_EQ(result.right.arm_angle.reference_source,
            ArmDirectionReferenceSource::kPico);
  EXPECT_LT(std::abs(result.left.arm_angle.error_rad), 0.10);
  EXPECT_LT(std::abs(result.right.arm_angle.error_rad), 0.10);
  EXPECT_LT(result.left.pose_error.head<3>().norm(), 1e-3);
  EXPECT_LT(result.right.pose_error.head<3>().norm(), 1e-3);
}

TEST(DualArmController, ReachablePositionTargetConverges) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft, midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight, midpoint(robot.mapping(ArmSide::kRight).limits));
  const QpIkConfig config = testConfig();
  DualArmController controller(robot, config);
  DualArmTargets targets = currentTargets(robot);
  targets.left.position.x() += 0.02;
  for (int step = 0; step < 400; ++step) {
    ASSERT_TRUE(controller.step(targets, kDt).accepted) << "step=" << step;
  }
  robot.forward();
  const double final_error =
      (targets.left.position - robot.tcpPose(ArmSide::kLeft).position).norm();
  std::cout << "reachable_position_final_error_m=" << final_error << '\n';
  EXPECT_LT(final_error, 0.002);
}

TEST(DualArmController, SolverFailureHoldsOnlyFailedArm) {
  MujocoRobot robot(modelPath());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmPosition(side, midpoint(robot.mapping(side).limits));
  }
  const QpIkConfig config = testConfig();
  const Vec7 left_before = robot.armPosition(ArmSide::kLeft);
  const Vec7 right_before = robot.armPosition(ArmSide::kRight);
  DualArmController controller(
      robot, config, IkAlgorithm::kHierarchicalQp,
      std::make_unique<FailingIk>(),
      makeArmVelocityIk(IkAlgorithm::kHierarchicalQp, config));
  DualArmTargets moving = currentTargets(robot);
  moving.right.position.x() += 0.02;
  const ControllerDiagnostics failed = controller.step(moving, kDt);
  EXPECT_FALSE(failed.accepted);
  EXPECT_EQ(failed.hold_reason, HoldReason::kSolverFailure);
  EXPECT_FALSE(failed.left.accepted);
  EXPECT_TRUE(failed.right.accepted);
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left_before));
  EXPECT_FALSE(robot.armPosition(ArmSide::kRight).isApprox(right_before));

  DualArmTargets invalid = currentTargets(robot);
  invalid.left.position.x() = std::numeric_limits<double>::quiet_NaN();
  const ControllerDiagnostics nan_result = controller.step(invalid, kDt);
  EXPECT_FALSE(nan_result.accepted);
  EXPECT_EQ(nan_result.hold_reason, HoldReason::kNonFiniteProblem);
  EXPECT_FALSE(nan_result.left.accepted);
  EXPECT_FALSE(nan_result.right.accepted);

  invalid = currentTargets(robot);
  invalid.right.rotation(0, 0) = 2.0;
  const ControllerDiagnostics invalid_rotation = controller.step(invalid, kDt);
  EXPECT_FALSE(invalid_rotation.accepted);
  EXPECT_EQ(invalid_rotation.hold_reason, HoldReason::kNonFiniteProblem);
}

TEST(DualArmController, LegacySolvedButRejectedResultFailsTheWholeCycle) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft,
                       midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight,
                       midpoint(robot.mapping(ArmSide::kRight).limits));
  const QpIkConfig config = testConfig();
  DualArmController controller(
      robot, config, std::make_unique<OutOfBoundsLegacySolver>(),
      makeSolver(SolverBackend::kQpoases, config));

  const ControllerDiagnostics result =
      controller.step(currentTargets(robot), kDt);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.hold_reason, HoldReason::kSolverFailure);
}

TEST(DualArmController, InvalidManualCommandHoldsAndLaterValidCommandRecovers) {
  MujocoRobot robot(modelPath());
  robot.setArmPosition(ArmSide::kLeft, midpoint(robot.mapping(ArmSide::kLeft).limits));
  robot.setArmPosition(ArmSide::kRight, midpoint(robot.mapping(ArmSide::kRight).limits));
  const QpIkConfig config = testConfig();
  const DualArmTargets initial = currentTargets(robot);
  TargetManager targets(config, initial);
  targets.setMode(TargetMode::kManual, 0.0);
  Pose invalid = initial.left;
  invalid.rotation(0, 0) = 2.0;
  targets.setManualTarget(ArmSide::kLeft, invalid);
  DualArmController controller(robot, config);
  const Vec7 left_before = robot.armPosition(ArmSide::kLeft);
  const Vec7 right_before = robot.armPosition(ArmSide::kRight);

  const ControllerDiagnostics held = controller.step(targets.sample(kDt), kDt);
  EXPECT_FALSE(held.accepted);
  EXPECT_EQ(held.hold_reason, HoldReason::kNonFiniteProblem);
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left_before));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_before));

  Pose recovered = initial.left;
  recovered.position.x() += 0.01;
  targets.setManualTarget(ArmSide::kLeft, recovered);
  const ControllerDiagnostics resumed = controller.step(targets.sample(2.0 * kDt), kDt);
  EXPECT_TRUE(resumed.accepted) << toString(resumed.hold_reason);
}

TEST(DualArmController, ActualPositionOutsidePhysicalLimitsHolds) {
  MujocoRobot robot(modelPath());
  const QpIkConfig config = testConfig();
  Vec7 invalid = midpoint(robot.mapping(ArmSide::kLeft).limits);
  invalid[0] = robot.mapping(ArmSide::kLeft).limits.upper_position[0] + 0.1;
  robot.setArmPosition(ArmSide::kLeft, invalid);
  DualArmController controller(robot, config);
  const ControllerDiagnostics result = controller.step(currentTargets(robot), kDt);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.hold_reason, HoldReason::kBoundViolation);
}

TEST(DualArmController, UnreachableTargetStaysBoundedAndCanRecover) {
  MujocoRobot robot(modelPath());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmPosition(side, midpoint(robot.mapping(side).limits));
  }
  const QpIkConfig config = testConfig();
  DualArmController controller(robot, config);
  const DualArmTargets home = currentTargets(robot);
  DualArmTargets unreachable = home;
  unreachable.left.position.x() += 3.0;
  for (int step = 0; step < 200; ++step) {
    const ControllerDiagnostics result = controller.step(unreachable, kDt);
    ASSERT_TRUE(result.accepted) << "step=" << step;
    EXPECT_LE(result.left.ik.qdot.cwiseAbs().maxCoeff(),
              config.joint_limits.velocity_scale * 3.1416 + 1e-8);
  }
  for (int step = 0; step < 600; ++step) {
    ASSERT_TRUE(controller.step(home, kDt).accepted) << "recovery step=" << step;
  }
  robot.forward();
  const double recovery_error =
      (home.left.position - robot.tcpPose(ArmSide::kLeft).position).norm();
  std::cout << "unreachable_recovery_final_error_m=" << recovery_error << '\n';
  EXPECT_LT(recovery_error, 0.003);
}

TEST(DualArmController, SingularStartAndOrientationTargetRemainFinite) {
  MujocoRobot robot(modelPath());
  const QpIkConfig config = testConfig();
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmPosition(side, safeSingularBenchmarkPosition(side));
  }
  robot.forward();
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const Eigen::JacobiSVD<Mat67> decomposition(robot.tcpJacobianWorld(side));
    const auto singular_values = decomposition.singularValues();
    double minimum_singular_value = singular_values[0];
    for (Eigen::Index index = 1; index < singular_values.size(); ++index) {
      minimum_singular_value = std::min(minimum_singular_value, singular_values[index]);
    }
    std::cout << toString(side) << "_safe_pose_minimum_singular_value="
              << minimum_singular_value << '\n';
    EXPECT_LT(minimum_singular_value, 1e-4);
  }
  DualArmController controller(robot, config);
  DualArmTargets targets = currentTargets(robot);
  targets.left.rotation =
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix() * targets.left.rotation;
  for (int step = 0; step < 600; ++step) {
    const ControllerDiagnostics result = controller.step(targets, kDt);
    ASSERT_TRUE(result.accepted) << "step=" << step;
    EXPECT_TRUE(result.left.ik.qdot.allFinite());
  }
  robot.forward();
  const double final_orientation_error =
      rotationDistance(targets.left.rotation, robot.tcpPose(ArmSide::kLeft).rotation);
  std::cout << "singular_orientation_final_error_rad=" << final_orientation_error << '\n';
  EXPECT_LT(final_orientation_error, 3.14159265358979323846 / 180.0);
}

}  // namespace
}  // namespace tianji_qp_ik
