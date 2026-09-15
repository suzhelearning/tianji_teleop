#include "tianji_qp_ik/acceleration_controller.hpp"
#include "tianji_qp_ik/acceleration_bounds.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace tianji_qp_ik {
namespace {

std::string modelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_test.xml")
      .string();
}

std::string fastModelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_pico_fast.xml")
      .string();
}

QpIkConfig config() {
  QpIkConfig result = loadConfig(
      (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "config" /
       "qp_ik_cartesian_otg_acceleration.yaml")
          .string());
  result.safety.reference_tracking_warn_rad = 10.0;
  result.safety.reference_tracking_stop_rad = 20.0;
  result.safety.velocity_reference_tracking_warn_rad_s = 10.0;
  result.safety.velocity_reference_tracking_stop_rad_s = 20.0;
  return result;
}

void initialize(MujocoRobot& robot, const Vec7& velocity = Vec7::Zero()) {
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    robot.setArmState(side, 0.5 * (limits.lower_position + limits.upper_position),
                      velocity);
  }
  robot.forward();
}

Vec7 redundantPose(const ArmLimits& limits) {
  Vec7 result = 0.5 * (limits.lower_position + limits.upper_position);
  for (int joint = 0; joint < kArmDof; ++joint) {
    result[joint] +=
        0.15 * (limits.upper_position[joint] - limits.lower_position[joint]) *
        std::sin(static_cast<double>(joint + 1));
  }
  return result;
}

void initializeRedundant(MujocoRobot& robot) {
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmState(side, redundantPose(robot.mapping(side).limits),
                      Vec7::Zero());
  }
  robot.forward();
}

DualArmReferences references(MujocoRobot& robot) {
  DualArmReferences result;
  robot.forward();
  result.left.pose = robot.tcpPose(ArmSide::kLeft);
  result.right.pose = robot.tcpPose(ArmSide::kRight);
  return result;
}

class FixedAccelerationSolver final : public IAccelerationQpSolver {
 public:
  FixedAccelerationSolver(Vec7 acceleration,
                          SolverStatus status = SolverStatus::kSolved)
      : acceleration_(std::move(acceleration)), status_(status) {}

  bool initialize(const AccelerationQpProblem&) override {
    ++initialize_count;
    return true;
  }
  AccelerationQpSolution solve(const AccelerationQpProblem& problem) override {
    ++solve_count;
    problems.push_back(problem);
    AccelerationQpSolution result;
    result.status = status_;
    result.x.head<7>() = acceleration_;
    result.x.segment<6>(kSlackStartIndex) =
        problem.equality - problem.A.leftCols<7>() * acceleration_;
    result.detail = "fixed_acceleration";
    return result;
  }
  void reset() override { ++reset_count; }

  int initialize_count{0};
  int solve_count{0};
  int reset_count{0};
  std::vector<AccelerationQpProblem> problems;

 private:
  Vec7 acceleration_;
  SolverStatus status_;
};

class FailAfterFirstAccelerationSolver final
    : public IAccelerationQpSolver {
 public:
  bool initialize(const AccelerationQpProblem&) override {
    ++initialize_count;
    return true;
  }
  AccelerationQpSolution solve(
      const AccelerationQpProblem& problem) override {
    ++solve_count;
    AccelerationQpSolution result;
    if (solve_count == 1) {
      result.status = SolverStatus::kSolved;
      result.x.head<kArmDof>() = Vec7::Constant(0.4);
      result.x.segment<6>(kSlackStartIndex) =
          problem.equality - problem.A.leftCols<kArmDof>() *
                                 result.x.head<kArmDof>();
      return result;
    }
    result.status = SolverStatus::kMaxIterations;
    result.iterations = 5;
    result.solve_time_us = 10.0;
    result.detail = "forced_failure_after_first";
    return result;
  }
  void reset() override { ++reset_count; }

  int initialize_count{0};
  int solve_count{0};
  int reset_count{0};
};

class InitializationThenSolveFailureSolver final
    : public IAccelerationQpSolver {
 public:
  bool initialize(const AccelerationQpProblem&) override {
    ++initialize_count;
    return initialize_count > 1;
  }
  AccelerationQpSolution solve(const AccelerationQpProblem&) override {
    ++solve_count;
    AccelerationQpSolution result;
    result.status = SolverStatus::kMaxIterations;
    result.iterations = 4;
    result.solve_time_us = 8.0;
    return result;
  }
  void reset() override { ++reset_count; }
  int initialize_count{0};
  int solve_count{0};
  int reset_count{0};
};

TEST(AccelerationController, InitializesVelocityAndUsesSecondOrderIntegration) {
  MujocoRobot robot(modelPath());
  const Vec7 initial_velocity = Vec7::Constant(0.1);
  initialize(robot, initial_velocity);
  const Vec7 initial_position = robot.armPosition(ArmSide::kLeft);
  const Vec7 qddot = Vec7::Constant(0.4);
  auto left = std::make_unique<FixedAccelerationSolver>(qddot);
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  DualArmAccelerationController controller(robot, config(), std::move(left),
                                           std::move(right));

  const AccelerationControllerDiagnostics result =
      controller.step(references(robot), 0.005);

  ASSERT_TRUE(result.accepted) << toString(result.hold_reason);
  EXPECT_TRUE(controller.velocityReference(ArmSide::kLeft).isApprox(
      initial_velocity + qddot * 0.005, 1e-12));
  EXPECT_TRUE(controller.positionReference(ArmSide::kLeft).isApprox(
      initial_position + initial_velocity * 0.005 +
          0.5 * qddot * 0.005 * 0.005,
      1e-12));
  EXPECT_TRUE(robot.armVelocity(ArmSide::kLeft).isApprox(
      controller.velocityReference(ArmSide::kLeft), 1e-12));
}

TEST(AccelerationController, OutwardOnlyAddsContinuityTaskAndBarrier) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  auto left = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  FixedAccelerationSolver* left_observer = left.get();
  FixedAccelerationSolver* right_observer = right.get();
  DualArmAccelerationController controller(robot, config(), std::move(left),
                                           std::move(right));
  controller.setArmAngleReferenceMode(ArmAngleReferenceMode::kOutwardOnly);

  const AccelerationControllerDiagnostics result =
      controller.step(references(robot), 0.005);

  ASSERT_FALSE(left_observer->problems.empty());
  ASSERT_FALSE(right_observer->problems.empty());
  EXPECT_TRUE(left_observer->problems.back().linear_constraint.active);
  EXPECT_TRUE(right_observer->problems.back().linear_constraint.active);
  EXPECT_TRUE(result.left.arm_angle_task_active);
  EXPECT_TRUE(result.right.arm_angle_task_active);
  EXPECT_NEAR(result.left.arm_angle_requested_acceleration, 0.0, 1e-12);
  EXPECT_NEAR(result.right.arm_angle_requested_acceleration, 0.0, 1e-12);
  EXPECT_TRUE(result.left.upper_arm_outward.constraint_active);
  EXPECT_TRUE(result.right.upper_arm_outward.constraint_active);
}

TEST(AccelerationController, PicoOutwardUsesPicoReferenceWithWeakTaskAndBarrier) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  auto left = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  DualArmAccelerationController controller(robot, config(), std::move(left),
                                           std::move(right));
  controller.setArmAngleReferenceMode(ArmAngleReferenceMode::kPicoOutward);
  DualArmDirectionReferences pico;
  pico.left = {true, Eigen::Vector3d::UnitX(),
               ArmDirectionReferenceSource::kPico};
  pico.right = pico.left;

  const AccelerationControllerDiagnostics result =
      controller.step(references(robot), pico, 0.005);

  EXPECT_EQ(result.left.arm_angle.reference_source,
            ArmDirectionReferenceSource::kPico);
  EXPECT_TRUE(result.left.arm_angle_task_active);
  EXPECT_FALSE(result.left.dls_posture_reference_active);
  EXPECT_TRUE(result.left.upper_arm_outward.constraint_active ||
              result.left.upper_arm_outward.feasibility_clipped);
}

TEST(AccelerationController, DisabledArmAngleReportsTaskInactive) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  QpIkConfig test_config = config();
  test_config.arm_angle.enabled = false;
  auto left = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  DualArmAccelerationController controller(
      robot, test_config, std::move(left), std::move(right));

  const AccelerationControllerDiagnostics result =
      controller.step(references(robot), 0.005);

  ASSERT_TRUE(result.accepted) << toString(result.hold_reason);
  EXPECT_FALSE(result.left.arm_angle_task_active);
  EXPECT_FALSE(result.right.arm_angle_task_active);
  EXPECT_DOUBLE_EQ(result.left.arm_angle_requested_acceleration, 0.0);
  EXPECT_DOUBLE_EQ(result.right.arm_angle_requested_acceleration, 0.0);
}

TEST(AccelerationController,
     StandaloneElbowDownReferenceYieldsToJointEnvelopeWithoutLosingPose) {
  MujocoRobot robot(modelPath());
  initializeRedundant(robot);
  const DualArmReferences stationary = references(robot);
  DualArmAccelerationController controller(robot, config());
  AccelerationControllerDiagnostics result;
  for (int cycle = 0; cycle < 200; ++cycle) {
    result = controller.step(stationary, defaultArmDirectionReferences(),
                             0.005);
    ASSERT_TRUE(result.accepted) << "cycle=" << cycle;
  }
  EXPECT_TRUE(std::isfinite(result.left.arm_angle.error_rad));
  EXPECT_TRUE(std::isfinite(result.right.arm_angle.error_rad));
  EXPECT_LT(result.left.pose_error.head<3>().norm(), 3e-3);
  EXPECT_LT(result.right.pose_error.head<3>().norm(), 3e-3);
}

TEST(AccelerationController, ExplicitPicoArmDirectionsChangeElbowPlane) {
  MujocoRobot robot(modelPath());
  initializeRedundant(robot);
  const DualArmReferences stationary = references(robot);
  DualArmDirectionReferences requested;
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmKinematicSample model =
        robot.armKinematicsAt(side, robot.armPosition(side));
    const Eigen::Vector3d axis =
        (model.wrist_position - model.shoulder_position).normalized();
    Eigen::Vector3d radial =
        model.elbow_position - model.shoulder_position;
    radial -= axis * axis.dot(radial);
    ArmDirectionReference reference;
    reference.valid = true;
    reference.direction =
        Eigen::AngleAxisd(side == ArmSide::kLeft ? 0.45 : -0.45, axis) *
        radial.normalized();
    reference.source = ArmDirectionReferenceSource::kPico;
    (side == ArmSide::kLeft ? requested.left : requested.right) = reference;
  }

  DualArmAccelerationController controller(robot, config());
  AccelerationControllerDiagnostics result;
  for (int cycle = 0; cycle < 200; ++cycle) {
    result = controller.step(stationary, requested, 0.005);
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

TEST(AccelerationController, UsesWeightedQpResultWithoutPostSolveMutation) {
  MujocoRobot robot(modelPath());
  initializeRedundant(robot);
  const DualArmReferences stationary = references(robot);
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
  auto left = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  FixedAccelerationSolver* left_observer = left.get();
  QpIkConfig test_config = config();
  test_config.acceleration_qp.arm_angle_weight = 10.0;
  DualArmAccelerationController controller(
      robot, test_config, std::move(left), std::move(right));

  const AccelerationControllerDiagnostics result =
      controller.step(stationary, requested, 0.005);

  ASSERT_TRUE(result.accepted) << toString(result.hold_reason);
  ASSERT_TRUE(result.left.arm_angle_task_active);
  ASSERT_EQ(left_observer->problems.size(), 1U);
  const AccelerationQpProblem& problem = left_observer->problems.front();
  const double base_weight = test_config.acceleration_qp.regularization +
                             test_config.acceleration_qp.jerk_weight +
                             test_config.acceleration_qp.posture_weight;
  EXPECT_GT((problem.H.topLeftCorner<kArmDof, kArmDof>().trace()),
            static_cast<double>(kArmDof) * base_weight);
  EXPECT_TRUE(result.left.qp.qddot.isZero(0.0));
  EXPECT_DOUBLE_EQ(result.left.arm_angle_achieved_acceleration, 0.0);
  EXPECT_NEAR(result.left.arm_angle_acceleration_residual,
              result.left.arm_angle_requested_acceleration, 1e-12);
}

TEST(AccelerationController, FailedArmFreezesWhileHealthyArmContinues) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  auto left = std::make_unique<FixedAccelerationSolver>(
      Vec7::Zero(), SolverStatus::kNumericalError);
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Constant(0.2));
  FixedAccelerationSolver* left_observer = left.get();
  FixedAccelerationSolver* right_observer = right.get();
  DualArmAccelerationController controller(robot, config(), std::move(left),
                                           std::move(right));
  const Vec7 left_before = robot.armPosition(ArmSide::kLeft);
  const Vec7 right_before = robot.armPosition(ArmSide::kRight);

  const AccelerationControllerDiagnostics result =
      controller.step(references(robot), 0.005);

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.left.accepted);
  EXPECT_TRUE(result.right.accepted);
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left_before));
  EXPECT_TRUE(robot.armVelocity(ArmSide::kLeft).isZero());
  EXPECT_FALSE(robot.armPosition(ArmSide::kRight).isApprox(right_before));
  EXPECT_EQ(left_observer->solve_count, 2);
  EXPECT_EQ(right_observer->solve_count, 1);
}

TEST(AccelerationController, LimitsColdRetryToTwoSolverAttempts) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  auto left = std::make_unique<InitializationThenSolveFailureSolver>();
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  InitializationThenSolveFailureSolver* observer = left.get();
  DualArmAccelerationController controller(
      robot, config(), std::move(left), std::move(right));

  const AccelerationControllerDiagnostics result =
      controller.step(references(robot), 0.005);

  EXPECT_FALSE(result.left.accepted);
  EXPECT_TRUE(result.left.fallback_applied);
  EXPECT_EQ(observer->initialize_count, 2);
  EXPECT_EQ(observer->solve_count, 1);
}

TEST(AccelerationController, FailedSolveUsesJerkBoundedBrakingFallback) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  auto left = std::make_unique<FailAfterFirstAccelerationSolver>();
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  DualArmAccelerationController controller(
      robot, config(), std::move(left), std::move(right));
  const DualArmReferences target = references(robot);
  ASSERT_TRUE(controller.step(target, 0.005).accepted);
  const ArmMotionState before = controller.referenceState(ArmSide::kLeft);

  const AccelerationControllerDiagnostics failed =
      controller.step(target, 0.005);

  EXPECT_FALSE(failed.left.accepted);
  EXPECT_TRUE(failed.left.fallback_applied);
  EXPECT_EQ(failed.left.hold_reason, HoldReason::kSolverFailure);
  const ArmMotionState after = controller.referenceState(ArmSide::kLeft);
  EXPECT_TRUE(after.qdot.isZero(1e-12));
  EXPECT_TRUE(after.qddot.isApprox(Vec7::Constant(-0.4), 1e-12));
  EXPECT_TRUE(((after.qddot - before.qddot).cwiseAbs().array() <=
               config().joint_acceleration_limits.max_jerk_rad_s3.array() *
                       0.005 +
                   1e-12)
                  .all());
}

TEST(AccelerationController, VelocityWatchdogFreezesOnlyLaggingArm) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  QpIkConfig test_config = config();
  test_config.safety.velocity_reference_tracking_warn_rad_s = 0.0005;
  test_config.safety.velocity_reference_tracking_stop_rad_s = 0.0008;
  auto left = std::make_unique<FixedAccelerationSolver>(Vec7::Constant(0.2));
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  DualArmAccelerationController controller(robot, test_config, std::move(left),
                                           std::move(right));
  ASSERT_TRUE(controller.step(references(robot), 0.005).accepted);
  robot.setArmState(ArmSide::kLeft, robot.armPosition(ArmSide::kLeft),
                    Vec7::Zero());

  const AccelerationControllerDiagnostics held =
      controller.step(references(robot), 0.005);

  EXPECT_FALSE(held.left.accepted);
  EXPECT_TRUE(held.left.reference_frozen);
  EXPECT_TRUE(held.right.accepted);
}

TEST(AccelerationController,
     UsesCommandReferenceKinematicsWhenActualFeedbackLags) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  const Vec7 initial_left = robot.armPosition(ArmSide::kLeft);
  Vec7 acceleration = Vec7::Zero();
  acceleration[0] = 1.0;
  acceleration[2] = -0.5;
  auto left = std::make_unique<FixedAccelerationSolver>(acceleration);
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  FixedAccelerationSolver* left_observer = left.get();
  DualArmAccelerationController controller(robot, config(), std::move(left),
                                           std::move(right));
  const DualArmReferences stationary_target = references(robot);
  constexpr double kDt = 0.005;
  for (int step = 0; step < 40; ++step) {
    ASSERT_TRUE(controller.step(stationary_target, kDt).accepted)
        << "step=" << step;
  }

  const Vec7 q_model = controller.positionReference(ArmSide::kLeft);
  const Vec7 qdot_model = controller.velocityReference(ArmSide::kLeft);
  const ArmKinematicSample model_sample =
      robot.armKinematicsAt(ArmSide::kLeft, q_model);
  const Vec6 model_jdot_qdot =
      robot.tcpJacobianDotTimesVelocityWorld(ArmSide::kLeft, q_model,
                                             qdot_model);
  DualArmReferences model_target = references(robot);
  robot.setArmState(ArmSide::kLeft, initial_left, Vec7::Zero());
  robot.forward();
  const Mat67 actual_jacobian = robot.tcpJacobianWorld(ArmSide::kLeft);
  ASSERT_GT((model_sample.tcp_jacobian - actual_jacobian).norm(), 1e-4);

  const AccelerationControllerDiagnostics result =
      controller.step(model_target, kDt);

  ASSERT_TRUE(result.left.accepted) << toString(result.left.hold_reason);
  ASSERT_FALSE(left_observer->problems.empty());
  EXPECT_TRUE(result.left.current.position.isApprox(
      model_sample.tcp_pose.position, 1e-12));
  EXPECT_TRUE(result.left.current.rotation.isApprox(
      model_sample.tcp_pose.rotation, 1e-12));
  EXPECT_LT(result.left.pose_error.norm(), 1e-10);
  EXPECT_GT(result.left.actual_pose_error.norm(), 1e-4);
  EXPECT_TRUE(result.left.q_actual.isApprox(initial_left, 1e-12));
  EXPECT_TRUE(result.left.qdot_actual.isZero(1e-12));
  EXPECT_TRUE(result.left.model_twist.isApprox(
      model_sample.tcp_jacobian * qdot_model, 1e-11));
  EXPECT_TRUE(result.left.jdot_qdot.isApprox(model_jdot_qdot, 1e-9));
  EXPECT_TRUE(left_observer->problems.back().A.leftCols<kArmDof>().isApprox(
      model_sample.tcp_jacobian, 1e-12));
}

TEST(AccelerationController, SynchronizesBothReferencesToSafeActualState) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  auto left = std::make_unique<FixedAccelerationSolver>(Vec7::Constant(0.5));
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Constant(-0.5));
  DualArmAccelerationController controller(robot, config(), std::move(left),
                                           std::move(right));
  ASSERT_TRUE(controller.step(references(robot), 0.005).accepted);

  Vec7 left_actual = controller.positionReference(ArmSide::kLeft);
  Vec7 right_actual = controller.positionReference(ArmSide::kRight);
  left_actual[0] -= 0.02;
  right_actual[1] += 0.02;
  const Vec7 left_velocity = Vec7::Constant(0.03);
  const Vec7 right_velocity = Vec7::Constant(-0.04);
  robot.setArmState(ArmSide::kLeft, left_actual, left_velocity);
  robot.setArmState(ArmSide::kRight, right_actual, right_velocity);

  ASSERT_TRUE(controller.synchronizeReferencesToActual());
  EXPECT_TRUE(controller.positionReference(ArmSide::kLeft).isApprox(
      left_actual, 1e-12));
  EXPECT_TRUE(controller.positionReference(ArmSide::kRight).isApprox(
      right_actual, 1e-12));
  EXPECT_TRUE(controller.velocityReference(ArmSide::kLeft).isApprox(
      left_velocity, 1e-12));
  EXPECT_TRUE(controller.velocityReference(ArmSide::kRight).isApprox(
      right_velocity, 1e-12));
}

TEST(AccelerationController,
     ResynchronizationSeedsAccelerationHistoryNearBrakingBoundary) {
  MujocoRobot robot(fastModelPath());
  QpIkConfig test_config = config();
  test_config.arm_angle.enabled = false;
  test_config.joint_acceleration_limits.max_acceleration_rad_s2
      << 60.0, 60.0, 60.0, 90.0, 90.0, 90.0, 90.0;
  test_config.joint_acceleration_limits.braking_acceleration_rad_s2
      << 45.0, 45.0, 45.0, 67.5, 67.5, 67.5, 67.5;
  test_config.joint_acceleration_limits.hard_jerk_enabled = true;
  test_config.joint_acceleration_limits.max_jerk_rad_s3
      << 1000.0, 1000.0, 1000.0, 1500.0, 1500.0, 1500.0, 1500.0;

  Vec7 left_q = 0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
                       robot.mapping(ArmSide::kLeft).limits.upper_position);
  left_q[5] = 0.80992196191674393;
  Vec7 left_qdot = Vec7::Zero();
  left_qdot[5] = 3.990005425148671;
  const Vec7 right_q =
      0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
             robot.mapping(ArmSide::kRight).limits.upper_position);
  robot.setArmState(ArmSide::kLeft, left_q, left_qdot);
  robot.setArmState(ArmSide::kRight, right_q, Vec7::Zero());
  robot.forward();

  const Vec7 seeded_acceleration = seedPreviousAccelerationForFeasibility(
      left_q, left_qdot, robot.mapping(ArmSide::kLeft).limits,
      test_config.joint_acceleration_limits, 0.005);
  const JointAccelerationBounds feasible_bounds = computeJointAccelerationBounds(
      left_q, left_qdot, seeded_acceleration,
      robot.mapping(ArmSide::kLeft).limits,
      test_config.joint_acceleration_limits, 0.005);
  ASSERT_TRUE((feasible_bounds.lower.array() <=
               feasible_bounds.upper.array()).all());
  const Vec7 feasible_acceleration =
      0.5 * (feasible_bounds.lower + feasible_bounds.upper);
  auto left =
      std::make_unique<FixedAccelerationSolver>(feasible_acceleration);
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  DualArmAccelerationController controller(robot, test_config,
                                           std::move(left), std::move(right));
  ASSERT_TRUE(controller.synchronizeReferencesToActual());
  EXPECT_LT(controller.previousAcceleration(ArmSide::kLeft)[5], 0.0);

  const AccelerationControllerDiagnostics result =
      controller.step(references(robot), 0.005);
  ASSERT_TRUE(result.left.accepted) << toString(result.left.hold_reason);
}

TEST(AccelerationController, ImportsAndExportsCompleteMotionState) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  auto left = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  DualArmAccelerationController controller(
      robot, config(), std::move(left), std::move(right));
  ArmMotionState requested;
  requested.q = controller.positionReference(ArmSide::kRight);
  requested.qdot = Vec7::Constant(-0.2);
  requested.qddot = Vec7::Constant(0.3);

  ASSERT_TRUE(controller.setReferenceState(ArmSide::kRight, requested));

  const ArmMotionState exported =
      controller.referenceState(ArmSide::kRight);
  EXPECT_TRUE(exported.q.isApprox(requested.q, 1e-12));
  EXPECT_TRUE(exported.qdot.isApprox(requested.qdot, 1e-12));
  EXPECT_TRUE(exported.qddot.isApprox(requested.qddot, 1e-12));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(requested.q, 1e-12));
  EXPECT_TRUE(robot.armVelocity(ArmSide::kRight).isApprox(
      requested.qdot, 1e-12));

  ArmMotionState unsafe = requested;
  unsafe.qddot[0] =
      config().joint_acceleration_limits.max_acceleration_rad_s2[0] + 1.0;
  EXPECT_FALSE(controller.setReferenceState(ArmSide::kRight, unsafe));
  EXPECT_TRUE(controller.referenceState(ArmSide::kRight).qddot.isApprox(
      requested.qddot, 1e-12));
}

TEST(AccelerationController, RejectsActualVelocityBeyondPhysicalLimit) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  const DualArmReferences target = references(robot);
  auto left = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  FixedAccelerationSolver* left_observer = left.get();
  DualArmAccelerationController controller(robot, config(), std::move(left),
                                           std::move(right));
  Vec7 unsafe_velocity = Vec7::Zero();
  unsafe_velocity[0] = robot.mapping(ArmSide::kLeft).limits.velocity[0] + 0.1;
  robot.setArmState(ArmSide::kLeft, robot.armPosition(ArmSide::kLeft),
                    unsafe_velocity);

  const AccelerationControllerDiagnostics result = controller.step(target, 0.005);

  EXPECT_FALSE(result.left.accepted);
  EXPECT_EQ(result.left.hold_reason, HoldReason::kBoundViolation);
  EXPECT_EQ(result.left.hold_joint_index, 0);
  EXPECT_EQ(left_observer->solve_count, 0);
  EXPECT_TRUE(result.right.accepted);
}

}  // namespace
}  // namespace tianji_qp_ik
