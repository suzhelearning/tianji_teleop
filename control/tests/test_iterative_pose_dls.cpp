#include "tianji_qp_ik/iterative_pose_dls.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace tianji_qp_ik {
namespace {

std::string modelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_test.xml")
      .string();
}

TEST(IterativePoseDls, ConvergesFromPreviousGoalToNearbyPose) {
  MujocoRobot robot(modelPath());
  const ArmSide side = ArmSide::kRight;
  const ArmLimits limits = robot.mapping(side).limits;
  const Vec7 seed = 0.5 * (limits.lower_position + limits.upper_position);
  Vec7 reachable = seed;
  reachable[0] += 0.04;
  reachable[1] -= 0.025;
  reachable[3] -= 0.03;

  IterativeDlsConfig config;
  IterativePoseDlsIk7 solver(config, 0.05);
  PoseDlsInput input;
  input.target = robot.armKinematicsAt(side, reachable).tcp_pose;
  input.seed = seed;
  input.limits = limits;
  input.evaluate = [&robot, side](const Vec7& q) {
    return robot.armKinematicsAt(side, q);
  };

  const PoseDlsResult result = solver.solve(input);

  EXPECT_EQ(result.status, PoseDlsStatus::kConverged) << result.detail;
  EXPECT_LE(result.position_error_m, config.position_tolerance_m);
  EXPECT_LE(result.orientation_error_rad, config.orientation_tolerance_rad);
  EXPECT_TRUE(result.q.allFinite());
}

TEST(IterativePoseDls, SecondaryMotionRemainsInStrictCartesianNullspace) {
  IterativeDlsConfig config;
  config.max_iterations = 1;
  config.position_tolerance_m = 1.0e-8;
  config.nominal_posture_gain = 0.0;
  config.arm_angle_gain = 0.5;
  config.maximum_joint_step_rad = 1.0;
  config.maximum_step_norm_rad = 2.0;
  IterativePoseDlsIk7 solver(config, 0.0);
  const auto evaluate = [](const Vec7& q) {
    ArmKinematicSample sample;
    sample.tcp_jacobian.setZero();
    sample.tcp_jacobian.block<6, 6>(0, 0) =
        0.01 * Eigen::Matrix<double, 6, 6>::Identity();
    sample.tcp_pose.position = (sample.tcp_jacobian * q).head<3>();
    return sample;
  };
  PoseDlsInput primary;
  primary.seed.setZero();
  primary.limits.lower_position.setConstant(-10.0);
  primary.limits.upper_position.setConstant(10.0);
  primary.target.position = Eigen::Vector3d(1.0e-3, 0.0, 0.0);
  primary.evaluate = evaluate;
  PoseDlsInput secondary = primary;
  secondary.secondary_task.active = true;
  secondary.secondary_task.jacobian.setOnes();
  secondary.secondary_task.target = 0.1;
  secondary.secondary_task.activation = 1.0;

  const PoseDlsResult without_secondary = solver.solve(primary);
  const PoseDlsResult with_secondary = solver.solve(secondary);
  const Mat67 jacobian = evaluate(Vec7::Zero()).tcp_jacobian;

  EXPECT_TRUE((jacobian * (with_secondary.q - without_secondary.q))
                  .isZero(1.0e-12));
  EXPECT_GT(std::abs(with_secondary.q[6]), 1.0e-6);
}

TEST(IterativePoseDls,
     ConvergedCartesianPoseDoesNotChaseRedundancyAcrossBranches) {
  IterativeDlsConfig config;
  config.max_iterations = 1;
  config.position_tolerance_m = 1.0e-8;
  config.orientation_tolerance_rad = 1.0e-8;
  config.nominal_posture_gain = 0.0;
  config.arm_angle_gain = 0.5;
  config.maximum_joint_step_rad = 1.0;
  config.maximum_step_norm_rad = 2.0;
  IterativePoseDlsIk7 solver(config, 0.0);
  PoseDlsInput input;
  input.seed.setZero();
  input.limits.lower_position.setConstant(-10.0);
  input.limits.upper_position.setConstant(10.0);
  input.secondary_task.active = true;
  input.secondary_task.jacobian[6] = 1.0;
  input.secondary_task.target = 0.4;
  input.secondary_task.activation = 1.0;
  input.evaluate = [](const Vec7& q) {
    ArmKinematicSample sample;
    sample.tcp_jacobian.leftCols<6>() =
        Eigen::Matrix<double, 6, 6>::Identity();
    sample.tcp_pose.position = q.head<3>();
    return sample;
  };

  const PoseDlsResult result = solver.solve(input);

  EXPECT_EQ(result.status, PoseDlsStatus::kConverged) << result.detail;
  // Match the proven DLS_IK baseline: once the Cartesian target is inside
  // tolerance, retain the nearest seed branch instead of continuing to chase
  // a potentially incompatible elbow-plane target.
  EXPECT_NEAR(result.q[6], 0.0, 1.0e-12);
  EXPECT_TRUE((input.evaluate(result.q).tcp_jacobian * result.q)
                  .isZero(1.0e-12));
}

TEST(IterativePoseDls,
     MultiIterationSolveConsumesSecondaryErrorInsteadOfReapplyingIt) {
  IterativeDlsConfig config;
  config.max_iterations = 10;
  config.position_tolerance_m = 1.0e-8;
  config.orientation_tolerance_rad = 1.0e-8;
  config.nominal_posture_gain = 0.0;
  config.arm_angle_gain = 0.5;
  config.maximum_joint_step_rad = 0.10;
  config.maximum_step_norm_rad = 1.0;
  IterativePoseDlsIk7 solver(config, 0.0);

  PoseDlsInput input;
  input.seed.setZero();
  input.limits.lower_position.setConstant(-10.0);
  input.limits.upper_position.setConstant(10.0);
  input.target.position.x() = 0.50;
  input.secondary_task.active = true;
  input.secondary_task.jacobian[6] = 1.0;
  input.secondary_task.target = 0.40;
  input.secondary_task.activation = 1.0;
  input.evaluate = [](const Vec7& q) {
    ArmKinematicSample sample;
    sample.tcp_jacobian.leftCols<6>() =
        Eigen::Matrix<double, 6, 6>::Identity();
    sample.tcp_pose.position = q.head<3>();
    return sample;
  };

  const PoseDlsResult result = solver.solve(input);

  EXPECT_EQ(result.status, PoseDlsStatus::kConverged) << result.detail;
  EXPECT_NEAR(result.q[0], 0.50, 1.0e-12);
  // The 0.40 rad secondary error is a state error, not a fresh command on
  // every internal Cartesian iteration.  Reapplying it five times would
  // incorrectly produce q[6] == 1.0 rad.
  EXPECT_GT(result.q[6], 0.35);
  EXPECT_LE(result.q[6], 0.40 + 1.0e-12);
}

TEST(IterativePoseDls, NeverPublishesAnInfeasiblePostureGuideCandidate) {
  IterativeDlsConfig config;
  config.max_iterations = 10;
  config.position_tolerance_m = 1.0e-8;
  config.orientation_tolerance_rad = 1.0e-8;
  config.nominal_posture_gain = 0.0;
  config.arm_angle_gain = 0.5;
  config.maximum_joint_step_rad = 0.10;
  config.maximum_step_norm_rad = 1.0;
  IterativePoseDlsIk7 solver(config, 0.0);

  PoseDlsInput input;
  input.seed.setZero();
  input.seed[6] = 0.05;
  input.limits.lower_position.setConstant(-10.0);
  input.limits.upper_position.setConstant(10.0);
  input.target.position.x() = 0.50;
  input.secondary_task.active = true;
  input.secondary_task.jacobian[6] = 1.0;
  input.secondary_task.target = -0.40;
  input.secondary_task.activation = 1.0;
  input.evaluate = [](const Vec7& q) {
    ArmKinematicSample sample;
    sample.tcp_jacobian.leftCols<6>() =
        Eigen::Matrix<double, 6, 6>::Identity();
    sample.tcp_pose.position = q.head<3>();
    sample.shoulder_position.setZero();
    sample.elbow_position.y() = q[6];
    return sample;
  };
  input.candidate_feasible = [](const ArmKinematicSample& sample) {
    return sample.elbow_position.y() >= -1.0e-12;
  };

  const PoseDlsResult result = solver.solve(input);

  EXPECT_NE(result.status, PoseDlsStatus::kRejected) << result.detail;
  EXPECT_GE(result.q[6], -1.0e-12);
  EXPECT_LT(result.position_error_m, result.initial_position_error_m);
}

TEST(IterativePoseDls, NeverPublishesACandidateWithWorseCartesianMerit) {
  IterativeDlsConfig config;
  config.max_iterations = 1;
  config.minimum_damping = 1.0e-8;
  config.maximum_damping = 1.0e-8;
  config.maximum_joint_step_rad = 0.20;
  config.maximum_step_norm_rad = 0.20;
  config.nominal_posture_gain = 0.0;
  config.arm_angle_gain = 0.0;
  IterativePoseDlsIk7 solver(config, 0.0);

  PoseDlsInput input;
  input.seed.setZero();
  input.seed[0] = -2.38;
  input.limits.lower_position.setConstant(-10.0);
  input.limits.upper_position.setConstant(10.0);
  input.target.position.x() = 0.98;
  input.evaluate = [](const Vec7& q) {
    ArmKinematicSample sample;
    sample.tcp_pose.position =
        Eigen::Vector3d(std::sin(2.0 * q[0]), q[1], q[2]);
    sample.tcp_jacobian.setZero();
    sample.tcp_jacobian(0, 0) = 2.0 * std::cos(2.0 * q[0]);
    sample.tcp_jacobian(1, 1) = 1.0;
    sample.tcp_jacobian(2, 2) = 1.0;
    sample.tcp_jacobian(3, 3) = 1.0;
    sample.tcp_jacobian(4, 4) = 1.0;
    sample.tcp_jacobian(5, 5) = 1.0;
    return sample;
  };

  const PoseDlsResult result = solver.solve(input);

  EXPECT_LE(result.position_error_m,
            result.initial_position_error_m + 1.0e-12)
      << result.detail;
}

TEST(IterativePoseDls, PostureGuideUsesAnInteriorMarginWithoutChangingLimits) {
  ArmLimits limits;
  limits.lower_position.setConstant(-1.0);
  limits.upper_position.setConstant(1.0);
  Vec7 requested;
  requested << -0.95, -0.81, -0.20, 0.0, 0.20, 0.81, 0.95;

  const Vec7 interior = clampPostureReferenceToInterior(
      requested, limits, 0.20);

  const Vec7 expected =
      (Vec7() << -0.80, -0.80, -0.20, 0.0, 0.20, 0.80, 0.80).finished();
  EXPECT_TRUE(interior.isApprox(expected));
  EXPECT_TRUE(limits.lower_position.isConstant(-1.0));
  EXPECT_TRUE(limits.upper_position.isConstant(1.0));
}

}  // namespace
}  // namespace tianji_qp_ik
