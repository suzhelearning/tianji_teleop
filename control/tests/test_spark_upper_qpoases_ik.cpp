#include "tianji_qp_ik/spark_upper_qpoases_ik.hpp"
#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <string>

namespace tianji_qp_ik {
namespace {

constexpr const char* kModelPath =
    TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_qp_test.xml";
constexpr const char* kUrdfPath =
    TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_s_ccs_696_v4_local.urdf";

SparkUpperQpoasesConfig ikConfig() {
  SparkUpperQpoasesConfig config;
  config.stage1_max_iterations = 10;
  config.stage2_max_iterations = 10;
  config.convergence_delta = 1.0e-8;
  config.integration_step = 1.0;
  config.trust_region_rad = 0.25;
  config.damping = 0.5;
  return config;
}

SparkUpperArmTarget targetFromSample(const ArmKinematicSample& sample) {
  SparkUpperArmTarget target;
  target.palm = sample.tcp_pose;
  target.elbow = sample.elbow_position;
  target.shoulder = sample.shoulder_position;
  target.wrist = sample.wrist_position;
  target.hand = sample.tcp_pose.position;
  return target;
}

Vec7 nominal(const ArmLimits& limits) {
  return 0.5 * (limits.lower_position + limits.upper_position);
}

TEST(SparkUpperDirectionTask, AnalyticJacobianMatchesCenteredDifference) {
  const Eigen::Vector3d target =
      Eigen::Vector3d(0.3, -0.4, 0.5).normalized();
  const Eigen::Vector3d start(0.1, -0.2, 0.7);
  const Eigen::Vector3d end(0.5, 0.1, 1.0);
  Mat37 start_jacobian;
  Mat37 end_jacobian;
  start_jacobian <<
      0.01, -0.03, 0.02, 0.00, 0.04, -0.02, 0.01,
      0.02, 0.01, -0.01, 0.03, 0.00, 0.02, -0.04,
      -0.01, 0.02, 0.03, -0.02, 0.01, 0.00, 0.02;
  end_jacobian <<
      0.05, -0.01, 0.03, 0.02, -0.02, 0.01, 0.04,
      -0.02, 0.04, 0.00, 0.01, 0.03, -0.01, 0.02,
      0.03, 0.00, -0.02, 0.04, 0.02, 0.01, -0.01;

  const auto linearization = linearizeSparkDirectionTask(
      target, start, end, start_jacobian, end_jacobian);
  ASSERT_TRUE(linearization.has_value());
  constexpr double kStep = 1.0e-7;
  for (int column = 0; column < kArmDof; ++column) {
    const Eigen::Vector3d plus_start =
        start + kStep * start_jacobian.col(column);
    const Eigen::Vector3d plus_end =
        end + kStep * end_jacobian.col(column);
    const Eigen::Vector3d minus_start =
        start - kStep * start_jacobian.col(column);
    const Eigen::Vector3d minus_end =
        end - kStep * end_jacobian.col(column);
    const Eigen::Vector3d plus_direction =
        (plus_end - plus_start).normalized();
    const Eigen::Vector3d minus_direction =
        (minus_end - minus_start).normalized();
    const Eigen::Vector3d finite_difference =
        (plus_direction - minus_direction) / (2.0 * kStep);
    EXPECT_TRUE(linearization->jacobian.col(column).isApprox(
        finite_difference, 1.0e-6));
  }
  EXPECT_TRUE(linearization->error.isApprox(
      target - (end - start).normalized(), 1.0e-12));
}

TEST(SparkUpperDirectionTask, RejectsDegenerateSegments) {
  const auto result = linearizeSparkDirectionTask(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Mat37::Zero(), Mat37::Zero());
  EXPECT_FALSE(result.has_value());
}

TEST(SparkUpperQpoasesIk7, ReducesTwoStagePoseAndElbowErrorWithinLimits) {
  MujocoRobot robot(kModelPath);
  const ArmSide side = ArmSide::kLeft;
  const ArmLimits limits = robot.mapping(side).limits;
  Vec7 known = nominal(limits);
  known << known[0] + 0.12, known[1] - 0.10, known[2] + 0.08,
      known[3] - 0.07, known[4] + 0.05, known[5] - 0.04, known[6] + 0.03;
  robot.setArmPosition(side, known);
  robot.forward();
  const SparkUpperArmTarget target =
      targetFromSample(robot.armKinematicsAt(side, known));
  Vec7 seed = known;
  seed[0] -= 0.18;
  seed[1] += 0.15;
  seed[2] -= 0.12;
  seed[3] += 0.10;
  seed[4] -= 0.08;
  seed[5] += 0.06;
  seed[6] -= 0.04;
  seed = seed.array()
             .max(limits.lower_position.array() + 0.06)
             .min(limits.upper_position.array() - 0.06)
             .matrix();
  const ArmKinematicSample initial = robot.armKinematicsAt(side, seed);
  const double initial_pose_error =
      poseErrorWorld(target.palm, initial.tcp_pose).norm();
  const double initial_elbow_error = (target.elbow - initial.elbow_position).norm();

  PinocchioArmKinematics kinematics(kUrdfPath);
  SparkUpperQpoasesIk7 solver(side, kinematics, limits, ikConfig());
  const SparkUpperIkResult result = solver.solve(target, seed);
  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_EQ(result.status, SolverStatus::kSolved);
  EXPECT_LT(result.stage2_error, initial_pose_error + initial_elbow_error);
  const ArmKinematicSample final_sample = robot.armKinematicsAt(side, result.q);
  const double final_pose_error =
      poseErrorWorld(target.palm, final_sample.tcp_pose).norm();
  const double final_elbow_error =
      (target.elbow - final_sample.elbow_position).norm();
  EXPECT_LT(final_pose_error, initial_pose_error);
  EXPECT_LT(final_pose_error + final_elbow_error,
            initial_pose_error + initial_elbow_error);
  EXPECT_LT((target.palm.position - final_sample.tcp_pose.position).norm(),
            1.0e-3);
  EXPECT_LE(result.stage1_iterations, 10);
  EXPECT_LE(result.stage2_iterations, 10);
  EXPECT_TRUE((result.q.array() >= limits.lower_position.array() + 0.05).all());
  EXPECT_TRUE((result.q.array() <= limits.upper_position.array() - 0.05).all());
}

TEST(SparkUpperQpoasesIk7, Stage1UsesDirectionsButNotAbsolutePositions) {
  MujocoRobot robot(kModelPath);
  const ArmSide side = ArmSide::kLeft;
  const ArmLimits limits = robot.mapping(side).limits;
  const Vec7 seed = nominal(limits);
  const ArmKinematicSample sample = robot.armKinematicsAt(side, seed);
  SparkUpperArmTarget original = targetFromSample(sample);
  original.palm.rotation =
      Eigen::AngleAxisd(0.18, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      original.palm.rotation;
  SparkUpperArmTarget translated = original;
  const Eigen::Vector3d translation(0.08, -0.03, 0.05);
  translated.shoulder += translation;
  translated.elbow += translation;
  translated.wrist += translation;
  translated.hand += translation;
  translated.palm.position += translation;

  PinocchioArmKinematics kinematics(kUrdfPath);
  SparkUpperQpoasesIk7 original_solver(side, kinematics, limits, ikConfig());
  SparkUpperQpoasesIk7 translated_solver(side, kinematics, limits, ikConfig());
  const SparkUpperIkResult original_result =
      original_solver.solve(original, seed);
  const SparkUpperIkResult translated_result =
      translated_solver.solve(translated, seed);
  ASSERT_TRUE(original_result.accepted) << original_result.detail;
  ASSERT_TRUE(translated_result.accepted) << translated_result.detail;
  EXPECT_TRUE(original_result.stage1_q.isApprox(translated_result.stage1_q,
                                                 1.0e-9));
  EXPECT_FALSE(original_result.q.isApprox(translated_result.q, 1.0e-4));
}

TEST(SparkUpperQpoasesIk7, RejectsInvalidTargetAndSupportsHotstart) {
  MujocoRobot robot(kModelPath);
  const ArmSide side = ArmSide::kRight;
  const ArmLimits limits = robot.mapping(side).limits;
  const Vec7 seed = nominal(limits);
  robot.setArmPosition(side, seed);
  robot.forward();
  SparkUpperArmTarget target =
      targetFromSample(robot.armKinematicsAt(side, seed));
  target.palm.rotation(0, 0) = 2.0;
  PinocchioArmKinematics kinematics(kUrdfPath);
  SparkUpperQpoasesIk7 solver(side, kinematics, limits, ikConfig());
  const SparkUpperIkResult invalid = solver.solve(target, seed);
  EXPECT_FALSE(invalid.accepted);
  EXPECT_EQ(invalid.status, SolverStatus::kInvalidInput);

  target = targetFromSample(robot.armKinematicsAt(side, seed));
  const SparkUpperIkResult first = solver.solve(target, seed);
  ASSERT_TRUE(first.accepted) << first.detail;
  const SparkUpperIkResult second = solver.solve(target, first.q);
  ASSERT_TRUE(second.accepted) << second.detail;
  EXPECT_TRUE(second.stage1_hotstart || second.stage2_hotstart);
}

TEST(SparkUpperQpoasesIk7, OtgConsistentSolvePreservesRequestedTcpPose) {
  MujocoRobot robot(kModelPath);
  const ArmSide side = ArmSide::kLeft;
  const ArmLimits limits = robot.mapping(side).limits;
  Vec7 target_q = nominal(limits);
  target_q << target_q[0] + 0.08, target_q[1] - 0.07,
      target_q[2] + 0.05, target_q[3] - 0.06, target_q[4] + 0.03,
      target_q[5] - 0.02, target_q[6] + 0.01;
  const ArmKinematicSample target_sample =
      robot.armKinematicsAt(side, target_q);
  Vec7 different_shape_q = target_q;
  different_shape_q[0] -= 0.20;
  different_shape_q[2] -= 0.18;
  different_shape_q[4] += 0.15;
  const SparkUpperArmTarget raw_shape =
      targetFromSample(robot.armKinematicsAt(side, different_shape_q));
  Vec7 previous_q = target_q;
  previous_q[1] += 0.10;
  previous_q[3] += 0.08;
  previous_q = previous_q.array()
                   .max(limits.lower_position.array() + 0.06)
                   .min(limits.upper_position.array() - 0.06)
                   .matrix();

  PinocchioArmKinematics kinematics(kUrdfPath);
  SparkUpperQpoasesConfig config = ikConfig();
  config.otg_position_tolerance_m = 2.0e-3;
  config.otg_orientation_tolerance_rad = 2.0e-2;
  SparkUpperQpoasesIk7 solver(side, kinematics, limits, config);
  const SparkUpperIkResult result = solver.solveOtgConsistent(
      raw_shape, target_sample.tcp_pose, previous_q);

  ASSERT_TRUE(result.accepted)
      << result.detail << " position_error=" << result.palm_position_error
      << " orientation_error=" << result.palm_orientation_error
      << " stage1_iterations=" << result.stage1_iterations;
  const ArmKinematicSample final_sample =
      robot.armKinematicsAt(side, result.q);
  const Vec6 error =
      poseErrorWorld(target_sample.tcp_pose, final_sample.tcp_pose);
  EXPECT_LT(error.head<3>().norm(), config.otg_position_tolerance_m);
  EXPECT_LT(error.tail<3>().norm(), config.otg_orientation_tolerance_rad);
  EXPECT_TRUE((result.q.array() >=
               limits.lower_position.array() +
                   config.joint_limit_margin_rad)
                  .all());
  EXPECT_TRUE((result.q.array() <=
               limits.upper_position.array() -
                   config.joint_limit_margin_rad)
                  .all());
}

}  // namespace
}  // namespace tianji_qp_ik
