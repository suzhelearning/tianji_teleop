#include "tianji_qp_ik/acceleration_qp.hpp"

#include <Eigen/LU>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace tianji_qp_ik {
namespace {

AccelerationQpConfig config() {
  AccelerationQpConfig result;
  result.slack_weight_position = 1.0e4;
  result.slack_weight_orientation = 3.0e3;
  result.slack_linear_scale = 5.0;
  result.slack_angular_scale = 15.0;
  result.regularization = 5.0e-4;
  result.jerk_weight = 2.0e-3;
  result.posture_weight = 5.0e-4;
  result.posture_kp = 0.5;
  result.posture_kd = 0.3;
  result.arm_angle_weight = 10.0;
  return result;
}

ArmAccelerationInput input() {
  ArmAccelerationInput result;
  result.q_model << -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6;
  result.qdot_model = Vec7::Constant(0.2);
  result.qddot_previous << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  result.limits.lower_position = Vec7::Constant(-1.0);
  result.limits.upper_position = Vec7::Constant(1.0);
  result.bounds.lower = Vec7::Constant(-20.0);
  result.bounds.upper = Vec7::Constant(20.0);
  result.desired_acceleration << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0;
  result.jdot_qdot << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  result.jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  return result;
}

TEST(AccelerationQpBuilder, BuildsNormalizedSlackAndDynamicEquality) {
  const ArmAccelerationInput data = input();
  const AccelerationQpProblem problem = AccelerationQpBuilder(config()).build(data);
  EXPECT_TRUE((problem.H.topLeftCorner<7, 7>().isApprox(
      0.003 * Mat77::Identity(), 1e-14)));
  Vec6 expected_slack;
  expected_slack << 400.0, 400.0, 400.0,
      3000.0 / 225.0, 3000.0 / 225.0, 3000.0 / 225.0;
  EXPECT_TRUE((problem.H.block<6, 6>(kSlackStartIndex, kSlackStartIndex)
      .diagonal().isApprox(
      expected_slack, 1e-14)));
  const Vec7 qddot_posture =
      -config().posture_kp * data.q_model -
      config().posture_kd * data.qdot_model;
  const Vec7 expected_gradient =
      -config().jerk_weight * data.qddot_previous -
      config().posture_weight * qddot_posture;
  EXPECT_TRUE(problem.g.head<7>().isApprox(expected_gradient, 1e-14));
  EXPECT_TRUE(problem.A.leftCols<7>().isApprox(data.jacobian));
  EXPECT_TRUE((problem.A.block<6, 6>(0, kSlackStartIndex).isApprox(
      Eigen::Matrix<double, 6, 6>::Identity())));
  EXPECT_TRUE(problem.equality.isApprox(
      data.desired_acceleration - data.jdot_qdot));
  EXPECT_TRUE(problem.lower.head<7>().isApprox(data.bounds.lower));
  EXPECT_TRUE(problem.upper.head<7>().isApprox(data.bounds.upper));
  for (int index = kSlackStartIndex;
       index < kSlackStartIndex + kSlackVariables; ++index) {
    EXPECT_TRUE(std::isinf(problem.lower[index]));
    EXPECT_TRUE(std::isinf(problem.upper[index]));
  }
  EXPECT_DOUBLE_EQ(problem.lower[kBetaPositionIndex], 0.0);
  EXPECT_DOUBLE_EQ(problem.upper[kBetaPositionIndex], 0.0);
  EXPECT_DOUBLE_EQ(problem.lower[kBetaOrientationIndex], 0.0);
  EXPECT_DOUBLE_EQ(problem.upper[kBetaOrientationIndex], 0.0);
}

TEST(AccelerationQpBuilder, AddsIndependentCartesianTaskScalingVariables) {
  AccelerationQpConfig scaling_config = config();
  scaling_config.task_scaling_enabled = true;
  scaling_config.task_scaling_min_position = 0.15;
  scaling_config.task_scaling_min_orientation = 0.25;
  scaling_config.task_scaling_weight_position = 120.0;
  scaling_config.task_scaling_weight_orientation = 80.0;
  const ArmAccelerationInput data = input();
  const AccelerationQpProblem problem =
      AccelerationQpBuilder(scaling_config).build(data);

  EXPECT_TRUE((problem.A.block<3, 1>(0, kBetaPositionIndex).isApprox(
      -data.desired_acceleration.head<3>(), 1.0e-14)));
  EXPECT_TRUE((problem.A.block<3, 1>(3, kBetaOrientationIndex).isApprox(
      -data.desired_acceleration.tail<3>(), 1.0e-14)));
  EXPECT_TRUE(problem.equality.isApprox(-data.jdot_qdot, 1.0e-14));
  EXPECT_DOUBLE_EQ(problem.H(kBetaPositionIndex, kBetaPositionIndex), 120.0);
  EXPECT_DOUBLE_EQ(problem.H(kBetaOrientationIndex, kBetaOrientationIndex),
                   80.0);
  EXPECT_DOUBLE_EQ(problem.g[kBetaPositionIndex], -120.0);
  EXPECT_DOUBLE_EQ(problem.g[kBetaOrientationIndex], -80.0);
  EXPECT_DOUBLE_EQ(problem.lower[kBetaPositionIndex], 0.15);
  EXPECT_DOUBLE_EQ(problem.upper[kBetaPositionIndex], 1.0);
  EXPECT_DOUBLE_EQ(problem.lower[kBetaOrientationIndex], 0.25);
  EXPECT_DOUBLE_EQ(problem.upper[kBetaOrientationIndex], 1.0);
}

TEST(AccelerationQpBuilder, AddsWeightedArmAngleObjective) {
  ArmAccelerationInput data = input();
  const AccelerationQpProblem baseline =
      AccelerationQpBuilder(config()).build(data);
  data.arm_angle_task.active = true;
  data.arm_angle_task.jacobian[6] = 2.0;
  data.arm_angle_task.target = -4.0;
  data.arm_angle_task.activation = 0.5;
  data.arm_angle_task.weight_scale = 3.0;
  const AccelerationQpProblem with_secondary =
      AccelerationQpBuilder(config()).build(data);

  EXPECT_NEAR(with_secondary.H(6, 6) - baseline.H(6, 6), 60.0,
              1e-14);
  EXPECT_NEAR(with_secondary.g[6] - baseline.g[6], 120.0, 1e-14);
  EXPECT_TRUE(with_secondary.A.isApprox(baseline.A, 0.0));
  EXPECT_TRUE(with_secondary.equality.isApprox(baseline.equality, 0.0));
  EXPECT_TRUE((with_secondary.lower.array() == baseline.lower.array()).all());
  EXPECT_TRUE((with_secondary.upper.array() == baseline.upper.array()).all());
}

TEST(AccelerationQpBuilder, ProjectsArmAngleObjectiveIntoCartesianNullspace) {
  ArmAccelerationInput data = input();
  data.arm_angle_task.active = true;
  data.arm_angle_task.nullspace_only = true;
  data.arm_angle_task.jacobian[6] = 2.0;
  data.arm_angle_task.target = -4.0;
  data.arm_angle_task.activation = 0.5;
  data.arm_angle_task.weight_scale = 3.0;
  const AccelerationQpProblem baseline =
      AccelerationQpBuilder(config()).build(input());
  const AccelerationQpProblem projected =
      AccelerationQpBuilder(config()).build(data);

  const Eigen::Matrix<double, 6, 6> projected_cartesian_h =
      projected.H.topLeftCorner<6, 6>();
  const Eigen::Matrix<double, 6, 6> baseline_cartesian_h =
      baseline.H.topLeftCorner<6, 6>();
  EXPECT_TRUE((projected_cartesian_h.isApprox(baseline_cartesian_h, 1e-14)));
  EXPECT_TRUE(projected.g.head<6>().isApprox(baseline.g.head<6>(), 1e-14));
  EXPECT_NEAR(projected.H(6, 6) - baseline.H(6, 6), 60.0, 1e-14);
  EXPECT_NEAR(projected.g[6] - baseline.g[6], 120.0, 1e-14);
}

TEST(AccelerationQpBuilder,
     ReferencesNullspaceArmAngleToCartesianParticularAcceleration) {
  ArmAccelerationInput data = input();
  data.jacobian.col(6) << 0.3, -0.2, 0.1, 0.4, -0.5, 0.2;
  data.arm_angle_task.active = true;
  data.arm_angle_task.nullspace_only = true;
  data.arm_angle_task.jacobian[6] = 1.0;
  data.arm_angle_task.target = 2.0;
  data.arm_angle_task.activation = 0.75;
  data.arm_angle_task.weight_scale = 2.0;

  AccelerationQpConfig baseline_config = config();
  baseline_config.arm_angle_weight = 0.0;
  const AccelerationQpProblem baseline =
      AccelerationQpBuilder(baseline_config).build(data);
  const AccelerationQpProblem projected =
      AccelerationQpBuilder(config()).build(data);

  const double weight = config().arm_angle_weight *
                        data.arm_angle_task.activation *
                        data.arm_angle_task.weight_scale;
  const Mat77 nullspace =
      Mat77::Identity() -
      data.jacobian.transpose() *
          (data.jacobian * data.jacobian.transpose()).inverse() *
          data.jacobian;
  const Vec7 effective_jacobian =
      nullspace * data.arm_angle_task.jacobian;
  const Vec7 cartesian_particular =
      data.jacobian.transpose() *
      (data.jacobian * data.jacobian.transpose()).inverse() *
      baseline.equality;
  const double effective_target =
      data.arm_angle_task.target -
      data.arm_angle_task.jacobian.dot(cartesian_particular);

  const Mat77 h_residual =
      projected.H.topLeftCorner<7, 7>() -
      baseline.H.topLeftCorner<7, 7>() -
      weight * effective_jacobian * effective_jacobian.transpose();
  const Vec7 g_residual =
      projected.g.head<7>() - baseline.g.head<7>() +
      weight * effective_target * effective_jacobian;
  EXPECT_LT(h_residual.cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT(g_residual.cwiseAbs().maxCoeff(), 1e-12);
}

TEST(AccelerationQpBuilder, UsesExternalDlsPostureReferenceWhenActive) {
  ArmAccelerationInput data = input();
  data.posture_reference_active = true;
  data.posture_reference = Vec7::Constant(0.5);
  data.jacobian.setZero();
  const AccelerationQpProblem problem =
      AccelerationQpBuilder(config()).build(data);
  const Vec7 qddot_posture =
      config().posture_kp * (data.posture_reference - data.q_model) -
      config().posture_kd * data.qdot_model;
  const Vec7 expected_gradient =
      -config().jerk_weight * data.qddot_previous -
      config().posture_weight * qddot_posture;
  EXPECT_TRUE(problem.g.head<7>().isApprox(expected_gradient, 1e-14));
}

TEST(AccelerationQpBuilder, UsesExternalPostureVelocityAndAccelerationFeedforward) {
  ArmAccelerationInput data = input();
  data.posture_reference_active = true;
  data.posture_reference = Vec7::Constant(0.5);
  data.posture_velocity_reference = Vec7::Constant(-0.1);
  data.posture_acceleration_reference = Vec7::Constant(0.7);
  data.jacobian.setZero();
  const AccelerationQpProblem problem =
      AccelerationQpBuilder(config()).build(data);
  const Vec7 qddot_posture =
      data.posture_acceleration_reference +
      config().posture_kd *
          (data.posture_velocity_reference - data.qdot_model) +
      config().posture_kp * (data.posture_reference - data.q_model);
  const Vec7 expected_gradient =
      -config().jerk_weight * data.qddot_previous -
      config().posture_weight * qddot_posture;
  EXPECT_TRUE(problem.g.head<7>().isApprox(expected_gradient, 1e-14));
}

TEST(AccelerationQpBuilder, ProjectsExternalPostureObjectiveIntoCartesianNullspace) {
  ArmAccelerationInput data = input();
  data.posture_reference_active = true;
  data.posture_reference = Vec7::Constant(0.5);
  data.posture_velocity_reference = Vec7::Constant(-0.1);
  data.posture_acceleration_reference = Vec7::Constant(0.7);
  ASSERT_TRUE(data.jacobian.leftCols<6>().isApprox(
      Eigen::Matrix<double, 6, 6>::Identity()));
  ASSERT_TRUE(data.jacobian.col(6).isZero());

  const AccelerationQpProblem problem =
      AccelerationQpBuilder(config()).build(data);
  const double base_weight = config().regularization + config().jerk_weight;
  for (int joint = 0; joint < 6; ++joint) {
    EXPECT_NEAR(problem.H(joint, joint), base_weight, 1e-14);
    EXPECT_NEAR(problem.g[joint],
                -config().jerk_weight * data.qddot_previous[joint], 1e-14);
  }
  EXPECT_NEAR(problem.H(6, 6), base_weight + config().posture_weight, 1e-14);
  const double qddot_posture_joint7 =
      data.posture_acceleration_reference[6] +
      config().posture_kd *
          (data.posture_velocity_reference[6] - data.qdot_model[6]) +
      config().posture_kp *
          (data.posture_reference[6] - data.q_model[6]);
  EXPECT_NEAR(problem.g[6],
              -config().jerk_weight * data.qddot_previous[6] -
                  config().posture_weight * qddot_posture_joint7,
              1e-14);
}

TEST(AccelerationQpBuilder, CarriesOptionalJointLinearConstraint) {
  ArmAccelerationInput data = input();
  data.linear_constraint.active = true;
  data.linear_constraint.jacobian << -1.0, 2.0, -0.5, 0.0, -0.2, 0.3, -0.7;
  data.linear_constraint.lower = -2.5;
  data.linear_constraint.upper = 7.5;
  const AccelerationQpProblem problem =
      AccelerationQpBuilder(config()).build(data);

  EXPECT_TRUE(problem.linear_constraint.active);
  EXPECT_TRUE(problem.linear_constraint.jacobian.isApprox(
      data.linear_constraint.jacobian));
  EXPECT_DOUBLE_EQ(problem.linear_constraint.lower, -2.5);
  EXPECT_DOUBLE_EQ(problem.linear_constraint.upper, 7.5);
}

TEST(AccelerationQpBuilder, InactiveAndZeroWeightArmAngleAreNoOps) {
  ArmAccelerationInput data = input();
  const AccelerationQpProblem baseline =
      AccelerationQpBuilder(config()).build(data);
  data.arm_angle_task.active = true;
  data.arm_angle_task.jacobian[6] = 2.0;
  data.arm_angle_task.target = -4.0;
  data.arm_angle_task.activation = 0.5;
  AccelerationQpConfig zero_weight = config();
  zero_weight.arm_angle_weight = 0.0;
  const AccelerationQpProblem disabled =
      AccelerationQpBuilder(zero_weight).build(data);

  EXPECT_TRUE(disabled.H.isApprox(baseline.H, 0.0));
  EXPECT_TRUE(disabled.g.isApprox(baseline.g, 0.0));
  EXPECT_TRUE(disabled.A.isApprox(baseline.A, 0.0));
  EXPECT_TRUE(disabled.equality.isApprox(baseline.equality, 0.0));
}

TEST(AccelerationQpBuilder, NonFiniteArmAngleTaskDoesNotPoisonPrimary) {
  ArmAccelerationInput data = input();
  const AccelerationQpProblem baseline =
      AccelerationQpBuilder(config()).build(data);
  data.arm_angle_task.active = true;
  data.arm_angle_task.jacobian[6] = 2.0;
  data.arm_angle_task.target = std::numeric_limits<double>::quiet_NaN();
  data.arm_angle_task.activation = 0.5;
  const AccelerationQpProblem non_finite =
      AccelerationQpBuilder(config()).build(data);

  EXPECT_TRUE(non_finite.H.isApprox(baseline.H, 0.0));
  EXPECT_TRUE(non_finite.g.isApprox(baseline.g, 0.0));
  EXPECT_TRUE(non_finite.A.isApprox(baseline.A, 0.0));
  EXPECT_TRUE(non_finite.equality.isApprox(baseline.equality, 0.0));
}

TEST(AccelerationQpValidation, RejectsInvalidBoundsAndHessian) {
  AccelerationQpProblem problem = AccelerationQpBuilder(config()).build(input());
  SafetyConfig safety;
  problem.lower[2] = problem.upper[2] + 1.0;
  EXPECT_EQ(validateAccelerationProblem(problem, safety).reason,
            HoldReason::kInfeasibleBounds);
  problem = AccelerationQpBuilder(config()).build(input());
  problem.H(0, 0) = -1.0;
  EXPECT_EQ(validateAccelerationProblem(problem, safety).reason,
            HoldReason::kInvalidHessian);
}

}  // namespace
}  // namespace tianji_qp_ik
