#include "tianji_qp_ik/hierarchical_qpoases_solver.hpp"

#include <gtest/gtest.h>

namespace tianji_qp_ik {
namespace {

QpoasesConfig accurateConfig() {
  QpoasesConfig config;
  config.max_working_set_recalculations = 200;
  config.cpu_time_limit_seconds = 0.02;
  return config;
}

HierarchicalQpConfig builderConfig() {
  HierarchicalQpConfig config;
  config.lambda_reg = 1e-3;
  config.posture_weight = 1e-2;
  config.continuity_weight = 1e-2;
  config.nominal_gain = 0.2;
  config.slack_weight_position = 1e4;
  config.slack_weight_orientation = 3e3;
  config.equality_tolerance = 1e-8;
  return config;
}

ArmIkInput trackingInput() {
  ArmIkInput input;
  input.q_measured.setZero();
  input.limits.lower_position = Vec7::Constant(-2.0);
  input.limits.upper_position = Vec7::Constant(2.0);
  input.bounds.lower = Vec7::Constant(-0.4);
  input.bounds.upper = Vec7::Constant(0.4);
  input.desired_twist << 0.35, -0.25, 0.20, 0.15, -0.10, 0.30;
  input.jacobian <<
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.1, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.1, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1,
      0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.1, 0.0, 0.0, 0.0, 1.0, 0.0;
  return input;
}

HierarchicalQpProblem fullRankTrackingProblem() {
  return HierarchicalQpBuilder(builderConfig()).build(trackingInput());
}

HierarchicalQpProblem zeroJacobianProblem() {
  ArmIkInput input = trackingInput();
  input.jacobian.setZero();
  return HierarchicalQpBuilder(builderConfig()).build(input);
}

TEST(HierarchicalQpoasesSolver, SolvesTrackingEqualityWithSlack) {
  const HierarchicalQpProblem problem = fullRankTrackingProblem();
  HierarchicalQpoasesSolver solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(problem));
  const HierarchicalQpSolution result = solver.solve(problem);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_GT(result.solve_time_us, 0.0);
  EXPECT_LT((problem.A * result.x - problem.equality).norm(), 1e-8);
  EXPECT_LE((result.x.head<7>() - problem.upper.head<7>()).maxCoeff(), 1e-9);
  EXPECT_LE((problem.lower.head<7>() - result.x.head<7>()).maxCoeff(), 1e-9);
}

TEST(HierarchicalQpoasesSolver, ZeroJacobianUsesFiniteSlack) {
  const HierarchicalQpProblem problem = zeroJacobianProblem();
  HierarchicalQpoasesSolver solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(problem));
  const HierarchicalQpSolution result = solver.solve(problem);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_TRUE((result.x.head<7>().isZero(1e-9)));
  EXPECT_TRUE((result.x.segment<6>(kSlackStartIndex)
                   .isApprox(problem.equality, 1e-8)));
}

TEST(HierarchicalQpoasesSolver,
     SolvesExtendedVelocityProblemWithBoundedTaskScaling) {
  HierarchicalQpConfig config = builderConfig();
  config.task_scaling_enabled = true;
  config.task_scaling_min_position = 0.20;
  config.task_scaling_min_orientation = 0.25;
  config.task_scaling_weight_position = 10.0;
  config.task_scaling_weight_orientation = 10.0;
  ArmIkInput input = trackingInput();
  input.bounds.lower = Vec7::Constant(-0.05);
  input.bounds.upper = Vec7::Constant(0.05);
  input.desired_twist.setConstant(0.8);
  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(config).build(input);
  HierarchicalQpoasesSolver solver(accurateConfig());

  ASSERT_TRUE(solver.initialize(problem));
  const HierarchicalQpSolution result = solver.solve(problem);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_LT((problem.A * result.x - problem.equality).norm(), 1.0e-8);
  EXPECT_GE(result.x[kBetaPositionIndex],
            config.task_scaling_min_position - 1.0e-9);
  EXPECT_LE(result.x[kBetaPositionIndex], 1.0 + 1.0e-9);
  EXPECT_GE(result.x[kBetaOrientationIndex],
            config.task_scaling_min_orientation - 1.0e-9);
  EXPECT_LE(result.x[kBetaOrientationIndex], 1.0 + 1.0e-9);
  EXPECT_LT(result.x[kBetaPositionIndex], 0.9);
  EXPECT_LT(result.x[kBetaOrientationIndex], 0.9);
}

TEST(HierarchicalQpoasesSolver, EnforcesOptionalJointLinearConstraint) {
  HierarchicalQpProblem problem = zeroJacobianProblem();
  problem.g[0] = 1.0;
  problem.linear_constraint.active = true;
  problem.linear_constraint.jacobian[0] = 1.0;
  problem.linear_constraint.lower = 0.2;
  problem.linear_constraint.upper = std::numeric_limits<double>::infinity();
  HierarchicalQpoasesSolver solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(problem));
  const HierarchicalQpSolution result = solver.solve(problem);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_GE(result.x[0], 0.2 - 1e-8);
}

TEST(HierarchicalQpoasesSolver, EnforcesTwoIndependentGeometryConstraints) {
  HierarchicalQpProblem problem = zeroJacobianProblem();
  problem.linear_constraint.active = true;
  problem.linear_constraint.jacobian[0] = 1.0;
  problem.linear_constraint.lower = 0.1;
  problem.branch_lock_constraint.active = true;
  problem.branch_lock_constraint.jacobian[1] = 1.0;
  problem.branch_lock_constraint.lower = 0.2;
  HierarchicalQpoasesSolver solver(accurateConfig());

  ASSERT_TRUE(solver.initialize(problem));
  const HierarchicalQpSolution result = solver.solve(problem);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_GE(result.x[0], 0.1 - 1.0e-8);
  EXPECT_GE(result.x[1], 0.2 - 1.0e-8);
}

TEST(HierarchicalQpoasesSolver, FullHotstartUpdatesMatricesEqualityAndBounds) {
  const HierarchicalQpProblem first = fullRankTrackingProblem();
  HierarchicalQpoasesSolver solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(first));
  ASSERT_EQ(solver.solve(first).status, SolverStatus::kSolved);
  EXPECT_EQ(solver.setupCount(), 1);
  EXPECT_EQ(solver.hotstartCount(), 0);

  ArmIkInput second_input = trackingInput();
  second_input.jacobian(0, 0) = 0.7;
  second_input.jacobian(2, 6) = 0.2;
  second_input.desired_twist *= -0.5;
  second_input.bounds.lower = Vec7::Constant(-0.2);
  second_input.bounds.upper = Vec7::Constant(0.25);
  HierarchicalQpConfig second_config = builderConfig();
  second_config.continuity_weight = 0.04;
  const HierarchicalQpProblem second =
      HierarchicalQpBuilder(second_config).build(second_input);

  const HierarchicalQpSolution result = solver.solve(second);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_LT((second.A * result.x - second.equality).norm(), 1e-8);
  EXPECT_LE((result.x.head<7>() - second.upper.head<7>()).maxCoeff(), 1e-9);
  EXPECT_LE((second.lower.head<7>() - result.x.head<7>()).maxCoeff(), 1e-9);
  EXPECT_EQ(solver.setupCount(), 1);
  EXPECT_GE(solver.hotstartCount(), 1);
}

TEST(HierarchicalQpoasesSolver, RejectsSolveBeforeInitializationAndAfterReset) {
  const HierarchicalQpProblem problem = fullRankTrackingProblem();
  HierarchicalQpoasesSolver solver(accurateConfig());
  EXPECT_EQ(solver.solve(problem).status, SolverStatus::kInvalidInput);
  ASSERT_TRUE(solver.initialize(problem));
  solver.reset();
  EXPECT_EQ(solver.solve(problem).status, SolverStatus::kInvalidInput);
}

}  // namespace
}  // namespace tianji_qp_ik
