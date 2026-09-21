#include "tianji_qp_ik/acceleration_qp.hpp"
#include "tianji_qp_ik/acceleration_qpoases_solver.hpp"

#include <gtest/gtest.h>

namespace tianji_qp_ik {
namespace {

AccelerationQpProblem problem(double scale = 1.0) {
  AccelerationQpConfig config;
  ArmAccelerationInput input;
  input.q_model.setZero();
  input.limits.lower_position = Vec7::Constant(-2.0);
  input.limits.upper_position = Vec7::Constant(2.0);
  input.bounds.lower = Vec7::Constant(-20.0);
  input.bounds.upper = Vec7::Constant(20.0);
  input.jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  input.desired_acceleration << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0;
  input.desired_acceleration *= scale;
  return AccelerationQpBuilder(config).build(input);
}

QpoasesConfig solverConfig() {
  QpoasesConfig config;
  config.max_working_set_recalculations = 200;
  config.cpu_time_limit_seconds = 0.02;
  return config;
}

TEST(AccelerationQpoasesSolver, InitializesSolvesAndHotstarts) {
  AccelerationQpoasesSolver solver(solverConfig());
  const AccelerationQpProblem first = problem();
  ASSERT_TRUE(solver.initialize(first));
  const AccelerationQpSolution initial = solver.solve(first);
  ASSERT_EQ(initial.status, SolverStatus::kSolved) << initial.detail;
  EXPECT_LT((first.A * initial.x - first.equality).norm(), 1e-8);

  const AccelerationQpProblem second = problem(-0.5);
  const AccelerationQpSolution updated = solver.solve(second);
  ASSERT_EQ(updated.status, SolverStatus::kSolved) << updated.detail;
  EXPECT_LT((second.A * updated.x - second.equality).norm(), 1e-8);
  EXPECT_GE(solver.hotstartCount(), 1);
}

TEST(AccelerationQpoasesSolver, EnforcesOptionalJointLinearConstraint) {
  AccelerationQpProblem constrained = problem();
  constrained.g[6] = 1.0;
  constrained.linear_constraint.active = true;
  constrained.linear_constraint.jacobian[6] = 1.0;
  constrained.linear_constraint.lower = 0.3;
  constrained.linear_constraint.upper = std::numeric_limits<double>::infinity();
  AccelerationQpoasesSolver solver(solverConfig());
  ASSERT_TRUE(solver.initialize(constrained));
  const AccelerationQpSolution result = solver.solve(constrained);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_GE(result.x[6], 0.3 - 1e-8);
}

TEST(AccelerationQpoasesSolver, ScalesInfeasibleCartesianAccelerationMagnitude) {
  AccelerationQpConfig config;
  config.slack_weight_position = 1.0e9;
  config.task_scaling_enabled = true;
  config.task_scaling_min_position = 0.0;
  config.task_scaling_min_orientation = 0.0;
  config.task_scaling_weight_position = 100.0;
  config.task_scaling_weight_orientation = 100.0;
  ArmAccelerationInput input;
  input.limits.lower_position = Vec7::Constant(-2.0);
  input.limits.upper_position = Vec7::Constant(2.0);
  input.bounds.lower = Vec7::Constant(-0.1);
  input.bounds.upper = Vec7::Constant(0.1);
  input.jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  input.desired_acceleration[0] = 10.0;
  const AccelerationQpProblem scaled =
      AccelerationQpBuilder(config).build(input);

  AccelerationQpoasesSolver solver(solverConfig());
  ASSERT_TRUE(solver.initialize(scaled));
  const AccelerationQpSolution result = solver.solve(scaled);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_NEAR(result.x[kBetaPositionIndex], 0.01, 2.0e-4);
  EXPECT_NEAR(result.x[0], 0.1, 2.0e-4);
  EXPECT_LT(result.x.segment<3>(kSlackStartIndex).norm(), 1.0e-5);
  EXPECT_NEAR(result.x[kBetaOrientationIndex], 1.0, 1.0e-8);
}

TEST(AccelerationQpoasesSolver, ResetRejectsSolveUntilReinitialized) {
  AccelerationQpoasesSolver solver(solverConfig());
  EXPECT_EQ(solver.solve(problem()).status, SolverStatus::kInvalidInput);
  ASSERT_TRUE(solver.initialize(problem()));
  solver.reset();
  EXPECT_EQ(solver.solve(problem()).status, SolverStatus::kInvalidInput);
}

}  // namespace
}  // namespace tianji_qp_ik
