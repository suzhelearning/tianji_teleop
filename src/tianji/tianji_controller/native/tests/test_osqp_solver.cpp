#include "tianji_qp_ik/osqp_solver.hpp"

#include <gtest/gtest.h>

#include <Eigen/LU>

namespace tianji_qp_ik {
namespace {

OsqpConfig accurateConfig() {
  OsqpConfig config;
  config.max_iterations = 2000;
  config.absolute_tolerance = 1e-9;
  config.relative_tolerance = 1e-9;
  config.polishing = true;
  return config;
}

QpProblem7 unconstrainedProblem() {
  QpProblem7 problem;
  problem.H.setZero();
  problem.H.diagonal() << 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0;
  problem.g << -0.2, 0.4, -0.8, 0.5, -0.3, 0.7, -0.6;
  problem.lower = Vec7::Constant(-10.0);
  problem.upper = Vec7::Constant(10.0);
  return problem;
}

TEST(OsqpSolver, SolvesAnalyticUnconstrainedProblem) {
  const QpProblem7 problem = unconstrainedProblem();
  OsqpSolver7 solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(problem));
  const SolverResult7 result = solver.solve(problem);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  const Vec7 expected = -problem.H.inverse() * problem.g;
  EXPECT_TRUE(result.qdot.isApprox(expected, 1e-7));
  EXPECT_GE(result.iterations, 1);
}

TEST(OsqpSolver, EnforcesActiveBoxBounds) {
  QpProblem7 problem = unconstrainedProblem();
  problem.lower = Vec7::Constant(-0.1);
  problem.upper = Vec7::Constant(0.1);
  OsqpSolver7 solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(problem));
  const SolverResult7 result = solver.solve(problem);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_LE((result.qdot - problem.upper).maxCoeff(), 1e-8);
  EXPECT_LE((problem.lower - result.qdot).maxCoeff(), 1e-8);
  EXPECT_NEAR(result.qdot[2], 0.1, 1e-7);
  EXPECT_NEAR(result.qdot[5], -0.1, 1e-7);
}

TEST(OsqpSolver, UpdatesChangingHessianWithoutRecreatingWorkspace) {
  QpProblem7 first = unconstrainedProblem();
  OsqpSolver7 solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(first));
  ASSERT_EQ(solver.setupCount(), 1);
  ASSERT_EQ(solver.solve(first).status, SolverStatus::kSolved);

  QpProblem7 second = first;
  second.H(0, 0) = 5.0;
  second.H(0, 1) = 0.3;
  second.H(1, 0) = 0.3;
  const SolverResult7 result = solver.solve(second);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_TRUE(result.qdot.isApprox(-second.H.inverse() * second.g, 1e-7));
  EXPECT_EQ(solver.setupCount(), 1);
}

TEST(OsqpSolver, RejectsSolveBeforeInitialization) {
  OsqpSolver7 solver(accurateConfig());
  EXPECT_EQ(solver.solve(unconstrainedProblem()).status, SolverStatus::kInvalidInput);
}

}  // namespace
}  // namespace tianji_qp_ik
