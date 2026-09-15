#include "tianji_qp_ik/qpoases_solver.hpp"

#include <gtest/gtest.h>

#include <Eigen/LU>

namespace tianji_qp_ik {
namespace {

QpoasesConfig accurateConfig() {
  QpoasesConfig config;
  config.max_working_set_recalculations = 100;
  config.cpu_time_limit_seconds = 0.01;
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

TEST(QpoasesSolver, SolvesAnalyticUnconstrainedProblem) {
  const QpProblem7 problem = unconstrainedProblem();
  QpoasesSolver7 solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(problem));
  const SolverResult7 result = solver.solve(problem);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_TRUE(result.qdot.isApprox(-problem.H.inverse() * problem.g, 1e-8));
}

TEST(QpoasesSolver, EnforcesActiveBounds) {
  QpProblem7 problem = unconstrainedProblem();
  problem.lower = Vec7::Constant(-0.1);
  problem.upper = Vec7::Constant(0.1);
  QpoasesSolver7 solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(problem));
  const SolverResult7 result = solver.solve(problem);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_LE((result.qdot - problem.upper).maxCoeff(), 1e-9);
  EXPECT_LE((problem.lower - result.qdot).maxCoeff(), 1e-9);
  EXPECT_NEAR(result.qdot[2], 0.1, 1e-9);
  EXPECT_NEAR(result.qdot[5], -0.1, 1e-9);
}

TEST(QpoasesSolver, FullHotstartUpdatesTheHessian) {
  QpProblem7 first = unconstrainedProblem();
  QpoasesSolver7 solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(first));
  ASSERT_EQ(solver.setupCount(), 1);
  const SolverResult7 first_result = solver.solve(first);
  ASSERT_EQ(first_result.status, SolverStatus::kSolved);

  QpProblem7 second = first;
  second.H(0, 0) = 5.0;
  second.H(0, 1) = 0.3;
  second.H(1, 0) = 0.3;
  const SolverResult7 second_result = solver.solve(second);
  ASSERT_EQ(second_result.status, SolverStatus::kSolved) << second_result.detail;
  EXPECT_TRUE(second_result.qdot.isApprox(-second.H.inverse() * second.g, 1e-8));
  EXPECT_FALSE(second_result.qdot.isApprox(first_result.qdot, 1e-3));
  EXPECT_EQ(solver.setupCount(), 1);
}

TEST(QpoasesSolver, FirstSolveReusesTheIdenticalInitializationResult) {
  const QpProblem7 first = unconstrainedProblem();
  QpoasesSolver7 solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(first));
  const SolverResult7 initialized_result = solver.solve(first);
  ASSERT_EQ(initialized_result.status, SolverStatus::kSolved);
  EXPECT_EQ(solver.hotstartCount(), 0);
  EXPECT_TRUE(initialized_result.qdot.isApprox(-first.H.inverse() * first.g, 1e-8));

  QpProblem7 changed = first;
  changed.g[0] += 0.01;
  ASSERT_EQ(solver.solve(changed).status, SolverStatus::kSolved);
  EXPECT_EQ(solver.hotstartCount(), 1);
}

TEST(QpoasesSolver, RejectsSolveBeforeInitialization) {
  QpoasesSolver7 solver(accurateConfig());
  EXPECT_EQ(solver.solve(unconstrainedProblem()).status, SolverStatus::kInvalidInput);
}

}  // namespace
}  // namespace tianji_qp_ik
