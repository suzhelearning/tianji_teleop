#include "tianji_qp_ik/equality_constrained_qpoases_solver.hpp"

#include <gtest/gtest.h>

namespace tianji_qp_ik {
namespace {

EqualityConstrainedQpProblem7 feasibleProblem() {
  EqualityConstrainedQpProblem7 problem;
  problem.H.setIdentity();
  problem.g.setZero();
  problem.A.setZero();
  problem.A.leftCols<6>().setIdentity();
  problem.equality << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  problem.lower.setConstant(-1.0);
  problem.upper.setConstant(1.0);
  return problem;
}

TEST(EqualityConstrainedQpoasesSolver7, SolvesSixExactEqualities) {
  EqualityConstrainedQpoasesSolver7 solver(QpoasesConfig{});
  const EqualityConstrainedQpProblem7 problem = feasibleProblem();

  const SolverResult7 result = solver.solve(problem);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_TRUE((problem.A * result.qdot)
                  .isApprox(problem.equality, 1.0e-9));
  EXPECT_NEAR(result.qdot[6], 0.0, 1.0e-9);
}

TEST(EqualityConstrainedQpoasesSolver7, HotstartsAChangedFeasibleProblem) {
  EqualityConstrainedQpoasesSolver7 solver(QpoasesConfig{});
  EqualityConstrainedQpProblem7 problem = feasibleProblem();
  ASSERT_EQ(solver.solve(problem).status, SolverStatus::kSolved);
  problem.equality[0] = 0.2;

  const SolverResult7 result = solver.solve(problem);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_TRUE((problem.A * result.qdot)
                  .isApprox(problem.equality, 1.0e-9));
  EXPECT_GT(solver.hotstartCount(), 0);
}

TEST(EqualityConstrainedQpoasesSolver7, RejectsInvalidAndInfeasibleProblems) {
  EqualityConstrainedQpoasesSolver7 solver(QpoasesConfig{});
  EqualityConstrainedQpProblem7 invalid = feasibleProblem();
  invalid.lower[0] = 2.0;
  invalid.upper[0] = 1.0;
  EXPECT_EQ(solver.solve(invalid).status, SolverStatus::kInvalidInput);

  EqualityConstrainedQpProblem7 infeasible = feasibleProblem();
  infeasible.equality[0] = 2.0;
  const SolverResult7 result = solver.solve(infeasible);
  EXPECT_EQ(result.status, SolverStatus::kInfeasible);
}

}  // namespace
}  // namespace tianji_qp_ik
