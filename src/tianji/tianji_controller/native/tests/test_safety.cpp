#include "tianji_qp_ik/safety.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace tianji_qp_ik {
namespace {

QpProblem7 validProblem() {
  QpProblem7 problem;
  problem.H = Mat77::Identity();
  problem.g.setZero();
  problem.lower = Vec7::Constant(-1.0);
  problem.upper = Vec7::Constant(1.0);
  return problem;
}

SafetyGuard guard() {
  SafetyConfig config;
  config.bound_tolerance = 1e-8;
  config.hessian_eigenvalue_tolerance = 1e-10;
  return SafetyGuard(config);
}

TEST(SafetyGuard, AcceptsValidSolvedResult) {
  SolverResult7 result;
  result.status = SolverStatus::kSolved;
  result.qdot = Vec7::Constant(0.2);
  const SafetyDecision decision = guard().validateResult(validProblem(), result);
  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.reason, HoldReason::kNone);
}

TEST(SafetyGuard, RejectsNonFiniteProblem) {
  QpProblem7 problem = validProblem();
  problem.g[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(guard().validateProblem(problem).reason, HoldReason::kNonFiniteProblem);
}

TEST(SafetyGuard, RejectsInfeasibleBounds) {
  QpProblem7 problem = validProblem();
  problem.lower[2] = 2.0;
  problem.upper[2] = 1.0;
  EXPECT_EQ(guard().validateProblem(problem).reason, HoldReason::kInfeasibleBounds);
}

TEST(SafetyGuard, RejectsInvalidHessian) {
  QpProblem7 problem = validProblem();
  problem.H(0, 0) = -1.0;
  EXPECT_EQ(guard().validateProblem(problem).reason, HoldReason::kInvalidHessian);
}

TEST(SafetyGuard, RejectsSolverFailureAndNeverReusesVelocity) {
  SolverResult7 result;
  result.status = SolverStatus::kMaxIterations;
  result.qdot = Vec7::Constant(0.5);
  EXPECT_EQ(guard().validateResult(validProblem(), result).reason, HoldReason::kSolverFailure);
}

TEST(SafetyGuard, RejectsNonFiniteAndOutOfBoundsSolutions) {
  SolverResult7 result;
  result.status = SolverStatus::kSolved;
  result.qdot.setZero();
  result.qdot[0] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(guard().validateResult(validProblem(), result).reason, HoldReason::kNonFiniteSolution);

  result.qdot.setZero();
  result.qdot[4] = 1.0 + 2e-8;
  EXPECT_EQ(guard().validateResult(validProblem(), result).reason, HoldReason::kBoundViolation);
}

TEST(SafetyGuard, NamesEqualityViolations) {
  EXPECT_EQ(toString(HoldReason::kEqualityViolation), "equality_violation");
}

}  // namespace
}  // namespace tianji_qp_ik
