#include "tianji_qp_ik/safety.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>

namespace tianji_qp_ik {

SafetyGuard::SafetyGuard(SafetyConfig config) : config_(config) {}

SafetyDecision SafetyGuard::validateProblem(const QpProblem7& problem) const {
  if (!problem.H.allFinite() || !problem.g.allFinite() || !problem.lower.allFinite() ||
      !problem.upper.allFinite()) {
    return {false, HoldReason::kNonFiniteProblem, -1};
  }

  for (int index = 0; index < kArmDof; ++index) {
    if (problem.lower[index] > problem.upper[index]) {
      return {false, HoldReason::kInfeasibleBounds, index};
    }
  }

  const double symmetry_error = (problem.H - problem.H.transpose()).cwiseAbs().maxCoeff();
  if (symmetry_error > config_.hessian_eigenvalue_tolerance) {
    return {false, HoldReason::kInvalidHessian, -1};
  }
  Eigen::SelfAdjointEigenSolver<Mat77> eigenvalues(problem.H, Eigen::EigenvaluesOnly);
  if (eigenvalues.info() != Eigen::Success ||
      eigenvalues.eigenvalues().minCoeff() <= config_.hessian_eigenvalue_tolerance) {
    return {false, HoldReason::kInvalidHessian, -1};
  }
  return {true, HoldReason::kNone, -1};
}

SafetyDecision SafetyGuard::validateResult(const QpProblem7& problem,
                                           const SolverResult7& result) const {
  const SafetyDecision problem_decision = validateProblem(problem);
  if (!problem_decision.accepted) {
    return problem_decision;
  }
  if (result.status != SolverStatus::kSolved) {
    return {false, HoldReason::kSolverFailure, -1};
  }
  if (!result.qdot.allFinite()) {
    return {false, HoldReason::kNonFiniteSolution, -1};
  }
  for (int index = 0; index < kArmDof; ++index) {
    if (result.qdot[index] < problem.lower[index] - config_.bound_tolerance ||
        result.qdot[index] > problem.upper[index] + config_.bound_tolerance) {
      return {false, HoldReason::kBoundViolation, index};
    }
  }
  return {true, HoldReason::kNone, -1};
}

std::string toString(HoldReason reason) {
  switch (reason) {
    case HoldReason::kNone:
      return "none";
    case HoldReason::kNonFiniteProblem:
      return "non_finite_problem";
    case HoldReason::kInfeasibleBounds:
      return "infeasible_bounds";
    case HoldReason::kInvalidHessian:
      return "invalid_hessian";
    case HoldReason::kSolverFailure:
      return "solver_failure";
    case HoldReason::kNonFiniteSolution:
      return "non_finite_solution";
    case HoldReason::kBoundViolation:
      return "bound_violation";
    case HoldReason::kEqualityViolation:
      return "equality_violation";
    case HoldReason::kReferenceTrackingError:
      return "reference_tracking_error";
  }
  return "unknown";
}

}  // namespace tianji_qp_ik
