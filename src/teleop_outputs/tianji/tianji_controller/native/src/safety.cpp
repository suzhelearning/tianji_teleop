#include "tianji_qp_ik/safety.hpp"

namespace tianji_qp_ik {

std::string toString(HoldReason reason) {
  switch (reason) {
    case HoldReason::kNone:
      return "none";
    case HoldReason::kNonFiniteProblem:
      return "non_finite_problem";
    case HoldReason::kInfeasibleBounds:
      return "infeasible_bounds";
    case HoldReason::kSolverFailure:
      return "solver_failure";
    case HoldReason::kNonFiniteSolution:
      return "non_finite_solution";
    case HoldReason::kBoundViolation:
      return "bound_violation";
    case HoldReason::kReferenceTrackingError:
      return "reference_tracking_error";
  }
  return "unknown";
}

}  // namespace tianji_qp_ik
