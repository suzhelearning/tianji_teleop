#pragma once

#include <string>

namespace tianji_qp_ik {

enum class HoldReason {
  kNone = 0,
  kNonFiniteProblem = 1,
  kInfeasibleBounds = 2,
  kSolverFailure = 4,
  kNonFiniteSolution = 5,
  kBoundViolation = 6,
  kReferenceTrackingError = 8,
};

struct SafetyDecision {
  bool accepted{false};
  HoldReason reason{HoldReason::kNonFiniteProblem};
  int joint_index{-1};
};

std::string toString(HoldReason reason);

}  // namespace tianji_qp_ik
