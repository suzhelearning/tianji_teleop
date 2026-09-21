#pragma once

#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/types.hpp"

#include <string>

namespace tianji_mapped_palm {

enum class HoldReason {
  kNone,
  kNonFiniteProblem,
  kInfeasibleBounds,
  kInvalidHessian,
  kSolverFailure,
  kNonFiniteSolution,
  kBoundViolation,
  kEqualityViolation,
  kReferenceTrackingError,
};

struct SafetyDecision {
  bool accepted{false};
  HoldReason reason{HoldReason::kNonFiniteProblem};
  int joint_index{-1};
};

class SafetyGuard {
 public:
  explicit SafetyGuard(SafetyConfig config);

  SafetyDecision validateProblem(const QpProblem7& problem) const;
  SafetyDecision validateResult(const QpProblem7& problem, const SolverResult7& result) const;

 private:
  SafetyConfig config_;
};

std::string toString(HoldReason reason);

}  // namespace tianji_mapped_palm
