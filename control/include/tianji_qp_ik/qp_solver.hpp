#pragma once

#include "tianji_qp_ik/types.hpp"

#include <string_view>

namespace tianji_qp_ik {

class IQpSolver7 {
 public:
  virtual ~IQpSolver7() = default;
  virtual bool initialize(const QpProblem7& initial_problem) = 0;
  virtual SolverResult7 solve(const QpProblem7& problem) = 0;
  virtual void reset() = 0;
  virtual std::string_view name() const noexcept = 0;
};

}  // namespace tianji_qp_ik
