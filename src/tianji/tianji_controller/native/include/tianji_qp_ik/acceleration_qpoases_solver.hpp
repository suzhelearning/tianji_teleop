#pragma once

#include "tianji_qp_ik/acceleration_qp.hpp"
#include <qpOASES.hpp>

#include <array>
#include <memory>

namespace tianji_qp_ik {

class IAccelerationQpSolver {
 public:
  virtual ~IAccelerationQpSolver() = default;
  virtual bool initialize(const AccelerationQpProblem& problem) = 0;
  virtual AccelerationQpSolution solve(const AccelerationQpProblem& problem) = 0;
  virtual void reset() = 0;
};

class AccelerationQpoasesSolver final : public IAccelerationQpSolver {
 public:
  explicit AccelerationQpoasesSolver(QpoasesConfig config);

  bool initialize(const AccelerationQpProblem& problem) override;
  AccelerationQpSolution solve(const AccelerationQpProblem& problem) override;
  void reset() override;
  int setupCount() const noexcept { return setup_count_; }
  int hotstartCount() const noexcept { return hotstart_count_; }

 private:
  void copyProblemData(const AccelerationQpProblem& problem);
  bool matchesInitializedProblem(const AccelerationQpProblem& problem) const noexcept;
  SolverStatus mapStatus(qpOASES::returnValue status) const;

  QpoasesConfig config_;
  std::unique_ptr<qpOASES::SQProblem> solver_;
  std::array<qpOASES::real_t,
             kAccelerationVariables * kAccelerationVariables> hessian_{};
  std::array<qpOASES::real_t, kAccelerationVariables> gradient_{};
  std::array<qpOASES::real_t,
             kAccelerationQpConstraints * kAccelerationVariables> constraints_{};
  std::array<qpOASES::real_t, kAccelerationVariables> lower_{};
  std::array<qpOASES::real_t, kAccelerationVariables> upper_{};
  std::array<qpOASES::real_t, kAccelerationQpConstraints> constraint_lower_{};
  std::array<qpOASES::real_t, kAccelerationQpConstraints> constraint_upper_{};
  AccelerationQpProblem initialized_problem_;
  Vec15 initialized_solution_{Vec15::Zero()};
  int initialized_iterations_{0};
  double initialized_solve_time_us_{0.0};
  bool initialized_solution_pending_{false};
  bool initialized_{false};
  int setup_count_{0};
  int hotstart_count_{0};
};

}  // namespace tianji_qp_ik
