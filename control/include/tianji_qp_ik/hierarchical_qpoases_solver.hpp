#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/hierarchical_qp.hpp"

#include <qpOASES.hpp>

#include <array>
#include <memory>

namespace tianji_qp_ik {

class IHierarchicalQpSolver {
 public:
  virtual ~IHierarchicalQpSolver() = default;
  virtual bool initialize(const HierarchicalQpProblem& problem) = 0;
  virtual HierarchicalQpSolution solve(const HierarchicalQpProblem& problem) = 0;
  virtual void reset() = 0;
};

class HierarchicalQpoasesSolver final : public IHierarchicalQpSolver {
 public:
  explicit HierarchicalQpoasesSolver(QpoasesConfig config);
  ~HierarchicalQpoasesSolver() override = default;

  HierarchicalQpoasesSolver(const HierarchicalQpoasesSolver&) = delete;
  HierarchicalQpoasesSolver& operator=(const HierarchicalQpoasesSolver&) = delete;

  bool initialize(const HierarchicalQpProblem& problem) override;
  HierarchicalQpSolution solve(const HierarchicalQpProblem& problem) override;
  void reset() override;

  int setupCount() const noexcept { return setup_count_; }
  int hotstartCount() const noexcept { return hotstart_count_; }

 private:
  void copyProblemData(const HierarchicalQpProblem& problem);
  bool matchesInitializedProblem(const HierarchicalQpProblem& problem) const noexcept;
  SolverStatus mapStatus(qpOASES::returnValue status) const;

  QpoasesConfig config_;
  std::unique_ptr<qpOASES::SQProblem> solver_;
  std::array<qpOASES::real_t,
             kHierarchicalVariables * kHierarchicalVariables> hessian_{};
  std::array<qpOASES::real_t, kHierarchicalVariables> gradient_{};
  std::array<qpOASES::real_t,
             kVelocityQpConstraints * kHierarchicalVariables> constraints_{};
  std::array<qpOASES::real_t, kHierarchicalVariables> lower_{};
  std::array<qpOASES::real_t, kHierarchicalVariables> upper_{};
  std::array<qpOASES::real_t, kVelocityQpConstraints> constraint_lower_{};
  std::array<qpOASES::real_t, kVelocityQpConstraints> constraint_upper_{};
  HierarchicalQpProblem initialized_problem_;
  VecVelocityQp initialized_solution_{VecVelocityQp::Zero()};
  int initialized_iterations_{0};
  double initialized_solve_time_us_{0.0};
  bool initialized_solution_pending_{false};
  bool initialized_{false};
  int setup_count_{0};
  int hotstart_count_{0};
};

}  // namespace tianji_qp_ik
