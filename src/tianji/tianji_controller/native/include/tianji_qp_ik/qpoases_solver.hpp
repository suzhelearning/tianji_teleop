#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/qp_solver.hpp"

#include <qpOASES.hpp>

#include <array>
#include <memory>
#include <string_view>

namespace tianji_qp_ik {

class QpoasesSolver7 final : public IQpSolver7 {
 public:
  explicit QpoasesSolver7(QpoasesConfig config);
  ~QpoasesSolver7() override = default;

  QpoasesSolver7(const QpoasesSolver7&) = delete;
  QpoasesSolver7& operator=(const QpoasesSolver7&) = delete;

  bool initialize(const QpProblem7& initial_problem) override;
  SolverResult7 solve(const QpProblem7& problem) override;
  void reset() override;
  std::string_view name() const noexcept override { return "qpoases"; }

  int setupCount() const noexcept { return setup_count_; }
  int hotstartCount() const noexcept { return hotstart_count_; }

 private:
  void copyProblemData(const QpProblem7& problem);
  bool matchesInitializedProblem(const QpProblem7& problem) const noexcept;
  SolverStatus mapStatus(qpOASES::returnValue status) const;

  QpoasesConfig config_;
  std::unique_ptr<qpOASES::SQProblem> solver_;
  std::array<qpOASES::real_t, kArmDof * kArmDof> hessian_{};
  std::array<qpOASES::real_t, kArmDof> gradient_{};
  std::array<qpOASES::real_t, kArmDof> lower_{};
  std::array<qpOASES::real_t, kArmDof> upper_{};
  QpProblem7 initialized_problem_;
  Vec7 initialized_solution_{Vec7::Zero()};
  int initialized_iterations_{0};
  bool initialized_solution_pending_{false};
  bool initialized_{false};
  int setup_count_{0};
  int hotstart_count_{0};
};

}  // namespace tianji_qp_ik
