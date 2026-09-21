#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/qp_solver.hpp"

#include <osqp.h>

#include <array>
#include <string_view>

namespace tianji_qp_ik {

class OsqpSolver7 final : public IQpSolver7 {
 public:
  explicit OsqpSolver7(OsqpConfig config);
  ~OsqpSolver7() override;

  OsqpSolver7(const OsqpSolver7&) = delete;
  OsqpSolver7& operator=(const OsqpSolver7&) = delete;

  bool initialize(const QpProblem7& initial_problem) override;
  SolverResult7 solve(const QpProblem7& problem) override;
  void reset() override;
  std::string_view name() const noexcept override { return "osqp"; }

  int setupCount() const noexcept { return setup_count_; }

 private:
  static constexpr int kHessianEntries = kArmDof * (kArmDof + 1) / 2;

  void initializeSparsity();
  void copyProblemData(const QpProblem7& problem);
  SolverStatus mapStatus(OSQPInt status) const;

  OsqpConfig config_;
  OSQPSolver* solver_{nullptr};
  OSQPCscMatrix hessian_matrix_{};
  OSQPCscMatrix constraint_matrix_{};
  std::array<OSQPFloat, kHessianEntries> hessian_values_{};
  std::array<OSQPInt, kHessianEntries> hessian_rows_{};
  std::array<OSQPInt, kArmDof + 1> hessian_columns_{};
  std::array<OSQPFloat, kArmDof> identity_values_{};
  std::array<OSQPInt, kArmDof> identity_rows_{};
  std::array<OSQPInt, kArmDof + 1> identity_columns_{};
  std::array<OSQPFloat, kArmDof> gradient_{};
  std::array<OSQPFloat, kArmDof> lower_{};
  std::array<OSQPFloat, kArmDof> upper_{};
  std::array<OSQPFloat, kArmDof> last_solution_{};
  bool has_solution_{false};
  int setup_count_{0};
};

}  // namespace tianji_qp_ik
