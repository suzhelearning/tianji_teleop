#include "tianji_qp_ik/equality_constrained_qpoases_solver.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>

namespace tianji_qp_ik {
namespace {

double elapsedMicroseconds(std::chrono::steady_clock::time_point start,
                           std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double, std::micro>(end - start).count();
}

}  // namespace

EqualityConstrainedQpoasesSolver7::EqualityConstrainedQpoasesSolver7(
    QpoasesConfig config)
    : config_(config) {}

bool EqualityConstrainedQpoasesSolver7::valid(
    const EqualityConstrainedQpProblem7& problem) const noexcept {
  return problem.H.allFinite() && problem.g.allFinite() &&
         problem.A.allFinite() && problem.equality.allFinite() &&
         problem.lower.allFinite() && problem.upper.allFinite() &&
         (problem.lower.array() <= problem.upper.array()).all();
}

void EqualityConstrainedQpoasesSolver7::copyProblemData(
    const EqualityConstrainedQpProblem7& problem) {
  for (int row = 0; row < kArmDof; ++row) {
    const std::size_t row_index = static_cast<std::size_t>(row);
    gradient_[row_index] = problem.g[row];
    lower_[row_index] = problem.lower[row];
    upper_[row_index] = problem.upper[row];
    for (int column = 0; column < kArmDof; ++column) {
      hessian_[static_cast<std::size_t>(row * kArmDof + column)] =
          problem.H(row, column);
    }
  }
  for (int row = 0; row < kEqualityConstrainedQpRows; ++row) {
    const std::size_t row_index = static_cast<std::size_t>(row);
    equality_lower_[row_index] = problem.equality[row];
    equality_upper_[row_index] = problem.equality[row];
    for (int column = 0; column < kArmDof; ++column) {
      constraints_[static_cast<std::size_t>(row * kArmDof + column)] =
          problem.A(row, column);
    }
  }
}

qpOASES::returnValue EqualityConstrainedQpoasesSolver7::initialize() {
  solver_ = std::make_unique<qpOASES::SQProblem>(
      kArmDof, kEqualityConstrainedQpRows, qpOASES::HST_POSDEF);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  options.terminationTolerance = 1.0e-10;
  options.boundTolerance = 1.0e-10;
  options.enableRegularisation = qpOASES::BT_TRUE;
  solver_->setOptions(options);
  qpOASES::int_t nwsr = static_cast<qpOASES::int_t>(
      config_.max_working_set_recalculations);
  qpOASES::real_t cpu_time = config_.cpu_time_limit_seconds;
  const qpOASES::returnValue status = solver_->init(
      hessian_.data(), gradient_.data(), constraints_.data(), lower_.data(),
      upper_.data(), equality_lower_.data(), equality_upper_.data(), nwsr,
      &cpu_time);
  initialized_ = status == qpOASES::SUCCESSFUL_RETURN;
  if (initialized_) {
    ++setup_count_;
  } else {
    solver_.reset();
  }
  return status;
}

SolverResult7 EqualityConstrainedQpoasesSolver7::solve(
    const EqualityConstrainedQpProblem7& problem) {
  SolverResult7 result;
  result.qdot.setZero();
  if (!valid(problem)) {
    result.status = SolverStatus::kInvalidInput;
    result.detail = "invalid_equality_constrained_qp";
    return result;
  }
  copyProblemData(problem);
  qpOASES::returnValue status = qpOASES::RET_QP_NOT_SOLVED;
  const auto solve_start = std::chrono::steady_clock::now();
  qpOASES::int_t nwsr = static_cast<qpOASES::int_t>(
      config_.max_working_set_recalculations);
  if (!initialized_ || solver_ == nullptr) {
    status = initialize();
  } else {
    qpOASES::real_t cpu_time = config_.cpu_time_limit_seconds;
    status = solver_->hotstart(
        hessian_.data(), gradient_.data(), constraints_.data(), lower_.data(),
        upper_.data(), equality_lower_.data(), equality_upper_.data(), nwsr,
        &cpu_time);
    ++hotstart_count_;
    if (status != qpOASES::SUCCESSFUL_RETURN) {
      reset();
      status = initialize();
    }
  }
  result.solve_time_us = elapsedMicroseconds(
      solve_start, std::chrono::steady_clock::now());
  result.iterations = static_cast<int>(nwsr);
  result.native_status_code = static_cast<int>(status);
  result.status = mapStatus(status);
  result.detail = result.status == SolverStatus::kSolved
                      ? "equality_constrained_qpOASES_solved"
                      : "equality_constrained_qpOASES_failed";
  if (result.status != SolverStatus::kSolved || solver_ == nullptr) {
    return result;
  }
  std::array<qpOASES::real_t, kArmDof> solution{};
  if (solver_->getPrimalSolution(solution.data()) !=
      qpOASES::SUCCESSFUL_RETURN) {
    result.status = SolverStatus::kNumericalError;
    result.detail = "equality_constrained_qpOASES_no_primal";
    return result;
  }
  for (int index = 0; index < kArmDof; ++index) {
    result.qdot[index] = solution[static_cast<std::size_t>(index)];
  }
  constexpr double kFeasibilityTolerance = 1.0e-7;
  if (!result.qdot.allFinite() ||
      ((problem.A * result.qdot - problem.equality).cwiseAbs().maxCoeff() >
       kFeasibilityTolerance) ||
      (result.qdot.array() <
       problem.lower.array() - kFeasibilityTolerance)
          .any() ||
      (result.qdot.array() >
       problem.upper.array() + kFeasibilityTolerance)
          .any()) {
    result.status = SolverStatus::kNumericalError;
    result.detail = "equality_constrained_qpOASES_invalid_solution";
    result.qdot.setZero();
  }
  return result;
}

void EqualityConstrainedQpoasesSolver7::reset() noexcept {
  solver_.reset();
  initialized_ = false;
}

SolverStatus EqualityConstrainedQpoasesSolver7::mapStatus(
    qpOASES::returnValue status) const noexcept {
  switch (status) {
    case qpOASES::SUCCESSFUL_RETURN:
      return SolverStatus::kSolved;
    case qpOASES::RET_MAX_NWSR_REACHED:
      return SolverStatus::kMaxIterations;
    case qpOASES::RET_QP_INFEASIBLE:
    case qpOASES::RET_INIT_FAILED_INFEASIBILITY:
    case qpOASES::RET_HOTSTART_STOPPED_INFEASIBILITY:
      return SolverStatus::kInfeasible;
    default:
      return SolverStatus::kNumericalError;
  }
}

}  // namespace tianji_qp_ik
