#include "tianji_qp_ik/osqp_solver.hpp"

#include <chrono>
#include <cstddef>
#include <string>

namespace tianji_qp_ik {
namespace {

double elapsedMicroseconds(std::chrono::steady_clock::time_point start,
                           std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double, std::micro>(end - start).count();
}

std::string_view statusDetail(SolverStatus status) noexcept {
  switch (status) {
    case SolverStatus::kSolved:
      return "osqp_solved";
    case SolverStatus::kMaxIterations:
      return "osqp_max_iterations";
    case SolverStatus::kInfeasible:
      return "osqp_infeasible";
    case SolverStatus::kNumericalError:
      return "osqp_numerical_error";
    case SolverStatus::kInvalidInput:
      return "osqp_invalid_input";
  }
  return "osqp_unknown_status";
}

}  // namespace

OsqpSolver7::OsqpSolver7(OsqpConfig config) : config_(config) { initializeSparsity(); }

OsqpSolver7::~OsqpSolver7() {
  if (solver_ != nullptr) {
    osqp_cleanup(solver_);
  }
}

void OsqpSolver7::initializeSparsity() {
  int cursor = 0;
  hessian_columns_[0] = 0;
  for (int column = 0; column < kArmDof; ++column) {
    for (int row = 0; row <= column; ++row) {
      hessian_rows_[static_cast<std::size_t>(cursor)] = row;
      ++cursor;
    }
    hessian_columns_[static_cast<std::size_t>(column + 1)] = cursor;
    identity_rows_[static_cast<std::size_t>(column)] = column;
    identity_columns_[static_cast<std::size_t>(column)] = column;
    identity_values_[static_cast<std::size_t>(column)] = 1.0;
  }
  identity_columns_[kArmDof] = kArmDof;

  OSQPCscMatrix_set_data(&hessian_matrix_, kArmDof, kArmDof, kHessianEntries,
                         hessian_values_.data(), hessian_rows_.data(), hessian_columns_.data());
  OSQPCscMatrix_set_data(&constraint_matrix_, kArmDof, kArmDof, kArmDof,
                         identity_values_.data(), identity_rows_.data(), identity_columns_.data());
}

void OsqpSolver7::copyProblemData(const QpProblem7& problem) {
  int cursor = 0;
  for (int column = 0; column < kArmDof; ++column) {
    for (int row = 0; row <= column; ++row) {
      hessian_values_[static_cast<std::size_t>(cursor)] = problem.H(row, column);
      ++cursor;
    }
    gradient_[static_cast<std::size_t>(column)] = problem.g[column];
    lower_[static_cast<std::size_t>(column)] = problem.lower[column];
    upper_[static_cast<std::size_t>(column)] = problem.upper[column];
  }
}

bool OsqpSolver7::initialize(const QpProblem7& initial_problem) {
  if (solver_ != nullptr) {
    osqp_cleanup(solver_);
    solver_ = nullptr;
  }
  copyProblemData(initial_problem);

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.warm_starting = 1;
  settings.polishing = config_.polishing ? 1 : 0;
  settings.adaptive_rho = OSQP_ADAPTIVE_RHO_UPDATE_DISABLED;
  settings.max_iter = static_cast<OSQPInt>(config_.max_iterations);
  settings.eps_abs = config_.absolute_tolerance;
  settings.eps_rel = config_.relative_tolerance;
  settings.check_termination = 1;

  const OSQPInt result = osqp_setup(&solver_, &hessian_matrix_, gradient_.data(),
                                    &constraint_matrix_, lower_.data(), upper_.data(), kArmDof,
                                    kArmDof, &settings);
  has_solution_ = false;
  last_solution_.fill(0.0);
  if (result != OSQP_NO_ERROR || solver_ == nullptr) {
    solver_ = nullptr;
    return false;
  }
  ++setup_count_;
  return true;
}

SolverResult7 OsqpSolver7::solve(const QpProblem7& problem) {
  SolverResult7 result;
  result.qdot.setZero();
  if (solver_ == nullptr) {
    result.status = SolverStatus::kInvalidInput;
    result.detail = "OSQP solve requested before initialize";
    return result;
  }

  copyProblemData(problem);
  const auto update_start = std::chrono::steady_clock::now();
  const OSQPInt matrix_result = osqp_update_data_mat(
      solver_, hessian_values_.data(), nullptr, kHessianEntries, nullptr, nullptr, 0);
  const OSQPInt vector_result =
      osqp_update_data_vec(solver_, gradient_.data(), lower_.data(), upper_.data());
  if (has_solution_) {
    osqp_warm_start(solver_, last_solution_.data(), nullptr);
  }
  const auto update_end = std::chrono::steady_clock::now();
  result.update_time_us = elapsedMicroseconds(update_start, update_end);
  if (matrix_result != OSQP_NO_ERROR || vector_result != OSQP_NO_ERROR) {
    result.status = SolverStatus::kInvalidInput;
    result.detail = "OSQP data update failed";
    has_solution_ = false;
    return result;
  }

  const auto solve_start = std::chrono::steady_clock::now();
  const OSQPInt solve_result = osqp_solve(solver_);
  const auto solve_end = std::chrono::steady_clock::now();
  result.solve_time_us = elapsedMicroseconds(solve_start, solve_end);
  if (solve_result != OSQP_NO_ERROR || solver_->info == nullptr) {
    result.status = SolverStatus::kNumericalError;
    result.detail = "OSQP solve call failed";
    has_solution_ = false;
    return result;
  }

  result.status = mapStatus(solver_->info->status_val);
  result.iterations = static_cast<int>(solver_->info->iter);
  result.native_status_code = static_cast<int>(solver_->info->status_val);
  result.detail = statusDetail(result.status);
  if (result.status == SolverStatus::kSolved && solver_->solution != nullptr &&
      solver_->solution->x != nullptr) {
    for (int index = 0; index < kArmDof; ++index) {
      result.qdot[index] = solver_->solution->x[index];
      last_solution_[static_cast<std::size_t>(index)] = solver_->solution->x[index];
    }
    has_solution_ = true;
  } else {
    result.qdot.setZero();
    has_solution_ = false;
  }
  return result;
}

void OsqpSolver7::reset() {
  if (solver_ != nullptr) {
    osqp_cold_start(solver_);
  }
  has_solution_ = false;
  last_solution_.fill(0.0);
}

SolverStatus OsqpSolver7::mapStatus(OSQPInt status) const {
  switch (status) {
    case OSQP_SOLVED:
      return SolverStatus::kSolved;
    case OSQP_MAX_ITER_REACHED:
    case OSQP_TIME_LIMIT_REACHED:
      return SolverStatus::kMaxIterations;
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    case OSQP_DUAL_INFEASIBLE:
    case OSQP_DUAL_INFEASIBLE_INACCURATE:
      return SolverStatus::kInfeasible;
    default:
      return SolverStatus::kNumericalError;
  }
}

}  // namespace tianji_qp_ik
