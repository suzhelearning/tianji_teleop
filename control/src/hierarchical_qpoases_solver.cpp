#include "tianji_qp_ik/hierarchical_qpoases_solver.hpp"

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

HierarchicalQpoasesSolver::HierarchicalQpoasesSolver(QpoasesConfig config)
    : config_(config) {}

void HierarchicalQpoasesSolver::copyProblemData(
    const HierarchicalQpProblem& problem) {
  for (int row = 0; row < kHierarchicalVariables; ++row) {
    const std::size_t row_index = static_cast<std::size_t>(row);
    gradient_[row_index] = problem.g[row];
    lower_[row_index] = std::isinf(problem.lower[row])
                            ? -qpOASES::INFTY
                            : problem.lower[row];
    upper_[row_index] = std::isinf(problem.upper[row])
                            ? qpOASES::INFTY
                            : problem.upper[row];
    for (int column = 0; column < kHierarchicalVariables; ++column) {
      const std::size_t offset = static_cast<std::size_t>(
          row * kHierarchicalVariables + column);
      hessian_[offset] = problem.H(row, column);
    }
  }

  for (int row = 0; row < kTrackingEqualities; ++row) {
    const std::size_t row_index = static_cast<std::size_t>(row);
    constraint_lower_[row_index] = problem.equality[row];
    constraint_upper_[row_index] = problem.equality[row];
    for (int column = 0; column < kHierarchicalVariables; ++column) {
      const std::size_t offset = static_cast<std::size_t>(
          row * kHierarchicalVariables + column);
      constraints_[offset] = problem.A(row, column);
    }
  }
  const auto copy_linear_constraint = [this](
      int row, const LinearJointConstraint& constraint) {
    const std::size_t row_index = static_cast<std::size_t>(row);
    constraint_lower_[row_index] =
        constraint.active ? constraint.lower : -qpOASES::INFTY;
    constraint_upper_[row_index] =
        constraint.active ? constraint.upper : qpOASES::INFTY;
    for (int column = 0; column < kHierarchicalVariables; ++column) {
      const std::size_t offset =
          row_index * kHierarchicalVariables +
          static_cast<std::size_t>(column);
      constraints_[offset] =
          column < kArmDof ? constraint.jacobian[column] : 0.0;
    }
  };
  copy_linear_constraint(kTrackingEqualities,
                         problem.linear_constraint);
  copy_linear_constraint(kTrackingEqualities + 1,
                         problem.branch_lock_constraint);
}

bool HierarchicalQpoasesSolver::matchesInitializedProblem(
    const HierarchicalQpProblem& problem) const noexcept {
  return (initialized_problem_.H.array() == problem.H.array()).all() &&
         (initialized_problem_.g.array() == problem.g.array()).all() &&
         (initialized_problem_.A.array() == problem.A.array()).all() &&
         (initialized_problem_.lower.array() == problem.lower.array()).all() &&
         (initialized_problem_.upper.array() == problem.upper.array()).all() &&
         (initialized_problem_.equality.array() == problem.equality.array()).all() &&
         initialized_problem_.linear_constraint.active ==
             problem.linear_constraint.active &&
         (initialized_problem_.linear_constraint.jacobian.array() ==
          problem.linear_constraint.jacobian.array()).all() &&
         initialized_problem_.linear_constraint.lower ==
             problem.linear_constraint.lower &&
         initialized_problem_.linear_constraint.upper ==
             problem.linear_constraint.upper &&
         initialized_problem_.branch_lock_constraint.active ==
             problem.branch_lock_constraint.active &&
         (initialized_problem_.branch_lock_constraint.jacobian.array() ==
          problem.branch_lock_constraint.jacobian.array()).all() &&
         initialized_problem_.branch_lock_constraint.lower ==
             problem.branch_lock_constraint.lower &&
         initialized_problem_.branch_lock_constraint.upper ==
             problem.branch_lock_constraint.upper;
}

bool HierarchicalQpoasesSolver::initialize(
    const HierarchicalQpProblem& problem) {
  initialized_solution_pending_ = false;
  copyProblemData(problem);
  solver_ = std::make_unique<qpOASES::SQProblem>(
      kHierarchicalVariables, kVelocityQpConstraints,
      qpOASES::HST_POSDEF);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  options.terminationTolerance = 1e-10;
  options.boundTolerance = 1e-10;
  options.enableRegularisation = qpOASES::BT_TRUE;
  solver_->setOptions(options);

  qpOASES::int_t working_set_recalculations =
      static_cast<qpOASES::int_t>(config_.max_working_set_recalculations);
  qpOASES::real_t cpu_time = config_.cpu_time_limit_seconds;
  const auto solve_start = std::chrono::steady_clock::now();
  const qpOASES::returnValue status = solver_->init(
      hessian_.data(), gradient_.data(), constraints_.data(), lower_.data(),
      upper_.data(), constraint_lower_.data(), constraint_upper_.data(),
      working_set_recalculations, &cpu_time);
  initialized_solve_time_us_ = elapsedMicroseconds(
      solve_start, std::chrono::steady_clock::now());
  initialized_ = status == qpOASES::SUCCESSFUL_RETURN;
  if (!initialized_) {
    solver_.reset();
    return false;
  }

  std::array<qpOASES::real_t, kHierarchicalVariables> solution{};
  if (solver_->getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN) {
    solver_.reset();
    initialized_ = false;
    return false;
  }
  initialized_problem_ = problem;
  for (int index = 0; index < kHierarchicalVariables; ++index) {
    initialized_solution_[index] = solution[static_cast<std::size_t>(index)];
  }
  initialized_iterations_ = static_cast<int>(working_set_recalculations);
  initialized_solution_pending_ = true;
  ++setup_count_;
  return true;
}

HierarchicalQpSolution HierarchicalQpoasesSolver::solve(
    const HierarchicalQpProblem& problem) {
  HierarchicalQpSolution result;
  if (!initialized_ || solver_ == nullptr) {
    result.status = SolverStatus::kInvalidInput;
    result.detail = "qpOASES solve requested before initialize";
    return result;
  }

  const auto update_start = std::chrono::steady_clock::now();
  if (initialized_solution_pending_ && matchesInitializedProblem(problem)) {
    initialized_solution_pending_ = false;
    result.status = SolverStatus::kSolved;
    result.x = initialized_solution_;
    result.iterations = initialized_iterations_;
    result.solve_time_us = initialized_solve_time_us_;
    result.detail = "qpOASES_initialized_solution";
    result.update_time_us =
        elapsedMicroseconds(update_start, std::chrono::steady_clock::now());
    return result;
  }

  initialized_solution_pending_ = false;
  copyProblemData(problem);
  const auto update_end = std::chrono::steady_clock::now();
  result.update_time_us = elapsedMicroseconds(update_start, update_end);

  qpOASES::int_t working_set_recalculations =
      static_cast<qpOASES::int_t>(config_.max_working_set_recalculations);
  qpOASES::real_t cpu_time = config_.cpu_time_limit_seconds;
  const auto solve_start = std::chrono::steady_clock::now();
  const qpOASES::returnValue status = solver_->hotstart(
      hessian_.data(), gradient_.data(), constraints_.data(), lower_.data(),
      upper_.data(), constraint_lower_.data(), constraint_upper_.data(),
      working_set_recalculations, &cpu_time);
  ++hotstart_count_;
  const auto solve_end = std::chrono::steady_clock::now();

  result.solve_time_us = elapsedMicroseconds(solve_start, solve_end);
  result.iterations = static_cast<int>(working_set_recalculations);
  result.status = mapStatus(status);
  result.detail = result.status == SolverStatus::kSolved
                      ? "qpOASES_solved"
                      : "qpOASES_solve_failed";
  if (result.status != SolverStatus::kSolved) {
    return result;
  }

  std::array<qpOASES::real_t, kHierarchicalVariables> solution{};
  if (solver_->getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN) {
    result.status = SolverStatus::kNumericalError;
    result.x.setZero();
    result.detail = "qpOASES failed to return primal solution";
    return result;
  }
  for (int index = 0; index < kHierarchicalVariables; ++index) {
    result.x[index] = solution[static_cast<std::size_t>(index)];
  }
  return result;
}

void HierarchicalQpoasesSolver::reset() {
  solver_.reset();
  initialized_ = false;
  initialized_solution_pending_ = false;
}

SolverStatus HierarchicalQpoasesSolver::mapStatus(
    qpOASES::returnValue status) const {
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
