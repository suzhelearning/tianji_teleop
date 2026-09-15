#include "tianji_qp_ik/qpoases_solver.hpp"

#include <chrono>
#include <cstddef>

namespace tianji_qp_ik {
namespace {

double elapsedMicroseconds(std::chrono::steady_clock::time_point start,
                           std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double, std::micro>(end - start).count();
}

}  // namespace

QpoasesSolver7::QpoasesSolver7(QpoasesConfig config) : config_(config) {}

void QpoasesSolver7::copyProblemData(const QpProblem7& problem) {
  for (int row = 0; row < kArmDof; ++row) {
    gradient_[static_cast<std::size_t>(row)] = problem.g[row];
    lower_[static_cast<std::size_t>(row)] = problem.lower[row];
    upper_[static_cast<std::size_t>(row)] = problem.upper[row];
    for (int column = 0; column < kArmDof; ++column) {
      hessian_[static_cast<std::size_t>(row * kArmDof + column)] = problem.H(row, column);
    }
  }
}

bool QpoasesSolver7::matchesInitializedProblem(const QpProblem7& problem) const noexcept {
  return (initialized_problem_.H.array() == problem.H.array()).all() &&
         (initialized_problem_.g.array() == problem.g.array()).all() &&
         (initialized_problem_.lower.array() == problem.lower.array()).all() &&
         (initialized_problem_.upper.array() == problem.upper.array()).all();
}

bool QpoasesSolver7::initialize(const QpProblem7& initial_problem) {
  initialized_solution_pending_ = false;
  copyProblemData(initial_problem);
  solver_ = std::make_unique<qpOASES::SQProblem>(kArmDof, 0, qpOASES::HST_POSDEF);
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
  const qpOASES::returnValue status =
      solver_->init(hessian_.data(), gradient_.data(), nullptr, lower_.data(), upper_.data(),
                    nullptr, nullptr, working_set_recalculations, &cpu_time);
  initialized_ = status == qpOASES::SUCCESSFUL_RETURN;
  if (initialized_) {
    std::array<qpOASES::real_t, kArmDof> solution{};
    if (solver_->getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN) {
      solver_.reset();
      initialized_ = false;
      return false;
    }
    initialized_problem_ = initial_problem;
    for (int index = 0; index < kArmDof; ++index) {
      initialized_solution_[index] = solution[static_cast<std::size_t>(index)];
    }
    initialized_iterations_ = static_cast<int>(working_set_recalculations);
    initialized_solution_pending_ = true;
    ++setup_count_;
  } else {
    solver_.reset();
  }
  return initialized_;
}

SolverResult7 QpoasesSolver7::solve(const QpProblem7& problem) {
  SolverResult7 result;
  result.qdot.setZero();
  if (!initialized_ || solver_ == nullptr) {
    result.status = SolverStatus::kInvalidInput;
    result.detail = "qpOASES solve requested before initialize";
    return result;
  }

  const auto update_start = std::chrono::steady_clock::now();
  if (initialized_solution_pending_ && matchesInitializedProblem(problem)) {
    initialized_solution_pending_ = false;
    result.status = SolverStatus::kSolved;
    result.qdot = initialized_solution_;
    result.iterations = initialized_iterations_;
    result.native_status_code = static_cast<int>(qpOASES::SUCCESSFUL_RETURN);
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
      hessian_.data(), gradient_.data(), nullptr, lower_.data(), upper_.data(), nullptr, nullptr,
      working_set_recalculations, &cpu_time);
  ++hotstart_count_;
  const auto solve_end = std::chrono::steady_clock::now();

  result.solve_time_us = elapsedMicroseconds(solve_start, solve_end);
  result.iterations = static_cast<int>(working_set_recalculations);
  result.native_status_code = static_cast<int>(status);
  result.status = mapStatus(status);
  result.detail = result.status == SolverStatus::kSolved ? "qpOASES_solved"
                                                         : "qpOASES_solve_failed";
  if (result.status == SolverStatus::kSolved) {
    std::array<qpOASES::real_t, kArmDof> solution{};
    if (solver_->getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN) {
      result.status = SolverStatus::kNumericalError;
      result.detail = "qpOASES failed to return primal solution";
      return result;
    }
    for (int index = 0; index < kArmDof; ++index) {
      result.qdot[index] = solution[static_cast<std::size_t>(index)];
    }
  }
  return result;
}

void QpoasesSolver7::reset() {
  solver_.reset();
  initialized_ = false;
  initialized_solution_pending_ = false;
}

SolverStatus QpoasesSolver7::mapStatus(qpOASES::returnValue status) const {
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
