#include "tianji_qp_ik/osqp_solver.hpp"
#include "tianji_qp_ik/qpoases_solver.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

namespace tianji_qp_ik {
namespace {

double objective(const QpProblem7& problem, const Vec7& solution) {
  return 0.5 * solution.dot(problem.H * solution) + problem.g.dot(solution);
}

double projectedKktResidual(const QpProblem7& problem, const Vec7& solution) {
  constexpr double kActiveTolerance = 1e-7;
  const Vec7 gradient = problem.H * solution + problem.g;
  double residual = 0.0;
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (solution[joint] <= problem.lower[joint] + kActiveTolerance) {
      residual = std::max(residual, std::max(0.0, -gradient[joint]));
    } else if (solution[joint] >= problem.upper[joint] - kActiveTolerance) {
      residual = std::max(residual, std::max(0.0, gradient[joint]));
    } else {
      residual = std::max(residual, std::abs(gradient[joint]));
    }
  }
  return residual;
}

TEST(SolverCrosscheck, AgreesOnOneThousandDeterministicStronglyConvexProblems) {
  OsqpConfig osqp_config;
  osqp_config.max_iterations = 3000;
  osqp_config.absolute_tolerance = 1e-9;
  osqp_config.relative_tolerance = 1e-9;
  osqp_config.polishing = true;
  QpoasesConfig qpoases_config;
  qpoases_config.max_working_set_recalculations = 200;
  qpoases_config.cpu_time_limit_seconds = 0.02;

  std::mt19937 generator(6174U);
  std::normal_distribution<double> normal(0.0, 1.0);
  std::uniform_real_distribution<double> center_distribution(-0.5, 0.5);
  std::uniform_real_distribution<double> width_distribution(0.05, 1.0);

  QpProblem7 first;
  first.H = Mat77::Identity();
  first.g.setZero();
  first.lower = Vec7::Constant(-1.0);
  first.upper = Vec7::Constant(1.0);
  OsqpSolver7 osqp(osqp_config);
  QpoasesSolver7 qpoases(qpoases_config);
  ASSERT_TRUE(osqp.initialize(first));
  ASSERT_TRUE(qpoases.initialize(first));

  double maximum_solution_difference = 0.0;
  double maximum_objective_difference = 0.0;
  double maximum_bound_violation = 0.0;
  double maximum_kkt_residual = 0.0;
  for (int sample = 0; sample < 1000; ++sample) {
    Mat77 factor;
    Vec7 gradient;
    QpProblem7 problem;
    for (int row = 0; row < kArmDof; ++row) {
      gradient[row] = normal(generator);
      const double center = center_distribution(generator);
      const double width = width_distribution(generator);
      problem.lower[row] = center - width;
      problem.upper[row] = center + width;
      for (int column = 0; column < kArmDof; ++column) {
        factor(row, column) = normal(generator);
      }
    }
    problem.H.noalias() = factor.transpose() * factor;
    problem.H.diagonal().array() += 0.1;
    problem.g = gradient;

    const SolverResult7 osqp_result = osqp.solve(problem);
    const SolverResult7 qpoases_result = qpoases.solve(problem);
    ASSERT_EQ(osqp_result.status, SolverStatus::kSolved) << "sample=" << sample;
    ASSERT_EQ(qpoases_result.status, SolverStatus::kSolved)
        << "sample=" << sample << " detail=" << qpoases_result.detail;

    maximum_solution_difference =
        std::max(maximum_solution_difference,
                 (osqp_result.qdot - qpoases_result.qdot).cwiseAbs().maxCoeff());
    maximum_objective_difference =
        std::max(maximum_objective_difference,
                 std::abs(objective(problem, osqp_result.qdot) -
                          objective(problem, qpoases_result.qdot)));
    for (const Vec7* solution : {&osqp_result.qdot, &qpoases_result.qdot}) {
      maximum_kkt_residual =
          std::max(maximum_kkt_residual, projectedKktResidual(problem, *solution));
      maximum_bound_violation =
          std::max(maximum_bound_violation, (problem.lower - *solution).maxCoeff());
      maximum_bound_violation =
          std::max(maximum_bound_violation, (*solution - problem.upper).maxCoeff());
    }
  }

  std::cout << "solver_crosscheck_seed=6174 maximum_solution_difference="
            << maximum_solution_difference
            << " maximum_objective_difference=" << maximum_objective_difference
            << " maximum_kkt_residual=" << maximum_kkt_residual
            << " maximum_bound_violation=" << maximum_bound_violation << '\n';
  EXPECT_LT(maximum_solution_difference, 1e-5);
  EXPECT_LT(maximum_objective_difference, 1e-7);
  EXPECT_LT(maximum_kkt_residual, 1e-5);
  EXPECT_LT(maximum_bound_violation, 1e-8);
}

}  // namespace
}  // namespace tianji_qp_ik
