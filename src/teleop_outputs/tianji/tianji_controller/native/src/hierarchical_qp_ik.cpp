#include "tianji_qp_ik/hierarchical_qp_ik.hpp"

#include "tianji_qp_ik/arm_angle.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace tianji_qp_ik {
namespace {

std::string_view holdDetail(HoldReason reason) {
  switch (reason) {
    case HoldReason::kNone:
      return "none";
    case HoldReason::kNonFiniteProblem:
      return "non_finite_problem";
    case HoldReason::kInfeasibleBounds:
      return "infeasible_bounds";
    case HoldReason::kInvalidHessian:
      return "invalid_hessian";
    case HoldReason::kSolverFailure:
      return "solver_failure";
    case HoldReason::kNonFiniteSolution:
      return "non_finite_solution";
    case HoldReason::kBoundViolation:
      return "bound_violation";
    case HoldReason::kEqualityViolation:
      return "equality_violation";
    case HoldReason::kReferenceTrackingError:
      return "reference_tracking_error";
  }
  return "unknown";
}

void fillBoundDiagnostics(const ArmIkInput& input, double tolerance,
                          ArmIkResult& result) {
  const auto count_source = [&result](BoundSource source) {
    switch (source) {
      case BoundSource::kPosition:
        ++result.active_position_bound_count;
        break;
      case BoundSource::kVelocity:
        ++result.active_velocity_bound_count;
        break;
      case BoundSource::kAcceleration:
        ++result.active_acceleration_bound_count;
        break;
      case BoundSource::kJerk:
        ++result.active_jerk_bound_count;
        break;
      case BoundSource::kBraking:
        ++result.active_braking_bound_count;
        break;
    }
  };
  for (int index = 0; index < kArmDof; ++index) {
    const bool lower_active =
        std::abs(result.qdot[index] - input.bounds.lower[index]) <= tolerance;
    const bool upper_active =
        std::abs(result.qdot[index] - input.bounds.upper[index]) <= tolerance;
    if (lower_active) {
      count_source(input.bounds.lower_source[static_cast<std::size_t>(index)]);
    }
    if (upper_active) {
      count_source(input.bounds.upper_source[static_cast<std::size_t>(index)]);
    }
    if (input.limits.velocity[index] > 0.0) {
      result.qdot_max_ratio = std::max(
          result.qdot_max_ratio,
          std::abs(result.qdot[index]) / input.limits.velocity[index]);
    }
  }
}

}  // namespace

HierarchicalQpIk7::HierarchicalQpIk7(
    HierarchicalQpConfig config, QpoasesConfig qpoases_config,
    SafetyConfig safety_config)
    : HierarchicalQpIk7(
          config, safety_config,
          std::make_unique<HierarchicalQpoasesSolver>(qpoases_config)) {}

HierarchicalQpIk7::HierarchicalQpIk7(
    HierarchicalQpConfig config, SafetyConfig safety_config,
    std::unique_ptr<IHierarchicalQpSolver> solver)
    : config_(config),
      safety_config_(safety_config),
      builder_(config),
      solver_(std::move(solver)) {}

ArmIkResult HierarchicalQpIk7::solve(const ArmIkInput& input) {
  ArmIkResult result;
  const HierarchicalQpProblem problem = builder_.build(input);
  const SafetyDecision problem_decision =
      validateHierarchicalProblem(problem, safety_config_);
  if (!problem_decision.accepted) {
    result.detail = holdDetail(problem_decision.reason);
    return result;
  }

  HierarchicalQpSolution solution;
  SafetyDecision solution_decision{
      false, HoldReason::kSolverFailure, -1};
  int total_iterations = 0;
  double total_solve_time_us = 0.0;
  bool cold_retry_solved = false;
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (!initialized_) {
      initialized_ = solver_->initialize(problem);
      if (!initialized_) {
        solution.status = SolverStatus::kNumericalError;
        solution.detail = attempt == 0
                              ? "hierarchical QP initialization failed"
                              : "hierarchical QP cold retry initialization failed";
        solution_decision =
            {false, HoldReason::kSolverFailure, -1};
      }
    }
    if (initialized_) {
      solution = solver_->solve(problem);
      total_iterations += solution.iterations;
      total_solve_time_us += solution.solve_time_us;
      solution_decision = validateHierarchicalSolution(
          problem, solution, config_, safety_config_);
      if (solution_decision.accepted) {
        cold_retry_solved = attempt > 0;
        break;
      }
    }
    if (attempt == 0) {
      solver_->reset();
      initialized_ = false;
    }
  }
  result.iterations = total_iterations;
  result.solve_time_us = total_solve_time_us;
  if (!solution_decision.accepted) {
    result.status = solution.status == SolverStatus::kSolved
                        ? SolverStatus::kNumericalError
                        : solution.status;
    result.detail = holdDetail(solution_decision.reason);
    reset();
    return result;
  }

  result.status = solution.status;
  if (config_.task_scaling_enabled) {
    result.task_scale_position = solution.x[kBetaPositionIndex];
    result.task_scale_orientation = solution.x[kBetaOrientationIndex];
  }
  const Vec7 primary = solution.x.head<kArmDof>();
  if (input.posture_task.active &&
      input.posture_task.source ==
          JointVelocityPostureSource::kLegacyNullspace) {
    result.qdot = refinePostureVelocityInNullspace(
                      primary, input.jacobian, input.posture_task,
                      input.bounds.lower, input.bounds.upper,
                      safety_config_.bound_tolerance,
                      input.linear_constraint,
                      input.branch_lock_constraint)
                      .value;
  } else if (input.posture_task.active &&
             (input.posture_task.source ==
                  JointVelocityPostureSource::kSparkSoftQp ||
              input.posture_task.source ==
                  JointVelocityPostureSource::
                      kSparkFeedforwardJointReference)) {
    result.qdot = primary;
  } else if (config_.arm_angle_weight > 0.0 &&
             input.arm_angle_task.active) {
    // The weighted arm-angle objective has already participated in the same
    // constrained QP as Cartesian tracking. Applying the historical exact
    // null-space correction again would discard that soft trade-off and can
    // drive the one redundant DoF directly onto a joint bound.
    result.qdot = primary;
  } else {
    result.qdot = refineArmAngleInNullspace(
                      primary, input.jacobian, input.arm_angle_task,
                      input.bounds.lower, input.bounds.upper,
                      safety_config_.bound_tolerance,
                      input.linear_constraint,
                      input.arm_angle_position_guard,
                      input.branch_lock_constraint)
                      .value;
  }
  Vec6 scaled_desired_twist = input.desired_twist;
  if (config_.task_scaling_enabled) {
    scaled_desired_twist.head<3>() *= result.task_scale_position;
    scaled_desired_twist.tail<3>() *= result.task_scale_orientation;
  }
  result.slack = scaled_desired_twist - input.jacobian * result.qdot;
  result.equality_residual =
      (input.jacobian * result.qdot + result.slack -
       scaled_desired_twist).norm();
  result.detail = cold_retry_solved
                      ? "hierarchical QP cold retry solved"
                      : solution.detail;
  fillBoundDiagnostics(input, safety_config_.bound_tolerance, result);
  return result;
}

void HierarchicalQpIk7::reset() {
  solver_->reset();
  initialized_ = false;
}

}  // namespace tianji_qp_ik
