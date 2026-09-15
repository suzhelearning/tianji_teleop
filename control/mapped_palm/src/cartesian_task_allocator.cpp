#include "tianji_mapped_palm/cartesian_task_allocator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace tianji_mapped_palm {
namespace {

bool finiteInput(const ArmIkInput& input) noexcept {
  return input.q_measured.allFinite() && input.q_ref.allFinite() &&
         input.qdot_prev.allFinite() && input.qddot_prev.allFinite() &&
         input.slack_prev.allFinite() && input.jacobian.allFinite() &&
         input.desired_twist.allFinite() && input.bounds.lower.allFinite() &&
         input.bounds.upper.allFinite() && input.limits.velocity.allFinite() &&
         std::isfinite(input.dt) && input.dt > 0.0;
}

double safeRatio(double numerator, double denominator) noexcept {
  if (!std::isfinite(numerator) || !std::isfinite(denominator) ||
      denominator <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::abs(numerator) / denominator;
}

}  // namespace

std::string_view toString(CartesianTaskAllocationLimit limit) noexcept {
  switch (limit) {
    case CartesianTaskAllocationLimit::kNone:
      return "none";
    case CartesianTaskAllocationLimit::kInvalidInput:
      return "invalid_input";
    case CartesianTaskAllocationLimit::kPreviewSolverFailure:
      return "preview_solver_failure";
    case CartesianTaskAllocationLimit::kPreviewNonFinite:
      return "preview_nonfinite";
    case CartesianTaskAllocationLimit::kTaskScale:
      return "preview_task_scale";
    case CartesianTaskAllocationLimit::kExecutionResidual:
      return "preview_execution_residual";
    case CartesianTaskAllocationLimit::kVelocityBudget:
      return "preview_velocity_budget";
    case CartesianTaskAllocationLimit::kAccelerationBudget:
      return "preview_acceleration_budget";
    case CartesianTaskAllocationLimit::kJerkBudget:
      return "preview_jerk_budget";
    case CartesianTaskAllocationLimit::kBudgetTimeout:
      return "budget_timeout";
  }
  return "unknown";
}

CartesianTaskAllocator7::CartesianTaskAllocator7(
    CartesianTaskAllocationConfig config, CartesianServoConfig servo,
    std::unique_ptr<IArmVelocityIk> preview_solver)
    : CartesianTaskAllocator7(std::move(config), JointLimitConfig{},
                              std::move(servo), std::move(preview_solver)) {}

CartesianTaskAllocator7::CartesianTaskAllocator7(
    CartesianTaskAllocationConfig config, JointLimitConfig dynamic_limits,
    CartesianServoConfig servo, std::unique_ptr<IArmVelocityIk> preview_solver)
    : config_(std::move(config)),
      dynamic_limits_(std::move(dynamic_limits)),
      servo_(std::move(servo)),
      preview_solver_(std::move(preview_solver)) {
  if (preview_solver_ == nullptr) {
    throw std::invalid_argument(
        "Cartesian task allocator requires a preview solver");
  }
}

void CartesianTaskAllocator7::reset() noexcept { preview_solver_->reset(); }

bool CartesianTaskAllocator7::timedOut() const noexcept {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        start_time_)
             .count() > config_.preview_budget_seconds_per_arm;
}

CartesianTaskAllocator7::Evaluation CartesianTaskAllocator7::evaluate(
    const ArmIkInput& base_input, double scale, const Vec6* feedback_twist,
    int& solve_count, double nullspace_alpha_override) {
  Evaluation evaluation;
  if (solve_count >= config_.max_preview_solves || timedOut()) {
    evaluation.limit = CartesianTaskAllocationLimit::kBudgetTimeout;
    return evaluation;
  }

  ArmIkInput candidate = base_input;
  Vec6 requested_twist = base_input.desired_twist;
  if (config_.nullspace_only) {
    if (!base_input.vector_nullspace_task.active ||
        !base_input.vector_nullspace_task.basis.allFinite() ||
        base_input.vector_nullspace_task.basis.norm() <= 1.0e-12 ||
        !std::isfinite(base_input.vector_nullspace_task.alpha_command)) {
      evaluation.limit = CartesianTaskAllocationLimit::kInvalidInput;
      return evaluation;
    }
    candidate.vector_nullspace_task.alpha_command =
        std::isfinite(nullspace_alpha_override)
            ? nullspace_alpha_override
            : scale * base_input.vector_nullspace_task.alpha_command;
    evaluation.nullspace_alpha_command =
        candidate.vector_nullspace_task.alpha_command;
  } else {
    requested_twist = scale * base_input.desired_twist;
    if (config_.preserve_feedback && feedback_twist != nullptr) {
      requested_twist = *feedback_twist +
                        scale * (base_input.desired_twist - *feedback_twist);
    }
  }
  evaluation.desired_twist = config_.nullspace_only
                                  ? requested_twist
                                  : clampCartesianTwist(servo_, requested_twist);
  candidate.desired_twist = evaluation.desired_twist;
  evaluation.preview = preview_solver_->solve(candidate);
  ++solve_count;
  if (timedOut()) {
    evaluation.limit = CartesianTaskAllocationLimit::kBudgetTimeout;
    return evaluation;
  }
  if (evaluation.preview.status != SolverStatus::kSolved) {
    evaluation.limit = CartesianTaskAllocationLimit::kPreviewSolverFailure;
    return evaluation;
  }
  if (!evaluation.preview.qdot.allFinite() ||
      !evaluation.preview.slack.allFinite() ||
      !std::isfinite(evaluation.preview.task_scale_position) ||
      !std::isfinite(evaluation.preview.task_scale_orientation)) {
    evaluation.limit = CartesianTaskAllocationLimit::kPreviewNonFinite;
    return evaluation;
  }

  const Vec6 executed = candidate.jacobian * evaluation.preview.qdot;
  const Vec6 residual = candidate.desired_twist - executed;
  const double linear_scale = std::max(servo_.max_linear_velocity, 1.0e-9);
  const double angular_scale = std::max(servo_.max_angular_velocity, 1.0e-9);
  evaluation.normalized_residual = std::hypot(
      residual.head<3>().norm() / linear_scale,
      residual.tail<3>().norm() / angular_scale);

  const Vec7 acceleration =
      (evaluation.preview.qdot - candidate.qdot_prev) / candidate.dt;
  const Vec7 jerk = (acceleration - candidate.qddot_prev) / candidate.dt;
  for (int index = 0; index < kArmDof; ++index) {
    evaluation.velocity_usage = std::max(
        evaluation.velocity_usage,
        safeRatio(evaluation.preview.qdot[index],
                  dynamic_limits_.velocity_scale *
                      candidate.limits.velocity[index]));
    evaluation.acceleration_usage = std::max(
        evaluation.acceleration_usage,
        safeRatio(acceleration[index],
                  dynamic_limits_.max_acceleration_rad_s2[index]));
    evaluation.jerk_usage = std::max(
        evaluation.jerk_usage,
        safeRatio(jerk[index], dynamic_limits_.max_jerk_rad_s3[index]));
  }

  // The search cost is diagnostic/selection-only.  The final live command is
  // still produced by the unchanged Velocity-Level QP.  Normalize the
  // preview dynamics before applying the configured scalar weights so the
  // candidate ranking is not dominated by physical units.
  const double qdot_step_scale = std::max(
      1.0, (candidate.limits.velocity * dynamic_limits_.velocity_scale).norm());
  const double jerk_scale = std::max(
      1.0, dynamic_limits_.max_jerk_rad_s3.norm());
  const double qdot_step_cost =
      (evaluation.preview.qdot - candidate.qdot_prev).norm() /
      qdot_step_scale;
  const double jerk_cost = jerk.norm() / jerk_scale;
  const double alpha_scale = std::max(
      {config_.nullspace_search_step, 0.05,
       std::abs(base_input.vector_nullspace_task.alpha_command),
       std::abs(base_input.vector_nullspace_task.alpha_previous)});
  const double alpha_reference_cost =
      (evaluation.nullspace_alpha_command -
       base_input.vector_nullspace_task.alpha_command) /
      alpha_scale;
  double arm_rate_cost = 0.0;
  if (base_input.arm_angle_task.jacobian.allFinite() &&
      base_input.arm_angle_task.jacobian.norm() > 1.0e-12 &&
      std::isfinite(base_input.arm_angle_task.target)) {
    arm_rate_cost =
        (base_input.arm_angle_task.jacobian.dot(evaluation.preview.qdot) -
         base_input.arm_angle_task.target) /
        std::max(1.0, std::abs(base_input.arm_angle_task.target));
  }
  evaluation.selection_cost =
      config_.nullspace_reference_weight * alpha_reference_cost *
          alpha_reference_cost +
      config_.nullspace_continuity_weight * qdot_step_cost * qdot_step_cost +
      config_.nullspace_jerk_weight * jerk_cost * jerk_cost +
      config_.nullspace_arm_rate_weight * arm_rate_cost * arm_rate_cost;

  if (!std::isfinite(evaluation.normalized_residual) ||
      !std::isfinite(evaluation.velocity_usage) ||
      !std::isfinite(evaluation.acceleration_usage) ||
      !std::isfinite(evaluation.jerk_usage) ||
      !std::isfinite(evaluation.selection_cost)) {
    evaluation.limit = CartesianTaskAllocationLimit::kPreviewNonFinite;
    return evaluation;
  }
  if (evaluation.preview.task_scale_position <
          config_.minimum_preview_task_scale ||
      evaluation.preview.task_scale_orientation <
          config_.minimum_preview_task_scale) {
    evaluation.limit = CartesianTaskAllocationLimit::kTaskScale;
    return evaluation;
  }
  if (evaluation.normalized_residual >
      config_.normalized_residual_tolerance) {
    evaluation.limit = CartesianTaskAllocationLimit::kExecutionResidual;
    return evaluation;
  }
  if (evaluation.velocity_usage > config_.maximum_soft_bound_usage) {
    evaluation.limit = CartesianTaskAllocationLimit::kVelocityBudget;
    return evaluation;
  }
  if (evaluation.acceleration_usage > config_.maximum_soft_bound_usage) {
    evaluation.limit = CartesianTaskAllocationLimit::kAccelerationBudget;
    return evaluation;
  }
  if (evaluation.jerk_usage > config_.maximum_soft_bound_usage) {
    evaluation.limit = CartesianTaskAllocationLimit::kJerkBudget;
    return evaluation;
  }
  evaluation.accepted = true;
  return evaluation;
}

CartesianTaskAllocationResult CartesianTaskAllocator7::allocate(
    const ArmIkInput& base_input) {
  return allocateImpl(base_input, nullptr);
}

CartesianTaskAllocationResult CartesianTaskAllocator7::allocate(
    const ArmIkInput& base_input, const Vec6& feedback_twist) {
  return allocateImpl(base_input, &feedback_twist);
}

CartesianTaskAllocationResult CartesianTaskAllocator7::allocateImpl(
    const ArmIkInput& base_input, const Vec6* feedback_twist) {
  CartesianTaskAllocationResult result;
  result.predicted_desired_twist = base_input.desired_twist;
  const auto start = std::chrono::steady_clock::now();
  start_time_ = start;

  if (!finiteInput(base_input) ||
      (feedback_twist != nullptr && !feedback_twist->allFinite()) ||
      !std::isfinite(config_.minimum_task_scale) ||
      config_.minimum_task_scale <= 0.0 ||
      config_.minimum_task_scale > 1.0 ||
      !std::isfinite(config_.minimum_preview_task_scale) ||
      config_.minimum_preview_task_scale <= 0.0 ||
      config_.minimum_preview_task_scale > 1.0 ||
      config_.bisection_steps < 0 || config_.max_preview_solves <= 0 ||
      !std::isfinite(config_.maximum_soft_bound_usage) ||
      config_.maximum_soft_bound_usage <= 0.0 ||
      config_.maximum_soft_bound_usage > 1.0 ||
      !std::isfinite(config_.normalized_residual_tolerance) ||
      config_.normalized_residual_tolerance < 0.0 ||
      !std::isfinite(config_.preview_budget_seconds_per_arm) ||
      config_.preview_budget_seconds_per_arm <= 0.0) {
    result.limiting_factor = CartesianTaskAllocationLimit::kInvalidInput;
    result.compute_time_us = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start)
            .count());
    return result;
  }

  if (config_.mode == PicoEeTaskAllocationMode::kDisabled) {
    result.valid = true;
    result.compute_time_us = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start)
            .count());
    return result;
  }

  int solve_count = 0;
  const Evaluation full =
      evaluate(base_input, 1.0, feedback_twist, solve_count);
  CartesianTaskAllocationLimit last_limit = full.limit;
  Evaluation best = full;
  double best_scale = 1.0;
  bool selected_nullspace_candidate = false;
  if (config_.nullspace_only && config_.nullspace_search_enabled) {
    bool found = false;
    const double commanded =
        base_input.vector_nullspace_task.alpha_command;
    const double previous = base_input.vector_nullspace_task.alpha_previous;
    const double trend = base_input.vector_nullspace_task.alpha_trend;
    const double step = config_.nullspace_search_step;
    const std::array<double, 8> candidates{
        commanded, previous, trend, 0.0, commanded + step,
        commanded - step, previous + step, previous - step};
    for (const double candidate_alpha : candidates) {
      if (!std::isfinite(candidate_alpha) || solve_count >=
          config_.max_preview_solves || timedOut()) {
        break;
      }
      const Evaluation candidate = evaluate(
          base_input, 1.0, feedback_twist, solve_count, candidate_alpha);
      last_limit = candidate.limit;
      if (candidate.accepted &&
          (!found || candidate.selection_cost < best.selection_cost)) {
        best = candidate;
        found = true;
      }
    }
    if (!found) {
      result.limiting_factor = last_limit;
      result.preview_solve_count = solve_count;
      result.compute_time_us = static_cast<double>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - start)
              .count());
      return result;
    }
    selected_nullspace_candidate = true;
  } else if (!full.accepted) {
    const double minimum = config_.minimum_task_scale;
    const Evaluation minimum_evaluation =
        evaluate(base_input, minimum, feedback_twist, solve_count);
    last_limit = minimum_evaluation.limit;
    if (!minimum_evaluation.accepted) {
      result.limiting_factor = last_limit;
      result.preview_solve_count = solve_count;
      result.compute_time_us = static_cast<double>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - start)
              .count());
      return result;
    }
    best = minimum_evaluation;
    best_scale = minimum;
    double lower = minimum;
    double upper = 1.0;
    for (int iteration = 0; iteration < config_.bisection_steps; ++iteration) {
      if (solve_count >= config_.max_preview_solves || timedOut()) {
        last_limit = CartesianTaskAllocationLimit::kBudgetTimeout;
        break;
      }
      const double middle = 0.5 * (lower + upper);
      const Evaluation candidate =
          evaluate(base_input, middle, feedback_twist, solve_count);
      if (candidate.accepted) {
        lower = middle;
        best = candidate;
        best_scale = middle;
      } else {
        upper = middle;
        last_limit = candidate.limit;
      }
    }
  }

  result.valid = best.accepted;
  result.authorized = result.valid &&
                      config_.mode == PicoEeTaskAllocationMode::kActive;
  result.predicted_task_scale = best_scale;
  result.predicted_nullspace_alpha_command =
      best.nullspace_alpha_command;
  result.predicted_desired_twist = best.desired_twist;
  result.preview_qdot = best.preview.qdot;
  result.normalized_residual = best.normalized_residual;
  result.velocity_usage = best.velocity_usage;
  result.acceleration_usage = best.acceleration_usage;
  result.jerk_usage = best.jerk_usage;
  result.preview_task_scale_position = best.preview.task_scale_position;
  result.preview_task_scale_orientation = best.preview.task_scale_orientation;
  result.preview_solve_count = solve_count;
  result.limiting_factor =
      selected_nullspace_candidate || best_scale < 1.0
          ? last_limit
          : CartesianTaskAllocationLimit::kNone;
  result.compute_time_us = static_cast<double>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - start)
          .count());
  return result;
}

}  // namespace tianji_mapped_palm
