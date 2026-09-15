#include "tianji_mapped_palm/cartesian_feedforward_allocator.hpp"

#include "tianji_mapped_palm/cartesian_servo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tianji_mapped_palm {
namespace {

bool finiteInput(const ArmIkInput& input, const Vec6& feedback,
                 const CartesianFeedforwardDemand& demand) noexcept {
  return demand.valid && feedback.allFinite() &&
         demand.low_frequency.allFinite() &&
         demand.high_frequency.allFinite() &&
         std::isfinite(demand.legacy_low_scale) &&
         demand.legacy_low_scale >= 0.0 && demand.legacy_low_scale <= 1.0 &&
         std::isfinite(demand.legacy_high_scale) &&
         demand.legacy_high_scale >= 0.0 && demand.legacy_high_scale <= 1.0 &&
         input.q_measured.allFinite() && input.qdot_prev.allFinite() &&
         input.qddot_prev.allFinite() && input.jacobian.allFinite() &&
         input.bounds.lower.allFinite() && input.bounds.upper.allFinite() &&
         input.limits.velocity.allFinite() && std::isfinite(input.dt) &&
         input.dt > 0.0;
}

double safeRatio(double numerator, double denominator) noexcept {
  if (!std::isfinite(numerator) || !std::isfinite(denominator) ||
      denominator <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::abs(numerator) / denominator;
}

}  // namespace

std::string_view toString(CartesianFeedforwardLimit limit) noexcept {
  switch (limit) {
    case CartesianFeedforwardLimit::kNone:
      return "none";
    case CartesianFeedforwardLimit::kInvalidInput:
      return "invalid_input";
    case CartesianFeedforwardLimit::kPreviewSolverFailure:
      return "preview_solver_failure";
    case CartesianFeedforwardLimit::kPreviewNonFinite:
      return "preview_nonfinite";
    case CartesianFeedforwardLimit::kTaskScale:
      return "preview_task_scale";
    case CartesianFeedforwardLimit::kExecutionResidual:
      return "preview_execution_residual";
    case CartesianFeedforwardLimit::kDirection:
      return "preview_direction";
    case CartesianFeedforwardLimit::kVelocityBudget:
      return "preview_velocity_budget";
    case CartesianFeedforwardLimit::kAccelerationBudget:
      return "preview_acceleration_budget";
    case CartesianFeedforwardLimit::kJerkBudget:
      return "preview_jerk_budget";
    case CartesianFeedforwardLimit::kArmAngle:
      return "preview_arm_angle";
    case CartesianFeedforwardLimit::kNullspaceLeakage:
      return "preview_nullspace_leakage";
    case CartesianFeedforwardLimit::kBudgetTimeout:
      return "budget_timeout";
  }
  return "unknown";
}

CartesianFeedforwardAllocator7::CartesianFeedforwardAllocator7(
    CartesianFeedforwardAllocatorConfig config,
    JointLimitConfig dynamic_limits, CartesianServoConfig servo,
    std::unique_ptr<IArmVelocityIk> preview_solver)
    : config_(config),
      dynamic_limits_(std::move(dynamic_limits)),
      servo_(std::move(servo)),
      preview_solver_(std::move(preview_solver)) {
  if (preview_solver_ == nullptr) {
    throw std::invalid_argument(
        "Cartesian feedforward allocator requires a preview solver");
  }
}

void CartesianFeedforwardAllocator7::reset() noexcept {
  initialized_ = false;
  previous_low_scale_ = 0.0;
  previous_high_scale_ = 0.0;
  preview_solver_->reset();
}

bool CartesianFeedforwardAllocator7::timedOut() const noexcept {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      start_time_)
             .count() > config_.preview_budget_seconds_per_arm;
}

CartesianFeedforwardAllocator7::Evaluation
CartesianFeedforwardAllocator7::evaluate(
    const ArmIkInput& base_input, const Vec6& feedback_twist,
    const CartesianFeedforwardDemand& demand, double low_scale,
    double high_scale, const Evaluation* legacy, int& solve_count) {
  Evaluation evaluation;
  if (solve_count >= config_.max_preview_solves || timedOut()) {
    evaluation.limit = CartesianFeedforwardLimit::kBudgetTimeout;
    return evaluation;
  }
  ArmIkInput candidate = base_input;
  candidate.desired_twist = clampCartesianTwist(
      servo_, feedback_twist + low_scale * demand.low_frequency +
                  high_scale * demand.high_frequency);
  evaluation.preview = preview_solver_->solve(candidate);
  ++solve_count;
  if (timedOut()) {
    evaluation.limit = CartesianFeedforwardLimit::kBudgetTimeout;
    return evaluation;
  }
  if (evaluation.preview.status != SolverStatus::kSolved) {
    evaluation.limit = CartesianFeedforwardLimit::kPreviewSolverFailure;
    return evaluation;
  }
  if (!evaluation.preview.qdot.allFinite() ||
      !evaluation.preview.slack.allFinite() ||
      !std::isfinite(evaluation.preview.task_scale_position) ||
      !std::isfinite(evaluation.preview.task_scale_orientation)) {
    evaluation.limit = CartesianFeedforwardLimit::kPreviewNonFinite;
    return evaluation;
  }

  const Vec6 executed = candidate.jacobian * evaluation.preview.qdot;
  const Vec6 residual = candidate.desired_twist - executed;
  evaluation.normalized_residual = std::hypot(
      residual.head<3>().norm() / servo_.max_linear_velocity,
      residual.tail<3>().norm() / servo_.max_angular_velocity);
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
  if (candidate.arm_angle_task.active) {
    evaluation.arm_angle_residual = std::abs(
        candidate.arm_angle_task.jacobian.dot(evaluation.preview.qdot) -
        candidate.arm_angle_task.target);
  }
  if (candidate.vector_nullspace_task.active) {
    evaluation.arm_angle_residual = std::max(
        evaluation.arm_angle_residual,
        std::abs(candidate.vector_nullspace_task.basis.dot(
                     evaluation.preview.qdot) -
                 candidate.vector_nullspace_task.alpha_command));
    evaluation.nullspace_leakage =
        (candidate.jacobian * candidate.vector_nullspace_task.basis).norm();
  }
  if (!std::isfinite(evaluation.normalized_residual) ||
      !std::isfinite(evaluation.velocity_usage) ||
      !std::isfinite(evaluation.acceleration_usage) ||
      !std::isfinite(evaluation.jerk_usage) ||
      !std::isfinite(evaluation.arm_angle_residual) ||
      !std::isfinite(evaluation.nullspace_leakage)) {
    evaluation.limit = CartesianFeedforwardLimit::kPreviewNonFinite;
    return evaluation;
  }

  if (legacy != nullptr) {
    if (evaluation.preview.task_scale_position <
            legacy->preview.task_scale_position -
                config_.task_scale_tolerance ||
        evaluation.preview.task_scale_orientation <
            legacy->preview.task_scale_orientation -
                config_.task_scale_tolerance) {
      evaluation.limit = CartesianFeedforwardLimit::kTaskScale;
      return evaluation;
    }
    if (evaluation.normalized_residual >
        std::max(legacy->normalized_residual +
                     config_.normalized_residual_tolerance,
                 config_.normalized_residual_floor)) {
      evaluation.limit = CartesianFeedforwardLimit::kExecutionResidual;
      return evaluation;
    }
    const Vec6 feedforward_increment =
        (low_scale - demand.legacy_low_scale) * demand.low_frequency +
        (high_scale - demand.legacy_high_scale) * demand.high_frequency;
    const Vec6 execution_increment =
        candidate.jacobian *
        (evaluation.preview.qdot - legacy->preview.qdot);
    if (feedforward_increment.dot(execution_increment) <
        -config_.normalized_residual_tolerance) {
      evaluation.limit = CartesianFeedforwardLimit::kDirection;
      return evaluation;
    }
    const double usage_limit =
        std::max(config_.maximum_soft_bound_usage, legacy->velocity_usage);
    if (evaluation.velocity_usage > usage_limit) {
      evaluation.limit = CartesianFeedforwardLimit::kVelocityBudget;
      return evaluation;
    }
    if (evaluation.acceleration_usage >
        std::max(config_.maximum_soft_bound_usage,
                 legacy->acceleration_usage)) {
      evaluation.limit = CartesianFeedforwardLimit::kAccelerationBudget;
      return evaluation;
    }
    if (evaluation.jerk_usage >
        std::max(config_.maximum_soft_bound_usage, legacy->jerk_usage)) {
      evaluation.limit = CartesianFeedforwardLimit::kJerkBudget;
      return evaluation;
    }
    if (evaluation.arm_angle_residual >
        legacy->arm_angle_residual + config_.arm_angle_residual_tolerance) {
      evaluation.limit = CartesianFeedforwardLimit::kArmAngle;
      return evaluation;
    }
  }
  if (evaluation.nullspace_leakage > config_.maximum_nullspace_leakage) {
    evaluation.limit = CartesianFeedforwardLimit::kNullspaceLeakage;
    return evaluation;
  }
  evaluation.accepted = true;
  return evaluation;
}

CartesianFeedforwardAllocationResult CartesianFeedforwardAllocator7::allocate(
    const ArmIkInput& base_input, const Vec6& feedback_twist,
    const CartesianFeedforwardDemand& demand) {
  CartesianFeedforwardAllocationResult result;
  result.low_scale = demand.legacy_low_scale;
  result.high_scale = demand.legacy_high_scale;
  if (demand.low_frequency.allFinite() &&
      demand.high_frequency.allFinite() &&
      std::isfinite(demand.legacy_low_scale) &&
      std::isfinite(demand.legacy_high_scale)) {
    result.allocated_feedforward =
        demand.legacy_low_scale * demand.low_frequency +
        demand.legacy_high_scale * demand.high_frequency;
  }
  if (!finiteInput(base_input, feedback_twist, demand)) {
    result.limiting_factor = CartesianFeedforwardLimit::kInvalidInput;
    return result;
  }
  if (config_.mode == PicoEeFeedforwardAllocatorMode::kLegacy) {
    result.valid = true;
    return result;
  }

  start_time_ = std::chrono::steady_clock::now();
  int solve_count = 0;
  const Evaluation legacy =
      evaluate(base_input, feedback_twist, demand, demand.legacy_low_scale,
               demand.legacy_high_scale, nullptr, solve_count);
  if (!legacy.accepted) {
    result.preview_solve_count = solve_count;
    result.compute_time_us =
        std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - start_time_)
            .count();
    result.limiting_factor = legacy.limit;
    return result;
  }
  result.legacy_preview_qdot = legacy.preview.qdot;

  double selected_low = demand.legacy_low_scale;
  double selected_high = demand.legacy_high_scale;
  Evaluation selected = legacy;
  CartesianFeedforwardLimit last_limit = CartesianFeedforwardLimit::kNone;
  if (initialized_) {
    const auto search_channel = [&](bool low_channel, double lower,
                                    double upper) {
      if (upper <= lower + 1.0e-15) {
        return lower;
      }
      const auto evaluate_scale = [&](double scale) {
        return evaluate(base_input, feedback_twist, demand,
                        low_channel ? scale : selected_low,
                        low_channel ? selected_high : scale, &legacy,
                        solve_count);
      };
      Evaluation upper_evaluation = evaluate_scale(upper);
      if (upper_evaluation.accepted) {
        selected = upper_evaluation;
        return upper;
      }
      last_limit = upper_evaluation.limit;
      double accepted = lower;
      double rejected = upper;
      for (int iteration = 0; iteration < config_.bisection_steps;
           ++iteration) {
        const double middle = 0.5 * (accepted + rejected);
        Evaluation middle_evaluation = evaluate_scale(middle);
        if (middle_evaluation.accepted) {
          accepted = middle;
          selected = middle_evaluation;
        } else {
          rejected = middle;
          last_limit = middle_evaluation.limit;
        }
        if (middle_evaluation.limit ==
            CartesianFeedforwardLimit::kBudgetTimeout) {
          break;
        }
      }
      return accepted;
    };

    const double low_upper = std::max(
        demand.legacy_low_scale,
        std::min(1.0, previous_low_scale_ +
                          config_.maximum_scale_increase_per_cycle));
    if (demand.low_frequency.squaredNorm() > 0.0) {
      selected_low =
          search_channel(true, demand.legacy_low_scale, low_upper);
    }
    const double high_upper = std::max(
        demand.legacy_high_scale,
        std::min(1.0, previous_high_scale_ +
                          config_.maximum_scale_increase_per_cycle));
    if (last_limit != CartesianFeedforwardLimit::kBudgetTimeout &&
        demand.high_frequency.squaredNorm() > 0.0) {
      selected_high =
          search_channel(false, demand.legacy_high_scale, high_upper);
    }
  }

  result.preview_solve_count = solve_count;
  result.compute_time_us =
      std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - start_time_)
          .count();
  if (last_limit == CartesianFeedforwardLimit::kBudgetTimeout || timedOut()) {
    result.limiting_factor = CartesianFeedforwardLimit::kBudgetTimeout;
    initialized_ = false;
    return result;
  }

  initialized_ = true;
  previous_low_scale_ = selected_low;
  previous_high_scale_ = selected_high;
  result.valid = true;
  result.limiting_factor = last_limit;
  result.preview_qdot = selected.preview.qdot;
  result.normalized_execution_residual = selected.normalized_residual;
  result.velocity_usage = selected.velocity_usage;
  result.acceleration_usage = selected.acceleration_usage;
  result.jerk_usage = selected.jerk_usage;
  result.arm_angle_residual = selected.arm_angle_residual;
  result.nullspace_leakage = selected.nullspace_leakage;
  if (config_.mode == PicoEeFeedforwardAllocatorMode::kActive) {
    result.low_scale = selected_low;
    result.high_scale = selected_high;
    result.allocated_feedforward =
        selected_low * demand.low_frequency +
        selected_high * demand.high_frequency;
    result.candidate_selected =
        selected_low > demand.legacy_low_scale + 1.0e-15 ||
        selected_high > demand.legacy_high_scale + 1.0e-15;
  }
  return result;
}

}  // namespace tianji_mapped_palm
