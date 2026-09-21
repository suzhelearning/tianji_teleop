#include "tianji_mapped_palm/pico_mapped_arm_angle_vector_governor.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace tianji_mapped_palm {
namespace {

constexpr double kBasisEpsilon = 1.0e-10;

double smoothStep(double value) noexcept {
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

bool validConfig(
    const PicoMappedArmAngleVectorGovernorConfig& config) noexcept {
  return std::isfinite(config.observability_min) &&
         config.observability_min > 0.0 &&
         std::isfinite(config.maximum_cartesian_leakage) &&
         config.maximum_cartesian_leakage > 0.0 &&
         std::isfinite(config.maximum_velocity_rad_s) &&
         config.maximum_velocity_rad_s > 0.0 &&
         std::isfinite(config.maximum_acceleration_rad_s2) &&
         config.maximum_acceleration_rad_s2 > 0.0 &&
         std::isfinite(config.maximum_jerk_rad_s3) &&
         config.maximum_jerk_rad_s3 > 0.0 &&
         std::isfinite(config.reference_weight) &&
         config.reference_weight >= 0.0 &&
         std::isfinite(config.continuity_weight) &&
         config.continuity_weight >= 0.0 &&
         std::isfinite(config.acceleration_weight) &&
         config.acceleration_weight >= 0.0 &&
         std::isfinite(config.jerk_weight) && config.jerk_weight >= 0.0 &&
         config.reference_weight + config.continuity_weight +
                 config.acceleration_weight + config.jerk_weight >
             0.0 &&
         std::isfinite(config.basis_rotation_full_health_rad_s) &&
         config.basis_rotation_full_health_rad_s > 0.0 &&
         std::isfinite(config.basis_rotation_zero_health_rad_s) &&
         config.basis_rotation_zero_health_rad_s >
             config.basis_rotation_full_health_rad_s;
}

}  // namespace

Vec7 orientPicoMappedNullspaceBasis(const Vec7& candidate,
                                    const Vec7& previous,
                                    bool previous_valid,
                                    bool* flipped) noexcept {
  const bool should_flip = previous_valid && candidate.allFinite() &&
                           previous.allFinite() &&
                           candidate.dot(previous) < 0.0;
  if (flipped != nullptr) {
    *flipped = should_flip;
  }
  return should_flip ? -candidate : candidate;
}

PicoMappedArmAngleVectorGovernor7::PicoMappedArmAngleVectorGovernor7(
    PicoMappedArmAngleVectorGovernorConfig config)
    : config_(std::move(config)) {
  if (!validConfig(config_)) {
    throw std::invalid_argument(
        "invalid mapped PICO arm-angle vector governor config");
  }
}

PicoMappedArmAngleVectorState
PicoMappedArmAngleVectorGovernor7::disabled(
    PicoMappedVectorDisableReason reason) noexcept {
  reset();
  PicoMappedArmAngleVectorState state;
  state.disable_reason = reason;
  return state;
}

PicoMappedArmAngleVectorState PicoMappedArmAngleVectorGovernor7::update(
    const PicoMappedArmAngleVectorInput& input) {
  if (!input.active) {
    return disabled(PicoMappedVectorDisableReason::kInactive);
  }
  if (input.epoch_reset) {
    return disabled(PicoMappedVectorDisableReason::kEpochReset);
  }
  if (input.stale) {
    return disabled(PicoMappedVectorDisableReason::kStaleInput);
  }
  if (!std::isfinite(input.dt) || input.dt <= 0.0) {
    return disabled(PicoMappedVectorDisableReason::kInvalidDt);
  }
  if (!input.cartesian_jacobian.allFinite() ||
      !input.arm_angle_jacobian.allFinite() ||
      !input.desired_twist.allFinite() ||
      !input.qdot_previous.allFinite() ||
      !input.qddot_previous.allFinite() ||
      !input.lower_velocity_bound.allFinite() ||
      !input.upper_velocity_bound.allFinite() ||
      !std::isfinite(input.requested_arm_rate) ||
      !std::isfinite(input.minimum_predicted_task_scale) ||
      input.minimum_predicted_task_scale < 0.0 ||
      input.minimum_predicted_task_scale > 1.0 ||
      !std::isfinite(input.task_health) || input.task_health < 0.0 ||
      input.task_health > 1.0 ||
      (input.lower_velocity_bound.array() >
       input.upper_velocity_bound.array()).any()) {
    return disabled(PicoMappedVectorDisableReason::kNonFiniteInput);
  }

  const Eigen::JacobiSVD<Mat67> svd(
      input.cartesian_jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success ||
      !svd.singularValues().allFinite() || !svd.matrixV().allFinite()) {
    return disabled(PicoMappedVectorDisableReason::kInvalidSvd);
  }
  const double sigma_max = svd.singularValues().maxCoeff();
  const double rank_tolerance = Eigen::NumTraits<double>::epsilon() *
      static_cast<double>(kArmDof) * std::max(1.0, sigma_max);
  int rank = 0;
  for (const double sigma : svd.singularValues()) {
    if (sigma > rank_tolerance) {
      ++rank;
    }
  }
  if (rank != 6 || svd.matrixV().cols() != kArmDof) {
    return disabled(PicoMappedVectorDisableReason::kInvalidSvd);
  }

  Vec7 candidate = svd.matrixV().col(6);
  if (!candidate.allFinite() || candidate.norm() <= kBasisEpsilon) {
    return disabled(PicoMappedVectorDisableReason::kInvalidSvd);
  }
  candidate.normalize();
  const Vec7 prior_basis = previous_basis_;
  const bool prior_basis_valid = previous_basis_valid_;
  bool flipped = false;
  const Vec7 basis = orientPicoMappedNullspaceBasis(
      candidate, prior_basis, prior_basis_valid, &flipped);
  const double leakage = (input.cartesian_jacobian * basis).norm();
  if (!std::isfinite(leakage) ||
      leakage > config_.maximum_cartesian_leakage) {
    return disabled(PicoMappedVectorDisableReason::kCartesianLeakage);
  }
  const double observability = input.arm_angle_jacobian.dot(basis);
  if (!std::isfinite(observability) ||
      std::abs(observability) < config_.observability_min) {
    return disabled(PicoMappedVectorDisableReason::kLowObservability);
  }

  const Vec7 full_qdot_task = svd.solve(input.desired_twist);
  if (!full_qdot_task.allFinite()) {
    return disabled(PicoMappedVectorDisableReason::kInvalidSvd);
  }
  const auto scalar_interval = [&](double task_scale, double& lower,
                                   double& upper) {
    const Vec7 qdot_task = task_scale * full_qdot_task;
    lower = -config_.maximum_velocity_rad_s;
    upper = config_.maximum_velocity_rad_s;
    for (int joint = 0; joint < kArmDof; ++joint) {
      if (std::abs(basis[joint]) <= kBasisEpsilon) {
        if (qdot_task[joint] < input.lower_velocity_bound[joint] ||
            qdot_task[joint] > input.upper_velocity_bound[joint]) {
          return false;
        }
        continue;
      }
      double one = (input.lower_velocity_bound[joint] - qdot_task[joint]) /
                   basis[joint];
      double two = (input.upper_velocity_bound[joint] - qdot_task[joint]) /
                   basis[joint];
      if (one > two) {
        std::swap(one, two);
      }
      lower = std::max(lower, one);
      upper = std::min(upper, two);
    }
    return std::isfinite(lower) && std::isfinite(upper) && lower <= upper;
  };

  double predicted_task_scale = 1.0;
  bool exact_prediction_feasible = true;
  double alpha_lower = 0.0;
  double alpha_upper = 0.0;
  if (!scalar_interval(predicted_task_scale, alpha_lower, alpha_upper)) {
    bool found = false;
    double infeasible_scale = 1.0;
    double feasible_scale = input.minimum_predicted_task_scale;
    constexpr int kScaleSearchSteps = 64;
    for (int step = 1; step <= kScaleSearchSteps; ++step) {
      const double candidate_scale =
          1.0 - (1.0 - input.minimum_predicted_task_scale) *
                    static_cast<double>(step) /
                    static_cast<double>(kScaleSearchSteps);
      if (scalar_interval(candidate_scale, alpha_lower, alpha_upper)) {
        feasible_scale = candidate_scale;
        found = true;
        break;
      }
      infeasible_scale = candidate_scale;
    }
    if (!found) {
      exact_prediction_feasible = false;
      predicted_task_scale = input.minimum_predicted_task_scale;
      alpha_lower = 0.0;
      alpha_upper = 0.0;
      for (int joint = 0; joint < kArmDof; ++joint) {
        const double projected_lower =
            basis[joint] >= 0.0 ? input.lower_velocity_bound[joint]
                                : input.upper_velocity_bound[joint];
        const double projected_upper =
            basis[joint] >= 0.0 ? input.upper_velocity_bound[joint]
                                : input.lower_velocity_bound[joint];
        alpha_lower += basis[joint] * projected_lower;
        alpha_upper += basis[joint] * projected_upper;
      }
      alpha_lower =
          std::max(alpha_lower, -config_.maximum_velocity_rad_s);
      alpha_upper =
          std::min(alpha_upper, config_.maximum_velocity_rad_s);
      if (alpha_lower > alpha_upper) {
        // Cartesian motion and derivative bounds may force the achieved
        // coefficient outside the governor's preferred rate range. Keep the
        // soft reference bounded and let the unchanged QP hard constraints
        // determine the achieved coefficient instead of toggling back to the
        // legacy scalar task.
        alpha_lower = -config_.maximum_velocity_rad_s;
        alpha_upper = config_.maximum_velocity_rad_s;
      }
    } else {
      for (int iteration = 0; iteration < 32; ++iteration) {
        const double candidate_scale =
            0.5 * (feasible_scale + infeasible_scale);
        double candidate_lower = 0.0;
        double candidate_upper = 0.0;
        if (scalar_interval(candidate_scale, candidate_lower,
                            candidate_upper)) {
          feasible_scale = candidate_scale;
          alpha_lower = candidate_lower;
          alpha_upper = candidate_upper;
        } else {
          infeasible_scale = candidate_scale;
        }
      }
      predicted_task_scale = feasible_scale;
    }
  }
  if (!std::isfinite(alpha_lower) || !std::isfinite(alpha_upper) ||
      alpha_lower > alpha_upper) {
    return disabled(PicoMappedVectorDisableReason::kInfeasibleScalarInterval);
  }
  Vec7 qdot_task;
  if (exact_prediction_feasible) {
    qdot_task = predicted_task_scale * full_qdot_task;
  } else {
    qdot_task =
        input.qdot_previous - basis * basis.dot(input.qdot_previous);
  }

  PicoMappedArmAngleVectorState state;
  state.valid = true;
  state.active = true;
  state.disable_reason = PicoMappedVectorDisableReason::kNone;
  state.basis = basis;
  state.qdot_task = qdot_task;
  state.basis_flipped = flipped;
  state.cartesian_leakage = leakage;
  state.arm_angle_observability = observability;
  state.alpha_feasible_lower = alpha_lower;
  state.alpha_feasible_upper = alpha_upper;
  state.alpha_raw = std::clamp(
      (input.requested_arm_rate -
       input.arm_angle_jacobian.dot(qdot_task)) /
          observability,
      alpha_lower, alpha_upper);
  const double raw_alpha_previous = basis.dot(input.qdot_previous);
  const double raw_alpha_trend =
      basis.dot(input.qdot_previous + input.qddot_previous * input.dt);
  state.alpha_previous = raw_alpha_previous;
  state.alpha_trend = raw_alpha_trend;

  double basis_health = 1.0;
  if (prior_basis_valid) {
    const double cosine = std::clamp(basis.dot(prior_basis), -1.0, 1.0);
    state.basis_rotation_rate_rad_s = std::acos(cosine) / input.dt;
    const double normalized =
        (state.basis_rotation_rate_rad_s -
         config_.basis_rotation_full_health_rad_s) /
        (config_.basis_rotation_zero_health_rad_s -
         config_.basis_rotation_full_health_rad_s);
    basis_health = 1.0 - smoothStep(normalized);
  }
  state.health = input.task_health * basis_health *
      (exact_prediction_feasible ? predicted_task_scale : 0.5);
  state.exact_prediction_feasible = exact_prediction_feasible;
  state.predicted_task_scale = predicted_task_scale;

  const Vec7 achieved_basis = prior_basis_valid ? prior_basis : basis;
  const Vec7 achieved_vector =
      achieved_basis * achieved_basis.dot(input.qdot_previous);
  Vec7 achieved_acceleration = Vec7::Zero();
  Vec7 achieved_jerk = Vec7::Zero();
  if (vector_history_valid_ && prior_basis_valid) {
    achieved_acceleration =
        (achieved_vector - previous_achieved_vector_) / input.dt;
    achieved_jerk =
        (achieved_acceleration - previous_achieved_acceleration_) / input.dt;
  }
  state.achieved_vector_velocity_norm = achieved_vector.norm();
  state.achieved_vector_acceleration_norm = achieved_acceleration.norm();
  state.achieved_vector_jerk_norm = achieved_jerk.norm();
  state.alpha_transport = basis.dot(achieved_vector);
  if (config_.basis_transport_enabled && vector_history_valid_ &&
      prior_basis_valid) {
    const double transported_trend =
        state.alpha_transport + basis.dot(achieved_acceleration) * input.dt;
    const double weight = std::clamp(config_.basis_transport_weight, 0.0, 1.0);
    state.alpha_previous =
        (1.0 - weight) * raw_alpha_previous + weight * state.alpha_transport;
    state.alpha_trend =
        (1.0 - weight) * raw_alpha_trend + weight * transported_trend;
    state.basis_transport_applied = true;
  }

  const double inverse_dt = 1.0 / input.dt;
  const double inverse_dt2 = inverse_dt * inverse_dt;
  const double inverse_dt3 = inverse_dt2 * inverse_dt;
  const double inverse_dt4 = inverse_dt2 * inverse_dt2;
  const double reference_weight = config_.reference_weight * state.health;
  const double projected_velocity =
      state.basis_transport_applied ? state.alpha_previous
                                    : basis.dot(achieved_vector);
  const double projected_acceleration =
      state.basis_transport_applied
          ? (state.alpha_trend - state.alpha_previous) / input.dt
          : basis.dot(achieved_acceleration);
  const double denominator =
      reference_weight + config_.continuity_weight +
      config_.acceleration_weight * inverse_dt2 +
      config_.jerk_weight * inverse_dt4;
  state.command_tracking_weight = denominator;
  const double numerator =
      reference_weight * state.alpha_raw +
      config_.continuity_weight * projected_velocity +
      config_.acceleration_weight * inverse_dt2 * projected_velocity +
      config_.jerk_weight *
          (inverse_dt4 * projected_velocity +
           inverse_dt3 * projected_acceleration);
  double target = std::clamp(
      denominator > 0.0 ? numerator / denominator : 0.0,
      alpha_lower, alpha_upper);

  if (!governor_valid_) {
    alpha_command_ =
        std::clamp(state.alpha_previous, alpha_lower, alpha_upper);
    alpha_acceleration_command_ = std::clamp(
        basis.dot(input.qddot_previous),
        -config_.maximum_acceleration_rad_s2,
        config_.maximum_acceleration_rad_s2);
    governor_valid_ = true;
  }
  if (config_.reversal_deadband_rad_s > 0.0 && governor_valid_ &&
      alpha_command_ * target < 0.0 &&
      std::abs(alpha_command_) < config_.reversal_deadband_rad_s) {
    target = 0.0;
    state.reversal_deadband_applied = true;
  }
  const double jerk_step = config_.maximum_jerk_rad_s3 * input.dt;
  const double acceleration_lower = std::max(
      -config_.maximum_acceleration_rad_s2,
      alpha_acceleration_command_ - jerk_step);
  const double acceleration_upper = std::min(
      config_.maximum_acceleration_rad_s2,
      alpha_acceleration_command_ + jerk_step);
  alpha_acceleration_command_ = std::clamp(
      (target - alpha_command_) / input.dt,
      acceleration_lower, acceleration_upper);
  const double previous_command = alpha_command_;
  alpha_command_ = std::clamp(
      alpha_command_ + alpha_acceleration_command_ * input.dt,
      alpha_lower, alpha_upper);
  alpha_acceleration_command_ =
      (alpha_command_ - previous_command) / input.dt;
  state.alpha_command = alpha_command_;
  state.alpha_acceleration_command = alpha_acceleration_command_;

  previous_basis_ = basis;
  previous_basis_valid_ = true;
  previous_achieved_vector_ = achieved_vector;
  previous_achieved_acceleration_ = achieved_acceleration;
  vector_history_valid_ = true;
  return state;
}

void PicoMappedArmAngleVectorGovernor7::reset() noexcept {
  previous_basis_.setZero();
  previous_basis_valid_ = false;
  previous_achieved_vector_.setZero();
  previous_achieved_acceleration_.setZero();
  vector_history_valid_ = false;
  alpha_command_ = 0.0;
  alpha_acceleration_command_ = 0.0;
  governor_valid_ = false;
}

}  // namespace tianji_mapped_palm
