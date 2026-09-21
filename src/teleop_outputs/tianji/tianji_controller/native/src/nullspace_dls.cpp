#include "tianji_qp_ik/nullspace_dls.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace tianji_qp_ik {
namespace {

bool validInput(const ArmIkInput& input) {
  if (!input.q_measured.allFinite() || !input.limits.lower_position.allFinite() ||
      !input.limits.upper_position.allFinite() ||
      !input.limits.velocity.allFinite() || !input.jacobian.allFinite() ||
      !input.desired_twist.allFinite() || !input.bounds.lower.allFinite() ||
      !input.bounds.upper.allFinite()) {
    return false;
  }
  return (input.bounds.lower.array() <= input.bounds.upper.array()).all();
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

NullspaceDlsIk7::NullspaceDlsIk7(DlsConfig config) : config_(config) {}

ArmIkResult NullspaceDlsIk7::solve(const ArmIkInput& input) {
  ArmIkResult result;
  const auto solve_start = std::chrono::steady_clock::now();
  if (!validInput(input)) {
    result.detail = "invalid DLS input";
    return result;
  }

  const double damping_squared = config_.damping * config_.damping;
  const Eigen::Matrix<double, 6, 6> regularized =
      input.jacobian * input.jacobian.transpose() +
      damping_squared * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> factorization(regularized);
  if (factorization.info() != Eigen::Success) {
    result.status = SolverStatus::kNumericalError;
    result.detail = "DLS factorization failed";
    return result;
  }
  const Eigen::Matrix<double, 6, 6> inverse_action =
      factorization.solve(Eigen::Matrix<double, 6, 6>::Identity());
  if (factorization.info() != Eigen::Success || !inverse_action.allFinite()) {
    result.status = SolverStatus::kNumericalError;
    result.detail = "DLS solve failed";
    return result;
  }
  const Eigen::Matrix<double, 7, 6> pinv =
      input.jacobian.transpose() * inverse_action;
  const Vec7 q_nominal =
      0.5 * (input.limits.lower_position + input.limits.upper_position);
  const Vec7 qdot_nominal =
      -config_.nominal_gain * (input.q_measured - q_nominal);
  const Vec7 raw = pinv * input.desired_twist +
                   (Mat77::Identity() - pinv * input.jacobian) * qdot_nominal;
  if (!raw.allFinite()) {
    result.status = SolverStatus::kNumericalError;
    result.detail = "non-finite DLS velocity";
    return result;
  }

  const Vec7 feasible_anchor =
      input.qdot_prev.cwiseMax(input.bounds.lower).cwiseMin(input.bounds.upper);
  const Vec7 direction = raw - feasible_anchor;
  double alpha = 1.0;
  for (int index = 0; index < kArmDof; ++index) {
    if (direction[index] > 0.0) {
      alpha = std::min(
          alpha, (input.bounds.upper[index] - feasible_anchor[index]) /
                     direction[index]);
    } else if (direction[index] < 0.0) {
      alpha = std::min(
          alpha, (input.bounds.lower[index] - feasible_anchor[index]) /
                     direction[index]);
    }
  }
  if (!std::isfinite(alpha) || alpha < 0.0) {
    result.detail = "DLS bounds cannot be satisfied from feasible anchor";
    return result;
  }
  alpha = std::clamp(alpha, 0.0, 1.0);
  result.qdot = feasible_anchor + alpha * direction;
  constexpr double kBoundTolerance = 1e-10;
  if ((result.qdot.array() < input.bounds.lower.array() - kBoundTolerance).any() ||
      (result.qdot.array() > input.bounds.upper.array() + kBoundTolerance).any()) {
    result.qdot.setZero();
    result.detail = "DLS scaled velocity violates bounds";
    return result;
  }

  result.slack = input.desired_twist - input.jacobian * result.qdot;
  result.equality_residual =
      (input.jacobian * result.qdot + result.slack -
       input.desired_twist).norm();
  result.status = SolverStatus::kSolved;
  result.detail = "nullspace_dls_solved";
  result.solve_time_us = std::chrono::duration<double, std::micro>(
                             std::chrono::steady_clock::now() - solve_start)
                             .count();
  fillBoundDiagnostics(input, kBoundTolerance, result);
  return result;
}

}  // namespace tianji_qp_ik
