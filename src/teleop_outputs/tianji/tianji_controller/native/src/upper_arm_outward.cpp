#include "tianji_qp_ik/upper_arm_outward.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tianji_qp_ik {
namespace {

constexpr double kJacobianEpsilon = 1.0e-12;

bool validBounds(const Vec7& lower, const Vec7& upper) noexcept {
  return lower.allFinite() && upper.allFinite() &&
         (lower.array() <= upper.array()).all();
}

double maximumAchievable(const Vec7& jacobian, const Vec7& lower,
                         const Vec7& upper) noexcept {
  double maximum = 0.0;
  for (int index = 0; index < kArmDof; ++index) {
    maximum += jacobian[index] *
               (jacobian[index] >= 0.0 ? upper[index] : lower[index]);
  }
  return maximum;
}

double minimumAchievable(const Vec7& jacobian, const Vec7& lower,
                         const Vec7& upper) noexcept {
  double minimum = 0.0;
  for (int index = 0; index < kArmDof; ++index) {
    minimum += jacobian[index] *
               (jacobian[index] >= 0.0 ? lower[index] : upper[index]);
  }
  return minimum;
}

LinearJointConstraint boundedLowerConstraint(
    const UpperArmOutwardState& state, double requested_lower,
    double viability_lower,
    const Vec7& lower, const Vec7& upper) noexcept {
  LinearJointConstraint result;
  if (!state.valid || !state.jacobian.allFinite() ||
      state.jacobian.norm() <= kJacobianEpsilon ||
      !std::isfinite(requested_lower) || !validBounds(lower, upper)) {
    return result;
  }
  const double maximum = maximumAchievable(state.jacobian, lower, upper);
  const double minimum = minimumAchievable(state.jacobian, lower, upper);
  if (!std::isfinite(maximum) || !std::isfinite(minimum) ||
      minimum > maximum) {
    return result;
  }
  result.jacobian = state.jacobian;
  result.requested_lower = requested_lower;
  const double feasibility_margin =
      1.0e-7 * std::max(1.0, std::abs(maximum));
  if (requested_lower > maximum - feasibility_margin) {
    // A smooth recovery barrier may ask for more than the joint box can
    // provide while the one-step geometric viability bound remains feasible.
    // In that case retain only the viability lower bound. This preserves the
    // hard no-crossing invariant without forcing an active-set QP onto the
    // maximizing vertex of the complete joint box.
    result.upper = std::numeric_limits<double>::infinity();
    result.feasibility_clipped = true;
    result.viability_clipped =
        viability_lower > maximum - feasibility_margin;
    if (result.viability_clipped) {
      result.lower = maximum - feasibility_margin;
      // The hard kinematic bounds have priority when the one-step geometric
      // barrier is already unrecoverable.  Keep the strongest feasible
      // outward command active so the QP preserves qdot/qddot/jerk history
      // and monotonically recovers viability.  Treating this as inactive
      // makes the controller clear its dynamic history, which is both less
      // safe and discontinuous.
      result.active = true;
      return result;
    }
    constexpr double kRecoveryBoxReserveFraction = 0.05;
    const double recovery_reserve = std::max(
        feasibility_margin,
        kRecoveryBoxReserveFraction * std::max(0.0, maximum - minimum));
    const double interior_recovery_lower = maximum - recovery_reserve;
    result.lower = std::max(viability_lower, interior_recovery_lower);
    result.active = true;
    return result;
  }
  result.active = true;
  result.lower = requested_lower;
  result.upper = std::numeric_limits<double>::infinity();
  return result;
}

bool validConfig(const UpperArmOutwardConfig& config) noexcept {
  return std::isfinite(config.minimum_outward_distance_m) &&
         config.minimum_outward_distance_m >= 0.0 &&
         std::isfinite(config.velocity_gain) && config.velocity_gain > 0.0 &&
         std::isfinite(config.acceleration_kp) &&
         config.acceleration_kp >= 0.0 &&
         std::isfinite(config.acceleration_kd) &&
         config.acceleration_kd >= 0.0;
}

}  // namespace

Eigen::Vector3d upperArmOutwardDirection(ArmSide side) noexcept {
  if (side == ArmSide::kLeft) {
    return Eigen::Vector3d::UnitY();
  }
  return Eigen::Vector3d(0.0, -1.0, 0.0);
}

UpperArmOutwardState computeUpperArmOutwardState(
    ArmSide side, const Eigen::Vector3d& shoulder_position,
    const Eigen::Vector3d& elbow_position,
    const Mat37& shoulder_position_jacobian,
    const Mat37& elbow_position_jacobian,
    double minimum_outward_distance_m) noexcept {
  UpperArmOutwardState result;
  if (!shoulder_position.allFinite() || !elbow_position.allFinite() ||
      !shoulder_position_jacobian.allFinite() ||
      !elbow_position_jacobian.allFinite() ||
      !std::isfinite(minimum_outward_distance_m) ||
      minimum_outward_distance_m < 0.0) {
    return result;
  }
  const Eigen::Vector3d outward = upperArmOutwardDirection(side);
  result.distance_m =
      outward.dot(elbow_position - shoulder_position) -
      minimum_outward_distance_m;
  result.jacobian =
      (elbow_position_jacobian - shoulder_position_jacobian).transpose() *
      outward;
  result.valid = std::isfinite(result.distance_m) &&
                 result.jacobian.allFinite();
  return result;
}

LinearJointConstraint makeVelocityOutwardConstraint(
    const UpperArmOutwardState& state, const Vec7& lower,
    const Vec7& upper, const UpperArmOutwardConfig& config,
    double dt) noexcept {
  if (!validConfig(config) || !std::isfinite(dt) || dt <= 0.0) {
    return {};
  }
  // Keep the smooth first-order barrier, but add a one-cycle viability bound.
  // With d_next ~= d + dt * J*qdot, the command must satisfy J*qdot >= -d/dt
  // or the reference can cross the shoulder plane in one integration step.
  const double barrier_lower =
      -config.velocity_gain * state.distance_m;
  const double position_lower = -state.distance_m / dt;
  return boundedLowerConstraint(
      state, std::max(barrier_lower, position_lower), position_lower, lower,
      upper);
}

LinearJointConstraint makeAccelerationOutwardConstraint(
    const UpperArmOutwardState& state, const Vec7& qdot,
    double jdot_qdot, const Vec7& lower, const Vec7& upper,
    const UpperArmOutwardConfig& config, double dt) noexcept {
  if (!validConfig(config) || !qdot.allFinite() ||
      !std::isfinite(jdot_qdot) || !std::isfinite(dt) || dt <= 0.0) {
    return {};
  }
  const double outward_velocity = state.jacobian.dot(qdot);
  const double barrier_lower =
      -jdot_qdot - config.acceleration_kd * outward_velocity -
      config.acceleration_kp * state.distance_m;
  // With d_next ~= d + dt*v + 0.5*dt^2*(J*qddot + jdot*qdot),
  // enforce the same half-space at the next reference sample.
  const double position_lower =
      (-state.distance_m - dt * outward_velocity -
       0.5 * dt * dt * jdot_qdot) /
      (0.5 * dt * dt);
  const double requested_lower = std::max(barrier_lower, position_lower);
  return boundedLowerConstraint(state, requested_lower, position_lower, lower,
                                upper);
}

}  // namespace tianji_qp_ik
