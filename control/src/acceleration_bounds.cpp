#include "tianji_qp_ik/acceleration_bounds.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

double maximumAccelerationPreservingBrakingEnvelope(
    double distance, double velocity_toward_limit,
    double braking_acceleration, double dt) {
  // Enforce v_next^2 <= 2*b*d_next for motion toward a limit, with
  // v_next = v + a*dt and d_next = d - v*dt - 0.5*a*dt^2.
  // The upper root is the largest acceleration that keeps the next state
  // inside the same braking envelope, making the bound recursively feasible.
  const double a = dt * dt;
  const double b = 2.0 * velocity_toward_limit * dt +
                   braking_acceleration * dt * dt;
  const double c = velocity_toward_limit * velocity_toward_limit +
                   2.0 * braking_acceleration * velocity_toward_limit * dt -
                   2.0 * braking_acceleration * std::max(distance, 0.0);
  const double discriminant = std::max(b * b - 4.0 * a * c, 0.0);
  constexpr double kRoundoffGuard = 1.0e-9;
  return (-b + std::sqrt(discriminant)) / (2.0 * a) - kRoundoffGuard;
}

double velocityIncrementWhileRemovingAcceleration(
    double acceleration, double jerk_delta, double dt) {
  if (acceleration <= 0.0) {
    return 0.0;
  }
  const double steps = std::ceil(acceleration / jerk_delta);
  return dt * (steps * acceleration -
               0.5 * jerk_delta * steps * (steps - 1.0));
}

double maximumJerkViableAcceleration(double velocity_headroom,
                                     double acceleration_limit,
                                     double jerk_delta, double dt) {
  if (velocity_headroom <= 0.0) {
    return 0.0;
  }
  if (velocityIncrementWhileRemovingAcceleration(
          acceleration_limit, jerk_delta, dt) <= velocity_headroom) {
    return acceleration_limit;
  }
  double lower = 0.0;
  double upper = acceleration_limit;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double candidate = 0.5 * (lower + upper);
    if (velocityIncrementWhileRemovingAcceleration(candidate, jerk_delta,
                                                    dt) <=
        velocity_headroom) {
      lower = candidate;
    } else {
      upper = candidate;
    }
  }
  return lower;
}

double travelTowardLimitWhileRemovingAcceleration(
    double velocity_toward_limit, double acceleration_toward_limit,
    double braking_acceleration, double jerk_delta, double dt) {
  double velocity = velocity_toward_limit;
  double acceleration = acceleration_toward_limit;
  double position = 0.0;
  double maximum_position = 0.0;
  for (int iteration = 0; iteration < 10000; ++iteration) {
    position += velocity * dt + 0.5 * acceleration * dt * dt;
    velocity += acceleration * dt;
    maximum_position = std::max(maximum_position, position);
    acceleration =
        std::max(acceleration - jerk_delta, -braking_acceleration);
    if (velocity <= 0.0 && acceleration <= 0.0) {
      break;
    }
  }
  return std::max(maximum_position, 0.0);
}

double maximumJerkViablePositionAcceleration(
    double distance, double velocity_toward_limit,
    double acceleration_limit, double braking_acceleration,
    double jerk_delta, double dt) {
  const auto travel = [&](double acceleration) {
    return travelTowardLimitWhileRemovingAcceleration(
        velocity_toward_limit, acceleration, braking_acceleration,
        jerk_delta, dt);
  };
  double lower = -braking_acceleration;
  double upper = acceleration_limit;
  if (travel(lower) > distance) {
    return lower;
  }
  if (travel(upper) <= distance) {
    return upper;
  }
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double candidate = 0.5 * (lower + upper);
    if (travel(candidate) <= distance) {
      lower = candidate;
    } else {
      upper = candidate;
    }
  }
  return lower;
}

JointAccelerationBounds computeJointAccelerationBoundsInternal(
    const Vec7& q_model, const Vec7& qdot_model,
    const Vec7& qddot_previous, const ArmLimits& limits,
    const JointAccelerationLimitConfig& config, double dt,
    bool include_history_jerk) {
  if (!q_model.allFinite() || !qdot_model.allFinite() ||
      !qddot_previous.allFinite() ||
      !limits.lower_position.allFinite() ||
      !limits.upper_position.allFinite() || !limits.velocity.allFinite() ||
      !std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("acceleration bounds input is invalid");
  }

  JointAccelerationBounds bounds;
  const double dt_squared = dt * dt;
  for (int joint = 0; joint < kArmDof; ++joint) {
    const double safe_lower =
        limits.lower_position[joint] + config.margin_rad;
    const double safe_upper =
        limits.upper_position[joint] - config.margin_rad;
    const double velocity_limit =
        config.velocity_scale * limits.velocity[joint];

    const double acceleration_limit = config.max_acceleration_rad_s2[joint];
    const double braking_acceleration =
        config.braking_acceleration_rad_s2[joint];
    const double jerk_limit = config.max_jerk_rad_s3[joint];
    double lower = -acceleration_limit;
    double upper = acceleration_limit;
    lower = std::max(lower,
                     (-velocity_limit - qdot_model[joint]) / dt);
    upper = std::min(upper,
                     (velocity_limit - qdot_model[joint]) / dt);
    if (config.hard_jerk_enabled) {
      const double jerk_delta = jerk_limit * dt;
      const double upper_headroom = velocity_limit - qdot_model[joint];
      const double lower_headroom = velocity_limit + qdot_model[joint];
      upper = std::min(
          upper, maximumJerkViableAcceleration(
                     upper_headroom, acceleration_limit,
                     jerk_delta, dt));
      lower = std::max(
          lower, -maximumJerkViableAcceleration(
                     lower_headroom, acceleration_limit,
                     jerk_delta, dt));
    }
    lower = std::max(
        lower, 2.0 * (safe_lower - q_model[joint] - qdot_model[joint] * dt) /
                   dt_squared);
    upper = std::min(
        upper, 2.0 * (safe_upper - q_model[joint] - qdot_model[joint] * dt) /
                   dt_squared);

    const double lower_distance =
        std::max(q_model[joint] - safe_lower, 0.0);
    const double upper_distance =
        std::max(safe_upper - q_model[joint], 0.0);
    if (config.hard_jerk_enabled) {
      const double jerk_delta = jerk_limit * dt;
      upper = std::min(
          upper, maximumJerkViablePositionAcceleration(
                     upper_distance, qdot_model[joint],
                     acceleration_limit, braking_acceleration, jerk_delta,
                     dt));
      lower = std::max(
          lower, -maximumJerkViablePositionAcceleration(
                     lower_distance, -qdot_model[joint],
                     acceleration_limit, braking_acceleration, jerk_delta,
                     dt));
    }
    const double braking_lower_velocity = -std::sqrt(
        2.0 * braking_acceleration * lower_distance);
    const double braking_upper_velocity = std::sqrt(
        2.0 * braking_acceleration * upper_distance);
    lower = std::max(
        lower, (braking_lower_velocity - qdot_model[joint]) / dt);
    upper = std::min(
        upper, (braking_upper_velocity - qdot_model[joint]) / dt);
    if (!config.hard_jerk_enabled) {
      const double predictive_upper =
          maximumAccelerationPreservingBrakingEnvelope(
              upper_distance, qdot_model[joint],
              braking_acceleration, dt);
      const double predictive_lower =
          -maximumAccelerationPreservingBrakingEnvelope(
              lower_distance, -qdot_model[joint],
              braking_acceleration, dt);
      lower = std::max(lower, predictive_lower);
      upper = std::min(upper, predictive_upper);
    }

    if (config.hard_jerk_enabled && include_history_jerk) {
      lower = std::max(
          lower, qddot_previous[joint] - jerk_limit * dt);
      upper = std::min(
          upper, qddot_previous[joint] + jerk_limit * dt);
    }
    // Independent velocity, braking and jerk viability calculations can
    // converge to the same physical boundary with a few 1e-8 rad/s^2 of
    // floating-point disagreement. Collapse only that numerical seam; a
    // genuinely infeasible hard intersection remains lower > upper.
    constexpr double kIntersectionRoundoff = 1.0e-7;
    if (lower > upper && lower - upper <= kIntersectionRoundoff) {
      const double shared = 0.5 * (lower + upper);
      lower = shared;
      upper = shared;
    }
    bounds.lower[joint] = lower;
    bounds.upper[joint] = upper;
  }
  return bounds;
}

}  // namespace

JointAccelerationBounds computeJointAccelerationBounds(
    const Vec7& q_model, const Vec7& qdot_model,
    const Vec7& qddot_previous, const ArmLimits& limits,
    const JointAccelerationLimitConfig& config, double dt) {
  return computeJointAccelerationBoundsInternal(
      q_model, qdot_model, qddot_previous, limits, config, dt, true);
}

Vec7 seedPreviousAccelerationForFeasibility(
    const Vec7& q_model, const Vec7& qdot_model, const ArmLimits& limits,
    const JointAccelerationLimitConfig& config, double dt) {
  Vec7 seed = Vec7::Zero();
  if (!config.hard_jerk_enabled) {
    return seed;
  }

  const JointAccelerationBounds without_history =
      computeJointAccelerationBoundsInternal(
          q_model, qdot_model, Vec7::Zero(), limits, config, dt, false);
  for (int joint = 0; joint < kArmDof; ++joint) {
    const double jerk_delta = config.max_jerk_rad_s3[joint] * dt;
    double lower = without_history.lower[joint] - jerk_delta;
    double upper = without_history.upper[joint] + jerk_delta;
    lower = std::max(lower, -config.max_acceleration_rad_s2[joint]);
    upper = std::min(upper, config.max_acceleration_rad_s2[joint]);
    if (lower <= upper) {
      seed[joint] = std::clamp(0.0, lower, upper);
    }
  }
  return seed;
}

}  // namespace tianji_qp_ik
