#include "tianji_qp_ik/velocity_ik.hpp"

#include "tianji_qp_ik/hierarchical_qp_ik.hpp"
#include "tianji_qp_ik/nullspace_dls.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace tianji_qp_ik {
namespace {

double stoppingDistanceWithBoundedAcceleration(
    double velocity_toward_limit, double velocity_delta, double dt) {
  if (velocity_toward_limit <= 0.0) {
    return 0.0;
  }
  const double steps = std::ceil(velocity_toward_limit / velocity_delta);
  return dt * (steps * velocity_toward_limit -
               0.5 * velocity_delta * steps * (steps - 1.0));
}

double maximumAccelerationViableVelocity(
    double distance, double velocity_limit, double velocity_delta,
    double dt) {
  if (distance <= 0.0 || velocity_limit <= 0.0) {
    return 0.0;
  }
  if (stoppingDistanceWithBoundedAcceleration(
          velocity_limit, velocity_delta, dt) <= distance) {
    return velocity_limit;
  }

  double lower = 0.0;
  double upper = velocity_limit;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double candidate = 0.5 * (lower + upper);
    if (stoppingDistanceWithBoundedAcceleration(
            candidate, velocity_delta, dt) <= distance) {
      lower = candidate;
    } else {
      upper = candidate;
    }
  }
  return lower;
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
    if (velocityIncrementWhileRemovingAcceleration(
            candidate, jerk_delta, dt) <= velocity_headroom) {
      lower = candidate;
    } else {
      upper = candidate;
    }
  }
  return lower;
}

double semiImplicitTravelWhileBraking(
    double velocity_toward_limit, double acceleration_toward_limit,
    double braking_acceleration, double jerk_delta, double dt) {
  double velocity = velocity_toward_limit;
  double acceleration = acceleration_toward_limit;
  double position = 0.0;
  double maximum_position = 0.0;
  for (int iteration = 0; iteration < 10000; ++iteration) {
    velocity += acceleration * dt;
    position += velocity * dt;
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
    return semiImplicitTravelWhileBraking(
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

}  // namespace

JointVelocityBounds computeJointVelocityBounds(
    const Vec7& q_measured, const Vec7& qdot_previous,
    const Vec7& qddot_previous, const ArmLimits& limits,
    const JointLimitConfig& config, double dt) {
  JointVelocityBounds bounds;

  for (int index = 0; index < kArmDof; ++index) {
    const double velocity_lower = -config.velocity_scale * limits.velocity[index];
    const double velocity_upper = config.velocity_scale * limits.velocity[index];
    const double position_lower =
        (limits.lower_position[index] + config.margin_rad - q_measured[index]) / dt;
    const double position_upper =
        (limits.upper_position[index] - config.margin_rad - q_measured[index]) / dt;
    const double distance_lower = std::max(
        q_measured[index] -
            (limits.lower_position[index] + config.margin_rad),
        0.0);
    const double distance_upper = std::max(
        limits.upper_position[index] - config.margin_rad -
            q_measured[index],
        0.0);
    const double acceleration_delta =
        config.max_acceleration_rad_s2[index] * dt;

    if (config.hard_jerk_enabled) {
      const double acceleration_limit =
          config.max_acceleration_rad_s2[index];
      const double braking_acceleration =
          config.braking_acceleration_rad_s2[index];
      const double jerk_delta = config.max_jerk_rad_s3[index] * dt;
      double acceleration_lower = -acceleration_limit;
      double acceleration_upper = acceleration_limit;
      BoundSource acceleration_lower_source = BoundSource::kAcceleration;
      BoundSource acceleration_upper_source = BoundSource::kAcceleration;
      const auto tighten_lower = [&](double candidate, BoundSource source) {
        if (candidate > acceleration_lower) {
          acceleration_lower = candidate;
          acceleration_lower_source = source;
        }
      };
      const auto tighten_upper = [&](double candidate, BoundSource source) {
        if (candidate < acceleration_upper) {
          acceleration_upper = candidate;
          acceleration_upper_source = source;
        }
      };

      tighten_lower((velocity_lower - qdot_previous[index]) / dt,
                    BoundSource::kVelocity);
      tighten_upper((velocity_upper - qdot_previous[index]) / dt,
                    BoundSource::kVelocity);

      const double dt_squared = dt * dt;
      const double safe_lower =
          limits.lower_position[index] + config.margin_rad;
      const double safe_upper =
          limits.upper_position[index] - config.margin_rad;
      tighten_lower((safe_lower - q_measured[index] -
                     qdot_previous[index] * dt) /
                        dt_squared,
                    BoundSource::kPosition);
      tighten_upper((safe_upper - q_measured[index] -
                     qdot_previous[index] * dt) /
                        dt_squared,
                    BoundSource::kPosition);

      tighten_upper(maximumJerkViableAcceleration(
                        velocity_upper - qdot_previous[index],
                        acceleration_limit, jerk_delta, dt),
                    BoundSource::kJerk);
      tighten_lower(-maximumJerkViableAcceleration(
                        qdot_previous[index] - velocity_lower,
                        acceleration_limit, jerk_delta, dt),
                    BoundSource::kJerk);
      tighten_upper(maximumJerkViablePositionAcceleration(
                        distance_upper, qdot_previous[index],
                        acceleration_limit, braking_acceleration,
                        jerk_delta, dt),
                    BoundSource::kBraking);
      tighten_lower(-maximumJerkViablePositionAcceleration(
                        distance_lower, -qdot_previous[index],
                        acceleration_limit, braking_acceleration,
                        jerk_delta, dt),
                    BoundSource::kBraking);

      tighten_lower(qddot_previous[index] - jerk_delta,
                    BoundSource::kJerk);
      tighten_upper(qddot_previous[index] + jerk_delta,
                    BoundSource::kJerk);

      constexpr double kIntersectionRoundoff = 1.0e-7;
      if (acceleration_lower > acceleration_upper &&
          acceleration_lower - acceleration_upper <=
              kIntersectionRoundoff) {
        const double shared =
            0.5 * (acceleration_lower + acceleration_upper);
        acceleration_lower = shared;
        acceleration_upper = shared;
      }
      bounds.lower[index] =
          qdot_previous[index] + acceleration_lower * dt;
      bounds.upper[index] =
          qdot_previous[index] + acceleration_upper * dt;
      bounds.lower_source[static_cast<std::size_t>(index)] =
          acceleration_lower_source;
      bounds.upper_source[static_cast<std::size_t>(index)] =
          acceleration_upper_source;
      continue;
    }

    const double braking_lower = -std::min(
        std::sqrt(2.0 * config.braking_acceleration_rad_s2[index] *
                  distance_lower),
        maximumAccelerationViableVelocity(
            distance_lower, -velocity_lower, acceleration_delta, dt));
    const double braking_upper = std::min(
        std::sqrt(2.0 * config.braking_acceleration_rad_s2[index] *
                  distance_upper),
        maximumAccelerationViableVelocity(
            distance_upper, velocity_upper, acceleration_delta, dt));

    bounds.lower[index] = velocity_lower;
    bounds.lower_source[static_cast<std::size_t>(index)] = BoundSource::kVelocity;
    if (position_lower > bounds.lower[index]) {
      bounds.lower[index] = position_lower;
      bounds.lower_source[static_cast<std::size_t>(index)] = BoundSource::kPosition;
    }
    if (braking_lower > bounds.lower[index]) {
      bounds.lower[index] = braking_lower;
      bounds.lower_source[static_cast<std::size_t>(index)] = BoundSource::kBraking;
    }

    bounds.upper[index] = velocity_upper;
    bounds.upper_source[static_cast<std::size_t>(index)] = BoundSource::kVelocity;
    if (position_upper < bounds.upper[index]) {
      bounds.upper[index] = position_upper;
      bounds.upper_source[static_cast<std::size_t>(index)] = BoundSource::kPosition;
    }
    if (braking_upper < bounds.upper[index]) {
      bounds.upper[index] = braking_upper;
      bounds.upper_source[static_cast<std::size_t>(index)] = BoundSource::kBraking;
    }

    const double acceleration_lower =
        qdot_previous[index] - acceleration_delta;
    const double acceleration_upper =
        qdot_previous[index] + acceleration_delta;
    if (acceleration_lower <= bounds.upper[index] &&
        acceleration_lower > bounds.lower[index]) {
      bounds.lower[index] = acceleration_lower;
      bounds.lower_source[static_cast<std::size_t>(index)] =
          BoundSource::kAcceleration;
    }
    if (acceleration_upper >= bounds.lower[index] &&
        acceleration_upper < bounds.upper[index]) {
      bounds.upper[index] = acceleration_upper;
      bounds.upper_source[static_cast<std::size_t>(index)] =
          BoundSource::kAcceleration;
    }

  }

  return bounds;
}

JointVelocityNullspaceResult refinePostureVelocityInNullspace(
    const Vec7& primary, const Mat67& cartesian_jacobian,
    const JointVelocityPostureTask& task, const Vec7& lower,
    const Vec7& upper, double tolerance,
    const LinearJointConstraint& linear_constraint,
    const LinearJointConstraint& secondary_linear_constraint) noexcept {
  JointVelocityNullspaceResult result;
  result.value = primary;
  if (!task.active || !primary.allFinite() ||
      !cartesian_jacobian.allFinite() || !task.target.allFinite() ||
      !lower.allFinite() || !upper.allFinite() ||
      !std::isfinite(task.activation) || task.activation <= 0.0 ||
      !std::isfinite(tolerance) || tolerance <= 0.0 ||
      (lower.array() > upper.array()).any() ||
      (primary.array() < lower.array() - tolerance).any() ||
      (primary.array() > upper.array() + tolerance).any() ||
      (linear_constraint.active &&
       (!linear_constraint.jacobian.allFinite() ||
        std::isnan(linear_constraint.lower) ||
        std::isnan(linear_constraint.upper) ||
        linear_constraint.lower > linear_constraint.upper)) ||
      (secondary_linear_constraint.active &&
       (!secondary_linear_constraint.jacobian.allFinite() ||
        std::isnan(secondary_linear_constraint.lower) ||
        std::isnan(secondary_linear_constraint.upper) ||
        secondary_linear_constraint.lower >
            secondary_linear_constraint.upper))) {
    return result;
  }

  Eigen::JacobiSVD<Mat67> svd(cartesian_jacobian, Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success || svd.matrixV().cols() != kArmDof) {
    return result;
  }
  int rank = 0;
  for (const double singular_value : svd.singularValues()) {
    if (singular_value > tolerance) {
      ++rank;
    }
  }
  Mat77 projector = Mat77::Zero();
  for (int column = rank; column < kArmDof; ++column) {
    const Vec7 basis = svd.matrixV().col(column);
    projector.noalias() += basis * basis.transpose();
  }
  const Vec7 correction = projector * (task.target - primary);
  if (!correction.allFinite() || correction.norm() <= tolerance) {
    return result;
  }
  const double full_cartesian_residual =
      (cartesian_jacobian * correction).norm();
  if (!std::isfinite(full_cartesian_residual) ||
      full_cartesian_residual > tolerance) {
    return result;
  }

  double scale_lower = 0.0;
  double scale_upper = std::clamp(task.activation, 0.0, 1.0);
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (std::abs(correction[joint]) <= tolerance) {
      continue;
    }
    double one = (lower[joint] - primary[joint]) / correction[joint];
    double two = (upper[joint] - primary[joint]) / correction[joint];
    if (one > two) {
      std::swap(one, two);
    }
    scale_lower = std::max(scale_lower, one);
    scale_upper = std::min(scale_upper, two);
  }
  const auto intersect_constraint = [&](
      const LinearJointConstraint& constraint) {
    if (!constraint.active) {
      return true;
    }
    const double before = constraint.jacobian.dot(primary);
    const double sensitivity = constraint.jacobian.dot(correction);
    if (!std::isfinite(before) || !std::isfinite(sensitivity)) {
      return false;
    }
    if (std::abs(sensitivity) <= tolerance) {
      return before >= constraint.lower - tolerance &&
             before <= constraint.upper + tolerance;
    } else {
      double one = (constraint.lower - before) / sensitivity;
      double two = (constraint.upper - before) / sensitivity;
      if (one > two) {
        std::swap(one, two);
      }
      scale_lower = std::max(scale_lower, one);
      scale_upper = std::min(scale_upper, two);
    }
    return true;
  };
  if (!intersect_constraint(linear_constraint) ||
      !intersect_constraint(secondary_linear_constraint)) {
    return result;
  }
  if (scale_lower > scale_upper + tolerance || scale_upper <= tolerance) {
    return result;
  }

  const double scale = scale_upper;
  const Vec7 candidate = primary + scale * correction;
  const double cartesian_residual =
      (cartesian_jacobian * (candidate - primary)).norm();
  if (!candidate.allFinite() || !std::isfinite(cartesian_residual) ||
      cartesian_residual > tolerance ||
      (candidate.array() < lower.array() - tolerance).any() ||
      (candidate.array() > upper.array() + tolerance).any() ||
      (linear_constraint.active &&
       (linear_constraint.jacobian.dot(candidate) <
            linear_constraint.lower - tolerance ||
        linear_constraint.jacobian.dot(candidate) >
            linear_constraint.upper + tolerance)) ||
      (secondary_linear_constraint.active &&
       (secondary_linear_constraint.jacobian.dot(candidate) <
            secondary_linear_constraint.lower - tolerance ||
        secondary_linear_constraint.jacobian.dot(candidate) >
            secondary_linear_constraint.upper + tolerance))) {
    return result;
  }

  result.value = candidate;
  result.active = true;
  result.scale = scale;
  result.correction_norm = (candidate - primary).norm();
  result.cartesian_residual = cartesian_residual;
  return result;
}

std::unique_ptr<IArmVelocityIk> makeArmVelocityIk(
    IkAlgorithm algorithm, const QpIkConfig& config) {
  if (algorithm == IkAlgorithm::kNullspaceDls) {
    return std::make_unique<NullspaceDlsIk7>(config.dls);
  }
  return std::make_unique<HierarchicalQpIk7>(
      config.hierarchical_qp, config.qpoases, config.safety);
}

}  // namespace tianji_qp_ik
