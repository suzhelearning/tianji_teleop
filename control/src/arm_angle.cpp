#include "tianji_qp_ik/arm_angle.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

constexpr double kDirectionEpsilon = 1e-9;
constexpr double kReferenceProjectionEpsilon = 1e-3;

Eigen::Vector3d fixedReferenceRadial(const Eigen::Vector3d& axis,
                                     ArmSide side) noexcept {
  Eigen::Vector3d reference = -Eigen::Vector3d::UnitZ();
  Eigen::Vector3d radial = reference - axis * axis.dot(reference);
  if (radial.norm() < kReferenceProjectionEpsilon) {
    reference = Eigen::Vector3d::UnitY();
    if (side == ArmSide::kRight) {
      reference = -reference;
    }
    radial = reference - axis * axis.dot(reference);
  }
  if (!radial.allFinite() || radial.norm() < kReferenceProjectionEpsilon) {
    radial = axis.unitOrthogonal();
  }
  return radial.normalized();
}

bool standardArmPlaneAngle(const Eigen::Vector3d& axis_input,
                           const Eigen::Vector3d& radial_input,
                           ArmSide side, double& angle) noexcept {
  if (!axis_input.allFinite() || !radial_input.allFinite() ||
      axis_input.norm() <= kDirectionEpsilon ||
      radial_input.norm() <= kDirectionEpsilon) {
    return false;
  }
  const Eigen::Vector3d axis = axis_input.normalized();
  Eigen::Vector3d radial = radial_input - axis * axis.dot(radial_input);
  if (!radial.allFinite() || radial.norm() < kReferenceProjectionEpsilon) {
    return false;
  }
  radial.normalize();
  const Eigen::Vector3d reference = fixedReferenceRadial(axis, side);
  angle = std::atan2(axis.dot(reference.cross(radial)),
                     reference.dot(radial));
  return std::isfinite(angle);
}

ArmDirectionReference downReference() noexcept {
  return {true, -Eigen::Vector3d::UnitZ(),
          ArmDirectionReferenceSource::kDefaultDown};
}

Eigen::Vector3d rotateToward(const Eigen::Vector3d& from,
                             const Eigen::Vector3d& to,
                             double maximum_angle) noexcept {
  const double dot = std::clamp(from.dot(to), -1.0, 1.0);
  const double angle = std::acos(dot);
  if (angle <= maximum_angle || angle < kDirectionEpsilon) {
    return to;
  }

  Eigen::Vector3d axis = from.cross(to);
  if (axis.norm() < kDirectionEpsilon) {
    axis = from.unitOrthogonal();
  } else {
    axis.normalize();
  }
  return (Eigen::AngleAxisd(maximum_angle, axis) * from).normalized();
}

bool validConstraintBounds(const Vec7& lower, const Vec7& upper) noexcept {
  return lower.allFinite() && upper.allFinite() &&
         (lower.array() <= upper.array()).all();
}

double maximumConstraintValue(const Vec7& jacobian, const Vec7& lower,
                              const Vec7& upper) noexcept {
  double maximum = 0.0;
  for (int index = 0; index < kArmDof; ++index) {
    maximum += jacobian[index] *
               (jacobian[index] >= 0.0 ? upper[index] : lower[index]);
  }
  return maximum;
}

double minimumConstraintValue(const Vec7& jacobian, const Vec7& lower,
                              const Vec7& upper) noexcept {
  double minimum = 0.0;
  for (int index = 0; index < kArmDof; ++index) {
    minimum += jacobian[index] *
               (jacobian[index] >= 0.0 ? lower[index] : upper[index]);
  }
  return minimum;
}

LinearJointConstraint boundedBranchLockLowerConstraint(
    const ArmAngleTask& task, double requested_lower, const Vec7& lower,
    const Vec7& upper) noexcept {
  LinearJointConstraint result;
  if (!task.branch_lock_active || !task.branch_lock_jacobian.allFinite() ||
      task.branch_lock_jacobian.norm() <= kDirectionEpsilon ||
      !std::isfinite(requested_lower) || !validConstraintBounds(lower, upper)) {
    return result;
  }
  const double maximum = maximumConstraintValue(task.branch_lock_jacobian,
                                                lower, upper);
  const double minimum = minimumConstraintValue(task.branch_lock_jacobian,
                                                lower, upper);
  if (!std::isfinite(maximum) || !std::isfinite(minimum) ||
      minimum > maximum) {
    return result;
  }
  result.jacobian = task.branch_lock_jacobian;
  result.requested_lower = requested_lower;
  const double feasibility_margin =
      1.0e-7 * std::max(1.0, std::abs(maximum));
  if (requested_lower > maximum - feasibility_margin) {
    // Do not pin all seven joints to the unique maximizing box vertex. That
    // numerically fragile corner can conflict with the simultaneous outward
    // row and exhaust qpOASES' working-set budget. Keep the branch row active
    // with the same small interior reserve used by the outward barrier.
    constexpr double kRecoveryBoxReserveFraction = 0.05;
    const double recovery_reserve = std::max(
        feasibility_margin,
        kRecoveryBoxReserveFraction * std::max(0.0, maximum - minimum));
    result.active = true;
    result.lower = maximum - recovery_reserve;
    result.upper = std::numeric_limits<double>::infinity();
    result.feasibility_clipped = true;
    return result;
  }
  result.active = true;
  result.lower = requested_lower;
  result.upper = std::numeric_limits<double>::infinity();
  return result;
}

}  // namespace

DualArmDirectionReferences defaultArmDirectionReferences() noexcept {
  return {downReference(), downReference()};
}

std::string_view toString(ArmDirectionReferenceSource source) noexcept {
  switch (source) {
    case ArmDirectionReferenceSource::kPico:
      return "pico";
    case ArmDirectionReferenceSource::kDefaultDown:
      return "default_down";
    case ArmDirectionReferenceSource::kPrevious:
      return "previous";
    case ArmDirectionReferenceSource::kDegenerate:
      return "degenerate";
  }
  return "degenerate";
}

std::string_view toString(ArmAngleReferenceMode mode) noexcept {
  switch (mode) {
    case ArmAngleReferenceMode::kPico:
      return "pico";
    case ArmAngleReferenceMode::kDefaultDown:
      return "default_down";
    case ArmAngleReferenceMode::kOutwardOnly:
      return "outward_only";
    case ArmAngleReferenceMode::kPicoOutward:
      return "pico_outward";
  }
  return "default_down";
}

ArmAngleReferenceMode armAngleReferenceModeFromString(std::string_view value) {
  if (value == "pico") {
    return ArmAngleReferenceMode::kPico;
  }
  if (value == "default_down") {
    return ArmAngleReferenceMode::kDefaultDown;
  }
  if (value == "outward_only") {
    return ArmAngleReferenceMode::kOutwardOnly;
  }
  if (value == "pico_outward") {
    return ArmAngleReferenceMode::kPicoOutward;
  }
  throw std::invalid_argument(
      "arm-angle mode must be pico, default_down, outward_only, or "
      "pico_outward");
}

ArmAngleReferenceMode toggleArmAngleReferenceMode(
    ArmAngleReferenceMode mode) noexcept {
  switch (mode) {
    case ArmAngleReferenceMode::kPico:
      return ArmAngleReferenceMode::kDefaultDown;
    case ArmAngleReferenceMode::kDefaultDown:
      return ArmAngleReferenceMode::kOutwardOnly;
    case ArmAngleReferenceMode::kOutwardOnly:
      return ArmAngleReferenceMode::kPicoOutward;
    case ArmAngleReferenceMode::kPicoOutward:
      return ArmAngleReferenceMode::kPico;
  }
  return ArmAngleReferenceMode::kPico;
}

bool usesPicoArmDirection(ArmAngleReferenceMode mode) noexcept {
  return mode == ArmAngleReferenceMode::kPico ||
         mode == ArmAngleReferenceMode::kPicoOutward;
}

bool usesContinuityArmDirection(ArmAngleReferenceMode mode) noexcept {
  return mode == ArmAngleReferenceMode::kOutwardOnly;
}

bool usesOutwardArmBarrier(ArmAngleReferenceMode mode) noexcept {
  return mode == ArmAngleReferenceMode::kOutwardOnly ||
         mode == ArmAngleReferenceMode::kPicoOutward;
}

DualArmDirectionReferences selectArmDirectionReferences(
    ArmAngleReferenceMode mode, bool pico_live,
    const DualArmDirectionReferences& pico_references) noexcept {
  const bool has_last_valid_pico =
      pico_references.left.valid || pico_references.right.valid;
  if (usesPicoArmDirection(mode) &&
      (pico_live || has_last_valid_pico)) {
    return pico_references;
  }
  return defaultArmDirectionReferences();
}

ArmDirectionReferenceManager::ArmDirectionReferenceManager(
    double rate_limit_rad_s)
    : rate_limit_rad_s_(rate_limit_rad_s) {
  if (!std::isfinite(rate_limit_rad_s_) || rate_limit_rad_s_ <= 0.0) {
    throw std::invalid_argument(
        "arm direction rate limit must be finite and positive");
  }
}

DualArmDirectionReferences ArmDirectionReferenceManager::update(
    const DualArmDirectionReferences& requested, double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("arm direction update dt must be positive");
  }
  DualArmDirectionReferences output;
  output.left = updateOne(requested.left, previous_.left, initialized_, dt);
  output.right = updateOne(requested.right, previous_.right, initialized_, dt);
  previous_ = output;
  initialized_ = true;
  return output;
}

void ArmDirectionReferenceManager::reset() noexcept {
  initialized_ = true;
  previous_ = defaultArmDirectionReferences();
}

ArmDirectionReference ArmDirectionReferenceManager::updateOne(
    const ArmDirectionReference& requested,
    const ArmDirectionReference& previous, bool initialized,
    double dt) const {
  const bool requested_valid =
      requested.valid && requested.direction.allFinite() &&
      requested.direction.norm() > kDirectionEpsilon;
  if (!requested_valid) {
    if (initialized && previous.valid && previous.direction.allFinite() &&
        previous.direction.norm() > kDirectionEpsilon) {
      return {true, previous.direction.normalized(),
              ArmDirectionReferenceSource::kPrevious};
    }
    return downReference();
  }
  ArmDirectionReference target = requested;
  target.direction.normalize();
  if (!initialized || !previous.valid) {
    return target;
  }
  target.direction = rotateToward(previous.direction.normalized(),
                                  target.direction,
                                  rate_limit_rad_s_ * dt);
  return target;
}

ArmAngleTaskBuilder::ArmAngleTaskBuilder(ArmSide side,
                                         double minimum_radius_m,
                                         double full_weight_radius_m,
                                         double reference_rate_limit_rad_s,
                                         double reference_projection_hold_enter,
                                         double reference_projection_hold_exit,
                                         double error_branch_hysteresis_rad,
                                         double branch_lock_radius_m,
                                         bool reference_governor_enabled,
                                         double reference_governor_enter_error_rad,
                                         double reference_governor_exit_error_rad,
                                         double reference_governor_tracking_error_rad)
    : side_(side),
      minimum_radius_m_(minimum_radius_m),
      full_weight_radius_m_(full_weight_radius_m),
      branch_lock_radius_m_(branch_lock_radius_m > 0.0
                                ? branch_lock_radius_m
                                : minimum_radius_m),
      reference_rate_limit_rad_s_(reference_rate_limit_rad_s),
      reference_projection_hold_enter_(reference_projection_hold_enter),
      reference_projection_hold_exit_(reference_projection_hold_exit),
      error_branch_hysteresis_rad_(error_branch_hysteresis_rad),
      reference_governor_enabled_(reference_governor_enabled),
      reference_governor_enter_error_rad_(
          reference_governor_enter_error_rad),
      reference_governor_exit_error_rad_(
          reference_governor_exit_error_rad),
      reference_governor_tracking_error_rad_(
          reference_governor_tracking_error_rad) {
  if (!std::isfinite(minimum_radius_m_) || minimum_radius_m_ <= 0.0 ||
      !std::isfinite(full_weight_radius_m_) ||
      full_weight_radius_m_ <= minimum_radius_m_ ||
      !std::isfinite(branch_lock_radius_m_) ||
      branch_lock_radius_m_ < minimum_radius_m_ ||
      branch_lock_radius_m_ > full_weight_radius_m_ ||
      !std::isfinite(reference_rate_limit_rad_s_) ||
      reference_rate_limit_rad_s_ <= 0.0 ||
      !std::isfinite(reference_projection_hold_enter_) ||
      !std::isfinite(reference_projection_hold_exit_) ||
      reference_projection_hold_enter_ <= 0.0 ||
      reference_projection_hold_enter_ >= reference_projection_hold_exit_ ||
      reference_projection_hold_exit_ > 1.0 ||
      !std::isfinite(error_branch_hysteresis_rad_) ||
      error_branch_hysteresis_rad_ <= 0.0 ||
      error_branch_hysteresis_rad_ >= 3.14159265358979323846 ||
      !std::isfinite(reference_governor_enter_error_rad_) ||
      !std::isfinite(reference_governor_exit_error_rad_) ||
      !std::isfinite(reference_governor_tracking_error_rad_) ||
      reference_governor_tracking_error_rad_ <= 0.0 ||
      reference_governor_tracking_error_rad_ >
          reference_governor_exit_error_rad_ ||
      reference_governor_exit_error_rad_ <= 0.0 ||
      reference_governor_exit_error_rad_ >=
          reference_governor_enter_error_rad_ ||
      reference_governor_enter_error_rad_ >=
          3.14159265358979323846) {
    throw std::invalid_argument("invalid arm-angle geometry configuration");
  }
}

ArmAngleTask ArmAngleTaskBuilder::compute(
    const ArmAngleGeometryInput& geometry,
    const ArmDirectionReference& reference, double dt) {
  ArmAngleTask result;
  result.shoulder_world_z = geometry.shoulder_position.z();
  result.elbow_world_z = geometry.elbow_position.z();
  if (!geometry.shoulder_position.allFinite() ||
      !geometry.elbow_position.allFinite() ||
      !geometry.wrist_position.allFinite() ||
      !geometry.shoulder_position_jacobian.allFinite() ||
      !geometry.elbow_position_jacobian.allFinite() ||
      !geometry.wrist_position_jacobian.allFinite() ||
      !std::isfinite(dt) || dt <= 0.0) {
    return result;
  }

  const Eigen::Vector3d shoulder_to_wrist =
      geometry.wrist_position - geometry.shoulder_position;
  const double shoulder_to_wrist_norm = shoulder_to_wrist.norm();
  if (shoulder_to_wrist_norm <= kDirectionEpsilon) {
    return result;
  }
  const Eigen::Vector3d axis = shoulder_to_wrist.normalized();
  const Eigen::Matrix3d axis_projector =
      Eigen::Matrix3d::Identity() - axis * axis.transpose();
  const Eigen::Vector3d shoulder_to_elbow =
      geometry.elbow_position - geometry.shoulder_position;
  const Eigen::Vector3d radial =
      shoulder_to_elbow - axis * axis.dot(shoulder_to_elbow);
  result.radius_m = radial.norm();

  // Preserve the last reliable physical elbow branch.  Updating only while
  // the plane radius is comfortably observable prevents the reference from
  // being overwritten by noise exactly where the signed angle is undefined.
  Eigen::Vector3d current = Eigen::Vector3d::Zero();
  if (result.radius_m > kDirectionEpsilon) {
    current = radial / result.radius_m;
    if (!current_direction_initialized_ ||
        result.radius_m >= full_weight_radius_m_) {
      current_direction_ = current;
      current_direction_initialized_ = true;
    }
  }

  const auto radialVelocityForColumn = [&](int column) -> Eigen::Vector3d {
    const Eigen::Vector3d shoulder_velocity =
        geometry.shoulder_position_jacobian.col(column);
    const Eigen::Vector3d elbow_velocity =
        geometry.elbow_position_jacobian.col(column);
    const Eigen::Vector3d wrist_velocity =
        geometry.wrist_position_jacobian.col(column);
    const Eigen::Vector3d axis_velocity =
        axis_projector * (wrist_velocity - shoulder_velocity) /
        shoulder_to_wrist_norm;
    const Eigen::Vector3d elbow_relative_velocity =
        elbow_velocity - shoulder_velocity;
    const double axis_elbow_projection_rate =
        axis_velocity.dot(shoulder_to_elbow) +
        axis.dot(elbow_relative_velocity);
    return elbow_relative_velocity -
           axis_velocity * axis.dot(shoulder_to_elbow) -
           axis * axis_elbow_projection_rate;
  };

  if (current_direction_initialized_ &&
      result.radius_m < full_weight_radius_m_) {
    // Keep the branch anchor fixed in world coordinates.  Projecting this
    // anchor onto the changing shoulder-wrist plane would make the
    // constraint's differential omit the anchor's own time derivative and
    // could let the signed distance cross zero during a fast pose change.
    Eigen::Vector3d branch = current_direction_;
    const double branch_norm = branch.norm();
    if (std::isfinite(branch_norm) &&
        branch_norm > kReferenceProjectionEpsilon) {
      branch /= branch_norm;
      result.branch_lock_distance_m =
          branch.dot(radial) - branch_lock_radius_m_;
      for (int column = 0; column < kArmDof; ++column) {
        result.branch_lock_jacobian[column] =
            branch.dot(radialVelocityForColumn(column));
      }
      result.branch_lock_active =
          result.branch_lock_jacobian.allFinite() &&
          result.branch_lock_jacobian.norm() > kDirectionEpsilon;
      if (!result.branch_lock_active) {
        result.branch_lock_jacobian.setZero();
      }
    }
  }

  if (result.radius_m <= kDirectionEpsilon) {
    return result;
  }

  double standard_target_angle = 0.0;
  const bool standard_target_valid =
      reference.valid && reference.shoulder_to_wrist_axis_valid &&
      standardArmPlaneAngle(reference.shoulder_to_wrist_axis,
                            reference.direction, side_,
                            standard_target_angle);

  Eigen::Vector3d requested = standard_target_valid
                                  ? -Eigen::Vector3d::UnitZ()
                                  : reference.direction;
  if (!reference.valid || !requested.allFinite() ||
      requested.norm() <= kDirectionEpsilon) {
    requested = -Eigen::Vector3d::UnitZ();
    result.reference_source = ArmDirectionReferenceSource::kDefaultDown;
  } else {
    requested.normalize();
    result.reference_source = reference.source;
  }
  Eigen::Vector3d differentiation_reference = requested;
  Eigen::Vector3d projected = requested - axis * axis.dot(requested);
  const double requested_projection_norm = projected.norm();
  result.requested_reference_projection_norm =
      std::isfinite(requested_projection_norm) ? requested_projection_norm
                                               : 0.0;
  const bool requested_projection_finite =
      std::isfinite(requested_projection_norm) &&
      requested_projection_norm >= kReferenceProjectionEpsilon;
  Eigen::Vector3d requested_in_plane = Eigen::Vector3d::Zero();
  if (requested_projection_finite) {
    requested_in_plane = projected / requested_projection_norm;
  }

  Eigen::Vector3d previous_in_plane = previous_projection_ -
      axis * axis.dot(previous_projection_);
  const bool previous_valid =
      has_previous_projection_ && previous_in_plane.allFinite() &&
      previous_in_plane.norm() >= kReferenceProjectionEpsilon;
  if (previous_valid) {
    previous_in_plane.normalize();
  }

  if (reference_projection_held_) {
    if (requested_projection_finite &&
        requested_projection_norm >= reference_projection_hold_exit_) {
      reference_projection_held_ = false;
    }
  } else if (!requested_projection_finite ||
             requested_projection_norm <
                 reference_projection_hold_enter_) {
    reference_projection_held_ = true;
  }
  result.reference_projection_held = reference_projection_held_;

  if (reference_projection_held_ && previous_valid) {
    projected = previous_in_plane;
    differentiation_reference = projected;
    result.reference_source = ArmDirectionReferenceSource::kPrevious;
  } else if (!reference_projection_held_ && requested_projection_finite) {
    projected /= requested_projection_norm;
    if (previous_valid) {
      const double signed_change = std::atan2(
          axis.dot(previous_in_plane.cross(projected)),
          previous_in_plane.dot(projected));
      const double maximum_change = reference_rate_limit_rad_s_ * dt;
      const double limited_change =
          std::clamp(signed_change, -maximum_change, maximum_change);
      if (std::abs(limited_change - signed_change) > kDirectionEpsilon) {
        projected =
            (Eigen::AngleAxisd(limited_change, axis) * previous_in_plane)
                .normalized();
        differentiation_reference = projected;
      }
    }
  }

  if ((reference_projection_held_ && !previous_valid) ||
      (!reference_projection_held_ && !requested_projection_finite)) {
    const Eigen::Vector3d outward =
        side_ == ArmSide::kLeft
            ? Eigen::Vector3d(Eigen::Vector3d::UnitY())
            : Eigen::Vector3d(-Eigen::Vector3d::UnitY());
    differentiation_reference = outward;
    projected = outward - axis * axis.dot(outward);
    if (projected.norm() < kReferenceProjectionEpsilon) {
      projected = axis.unitOrthogonal();
      differentiation_reference = projected;
    }
    result.reference_source = ArmDirectionReferenceSource::kDegenerate;
  }
  projected.normalize();

  if (reference_governor_enabled_ && previous_valid &&
      result.radius_m >= minimum_radius_m_) {
    const auto signedErrorTo = [&](const Eigen::Vector3d& direction) {
      return std::atan2(axis.dot(current.cross(direction)),
                        current.dot(direction));
    };
    const double candidate_error = signedErrorTo(projected);
    const double raw_error =
        !reference_projection_held_ && requested_projection_finite
            ? signedErrorTo(requested_in_plane)
            : candidate_error;
    if (reference_governor_held_) {
      if (!reference_projection_held_ &&
          std::abs(raw_error) <=
          reference_governor_exit_error_rad_) {
        reference_governor_held_ = false;
      }
    } else if (std::abs(candidate_error) >=
               reference_governor_enter_error_rad_) {
      reference_governor_held_ = true;
      reference_governor_error_sign_ = std::copysign(1.0, candidate_error);
    }

    if (reference_governor_held_) {
      // The Cartesian primary can rotate the physical elbow plane faster
      // than the one redundant DoF can follow the skeletal reference. Do not
      // let that infeasible secondary error wind up to +/-pi. Back-calculate
      // the effective reference to a small tracking bias around the modeled
      // elbow, so the secondary task keeps the intended turn direction
      // without continuously saturating against the Cartesian primary.
      if (std::abs(raw_error) < reference_governor_enter_error_rad_ &&
          std::abs(raw_error) > kDirectionEpsilon) {
        reference_governor_error_sign_ = std::copysign(1.0, raw_error);
      }
      const double governed_error = reference_governor_error_sign_ *
          std::min(std::abs(raw_error),
                   reference_governor_tracking_error_rad_);
      projected =
          (Eigen::AngleAxisd(governed_error, axis) * current).normalized();
      differentiation_reference = projected;
    }
  } else {
    reference_governor_held_ = false;
  }
  result.reference_governor_held = reference_governor_held_;
  if (previous_valid) {
    const double signed_reference_change = std::atan2(
        axis.dot(previous_in_plane.cross(projected)),
        previous_in_plane.dot(projected));
    const double raw_reference_rate = signed_reference_change / dt;
    if (std::isfinite(raw_reference_rate)) {
      result.reference_rate_rad_s = std::clamp(
          raw_reference_rate, -reference_rate_limit_rad_s_,
          reference_rate_limit_rad_s_);
    }
  }
  result.projected_reference = projected;
  previous_projection_ = projected;
  has_previous_projection_ = true;

  const double wrapped_error = std::atan2(
      axis.dot(current.cross(projected)), current.dot(projected));
  result.activation = std::clamp(
      (result.radius_m - minimum_radius_m_) /
          (full_weight_radius_m_ - minimum_radius_m_),
      0.0, 1.0);
  if (result.activation <= 0.0) {
    return result;
  }
  result.robot_angle_rad = -wrapped_error;
  result.target_angle_rad =
      standard_target_valid ? standard_target_angle : 0.0;
  result.error_rad = standard_target_valid
                         ? std::atan2(
                               std::sin(result.target_angle_rad -
                                        result.robot_angle_rad),
                               std::cos(result.target_angle_rad -
                                        result.robot_angle_rad))
                         : wrapped_error;
  result.control_error_rad = result.error_rad;
  if (standard_target_valid) {
    if (target_angle_initialized_) {
      const double target_delta = std::atan2(
          std::sin(standard_target_angle - previous_target_angle_rad_),
          std::cos(standard_target_angle - previous_target_angle_rad_));
      result.reference_rate_rad_s = std::clamp(
          target_delta / dt, -reference_rate_limit_rad_s_,
          reference_rate_limit_rad_s_);
    } else {
      result.reference_rate_rad_s = 0.0;
    }
    previous_target_angle_rad_ = standard_target_angle;
    target_angle_initialized_ = true;
    result.reference_source = reference.source;
  }
  constexpr double kPi = 3.14159265358979323846;
  const bool in_branch_hysteresis =
      std::abs(wrapped_error) >= kPi - error_branch_hysteresis_rad_;
  if (control_error_initialized_ && in_branch_hysteresis &&
      std::abs(previous_control_error_rad_) >=
          kPi - error_branch_hysteresis_rad_) {
    result.control_error_rad =
        std::copysign(std::abs(wrapped_error), previous_control_error_rad_);
  }
  previous_control_error_rad_ = result.control_error_rad;
  control_error_initialized_ = true;

  differentiation_reference.normalize();
  const Eigen::Vector3d reference_projection =
      differentiation_reference -
      axis * axis.dot(differentiation_reference);
  const double reference_projection_norm = reference_projection.norm();
  if (reference_projection_norm <= kDirectionEpsilon) {
    return result;
  }

  const Eigen::Matrix3d radial_projector =
      Eigen::Matrix3d::Identity() - current * current.transpose();
  const Eigen::Matrix3d reference_projector =
      Eigen::Matrix3d::Identity() - projected * projected.transpose();
  const double axis_reference_projection =
      axis.dot(differentiation_reference);
  const double cosine = current.dot(projected);
  const double sine = axis.dot(current.cross(projected));
  const double angle_denominator = cosine * cosine + sine * sine;
  if (!std::isfinite(angle_denominator) ||
      angle_denominator <= kDirectionEpsilon) {
    return result;
  }

  for (int column = 0; column < kArmDof; ++column) {
    const Eigen::Vector3d shoulder_velocity =
        geometry.shoulder_position_jacobian.col(column);
    const Eigen::Vector3d wrist_velocity =
        geometry.wrist_position_jacobian.col(column);
    const Eigen::Vector3d axis_velocity =
        axis_projector * (wrist_velocity - shoulder_velocity) /
        shoulder_to_wrist_norm;
    const Eigen::Vector3d radial_velocity = radialVelocityForColumn(column);
    const Eigen::Vector3d current_velocity =
        radial_projector * radial_velocity / result.radius_m;
    const Eigen::Vector3d projected_reference_velocity =
        -axis_velocity * axis_reference_projection -
        axis * axis_velocity.dot(differentiation_reference);
    const Eigen::Vector3d projected_velocity =
        reference_projector * projected_reference_velocity /
        reference_projection_norm;
    const double cosine_rate = current_velocity.dot(projected) +
                               current.dot(projected_velocity);
    const double sine_rate =
        axis_velocity.dot(current.cross(projected)) +
        axis.dot(current_velocity.cross(projected) +
                 current.cross(projected_velocity));
    const double error_rate =
        (cosine * sine_rate - sine * cosine_rate) / angle_denominator;
    result.jacobian[column] = -error_rate;
  }
  result.active = result.jacobian.allFinite() &&
                  result.jacobian.norm() > kDirectionEpsilon;
  result.jacobian_norm =
      result.jacobian.allFinite() ? result.jacobian.norm() : 0.0;
  if (!result.active) {
    result.jacobian.setZero();
    result.jacobian_norm = 0.0;
  }
  return result;
}

LinearJointConstraint makeArmAngleBranchLockVelocityConstraint(
    const ArmAngleTask& task, const Vec7& lower, const Vec7& upper,
    double gain) noexcept {
  if (!std::isfinite(gain) || gain < 0.0 ||
      !std::isfinite(task.branch_lock_distance_m)) {
    return {};
  }
  return boundedBranchLockLowerConstraint(
      task, -gain * task.branch_lock_distance_m, lower, upper);
}

LinearJointConstraint makeArmAngleBranchLockAccelerationConstraint(
    const ArmAngleTask& task, const Vec7& qdot, double jdot_qdot,
    const Vec7& lower, const Vec7& upper, double kp, double kd) noexcept {
  if (!qdot.allFinite() || !std::isfinite(jdot_qdot) ||
      !std::isfinite(kp) || kp < 0.0 || !std::isfinite(kd) || kd < 0.0 ||
      !std::isfinite(task.branch_lock_distance_m)) {
    return {};
  }
  const double current_rate = task.branch_lock_jacobian.dot(qdot);
  if (!std::isfinite(current_rate)) {
    return {};
  }
  const double requested_lower =
      -jdot_qdot - kd * current_rate - kp * task.branch_lock_distance_m;
  return boundedBranchLockLowerConstraint(task, requested_lower, lower, upper);
}

LinearJointConstraint makeArmAngleTrackingEnvelopeVelocityConstraint(
    const ArmAngleTask& task, const Vec7& lower, const Vec7& upper,
    double maximum_error_rad, double gain) noexcept {
  LinearJointConstraint result;
  if (!task.active || !task.jacobian.allFinite() ||
      task.jacobian.norm() <= kDirectionEpsilon ||
      !std::isfinite(task.control_error_rad) ||
      !std::isfinite(task.reference_rate_rad_s) ||
      !std::isfinite(maximum_error_rad) || maximum_error_rad <= 0.0 ||
      !std::isfinite(gain) || gain <= 0.0 ||
      !validConstraintBounds(lower, upper)) {
    return result;
  }

  // e = reference - model arm angle and e_dot = reference_rate - J*qdot.
  // Applying a first-order barrier to both e <= e_max and e >= -e_max gives
  // this interval for the modeled arm-angle rate.
  const double requested_lower =
      task.reference_rate_rad_s -
      gain * (maximum_error_rad - task.control_error_rad);
  const double requested_upper =
      task.reference_rate_rad_s +
      gain * (maximum_error_rad + task.control_error_rad);
  if (!std::isfinite(requested_lower) || !std::isfinite(requested_upper) ||
      requested_lower > requested_upper) {
    return result;
  }

  const double feasible_minimum =
      minimumConstraintValue(task.jacobian, lower, upper);
  const double feasible_maximum =
      maximumConstraintValue(task.jacobian, lower, upper);
  if (!std::isfinite(feasible_minimum) || !std::isfinite(feasible_maximum) ||
      feasible_minimum > feasible_maximum) {
    return result;
  }

  result.active = true;
  result.jacobian = task.jacobian;
  result.requested_lower = requested_lower;
  result.requested_upper = requested_upper;
  result.lower = requested_lower;
  result.upper = requested_upper;

  const double feasibility_margin =
      1.0e-7 * std::max({1.0, std::abs(feasible_minimum),
                         std::abs(feasible_maximum)});
  constexpr double kRecoveryBoxReserveFraction = 0.05;
  const double recovery_reserve = std::max(
      feasibility_margin,
      kRecoveryBoxReserveFraction *
          std::max(0.0, feasible_maximum - feasible_minimum));
  if (requested_lower > feasible_maximum - feasibility_margin) {
    // The requested recovery is beyond the joint box. Keep a one-sided row
    // near the best reachable rate without pinning every joint at a corner.
    result.lower = feasible_maximum - recovery_reserve;
    result.upper = std::numeric_limits<double>::infinity();
    result.feasibility_clipped = true;
  } else if (requested_upper < feasible_minimum + feasibility_margin) {
    result.lower = -std::numeric_limits<double>::infinity();
    result.upper = feasible_minimum + recovery_reserve;
    result.feasibility_clipped = true;
  }
  return result;
}

ArmAngleTask ArmAngleTaskBuilder::computeContinuity(
    const ArmAngleGeometryInput& geometry, double dt) {
  const Eigen::Vector3d shoulder_to_wrist =
      geometry.wrist_position - geometry.shoulder_position;
  const Eigen::Vector3d shoulder_to_elbow =
      geometry.elbow_position - geometry.shoulder_position;
  const bool geometry_valid = geometry.shoulder_position.allFinite() &&
      geometry.elbow_position.allFinite() &&
      geometry.wrist_position.allFinite() &&
      shoulder_to_wrist.norm() > kDirectionEpsilon;
  if (!geometry_valid) {
    return compute(geometry, {}, dt);
  }

  const Eigen::Vector3d axis = shoulder_to_wrist.normalized();
  const Eigen::Vector3d radial =
      shoulder_to_elbow - axis * axis.dot(shoulder_to_elbow);
  if (!continuity_initialized_ && radial.norm() > kDirectionEpsilon) {
    continuity_reference_ = radial.normalized();
    continuity_initialized_ = true;
    previous_projection_ = continuity_reference_;
    has_previous_projection_ = true;
    reference_projection_held_ = false;
  }

  ArmDirectionReference reference;
  reference.valid = continuity_initialized_;
  reference.direction = continuity_reference_;
  reference.source = ArmDirectionReferenceSource::kPrevious;
  ArmAngleTask result = compute(geometry, reference, dt);
  if (result.projected_reference.allFinite() &&
      result.projected_reference.norm() > kDirectionEpsilon) {
    continuity_reference_ = result.projected_reference.normalized();
    continuity_initialized_ = true;
    result.reference_source = ArmDirectionReferenceSource::kPrevious;
  }
  return result;
}

void ArmAngleTaskBuilder::reset() noexcept {
  reference_projection_held_ = false;
  reference_governor_held_ = false;
  reference_governor_error_sign_ = 1.0;
  has_previous_projection_ = false;
  previous_projection_.setZero();
  continuity_initialized_ = false;
  continuity_reference_.setZero();
  current_direction_initialized_ = false;
  current_direction_.setZero();
  control_error_initialized_ = false;
  previous_control_error_rad_ = 0.0;
  target_angle_initialized_ = false;
  previous_target_angle_rad_ = 0.0;
}

double estimateArmAngleJacobianDotTimesVelocity(
    const Vec7& current_jacobian, const Vec7& previous_jacobian,
    const Vec7& velocity, double dt, bool previous_valid) noexcept {
  if (!previous_valid || !current_jacobian.allFinite() ||
      !previous_jacobian.allFinite() || !velocity.allFinite() ||
      !std::isfinite(dt) || dt <= 0.0) {
    return 0.0;
  }
  const double value =
      ((current_jacobian - previous_jacobian) / dt).dot(velocity);
  return std::isfinite(value) ? value : 0.0;
}

ScalarJointTask applyArmAngleJointLimitRecovery(
    const ScalarJointTask& task, const Mat67& cartesian_jacobian,
    const JointPositionGuard& position_guard,
    double maximum_rate_rad_s) noexcept {
  ScalarJointTask result = task;
  if (!task.active || !position_guard.active ||
      !task.jacobian.allFinite() || !cartesian_jacobian.allFinite() ||
      !position_guard.position.allFinite() ||
      !position_guard.lower.allFinite() ||
      !position_guard.upper.allFinite() ||
      !std::isfinite(task.target) ||
      !std::isfinite(position_guard.soft_margin_rad) ||
      position_guard.soft_margin_rad <= 0.0 ||
      !std::isfinite(position_guard.recovery_gain_rad_s_per_rad) ||
      position_guard.recovery_gain_rad_s_per_rad <= 0.0 ||
      !std::isfinite(maximum_rate_rad_s) || maximum_rate_rad_s <= 0.0 ||
      (position_guard.lower.array() >= position_guard.upper.array()).any()) {
    return result;
  }

  Eigen::JacobiSVD<Mat67> svd(cartesian_jacobian, Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success ||
      svd.matrixV().cols() != kArmDof ||
      !svd.singularValues().allFinite()) {
    return result;
  }
  const double maximum_singular_value =
      svd.singularValues().size() > 0
          ? svd.singularValues().maxCoeff()
          : 0.0;
  const double rank_tolerance =
      Eigen::NumTraits<double>::epsilon() *
      static_cast<double>(std::max(cartesian_jacobian.rows(),
                                   cartesian_jacobian.cols())) *
      std::max(1.0, maximum_singular_value);
  Mat77 nullspace = Mat77::Zero();
  for (int column = 0; column < kArmDof; ++column) {
    if (column >= svd.singularValues().size() ||
        svd.singularValues()[column] <= rank_tolerance) {
      nullspace.noalias() += svd.matrixV().col(column) *
                             svd.matrixV().col(column).transpose();
    }
  }
  const Vec7 task_effective_direction = nullspace * task.jacobian;
  const double maximum_effective_component =
      task_effective_direction.cwiseAbs().maxCoeff();
  if (!task_effective_direction.allFinite() ||
      maximum_effective_component <= kDirectionEpsilon) {
    return result;
  }

  double recovery_activation = 0.0;
  for (int joint = 0; joint < kArmDof; ++joint) {
    const double half_range = 0.5 *
        (position_guard.upper[joint] - position_guard.lower[joint]);
    const double soft_margin =
        std::min(position_guard.soft_margin_rad, half_range);
    if (soft_margin <= kDirectionEpsilon) {
      continue;
    }
    const double distance = std::max(
        std::min(position_guard.position[joint] -
                     position_guard.lower[joint],
                 position_guard.upper[joint] -
                     position_guard.position[joint]),
        0.0);
    const double proximity =
        std::clamp(1.0 - distance / soft_margin, 0.0, 1.0);
    const double smooth_proximity =
        proximity * proximity * (3.0 - 2.0 * proximity);
    const double task_relevance =
        std::abs(task_effective_direction[joint]) /
        maximum_effective_component;
    recovery_activation =
        std::max(recovery_activation, smooth_proximity * task_relevance);
  }
  if (recovery_activation <= 0.0) {
    return result;
  }

  const Vec7 center =
      0.5 * (position_guard.lower + position_guard.upper);
  const Vec7 center_velocity =
      position_guard.recovery_gain_rad_s_per_rad *
      (center - position_guard.position);
  const double recovery_rate =
      task.jacobian.dot(nullspace * center_velocity);
  if (!std::isfinite(recovery_rate)) {
    return result;
  }
  result.target = std::clamp(
      (1.0 - recovery_activation) * task.target +
          recovery_activation * recovery_rate,
      -maximum_rate_rad_s, maximum_rate_rad_s);
  return result;
}

ArmAngleNullspaceResult refineArmAngleInNullspace(
    const Vec7& primary, const Mat67& cartesian_jacobian,
    const ScalarJointTask& task, const Vec7& lower, const Vec7& upper,
    double tolerance,
    const LinearJointConstraint& linear_constraint,
    const JointPositionGuard& position_guard,
    const LinearJointConstraint& secondary_linear_constraint) noexcept {
  ArmAngleNullspaceResult result;
  result.value = primary;
  if (!task.active || !primary.allFinite() ||
      !cartesian_jacobian.allFinite() || !task.jacobian.allFinite() ||
      !lower.allFinite() || !upper.allFinite() ||
      !std::isfinite(task.target) || !std::isfinite(task.activation) ||
      !std::isfinite(tolerance) || tolerance <= 0.0 ||
      task.activation <= 0.0 || (lower.array() > upper.array()).any() ||
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
            secondary_linear_constraint.upper)) ||
      (position_guard.active &&
       (!position_guard.position.allFinite() ||
        !position_guard.lower.allFinite() ||
        !position_guard.upper.allFinite() ||
        !std::isfinite(position_guard.soft_margin_rad) ||
        position_guard.soft_margin_rad <= 0.0 ||
        !std::isfinite(position_guard.recovery_gain_rad_s_per_rad) ||
        position_guard.recovery_gain_rad_s_per_rad <= 0.0 ||
        (position_guard.lower.array() >=
         position_guard.upper.array()).any()))) {
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
  Vec7 nullspace = Vec7::Zero();
  for (int column = rank; column < kArmDof; ++column) {
    const Vec7 basis = svd.matrixV().col(column);
    nullspace += basis * basis.dot(task.jacobian);
  }
  if (!nullspace.allFinite() || nullspace.norm() <= tolerance) {
    return result;
  }
  nullspace.normalize();
  const double nullspace_residual =
      (cartesian_jacobian * nullspace).norm();
  const double sensitivity = task.jacobian.dot(nullspace);
  if (!std::isfinite(nullspace_residual) ||
      nullspace_residual > tolerance || !std::isfinite(sensitivity) ||
      std::abs(sensitivity) <= tolerance) {
    return result;
  }

  result.before = task.jacobian.dot(primary);
  double alpha = std::clamp(task.activation, 0.0, 1.0) *
                 (task.target - result.before) / sensitivity;
  double alpha_lower = -std::numeric_limits<double>::infinity();
  double alpha_upper = std::numeric_limits<double>::infinity();
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (std::abs(nullspace[joint]) <= tolerance) {
      continue;
    }
    double one = (lower[joint] - primary[joint]) / nullspace[joint];
    double two = (upper[joint] - primary[joint]) / nullspace[joint];
    if (one > two) {
      std::swap(one, two);
    }
    alpha_lower = std::max(alpha_lower, one);
    alpha_upper = std::min(alpha_upper, two);
  }
  const auto intersectLinearConstraint =
      [&](const LinearJointConstraint& constraint) {
    if (!constraint.active) {
      return true;
    }
    const double before_constraint = constraint.jacobian.dot(primary);
    const double constraint_sensitivity = constraint.jacobian.dot(nullspace);
    if (!std::isfinite(before_constraint) ||
        !std::isfinite(constraint_sensitivity)) {
      return false;
    }
    if (std::abs(constraint_sensitivity) <= tolerance) {
      return before_constraint >= constraint.lower - tolerance &&
             before_constraint <= constraint.upper + tolerance;
    } else {
      double one =
          (constraint.lower - before_constraint) /
          constraint_sensitivity;
      double two =
          (constraint.upper - before_constraint) /
          constraint_sensitivity;
      if (one > two) {
        std::swap(one, two);
      }
      alpha_lower = std::max(alpha_lower, one);
      alpha_upper = std::min(alpha_upper, two);
    }
    return alpha_lower <= alpha_upper + tolerance;
  };
  if (!intersectLinearConstraint(linear_constraint) ||
      !intersectLinearConstraint(secondary_linear_constraint)) {
    return result;
  }
  if (alpha_lower > alpha_upper + tolerance) {
    return result;
  }
  alpha = std::clamp(alpha, alpha_lower, alpha_upper);
  if (position_guard.active) {
    double recovery_activation = 0.0;
    for (int joint = 0; joint < kArmDof; ++joint) {
      const double half_range = 0.5 *
          (position_guard.upper[joint] - position_guard.lower[joint]);
      const double soft_margin =
          std::min(position_guard.soft_margin_rad, half_range);
      if (soft_margin <= tolerance) {
        continue;
      }
      const double distance = std::max(
          std::min(position_guard.position[joint] -
                       position_guard.lower[joint],
                   position_guard.upper[joint] -
                       position_guard.position[joint]),
          0.0);
      const double normalized_proximity =
          std::clamp(1.0 - distance / soft_margin, 0.0, 1.0);
      // Smoothstep avoids a slope discontinuity when entering or leaving the
      // recovery buffer, which would otherwise appear as a qddot impulse.
      const double smooth_proximity = normalized_proximity *
          normalized_proximity * (3.0 - 2.0 * normalized_proximity);
      recovery_activation =
          std::max(recovery_activation, smooth_proximity);
    }
    if (recovery_activation > 0.0) {
      const Vec7 center =
          0.5 * (position_guard.lower + position_guard.upper);
      const Vec7 center_velocity =
          position_guard.recovery_gain_rad_s_per_rad *
          (center - position_guard.position);
      const double center_alpha = std::clamp(
          nullspace.dot(center_velocity - primary), alpha_lower, alpha_upper);
      alpha = std::clamp(
          (1.0 - recovery_activation) * alpha +
              recovery_activation * center_alpha,
          alpha_lower, alpha_upper);
    }
  }
  const Vec7 candidate = primary + alpha * nullspace;
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
  result.active = std::abs(alpha) > tolerance;
  result.alpha = alpha;
  result.after = task.jacobian.dot(candidate);
  result.cartesian_residual = cartesian_residual;
  return result;
}

}  // namespace tianji_qp_ik
