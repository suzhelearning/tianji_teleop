#include "tianji_qp_ik/pico_ee_franka_dls.hpp"

#include <Eigen/Geometry>
#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tianji_qp_ik {
namespace {

bool finiteLimits(const ArmLimits& limits) noexcept {
  return limits.lower_position.allFinite() &&
         limits.upper_position.allFinite() && limits.velocity.allFinite() &&
         (limits.lower_position.array() <= limits.upper_position.array())
             .all() &&
         (limits.velocity.array() > 0.0).all();
}

// Transport the seed's elbow radial vector into the candidate shoulder-wrist
// plane, matching the Ceres guard and the replay arm-motion measurement.
bool armPlane(const ArmKinematicSample& sample, Eigen::Vector3d& axis,
              Eigen::Vector3d& radial) {
  axis = sample.wrist_position - sample.shoulder_position;
  radial = sample.elbow_position - sample.shoulder_position;
  if (!axis.allFinite() || !radial.allFinite() || axis.norm() < 1e-8)
    return false;
  axis.normalize();
  radial -= axis * axis.dot(radial);
  return radial.allFinite() && radial.norm() >= 1e-8;
}

bool finiteVelocityBounds(const JointVelocityBounds& bounds) noexcept {
  return bounds.lower.allFinite() && bounds.upper.allFinite() &&
         (bounds.lower.array() <= bounds.upper.array()).all();
}

bool finitePositive(const Vec7& values) noexcept {
  return values.allFinite() && (values.array() > 0.0).all();
}

double maximumRatio(const Vec7& value, const Vec7& limit) noexcept {
  if (!value.allFinite() || !finitePositive(limit)) {
    return std::numeric_limits<double>::infinity();
  }
  return (value.cwiseAbs().array() / limit.array()).maxCoeff();
}

double limitOutwardAcceleration(double velocity, double acceleration,
                                double max_velocity, double max_jerk,
                                double dt) noexcept {
  if (velocity == 0.0 || velocity * acceleration <= 0.0 ||
      !std::isfinite(max_velocity) || max_velocity <= 0.0 ||
      !std::isfinite(max_jerk) || max_jerk <= 0.0 || !std::isfinite(dt) ||
      dt <= 0.0) {
    return acceleration;
  }
  const double speed_gap = std::max(0.0, max_velocity - std::abs(velocity));
  const double jerk_dt = max_jerk * dt;
  const double acceleration_limit = std::max(
      0.0, -jerk_dt +
               std::sqrt(jerk_dt * jerk_dt + 2.0 * max_jerk * speed_gap));
  const double signed_limit = std::copysign(acceleration_limit, velocity);
  return velocity > 0.0 ? std::min(acceleration, signed_limit)
                        : std::max(acceleration, signed_limit);
}

}  // namespace

RealTimeConstrainedPlanner::RealTimeConstrainedPlanner(
    double omega_c, double sample_time, double max_velocity,
    double max_acceleration, double max_jerk, double lower_position,
    double upper_position) noexcept {
  if (!configure(omega_c, sample_time, max_velocity, max_acceleration,
                 max_jerk, lower_position, upper_position)) {
    configured_ = false;
  }
}

bool RealTimeConstrainedPlanner::configure(
    double omega_c, double sample_time, double max_velocity,
    double max_acceleration, double max_jerk, double lower_position,
    double upper_position) noexcept {
  configured_ = false;
  if (!std::isfinite(omega_c) || omega_c <= 0.0 ||
      !std::isfinite(sample_time) || sample_time <= 0.0 ||
      !std::isfinite(max_velocity) || max_velocity <= 0.0 ||
      !std::isfinite(max_acceleration) || max_acceleration <= 0.0 ||
      !std::isfinite(max_jerk) || max_jerk <= 0.0 ||
      !std::isfinite(lower_position) || !std::isfinite(upper_position) ||
      lower_position >= upper_position) {
    return false;
  }
  omega_c_ = omega_c;
  sample_time_ = sample_time;
  max_velocity_ = max_velocity;
  max_acceleration_ = max_acceleration;
  max_jerk_ = max_jerk;
  lower_position_ = lower_position;
  upper_position_ = upper_position;
  recalculateCoefficients();
  configured_ = true;
  return reset(std::clamp(0.0, lower_position_, upper_position_));
}

bool RealTimeConstrainedPlanner::reset(double position, double velocity,
                                        double acceleration) noexcept {
  if (!configured_ || !std::isfinite(position) || !std::isfinite(velocity) ||
      !std::isfinite(acceleration)) {
    return false;
  }
  position_ = std::clamp(position, lower_position_, upper_position_);
  velocity_ = std::clamp(velocity, -max_velocity_, max_velocity_);
  acceleration_ =
      std::clamp(acceleration, -max_acceleration_, max_acceleration_);
  if ((position_ <= lower_position_ && velocity_ < 0.0) ||
      (position_ >= upper_position_ && velocity_ > 0.0)) {
    velocity_ = 0.0;
  }
  if ((position_ <= lower_position_ && acceleration_ < 0.0) ||
      (position_ >= upper_position_ && acceleration_ > 0.0)) {
    acceleration_ = 0.0;
  }
  acceleration_ = limitOutwardAcceleration(
      velocity_, acceleration_, max_velocity_, max_jerk_, sample_time_);
  jerk_ = 0.0;
  target_ = position_;
  return true;
}

void RealTimeConstrainedPlanner::setTarget(double target) noexcept {
  if (!configured_) {
    return;
  }
  target_ = std::isfinite(target)
                ? std::clamp(target, lower_position_, upper_position_)
                : position_;
}

bool RealTimeConstrainedPlanner::update(double dt) noexcept {
  if (!configured_ || !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(position_) || !std::isfinite(velocity_) ||
      !std::isfinite(acceleration_) || !std::isfinite(jerk_) ||
      !std::isfinite(target_)) {
    return false;
  }

  // The reference planner uses fourth-order error feedback.  The control
  // input is a snap-like quantity; integrating it updates the bounded jerk
  // state before acceleration, velocity, and position are updated.
  const double position_error = position_ - target_;
  const double unconstrained_snap =
      -k0_ * position_error - k1_ * velocity_ - k2_ * acceleration_ -
      k3_ * jerk_;
  if (!std::isfinite(unconstrained_snap)) {
    return false;
  }

  const double next_jerk = std::clamp(
      jerk_ + unconstrained_snap * dt, -max_jerk_, max_jerk_);
  double next_acceleration = std::clamp(
      acceleration_ + next_jerk * dt, -max_acceleration_, max_acceleration_);

  // A direct velocity clamp can make the velocity jump to the limit while
  // leaving acceleration unchanged.  The resulting finite-difference jerk
  // is then much larger than max_jerk_.  Reserve the velocity headroom needed
  // to bring an outward acceleration to zero with the jerk bound, and reduce
  // the candidate acceleration before integrating velocity.
  next_acceleration = limitOutwardAcceleration(
      velocity_, next_acceleration, max_velocity_, max_jerk_, dt);

  double next_velocity = velocity_ + next_acceleration * dt;
  if (std::abs(next_velocity) > max_velocity_) {
    // This path is only expected for a state supplied by reset that was
    // already outside the jerk-feasible velocity envelope.  Keep the hard
    // velocity bound and make acceleration agree with the applied velocity
    // increment so the published state remains self-consistent.
    next_velocity = std::clamp(next_velocity, -max_velocity_, max_velocity_);
    next_acceleration = (next_velocity - velocity_) / dt;
  }
  const double proposed_position = position_ + next_velocity * dt;
  const double next_position =
      std::clamp(proposed_position, lower_position_, upper_position_);

  double applied_velocity = next_velocity;
  double applied_acceleration = next_acceleration;
  double applied_jerk = (next_acceleration - acceleration_) / dt;
  if (next_position != proposed_position) {
    // A position clamp is a hard state projection.  Clear the derivatives at
    // the wall so the next cycle cannot immediately push back through it.
    applied_velocity = (next_position - position_) / dt;
    applied_acceleration = 0.0;
    applied_jerk = 0.0;
  }

  if (!std::isfinite(next_position) || !std::isfinite(applied_velocity) ||
      !std::isfinite(applied_acceleration) || !std::isfinite(applied_jerk) ||
      std::abs(applied_jerk) > max_jerk_ + 1.0e-8) {
    return false;
  }
  position_ = next_position;
  velocity_ = std::clamp(applied_velocity, -max_velocity_, max_velocity_);
  acceleration_ = std::clamp(applied_acceleration, -max_acceleration_,
                              max_acceleration_);
  jerk_ = std::clamp(applied_jerk, -max_jerk_, max_jerk_);
  return true;
}

void RealTimeConstrainedPlanner::getState(double& position, double& velocity,
                                           double& acceleration,
                                           double& jerk) const noexcept {
  position = position_;
  velocity = velocity_;
  acceleration = acceleration_;
  jerk = jerk_;
}

void RealTimeConstrainedPlanner::recalculateCoefficients() noexcept {
  const double omega_squared = omega_c_ * omega_c_;
  const double omega_cubed = omega_squared * omega_c_;
  k0_ = omega_squared * omega_squared;
  k1_ = 4.0 * omega_cubed;
  k2_ = 6.0 * omega_squared;
  k3_ = 4.0 * omega_c_;
}

PicoEeFrankaDlsIk7::PicoEeFrankaDlsIk7(
    IterativeDlsConfig dls_config, PicoEeFrankaDlsConfig planner_config,
    double joint_margin_rad)
    : dls_(std::move(dls_config), joint_margin_rad),
      planner_config_(std::move(planner_config)),
      joint_margin_rad_(joint_margin_rad) {}

double PicoEeFrankaDlsIk7::effectiveVelocityLimit(
    const PicoEeFrankaDlsInput& input, int joint) const noexcept {
  if (joint < 0 || joint >= kArmDof ||
      !std::isfinite(input.limits.velocity[joint]) ||
      !std::isfinite(input.velocity_bounds.lower[joint]) ||
      !std::isfinite(input.velocity_bounds.upper[joint])) {
    return 0.0;
  }
  const double symmetric_bound = std::min(
      std::abs(input.velocity_bounds.lower[joint]),
      std::abs(input.velocity_bounds.upper[joint]));
  return std::min({planner_config_.max_velocity_rad_s[joint],
                   input.limits.velocity[joint], symmetric_bound});
}

bool PicoEeFrankaDlsIk7::validInput(
    const PicoEeFrankaDlsInput& input) const noexcept {
  if (!std::isfinite(planner_config_.max_arm_plane_rate_rad_s) ||
      planner_config_.max_arm_plane_rate_rad_s < 0.0 ||
      (planner_config_.max_arm_plane_rate_rad_s > 0.0 &&
       (planner_config_.planner_enabled || planner_config_.direct_velocity_limit_enabled)))
    return false;
  if (!input.seed.allFinite() || !input.seed_velocity.allFinite() ||
      !input.seed_acceleration.allFinite() || !finiteLimits(input.limits) ||
      !finiteVelocityBounds(input.velocity_bounds) ||
      !input.home_reference.allFinite() ||
      !std::isfinite(input.dt) || input.dt <= 0.0 ||
      !std::isfinite(joint_margin_rad_) || joint_margin_rad_ < 0.0 ||
      !planner_config_.bandwidth_rad_s.allFinite() ||
      !planner_config_.max_velocity_rad_s.allFinite() ||
      !planner_config_.max_acceleration_rad_s2.allFinite() ||
      !planner_config_.max_jerk_rad_s3.allFinite() ||
      (planner_config_.bandwidth_rad_s.array() <= 0.0).any() ||
      (planner_config_.max_velocity_rad_s.array() <= 0.0).any() ||
      (planner_config_.max_acceleration_rad_s2.array() <= 0.0).any() ||
      (planner_config_.max_jerk_rad_s3.array() <= 0.0).any() ||
      !std::isfinite(planner_config_.maximum_target_joint_step_rad) ||
      planner_config_.maximum_target_joint_step_rad <= 0.0 ||
      !std::isfinite(planner_config_.maximum_target_step_norm_rad) ||
      planner_config_.maximum_target_step_norm_rad <= 0.0 ||
      !std::isfinite(planner_config_.planner_validation_tolerance) ||
      planner_config_.planner_validation_tolerance <= 0.0) {
    return false;
  }
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (!(effectiveVelocityLimit(input, joint) > 0.0)) {
      return false;
    }
  }
  return true;
}

bool PicoEeFrankaDlsIk7::initialize(
    const PicoEeFrankaDlsInput& input) noexcept {
  if (!validInput(input)) {
    return false;
  }
  safe_lower_ = input.limits.lower_position.array() + joint_margin_rad_;
  safe_upper_ = input.limits.upper_position.array() - joint_margin_rad_;
  if (!safe_lower_.allFinite() || !safe_upper_.allFinite() ||
      (safe_lower_.array() >= safe_upper_.array()).any()) {
    return false;
  }

  const ArmMotionState initial =
      pending_reset_valid_
          ? pending_reset_
          : ArmMotionState{input.seed, input.seed_velocity,
                           input.seed_acceleration};
  if (!initial.q.allFinite() || !initial.qdot.allFinite() ||
      !initial.qddot.allFinite() ||
      (initial.q.array() < input.limits.lower_position.array()).any() ||
      (initial.q.array() > input.limits.upper_position.array()).any()) {
    return false;
  }

  if (!planner_config_.planner_enabled) {
    goal_ = initial.q.cwiseMax(safe_lower_).cwiseMin(safe_upper_);
    planner_target_ = goal_;
    pending_reset_valid_ = false;
    initialized_ = true;
    return true;
  }

  for (int joint = 0; joint < kArmDof; ++joint) {
    if (!planners_[joint].configure(
            planner_config_.bandwidth_rad_s[joint], input.dt,
            effectiveVelocityLimit(input, joint),
            planner_config_.max_acceleration_rad_s2[joint],
            planner_config_.max_jerk_rad_s3[joint], safe_lower_[joint],
            safe_upper_[joint]) ||
        !planners_[joint].reset(initial.q[joint], initial.qdot[joint],
                                initial.qddot[joint])) {
      initialized_ = false;
      return false;
    }
    goal_[joint] = planners_[joint].position();
  }
  planner_target_ = goal_;
  pending_reset_valid_ = false;
  initialized_ = true;
  return true;
}

void PicoEeFrankaDlsIk7::reset(const ArmMotionState& state) noexcept {
  pending_reset_ = state;
  pending_reset_valid_ = state.q.allFinite() && state.qdot.allFinite() &&
                          state.qddot.allFinite();
  goal_ = state.q;
  planner_target_ = state.q;
  initialized_ = false;
}

PicoEeFrankaDlsResult PicoEeFrankaDlsIk7::solve(
    const PicoEeFrankaDlsInput& input) {
  PicoEeFrankaDlsResult result;
  result.goal = goal_;
  result.planner_target = planner_target_;

  if (!validInput(input)) {
    result.target_held = true;
    result.detail = "pico_ee_franka_dls_invalid_input";
    return result;
  }
  if (!initialized_ && !initialize(input)) {
    result.target_held = true;
    result.detail = "pico_ee_franka_dls_planner_init_failed";
    return result;
  }

  if (input.target_valid && !input.target_stale && input.evaluate) {
    // Keep IK continuity on the previous best DLS solution.  input.seed is
    // the current model reference used to initialize and publish the
    // trajectory planner; feeding it back here would make the smoother alter
    // the next IK solve.  The first solve uses the reset model reference
    // because goal_ is initialized from it.
    const Vec7 ik_seed = goal_;
    FrankaPoseDlsInput dls_input;
    dls_input.target = input.target;
    dls_input.seed = ik_seed;
    dls_input.limits = input.limits;
    // Match Ceres' publication policy: retain the lowest-merit feasible
    // candidate evaluated in this cycle, including the seed on no-improvement.
    dls_input.return_final_iterate = false;
    dls_input.fixed_posture_reference_valid = true;
    dls_input.fixed_posture_reference = input.home_reference;
    dls_input.evaluate = input.evaluate;
    if (planner_config_.max_arm_plane_rate_rad_s > 0.0) {
      Eigen::Vector3d axis, radial;
      bool valid = false;
      try {
        valid = armPlane(input.evaluate(ik_seed), axis, radial);
      } catch (...) {
        valid = false;
      }
      if (!valid) {
        result.detail = "franka_invalid_arm_plane";
        return result;
      }
      const double radius_floor = std::min(radial.norm(), 0.025);
      const double angle_limit = planner_config_.max_arm_plane_rate_rad_s * input.dt;
      if (planner_config_.arm_plane_direction_guidance) {
        const auto angle = [radial](const ArmKinematicSample& sample) {
          Eigen::Vector3d a, r;
          if (!armPlane(sample, a, r))
            return std::numeric_limits<double>::quiet_NaN();
          Eigen::Vector3d ref = radial - a * a.dot(radial);
          if (ref.norm() < 1e-8)
            return std::numeric_limits<double>::quiet_NaN();
          ref.normalize();
          r.normalize();
          return std::atan2(a.dot(ref.cross(r)), ref.dot(r));
        };
        dls_input.constrain_step = [angle, angle_limit, &input](
            const Vec7& q, const ArmKinematicSample& sample,
            double damping, const Vec7& proposed) -> Vec7 {
          const double current_angle = angle(sample);
          Vec7 gradient;
          constexpr double epsilon = 1e-6;
          for (int j = 0; j < kArmDof; ++j) {
            Vec7 plus = q, minus = q;
            plus[j] += epsilon;
            minus[j] -= epsilon;
            gradient[j] = (angle(input.evaluate(plus)) -
                           angle(input.evaluate(minus))) / (2.0 * epsilon);
          }
          if (!std::isfinite(current_angle) || !gradient.allFinite())
            return proposed;
          const double predicted = current_angle + gradient.dot(proposed);
          const double bound = 0.95 * angle_limit;
          if (std::abs(predicted) <= bound) return proposed;
          // Equality-constrained DLS correction in the local task metric:
          // exploit redundancy first, then trade task error if necessary.
          Mat77 h = sample.tcp_jacobian.transpose() * sample.tcp_jacobian;
          h.diagonal().array() += std::max(1e-8, damping * damping);
          const Vec7 direction = h.ldlt().solve(gradient);
          const double denominator = gradient.dot(direction);
          if (!direction.allFinite() || denominator < 1e-12) return proposed;
          return proposed - direction *
              ((predicted - std::clamp(predicted, -bound, bound)) / denominator);
        };
      }
      // Anchor stays fixed for ALL iterations: each iteration cannot spend a
      // fresh cycle's angular budget. Only feasible trials enter best selection.
      dls_input.candidate_feasible = [radial, radius_floor, angle_limit](
          const ArmKinematicSample& sample) {
        Eigen::Vector3d next_axis, next_radial;
        if (!armPlane(sample, next_axis, next_radial) ||
            next_radial.norm() + 1e-10 < radius_floor)
          return false;
        Eigen::Vector3d ref = radial - next_axis * next_axis.dot(radial);
        if (ref.norm() < 1e-8) return false;
        ref.normalize();
        next_radial.normalize();
        const double angle = std::atan2(next_axis.dot(ref.cross(next_radial)),
                                        ref.dot(next_radial));
        return std::abs(angle) <= angle_limit + 1e-10;
      };
    }
    result.dls = dls_.solve(dls_input);
    if (result.dls.status == PoseDlsStatus::kRejected) {
      // A valid target can still be rejected by the iterative solver. Do not
      // let direct mode publish the previous goal as if this solve succeeded.
      result.target_held = true;
      result.detail = "pico_ee_franka_dls_rejected_candidate";
      return result;
    }
    if (result.dls.q.allFinite()) {
      goal_ = frankaClampPostureReferenceToInterior(result.dls.q, input.limits,
                                              joint_margin_rad_);
      if (!planner_config_.planner_enabled) {
        if (planner_config_.direct_velocity_limit_enabled) {
          // Preserve the solved direction with a common scale, without OTG.
          const Vec7 step = goal_ - input.seed;
          Vec7 velocity_limit = Vec7::Zero();
          for (int joint = 0; joint < kArmDof; ++joint) {
            velocity_limit[joint] = effectiveVelocityLimit(input, joint);
          }
          const Vec7 limit = velocity_limit * input.dt;
          const double ratio = (step.cwiseAbs().array() / limit.array()).maxCoeff();
          if (ratio > 1.0) goal_ = input.seed + step / ratio;
        }
        planner_target_ = goal_;
      } else {
        const double maximum_joint_step =
            planner_config_.maximum_target_joint_step_rad;
        Vec7 target_step = (goal_ - planner_target_)
                               .cwiseMax(Vec7::Constant(-maximum_joint_step))
                               .cwiseMin(Vec7::Constant(maximum_joint_step));
        const double target_step_norm = target_step.norm();
        if (target_step_norm > planner_config_.maximum_target_step_norm_rad) {
          target_step *=
              planner_config_.maximum_target_step_norm_rad / target_step_norm;
        }
        planner_target_ = (planner_target_ + target_step)
                              .cwiseMax(safe_lower_)
                              .cwiseMin(safe_upper_);
      }
    } else {
      result.target_held = true;
    }
  } else {
    result.target_held = true;
    result.dls.detail = input.target_stale
                            ? "pico_ee_franka_dls_target_stale"
                            : "pico_ee_franka_dls_target_held";
  }

  result.goal = goal_;
  result.planner_target = planner_target_;
  if (!planner_config_.planner_enabled) {
    result.planner_state.q = goal_;
    result.qdot = (goal_ - input.seed) / input.dt;
    result.planner_state.qdot = result.qdot;
    result.planner_state.qddot =
        (result.qdot - input.seed_velocity) / input.dt;
    result.planner_jerk =
        (result.planner_state.qddot - input.seed_acceleration) / input.dt;
    result.accepted = goal_.allFinite() && result.qdot.allFinite() &&
                      result.planner_state.qddot.allFinite() &&
                      result.planner_jerk.allFinite() &&
                      (goal_.array() >= safe_lower_.array()).all() &&
                      (goal_.array() <= safe_upper_.array()).all();
    result.detail = result.accepted
                        ? "pico_ee_franka_dls_planner_bypassed"
                        : "pico_ee_franka_dls_direct_state_invalid";
    return result;
  }
  for (int joint = 0; joint < kArmDof; ++joint) {
    planners_[joint].setTarget(planner_target_[joint]);
  }
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (!planners_[joint].update(input.dt)) {
      result.detail = "pico_ee_franka_dls_planner_rejected";
      return result;
    }
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    double jerk = 0.0;
    planners_[joint].getState(position, velocity, acceleration, jerk);
    result.planner_state.q[joint] = position;
    result.planner_state.qdot[joint] = velocity;
    result.planner_state.qddot[joint] = acceleration;
    result.planner_jerk[joint] = jerk;
  }

  const Vec7 velocity_limit = [&input, this]() {
    Vec7 limits = Vec7::Zero();
    for (int joint = 0; joint < kArmDof; ++joint) {
      limits[joint] = effectiveVelocityLimit(input, joint);
    }
    return limits;
  }();
  const double tolerance = planner_config_.planner_validation_tolerance;
  const bool state_valid =
      result.planner_state.q.allFinite() &&
      result.planner_state.qdot.allFinite() &&
      result.planner_state.qddot.allFinite() && result.planner_jerk.allFinite() &&
      (result.planner_state.q.array() >= safe_lower_.array() - tolerance)
          .all() &&
      (result.planner_state.q.array() <= safe_upper_.array() + tolerance)
          .all() &&
      (result.planner_state.qdot.array() <= velocity_limit.array() + tolerance)
          .all() &&
      (result.planner_state.qdot.array() >= -velocity_limit.array() - tolerance)
          .all() &&
      (result.planner_state.qddot.array() <=
       planner_config_.max_acceleration_rad_s2.array() + tolerance)
          .all() &&
      (result.planner_state.qddot.array() >=
       -planner_config_.max_acceleration_rad_s2.array() - tolerance)
          .all() &&
      (result.planner_jerk.array() <=
       planner_config_.max_jerk_rad_s3.array() + tolerance)
          .all() &&
      (result.planner_jerk.array() >=
       -planner_config_.max_jerk_rad_s3.array() - tolerance)
          .all();
  if (!state_valid) {
    result.detail = "pico_ee_franka_dls_planner_state_invalid";
    return result;
  }

  result.qdot = result.planner_state.qdot;
  result.velocity_ratio = maximumRatio(result.qdot, velocity_limit);
  result.acceleration_ratio = maximumRatio(
      result.planner_state.qddot, planner_config_.max_acceleration_rad_s2);
  result.jerk_ratio =
      maximumRatio(result.planner_jerk, planner_config_.max_jerk_rad_s3);
  result.planner_accepted = true;
  result.accepted = true;
  result.detail = result.target_held
                      ? "pico_ee_franka_dls_target_held"
                      : "pico_ee_franka_dls_realtime_planner";
  return result;
}

}  // namespace tianji_qp_ik
