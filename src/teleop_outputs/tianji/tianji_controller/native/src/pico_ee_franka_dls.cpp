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
// plane, matching the replay arm-motion measurement.
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

}  // namespace
PicoEeFrankaDlsIk7::PicoEeFrankaDlsIk7(
    IterativeDlsConfig dls_config, PicoEeFrankaDlsConfig config,
    double joint_margin_rad)
    : dls_(std::move(dls_config), joint_margin_rad),
      config_(std::move(config)),
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
  return std::min({config_.max_velocity_rad_s[joint],
                   input.limits.velocity[joint], symmetric_bound});
}

bool PicoEeFrankaDlsIk7::validInput(
    const PicoEeFrankaDlsInput& input) const noexcept {
  if (!std::isfinite(config_.max_arm_plane_rate_rad_s) ||
      config_.max_arm_plane_rate_rad_s < 0.0) return false;
  if (!input.seed.allFinite() || !input.seed_velocity.allFinite() ||
      !input.seed_acceleration.allFinite() || !finiteLimits(input.limits) ||
      !finiteVelocityBounds(input.velocity_bounds) ||
      !input.home_reference.allFinite() ||
      !std::isfinite(input.dt) || input.dt <= 0.0 ||
      !std::isfinite(joint_margin_rad_) || joint_margin_rad_ < 0.0 ||
      !config_.max_velocity_rad_s.allFinite() ||
      (config_.max_velocity_rad_s.array() <= 0.0).any()) {
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

    goal_ = initial.q.cwiseMax(safe_lower_).cwiseMin(safe_upper_);
        pending_reset_valid_ = false;
    initialized_ = true;
    return true;
}

void PicoEeFrankaDlsIk7::reset(const ArmMotionState& state) noexcept {
  pending_reset_ = state;
  pending_reset_valid_ = state.q.allFinite() && state.qdot.allFinite() &&
                          state.qddot.allFinite();
  goal_ = state.q;
  initialized_ = false;
}

PicoEeFrankaDlsResult PicoEeFrankaDlsIk7::solve(
    const PicoEeFrankaDlsInput& input) {
  PicoEeFrankaDlsResult result;
  result.goal = goal_;
  if (!validInput(input)) {
    result.target_held = true;
    result.detail = "pico_ee_franka_dls_invalid_input";
    return result;
  }
  if (!initialized_ && !initialize(input)) {
    result.target_held = true;
    result.detail = "pico_ee_franka_dls_init_failed";
    return result;
  }

  if (input.target_valid && !input.target_stale && input.evaluate) {
    // Keep IK continuity on the previous best DLS solution. The current
    // model reference initializes the raw goal after reset, but Ruckig's
    // sampled reference must not feed back into the next IK seed.
    const Vec7 ik_seed = goal_;
    FrankaPoseDlsInput dls_input;
    dls_input.target = input.target;
    dls_input.seed = ik_seed;
    dls_input.limits = input.limits;
    // Retain the lowest-merit feasible
    // candidate evaluated in this cycle, including the seed on no-improvement.
    dls_input.return_final_iterate = false;
    dls_input.fixed_posture_reference_valid = true;
    dls_input.fixed_posture_reference = input.home_reference;
    dls_input.evaluate = input.evaluate;
    if (config_.max_arm_plane_rate_rad_s > 0.0) {
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
      const double angle_limit = config_.max_arm_plane_rate_rad_s * input.dt;
      if (config_.arm_plane_direction_guidance) {
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
    result.state.q = goal_;
    result.qdot = (goal_ - input.seed) / input.dt;
    result.state.qdot = result.qdot;
    result.state.qddot =
        (result.qdot - input.seed_velocity) / input.dt;
    result.jerk =
        (result.state.qddot - input.seed_acceleration) / input.dt;
    result.accepted = goal_.allFinite() && result.qdot.allFinite() &&
                      result.state.qddot.allFinite() &&
                      result.jerk.allFinite() &&
                      (goal_.array() >= safe_lower_.array()).all() &&
                      (goal_.array() <= safe_upper_.array()).all();
    result.detail = result.accepted
                        ? "pico_ee_franka_dls_raw_goal"
                        : "pico_ee_franka_dls_direct_state_invalid";
    return result;
 }

}  // namespace tianji_qp_ik
