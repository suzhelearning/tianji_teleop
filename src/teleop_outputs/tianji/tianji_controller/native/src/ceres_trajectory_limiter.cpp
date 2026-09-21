#include "tianji_qp_ik/ceres_trajectory_limiter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace tianji_qp_ik {
namespace {

std::array<double, kArmDof> toArray(const Vec7& value) {
  std::array<double, kArmDof> result{};
  for (int index = 0; index < kArmDof; ++index) {
    result[static_cast<std::size_t>(index)] = value[index];
  }
  return result;
}

Vec7 toEigen(const std::array<double, kArmDof>& value) {
  Vec7 result;
  for (int index = 0; index < kArmDof; ++index) {
    result[index] = value[static_cast<std::size_t>(index)];
  }
  return result;
}

double maximumRatio(const Vec7& value, const Vec7& limit) {
  return (value.cwiseAbs().array() / limit.array()).maxCoeff();
}

}  // namespace

CeresTrajectoryLimiter7::CeresTrajectoryLimiter7(
    DlsPostureRuckigConfig config, ArmLimits limits, double initial_dt)
    : config_(std::move(config)),
      nominal_config_(config_),
      sampled_limits_(config_),
      limits_(std::move(limits)),
      otg_(initial_dt) {
  input_.control_interface = ruckig::ControlInterface::Position;
  input_.synchronization = ruckig::Synchronization::Time;
}

bool CeresTrajectoryLimiter7::validState(
    const ArmMotionState& state) const noexcept {
  if (!state.q.allFinite() || !state.qdot.allFinite() ||
      !state.qddot.allFinite()) {
    return false;
  }
  const Vec7 velocity_limit =
      (config_.velocity_scale * limits_.velocity)
          .cwiseMin(config_.max_velocity_rad_s);
  const double tolerance = config_.validation_tolerance;
  return (state.q.array() >= limits_.lower_position.array() - tolerance).all() &&
         (state.q.array() <= limits_.upper_position.array() + tolerance).all() &&
         (state.qdot.cwiseAbs().array() <=
          velocity_limit.array() + tolerance).all() &&
         (state.qddot.cwiseAbs().array() <=
          config_.max_acceleration_rad_s2.array() + tolerance).all();
}

bool CeresTrajectoryLimiter7::reset(const ArmMotionState& state) noexcept {
  if (!canReset(state)) {
    return false;
  }
  state_ = state;
  initialized_ = true;
  otg_.reset();
  return true;
}

bool CeresTrajectoryLimiter7::canReset(
    const ArmMotionState& state) const noexcept {
  return validState(state);
}

bool CeresTrajectoryLimiter7::beginSoftStart(SimulationSoftStartLimits limits) noexcept {
  if (!limits.valid() || !initialized_ || state_.qdot.cwiseAbs().maxCoeff() > 1e-6 ||
      state_.qddot.cwiseAbs().maxCoeff() > 1e-5) return false;
  soft_start_ = true;
  soft_start_limits_ = limits;
  close_seconds_ = 0.; ramp_seconds_ = -1.;
  config_ = nominal_config_;
  config_.max_velocity_rad_s = config_.max_velocity_rad_s.cwiseMin(Vec7::Constant(limits.velocity));
  config_.max_acceleration_rad_s2 = config_.max_acceleration_rad_s2.cwiseMin(Vec7::Constant(limits.acceleration));
  config_.max_jerk_rad_s3 = config_.max_jerk_rad_s3.cwiseMin(Vec7::Constant(limits.jerk));
  return true;
}

CeresTrajectoryResult CeresTrajectoryLimiter7::update(const Vec7& target,
                                                       double dt, bool allow_soft_start_ramp) {
  if (soft_start_ && ramp_seconds_ >= 0.) {
    const double t = std::clamp(ramp_seconds_ / .5, 0., 1.);
    const double blend = t*t*(3.-2.*t);
    const auto ramp = [blend](const Vec7& nominal, double cap) -> Vec7 {
      const Vec7 slow = nominal.cwiseMin(Vec7::Constant(cap));
      return slow + blend * (nominal-slow);
    };
    config_.max_velocity_rad_s = ramp(nominal_config_.max_velocity_rad_s,soft_start_limits_.velocity);
    config_.max_acceleration_rad_s2 = ramp(nominal_config_.max_acceleration_rad_s2,soft_start_limits_.acceleration);
    config_.max_jerk_rad_s3 = ramp(nominal_config_.max_jerk_rad_s3,soft_start_limits_.jerk);
  }
  CeresTrajectoryResult result;
  result.state = state_;
  if (!initialized_ || !target.allFinite() || !std::isfinite(dt) ||
      dt <= 0.0 ||
      (target.array() < limits_.lower_position.array()).any() ||
      (target.array() > limits_.upper_position.array()).any()) {
    result.detail = "invalid_joint_trajectory_input";
    return result;
  }

  const Vec7 velocity_limit =
      (config_.velocity_scale * limits_.velocity)
          .cwiseMin(config_.max_velocity_rad_s);
  otg_.delta_time = dt;
  input_.current_position = toArray(state_.q);
  input_.current_velocity = toArray(state_.qdot);
  input_.current_acceleration = toArray(state_.qddot);
  // A finished trajectory can retain velocities of a few floating-point ulps.
  // Replanning an effectively zero move with those residues can make Ruckig's
  // time synchronization fail (-111). Canonicalize only a fully settled arm;
  // do not suppress real motion or acceleration, snap positions, or relax limits.
  // Allow accumulated arithmetic roundoff, not just one operation's epsilon.
  constexpr double kRoundoff = 1e-12;
  const double position_scale =
      std::max({1.0, state_.q.cwiseAbs().maxCoeff(), target.cwiseAbs().maxCoeff()});
  if ((target - state_.q).cwiseAbs().maxCoeff() <= kRoundoff * position_scale &&
      (state_.qdot.cwiseAbs().array() <=
       kRoundoff * velocity_limit.array().max(1.0)).all() &&
      (state_.qddot.cwiseAbs().array() <=
       kRoundoff * config_.max_acceleration_rad_s2.array().max(1.0)).all()) {
    input_.current_velocity.fill(0.0);
    input_.current_acceleration.fill(0.0);
  }
  input_.target_position = toArray(target);
  input_.target_velocity = toArray(Vec7::Zero());
  input_.target_acceleration = toArray(Vec7::Zero());
  input_.max_velocity = toArray(velocity_limit);
  input_.max_acceleration = toArray(config_.max_acceleration_rad_s2);
  input_.max_jerk = toArray(config_.max_jerk_rad_s3);
  input_.synchronization = ruckig::Synchronization::Time;

  const ruckig::Result update_result = otg_.update(input_, output_);
  if (static_cast<int>(update_result) < 0) {
    result.detail = "joint_trajectory_ruckig_failed";
    return result;
  }

  ArmMotionState candidate;
  candidate.q = toEigen(output_.new_position);
  candidate.qdot = toEigen(output_.new_velocity);
  candidate.qddot = toEigen(output_.new_acceleration);
  bool position_projected = false;
  // Project the sampled position onto the caller's safety interval before
  // validation and committing the next reference state. Preserve non-finite
  // samples for rejection, and leave trajectory derivatives unchanged.
  if (candidate.q.allFinite()) {
    const Vec7 bounded_q = candidate.q.cwiseMax(limits_.lower_position)
                              .cwiseMin(limits_.upper_position);
    position_projected = (bounded_q.array() != candidate.q.array()).any();
    candidate.q = bounded_q;
  }
  bool velocity_projected = false;
  // Ruckig can return a discrete sample a few ulps (and, at high dynamic
  // limits, a small fixed-step amount) above max_velocity.  Keep the
  // configured limit authoritative by projecting only the sampled velocity;
  // the acceleration/jerk state remains the trajectory state generated by
  // Ruckig and is checked below as usual.
  if (candidate.qdot.allFinite()) {
    const Vec7 bounded_qdot =
        candidate.qdot.cwiseMax(-velocity_limit).cwiseMin(velocity_limit);
    velocity_projected = !bounded_qdot.isApprox(candidate.qdot, 0.0);
    candidate.qdot = bounded_qdot;
  }
  result.jerk = (candidate.qddot - state_.qddot) / dt;
  result.velocity_ratio = maximumRatio(candidate.qdot, velocity_limit);
  result.acceleration_ratio = maximumRatio(
      candidate.qddot, config_.max_acceleration_rad_s2);
  result.jerk_ratio = maximumRatio(result.jerk, config_.max_jerk_rad_s3);
  const double maximum_ratio = std::max(
      {result.velocity_ratio, result.acceleration_ratio, result.jerk_ratio});
  if (!validState(candidate) || !result.jerk.allFinite() ||
      maximum_ratio > 1.0 + config_.validation_tolerance) {
    result.detail = "joint_trajectory_output_rejected";
    return result;
  }

  state_ = candidate;
  sampled_limits_ = config_; // Limits for this accepted sample, before ramp advances.
  if (soft_start_) {
    if (ramp_seconds_ < 0.) {
      close_seconds_ = allow_soft_start_ramp && (target-state_.q).cwiseAbs().maxCoeff() < .08
          ? close_seconds_ + dt : 0.;
      if (close_seconds_ >= .2) ramp_seconds_ = 0.;
    } else {
      ramp_seconds_ += dt;
      if (ramp_seconds_ >= .5) { soft_start_ = false; config_ = nominal_config_; }
    }
  }
  result.state = state_;
  result.accepted = true;
  result.detail = position_projected
                      ? "joint_trajectory_position_projected"
                  : velocity_projected
                      ? "joint_trajectory_velocity_projected"
                  : update_result == ruckig::Result::Finished
                      ? "joint_trajectory_finished"
                      : "joint_trajectory_working";
  return result;
}

}  // namespace tianji_qp_ik
