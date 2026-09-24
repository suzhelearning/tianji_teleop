#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/shared_root_options.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
DlsPostureRuckigConfig DualArmController::trajectorySampleLimits(ArmSide side) const {
  const auto& limiter = side == ArmSide::kLeft ? left_smoother_ : right_smoother_;
  if (limiter) return limiter->sampledLimits();
  return config_.pico_ee_franka_dls.post_smoothing;
}
bool DualArmController::beginSimulationSoftStart(SimulationSoftStartLimits limits) {
  if (!limits.valid() || !config_.controller.model_state_only) return false;
  for (auto side : {ArmSide::kLeft, ArmSide::kRight}) {
    const auto state = referenceState(side);
    if (!state.q.allFinite() || !state.qdot.allFinite() || !state.qddot.allFinite() ||
        state.qdot.cwiseAbs().maxCoeff()>1e-6 || state.qddot.cwiseAbs().maxCoeff()>1e-5)
      return false;
  }
  simulation_soft_start_pending_ = true;
  simulation_soft_start_limits_ = limits;
  return true;
}
bool DualArmController::resetEpisodeReference(
    const std::array<Vec7, 2>& measured_q, DualArmTargets& tcp) {
  const std::array<ArmMotionState, 2> stationary{
      ArmMotionState{measured_q[0], Vec7::Zero(), Vec7::Zero()},
      ArmMotionState{measured_q[1], Vec7::Zero(), Vec7::Zero()}};
  if (!left_smoother_->canReset(stationary[0]) ||
      !right_smoother_->canReset(stationary[1])) return false;
  const auto left_tcp = dls_kinematics_->sampleTcp(ArmSide::kLeft, measured_q[0]).tcp_pose;
  const auto right_tcp = dls_kinematics_->sampleTcp(ArmSide::kRight, measured_q[1]).tcp_pose;
  if (!left_tcp.position.allFinite() || !left_tcp.rotation.allFinite() ||
      !right_tcp.position.allFinite() || !right_tcp.rotation.allFinite()) return false;
  // All potentially rejecting checks precede this bilateral commit.
  (void)left_smoother_->reset(stationary[0]);
  (void)right_smoother_->reset(stationary[1]);
  left_smoother_->preserveConstraints();
  right_smoother_->preserveConstraints();
  left_franka_dls_->reset(stationary[0]); right_franka_dls_->reset(stationary[1]);
  left_state_ = {measured_q[0], Vec7::Zero(), Vec7::Zero()};
  right_state_ = {measured_q[1], Vec7::Zero(), Vec7::Zero()};
  robot_.setArmState(ArmSide::kLeft, measured_q[0], Vec7::Zero());
  robot_.setArmState(ArmSide::kRight, measured_q[1], Vec7::Zero());
  robot_.forward();
  episode_posture_ = measured_q; episode_relative_ = true;
  simulation_soft_start_pending_ = false;
  left_dls_ = {}; right_dls_ = {};
  tcp = {left_tcp, right_tcp};
  return true;
}
void DualArmController::initializeDls() {
  const auto& d = config_.pico_ee_franka_dls;
  const auto& p = d.post_smoothing;
  const Vec7 home_left = d.home_left_rad;
  const Vec7 home_right = d.home_right_rad;
  // Model references remain independent of hardware authority.
  if (!d.enabled || !config_.controller.model_state_only ||
      !p.enabled || p.velocity_scale != 1.0 ||
      !home_left.allFinite() || !home_right.allFinite() ||
      !p.max_velocity_rad_s.allFinite() || !p.max_acceleration_rad_s2.allFinite() ||
      !p.max_jerk_rad_s3.allFinite() || (p.max_velocity_rad_s.array() <= 0).any() ||
      (p.max_acceleration_rad_s2.array() <= 0).any() || (p.max_jerk_rad_s3.array() <= 0).any() ||
      (p.max_acceleration_rad_s2.array() > config_.joint_limits.max_acceleration_rad_s2.array()).any() ||
      (p.max_jerk_rad_s3.array() > config_.joint_limits.max_jerk_rad_s3.array()).any() ||
      !std::isfinite(p.validation_tolerance) || p.validation_tolerance <= 0 ||
      p.validation_tolerance > 1e-6 || !std::isfinite(config_.controller.rate_hz) ||
      config_.controller.rate_hz <= 0)
    throw std::invalid_argument("DLS requires enabled model-only Ruckig within current motion limits");
  auto left_dls = std::make_unique<PicoEeFrankaDlsIk7>(config_.iterative_dls, d, config_.joint_limits.margin_rad);
  auto right_dls = std::make_unique<PicoEeFrankaDlsIk7>(*left_dls);
  const auto& urdf=config_.controller.pico_ee_dls_kinematics_urdf_path;
  if (urdf.empty() || sharedRootSha256File(urdf)!=
      "f72dcd970dd9d539c55c5897e63ebd18d5e61b6895ef460fe0d73c8179cec006")
    throw std::invalid_argument("DLS requires its reviewed source Pinocchio URDF; no MuJoCo FK fallback");
  // Retain the configured model's explicit TCP (shared-root palm is 161.5 mm,
  // source default is 95 mm). Never silently substitute one control point.
  auto kinematics=std::make_unique<DlsPinocchioArmKinematics>(urdf,
      std::array<Pose,2>{robot_.tcpRelativeToLink7(ArmSide::kLeft),
                        robot_.tcpRelativeToLink7(ArmSide::kRight)});
  for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
    const auto& limits=robot_.mapping(side).limits;
    const auto pin_limits=kinematics->limits(side);
    if(!pin_limits.lower_position.isApprox(limits.lower_position,1e-12) ||
       !pin_limits.upper_position.isApprox(limits.upper_position,1e-12) ||
       !pin_limits.velocity.isApprox(limits.velocity,1e-12))
      throw std::invalid_argument("DLS Pinocchio/MuJoCo joint limits mismatch");
    for(int probe=0;probe<2;++probe) {
      Vec7 q=robot_.armPosition(side);
      if(probe)for(int j=0;j<7;++j)q[j]=std::clamp(q[j]+(j%2==0?.031:-.031),
          limits.lower_position[j],limits.upper_position[j]);
      const auto pin=kinematics->sampleTcp(side,q),mj=robot_.armKinematicsAt(side,q);
      if(!pin.tcp_pose.position.isApprox(mj.tcp_pose.position,1e-5) ||
         rotationDistance(pin.tcp_pose.rotation,mj.tcp_pose.rotation)>1e-5 ||
         !pin.tcp_jacobian.isApprox(mj.tcp_jacobian,1e-5))
        throw std::invalid_argument("DLS Pinocchio/MuJoCo FK or Jacobian mismatch");
    }
  }
  const auto make = [&](ArmSide side) {
    auto limits = motionLimits(side);
    limits.lower_position.array() += config_.joint_limits.margin_rad;
    limits.upper_position.array() -= config_.joint_limits.margin_rad;
    return std::make_unique<RuckigTrajectoryLimiter7>(p, limits, 1.0/config_.controller.rate_hz);
  };
  auto left = make(ArmSide::kLeft), right = make(ArmSide::kRight);
  if (config_.joint_limits.execution_limits &&
      (!left->canReset(referenceState(ArmSide::kLeft)) ||
       !right->canReset(referenceState(ArmSide::kRight)))) {
    throw std::invalid_argument(
        "initial reference is outside effective Ruckig motion limits (including joint margin)");
  }
  left_franka_dls_ = std::move(left_dls); right_franka_dls_ = std::move(right_dls);
  dls_kinematics_ = std::move(kinematics);
  left_smoother_ = std::move(left); right_smoother_ = std::move(right);
  left_dls_ = {}; right_dls_ = {};
}

ControllerDiagnostics DualArmController::stepDls(
    const DualArmTargets& targets, const DualArmReferences* references, double dt) {
  const auto start = std::chrono::steady_clock::now();
  ControllerDiagnostics out;
  out.left.q_ref = left_state_.q_ref; out.right.q_ref = right_state_.q_ref;
  if (!left_franka_dls_) initializeDls();
  if (!std::isfinite(dt) || dt <= 0) return out;
  const bool live = targetsAreFinite(targets) && !targets.left_stale && !targets.right_stale &&
      (!references || (referenceIsFinite(references->left) && referenceIsFinite(references->right) &&
                       !references->left.stale && !references->right.stale));
  // Copies make a bilateral update transactional. A rejected arm cannot
  // advance the other arm, its IK seed, or either Ruckig history.
  auto ls = std::make_unique<RuckigTrajectoryLimiter7>(*left_smoother_);
  auto rs = std::make_unique<RuckigTrajectoryLimiter7>(*right_smoother_);
  auto lc = left_dls_, rc = right_dls_;
  auto ld = std::make_unique<PicoEeFrankaDlsIk7>(*left_franka_dls_);
  auto rd = std::make_unique<PicoEeFrankaDlsIk7>(*right_franka_dls_);
  // Actual state remains MuJoCo; candidate/reference kinematics use Pinocchio.
  robot_.forward();
  ArmMotionState next_left, next_right;
  const auto prepare = [&](ArmSide side, RuckigTrajectoryLimiter7& smoother,
                            DlsState& cache, ArmControllerDiagnostics& arm,
                            ArmMotionState& next) {
    const auto current = referenceState(side);
    const auto& limits = motionLimits(side);
    if (!current.q.allFinite() || !current.qdot.allFinite() || !current.qddot.allFinite() ||
        !robot_.armPosition(side).allFinite()) return false;
    if (!cache.valid && !smoother.reset(current)) return false;
    auto* dls_solver = side == ArmSide::kLeft ? ld.get() : rd.get();
    if (!cache.valid) dls_solver->reset(current);
    arm.current = dls_kinematics_->sampleTcp(side, current.q).tcp_pose;
    arm.ee_pinocchio_kinematics = true;
    arm.q_actual = robot_.armPosition(side);
    arm.tcp_actual = robot_.tcpPose(side);
    arm.target = side == ArmSide::kLeft ? targets.left : targets.right;
    arm.reference.pose = arm.target; arm.reference.valid = live; arm.reference.stale = !live;
    if (live) {
      arm.pose_error = poseErrorWorld(arm.target, arm.current);
      arm.actual_pose_error = poseErrorWorld(arm.target, arm.tcp_actual);
    }
    Vec7 goal = current.q;
    const auto pipeline_start=std::chrono::steady_clock::now();
    const bool unchanged_episode_target = episode_relative_ && cache.valid &&
        arm.target.position.isApprox(cache.target.position, 1e-12) &&
        arm.target.rotation.isApprox(cache.target.rotation, 1e-12);
    if (live && unchanged_episode_target) {
      // Do not spend another nullspace/posture iteration on identical human input.
      // The existing Ruckig trajectory still settles under all motion constraints.
      goal = dls_solver->goal();
      arm.dls_posture_reference_active = true;
      arm.dls_posture_status = cache.status;
    } else if (live) {
      PicoEeFrankaDlsInput input;
      input.target = arm.target; input.target_valid = true; input.dt = dt; input.limits = limits;
        // Source DLS owns the previous raw IK goal internally; its external
        // seed is the current model reference, not the Ruckig destination.
        input.seed = current.q; input.seed_velocity = current.qdot;
        input.seed_acceleration = current.qddot;
        const Vec7 velocity_limit = config_.pico_ee_franka_dls.max_velocity_rad_s.cwiseMin(limits.velocity);
        input.velocity_bounds.lower = -velocity_limit;
        input.velocity_bounds.upper = velocity_limit;
        input.home_reference = episode_relative_
            ? episode_posture_[side == ArmSide::kLeft ? 0U : 1U]
            : (side == ArmSide::kLeft ? config_.pico_ee_franka_dls.home_left_rad
                                      : config_.pico_ee_franka_dls.home_right_rad);
      input.evaluate = [&, side](const Vec7& q) {
        return config_.pico_ee_franka_dls.max_arm_plane_rate_rad_s > 0.0
            ? dls_kinematics_->sample(side, q)
            : dls_kinematics_->sampleTcp(side, q);
      };
      const auto solved = dls_solver->solve(input);
      arm.ee_ik_wall_time_us=std::chrono::duration<double,std::micro>(
          std::chrono::steady_clock::now()-pipeline_start).count();
      arm.ee_ik_to_ruckig_wall_time_us=arm.ee_ik_wall_time_us;
      arm.dls_posture_reference_active = true;
      arm.dls_posture_status = solved.dls.status;
      arm.dls_posture_iterations = solved.dls.iterations;
      arm.dls_posture_solve_time_us = solved.dls.solve_time_us;
      arm.dls_posture_initial_position_error_m = solved.dls.initial_position_error_m;
      arm.dls_posture_initial_orientation_error_rad = solved.dls.initial_orientation_error_rad;
      arm.dls_posture_final_position_error_m = solved.dls.position_error_m;
      arm.dls_posture_final_orientation_error_rad = solved.dls.orientation_error_rad;
      arm.ik.detail = solved.detail;
      if (!solved.accepted || solved.target_held) return false;
      goal = solved.goal;
      cache.valid = true;
      cache.target = arm.target;
      cache.status = solved.dls.status;
    } else {
      // Stale/invalid input brakes toward the current model reference, never
      // continues toward the last IK goal. It cannot acknowledge mapping data.
      cache.valid = false;
    }
    if (live && simulation_soft_start_pending_ && !smoother.beginSoftStart(simulation_soft_start_limits_)) return false;
    const auto smoother_start=std::chrono::steady_clock::now();
    // A locally limited IK increment can be small even while the actual palm
    // target is far away. Do not mistake that increment for completed takeover.
    const bool near_target = live &&
        (arm.target.position-arm.current.position).norm() < .03 &&
        rotationDistance(arm.target.rotation,arm.current.rotation) < .1;
    const auto trajectory = smoother.update(goal, dt, near_target);
    const auto smoother_end=std::chrono::steady_clock::now();
    arm.ee_ruckig_invoked=true;
    arm.ee_ruckig_wall_time_us=std::chrono::duration<double,std::micro>(smoother_end-smoother_start).count();
    arm.ee_ik_to_ruckig_wall_time_us=std::chrono::duration<double,std::micro>(smoother_end-pipeline_start).count();
    arm.dls_posture_goal = goal;
    arm.dls_posture_ruckig_accepted = trajectory.accepted;
    arm.dls_posture_ruckig_detail = trajectory.detail;
    if (!trajectory.accepted) return false;
    next = trajectory.state;
    arm.dls_posture_reference = next.q;
    arm.dls_posture_velocity_reference = next.qdot;
    arm.ik.qdot = next.qdot;
    arm.ik.status = SolverStatus::kSolved;
    arm.ik.solve_time_us = arm.dls_posture_solve_time_us;
    arm.ik.detail = live ? "franka_dls_ruckig" : "franka_dls_stale_bounded_stop";
    return true;
  };
  const bool left_ok = prepare(ArmSide::kLeft, *ls, lc, out.left, next_left);
  const bool right_ok = prepare(ArmSide::kRight, *rs, rc, out.right, next_right);
  if (left_ok && right_ok) {
    if (live) simulation_soft_start_pending_ = false;
    left_smoother_ = std::move(ls); right_smoother_ = std::move(rs);
    left_dls_ = lc; right_dls_ = rc;
    left_franka_dls_ = std::move(ld); right_franka_dls_ = std::move(rd);
    const auto commit = [&](ArmSide side, const ArmMotionState& next) {
      auto& s = state(side); s.q_ref = next.q; s.qdot_prev = next.qdot;
      s.qddot_prev = next.qddot;
      robot_.setArmState(side, next.q, next.qdot);
    };
    commit(ArmSide::kLeft, next_left); commit(ArmSide::kRight, next_right);
  } else {
    left_dls_.valid = right_dls_.valid = false;
  }
  out.accepted = out.left.accepted = out.right.accepted = live && left_ok && right_ok;
  out.hold_reason = out.left.hold_reason = out.right.hold_reason =
      left_ok && right_ok ? HoldReason::kNone : HoldReason::kSolverFailure;
  out.left.safety = out.right.safety = {out.accepted, out.hold_reason, -1};
  out.left.q_ref = left_state_.q_ref; out.right.q_ref = right_state_.q_ref;
  out.compute_time_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-start).count();
  return out;
}
} // namespace tianji_qp_ik
