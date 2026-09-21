#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/shared_root_options.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
DlsPostureRuckigConfig DualArmController::trajectorySampleLimits(ArmSide side) const {
  const auto& limiter = side == ArmSide::kLeft ? left_ceres_smoother_ : right_ceres_smoother_;
  if (limiter) return limiter->sampledLimits();
  return algorithm_ == IkAlgorithm::kPicoEeFrankaDls
      ? config_.pico_ee_franka_dls.post_smoothing : config_.pico_ee_franka_ceres_lm.post_smoothing;
}
bool DualArmController::beginSimulationSoftStart(SimulationSoftStartLimits limits) {
  if (!limits.valid() || !config_.controller.model_state_only ||
      (algorithm_ != IkAlgorithm::kPicoEeFrankaDls &&
       algorithm_ != IkAlgorithm::kPicoEeFrankaCeresLm)) return false;
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
void DualArmController::initializeCeres(IkAlgorithm algorithm) {
  const auto& c = config_.pico_ee_franka_ceres_lm;
  const auto& d = config_.pico_ee_franka_dls;
  const bool dls = algorithm == IkAlgorithm::kPicoEeFrankaDls;
  const auto& p = dls ? d.post_smoothing : c.post_smoothing;
  const Vec7 home_left = dls ? d.home_left_rad : c.home_left_rad;
  const Vec7 home_right = dls ? d.home_right_rad : c.home_right_rad;
  if (dls && (d.planner_enabled || d.direct_velocity_limit_enabled))
    throw std::invalid_argument("Franka DLS port requires raw IK followed by Ruckig");
  // Model references are independent of hardware authority. Only the explicit
  // viewer executor gate may export them; Python retains motion supervision.
  if (!(dls ? d.enabled : c.enabled) || !config_.controller.model_state_only ||
      config_.control_level != ControlLevel::kVelocity ||
      !p.enabled || p.velocity_scale != 1.0 ||
      config_.upper_arm_outward.minimum_outward_distance_m != 0.0 ||
      !home_left.allFinite() || !home_right.allFinite() ||
      !p.max_velocity_rad_s.allFinite() || !p.max_acceleration_rad_s2.allFinite() ||
      !p.max_jerk_rad_s3.allFinite() || (p.max_velocity_rad_s.array() <= 0).any() ||
      (p.max_acceleration_rad_s2.array() <= 0).any() || (p.max_jerk_rad_s3.array() <= 0).any() ||
      (p.max_acceleration_rad_s2.array() > config_.joint_limits.max_acceleration_rad_s2.array()).any() ||
      (p.max_jerk_rad_s3.array() > config_.joint_limits.max_jerk_rad_s3.array()).any() ||
      !std::isfinite(p.validation_tolerance) || p.validation_tolerance <= 0 ||
      p.validation_tolerance > 1e-6 || !std::isfinite(config_.controller.rate_hz) ||
      config_.controller.rate_hz <= 0)
    throw std::invalid_argument("Ceres requires enabled model-only Ruckig within current motion limits; outward constraint unsupported");
  std::unique_ptr<PicoEeFrankaCeresLmIk7> solver;
  std::unique_ptr<PicoEeFrankaDlsIk7> left_dls, right_dls;
  if (dls) {
    left_dls = std::make_unique<PicoEeFrankaDlsIk7>(config_.iterative_dls, d, config_.joint_limits.margin_rad);
    right_dls = std::make_unique<PicoEeFrankaDlsIk7>(*left_dls);
  } else {
    solver = std::make_unique<PicoEeFrankaCeresLmIk7>(c, config_.joint_limits.margin_rad);
  }
  const auto& urdf=config_.controller.pico_ee_dls_kinematics_urdf_path;
  if (urdf.empty() || sharedRootSha256File(urdf)!=
      "f72dcd970dd9d539c55c5897e63ebd18d5e61b6895ef460fe0d73c8179cec006")
    throw std::invalid_argument("Ceres requires its reviewed source Pinocchio URDF; no MuJoCo FK fallback");
  // Retain the configured model's explicit TCP (shared-root palm is 161.5 mm,
  // source default is 95 mm). Never silently substitute one control point.
  auto kinematics=std::make_unique<CeresPinocchioArmKinematics>(urdf,
      std::array<Pose,2>{robot_.tcpRelativeToLink7(ArmSide::kLeft),
                        robot_.tcpRelativeToLink7(ArmSide::kRight)});
  for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
    const auto& limits=robot_.mapping(side).limits;
    const auto pin_limits=kinematics->limits(side);
    if(!pin_limits.lower_position.isApprox(limits.lower_position,1e-12) ||
       !pin_limits.upper_position.isApprox(limits.upper_position,1e-12) ||
       !pin_limits.velocity.isApprox(limits.velocity,1e-12))
      throw std::invalid_argument("Ceres Pinocchio/MuJoCo joint limits mismatch");
    for(int probe=0;probe<2;++probe) {
      Vec7 q=robot_.armPosition(side);
      if(probe)for(int j=0;j<7;++j)q[j]=std::clamp(q[j]+(j%2==0?.031:-.031),
          limits.lower_position[j],limits.upper_position[j]);
      const auto pin=kinematics->sampleTcp(side,q),mj=robot_.armKinematicsAt(side,q);
      if(!pin.tcp_pose.position.isApprox(mj.tcp_pose.position,1e-5) ||
         rotationDistance(pin.tcp_pose.rotation,mj.tcp_pose.rotation)>1e-5 ||
         !pin.tcp_jacobian.isApprox(mj.tcp_jacobian,1e-5))
        throw std::invalid_argument("Ceres Pinocchio/MuJoCo FK or Jacobian mismatch");
    }
  }
  const auto make = [&](ArmSide side) {
    auto limits = robot_.mapping(side).limits;
    limits.lower_position.array() += config_.joint_limits.margin_rad;
    limits.upper_position.array() -= config_.joint_limits.margin_rad;
    return std::make_unique<CeresTrajectoryLimiter7>(p, limits, 1.0/config_.controller.rate_hz);
  };
  auto left = make(ArmSide::kLeft), right = make(ArmSide::kRight);
  ceres_ = std::move(solver);
  left_franka_dls_ = std::move(left_dls); right_franka_dls_ = std::move(right_dls);
  ceres_kinematics_ = std::move(kinematics);
  left_ceres_smoother_ = std::move(left); right_ceres_smoother_ = std::move(right);
  left_ceres_ = {}; right_ceres_ = {};
}

ControllerDiagnostics DualArmController::stepCeres(
    const DualArmTargets& targets, const DualArmReferences* references, double dt) {
  const auto start = std::chrono::steady_clock::now();
  ControllerDiagnostics out;
  out.left.q_ref = left_state_.q_ref; out.right.q_ref = right_state_.q_ref;
  const bool dls = algorithm_ == IkAlgorithm::kPicoEeFrankaDls;
  if (dls ? !left_franka_dls_ : !ceres_) initializeCeres(algorithm_);
  if (!std::isfinite(dt) || dt <= 0) return out;
  const bool live = targetsAreFinite(targets) && !targets.left_stale && !targets.right_stale &&
      (!references || (referenceIsFinite(references->left) && referenceIsFinite(references->right) &&
                       !references->left.stale && !references->right.stale));
  // Copies make a bilateral update transactional. A rejected arm cannot
  // advance the other arm, its IK seed, or either Ruckig history.
  auto ls = std::make_unique<CeresTrajectoryLimiter7>(*left_ceres_smoother_);
  auto rs = std::make_unique<CeresTrajectoryLimiter7>(*right_ceres_smoother_);
  auto lc = left_ceres_, rc = right_ceres_;
  auto ld = dls ? std::make_unique<PicoEeFrankaDlsIk7>(*left_franka_dls_) : nullptr;
  auto rd = dls ? std::make_unique<PicoEeFrankaDlsIk7>(*right_franka_dls_) : nullptr;
  // Actual state remains MuJoCo; candidate/reference kinematics use Pinocchio.
  robot_.forward();
  ArmMotionState next_left, next_right;
  const auto prepare = [&](ArmSide side, CeresTrajectoryLimiter7& smoother,
                            CeresState& cache, ArmControllerDiagnostics& arm,
                            ArmMotionState& next) {
    const auto current = referenceState(side);
    const auto limits = robot_.mapping(side).limits;
    if (!current.q.allFinite() || !current.qdot.allFinite() || !current.qddot.allFinite() ||
        !robot_.armPosition(side).allFinite()) return false;
    if (!cache.valid && !smoother.reset(current)) return false;
    auto* dls_solver = side == ArmSide::kLeft ? ld.get() : rd.get();
    if (dls_solver && !cache.valid) dls_solver->reset(current);
    arm.current = ceres_kinematics_->sampleTcp(side, current.q).tcp_pose;
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
    if (live) {
      PicoEeFrankaDlsInput input;
      input.target = arm.target; input.target_valid = true; input.dt = dt; input.limits = limits;
      input.seed = cache.valid ? cache.goal : current.q;
      input.seed_velocity = cache.valid ? cache.velocity : current.qdot;
      input.seed_acceleration = cache.valid ? cache.acceleration : current.qddot;
      input.home_reference = side == ArmSide::kLeft ? config_.pico_ee_franka_ceres_lm.home_left_rad
                                                   : config_.pico_ee_franka_ceres_lm.home_right_rad;
      if (dls) {
        // Source DLS owns the previous raw IK goal internally; its external
        // seed is the current model reference, not the Ruckig destination.
        input.seed = current.q; input.seed_velocity = current.qdot;
        input.seed_acceleration = current.qddot;
        const Vec7 velocity_limit = config_.pico_ee_franka_dls.max_velocity_rad_s.cwiseMin(limits.velocity);
        input.velocity_bounds.lower = -velocity_limit;
        input.velocity_bounds.upper = velocity_limit;
        input.home_reference = side == ArmSide::kLeft ? config_.pico_ee_franka_dls.home_left_rad
                                                     : config_.pico_ee_franka_dls.home_right_rad;
      }
      input.evaluate = [&, side](const Vec7& q) {
        return dls && config_.pico_ee_franka_dls.max_arm_plane_rate_rad_s > 0.0
            ? ceres_kinematics_->sample(side, q)
            : ceres_kinematics_->sampleTcp(side, q);
      };
      const auto solved = dls_solver ? dls_solver->solve(input) : ceres_->solve(input);
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
      const Vec7 velocity = (goal-input.seed)/dt;
      cache.acceleration = (velocity-input.seed_velocity)/dt;
      cache.velocity = velocity; cache.goal = goal; cache.valid = true;
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
    arm.ik.detail = dls ? (live ? "franka_dls_ruckig" : "franka_dls_stale_bounded_stop")
                        : (live ? "ceres_lm_ruckig" : "ceres_stale_bounded_stop");
    return true;
  };
  const bool left_ok = prepare(ArmSide::kLeft, *ls, lc, out.left, next_left);
  const bool right_ok = prepare(ArmSide::kRight, *rs, rc, out.right, next_right);
  if (left_ok && right_ok) {
    if (live) simulation_soft_start_pending_ = false;
    left_ceres_smoother_ = std::move(ls); right_ceres_smoother_ = std::move(rs);
    left_ceres_ = lc; right_ceres_ = rc;
    if (dls) { left_franka_dls_ = std::move(ld); right_franka_dls_ = std::move(rd); }
    const auto commit = [&](ArmSide side, const ArmMotionState& next) {
      auto& s = state(side); s.q_ref = next.q; s.qdot_prev = next.qdot;
      s.qddot_prev = next.qddot; s.slack_prev.setZero();
      robot_.setArmState(side, next.q, next.qdot);
    };
    commit(ArmSide::kLeft, next_left); commit(ArmSide::kRight, next_right);
  } else {
    left_ceres_.valid = right_ceres_.valid = false;
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
