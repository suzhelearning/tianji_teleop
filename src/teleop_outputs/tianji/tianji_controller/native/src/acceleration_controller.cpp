#include "tianji_qp_ik/acceleration_controller.hpp"

#include "tianji_qp_ik/acceleration_bounds.hpp"
#include "tianji_qp_ik/acceleration_qp.hpp"
#include "tianji_qp_ik/cartesian_acceleration_servo.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/upper_arm_outward.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tianji_qp_ik {
namespace {

double trackingScale(double error, double warning, double stop) {
  if (error <= warning) {
    return 1.0;
  }
  if (error >= stop) {
    return 0.0;
  }
  return (stop - error) / (stop - warning);
}

double elapsedMicroseconds(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::micro>(
             std::chrono::steady_clock::now() - start)
      .count();
}

SafetyDecision validateActualState(const Vec7& position, const Vec7& velocity,
                                   const ArmLimits& limits,
                                   double tolerance) {
  if (!position.allFinite() || !velocity.allFinite()) {
    return {false, HoldReason::kNonFiniteProblem, -1};
  }
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (position[joint] < limits.lower_position[joint] - tolerance ||
        position[joint] > limits.upper_position[joint] + tolerance ||
        std::abs(velocity[joint]) > limits.velocity[joint] + tolerance) {
      return {false, HoldReason::kBoundViolation, joint};
    }
  }
  return {true, HoldReason::kNone, -1};
}

ArmAngleGeometryInput armAngleGeometry(const ArmKinematicSample& sample) {
  return {sample.shoulder_position, sample.elbow_position,
          sample.wrist_position, sample.shoulder_position_jacobian,
          sample.elbow_position_jacobian, sample.wrist_position_jacobian};
}

}  // namespace

DualArmAccelerationController::DualArmAccelerationController(
    MujocoRobot& robot, QpIkConfig config)
    : DualArmAccelerationController(
          robot, config,
          std::make_unique<AccelerationQpoasesSolver>(config.qpoases),
          std::make_unique<AccelerationQpoasesSolver>(config.qpoases)) {}

DualArmAccelerationController::DualArmAccelerationController(
    MujocoRobot& robot, QpIkConfig config,
    std::unique_ptr<IAccelerationQpSolver> left_solver,
    std::unique_ptr<IAccelerationQpSolver> right_solver)
    : robot_(robot),
      config_(std::move(config)),
      left_dls_posture_(config_.iterative_dls,
                        config_.joint_acceleration_limits.margin_rad),
      right_dls_posture_(config_.iterative_dls,
                         config_.joint_acceleration_limits.margin_rad),
      left_dls_posture_ruckig_(
          config_.dls_posture_ruckig,
          robot_.mapping(ArmSide::kLeft).limits,
          1.0 / config_.controller.rate_hz),
      right_dls_posture_ruckig_(
          config_.dls_posture_ruckig,
          robot_.mapping(ArmSide::kRight).limits,
          1.0 / config_.controller.rate_hz),
      left_solver_(std::move(left_solver)),
      right_solver_(std::move(right_solver)),
      left_arm_angle_(ArmSide::kLeft, config_.arm_angle.minimum_radius_m,
                      config_.arm_angle.full_weight_radius_m,
                      config_.arm_angle.reference_rate_limit_rad_s,
                      config_.arm_angle.reference_projection_hold_enter,
                      config_.arm_angle.reference_projection_hold_exit,
                      config_.arm_angle.error_branch_hysteresis_rad,
                      config_.arm_angle.branch_lock_radius_m,
                      config_.arm_angle.reference_governor_enabled,
                      config_.arm_angle.reference_governor_enter_error_rad,
                      config_.arm_angle.reference_governor_exit_error_rad,
                      config_.arm_angle.reference_governor_tracking_error_rad),
      right_arm_angle_(ArmSide::kRight, config_.arm_angle.minimum_radius_m,
                       config_.arm_angle.full_weight_radius_m,
                       config_.arm_angle.reference_rate_limit_rad_s,
                       config_.arm_angle.reference_projection_hold_enter,
                       config_.arm_angle.reference_projection_hold_exit,
                       config_.arm_angle.error_branch_hysteresis_rad,
                       config_.arm_angle.branch_lock_radius_m,
                       config_.arm_angle.reference_governor_enabled,
                       config_.arm_angle.reference_governor_enter_error_rad,
                       config_.arm_angle.reference_governor_exit_error_rad,
                       config_.arm_angle.reference_governor_tracking_error_rad) {
  if (left_solver_ == nullptr || right_solver_ == nullptr) {
    throw std::invalid_argument(
        "DualArmAccelerationController requires two solvers");
  }
  resetReferences();
}

AccelerationControllerDiagnostics DualArmAccelerationController::step(
    const DualArmReferences& references, double dt) {
  return step(references, defaultArmDirectionReferences(), dt);
}

AccelerationControllerDiagnostics DualArmAccelerationController::step(
    const DualArmReferences& references,
    const DualArmDirectionReferences& arm_directions, double dt) {
  const auto start = std::chrono::steady_clock::now();
  AccelerationControllerDiagnostics diagnostics;
  if (!std::isfinite(dt) || dt <= 0.0) {
    diagnostics.compute_time_us = elapsedMicroseconds(start);
    return diagnostics;
  }
  robot_.forward();

  const auto process = [&](ArmSide side, const CartesianReference& reference,
                           const ArmDirectionReference& arm_direction,
                           IAccelerationQpSolver& solver,
                           ArmAccelerationDiagnostics& arm_diagnostics) {
    ArmState& arm_state = state(side);
    arm_diagnostics.reference = reference;
    arm_diagnostics.q_ref = arm_state.q_ref;
    arm_diagnostics.qdot_ref = arm_state.qdot_ref;
    const Vec7 q_actual = robot_.armPosition(side);
    const Vec7 qdot_actual = robot_.armVelocity(side);
    arm_diagnostics.q_actual = q_actual;
    arm_diagnostics.qdot_actual = qdot_actual;
    arm_diagnostics.tcp_actual = robot_.tcpPose(side);
    const ArmKinematicSample model =
        robot_.armKinematicsAt(side, arm_state.q_ref);
    arm_diagnostics.current = model.tcp_pose;
    ArmAngleTaskBuilder& arm_angle_builder =
        side == ArmSide::kLeft ? left_arm_angle_ : right_arm_angle_;
    arm_diagnostics.arm_angle =
        usesContinuityArmDirection(arm_angle_reference_mode_)
        ? arm_angle_builder.computeContinuity(armAngleGeometry(model), dt)
        : arm_angle_builder.compute(armAngleGeometry(model), arm_direction,
                                    dt);
    if (!referenceIsFinite(reference)) {
      arm_diagnostics.hold_reason = HoldReason::kNonFiniteProblem;
      freezeArm(side, solver);
      return;
    }

    arm_diagnostics.pose_error =
        poseErrorWorld(reference.pose, arm_diagnostics.current);
    arm_diagnostics.actual_pose_error =
        poseErrorWorld(reference.pose, arm_diagnostics.tcp_actual);
    arm_diagnostics.position_reference_error =
        (arm_state.q_ref - q_actual).cwiseAbs().maxCoeff();
    arm_diagnostics.velocity_reference_error =
        (arm_state.qdot_ref - qdot_actual).cwiseAbs().maxCoeff();
    const ArmLimits& limits = robot_.mapping(side).limits;
    const SafetyDecision actual_safety = validateActualState(
        q_actual, qdot_actual, limits, config_.safety.bound_tolerance);
    if (!actual_safety.accepted) {
      arm_diagnostics.hold_reason = actual_safety.reason;
      arm_diagnostics.hold_joint_index = actual_safety.joint_index;
      freezeArm(side, solver);
      return;
    }
    const double position_scale = trackingScale(
        arm_diagnostics.position_reference_error,
        config_.safety.reference_tracking_warn_rad,
        config_.safety.reference_tracking_stop_rad);
    const double velocity_scale = trackingScale(
        arm_diagnostics.velocity_reference_error,
        config_.safety.velocity_reference_tracking_warn_rad_s,
        config_.safety.velocity_reference_tracking_stop_rad_s);
    arm_diagnostics.reference_scale =
        std::min(position_scale, velocity_scale);
    arm_diagnostics.reference_frozen =
        arm_diagnostics.reference_scale <= 0.0;
    if (arm_diagnostics.reference_frozen) {
      arm_diagnostics.hold_reason = HoldReason::kReferenceTrackingError;
      freezeArm(side, solver);
      return;
    }

    ArmAccelerationInput input;
    input.q_model = arm_state.q_ref;
    input.qdot_model = arm_state.qdot_ref;
    input.qddot_previous = arm_state.qddot_previous;
    input.jacobian = model.tcp_jacobian;
    arm_diagnostics.arm_angle_current_rate =
        arm_diagnostics.arm_angle.jacobian.dot(arm_state.qdot_ref);
    const bool continuity_mode =
        usesContinuityArmDirection(arm_angle_reference_mode_);
    const double arm_angle_jdot_qdot = continuity_mode
        ? estimateArmAngleJacobianDotTimesVelocity(
              arm_diagnostics.arm_angle.jacobian,
              arm_state.previous_arm_angle_jacobian, arm_state.qdot_ref, dt,
              arm_state.previous_arm_angle_jacobian_valid)
        : 0.0;
    if (continuity_mode && arm_diagnostics.arm_angle.active) {
      arm_state.previous_arm_angle_jacobian =
          arm_diagnostics.arm_angle.jacobian;
      arm_state.previous_arm_angle_jacobian_valid = true;
    } else {
      arm_state.previous_arm_angle_jacobian_valid = false;
    }
    double arm_branch_lock_jdot_qdot = 0.0;
    if (arm_state.previous_arm_branch_lock_jacobian_valid &&
        arm_diagnostics.arm_angle.branch_lock_active) {
      arm_branch_lock_jdot_qdot =
          ((arm_diagnostics.arm_angle.branch_lock_jacobian -
            arm_state.previous_arm_branch_lock_jacobian) /
           dt)
              .dot(arm_state.qdot_ref);
      if (!std::isfinite(arm_branch_lock_jdot_qdot)) {
        arm_branch_lock_jdot_qdot = 0.0;
      }
    }
    if (arm_diagnostics.arm_angle.branch_lock_active) {
      arm_state.previous_arm_branch_lock_jacobian =
          arm_diagnostics.arm_angle.branch_lock_jacobian;
      arm_state.previous_arm_branch_lock_jacobian_valid = true;
    } else {
      arm_state.previous_arm_branch_lock_jacobian_valid = false;
    }
    input.arm_angle_task.jacobian = arm_diagnostics.arm_angle.jacobian;
    input.arm_angle_task.nullspace_only =
        config_.arm_angle.nullspace_only &&
        usesOutwardArmBarrier(arm_angle_reference_mode_);
    const bool weak_arm_angle_mode =
        usesOutwardArmBarrier(arm_angle_reference_mode_);
    const double arm_angle_kp = weak_arm_angle_mode
        ? config_.arm_angle.continuity_kp_acceleration
        : config_.arm_angle.kp_acceleration;
    const double arm_angle_kd = weak_arm_angle_mode
        ? config_.arm_angle.continuity_kd_acceleration
        : config_.arm_angle.kd_acceleration;
    const double arm_angle_acceleration_limit = weak_arm_angle_mode
        ? config_.arm_angle.continuity_max_acceleration_rad_s2
        : config_.arm_angle.max_acceleration_rad_s2;
    const double requested_arm_angle_acceleration = std::clamp(
        arm_angle_kp * arm_diagnostics.arm_angle.control_error_rad -
            arm_angle_kd * arm_diagnostics.arm_angle_current_rate,
        -arm_angle_acceleration_limit, arm_angle_acceleration_limit);
    input.arm_angle_task.target =
        requested_arm_angle_acceleration - arm_angle_jdot_qdot;
    input.arm_angle_task.activation = arm_diagnostics.arm_angle.activation;
    input.arm_angle_task.weight_scale = weak_arm_angle_mode
        ? config_.arm_angle.continuity_weight_scale
        : 1.0;
    input.arm_angle_task.active =
        config_.arm_angle.enabled && arm_diagnostics.arm_angle.active &&
        input.arm_angle_task.activation > 0.0;
    const bool dls_posture_mode =
        config_.iterative_dls.posture_reference_enabled &&
        arm_angle_reference_mode_ == ArmAngleReferenceMode::kPicoOutward;
    if (dls_posture_mode) {
      if (!arm_state.dls_posture_reference_initialized) {
        arm_state.dls_posture_reference = arm_state.q_ref;
        arm_state.dls_posture_reference_initialized = true;
      }
      if (!reference.stale) {
        PoseDlsInput dls_input;
        dls_input.target = reference.pose;
        dls_input.seed = arm_state.dls_posture_reference;
        dls_input.limits = limits;
        dls_input.secondary_task.active =
            config_.arm_angle.enabled && arm_diagnostics.arm_angle.active;
        dls_input.secondary_task.jacobian =
            arm_diagnostics.arm_angle.jacobian;
        dls_input.secondary_task.target =
            arm_diagnostics.arm_angle.control_error_rad;
        dls_input.secondary_task.activation =
            arm_diagnostics.arm_angle.activation;
        dls_input.evaluate = [this, side](const Vec7& q) {
          return robot_.armKinematicsAt(side, q);
        };
        IterativePoseDlsIk7& dls = side == ArmSide::kLeft
            ? left_dls_posture_
            : right_dls_posture_;
        const PoseDlsResult dls_result = dls.solve(dls_input);
        arm_diagnostics.dls_posture_status = dls_result.status;
        arm_diagnostics.dls_posture_iterations = dls_result.iterations;
        arm_diagnostics.dls_posture_solve_time_us =
            dls_result.solve_time_us;
        if (dls_result.status != PoseDlsStatus::kRejected &&
            dls_result.q.allFinite()) {
          const Vec7 delta =
              (dls_result.q - arm_state.dls_posture_reference)
                  .cwiseMax(-config_.iterative_dls.maximum_goal_step_rad)
                  .cwiseMin(config_.iterative_dls.maximum_goal_step_rad);
          arm_state.dls_posture_reference += delta;
        }
      }
      JointTrajectoryLimiter7& posture_ruckig = side == ArmSide::kLeft
          ? left_dls_posture_ruckig_
          : right_dls_posture_ruckig_;
      const JointTrajectoryResult posture_result =
          posture_ruckig.update(arm_state.dls_posture_reference, dt);
      const ArmMotionState& posture_state = posture_result.accepted
          ? posture_result.state
          : posture_ruckig.state();
      input.posture_reference_active = true;
      input.posture_reference = posture_state.q;
      input.posture_velocity_reference = posture_state.qdot;
      input.posture_acceleration_reference = posture_state.qddot;
      input.arm_angle_task.active = false;
      arm_diagnostics.dls_posture_reference_active = true;
      arm_diagnostics.dls_posture_reference = posture_state.q;
      arm_diagnostics.dls_posture_velocity_reference = posture_state.qdot;
      arm_diagnostics.dls_posture_acceleration_reference =
          posture_state.qddot;
      arm_diagnostics.dls_posture_ruckig_accepted = posture_result.accepted;
      arm_diagnostics.dls_posture_ruckig_detail = posture_result.detail;
    }
    arm_diagnostics.arm_angle_task_active = input.arm_angle_task.active;
    arm_diagnostics.arm_angle_requested_acceleration =
        input.arm_angle_task.active ? requested_arm_angle_acceleration : 0.0;
    input.jdot_qdot = robot_.tcpJacobianDotTimesVelocityWorld(
        side, arm_state.q_ref, arm_state.qdot_ref);
    arm_diagnostics.model_twist = input.jacobian * arm_state.qdot_ref;
    input.desired_acceleration = cartesianAccelerationCommand(
        config_.cartesian_acceleration, reference, arm_diagnostics.current,
        arm_diagnostics.model_twist) *
        arm_diagnostics.reference_scale;
    input.limits = limits;
    input.bounds = computeJointAccelerationBounds(
        arm_state.q_ref, arm_state.qdot_ref, arm_state.qddot_previous,
        input.limits,
        config_.joint_acceleration_limits, dt);
    arm_diagnostics.upper_arm_outward.state = computeUpperArmOutwardState(
        side, model.shoulder_position, model.elbow_position,
        model.shoulder_position_jacobian, model.elbow_position_jacobian,
        config_.upper_arm_outward.minimum_outward_distance_m);
    double outward_jdot_qdot = 0.0;
    if (arm_state.previous_outward_jacobian_valid &&
        arm_diagnostics.upper_arm_outward.state.valid) {
      outward_jdot_qdot =
          ((arm_diagnostics.upper_arm_outward.state.jacobian -
            arm_state.previous_outward_jacobian) /
           dt)
              .dot(arm_state.qdot_ref);
    }
    if (arm_diagnostics.upper_arm_outward.state.valid) {
      arm_state.previous_outward_jacobian =
          arm_diagnostics.upper_arm_outward.state.jacobian;
      arm_state.previous_outward_jacobian_valid = true;
    } else {
      arm_state.previous_outward_jacobian_valid = false;
    }
    if (usesOutwardArmBarrier(arm_angle_reference_mode_)) {
      input.linear_constraint = makeAccelerationOutwardConstraint(
          arm_diagnostics.upper_arm_outward.state, arm_state.qdot_ref,
          outward_jdot_qdot, input.bounds.lower, input.bounds.upper,
          config_.upper_arm_outward, dt);
    } else if (config_.arm_angle.branch_lock_enabled &&
               arm_diagnostics.arm_angle.branch_lock_active) {
      input.linear_constraint =
          makeArmAngleBranchLockAccelerationConstraint(
              arm_diagnostics.arm_angle, arm_state.qdot_ref,
              arm_branch_lock_jdot_qdot, input.bounds.lower, input.bounds.upper,
              config_.arm_angle.branch_lock_kp_acceleration,
              config_.arm_angle.branch_lock_kd_acceleration);
    }
    if (usesOutwardArmBarrier(arm_angle_reference_mode_)) {
      arm_diagnostics.upper_arm_outward.constraint_active =
          input.linear_constraint.active;
      arm_diagnostics.upper_arm_outward.requested_lower =
          input.linear_constraint.requested_lower;
      arm_diagnostics.upper_arm_outward.effective_lower =
          input.linear_constraint.lower;
      arm_diagnostics.upper_arm_outward.feasibility_clipped =
          input.linear_constraint.feasibility_clipped;
    }
    if (arm_diagnostics.arm_angle.branch_lock_active &&
        !usesOutwardArmBarrier(arm_angle_reference_mode_)) {
      arm_diagnostics.arm_angle.branch_lock_constraint_active =
          input.linear_constraint.active;
      arm_diagnostics.arm_angle.branch_lock_requested_lower =
          input.linear_constraint.requested_lower;
      arm_diagnostics.arm_angle.branch_lock_effective_lower =
          input.linear_constraint.lower;
      arm_diagnostics.arm_angle.branch_lock_feasibility_clipped =
              input.linear_constraint.feasibility_clipped;
    }
    if (usesOutwardArmBarrier(arm_angle_reference_mode_) &&
        input.linear_constraint.viability_clipped) {
      // The one-cycle outward viability bound cannot be satisfied inside the
      // joint acceleration box. Advancing the reference would knowingly
      // cross the shoulder plane, so hold this arm and retry from zero
      // reference velocity on the next cycle.
      arm_diagnostics.hold_reason = HoldReason::kInfeasibleBounds;
      freezeArm(side, solver);
      return;
    }
    arm_diagnostics.bounds = input.bounds;
    input.dt = dt;
    arm_diagnostics.desired_acceleration = input.desired_acceleration;
    arm_diagnostics.jdot_qdot = input.jdot_qdot;

    const AccelerationQpProblem problem =
        AccelerationQpBuilder(config_.acceleration_qp).build(input);
    SafetyDecision decision =
        validateAccelerationProblem(problem, config_.safety);
    if (!decision.accepted) {
      arm_diagnostics.hold_reason = decision.reason;
      arm_diagnostics.hold_joint_index = decision.joint_index;
      freezeArm(side, solver);
      return;
    }
    AccelerationQpSolution solution;
    int total_iterations = 0;
    double total_update_time_us = 0.0;
    double total_solve_time_us = 0.0;
    bool cold_retry_solved = false;
    for (int attempt = 0; attempt < 2; ++attempt) {
      if (!arm_state.solver_initialized) {
        arm_state.solver_initialized = solver.initialize(problem);
        if (!arm_state.solver_initialized) {
          solution.status = SolverStatus::kNumericalError;
          solution.detail = attempt == 0
                                ? "acceleration QP initialization failed"
                                : "acceleration QP cold retry initialization failed";
          decision = {false, HoldReason::kSolverFailure, -1};
        }
      }
      if (arm_state.solver_initialized) {
        solution = solver.solve(problem);
        total_iterations += solution.iterations;
        total_update_time_us += solution.update_time_us;
        total_solve_time_us += solution.solve_time_us;
        decision = validateAccelerationSolution(
            problem, solution, config_.acceleration_qp, config_.safety);
        if (decision.accepted) {
          cold_retry_solved = attempt > 0;
          break;
        }
      }
      if (attempt == 0) {
        solver.reset();
        arm_state.solver_initialized = false;
      }
    }
    solution.iterations = total_iterations;
    solution.update_time_us = total_update_time_us;
    solution.solve_time_us = total_solve_time_us;
    if (cold_retry_solved) {
      solution.detail = "acceleration QP cold retry solved";
    }
    arm_diagnostics.qp.status = solution.status;
    arm_diagnostics.qp.iterations = solution.iterations;
    arm_diagnostics.qp.solve_time_us = solution.solve_time_us;
    arm_diagnostics.qp.detail = solution.detail;
    if (decision.accepted) {
      arm_diagnostics.qp.qddot = solution.x.head<kArmDof>();
      arm_diagnostics.qp.slack =
          solution.x.segment<6>(kSlackStartIndex);
      if (config_.acceleration_qp.task_scaling_enabled) {
        arm_diagnostics.qp.task_scale_position =
            solution.x[kBetaPositionIndex];
        arm_diagnostics.qp.task_scale_orientation =
            solution.x[kBetaOrientationIndex];
      }
    } else {
      arm_diagnostics.hold_reason = decision.reason;
      arm_diagnostics.hold_joint_index = decision.joint_index;
      const Vec7 braking = -arm_state.qdot_ref / dt;
      arm_diagnostics.qp.qddot =
          braking.cwiseMax(input.bounds.lower).cwiseMin(input.bounds.upper);
      if (!arm_diagnostics.qp.qddot.allFinite()) {
        freezeArm(side, solver);
        return;
      }
      arm_diagnostics.fallback_applied = true;
      solver.reset();
      arm_state.solver_initialized = false;
    }
    if (input.arm_angle_task.active) {
      arm_diagnostics.arm_angle_achieved_acceleration =
          input.arm_angle_task.jacobian.dot(arm_diagnostics.qp.qddot) +
          arm_angle_jdot_qdot;
      arm_diagnostics.arm_angle_acceleration_residual =
          requested_arm_angle_acceleration -
          arm_diagnostics.arm_angle_achieved_acceleration;
    }
    if (input.linear_constraint.active) {
      arm_diagnostics.upper_arm_outward.achieved =
          input.linear_constraint.jacobian.dot(arm_diagnostics.qp.qddot);
      arm_diagnostics.upper_arm_outward.residual =
          arm_diagnostics.upper_arm_outward.achieved -
          input.linear_constraint.lower;
    }
    if (!decision.accepted) {
      arm_diagnostics.qp.slack =
          problem.equality - problem.A.leftCols<kArmDof>() *
              arm_diagnostics.qp.qddot;
    }
    arm_diagnostics.qp.equality_residual = decision.accepted
        ? (problem.A * solution.x - problem.equality).norm()
        : 0.0;
    arm_diagnostics.qp.qddot_max_ratio =
        arm_diagnostics.qp.qddot.cwiseAbs()
            .cwiseQuotient(
                config_.joint_acceleration_limits.max_acceleration_rad_s2)
            .maxCoeff();
    for (int joint = 0; joint < kArmDof; ++joint) {
      if (std::abs(arm_diagnostics.qp.qddot[joint] -
                   input.bounds.lower[joint]) <=
              config_.safety.bound_tolerance ||
          std::abs(arm_diagnostics.qp.qddot[joint] -
                   input.bounds.upper[joint]) <=
              config_.safety.bound_tolerance) {
        ++arm_diagnostics.qp.active_bound_count;
      }
    }

    const Vec7 qdot_candidate =
        arm_state.qdot_ref + arm_diagnostics.qp.qddot * dt;
    const Vec7 q_candidate =
        arm_state.q_ref + arm_state.qdot_ref * dt +
        0.5 * arm_diagnostics.qp.qddot * dt * dt;
    const Vec7 safe_lower =
        input.limits.lower_position.array() +
        config_.joint_acceleration_limits.margin_rad;
    const Vec7 safe_upper =
        input.limits.upper_position.array() -
        config_.joint_acceleration_limits.margin_rad;
    const Vec7 velocity_limit =
        config_.joint_acceleration_limits.velocity_scale *
        input.limits.velocity;
    if (!q_candidate.allFinite() || !qdot_candidate.allFinite() ||
        (q_candidate.array() <
         safe_lower.array() - config_.safety.bound_tolerance)
            .any() ||
        (q_candidate.array() >
         safe_upper.array() + config_.safety.bound_tolerance)
            .any() ||
        (qdot_candidate.cwiseAbs().array() >
         velocity_limit.array() + config_.safety.bound_tolerance)
            .any()) {
      arm_diagnostics.hold_reason = HoldReason::kBoundViolation;
      arm_diagnostics.fallback_applied = false;
      freezeArm(side, solver);
      return;
    }

    arm_state.q_ref = q_candidate;
    arm_state.qdot_ref = qdot_candidate;
    arm_state.qddot_previous = arm_diagnostics.qp.qddot;
    robot_.setArmState(side, q_candidate, qdot_candidate);
    arm_diagnostics.q_ref = q_candidate;
    arm_diagnostics.qdot_ref = qdot_candidate;
    if (!arm_diagnostics.fallback_applied) {
      arm_diagnostics.accepted = true;
      arm_diagnostics.hold_reason = HoldReason::kNone;
    }
  };

  process(ArmSide::kLeft, references.left, arm_directions.left, *left_solver_,
          diagnostics.left);
  process(ArmSide::kRight, references.right, arm_directions.right,
          *right_solver_, diagnostics.right);
  diagnostics.accepted = diagnostics.left.accepted && diagnostics.right.accepted;
  diagnostics.hold_reason = !diagnostics.left.accepted
                                ? diagnostics.left.hold_reason
                                : (!diagnostics.right.accepted
                                       ? diagnostics.right.hold_reason
                                       : HoldReason::kNone);
  diagnostics.compute_time_us = elapsedMicroseconds(start);
  return diagnostics;
}

void DualArmAccelerationController::resetReferences() {
  const double reset_dt = 1.0 / config_.controller.rate_hz;
  left_state_.q_ref = robot_.armPosition(ArmSide::kLeft);
  left_state_.qdot_ref = robot_.armVelocity(ArmSide::kLeft);
  left_state_.qddot_previous = seedPreviousAccelerationForFeasibility(
      left_state_.q_ref, left_state_.qdot_ref,
      robot_.mapping(ArmSide::kLeft).limits,
      config_.joint_acceleration_limits, reset_dt);
  left_state_.solver_initialized = false;
  left_state_.previous_outward_jacobian_valid = false;
  left_state_.previous_arm_angle_jacobian_valid = false;
  left_state_.previous_arm_branch_lock_jacobian_valid = false;
  left_state_.dls_posture_reference = left_state_.q_ref;
  left_state_.dls_posture_reference_initialized = true;
  left_dls_posture_ruckig_.reset(
      {left_state_.q_ref, Vec7::Zero(), Vec7::Zero()});
  right_state_.q_ref = robot_.armPosition(ArmSide::kRight);
  right_state_.qdot_ref = robot_.armVelocity(ArmSide::kRight);
  right_state_.qddot_previous = seedPreviousAccelerationForFeasibility(
      right_state_.q_ref, right_state_.qdot_ref,
      robot_.mapping(ArmSide::kRight).limits,
      config_.joint_acceleration_limits, reset_dt);
  right_state_.solver_initialized = false;
  right_state_.previous_outward_jacobian_valid = false;
  right_state_.previous_arm_angle_jacobian_valid = false;
  right_state_.previous_arm_branch_lock_jacobian_valid = false;
  right_state_.dls_posture_reference = right_state_.q_ref;
  right_state_.dls_posture_reference_initialized = true;
  right_dls_posture_ruckig_.reset(
      {right_state_.q_ref, Vec7::Zero(), Vec7::Zero()});
  left_solver_->reset();
  right_solver_->reset();
  left_arm_angle_.reset();
  right_arm_angle_.reset();
}

void DualArmAccelerationController::setArmAngleReferenceMode(
    ArmAngleReferenceMode mode) noexcept {
  if (mode == arm_angle_reference_mode_) {
    return;
  }
  arm_angle_reference_mode_ = mode;
  left_arm_angle_.reset();
  right_arm_angle_.reset();
  left_state_.previous_arm_angle_jacobian_valid = false;
  right_state_.previous_arm_angle_jacobian_valid = false;
  left_state_.previous_arm_branch_lock_jacobian_valid = false;
  right_state_.previous_arm_branch_lock_jacobian_valid = false;
  left_state_.dls_posture_reference = left_state_.q_ref;
  right_state_.dls_posture_reference = right_state_.q_ref;
  left_dls_posture_ruckig_.reset(
      {left_state_.q_ref, Vec7::Zero(), Vec7::Zero()});
  right_dls_posture_ruckig_.reset(
      {right_state_.q_ref, Vec7::Zero(), Vec7::Zero()});
}

bool DualArmAccelerationController::synchronizeReferencesToActual() {
  const Vec7 left_q = robot_.armPosition(ArmSide::kLeft);
  const Vec7 left_qdot = robot_.armVelocity(ArmSide::kLeft);
  const Vec7 right_q = robot_.armPosition(ArmSide::kRight);
  const Vec7 right_qdot = robot_.armVelocity(ArmSide::kRight);
  const SafetyDecision left_safety = validateActualState(
      left_q, left_qdot, robot_.mapping(ArmSide::kLeft).limits,
      config_.safety.bound_tolerance);
  const SafetyDecision right_safety = validateActualState(
      right_q, right_qdot, robot_.mapping(ArmSide::kRight).limits,
      config_.safety.bound_tolerance);
  if (!left_safety.accepted || !right_safety.accepted) {
    return false;
  }
  const double reset_dt = 1.0 / config_.controller.rate_hz;
  left_state_.q_ref = left_q;
  left_state_.qdot_ref = left_qdot;
  left_state_.qddot_previous = seedPreviousAccelerationForFeasibility(
      left_q, left_qdot, robot_.mapping(ArmSide::kLeft).limits,
      config_.joint_acceleration_limits, reset_dt);
  left_state_.solver_initialized = false;
  left_state_.previous_outward_jacobian_valid = false;
  left_state_.previous_arm_angle_jacobian_valid = false;
  left_state_.previous_arm_branch_lock_jacobian_valid = false;
  left_state_.dls_posture_reference = left_q;
  left_state_.dls_posture_reference_initialized = true;
  left_dls_posture_ruckig_.reset({left_q, Vec7::Zero(), Vec7::Zero()});
  right_state_.q_ref = right_q;
  right_state_.qdot_ref = right_qdot;
  right_state_.qddot_previous = seedPreviousAccelerationForFeasibility(
      right_q, right_qdot, robot_.mapping(ArmSide::kRight).limits,
      config_.joint_acceleration_limits, reset_dt);
  right_state_.solver_initialized = false;
  right_state_.previous_outward_jacobian_valid = false;
  right_state_.previous_arm_angle_jacobian_valid = false;
  right_state_.previous_arm_branch_lock_jacobian_valid = false;
  right_state_.dls_posture_reference = right_q;
  right_state_.dls_posture_reference_initialized = true;
  right_dls_posture_ruckig_.reset({right_q, Vec7::Zero(), Vec7::Zero()});
  left_solver_->reset();
  right_solver_->reset();
  return true;
}

const Vec7& DualArmAccelerationController::positionReference(
    ArmSide side) const noexcept {
  return state(side).q_ref;
}

const Vec7& DualArmAccelerationController::velocityReference(
    ArmSide side) const noexcept {
  return state(side).qdot_ref;
}

const Vec7& DualArmAccelerationController::previousAcceleration(
    ArmSide side) const noexcept {
  return state(side).qddot_previous;
}

ArmMotionState DualArmAccelerationController::referenceState(
    ArmSide side) const noexcept {
  const ArmState& arm_state = state(side);
  return {arm_state.q_ref, arm_state.qdot_ref,
          arm_state.qddot_previous};
}

bool DualArmAccelerationController::setReferenceState(
    ArmSide side, const ArmMotionState& motion) {
  const ArmLimits& limits = robot_.mapping(side).limits;
  const SafetyDecision safety = validateActualState(
      motion.q, motion.qdot, limits, config_.safety.bound_tolerance);
  if (!safety.accepted || !motion.qddot.allFinite() ||
      (motion.qddot.cwiseAbs().array() >
       (config_.joint_acceleration_limits.max_acceleration_rad_s2.array() +
        config_.safety.bound_tolerance))
          .any()) {
    return false;
  }
  ArmState& arm_state = state(side);
  arm_state.q_ref = motion.q;
  arm_state.qdot_ref = motion.qdot;
  arm_state.qddot_previous = motion.qddot;
  arm_state.solver_initialized = false;
  arm_state.previous_outward_jacobian_valid = false;
  arm_state.previous_arm_angle_jacobian_valid = false;
  arm_state.previous_arm_branch_lock_jacobian_valid = false;
  arm_state.dls_posture_reference = motion.q;
  arm_state.dls_posture_reference_initialized = true;
  JointTrajectoryLimiter7& posture_ruckig = side == ArmSide::kLeft
      ? left_dls_posture_ruckig_
      : right_dls_posture_ruckig_;
  posture_ruckig.reset({motion.q, Vec7::Zero(), Vec7::Zero()});
  IAccelerationQpSolver& solver =
      side == ArmSide::kLeft ? *left_solver_ : *right_solver_;
  solver.reset();
  robot_.setArmState(side, motion.q, motion.qdot);
  return true;
}

DualArmAccelerationController::ArmState&
DualArmAccelerationController::state(ArmSide side) noexcept {
  return side == ArmSide::kLeft ? left_state_ : right_state_;
}

const DualArmAccelerationController::ArmState&
DualArmAccelerationController::state(ArmSide side) const noexcept {
  return side == ArmSide::kLeft ? left_state_ : right_state_;
}

void DualArmAccelerationController::freezeArm(
    ArmSide side, IAccelerationQpSolver& solver) {
  ArmState& arm_state = state(side);
  arm_state.qdot_ref.setZero();
  arm_state.qddot_previous.setZero();
  arm_state.solver_initialized = false;
  arm_state.previous_outward_jacobian_valid = false;
  arm_state.dls_posture_reference = arm_state.q_ref;
  arm_state.dls_posture_reference_initialized = true;
  JointTrajectoryLimiter7& posture_ruckig = side == ArmSide::kLeft
      ? left_dls_posture_ruckig_
      : right_dls_posture_ruckig_;
  posture_ruckig.reset({arm_state.q_ref, Vec7::Zero(), Vec7::Zero()});
  solver.reset();
  robot_.setArmState(side, arm_state.q_ref, Vec7::Zero());
}

bool DualArmAccelerationController::referenceIsFinite(
    const CartesianReference& reference) const {
  return reference.valid && reference.pose.position.allFinite() &&
         reference.pose.rotation.allFinite() && reference.twist.allFinite() &&
         reference.acceleration.allFinite() &&
         isProperRotation(reference.pose.rotation);
}

}  // namespace tianji_qp_ik
