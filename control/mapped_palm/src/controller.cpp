#include "tianji_mapped_palm/controller.hpp"

#include "tianji_mapped_palm/cartesian_servo.hpp"
#include "tianji_mapped_palm/osqp_solver.hpp"
#include "tianji_mapped_palm/qp_builder.hpp"
#include "tianji_mapped_palm/qpoases_solver.hpp"
#include "tianji_mapped_palm/so3.hpp"
#include "tianji_mapped_palm/upper_arm_outward.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tianji_mapped_palm {
namespace {

double elapsedMicroseconds(std::chrono::steady_clock::time_point start,
                           std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double, std::micro>(end - start).count();
}

class LegacyVelocityIkAdapter final : public IArmVelocityIk {
 public:
  LegacyVelocityIkAdapter(QpIkConfig config,
                          std::unique_ptr<IQpSolver7> solver)
      : algorithm_(config.ik_algorithm),
        builder_(config.qp, config.joint_limits),
        safety_(config.safety),
        solver_(std::move(solver)) {}

  ArmIkResult solve(const ArmIkInput& input) override {
    ArmIkResult result;
    const QpProblem7 problem = builder_.build(
        input.jacobian, input.desired_twist, input.q_measured,
        input.limits, input.dt);
    const SafetyDecision problem_decision = safety_.validateProblem(problem);
    if (!problem_decision.accepted) {
      result.detail = "legacy QP problem rejected";
      return result;
    }
    if (!initialized_) {
      initialized_ = solver_->initialize(problem);
      if (!initialized_) {
        result.status = SolverStatus::kNumericalError;
        result.detail = "legacy QP initialization failed";
        return result;
      }
    }
    const SolverResult7 legacy = solver_->solve(problem);
    const SafetyDecision solution_decision =
        safety_.validateResult(problem, legacy);
    result.status = legacy.status;
    result.iterations = legacy.iterations;
    result.solve_time_us = legacy.solve_time_us;
    result.detail = legacy.detail;
    if (!solution_decision.accepted) {
      if (result.status == SolverStatus::kSolved) {
        result.status = SolverStatus::kNumericalError;
      }
      result.detail = "legacy QP solution rejected";
      return result;
    }
    result.qdot = legacy.qdot;
    result.slack = input.desired_twist - input.jacobian * result.qdot;
    result.equality_residual =
        (input.jacobian * result.qdot + result.slack -
         input.desired_twist).norm();
    return result;
  }

  void reset() override {
    solver_->reset();
    initialized_ = false;
  }

  IkAlgorithm algorithm() const noexcept override { return algorithm_; }

 private:
  IkAlgorithm algorithm_;
  QpBuilder builder_;
  SafetyGuard safety_;
  std::unique_ptr<IQpSolver7> solver_;
  bool initialized_{false};
};

SafetyDecision validateBounds(const JointVelocityBounds& bounds) {
  if (!bounds.lower.allFinite() || !bounds.upper.allFinite()) {
    return {false, HoldReason::kNonFiniteProblem, -1};
  }
  for (int index = 0; index < kArmDof; ++index) {
    if (bounds.lower[index] > bounds.upper[index]) {
      return {false, HoldReason::kInfeasibleBounds, index};
    }
  }
  return {true, HoldReason::kNone, -1};
}

SafetyDecision validateActualPosition(const Vec7& position,
                                      const ArmLimits& limits,
                                      double tolerance) {
  if (!position.allFinite()) {
    return {false, HoldReason::kNonFiniteProblem, -1};
  }
  for (int index = 0; index < kArmDof; ++index) {
    if (position[index] < limits.lower_position[index] - tolerance ||
        position[index] > limits.upper_position[index] + tolerance) {
      return {false, HoldReason::kBoundViolation, index};
    }
  }
  return {true, HoldReason::kNone, -1};
}

double referenceTrackingScale(double error, const SafetyConfig& config) {
  if (error <= config.reference_tracking_warn_rad) {
    return 1.0;
  }
  if (error >= config.reference_tracking_stop_rad) {
    return 0.0;
  }
  return (config.reference_tracking_stop_rad - error) /
         (config.reference_tracking_stop_rad -
          config.reference_tracking_warn_rad);
}

ArmAngleGeometryInput armAngleGeometry(const ArmKinematicSample& sample) {
  return {sample.shoulder_position, sample.elbow_position,
          sample.wrist_position, sample.shoulder_position_jacobian,
          sample.elbow_position_jacobian, sample.wrist_position_jacobian};
}

ArmDirectionReference withTargetShoulderWristAxis(
    const ArmDirectionReference& reference,
    const ArmKinematicSample& current,
    const Pose& target) noexcept {
  ArmDirectionReference result = reference;
  if (!reference.valid ||
      reference.source != ArmDirectionReferenceSource::kPico ||
      !target.position.allFinite() || !target.rotation.allFinite() ||
      !current.tcp_pose.position.allFinite() ||
      !current.tcp_pose.rotation.allFinite() ||
      !current.wrist_position.allFinite() ||
      !current.shoulder_position.allFinite()) {
    return result;
  }
  const Eigen::Vector3d wrist_offset_tcp =
      current.tcp_pose.rotation.transpose() *
      (current.wrist_position - current.tcp_pose.position);
  const Eigen::Vector3d target_wrist =
      target.position + target.rotation * wrist_offset_tcp;
  const Eigen::Vector3d target_axis =
      target_wrist - current.shoulder_position;
  if (!target_axis.allFinite() || target_axis.norm() <= 1.0e-9) {
    return result;
  }
  result.shoulder_to_wrist_axis = target_axis.normalized();
  result.shoulder_to_wrist_axis_valid = true;
  return result;
}

ScalarJointTask velocityArmAngleTask(const ArmAngleTask& task,
                                     const ArmAngleConfig& config) {
  ScalarJointTask result;
  result.jacobian = task.jacobian;
  result.target = std::clamp(config.kp_velocity * task.control_error_rad,
                             -config.max_velocity_rad_s,
                             config.max_velocity_rad_s);
  result.activation = task.activation;
  result.nullspace_only = config.nullspace_only;
  result.active = config.enabled && task.active && result.activation > 0.0;
  return result;
}

ScalarJointTask velocityArmAngleContinuityTask(
    const ArmAngleTask& task, const ArmAngleConfig& config) {
  ScalarJointTask result;
  result.jacobian = task.jacobian;
  result.target = std::clamp(
      config.continuity_kp_velocity * task.control_error_rad,
      -config.continuity_max_velocity_rad_s,
      config.continuity_max_velocity_rad_s);
  result.activation = task.activation;
  result.weight_scale = config.continuity_weight_scale;
  result.nullspace_only = config.nullspace_only;
  result.active = config.enabled && task.active && result.activation > 0.0;
  return result;
}

JointPositionGuard armAnglePositionGuard(
    const Vec7& position, const ArmLimits& limits,
    const JointLimitConfig& joint_limits,
    const ArmAngleConfig& arm_angle) {
  JointPositionGuard result;
  result.active = arm_angle.enabled &&
                  arm_angle.joint_limit_soft_margin_rad > 0.0;
  result.position = position;
  result.lower = limits.lower_position.array() + joint_limits.margin_rad;
  result.upper = limits.upper_position.array() - joint_limits.margin_rad;
  result.soft_margin_rad = arm_angle.joint_limit_soft_margin_rad;
  result.recovery_gain_rad_s_per_rad =
      arm_angle.joint_limit_recovery_gain_rad_s_per_rad;
  return result;
}

}  // namespace

DualArmController::DualArmController(MujocoRobot& robot, QpIkConfig config)
    : robot_(robot),
      config_(std::move(config)),
      algorithm_(config_.ik_algorithm),
      left_ik_(makeArmVelocityIk(algorithm_, config_)),
      right_ik_(makeArmVelocityIk(algorithm_, config_)),
      left_ff_allocator_(std::make_unique<CartesianFeedforwardAllocator7>(
          config_.pico_ee_feedforward_allocator, config_.joint_limits,
          config_.cartesian_servo, makeArmVelocityIk(algorithm_, config_))),
      right_ff_allocator_(std::make_unique<CartesianFeedforwardAllocator7>(
          config_.pico_ee_feedforward_allocator, config_.joint_limits,
          config_.cartesian_servo, makeArmVelocityIk(algorithm_, config_))),
      left_task_allocator_(std::make_unique<CartesianTaskAllocator7>(
          config_.pico_ee_task_allocator, config_.joint_limits,
          config_.cartesian_servo, makeArmVelocityIk(algorithm_, config_))),
      right_task_allocator_(std::make_unique<CartesianTaskAllocator7>(
          config_.pico_ee_task_allocator, config_.joint_limits,
          config_.cartesian_servo, makeArmVelocityIk(algorithm_, config_))),
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
                       config_.arm_angle.reference_governor_tracking_error_rad),
      left_mapped_vector_governor_(
          config_.pico_mapped_arm_angle_vector_nullspace.governor),
      right_mapped_vector_governor_(
          config_.pico_mapped_arm_angle_vector_nullspace.governor),
      left_motion_confidence_(config_.pico_ee_motion_confidence),
      right_motion_confidence_(config_.pico_ee_motion_confidence),
      left_dls_posture_(config_.iterative_dls,
                        config_.joint_limits.margin_rad),
      right_dls_posture_(config_.iterative_dls,
                         config_.joint_limits.margin_rad),
      left_dls_posture_ruckig_(
          config_.dls_posture_ruckig,
          robot_.mapping(ArmSide::kLeft).limits,
          1.0 / config_.controller.rate_hz),
      right_dls_posture_ruckig_(
          config_.dls_posture_ruckig,
          robot_.mapping(ArmSide::kRight).limits,
          1.0 / config_.controller.rate_hz) {
  left_state_.q_ref = robot_.armPosition(ArmSide::kLeft);
  right_state_.q_ref = robot_.armPosition(ArmSide::kRight);
  resetDlsPosture(ArmSide::kLeft);
  resetDlsPosture(ArmSide::kRight);
}

DualArmController::DualArmController(
    MujocoRobot& robot, QpIkConfig config, IkAlgorithm algorithm,
    std::unique_ptr<IArmVelocityIk> left,
    std::unique_ptr<IArmVelocityIk> right)
    : robot_(robot),
      config_(std::move(config)),
      algorithm_(algorithm),
      left_ik_(std::move(left)),
      right_ik_(std::move(right)),
      left_ff_allocator_(std::make_unique<CartesianFeedforwardAllocator7>(
          config_.pico_ee_feedforward_allocator, config_.joint_limits,
          config_.cartesian_servo, makeArmVelocityIk(algorithm_, config_))),
      right_ff_allocator_(std::make_unique<CartesianFeedforwardAllocator7>(
          config_.pico_ee_feedforward_allocator, config_.joint_limits,
          config_.cartesian_servo, makeArmVelocityIk(algorithm_, config_))),
      left_task_allocator_(std::make_unique<CartesianTaskAllocator7>(
          config_.pico_ee_task_allocator, config_.joint_limits,
          config_.cartesian_servo, makeArmVelocityIk(algorithm_, config_))),
      right_task_allocator_(std::make_unique<CartesianTaskAllocator7>(
          config_.pico_ee_task_allocator, config_.joint_limits,
          config_.cartesian_servo, makeArmVelocityIk(algorithm_, config_))),
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
                       config_.arm_angle.reference_governor_tracking_error_rad),
      left_mapped_vector_governor_(
          config_.pico_mapped_arm_angle_vector_nullspace.governor),
      right_mapped_vector_governor_(
          config_.pico_mapped_arm_angle_vector_nullspace.governor),
      left_motion_confidence_(config_.pico_ee_motion_confidence),
      right_motion_confidence_(config_.pico_ee_motion_confidence),
      left_dls_posture_(config_.iterative_dls,
                        config_.joint_limits.margin_rad),
      right_dls_posture_(config_.iterative_dls,
                         config_.joint_limits.margin_rad),
      left_dls_posture_ruckig_(
          config_.dls_posture_ruckig,
          robot_.mapping(ArmSide::kLeft).limits,
          1.0 / config_.controller.rate_hz),
      right_dls_posture_ruckig_(
          config_.dls_posture_ruckig,
          robot_.mapping(ArmSide::kRight).limits,
          1.0 / config_.controller.rate_hz) {
  if (left_ik_ == nullptr || right_ik_ == nullptr) {
    throw std::invalid_argument("DualArmController requires two IK backends");
  }
  left_state_.q_ref = robot_.armPosition(ArmSide::kLeft);
  right_state_.q_ref = robot_.armPosition(ArmSide::kRight);
  resetDlsPosture(ArmSide::kLeft);
  resetDlsPosture(ArmSide::kRight);
}

DualArmController::DualArmController(
    MujocoRobot& robot, QpIkConfig config,
    std::unique_ptr<IQpSolver7> left_solver,
    std::unique_ptr<IQpSolver7> right_solver)
    : DualArmController(
          robot, config, config.ik_algorithm,
          std::make_unique<LegacyVelocityIkAdapter>(config, std::move(left_solver)),
          std::make_unique<LegacyVelocityIkAdapter>(config, std::move(right_solver))) {}

ControllerDiagnostics DualArmController::step(const DualArmTargets& targets,
                                              double dt) {
  return stepImpl(targets, nullptr, defaultArmDirectionReferences(), nullptr,
                  nullptr, dt);
}

ControllerDiagnostics DualArmController::step(
    const DualArmTargets& targets,
    const DualArmDirectionReferences& arm_directions, double dt) {
  return stepImpl(targets, nullptr, arm_directions, nullptr, nullptr, dt);
}

ControllerDiagnostics DualArmController::step(
    const DualArmReferences& references, double dt) {
  DualArmTargets targets;
  targets.left = references.left.pose;
  targets.right = references.right.pose;
  targets.left_twist = references.left.twist;
  targets.right_twist = references.right.twist;
  targets.left_stale = references.left.stale;
  targets.right_stale = references.right.stale;
  return stepImpl(targets, &references, defaultArmDirectionReferences(),
                  nullptr, nullptr, dt);
}

ControllerDiagnostics DualArmController::step(
    const DualArmReferences& references,
    const DualArmDirectionReferences& arm_directions, double dt) {
  DualArmTargets targets;
  targets.left = references.left.pose;
  targets.right = references.right.pose;
  targets.left_twist = references.left.twist;
  targets.right_twist = references.right.twist;
  targets.left_stale = references.left.stale;
  targets.right_stale = references.right.stale;
  return stepImpl(targets, &references, arm_directions, nullptr, nullptr, dt);
}

ControllerDiagnostics DualArmController::step(
    const DualArmReferences& references,
    const DualArmDirectionReferences& arm_directions,
    const DualArmJointVelocityPostureTasks& posture_tasks, double dt) {
  DualArmTargets targets;
  targets.left = references.left.pose;
  targets.right = references.right.pose;
  targets.left_twist = references.left.twist;
  targets.right_twist = references.right.twist;
  targets.left_stale = references.left.stale;
  targets.right_stale = references.right.stale;
  return stepImpl(targets, &references, arm_directions, &posture_tasks,
                  nullptr, dt);
}

ControllerDiagnostics DualArmController::step(
    const DualArmReferences& references,
    const DualArmDirectionReferences& arm_directions,
    const DualArmCartesianFeedforwardDemand& feedforward, double dt) {
  DualArmTargets targets;
  targets.left = references.left.pose;
  targets.right = references.right.pose;
  targets.left_twist = references.left.twist;
  targets.right_twist = references.right.twist;
  targets.left_stale = references.left.stale;
  targets.right_stale = references.right.stale;
  return stepImpl(targets, &references, arm_directions, nullptr, &feedforward,
                  dt);
}

ControllerDiagnostics DualArmController::stepImpl(
    const DualArmTargets& targets, const DualArmReferences* references,
    const DualArmDirectionReferences& arm_directions,
    const DualArmJointVelocityPostureTasks* posture_tasks,
    const DualArmCartesianFeedforwardDemand* feedforward, double dt) {
  const auto cycle_start = std::chrono::steady_clock::now();
  ControllerDiagnostics diagnostics;
  diagnostics.left.target = targets.left;
  diagnostics.right.target = targets.right;
  if (references != nullptr) {
    diagnostics.left.reference = references->left;
    diagnostics.right.reference = references->right;
  } else {
    diagnostics.left.reference.pose = targets.left;
    diagnostics.left.reference.twist = targets.left_twist;
    diagnostics.left.reference.stale = targets.left_stale;
    diagnostics.right.reference.pose = targets.right;
    diagnostics.right.reference.twist = targets.right_twist;
    diagnostics.right.reference.stale = targets.right_stale;
  }
  diagnostics.left.q_ref = left_state_.q_ref;
  diagnostics.right.q_ref = right_state_.q_ref;
  if (!std::isfinite(dt) || dt <= 0.0 ||
      (references == nullptr && !targetsAreFinite(targets))) {
    clearHistory();
    diagnostics.hold_reason = HoldReason::kNonFiniteProblem;
    diagnostics.compute_time_us =
        elapsedMicroseconds(cycle_start, std::chrono::steady_clock::now());
    return diagnostics;
  }

  const Vec7 left_q = robot_.armPosition(ArmSide::kLeft);
  const Vec7 right_q = robot_.armPosition(ArmSide::kRight);
  diagnostics.left.q_actual = left_q;
  diagnostics.right.q_actual = right_q;
  if (!left_q.allFinite() || !right_q.allFinite()) {
    clearHistory();
    diagnostics.left.safety =
        {false, HoldReason::kNonFiniteProblem, -1};
    diagnostics.right.safety =
        {false, HoldReason::kNonFiniteProblem, -1};
    diagnostics.left.hold_reason = HoldReason::kNonFiniteProblem;
    diagnostics.right.hold_reason = HoldReason::kNonFiniteProblem;
    diagnostics.hold_reason = HoldReason::kNonFiniteProblem;
    diagnostics.compute_time_us =
        elapsedMicroseconds(cycle_start, std::chrono::steady_clock::now());
    return diagnostics;
  }
  robot_.forward();
  diagnostics.left.tcp_actual = robot_.tcpPose(ArmSide::kLeft);
  diagnostics.right.tcp_actual = robot_.tcpPose(ArmSide::kRight);
  const ArmKinematicSample left_model =
      robot_.armKinematicsAt(ArmSide::kLeft, left_state_.q_ref);
  const ArmKinematicSample right_model =
      robot_.armKinematicsAt(ArmSide::kRight, right_state_.q_ref);
  diagnostics.left.current = left_model.tcp_pose;
  diagnostics.right.current = right_model.tcp_pose;
  const ArmDirectionReference left_arm_direction =
      usesPicoEeMappedCorrectedPalm(algorithm_)
          ? arm_directions.left
          : withTargetShoulderWristAxis(arm_directions.left, left_model,
                                        diagnostics.left.reference.pose);
  const ArmDirectionReference right_arm_direction =
      usesPicoEeMappedCorrectedPalm(algorithm_)
          ? arm_directions.right
          : withTargetShoulderWristAxis(arm_directions.right, right_model,
                                        diagnostics.right.reference.pose);
  if (usesContinuityArmDirection(arm_angle_reference_mode_)) {
    diagnostics.left.arm_angle =
        left_arm_angle_.computeContinuity(armAngleGeometry(left_model), dt);
    diagnostics.right.arm_angle =
        right_arm_angle_.computeContinuity(armAngleGeometry(right_model), dt);
  } else {
    diagnostics.left.arm_angle = left_arm_angle_.compute(
        armAngleGeometry(left_model), left_arm_direction, dt);
    diagnostics.right.arm_angle = right_arm_angle_.compute(
        armAngleGeometry(right_model), right_arm_direction, dt);
  }
  const bool left_reference_valid =
      references == nullptr || referenceIsFinite(references->left);
  const bool right_reference_valid =
      references == nullptr || referenceIsFinite(references->right);
  if (left_reference_valid) {
    diagnostics.left.pose_error =
        poseErrorWorld(targets.left, diagnostics.left.current);
    diagnostics.left.actual_pose_error =
        poseErrorWorld(targets.left, diagnostics.left.tcp_actual);
  }
  if (right_reference_valid) {
    diagnostics.right.pose_error =
        poseErrorWorld(targets.right, diagnostics.right.current);
    diagnostics.right.actual_pose_error =
        poseErrorWorld(targets.right, diagnostics.right.tcp_actual);
  }
  diagnostics.left.reference_error_max_abs =
      (left_state_.q_ref - left_q).cwiseAbs().maxCoeff();
  diagnostics.right.reference_error_max_abs =
      (right_state_.q_ref - right_q).cwiseAbs().maxCoeff();
  diagnostics.left.reference_scale = config_.controller.model_state_only
                                         ? 1.0
                                         : referenceTrackingScale(
                                               diagnostics.left
                                                   .reference_error_max_abs,
                                               config_.safety);
  diagnostics.right.reference_scale = config_.controller.model_state_only
                                          ? 1.0
                                          : referenceTrackingScale(
                                                diagnostics.right
                                                    .reference_error_max_abs,
                                                config_.safety);
  diagnostics.left.reference_frozen = diagnostics.left.reference_scale <= 0.0;
  diagnostics.right.reference_frozen = diagnostics.right.reference_scale <= 0.0;

  ArmIkInput left_input;
  Vec6 left_feedback_twist = Vec6::Zero();
  left_input.q_measured = left_state_.q_ref;
  left_input.q_ref = left_state_.q_ref;
  left_input.qdot_prev = left_state_.qdot_prev;
  left_input.qddot_prev = left_state_.qddot_prev;
  left_input.slack_prev = left_state_.slack_prev;
  diagnostics.left.redundancy_authority =
      feedforward != nullptr
          ? std::clamp(feedforward->left.redundancy_authority, 0.0, 1.0)
          : 1.0;
  left_input.jacobian = left_model.tcp_jacobian;
  left_input.arm_angle_task = usesOutwardArmBarrier(arm_angle_reference_mode_)
      ? velocityArmAngleContinuityTask(diagnostics.left.arm_angle,
                                       config_.arm_angle)
      : velocityArmAngleTask(diagnostics.left.arm_angle, config_.arm_angle);
  if (algorithm_ != IkAlgorithm::kHierarchicalQp &&
      !usesPicoEeMappedCorrectedPalm(algorithm_)) {
    left_input.arm_angle_task.active = false;
  }
  diagnostics.left.arm_angle_task_active = left_input.arm_angle_task.active;
  diagnostics.left.arm_angle_requested_rate =
      left_input.arm_angle_task.active ? left_input.arm_angle_task.target : 0.0;
  if (left_reference_valid) {
    if (references != nullptr) {
      PicoEeMotionEvidenceInput motion_input;
      motion_input.dt = dt;
      motion_input.stale = references->left.stale;
      if (feedforward != nullptr) {
        motion_input.low_frequency_twist = feedforward->left.low_frequency;
        motion_input.freshness = feedforward->left.freshness;
        motion_input.estimator_confidence =
            feedforward->left.estimator_confidence;
        motion_input.reset = feedforward->left.reset;
      }
      diagnostics.left.motion_evidence =
          left_motion_confidence_.update(motion_input);
      diagnostics.left.cartesian_feedback_gains = cartesianFeedbackGainSchedule(
          config_.cartesian_servo, references->left.pose,
          diagnostics.left.current,
          diagnostics.left.motion_evidence.linear_filtered,
          diagnostics.left.motion_evidence.angular_filtered);
      left_feedback_twist = cartesianReferenceFeedbackTwist(
                                config_.cartesian_servo,
                                references->left.pose,
                                diagnostics.left.current,
                                0.0) *
                            diagnostics.left.reference_scale;
      left_feedback_twist.head<3>() =
          diagnostics.left.cartesian_feedback_gains.position.cwiseProduct(
              diagnostics.left.pose_error.head<3>()) *
          diagnostics.left.reference_scale;
      left_feedback_twist.tail<3>() =
          diagnostics.left.cartesian_feedback_gains.orientation.cwiseProduct(
              diagnostics.left.pose_error.tail<3>()) *
          diagnostics.left.reference_scale;
      diagnostics.left.cartesian_feedback_twist = left_feedback_twist;
      diagnostics.left.preallocation_feedforward_twist =
          references->left.twist * diagnostics.left.reference_scale;
    }
    if (references != nullptr && config_.cartesian_servo.motion_gain_enabled) {
      left_input.desired_twist = clampCartesianTwist(
          config_.cartesian_servo,
          left_feedback_twist +
              references->left.twist * diagnostics.left.reference_scale);
    } else {
      left_input.desired_twist =
          (references == nullptr
               ? cartesianServoTwist(config_.cartesian_servo, targets.left,
                                     diagnostics.left.current,
                                     targets.left_twist)
               : cartesianReferenceServoTwist(config_.cartesian_servo,
                                               references->left,
                                               diagnostics.left.current)) *
          diagnostics.left.reference_scale;
    }
  }
  left_input.limits = robot_.mapping(ArmSide::kLeft).limits;
  left_input.bounds = computeJointVelocityBounds(
      left_state_.q_ref, left_state_.qdot_prev, left_state_.qddot_prev,
      left_input.limits,
      config_.joint_limits, dt);
  left_input.arm_angle_position_guard = armAnglePositionGuard(
      left_state_.q_ref, left_input.limits, config_.joint_limits,
      config_.arm_angle);
  left_input.arm_angle_task = applyArmAngleJointLimitRecovery(
      left_input.arm_angle_task, left_input.jacobian,
      left_input.arm_angle_position_guard,
      usesOutwardArmBarrier(arm_angle_reference_mode_)
          ? config_.arm_angle.continuity_max_velocity_rad_s
          : config_.arm_angle.max_velocity_rad_s);
  diagnostics.left.arm_angle_task_active =
      left_input.arm_angle_task.active;
  diagnostics.left.arm_angle_requested_rate =
      left_input.arm_angle_task.active
          ? left_input.arm_angle_task.target
          : 0.0;
  diagnostics.left.upper_arm_outward.state = computeUpperArmOutwardState(
      ArmSide::kLeft, left_model.shoulder_position, left_model.elbow_position,
      left_model.shoulder_position_jacobian,
      left_model.elbow_position_jacobian,
      config_.upper_arm_outward.minimum_outward_distance_m);
  if (usesHierarchicalVelocityQp(algorithm_) &&
      usesOutwardArmBarrier(arm_angle_reference_mode_)) {
    left_input.linear_constraint = makeVelocityOutwardConstraint(
        diagnostics.left.upper_arm_outward.state, left_input.bounds.lower,
        left_input.bounds.upper, config_.upper_arm_outward, dt);
  }
  if (usesHierarchicalVelocityQp(algorithm_) &&
      config_.arm_angle.branch_lock_enabled &&
      diagnostics.left.arm_angle.branch_lock_active) {
    LinearJointConstraint branch_lock =
        makeArmAngleBranchLockVelocityConstraint(
            diagnostics.left.arm_angle, left_input.bounds.lower,
            left_input.bounds.upper,
            config_.arm_angle.branch_lock_kp_velocity);
    if (usesOutwardArmBarrier(arm_angle_reference_mode_)) {
      left_input.branch_lock_constraint = branch_lock;
    } else {
      left_input.linear_constraint = branch_lock;
    }
  }
  if (usesHierarchicalVelocityQp(algorithm_) &&
      config_.arm_angle.enabled &&
      config_.arm_angle.tracking_envelope_enabled &&
      usesPicoArmDirection(arm_angle_reference_mode_) &&
      !diagnostics.left.arm_angle.branch_lock_active) {
    left_input.branch_lock_constraint =
        makeArmAngleTrackingEnvelopeVelocityConstraint(
            diagnostics.left.arm_angle, left_input.bounds.lower,
            left_input.bounds.upper,
            config_.arm_angle.tracking_envelope_max_error_rad,
            config_.arm_angle.tracking_envelope_gain);
  }
  if (usesOutwardArmBarrier(arm_angle_reference_mode_)) {
    diagnostics.left.upper_arm_outward.constraint_active =
        left_input.linear_constraint.active;
    diagnostics.left.upper_arm_outward.requested_lower =
        left_input.linear_constraint.requested_lower;
    diagnostics.left.upper_arm_outward.effective_lower =
        left_input.linear_constraint.lower;
    diagnostics.left.upper_arm_outward.feasibility_clipped =
        left_input.linear_constraint.feasibility_clipped;
  }
  if (diagnostics.left.arm_angle.branch_lock_active) {
    const LinearJointConstraint& branch_lock =
        usesOutwardArmBarrier(arm_angle_reference_mode_)
            ? left_input.branch_lock_constraint
            : left_input.linear_constraint;
    diagnostics.left.arm_angle.branch_lock_constraint_active =
        branch_lock.active;
    diagnostics.left.arm_angle.branch_lock_requested_lower =
        branch_lock.requested_lower;
    diagnostics.left.arm_angle.branch_lock_effective_lower =
        branch_lock.lower;
    diagnostics.left.arm_angle.branch_lock_feasibility_clipped =
        branch_lock.feasibility_clipped;
  }
  diagnostics.left.bounds = left_input.bounds;
  left_input.dt = dt;
  configureDlsPostureTask(
      ArmSide::kLeft,
      references != nullptr ? references->left.pose : targets.left,
      references != nullptr ? references->left.stale : targets.left_stale,
      diagnostics.left.arm_angle, dt, left_input, diagnostics.left);
  if (posture_tasks != nullptr && posture_tasks->left.active) {
    left_input.posture_task = posture_tasks->left;
    if (left_input.posture_task.source ==
        JointVelocityPostureSource::kSparkJointReference) {
      left_input.desired_twist +=
          left_input.jacobian * left_input.posture_task.target;
    }
    left_input.arm_angle_task.active = false;
    diagnostics.left.arm_angle_task_active = false;
    diagnostics.left.arm_angle_requested_rate = 0.0;
  }
  configureMappedArmAngleVectorTask(
      ArmSide::kLeft,
      references != nullptr ? references->left.stale : targets.left_stale,
      dt, left_input, diagnostics.left);
  if (feedforward != nullptr && references != nullptr &&
      usesPicoEeMappedCorrectedPalm(algorithm_)) {
    CartesianFeedforwardDemand scaled = feedforward->left;
    scaled.low_frequency *= diagnostics.left.reference_scale;
    scaled.high_frequency *= diagnostics.left.reference_scale;
    diagnostics.left.feedforward_allocation = left_ff_allocator_->allocate(
        left_input, left_feedback_twist, scaled);
    if (config_.pico_ee_feedforward_allocator.mode ==
            PicoEeFeedforwardAllocatorMode::kActive &&
        diagnostics.left.feedforward_allocation.valid) {
      left_input.desired_twist = clampCartesianTwist(
          config_.cartesian_servo,
          left_feedback_twist +
              diagnostics.left.feedforward_allocation.allocated_feedforward);
    }
  }
  if (left_reference_valid && usesPicoEeMappedCorrectedPalm(algorithm_) &&
      config_.pico_ee_task_allocator.mode !=
          PicoEeTaskAllocationMode::kDisabled) {
    diagnostics.left.task_allocation =
        left_task_allocator_->allocate(left_input, left_feedback_twist);
    if (diagnostics.left.task_allocation.authorized) {
      if (config_.pico_ee_task_allocator.nullspace_only) {
        left_input.vector_nullspace_task.alpha_command =
            diagnostics.left.task_allocation
                .predicted_nullspace_alpha_command;
      } else {
        left_input.desired_twist =
            diagnostics.left.task_allocation.predicted_desired_twist;
      }
    }
  }
  diagnostics.left.final_desired_twist = left_input.desired_twist;

  ArmIkInput right_input;
  Vec6 right_feedback_twist = Vec6::Zero();
  right_input.q_measured = right_state_.q_ref;
  right_input.q_ref = right_state_.q_ref;
  right_input.qdot_prev = right_state_.qdot_prev;
  right_input.qddot_prev = right_state_.qddot_prev;
  right_input.slack_prev = right_state_.slack_prev;
  diagnostics.right.redundancy_authority =
      feedforward != nullptr
          ? std::clamp(feedforward->right.redundancy_authority, 0.0, 1.0)
          : 1.0;
  right_input.jacobian = right_model.tcp_jacobian;
  right_input.arm_angle_task = usesOutwardArmBarrier(arm_angle_reference_mode_)
      ? velocityArmAngleContinuityTask(diagnostics.right.arm_angle,
                                       config_.arm_angle)
      : velocityArmAngleTask(diagnostics.right.arm_angle, config_.arm_angle);
  if (algorithm_ != IkAlgorithm::kHierarchicalQp &&
      !usesPicoEeMappedCorrectedPalm(algorithm_)) {
    right_input.arm_angle_task.active = false;
  }
  diagnostics.right.arm_angle_task_active = right_input.arm_angle_task.active;
  diagnostics.right.arm_angle_requested_rate =
      right_input.arm_angle_task.active ? right_input.arm_angle_task.target : 0.0;
  if (right_reference_valid) {
    if (references != nullptr) {
      PicoEeMotionEvidenceInput motion_input;
      motion_input.dt = dt;
      motion_input.stale = references->right.stale;
      if (feedforward != nullptr) {
        motion_input.low_frequency_twist = feedforward->right.low_frequency;
        motion_input.freshness = feedforward->right.freshness;
        motion_input.estimator_confidence =
            feedforward->right.estimator_confidence;
        motion_input.reset = feedforward->right.reset;
      }
      diagnostics.right.motion_evidence =
          right_motion_confidence_.update(motion_input);
      diagnostics.right.cartesian_feedback_gains = cartesianFeedbackGainSchedule(
          config_.cartesian_servo, references->right.pose,
          diagnostics.right.current,
          diagnostics.right.motion_evidence.linear_filtered,
          diagnostics.right.motion_evidence.angular_filtered);
      right_feedback_twist = cartesianReferenceFeedbackTwist(
                                 config_.cartesian_servo,
                                 references->right.pose,
                                 diagnostics.right.current,
                                 0.0) *
                             diagnostics.right.reference_scale;
      right_feedback_twist.head<3>() =
          diagnostics.right.cartesian_feedback_gains.position.cwiseProduct(
              diagnostics.right.pose_error.head<3>()) *
          diagnostics.right.reference_scale;
      right_feedback_twist.tail<3>() =
          diagnostics.right.cartesian_feedback_gains.orientation.cwiseProduct(
              diagnostics.right.pose_error.tail<3>()) *
          diagnostics.right.reference_scale;
      diagnostics.right.cartesian_feedback_twist = right_feedback_twist;
      diagnostics.right.preallocation_feedforward_twist =
          references->right.twist * diagnostics.right.reference_scale;
    }
    if (references != nullptr && config_.cartesian_servo.motion_gain_enabled) {
      right_input.desired_twist = clampCartesianTwist(
          config_.cartesian_servo,
          right_feedback_twist +
              references->right.twist * diagnostics.right.reference_scale);
    } else {
      right_input.desired_twist =
          (references == nullptr
               ? cartesianServoTwist(config_.cartesian_servo, targets.right,
                                     diagnostics.right.current,
                                     targets.right_twist)
               : cartesianReferenceServoTwist(config_.cartesian_servo,
                                               references->right,
                                               diagnostics.right.current)) *
          diagnostics.right.reference_scale;
    }
  }
  right_input.limits = robot_.mapping(ArmSide::kRight).limits;
  right_input.bounds = computeJointVelocityBounds(
      right_state_.q_ref, right_state_.qdot_prev, right_state_.qddot_prev,
      right_input.limits,
      config_.joint_limits, dt);
  right_input.arm_angle_position_guard = armAnglePositionGuard(
      right_state_.q_ref, right_input.limits, config_.joint_limits,
      config_.arm_angle);
  right_input.arm_angle_task = applyArmAngleJointLimitRecovery(
      right_input.arm_angle_task, right_input.jacobian,
      right_input.arm_angle_position_guard,
      usesOutwardArmBarrier(arm_angle_reference_mode_)
          ? config_.arm_angle.continuity_max_velocity_rad_s
          : config_.arm_angle.max_velocity_rad_s);
  diagnostics.right.arm_angle_task_active =
      right_input.arm_angle_task.active;
  diagnostics.right.arm_angle_requested_rate =
      right_input.arm_angle_task.active
          ? right_input.arm_angle_task.target
          : 0.0;
  diagnostics.right.upper_arm_outward.state = computeUpperArmOutwardState(
      ArmSide::kRight, right_model.shoulder_position,
      right_model.elbow_position, right_model.shoulder_position_jacobian,
      right_model.elbow_position_jacobian,
      config_.upper_arm_outward.minimum_outward_distance_m);
  if (usesHierarchicalVelocityQp(algorithm_) &&
      usesOutwardArmBarrier(arm_angle_reference_mode_)) {
    right_input.linear_constraint = makeVelocityOutwardConstraint(
        diagnostics.right.upper_arm_outward.state, right_input.bounds.lower,
        right_input.bounds.upper, config_.upper_arm_outward, dt);
  }
  if (usesHierarchicalVelocityQp(algorithm_) &&
      config_.arm_angle.branch_lock_enabled &&
      diagnostics.right.arm_angle.branch_lock_active) {
    LinearJointConstraint branch_lock =
        makeArmAngleBranchLockVelocityConstraint(
            diagnostics.right.arm_angle, right_input.bounds.lower,
            right_input.bounds.upper,
            config_.arm_angle.branch_lock_kp_velocity);
    if (usesOutwardArmBarrier(arm_angle_reference_mode_)) {
      right_input.branch_lock_constraint = branch_lock;
    } else {
      right_input.linear_constraint = branch_lock;
    }
  }
  if (usesHierarchicalVelocityQp(algorithm_) &&
      config_.arm_angle.enabled &&
      config_.arm_angle.tracking_envelope_enabled &&
      usesPicoArmDirection(arm_angle_reference_mode_) &&
      !diagnostics.right.arm_angle.branch_lock_active) {
    right_input.branch_lock_constraint =
        makeArmAngleTrackingEnvelopeVelocityConstraint(
            diagnostics.right.arm_angle, right_input.bounds.lower,
            right_input.bounds.upper,
            config_.arm_angle.tracking_envelope_max_error_rad,
            config_.arm_angle.tracking_envelope_gain);
  }
  if (usesOutwardArmBarrier(arm_angle_reference_mode_)) {
    diagnostics.right.upper_arm_outward.constraint_active =
        right_input.linear_constraint.active;
    diagnostics.right.upper_arm_outward.requested_lower =
        right_input.linear_constraint.requested_lower;
    diagnostics.right.upper_arm_outward.effective_lower =
        right_input.linear_constraint.lower;
    diagnostics.right.upper_arm_outward.feasibility_clipped =
        right_input.linear_constraint.feasibility_clipped;
  }
  if (diagnostics.right.arm_angle.branch_lock_active) {
    const LinearJointConstraint& branch_lock =
        usesOutwardArmBarrier(arm_angle_reference_mode_)
            ? right_input.branch_lock_constraint
            : right_input.linear_constraint;
    diagnostics.right.arm_angle.branch_lock_constraint_active =
        branch_lock.active;
    diagnostics.right.arm_angle.branch_lock_requested_lower =
        branch_lock.requested_lower;
    diagnostics.right.arm_angle.branch_lock_effective_lower =
        branch_lock.lower;
    diagnostics.right.arm_angle.branch_lock_feasibility_clipped =
        branch_lock.feasibility_clipped;
  }
  diagnostics.right.bounds = right_input.bounds;
  right_input.dt = dt;
  configureDlsPostureTask(
      ArmSide::kRight,
      references != nullptr ? references->right.pose : targets.right,
      references != nullptr ? references->right.stale : targets.right_stale,
      diagnostics.right.arm_angle, dt, right_input, diagnostics.right);
  if (posture_tasks != nullptr && posture_tasks->right.active) {
    right_input.posture_task = posture_tasks->right;
    if (right_input.posture_task.source ==
        JointVelocityPostureSource::kSparkJointReference) {
      right_input.desired_twist +=
          right_input.jacobian * right_input.posture_task.target;
    }
    right_input.arm_angle_task.active = false;
    diagnostics.right.arm_angle_task_active = false;
    diagnostics.right.arm_angle_requested_rate = 0.0;
  }
  configureMappedArmAngleVectorTask(
      ArmSide::kRight,
      references != nullptr ? references->right.stale : targets.right_stale,
      dt, right_input, diagnostics.right);
  if (feedforward != nullptr && references != nullptr &&
      usesPicoEeMappedCorrectedPalm(algorithm_)) {
    CartesianFeedforwardDemand scaled = feedforward->right;
    scaled.low_frequency *= diagnostics.right.reference_scale;
    scaled.high_frequency *= diagnostics.right.reference_scale;
    diagnostics.right.feedforward_allocation = right_ff_allocator_->allocate(
        right_input, right_feedback_twist, scaled);
    if (config_.pico_ee_feedforward_allocator.mode ==
            PicoEeFeedforwardAllocatorMode::kActive &&
        diagnostics.right.feedforward_allocation.valid) {
      right_input.desired_twist = clampCartesianTwist(
          config_.cartesian_servo,
          right_feedback_twist +
              diagnostics.right.feedforward_allocation.allocated_feedforward);
    }
  }
  if (right_reference_valid && usesPicoEeMappedCorrectedPalm(algorithm_) &&
      config_.pico_ee_task_allocator.mode !=
          PicoEeTaskAllocationMode::kDisabled) {
    diagnostics.right.task_allocation =
        right_task_allocator_->allocate(right_input, right_feedback_twist);
    if (diagnostics.right.task_allocation.authorized) {
      if (config_.pico_ee_task_allocator.nullspace_only) {
        right_input.vector_nullspace_task.alpha_command =
            diagnostics.right.task_allocation
                .predicted_nullspace_alpha_command;
      } else {
        right_input.desired_twist =
            diagnostics.right.task_allocation.predicted_desired_twist;
      }
    }
  }
  diagnostics.right.final_desired_twist = right_input.desired_twist;

  diagnostics.left.safety = config_.controller.model_state_only
                                ? SafetyDecision{true, HoldReason::kNone, -1}
                                : validateActualPosition(
                                      left_q, left_input.limits,
                                      config_.safety.bound_tolerance);
  diagnostics.right.safety = config_.controller.model_state_only
                                 ? SafetyDecision{true, HoldReason::kNone, -1}
                                 : validateActualPosition(
                                       right_q, right_input.limits,
                                       config_.safety.bound_tolerance);
  if (diagnostics.left.safety.accepted) {
    diagnostics.left.safety =
        !left_reference_valid
            ? SafetyDecision{false, HoldReason::kNonFiniteProblem, -1}
            : diagnostics.left.reference_frozen
            ? SafetyDecision{false, HoldReason::kReferenceTrackingError, -1}
            : validateBounds(left_input.bounds);
  }
  if (diagnostics.right.safety.accepted) {
    diagnostics.right.safety =
        !right_reference_valid
            ? SafetyDecision{false, HoldReason::kNonFiniteProblem, -1}
            : diagnostics.right.reference_frozen
            ? SafetyDecision{false, HoldReason::kReferenceTrackingError, -1}
            : validateBounds(right_input.bounds);
  }
  if (diagnostics.left.safety.accepted) {
    diagnostics.left.ik = left_ik_->solve(left_input);
    diagnostics.left.cartesian_execution_residual =
        left_input.desired_twist -
        left_input.jacobian * diagnostics.left.ik.qdot;
    if (diagnostics.left.feedforward_allocation.preview_solve_count > 0) {
      diagnostics.left.feedforward_preview_actual_mismatch_rad_s =
          (diagnostics.left.feedforward_allocation.legacy_preview_qdot -
           diagnostics.left.ik.qdot)
              .cwiseAbs()
              .maxCoeff();
    }
    diagnostics.left.arm_angle_current_rate =
        diagnostics.left.arm_angle.jacobian.dot(diagnostics.left.ik.qdot);
    if (left_input.linear_constraint.active) {
      diagnostics.left.upper_arm_outward.achieved =
          left_input.linear_constraint.jacobian.dot(diagnostics.left.ik.qdot);
      diagnostics.left.upper_arm_outward.residual =
          diagnostics.left.upper_arm_outward.achieved -
          left_input.linear_constraint.lower;
    }
  }
  if (diagnostics.right.safety.accepted) {
    diagnostics.right.ik = right_ik_->solve(right_input);
    diagnostics.right.cartesian_execution_residual =
        right_input.desired_twist -
        right_input.jacobian * diagnostics.right.ik.qdot;
    if (diagnostics.right.feedforward_allocation.preview_solve_count > 0) {
      diagnostics.right.feedforward_preview_actual_mismatch_rad_s =
          (diagnostics.right.feedforward_allocation.legacy_preview_qdot -
           diagnostics.right.ik.qdot)
              .cwiseAbs()
              .maxCoeff();
    }
    diagnostics.right.arm_angle_current_rate =
        diagnostics.right.arm_angle.jacobian.dot(diagnostics.right.ik.qdot);
    if (right_input.linear_constraint.active) {
      diagnostics.right.upper_arm_outward.achieved =
          right_input.linear_constraint.jacobian.dot(diagnostics.right.ik.qdot);
      diagnostics.right.upper_arm_outward.residual =
          diagnostics.right.upper_arm_outward.achieved -
          right_input.linear_constraint.lower;
    }
  }
  const Vec7 left_candidate =
      left_state_.q_ref + dt * diagnostics.left.ik.qdot;
  const Vec7 right_candidate =
      right_state_.q_ref + dt * diagnostics.right.ik.qdot;
  if (diagnostics.left.safety.accepted) {
    diagnostics.left.safety =
        validateArmResult(left_input, diagnostics.left.ik, left_candidate);
  }
  if (diagnostics.right.safety.accepted) {
    diagnostics.right.safety =
        validateArmResult(right_input, diagnostics.right.ik, right_candidate);
  }

  diagnostics.left.accepted = diagnostics.left.safety.accepted;
  diagnostics.right.accepted = diagnostics.right.safety.accepted;
  diagnostics.left.hold_reason = diagnostics.left.safety.reason;
  diagnostics.right.hold_reason = diagnostics.right.safety.reason;

  if (diagnostics.left.accepted) {
    left_state_.qddot_prev =
        (diagnostics.left.ik.qdot - left_state_.qdot_prev) / dt;
    left_state_.q_ref = left_candidate;
    left_state_.qdot_prev = diagnostics.left.ik.qdot;
    left_state_.slack_prev = diagnostics.left.ik.slack;
    robot_.setArmState(ArmSide::kLeft, left_state_.q_ref,
                       left_state_.qdot_prev);
  } else {
    if (!applyBoundedFallback(ArmSide::kLeft, left_input, dt,
                              diagnostics.left)) {
      clearHistory(ArmSide::kLeft);
    }
  }
  if (diagnostics.right.accepted) {
    right_state_.qddot_prev =
        (diagnostics.right.ik.qdot - right_state_.qdot_prev) / dt;
    right_state_.q_ref = right_candidate;
    right_state_.qdot_prev = diagnostics.right.ik.qdot;
    right_state_.slack_prev = diagnostics.right.ik.slack;
    robot_.setArmState(ArmSide::kRight, right_state_.q_ref,
                       right_state_.qdot_prev);
  } else {
    if (!applyBoundedFallback(ArmSide::kRight, right_input, dt,
                              diagnostics.right)) {
      clearHistory(ArmSide::kRight);
    }
  }

  diagnostics.left.q_ref = left_state_.q_ref;
  diagnostics.right.q_ref = right_state_.q_ref;
  diagnostics.accepted = diagnostics.left.accepted && diagnostics.right.accepted;
  diagnostics.hold_reason = !diagnostics.left.accepted
                                ? diagnostics.left.hold_reason
                                : (!diagnostics.right.accepted
                                       ? diagnostics.right.hold_reason
                                       : HoldReason::kNone);
  diagnostics.compute_time_us =
      elapsedMicroseconds(cycle_start, std::chrono::steady_clock::now());
  return diagnostics;
}

void DualArmController::configureDlsPostureTask(
    ArmSide side, const Pose& target, bool stale,
    const ArmAngleTask& arm_angle, double dt, ArmIkInput& input,
    ArmControllerDiagnostics& diagnostics) {
  if (!config_.iterative_dls.posture_reference_enabled ||
      algorithm_ != IkAlgorithm::kHierarchicalQp ||
      arm_angle_reference_mode_ != ArmAngleReferenceMode::kPicoOutward ||
      !std::isfinite(dt) || dt <= 0.0) {
    return;
  }

  ArmReferenceState& arm_state = state(side);
  IterativePoseDlsIk7& dls = side == ArmSide::kLeft
      ? left_dls_posture_
      : right_dls_posture_;
  JointTrajectoryLimiter7& posture_ruckig = side == ArmSide::kLeft
      ? left_dls_posture_ruckig_
      : right_dls_posture_ruckig_;
  if (!arm_state.dls_posture_initialized) {
    resetDlsPosture(side);
  }
  if (!arm_state.dls_posture_initialized) {
    return;
  }

  if (!stale) {
    PoseDlsInput dls_input;
    dls_input.target = target;
    // The working position-level DLS controller always starts from the
    // current command model.  This selects the nearest IK branch instead of
    // allowing an independent posture seed to drift away from the QP state.
    dls_input.seed = arm_state.q_ref;
    dls_input.limits = input.limits;
    dls_input.secondary_task.active =
        config_.arm_angle.enabled && arm_angle.active;
    dls_input.secondary_task.jacobian = arm_angle.jacobian;
    dls_input.secondary_task.target = arm_angle.error_rad;
    dls_input.secondary_task.activation = arm_angle.activation;
    dls_input.evaluate = [this, side](const Vec7& q) {
      return robot_.armKinematicsAt(side, q);
    };
    // Match DLS_IK's nearest-seed branch selection. The published QP command
    // remains protected by its hard outward inequality; backtracking every
    // internal DLS candidate against that inequality made this guide both
    // discontinuous and too expensive for the 5 ms control period.
    const PoseDlsResult dls_result = dls.solve(dls_input);
    diagnostics.dls_posture_status = dls_result.status;
    diagnostics.dls_posture_iterations = dls_result.iterations;
    diagnostics.dls_posture_joint_projection_count =
        dls_result.joint_projection_count;
    diagnostics.dls_posture_solve_time_us = dls_result.solve_time_us;
    diagnostics.dls_posture_initial_position_error_m =
        dls_result.initial_position_error_m;
    diagnostics.dls_posture_initial_orientation_error_rad =
        dls_result.initial_orientation_error_rad;
    diagnostics.dls_posture_final_position_error_m =
        dls_result.position_error_m;
    diagnostics.dls_posture_final_orientation_error_rad =
        dls_result.orientation_error_rad;
    if (dls_result.status != PoseDlsStatus::kRejected &&
        dls_result.q.allFinite()) {
      const Vec7 interior_goal = clampPostureReferenceToInterior(
          dls_result.q, input.limits,
          config_.iterative_dls.posture_reference_margin_rad);
      const Vec7 delta =
          (interior_goal - arm_state.dls_posture_goal)
              .cwiseMax(-config_.iterative_dls.maximum_goal_step_rad)
              .cwiseMin(config_.iterative_dls.maximum_goal_step_rad);
      arm_state.dls_posture_goal += delta;
    }
  }

  JointTrajectoryResult posture_result;
  ArmMotionState posture_state;
  if (config_.dls_posture_ruckig.enabled) {
    posture_result = posture_ruckig.update(arm_state.dls_posture_goal, dt);
    posture_state = posture_result.accepted
        ? posture_result.state
        : posture_ruckig.state();
  } else {
    posture_state.q = arm_state.dls_posture_goal;
    posture_state.qdot.setZero();
    posture_state.qddot.setZero();
    posture_result.detail = "joint_trajectory_bypassed_for_velocity_qp";
  }
  if (!posture_state.q.allFinite() || !posture_state.qdot.allFinite()) {
    return;
  }

  input.posture_task.active = true;
  input.posture_task.target =
      posture_state.qdot + config_.hierarchical_qp.nominal_gain *
          (posture_state.q - arm_state.q_ref);
  input.posture_task.activation = 1.0;
  // The DLS posture already contains the PICO arm-angle objective.  Running
  // the raw scalar refinement afterwards would overwrite the smooth Ruckig
  // redundancy velocity and can drive the model back into a joint boundary.
  input.arm_angle_task.active = false;
  diagnostics.arm_angle_task_active = false;
  diagnostics.arm_angle_requested_rate = 0.0;
  diagnostics.dls_posture_reference_active = true;
  diagnostics.dls_posture_goal = arm_state.dls_posture_goal;
  diagnostics.dls_posture_reference = posture_state.q;
  diagnostics.dls_posture_velocity_reference = posture_state.qdot;
  diagnostics.dls_posture_velocity_target = input.posture_task.target;
  diagnostics.dls_posture_ruckig_accepted = posture_result.accepted;
  diagnostics.dls_posture_ruckig_detail = posture_result.detail;
  const Vec7 safe_lower = input.limits.lower_position.array() +
                          config_.joint_limits.margin_rad;
  const Vec7 safe_upper = input.limits.upper_position.array() -
                          config_.joint_limits.margin_rad;
  diagnostics.dls_posture_goal_limit_margin_rad = std::min(
      (arm_state.dls_posture_goal - safe_lower).minCoeff(),
      (safe_upper - arm_state.dls_posture_goal).minCoeff());
}

void DualArmController::configureMappedArmAngleVectorTask(
    ArmSide side, bool stale, double dt, ArmIkInput& input,
    ArmControllerDiagnostics& diagnostics) {
  const PicoMappedArmAngleVectorConfig& vector_config =
      config_.pico_mapped_arm_angle_vector_nullspace;
  if (!usesPicoEeMappedCorrectedPalm(algorithm_) ||
      vector_config.mode == PicoMappedArmAngleVectorMode::kDisabled) {
    return;
  }

  PicoMappedArmAngleVectorInput vector_input;
  vector_input.cartesian_jacobian = input.jacobian;
  vector_input.arm_angle_jacobian = input.arm_angle_task.jacobian;
  vector_input.desired_twist = input.desired_twist;
  vector_input.qdot_previous = input.qdot_prev;
  vector_input.qddot_previous = input.qddot_prev;
  vector_input.lower_velocity_bound = input.bounds.lower;
  vector_input.upper_velocity_bound = input.bounds.upper;
  vector_input.requested_arm_rate = input.arm_angle_task.target;
  // This scale belongs only to the local feasibility model. The real QP keeps
  // its configured beta lower bounds and all hard constraints. Allowing the
  // predictor to reach zero avoids switching back to the legacy scalar task
  // merely because the instantaneous exact Cartesian particular is outside
  // acceleration/jerk bounds; low predicted scale proportionally removes
  // PICO intent authority below.
  vector_input.minimum_predicted_task_scale = 0.0;
  vector_input.task_health = std::clamp(
      diagnostics.reference_scale * input.arm_angle_task.activation,
      0.0, 1.0);
  vector_input.dt = dt;
  vector_input.active = input.arm_angle_task.active;
  vector_input.stale = stale;

  PicoMappedArmAngleVectorGovernor7& governor =
      side == ArmSide::kLeft ? left_mapped_vector_governor_
                             : right_mapped_vector_governor_;
  diagnostics.mapped_vector_nullspace = governor.update(vector_input);
  if (vector_config.mode != PicoMappedArmAngleVectorMode::kActive ||
      !diagnostics.mapped_vector_nullspace.valid) {
    return;
  }

  const PicoMappedArmAngleVectorState& state =
      diagnostics.mapped_vector_nullspace;
  input.vector_nullspace_task.active = true;
  input.vector_nullspace_task.basis = state.basis;
  input.vector_nullspace_task.alpha_command = state.alpha_command;
  input.vector_nullspace_task.alpha_previous = state.alpha_previous;
  input.vector_nullspace_task.alpha_trend = state.alpha_trend;
  input.vector_nullspace_task.reference_weight =
      vector_config.qp_weight_mode == PicoMappedVectorQpWeightMode::kLegacy
          ? state.command_tracking_weight
          : vector_config.qp_tracking_weight * state.health *
                diagnostics.redundancy_authority;
  input.vector_nullspace_task.continuity_weight =
      vector_config.qp_continuity_weight;
  input.vector_nullspace_task.jerk_weight =
      vector_config.qp_jerk_weight;
  input.arm_angle_task.active = false;
  diagnostics.arm_angle_task_active = false;
}

void DualArmController::resetDlsPosture(ArmSide side) {
  ArmReferenceState& arm_state = state(side);
  arm_state.dls_posture_goal = arm_state.q_ref;
  JointTrajectoryLimiter7& posture_ruckig = side == ArmSide::kLeft
      ? left_dls_posture_ruckig_
      : right_dls_posture_ruckig_;
  arm_state.dls_posture_initialized = posture_ruckig.reset(
      {arm_state.q_ref, Vec7::Zero(), Vec7::Zero()});
}

void DualArmController::resetSolvers() {
  left_ik_->reset();
  right_ik_->reset();
  left_ff_allocator_->reset();
  right_ff_allocator_->reset();
  left_task_allocator_->reset();
  right_task_allocator_->reset();
  left_state_.slack_prev.setZero();
  right_state_.slack_prev.setZero();
  left_mapped_vector_governor_.reset();
  right_mapped_vector_governor_.reset();
}

bool DualArmController::synchronizeReferencesToActual() {
  if (config_.controller.model_state_only) {
    clearHistory();
    left_ik_->reset();
    right_ik_->reset();
    left_arm_angle_.reset();
    right_arm_angle_.reset();
    resetDlsPosture(ArmSide::kLeft);
    resetDlsPosture(ArmSide::kRight);
    return true;
  }
  const Vec7 left_actual = robot_.armPosition(ArmSide::kLeft);
  const Vec7 right_actual = robot_.armPosition(ArmSide::kRight);
  const SafetyDecision left_safety = validateActualPosition(
      left_actual, robot_.mapping(ArmSide::kLeft).limits,
      config_.safety.bound_tolerance);
  const SafetyDecision right_safety = validateActualPosition(
      right_actual, robot_.mapping(ArmSide::kRight).limits,
      config_.safety.bound_tolerance);
  if (!left_safety.accepted || !right_safety.accepted) {
    return false;
  }
  left_state_.q_ref = left_actual;
  right_state_.q_ref = right_actual;
  clearHistory();
  left_ik_->reset();
  right_ik_->reset();
  left_arm_angle_.reset();
  right_arm_angle_.reset();
  resetDlsPosture(ArmSide::kLeft);
  resetDlsPosture(ArmSide::kRight);
  return true;
}

void DualArmController::setArmAngleReferenceMode(
    ArmAngleReferenceMode mode) noexcept {
  if (mode == arm_angle_reference_mode_) {
    return;
  }
  arm_angle_reference_mode_ = mode;
  left_arm_angle_.reset();
  right_arm_angle_.reset();
  left_mapped_vector_governor_.reset();
  right_mapped_vector_governor_.reset();
  left_ff_allocator_->reset();
  right_ff_allocator_->reset();
  left_task_allocator_->reset();
  right_task_allocator_->reset();
  resetDlsPosture(ArmSide::kLeft);
  resetDlsPosture(ArmSide::kRight);
}

void DualArmController::setAlgorithm(IkAlgorithm algorithm) {
  if (algorithm == algorithm_) {
    resetSolvers();
    return;
  }
  left_ik_ = makeArmVelocityIk(algorithm, config_);
  right_ik_ = makeArmVelocityIk(algorithm, config_);
  left_ff_allocator_ = std::make_unique<CartesianFeedforwardAllocator7>(
      config_.pico_ee_feedforward_allocator, config_.joint_limits,
      config_.cartesian_servo, makeArmVelocityIk(algorithm, config_));
  right_ff_allocator_ = std::make_unique<CartesianFeedforwardAllocator7>(
      config_.pico_ee_feedforward_allocator, config_.joint_limits,
      config_.cartesian_servo, makeArmVelocityIk(algorithm, config_));
  left_task_allocator_ = std::make_unique<CartesianTaskAllocator7>(
      config_.pico_ee_task_allocator, config_.joint_limits,
      config_.cartesian_servo, makeArmVelocityIk(algorithm, config_));
  right_task_allocator_ = std::make_unique<CartesianTaskAllocator7>(
      config_.pico_ee_task_allocator, config_.joint_limits,
      config_.cartesian_servo, makeArmVelocityIk(algorithm, config_));
  algorithm_ = algorithm;
  left_state_.slack_prev.setZero();
  right_state_.slack_prev.setZero();
  left_mapped_vector_governor_.reset();
  right_mapped_vector_governor_.reset();
  resetDlsPosture(ArmSide::kLeft);
  resetDlsPosture(ArmSide::kRight);
}

const Vec7& DualArmController::reference(ArmSide side) const noexcept {
  return state(side).q_ref;
}

const Vec7& DualArmController::previousVelocity(ArmSide side) const noexcept {
  return state(side).qdot_prev;
}

const Vec7& DualArmController::previousAcceleration(
    ArmSide side) const noexcept {
  return state(side).qddot_prev;
}

ArmMotionState DualArmController::referenceState(
    ArmSide side) const noexcept {
  const ArmReferenceState& arm_state = state(side);
  return {arm_state.q_ref, arm_state.qdot_prev, arm_state.qddot_prev};
}

bool DualArmController::setReferenceState(
    ArmSide side, const ArmMotionState& motion) {
  const ArmLimits& limits = robot_.mapping(side).limits;
  if (!motion.q.allFinite() || !motion.qdot.allFinite() ||
      !motion.qddot.allFinite()) {
    return false;
  }
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (motion.q[joint] <
            limits.lower_position[joint] - config_.safety.bound_tolerance ||
        motion.q[joint] >
            limits.upper_position[joint] + config_.safety.bound_tolerance ||
        std::abs(motion.qdot[joint]) >
            limits.velocity[joint] + config_.safety.bound_tolerance ||
        std::abs(motion.qddot[joint]) >
            config_.joint_limits.max_acceleration_rad_s2[joint] +
                config_.safety.bound_tolerance) {
      return false;
    }
  }
  ArmReferenceState& arm_state = state(side);
  arm_state.q_ref = motion.q;
  arm_state.qdot_prev = motion.qdot;
  arm_state.qddot_prev = motion.qddot;
  arm_state.slack_prev.setZero();
  resetDlsPosture(side);
  IArmVelocityIk& ik = side == ArmSide::kLeft ? *left_ik_ : *right_ik_;
  ik.reset();
  (side == ArmSide::kLeft ? left_ff_allocator_ : right_ff_allocator_)->reset();
  (side == ArmSide::kLeft ? left_task_allocator_ : right_task_allocator_)
      ->reset();
  robot_.setArmState(side, motion.q, motion.qdot);
  return true;
}

ArmReferenceState& DualArmController::state(ArmSide side) noexcept {
  return side == ArmSide::kLeft ? left_state_ : right_state_;
}

const ArmReferenceState& DualArmController::state(ArmSide side) const noexcept {
  return side == ArmSide::kLeft ? left_state_ : right_state_;
}

SafetyDecision DualArmController::validateArmResult(
    const ArmIkInput& input, const ArmIkResult& result,
    const Vec7& candidate) const {
  if (result.status != SolverStatus::kSolved) {
    return {false, HoldReason::kSolverFailure, -1};
  }
  if (!result.qdot.allFinite() || !result.slack.allFinite() ||
      !std::isfinite(result.equality_residual) || !candidate.allFinite()) {
    return {false, HoldReason::kNonFiniteSolution, -1};
  }
  for (int index = 0; index < kArmDof; ++index) {
    if (result.qdot[index] <
            input.bounds.lower[index] - config_.safety.bound_tolerance ||
        result.qdot[index] >
            input.bounds.upper[index] + config_.safety.bound_tolerance) {
      return {false, HoldReason::kBoundViolation, index};
    }
    const double safe_lower =
        input.limits.lower_position[index] + config_.joint_limits.margin_rad;
    const double safe_upper =
        input.limits.upper_position[index] - config_.joint_limits.margin_rad;
    if (candidate[index] < safe_lower - config_.safety.bound_tolerance ||
        candidate[index] > safe_upper + config_.safety.bound_tolerance) {
      return {false, HoldReason::kBoundViolation, index};
    }
  }
  const auto constraint_satisfied = [this, &result](
      const LinearJointConstraint& constraint) {
    if (!constraint.active) {
      return true;
    }
    const double value = constraint.jacobian.dot(result.qdot);
    return std::isfinite(value) &&
           value >= constraint.lower - config_.safety.bound_tolerance &&
           value <= constraint.upper + config_.safety.bound_tolerance;
  };
  if (!constraint_satisfied(input.linear_constraint) ||
      !constraint_satisfied(input.branch_lock_constraint)) {
    return {false, HoldReason::kBoundViolation, -1};
  }
  Vec6 scaled_desired_twist = input.desired_twist;
  if (config_.hierarchical_qp.task_scaling_enabled) {
    if (!std::isfinite(result.task_scale_position) ||
        !std::isfinite(result.task_scale_orientation) ||
        result.task_scale_position <
            config_.hierarchical_qp.task_scaling_min_position -
                config_.safety.bound_tolerance ||
        result.task_scale_position >
            1.0 + config_.safety.bound_tolerance ||
        result.task_scale_orientation <
            config_.hierarchical_qp.task_scaling_min_orientation -
                config_.safety.bound_tolerance ||
        result.task_scale_orientation >
            1.0 + config_.safety.bound_tolerance) {
      return {false, HoldReason::kBoundViolation, -1};
    }
    scaled_desired_twist.head<3>() *= result.task_scale_position;
    scaled_desired_twist.tail<3>() *= result.task_scale_orientation;
  }
  const double equality_residual =
      (input.jacobian * result.qdot + result.slack -
       scaled_desired_twist).norm();
  if (!std::isfinite(equality_residual) ||
      equality_residual > config_.hierarchical_qp.equality_tolerance) {
    return {false, HoldReason::kEqualityViolation, -1};
  }
  return {true, HoldReason::kNone, -1};
}

bool DualArmController::applyBoundedFallback(
    ArmSide side, const ArmIkInput& input, double dt,
    ArmControllerDiagnostics& diagnostics) {
  if (diagnostics.hold_reason != HoldReason::kSolverFailure ||
      !std::isfinite(dt) || dt <= 0.0) {
    return false;
  }
  ArmReferenceState& arm_state = state(side);
  const Vec7 requested = arm_state.qdot_prev + arm_state.qddot_prev * dt;
  const Vec7 fallback = requested.cwiseMax(input.bounds.lower)
                            .cwiseMin(input.bounds.upper);
  const Vec7 candidate = arm_state.q_ref + fallback * dt;
  if (!fallback.allFinite() || !candidate.allFinite()) {
    return false;
  }
  for (int joint = 0; joint < kArmDof; ++joint) {
    const double safe_lower =
        input.limits.lower_position[joint] + config_.joint_limits.margin_rad;
    const double safe_upper =
        input.limits.upper_position[joint] - config_.joint_limits.margin_rad;
    if (candidate[joint] <
            safe_lower - config_.safety.bound_tolerance ||
        candidate[joint] >
            safe_upper + config_.safety.bound_tolerance) {
      return false;
    }
  }

  diagnostics.ik.qdot = fallback;
  diagnostics.ik.slack =
      input.desired_twist - input.jacobian * fallback;
  diagnostics.ik.equality_residual = 0.0;
  diagnostics.fallback_applied = true;
  arm_state.qddot_prev = (fallback - arm_state.qdot_prev) / dt;
  arm_state.qdot_prev = fallback;
  arm_state.q_ref = candidate;
  arm_state.slack_prev = diagnostics.ik.slack;
  IArmVelocityIk& ik = side == ArmSide::kLeft ? *left_ik_ : *right_ik_;
  ik.reset();
  (side == ArmSide::kLeft ? left_ff_allocator_ : right_ff_allocator_)->reset();
  (side == ArmSide::kLeft ? left_task_allocator_ : right_task_allocator_)
      ->reset();
  robot_.setArmState(side, arm_state.q_ref, arm_state.qdot_prev);
  return true;
}

void DualArmController::clearHistory() {
  clearHistory(ArmSide::kLeft);
  clearHistory(ArmSide::kRight);
}

void DualArmController::clearHistory(ArmSide side) {
  ArmReferenceState& arm_state = state(side);
  arm_state.qdot_prev.setZero();
  arm_state.qddot_prev.setZero();
  arm_state.slack_prev.setZero();
  (side == ArmSide::kLeft ? left_mapped_vector_governor_
                          : right_mapped_vector_governor_)
      .reset();
  (side == ArmSide::kLeft ? left_motion_confidence_
                          : right_motion_confidence_)
      .reset();
  (side == ArmSide::kLeft ? left_ff_allocator_ : right_ff_allocator_)->reset();
  resetDlsPosture(side);
  if (arm_state.q_ref.allFinite()) {
    robot_.setArmState(side, arm_state.q_ref, Vec7::Zero());
  }
}

bool DualArmController::targetsAreFinite(const DualArmTargets& targets) const {
  return targets.left.position.allFinite() &&
         targets.left.rotation.allFinite() &&
         targets.left_twist.allFinite() &&
         targets.right.position.allFinite() &&
         targets.right.rotation.allFinite() &&
         targets.right_twist.allFinite() &&
         isProperRotation(targets.left.rotation) &&
         isProperRotation(targets.right.rotation);
}

bool DualArmController::referenceIsFinite(
    const CartesianReference& reference) const {
  return reference.valid && reference.pose.position.allFinite() &&
         reference.pose.rotation.allFinite() && reference.twist.allFinite() &&
         reference.acceleration.allFinite() &&
         isProperRotation(reference.pose.rotation);
}

std::unique_ptr<IQpSolver7> makeSolver(SolverBackend backend,
                                       const QpIkConfig& config) {
  if (backend == SolverBackend::kOsqp) {
    return std::make_unique<OsqpSolver7>(config.osqp);
  }
  return std::make_unique<QpoasesSolver7>(config.qpoases);
}

}  // namespace tianji_mapped_palm
