#include "tianji_qp_ik/spark_guidance.hpp"

#include "tianji_qp_ik/spark_qpoases_diagnostic.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <utility>

namespace tianji_qp_ik {
namespace {

std::array<Pose, 2> tcpRelativeFrames(const MujocoRobot& robot) {
  return {robot.tcpRelativeToLink7(ArmSide::kLeft),
          robot.tcpRelativeToLink7(ArmSide::kRight)};
}

SparkUpperRobotGeometry geometryFromKinematics(
    PinocchioArmKinematics& kinematics) {
  const ArmKinematicSample left =
      kinematics.sample(ArmSide::kLeft, Vec7::Zero());
  const ArmKinematicSample right =
      kinematics.sample(ArmSide::kRight, Vec7::Zero());
  SparkUpperRobotGeometry geometry;
  geometry.left_shoulder = left.shoulder_position;
  geometry.right_shoulder = right.shoulder_position;
  geometry.left_upper_arm_local = left.shoulder_rotation.transpose() *
      (left.elbow_position - left.shoulder_position);
  geometry.left_forearm_local = left.elbow_rotation.transpose() *
      (left.wrist_position - left.elbow_position);
  geometry.left_wrist_to_palm_local = left.wrist_rotation.transpose() *
      (left.tcp_pose.position - left.wrist_position);
  geometry.right_upper_arm_local = right.shoulder_rotation.transpose() *
      (right.elbow_position - right.shoulder_position);
  geometry.right_forearm_local = right.elbow_rotation.transpose() *
      (right.wrist_position - right.elbow_position);
  geometry.right_wrist_to_palm_local = right.wrist_rotation.transpose() *
      (right.tcp_pose.position - right.wrist_position);
  return geometry;
}

SparkPalmTwistEstimatorConfig palmTwistConfig(const QpIkConfig& config) {
  SparkPalmTwistEstimatorConfig result;
  result.filter_alpha =
      config.spark_feedforward_velocity_qp.palm_twist_filter_alpha;
  result.dt_median_window =
      config.spark_feedforward_velocity_qp.dt_median_window;
  result.dt_min_ratio = config.spark_feedforward_velocity_qp.dt_min_ratio;
  result.dt_max_ratio = config.spark_feedforward_velocity_qp.dt_max_ratio;
  result.stationary_decay =
      config.spark_feedforward_velocity_qp.velocity_stationary_decay;
  result.reversal_decay =
      config.spark_feedforward_velocity_qp.velocity_reversal_decay;
  result.maximum_linear_velocity_m_s =
      config.cartesian_servo.max_linear_velocity;
  result.maximum_angular_velocity_rad_s =
      config.cartesian_servo.max_angular_velocity;
  result.lowpass_cutoff_hz =
      config.spark_feedforward_velocity_qp.palm_twist_lowpass_cutoff_hz;
  return result;
}

double smoothstep5(double value) {
  const double u = std::clamp(value, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

Eigen::Vector3d blendVector(const Eigen::Vector3d& first,
                            const Eigen::Vector3d& second, double alpha) {
  return (1.0 - alpha) * first + alpha * second;
}

Eigen::Vector3d blendDirection(const Eigen::Vector3d& first,
                               const Eigen::Vector3d& second, double alpha) {
  const Eigen::Vector3d blended =
      blendVector(first.normalized(), second.normalized(), alpha);
  return blended.norm() > 1.0e-9 ? blended.normalized() : second.normalized();
}

SparkUpperArmTarget blendTarget(const SparkUpperArmTarget& start,
                                const SparkUpperArmTarget& goal,
                                double alpha) {
  SparkUpperArmTarget result = goal;
  result.shoulder = blendVector(start.shoulder, goal.shoulder, alpha);
  const auto append_segment = [alpha](
                                  const Eigen::Vector3d& start_segment,
                                  const Eigen::Vector3d& goal_segment)
      -> Eigen::Vector3d {
    const double length = (1.0 - alpha) * start_segment.norm() +
                          alpha * goal_segment.norm();
    return (length * blendDirection(start_segment, goal_segment, alpha))
        .eval();
  };
  result.elbow = result.shoulder +
      append_segment(start.elbow - start.shoulder,
                     goal.elbow - goal.shoulder);
  result.wrist = result.elbow +
      append_segment(start.wrist - start.elbow, goal.wrist - goal.elbow);
  result.hand = result.wrist +
      append_segment(start.hand - start.wrist, goal.hand - goal.wrist);
  result.palm.position = result.hand;
  Eigen::Quaterniond start_rotation(start.palm.rotation);
  Eigen::Quaterniond goal_rotation(goal.palm.rotation);
  if (start_rotation.dot(goal_rotation) < 0.0) {
    goal_rotation.coeffs() *= -1.0;
  }
  result.palm.rotation = start_rotation.slerp(alpha, goal_rotation)
                             .normalized()
                             .toRotationMatrix();
  return result;
}

SparkUpperArmTarget targetFromSample(const ArmKinematicSample& sample,
                                     const SparkUpperArmTarget& metadata) {
  SparkUpperArmTarget target = metadata;
  target.palm = sample.tcp_pose;
  target.shoulder = sample.shoulder_position;
  target.elbow = sample.elbow_position;
  target.wrist = sample.wrist_position;
  target.hand = sample.tcp_pose.position;
  return target;
}

}  // namespace

DualArmSparkGuidance::ArmState::ArmState(
    ArmSide side, PinocchioArmKinematics& kinematics,
    const QpIkConfig& config, const ArmLimits& limits, double initial_dt)
    : ik(side, kinematics, limits, config.spark_upper_qpoases,
         config.qpoases),
      reference(config.dls_posture_ruckig, limits, initial_dt,
                config.spark_upper_qpoases.posture_position_gain),
      otg(config.cartesian_otg, initial_dt),
      feedforward(
          config.spark_feedforward_velocity_qp, limits,
          config.joint_limits, initial_dt),
      palm_twist(palmTwistConfig(config)),
      motion_intent_twist(palmTwistConfig(config)),
      headroom(config.spark_headroom_feedforward_velocity_qp, limits,
               config.joint_acceleration_limits),
      arm_limits(limits) {}

DualArmSparkGuidance::DualArmSparkGuidance(
    MujocoRobot& robot, const QpIkConfig& config, const std::string& urdf_path,
    SparkPostureGuideMode posture_mode)
    : robot_(robot),
      config_(config),
      posture_mode_(posture_mode),
      kinematics_(urdf_path, tcpRelativeFrames(robot)),
      scaler_(geometryFromKinematics(kinematics_),
              config_.spark_upper_qpoases),
      left_(ArmSide::kLeft, kinematics_, config_,
            robot.mapping(ArmSide::kLeft).limits,
            1.0 / config_.controller.rate_hz),
      right_(ArmSide::kRight, kinematics_, config_,
             robot.mapping(ArmSide::kRight).limits,
             1.0 / config_.controller.rate_hz) {
  ArmMotionState left_model;
  left_model.q = robot.armPosition(ArmSide::kLeft);
  ArmMotionState right_model;
  right_model.q = robot.armPosition(ArmSide::kRight);
  if (!reset(left_model, right_model)) {
    throw std::invalid_argument("cannot initialize Spark guidance state");
  }
}

bool DualArmSparkGuidance::reset(const ArmMotionState& left_model,
                                 const ArmMotionState& right_model) noexcept {
  scaler_.reset();
  left_.ik.reset();
  right_.ik.reset();
  raw_targets_ = {};
  blend_start_targets_ = {};
  latest_targets_ = {};
  held_targets_ = {};
  blend_restart_pending_ = false;
  blend_elapsed_seconds_ = 0.0;
  joint_reference_phase_ = 0.0;
  last_tracking_epoch_ = 0U;
  latest_source_sequence_ = 0U;
  latest_source_timestamp_ns_ = 0;
  latest_source_epoch_ = 0U;
  latest_source_discontinuity_ = false;
  latest_left_input_palm_ = {};
  latest_right_input_palm_ = {};
  last_feedforward_sequence_ = 0U;
  last_palm_twist_sequence_ = 0U;
  left_.last_valid_q_ik = left_model.q;
  right_.last_valid_q_ik = right_model.q;
  left_.otg.reset(
      kinematics_.sample(ArmSide::kLeft, left_model.q).tcp_pose);
  right_.otg.reset(
      kinematics_.sample(ArmSide::kRight, right_model.q).tcp_pose);
  left_.feedforward.reset(left_model, 0U);
  right_.feedforward.reset(right_model, 0U);
  left_.palm_twist.reset();
  right_.palm_twist.reset();
  left_.motion_intent_twist.reset();
  right_.motion_intent_twist.reset();
  left_.headroom.reset();
  right_.headroom.reset();
  left_.last_feedforward_target = {};
  right_.last_feedforward_target = {};
  left_.settled_hold_active = false;
  right_.settled_hold_active = false;
  left_.settled_hold_dwell_seconds = 0.0;
  right_.settled_hold_dwell_seconds = 0.0;
  left_.settled_hold_reason = SparkSettledHoldReason::kNone;
  right_.settled_hold_reason = SparkSettledHoldReason::kNone;
  return left_.reference.reset(left_model) &&
         right_.reference.reset(right_model);
}

void DualArmSparkGuidance::updateHeadroomFeedback(
    const SparkConstraintHeadroomFeedback& left,
    const SparkConstraintHeadroomFeedback& right, double dt) noexcept {
  if (posture_mode_ !=
      SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity) {
    return;
  }
  (void)left_.headroom.update(left, dt);
  (void)right_.headroom.update(right, dt);
}

SparkUpperTargets DualArmSparkGuidance::updatePicoFrame(
    const PicoTeleopFrame& frame) {
  const bool previously_valid = raw_targets_.valid;
  raw_targets_ = scaler_.update(frame.upper_limb_skeleton, frame.left,
                                frame.right);
  const bool epoch_changed = last_tracking_epoch_ != 0U &&
                             last_tracking_epoch_ != frame.tracking_epoch;
  last_tracking_epoch_ = frame.tracking_epoch;
  latest_source_sequence_ = frame.sequence;
  latest_source_timestamp_ns_ = frame.source_timestamp_ns;
  latest_source_epoch_ = frame.tracking_epoch;
  latest_source_discontinuity_ = frame.stream_discontinuity;
  if (raw_targets_.valid) {
    latest_left_input_palm_ = frame.left;
    latest_right_input_palm_ = frame.right;
  }
  if (raw_targets_.valid &&
      (!previously_valid || epoch_changed || frame.stream_discontinuity)) {
    blend_restart_pending_ = true;
  }
  return raw_targets_;
}

bool DualArmSparkGuidance::startJointSpaceTakeover(
    const SparkUpperTargets& targets, const ArmMotionState& left_model,
    const ArmMotionState& right_model) {
  if (joint_takeover_active_ || !targets.valid || !left_model.q.allFinite() ||
      !right_model.q.allFinite()) {
    return false;
  }

  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              config_.spark_upper_qpoases.ik_cycle_budget_seconds));
  const SparkUpperIkResult left_goal =
      left_.ik.solve(targets.left, left_model.q, deadline);
  const SparkUpperIkResult right_goal =
      right_.ik.solve(targets.right, right_model.q, deadline);
  if (!left_goal.accepted || !right_goal.accepted) {
    return false;
  }

  ArmMotionState left_start = left_model;
  ArmMotionState right_start = right_model;
  left_start.qdot.setZero();
  left_start.qddot.setZero();
  right_start.qdot.setZero();
  right_start.qddot.setZero();
  if (!left_.reference.reset(left_start) ||
      !right_.reference.reset(right_start)) {
    return false;
  }

  joint_takeover_left_goal_ = left_goal.q;
  joint_takeover_right_goal_ = right_goal.q;
  left_.last_valid_q_ik = left_goal.q;
  right_.last_valid_q_ik = right_goal.q;
  joint_takeover_active_ = true;
  return true;
}

SparkGuidanceDiagnostics DualArmSparkGuidance::stepJointSpaceTakeover(
    const ArmMotionState& left_model, const ArmMotionState& right_model,
    double dt) {
  const auto start = std::chrono::steady_clock::now();
  SparkGuidanceDiagnostics result;
  result.joint_takeover_active = joint_takeover_active_;
  if (!joint_takeover_active_ || !left_model.q.allFinite() ||
      !right_model.q.allFinite() || !std::isfinite(dt) || dt <= 0.0) {
    result.detail = "invalid_joint_takeover_input";
    return result;
  }

  const SparkPostureReferenceResult left_reference = left_.reference.update(
      joint_takeover_left_goal_, left_model.q, dt);
  const SparkPostureReferenceResult right_reference = right_.reference.update(
      joint_takeover_right_goal_, right_model.q, dt);
  if (!left_reference.accepted || !right_reference.accepted) {
    result.detail = !left_reference.accepted ? left_reference.detail
                                             : right_reference.detail;
    joint_takeover_active_ = false;
    return result;
  }

  const ArmKinematicSample left_trajectory = robot_.armKinematicsAt(
      ArmSide::kLeft, left_reference.state.q);
  const ArmKinematicSample right_trajectory = robot_.armKinematicsAt(
      ArmSide::kRight, right_reference.state.q);
  const ArmKinematicSample left_current =
      robot_.armKinematicsAt(ArmSide::kLeft, left_model.q);
  const ArmKinematicSample right_current =
      robot_.armKinematicsAt(ArmSide::kRight, right_model.q);

  result.left.reference = left_reference;
  result.right.reference = right_reference;
  result.left.q_ik = joint_takeover_left_goal_;
  result.right.q_ik = joint_takeover_right_goal_;
  const auto fill_takeover_ik = [](const SparkUpperArmTarget& target,
                                  const ArmKinematicSample& sample,
                                  const Vec7& goal,
                                  SparkUpperIkResult& ik) {
    const Vec6 error = poseErrorWorld(target.palm, sample.tcp_pose);
    ik.accepted = true;
    ik.status = SolverStatus::kSolved;
    ik.q = goal;
    ik.stage1_q = goal;
    ik.palm_position_error = error.head<3>().norm();
    ik.palm_orientation_error = error.tail<3>().norm();
    ik.detail = "spark_joint_takeover_trajectory";
  };
  fill_takeover_ik(raw_targets_.left, left_trajectory,
                   joint_takeover_left_goal_, result.left.ik);
  fill_takeover_ik(raw_targets_.right, right_trajectory,
                   joint_takeover_right_goal_, result.right.ik);
  result.left.target = targetFromSample(left_trajectory, raw_targets_.left);
  result.right.target = targetFromSample(right_trajectory, raw_targets_.right);
  result.cartesian_targets.left = left_trajectory.tcp_pose;
  result.cartesian_targets.right = right_trajectory.tcp_pose;
  result.cartesian_targets.left_twist =
      left_trajectory.tcp_jacobian * left_reference.state.qdot;
  result.cartesian_targets.right_twist =
      right_trajectory.tcp_jacobian * right_reference.state.qdot;
  result.cartesian_references.left.pose = left_current.tcp_pose;
  result.cartesian_references.right.pose = right_current.tcp_pose;
  result.cartesian_references.left.twist.setZero();
  result.cartesian_references.right.twist.setZero();
  result.cartesian_references.left.acceleration.setZero();
  result.cartesian_references.right.acceleration.setZero();
  result.cartesian_references.left.stale = false;
  result.cartesian_references.right.stale = false;
  result.cartesian_references.left.valid = true;
  result.cartesian_references.right.valid = true;
  result.cartesian_targets.left_stale = false;
  result.cartesian_targets.right_stale = false;

  const auto fill_posture = [this](const SparkPostureReferenceResult& reference,
                                   JointVelocityPostureTask& task) {
    task.active = true;
    task.source = JointVelocityPostureSource::kSparkJointReference;
    task.target = reference.posture_velocity;
    task.activation = 1.0;
    task.weight = config_.spark_upper_qpoases.joint_reference_weight;
    task.smoothness_weight =
        config_.spark_upper_qpoases.joint_reference_smoothness_weight;
    task.jerk_smoothness_weight = 0.0;
  };
  fill_posture(left_reference, result.posture_tasks.left);
  fill_posture(right_reference, result.posture_tasks.right);

  result.left.accepted = true;
  result.right.accepted = true;
  result.accepted = true;
  result.target_valid = true;
  result.cartesian_references_valid = true;
  result.compute_time_us = std::chrono::duration<double, std::micro>(
                               std::chrono::steady_clock::now() - start)
                               .count();
  result.joint_takeover_finished =
      left_reference.detail == "joint_trajectory_finished" &&
      right_reference.detail == "joint_trajectory_finished";
  result.detail = result.joint_takeover_finished
                      ? "spark_joint_takeover_finished"
                      : "spark_joint_takeover_active";
  if (result.joint_takeover_finished) {
    joint_takeover_active_ = false;
    latest_targets_ = raw_targets_;
    blend_start_targets_ = raw_targets_;
    blend_elapsed_seconds_ = config_.spark_upper_qpoases.target_blend_seconds;
    blend_restart_pending_ = false;
  }
  result.joint_takeover_active = !result.joint_takeover_finished;
  return result;
}

void DualArmSparkGuidance::cancelJointSpaceTakeover() noexcept {
  joint_takeover_active_ = false;
}

void DualArmSparkGuidance::restartBlend(
    const ArmMotionState& left_model, const ArmMotionState& right_model) {
  blend_start_targets_ = raw_targets_;
  blend_start_targets_.left = targetFromSample(
      kinematics_.sample(ArmSide::kLeft, left_model.q), raw_targets_.left);
  blend_start_targets_.right = targetFromSample(
      kinematics_.sample(ArmSide::kRight, right_model.q), raw_targets_.right);
  blend_start_targets_.valid = true;
  blend_elapsed_seconds_ = 0.0;
  blend_restart_pending_ = false;
}

void DualArmSparkGuidance::updateBlend(
    const ArmMotionState& left_model, const ArmMotionState& right_model,
    double dt) {
  if (!raw_targets_.valid) {
    latest_targets_.valid = false;
    latest_targets_.detail = raw_targets_.detail;
    return;
  }
  if (blend_restart_pending_ || !blend_start_targets_.valid) {
    restartBlend(left_model, right_model);
  }
  blend_elapsed_seconds_ = std::min(
      config_.spark_upper_qpoases.target_blend_seconds,
      blend_elapsed_seconds_ + dt);
  const double progress = smoothstep5(
      blend_elapsed_seconds_ /
      config_.spark_upper_qpoases.target_blend_seconds);
  latest_targets_ = raw_targets_;
  latest_targets_.left =
      blendTarget(blend_start_targets_.left, raw_targets_.left, progress);
  latest_targets_.right =
      blendTarget(blend_start_targets_.right, raw_targets_.right, progress);
  latest_targets_.valid = true;
  latest_targets_.detail = progress < 1.0 ? "spark_guidance_blending"
                                          : "spark_guidance_valid";
}

SparkGuidanceDiagnostics DualArmSparkGuidance::step(
    const ArmMotionState& left_model, const ArmMotionState& right_model,
    double dt) {
  const auto start = std::chrono::steady_clock::now();
  SparkGuidanceDiagnostics result;
  if (!std::isfinite(dt) || dt <= 0.0 || !left_model.q.allFinite() ||
      !right_model.q.allFinite()) {
    result.detail = "invalid_spark_guidance_input";
    return result;
  }
  updateBlend(left_model, right_model, dt);
  const bool otg_consistent_mode =
      posture_mode_ ==
      SparkPostureGuideMode::kOtgConsistentJointReferenceVelocity;
  const bool feedforward_mode =
      posture_mode_ ==
          SparkPostureGuideMode::kFeedforwardJointReferenceVelocity ||
      posture_mode_ ==
          SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity;
  const bool headroom_feedforward_mode =
      posture_mode_ ==
      SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity;
  const bool joint_reference_mode =
      posture_mode_ == SparkPostureGuideMode::kJointReferenceVelocity ||
      otg_consistent_mode;
  result.target_valid = latest_targets_.valid;
  result.blend_progress = std::clamp(
      blend_elapsed_seconds_ /
          config_.spark_upper_qpoases.target_blend_seconds,
      0.0, 1.0);
  result.blend_active = latest_targets_.valid && result.blend_progress < 1.0;
  if (feedforward_mode) {
    result.left.headroom = left_.headroom.state();
    result.right.headroom = right_.headroom.state();
    result.left.feedforward_target = left_.last_feedforward_target;
    result.right.feedforward_target = right_.last_feedforward_target;
    const bool target_live = latest_targets_.valid;
    const auto copy_settled_hold_diagnostics = [](
                                                  const ArmState& state,
                                                  SparkGuidanceArmDiagnostics&
                                                      diagnostics) {
      diagnostics.settled_hold_active = state.settled_hold_active;
      diagnostics.settled_hold_dwell_seconds =
          state.settled_hold_dwell_seconds;
      diagnostics.settled_hold_reason = state.settled_hold_reason;
    };
    if (headroom_feedforward_mode &&
        config_.spark_headroom_feedforward_velocity_qp.settled_hold_enabled &&
        !target_live) {
      left_.settled_hold_active = true;
      right_.settled_hold_active = true;
      left_.settled_hold_dwell_seconds = 0.0;
      right_.settled_hold_dwell_seconds = 0.0;
      left_.settled_hold_reason = SparkSettledHoldReason::kStale;
      right_.settled_hold_reason = SparkSettledHoldReason::kStale;
    }
    bool left_released_this_cycle = false;
    bool right_released_this_cycle = false;
    if (target_live) {
      result.left.target = latest_targets_.left;
      result.right.target = latest_targets_.right;
      if (latest_source_sequence_ != last_palm_twist_sequence_) {
        result.left.palm_twist = left_.palm_twist.update(
            latest_targets_.left.palm, latest_source_sequence_,
            latest_source_timestamp_ns_, latest_source_epoch_,
            latest_source_discontinuity_);
        result.right.palm_twist = right_.palm_twist.update(
            latest_targets_.right.palm, latest_source_sequence_,
            latest_source_timestamp_ns_, latest_source_epoch_,
            latest_source_discontinuity_);
        if (headroom_feedforward_mode) {
          result.left.motion_intent_twist = left_.motion_intent_twist.update(
              latest_left_input_palm_, latest_source_sequence_,
              latest_source_timestamp_ns_, latest_source_epoch_,
              latest_source_discontinuity_);
          result.right.motion_intent_twist = right_.motion_intent_twist.update(
              latest_right_input_palm_, latest_source_sequence_,
              latest_source_timestamp_ns_, latest_source_epoch_,
              latest_source_discontinuity_);
        }
        last_palm_twist_sequence_ = latest_source_sequence_;
      } else {
        result.left.palm_twist.twist = left_.palm_twist.twist();
        result.left.palm_twist.low_frequency_twist =
            left_.palm_twist.lowFrequencyTwist();
        result.left.palm_twist.high_frequency_twist =
            left_.palm_twist.highFrequencyTwist();
        result.left.palm_twist.detail = "palm_twist_held";
        result.right.palm_twist.twist = right_.palm_twist.twist();
        result.right.palm_twist.low_frequency_twist =
            right_.palm_twist.lowFrequencyTwist();
        result.right.palm_twist.high_frequency_twist =
            right_.palm_twist.highFrequencyTwist();
        result.right.palm_twist.detail = "palm_twist_held";
        if (headroom_feedforward_mode) {
          result.left.motion_intent_twist.twist =
              left_.motion_intent_twist.twist();
          result.left.motion_intent_twist.low_frequency_twist =
              left_.motion_intent_twist.lowFrequencyTwist();
          result.left.motion_intent_twist.high_frequency_twist =
              left_.motion_intent_twist.highFrequencyTwist();
          result.left.motion_intent_twist.detail = "motion_intent_twist_held";
          result.right.motion_intent_twist.twist =
              right_.motion_intent_twist.twist();
          result.right.motion_intent_twist.low_frequency_twist =
              right_.motion_intent_twist.lowFrequencyTwist();
          result.right.motion_intent_twist.high_frequency_twist =
              right_.motion_intent_twist.highFrequencyTwist();
          result.right.motion_intent_twist.detail =
              "motion_intent_twist_held";
        }
      }
      if (headroom_feedforward_mode &&
          config_.spark_headroom_feedforward_velocity_qp
              .settled_hold_enabled) {
        const auto stationary_for_entry = [this](
                                      const SparkPalmTwistDecision& motion) {
          return !motion.reset &&
              motion.low_frequency_twist.head<3>().norm() <=
                  config_.spark_headroom_feedforward_velocity_qp
                      .stationary_reference_hold_linear_velocity_m_s &&
              motion.low_frequency_twist.tail<3>().norm() <=
                  config_.spark_headroom_feedforward_velocity_qp
                      .stationary_reference_hold_angular_velocity_rad_s;
        };
        const auto moving_for_release = [this](
                                      const SparkPalmTwistDecision& motion) {
          return !motion.reset &&
              (motion.twist.head<3>().norm() >
                   2.0 * config_.spark_headroom_feedforward_velocity_qp
                             .stationary_reference_hold_linear_velocity_m_s ||
               motion.twist.tail<3>().norm() >
                   2.0 * config_.spark_headroom_feedforward_velocity_qp
                             .stationary_reference_hold_angular_velocity_rad_s);
        };
        const auto update_settled_hold =
            [this, dt, &stationary_for_entry, &moving_for_release](
                                             ArmState& state,
                                             const ArmMotionState& model,
                                             const SparkPalmTwistDecision&
                                                 target_motion,
                                             const SparkPalmTwistDecision&
                                                 raw_motion,
                                             bool& released_this_cycle) {
          const bool target_stationary = stationary_for_entry(target_motion);
          const bool raw_stationary = stationary_for_entry(raw_motion);
          const bool corroborated_motion =
              moving_for_release(target_motion) &&
              moving_for_release(raw_motion);
          if (state.settled_hold_active) {
            if (corroborated_motion) {
              state.settled_hold_active = false;
              state.settled_hold_dwell_seconds = 0.0;
              state.settled_hold_reason = SparkSettledHoldReason::kNone;
              state.last_valid_q_ik = model.q;
              state.feedforward.reset(model, latest_source_epoch_);
              state.last_feedforward_target = {};
              released_this_cycle = true;
            }
            return;
          }
          const bool constrained_stationary =
              (target_stationary || raw_stationary) &&
              state.headroom.state().scale <=
                  config_.spark_headroom_feedforward_velocity_qp
                      .settled_hold_headroom_enter;
          state.settled_hold_dwell_seconds =
              constrained_stationary
                  ? state.settled_hold_dwell_seconds + dt
                  : 0.0;
          if (state.settled_hold_dwell_seconds >=
              config_.spark_headroom_feedforward_velocity_qp
                  .settled_hold_dwell_seconds) {
            state.settled_hold_active = true;
            state.settled_hold_reason =
                SparkSettledHoldReason::kHeadroomExhausted;
          }
        };
        update_settled_hold(
            left_, left_model, result.left.palm_twist,
            result.left.motion_intent_twist, left_released_this_cycle);
        update_settled_hold(
            right_, right_model, result.right.palm_twist,
            result.right.motion_intent_twist, right_released_this_cycle);
        if (left_released_this_cycle || right_released_this_cycle) {
          blend_restart_pending_ = true;
        }
      }
      const auto deadline =
          start +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(
                  config_.spark_upper_qpoases.ik_cycle_budget_seconds));
      result.left.ik = left_.ik.solve(latest_targets_.left,
                                      left_.last_valid_q_ik, deadline);
      result.right.ik = right_.ik.solve(latest_targets_.right,
                                        right_.last_valid_q_ik, deadline);
      if (result.left.ik.accepted && result.right.ik.accepted &&
          latest_source_sequence_ != last_feedforward_sequence_) {
        const auto should_hold_stationary_reference =
            [this, headroom_feedforward_mode](
                const SparkPalmTwistDecision& target_motion,
                const SparkPalmTwistDecision& raw_motion) {
              const auto stationary = [this](
                                          const SparkPalmTwistDecision& motion) {
                return !motion.reset &&
                    motion.low_frequency_twist.head<3>().norm() <=
                        config_.spark_headroom_feedforward_velocity_qp
                            .stationary_reference_hold_linear_velocity_m_s &&
                    motion.low_frequency_twist.tail<3>().norm() <=
                        config_.spark_headroom_feedforward_velocity_qp
                            .stationary_reference_hold_angular_velocity_rad_s;
              };
              return headroom_feedforward_mode &&
                  last_feedforward_sequence_ != 0U &&
                  (stationary(target_motion) || stationary(raw_motion));
            };
        result.left.stationary_joint_reference_held =
            should_hold_stationary_reference(
                result.left.palm_twist, result.left.motion_intent_twist);
        result.right.stationary_joint_reference_held =
            should_hold_stationary_reference(
                result.right.palm_twist, result.right.motion_intent_twist);
        const Vec7& left_q_ik_target =
            result.left.stationary_joint_reference_held
                ? left_.last_valid_q_ik
                : result.left.ik.q;
        const Vec7& right_q_ik_target =
            result.right.stationary_joint_reference_held
                ? right_.last_valid_q_ik
                : result.right.ik.q;
        result.left.feedforward_target = left_.feedforward.acceptTarget(
            left_q_ik_target, latest_source_sequence_,
            latest_source_timestamp_ns_, latest_source_epoch_, left_model);
        result.right.feedforward_target = right_.feedforward.acceptTarget(
            right_q_ik_target, latest_source_sequence_,
            latest_source_timestamp_ns_, latest_source_epoch_, right_model);
        left_.last_feedforward_target = result.left.feedforward_target;
        right_.last_feedforward_target = result.right.feedforward_target;
        last_feedforward_sequence_ = latest_source_sequence_;
        if (result.left.feedforward_target.accepted &&
            result.right.feedforward_target.accepted) {
          // Commit the position-IK seed only when both reference estimators
          // accept the same source frame. A rejected jump/timestamp must not
          // poison the next two-stage IK warm start.
          left_.last_valid_q_ik = left_q_ik_target;
          right_.last_valid_q_ik = right_q_ik_target;
          held_targets_ = latest_targets_;
        }
      }
    } else if (held_targets_.valid) {
      result.left.target = held_targets_.left;
      result.right.target = held_targets_.right;
      result.left.palm_twist.twist = left_.palm_twist.twist();
      result.left.palm_twist.low_frequency_twist =
          left_.palm_twist.lowFrequencyTwist();
      result.left.palm_twist.high_frequency_twist =
          left_.palm_twist.highFrequencyTwist();
      result.right.palm_twist.twist = right_.palm_twist.twist();
      result.right.palm_twist.low_frequency_twist =
          right_.palm_twist.lowFrequencyTwist();
      result.right.palm_twist.high_frequency_twist =
          right_.palm_twist.highFrequencyTwist();
    }

    result.left.q_ik = left_.last_valid_q_ik;
    result.right.q_ik = right_.last_valid_q_ik;
    result.left.feedforward =
        left_.feedforward.step(left_model, dt, target_live);
    result.right.feedforward =
        right_.feedforward.step(right_model, dt, target_live);
    if (!result.left.feedforward.valid || !result.right.feedforward.valid) {
      result.detail = !result.left.feedforward.valid
                          ? result.left.feedforward.detail
                          : result.right.feedforward.detail;
      return result;
    }
    if (!target_live && !left_.settled_hold_active &&
        !right_.settled_hold_active &&
        result.left.feedforward.state == SparkFeedforwardState::kHold &&
        result.right.feedforward.state == SparkFeedforwardState::kHold) {
      result.detail = "spark_feedforward_released";
      return result;
    }

    Vec6 left_twist = result.left.palm_twist.twist;
    Vec6 right_twist = result.right.palm_twist.twist;
    left_twist.head<3>() =
        config_.spark_feedforward_velocity_qp.position_feedforward_gain *
            left_twist.head<3>() +
        config_.spark_feedforward_velocity_qp
                .position_high_frequency_feedforward_gain *
            result.left.palm_twist.high_frequency_twist.head<3>();
    right_twist.head<3>() =
        config_.spark_feedforward_velocity_qp.position_feedforward_gain *
            right_twist.head<3>() +
        config_.spark_feedforward_velocity_qp
                .position_high_frequency_feedforward_gain *
            result.right.palm_twist.high_frequency_twist.head<3>();
    left_twist.tail<3>() =
        config_.spark_feedforward_velocity_qp.orientation_feedforward_gain *
            left_twist.tail<3>() +
        config_.spark_feedforward_velocity_qp
                .orientation_high_frequency_feedforward_gain *
            result.left.palm_twist.high_frequency_twist.tail<3>();
    right_twist.tail<3>() =
        config_.spark_feedforward_velocity_qp.orientation_feedforward_gain *
            right_twist.tail<3>() +
        config_.spark_feedforward_velocity_qp
                .orientation_high_frequency_feedforward_gain *
            result.right.palm_twist.high_frequency_twist.tail<3>();
    left_twist *= result.left.feedforward.activation;
    right_twist *= result.right.feedforward.activation;
    if (headroom_feedforward_mode) {
      left_twist *= result.left.headroom.scale;
      right_twist *= result.right.headroom.scale;
    }

    result.cartesian_targets.left = result.left.target.palm;
    result.cartesian_targets.right = result.right.target.palm;
    result.cartesian_targets.left_twist = left_twist;
    result.cartesian_targets.right_twist = right_twist;
    result.cartesian_targets.left_stale = !target_live;
    result.cartesian_targets.right_stale = !target_live;
    result.cartesian_references.left.pose = result.left.target.palm;
    result.cartesian_references.right.pose = result.right.target.palm;
    result.cartesian_references.left.twist = left_twist;
    result.cartesian_references.right.twist = right_twist;
    result.cartesian_references.left.stale = !target_live;
    result.cartesian_references.right.stale = !target_live;
    result.cartesian_references_valid = true;
    result.feedforward_references_valid = true;

    const auto make_joint_target = [this](
                                       const SparkFeedforwardReferenceResult&
                                           reference,
                                       const ArmMotionState& model,
                                       const ArmLimits& limits) -> Vec7 {
      Vec7 target =
          reference.qdot +
          config_.spark_feedforward_velocity_qp.joint_position_gain *
              (reference.q - model.q);
      return target.cwiseMax(-limits.velocity)
          .cwiseMin(limits.velocity)
          .eval();
    };
    result.posture_tasks.left.active = true;
    result.posture_tasks.left.source =
        JointVelocityPostureSource::kSparkFeedforwardJointReference;
    result.posture_tasks.left.target = make_joint_target(
        result.left.feedforward, left_model, left_.arm_limits);
    result.posture_tasks.left.activation =
        result.left.feedforward.activation;
    result.posture_tasks.left.weight =
        config_.spark_upper_qpoases.joint_reference_weight;
    result.posture_tasks.left.smoothness_weight =
        headroom_feedforward_mode
            ? config_.spark_headroom_feedforward_velocity_qp
                  .joint_reference_smoothness_weight
            : config_.spark_upper_qpoases.joint_reference_smoothness_weight;
    result.posture_tasks.left.jerk_smoothness_weight =
        headroom_feedforward_mode
            ? config_.spark_headroom_feedforward_velocity_qp
                  .joint_reference_jerk_smoothness_weight
            : 0.0;
    result.posture_tasks.right = result.posture_tasks.left;
    result.posture_tasks.right.target = make_joint_target(
        result.right.feedforward, right_model, right_.arm_limits);
    result.posture_tasks.right.activation =
        result.right.feedforward.activation;
    const auto apply_settled_hold = [this](
                                        ArmSide side, const ArmState& state,
                                        const ArmMotionState& model,
                                        bool released_this_cycle,
                                        SparkGuidanceArmDiagnostics& arm,
                                        Pose& cartesian_target,
                                        Vec6& cartesian_target_twist,
                                        CartesianReference& reference,
                                        JointVelocityPostureTask& posture) {
      if (!state.settled_hold_active && !released_this_cycle) {
        return;
      }
      const Pose model_tcp = robot_.armKinematicsAt(side, model.q).tcp_pose;
      arm.target.palm = model_tcp;
      cartesian_target = model_tcp;
      cartesian_target_twist.setZero();
      reference.pose = model_tcp;
      reference.twist.setZero();
      posture.active = true;
      posture.source =
          JointVelocityPostureSource::kSparkFeedforwardJointReference;
      posture.target.setZero();
      posture.activation = 1.0;
    };
    apply_settled_hold(
        ArmSide::kLeft, left_, left_model, left_released_this_cycle,
        result.left, result.cartesian_targets.left,
        result.cartesian_targets.left_twist, result.cartesian_references.left,
        result.posture_tasks.left);
    apply_settled_hold(
        ArmSide::kRight, right_, right_model, right_released_this_cycle,
        result.right, result.cartesian_targets.right,
        result.cartesian_targets.right_twist,
        result.cartesian_references.right, result.posture_tasks.right);
    copy_settled_hold_diagnostics(left_, result.left);
    copy_settled_hold_diagnostics(right_, result.right);
    result.left.accepted = true;
    result.right.accepted = true;
    result.accepted = true;
    result.detail = target_live ? "spark_feedforward_guidance_accepted"
                                : "spark_feedforward_stopping";
    result.compute_time_us = std::chrono::duration<double, std::micro>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
    return result;
  }

  const bool releasing_joint_reference =
      joint_reference_mode && !latest_targets_.valid && held_targets_.valid &&
      joint_reference_phase_ > 0.0;
  if (!latest_targets_.valid && !releasing_joint_reference) {
    result.detail = latest_targets_.detail;
    return result;
  }

  const SparkUpperTargets& step_targets =
      releasing_joint_reference ? held_targets_ : latest_targets_;
  result.left.target = step_targets.left;
  result.right.target = step_targets.right;
  result.cartesian_targets.left = step_targets.left.palm;
  result.cartesian_targets.right = step_targets.right.palm;
  if (releasing_joint_reference) {
    joint_reference_phase_ = std::max(
        0.0, joint_reference_phase_ -
                 dt / config_.spark_upper_qpoases
                          .joint_reference_release_seconds);
    const double activation = smoothstep5(joint_reference_phase_);
    if (activation <= 0.0) {
      result.detail = "spark_joint_reference_released";
      return result;
    }
    if (otg_consistent_mode) {
      result.cartesian_references.left = left_.otg.update(
          held_targets_.left.palm, Vec6::Zero(), true, dt);
      result.cartesian_references.right = right_.otg.update(
          held_targets_.right.palm, Vec6::Zero(), true, dt);
      result.cartesian_references_valid =
          result.cartesian_references.left.valid &&
          result.cartesian_references.right.valid;
      if (!result.cartesian_references_valid) {
        result.detail = "spark_otg_stale_stop_rejected";
        return result;
      }
      result.cartesian_targets.left = result.cartesian_references.left.pose;
      result.cartesian_targets.right = result.cartesian_references.right.pose;
      result.cartesian_targets.left_twist =
          result.cartesian_references.left.twist;
      result.cartesian_targets.right_twist =
          result.cartesian_references.right.twist;
      result.cartesian_targets.left_stale = true;
      result.cartesian_targets.right_stale = true;
      const auto release_deadline =
          start +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(
                  config_.spark_upper_qpoases.ik_cycle_budget_seconds));
      result.left.ik = left_.ik.solveOtgConsistent(
          held_targets_.left, result.cartesian_references.left.pose,
          left_.last_valid_q_ik, release_deadline);
      result.right.ik = right_.ik.solveOtgConsistent(
          held_targets_.right, result.cartesian_references.right.pose,
          right_.last_valid_q_ik, release_deadline);
      if (!result.left.ik.accepted || !result.right.ik.accepted) {
        result.detail = !result.left.ik.accepted ? result.left.ik.detail
                                                 : result.right.ik.detail;
        return result;
      }
      left_.last_valid_q_ik = result.left.ik.q;
      right_.last_valid_q_ik = result.right.ik.q;
    }
    result.left.q_ik = left_.last_valid_q_ik;
    result.right.q_ik = right_.last_valid_q_ik;
    const SparkPostureVelocityResult left_posture = makePostureVelocity(
        result.left.q_ik, left_model.q,
        activation * config_.spark_upper_qpoases.posture_position_gain,
        left_.arm_limits.velocity);
    const SparkPostureVelocityResult right_posture = makePostureVelocity(
        result.right.q_ik, right_model.q,
        activation * config_.spark_upper_qpoases.posture_position_gain,
        right_.arm_limits.velocity);
    if (!left_posture.accepted || !right_posture.accepted) {
      result.detail = !left_posture.accepted ? left_posture.detail
                                              : right_posture.detail;
      return result;
    }
    result.posture_tasks.left.active = true;
    result.posture_tasks.left.source =
        JointVelocityPostureSource::kSparkJointReference;
    result.posture_tasks.left.target = left_posture.value;
    result.posture_tasks.left.activation = activation;
    result.posture_tasks.left.weight =
        config_.spark_upper_qpoases.joint_reference_weight;
    result.posture_tasks.left.smoothness_weight =
        config_.spark_upper_qpoases.joint_reference_smoothness_weight;
    result.posture_tasks.right = result.posture_tasks.left;
    result.posture_tasks.right.target = right_posture.value;
    result.left.accepted = true;
    result.right.accepted = true;
    result.accepted = true;
    result.detail = otg_consistent_mode
                        ? "spark_otg_consistent_stale_stopping"
                        : "spark_joint_reference_releasing";
    result.compute_time_us = std::chrono::duration<double, std::micro>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
    return result;
  }
  if (posture_mode_ == SparkPostureGuideMode::kDisabled) {
    result.left.accepted = true;
    result.right.accepted = true;
    result.accepted = true;
    result.detail = "spark_pose_guidance_accepted";
    result.compute_time_us = std::chrono::duration<double, std::micro>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
    return result;
  }
  const auto deadline =
      start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(
                      config_.spark_upper_qpoases.ik_cycle_budget_seconds));
  if (otg_consistent_mode) {
    result.cartesian_references.left = left_.otg.update(
        latest_targets_.left.palm, Vec6::Zero(), false, dt);
    result.cartesian_references.right = right_.otg.update(
        latest_targets_.right.palm, Vec6::Zero(), false, dt);
    result.cartesian_references_valid =
        result.cartesian_references.left.valid &&
        result.cartesian_references.right.valid;
    if (!result.cartesian_references_valid) {
      result.detail = "spark_otg_reference_rejected";
      return result;
    }
    result.cartesian_targets.left = result.cartesian_references.left.pose;
    result.cartesian_targets.right = result.cartesian_references.right.pose;
    result.cartesian_targets.left_twist =
        result.cartesian_references.left.twist;
    result.cartesian_targets.right_twist =
        result.cartesian_references.right.twist;
    result.left.ik = left_.ik.solveOtgConsistent(
        latest_targets_.left, result.cartesian_references.left.pose,
        left_.last_valid_q_ik, deadline);
    result.right.ik = right_.ik.solveOtgConsistent(
        latest_targets_.right, result.cartesian_references.right.pose,
        right_.last_valid_q_ik, deadline);
  } else {
    result.left.ik = left_.ik.solve(latest_targets_.left,
                                    left_.last_valid_q_ik, deadline);
    result.right.ik = right_.ik.solve(latest_targets_.right,
                                      right_.last_valid_q_ik, deadline);
  }
  if (!result.left.ik.accepted || !result.right.ik.accepted) {
    result.detail = !result.left.ik.accepted ? result.left.ik.detail
                                             : result.right.ik.detail;
    return result;
  }
  left_.last_valid_q_ik = result.left.ik.q;
  right_.last_valid_q_ik = result.right.ik.q;
  held_targets_ = latest_targets_;
  result.left.q_ik = left_.last_valid_q_ik;
  result.right.q_ik = right_.last_valid_q_ik;
  if (posture_mode_ == SparkPostureGuideMode::kDirect ||
      joint_reference_mode) {
    if (joint_reference_mode) {
      joint_reference_phase_ = std::min(
          1.0, joint_reference_phase_ +
                   dt / config_.spark_upper_qpoases
                            .joint_reference_attack_seconds);
    }
    const double activation =
        joint_reference_mode ? smoothstep5(joint_reference_phase_) : 1.0;
    const SparkPostureVelocityResult left_posture = makePostureVelocity(
        result.left.q_ik, left_model.q,
        activation * config_.spark_upper_qpoases.posture_position_gain,
        left_.arm_limits.velocity);
    const SparkPostureVelocityResult right_posture = makePostureVelocity(
        result.right.q_ik, right_model.q,
        activation * config_.spark_upper_qpoases.posture_position_gain,
        right_.arm_limits.velocity);
    if (!left_posture.accepted || !right_posture.accepted) {
      result.detail = !left_posture.accepted ? left_posture.detail
                                              : right_posture.detail;
      return result;
    }
    result.posture_tasks.left.active = true;
    result.posture_tasks.left.source =
        joint_reference_mode
            ? JointVelocityPostureSource::kSparkJointReference
            : JointVelocityPostureSource::kSparkSoftQp;
    result.posture_tasks.left.target = left_posture.value;
    result.posture_tasks.left.activation = activation;
    result.posture_tasks.left.weight =
        joint_reference_mode
            ? config_.spark_upper_qpoases.joint_reference_weight
            : config_.spark_upper_qpoases.posture_weight;
    result.posture_tasks.left.smoothness_weight =
        joint_reference_mode
            ? config_.spark_upper_qpoases.joint_reference_smoothness_weight
            : 0.0;
    result.posture_tasks.right = result.posture_tasks.left;
    result.posture_tasks.right.target = right_posture.value;
    result.left.accepted = true;
    result.right.accepted = true;
    result.accepted = true;
    result.detail = otg_consistent_mode
                        ? "spark_otg_consistent_guidance_accepted"
                    : joint_reference_mode
                        ? "spark_joint_reference_guidance_accepted"
                        : "spark_direct_guidance_accepted";
    result.compute_time_us = std::chrono::duration<double, std::micro>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
    return result;
  }
  const ArmMotionState left_before = left_.reference.state();
  const ArmMotionState right_before = right_.reference.state();
  result.left.reference =
      left_.reference.update(result.left.ik.q, left_model.q, dt);
  result.right.reference =
      right_.reference.update(result.right.ik.q, right_model.q, dt);
  if (!result.left.reference.accepted || !result.right.reference.accepted) {
    (void)left_.reference.reset(left_before);
    (void)right_.reference.reset(right_before);
    result.detail = !result.left.reference.accepted
                        ? result.left.reference.detail
                        : result.right.reference.detail;
    return result;
  }
  result.left.accepted = true;
  result.right.accepted = true;
  result.posture_tasks.left.active = true;
  result.posture_tasks.left.source =
      JointVelocityPostureSource::kSparkSoftQp;
  result.posture_tasks.left.target = result.left.reference.posture_velocity;
  result.posture_tasks.left.weight = config_.spark_upper_qpoases.posture_weight;
  result.posture_tasks.right = result.posture_tasks.left;
  result.posture_tasks.right.target = result.right.reference.posture_velocity;
  result.accepted = true;
  result.detail = "spark_guidance_accepted";
  result.compute_time_us = std::chrono::duration<double, std::micro>(
                               std::chrono::steady_clock::now() - start)
                               .count();
  return result;
}

void DualArmSparkGuidance::invalidateTarget(std::string_view detail) noexcept {
  raw_targets_.valid = false;
  raw_targets_.detail = detail;
  latest_targets_.valid = false;
  latest_targets_.detail = detail;
}

std::string_view toString(SparkSettledHoldReason reason) noexcept {
  switch (reason) {
    case SparkSettledHoldReason::kNone:
      return "none";
    case SparkSettledHoldReason::kStale:
      return "stale";
    case SparkSettledHoldReason::kHeadroomExhausted:
      return "headroom_exhausted";
  }
  return "none";
}

}  // namespace tianji_qp_ik
