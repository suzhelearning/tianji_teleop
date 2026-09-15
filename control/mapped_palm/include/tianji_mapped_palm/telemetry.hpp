#pragma once

#include "tianji_mapped_palm/arm_angle.hpp"
#include "tianji_mapped_palm/causal_se3_twist_estimator.hpp"
#include "tianji_mapped_palm/cartesian_feedforward_allocator.hpp"
#include "tianji_mapped_palm/cartesian_task_allocator.hpp"
#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/mujoco_robot.hpp"
#include "tianji_mapped_palm/pico_ee_headroom.hpp"
#include "tianji_mapped_palm/pico_ee_motion_confidence.hpp"
#include "tianji_mapped_palm/pico_teleop_protocol.hpp"
#include "tianji_mapped_palm/safety.hpp"
#include "tianji_mapped_palm/target_manager.hpp"
#include "tianji_mapped_palm/types.hpp"

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace tianji_mapped_palm {

struct PicoEeExecutionSnapshot {
  bool motion_confidence_valid{false};
  double motion_confidence_linear_raw{0.0};
  double motion_confidence_angular_raw{0.0};
  double motion_confidence_linear_filtered{0.0};
  double motion_confidence_angular_filtered{0.0};
  double motion_confidence_raw{0.0};
  double motion_confidence_filtered{0.0};
  MotionConfidenceReason motion_confidence_reason{
      MotionConfidenceReason::kDisabled};
  Eigen::Vector3d scheduled_position_gain{Eigen::Vector3d::Zero()};
  Eigen::Vector3d scheduled_orientation_gain{Eigen::Vector3d::Zero()};
  double redundancy_authority{1.0};
  PicoEeTwistEstimatorMode estimator_mode{PicoEeTwistEstimatorMode::kLegacy};
  bool estimator_valid{false};
  double estimator_confidence{0.0};
  CausalSe3TwistFailure estimator_failure{
      CausalSe3TwistFailure::kInsufficientHistory};
  int estimator_fit_window{0};
  double estimator_condition_number{0.0};
  double estimator_position_residual_m{0.0};
  double estimator_orientation_residual_rad{0.0};
  double estimator_observation_age_s{0.0};
  double estimator_source_linear_velocity{0.0};
  double estimator_source_angular_velocity{0.0};
  double estimator_source_linear_acceleration{0.0};
  double estimator_source_angular_acceleration{0.0};
  double estimator_age_compensation_linear_velocity{0.0};
  double estimator_age_compensation_angular_velocity{0.0};
  double legacy_lf_linear_velocity{0.0};
  double legacy_lf_angular_velocity{0.0};
  double legacy_hf_linear_velocity{0.0};
  double legacy_hf_angular_velocity{0.0};
  double causal_lf_linear_velocity{0.0};
  double causal_lf_angular_velocity{0.0};
  double causal_hf_linear_velocity{0.0};
  double causal_hf_angular_velocity{0.0};
  Vec6 raw_feedforward{Vec6::Zero()};
  Vec6 raw_low_frequency_feedforward{Vec6::Zero()};
  Vec6 raw_high_frequency_feedforward{Vec6::Zero()};
  double feedback_linear_velocity{0.0};
  double feedback_angular_velocity{0.0};
  Vec6 feedback_twist{Vec6::Zero()};
  double preallocation_ff_linear_velocity{0.0};
  double preallocation_ff_angular_velocity{0.0};
  Vec6 preallocation_feedforward{Vec6::Zero()};
  double allocated_ff_linear_velocity{0.0};
  double allocated_ff_angular_velocity{0.0};
  Vec6 allocated_feedforward{Vec6::Zero()};
  double final_desired_linear_velocity{0.0};
  double final_desired_angular_velocity{0.0};
  Vec6 final_desired_twist{Vec6::Zero()};
  double executed_linear_velocity{0.0};
  double executed_angular_velocity{0.0};
  Vec6 executed_twist{Vec6::Zero()};
  double execution_residual_linear_velocity{0.0};
  double execution_residual_angular_velocity{0.0};
  Vec6 execution_residual{Vec6::Zero()};
  double legacy_low_scale{0.0};
  double legacy_high_scale{0.0};
  double selected_low_scale{0.0};
  double selected_high_scale{0.0};
  PicoEeFeedforwardAllocatorMode allocator_mode{
      PicoEeFeedforwardAllocatorMode::kLegacy};
  bool allocator_valid{false};
  bool allocator_selected{false};
  CartesianFeedforwardLimit allocator_failure{
      CartesianFeedforwardLimit::kNone};
  int allocator_preview_solves{0};
  double allocator_compute_time_us{0.0};
  double allocator_preview_actual_mismatch_rad_s{0.0};
  double allocator_normalized_residual{0.0};
  double allocator_velocity_usage{0.0};
  double allocator_acceleration_usage{0.0};
  double allocator_jerk_usage{0.0};
  double allocator_arm_angle_residual{0.0};
  double allocator_nullspace_leakage{0.0};
  PicoEeTaskAllocationMode task_allocator_mode{
      PicoEeTaskAllocationMode::kDisabled};
  bool task_allocator_valid{false};
  bool task_allocator_authorized{false};
  CartesianTaskAllocationLimit task_allocator_failure{
      CartesianTaskAllocationLimit::kNone};
  double task_allocator_scale{1.0};
  double task_allocator_nullspace_alpha_command{0.0};
  double task_allocator_normalized_residual{0.0};
  double task_allocator_velocity_usage{0.0};
  double task_allocator_acceleration_usage{0.0};
  double task_allocator_jerk_usage{0.0};
  double task_allocator_compute_time_us{0.0};
  int task_allocator_preview_solves{0};
};

struct ArmIkSnapshot {
  bool accepted{false};
  bool fallback_applied{false};
  HoldReason hold_reason{HoldReason::kNone};
  double position_error{0.0};
  double orientation_error{0.0};
  double actual_position_error{0.0};
  double actual_orientation_error{0.0};
  double slack_position_norm{0.0};
  double slack_orientation_norm{0.0};
  double equality_residual{0.0};
  double reference_error_max_abs{0.0};
  double reference_scale{1.0};
  bool reference_frozen{false};
  double qdot_max_ratio{0.0};
  double solve_time_us{0.0};
  int active_position_bounds{0};
  int active_velocity_bounds{0};
  int active_acceleration_bounds{0};
  int active_braking_bounds{0};
  int iterations{0};
  SolverStatus status{SolverStatus::kInvalidInput};
  bool otg_valid{true};
  bool otg_stale{false};
  double reference_linear_velocity{0.0};
  double reference_angular_velocity{0.0};
  double reference_linear_acceleration{0.0};
  double reference_angular_acceleration{0.0};
  double qdot_reference_error_max_abs{0.0};
  double qddot_max_ratio{0.0};
  int active_qddot_bounds{0};
  double task_scale_position{1.0};
  double task_scale_orientation{1.0};
  ArmDirectionReferenceSource arm_angle_reference_source{
      ArmDirectionReferenceSource::kDegenerate};
  bool arm_angle_active{false};
  double arm_angle_error_rad{0.0};
  double arm_angle_robot_rad{0.0};
  double arm_angle_target_rad{0.0};
  double arm_angle_control_error_rad{0.0};
  double arm_angle_current_rate_rad_s{0.0};
  double arm_angle_requested_velocity_rad_s{0.0};
  double arm_angle_requested_acceleration_rad_s2{0.0};
  double arm_angle_radius_m{0.0};
  double arm_angle_reference_projection_norm{0.0};
  double arm_angle_jacobian_norm{0.0};
  bool arm_angle_projection_held{false};
  bool arm_angle_reference_governor_held{false};
  bool arm_angle_branch_lock_active{false};
  double arm_angle_branch_lock_distance_m{0.0};
  bool arm_angle_branch_lock_constraint_active{false};
  double arm_angle_branch_lock_requested_lower{0.0};
  double arm_angle_branch_lock_effective_lower{0.0};
  bool arm_angle_branch_lock_feasibility_clipped{false};
  double arm_angle_achieved_acceleration_rad_s2{0.0};
  double arm_angle_acceleration_residual_rad_s2{0.0};
  int mapped_vector_mode{0};
  bool mapped_vector_valid{false};
  bool mapped_vector_active{false};
  int mapped_vector_disable_reason{0};
  bool mapped_vector_basis_flipped{false};
  double mapped_vector_basis_rotation_rate_rad_s{0.0};
  double mapped_vector_cartesian_leakage{0.0};
  double mapped_vector_arm_angle_observability{0.0};
  bool mapped_vector_exact_prediction_feasible{false};
  double mapped_vector_predicted_task_scale{1.0};
  double mapped_vector_command_tracking_weight{0.0};
  double mapped_vector_u_pico_rad_s{0.0};
  double mapped_vector_alpha_raw{0.0};
  double mapped_vector_alpha_command{0.0};
  double mapped_vector_alpha_previous{0.0};
  double mapped_vector_alpha_trend{0.0};
  double mapped_vector_alpha_achieved{0.0};
  double mapped_vector_feasible_lower{0.0};
  double mapped_vector_feasible_upper{0.0};
  double mapped_vector_velocity_norm{0.0};
  double mapped_vector_acceleration_norm{0.0};
  double mapped_vector_jerk_norm{0.0};
  double mapped_vector_reference_cost{0.0};
  double mapped_vector_continuity_cost{0.0};
  double mapped_vector_acceleration_cost{0.0};
  double mapped_vector_jerk_cost{0.0};
  double velocity_qp_full_acceleration_cost{0.0};
  double velocity_qp_full_jerk_cost{0.0};
  double velocity_qp_nullspace_acceleration_cost{0.0};
  double velocity_qp_nullspace_jerk_cost{0.0};
  bool mapped_vector_legacy_fallback{false};
  double elbow_world_z{0.0};
  double shoulder_world_z{0.0};
  bool upper_arm_outward_active{false};
  double upper_arm_outward_distance_m{0.0};
  double upper_arm_outward_requested_lower{0.0};
  double upper_arm_outward_effective_lower{0.0};
  double upper_arm_outward_achieved{0.0};
  double upper_arm_outward_residual{0.0};
  bool upper_arm_outward_feasibility_clipped{false};
  bool dls_posture_reference_active{false};
  // PoseDlsStatus::kRejected; kept as an integer to keep telemetry.hpp
  // independent from the iterative solver implementation header.
  int dls_posture_status{2};
  int dls_posture_iterations{0};
  int dls_posture_joint_projection_count{0};
  bool dls_posture_ruckig_accepted{false};
  double dls_posture_solve_time_us{0.0};
  double dls_posture_initial_position_error_m{0.0};
  double dls_posture_initial_orientation_error_rad{0.0};
  double dls_posture_final_position_error_m{0.0};
  double dls_posture_final_orientation_error_rad{0.0};
  double dls_posture_goal_error_max_abs{0.0};
  double dls_posture_reference_error_max_abs{0.0};
  double dls_posture_velocity_target_max_abs{0.0};
  double dls_posture_qdot_error_max_abs{0.0};
  double dls_posture_goal_limit_margin_rad{0.0};
  bool spark_posture_active{false};
  bool spark_ik_accepted{false};
  int spark_stage1_iterations{0};
  int spark_stage2_iterations{0};
  double spark_solve_time_us{0.0};
  double spark_palm_position_error_m{0.0};
  double spark_palm_orientation_error_rad{0.0};
  double spark_reference_velocity_ratio{0.0};
  double spark_reference_acceleration_ratio{0.0};
  double spark_reference_jerk_ratio{0.0};
  double spark_q_ik_error_max_abs{0.0};
  double spark_q_ref_error_max_abs{0.0};
  double spark_posture_velocity_max_abs{0.0};
  bool spark_feedforward_valid{false};
  int spark_feedforward_state{0};
  bool spark_feedforward_dt_valid{false};
  bool spark_feedforward_jump_rejected{false};
  bool spark_feedforward_epoch_reset{false};
  double spark_feedforward_source_dt_seconds{0.0};
  double spark_feedforward_median_dt_seconds{0.0};
  double spark_feedforward_activation{0.0};
  double spark_feedforward_linear_velocity{0.0};
  double spark_feedforward_angular_velocity{0.0};
  double spark_motion_intent_linear_velocity{0.0};
  double spark_motion_intent_angular_velocity{0.0};
  bool spark_stationary_joint_reference_held{false};
  bool spark_settled_hold_active{false};
  double spark_settled_hold_dwell_seconds{0.0};
  int spark_settled_hold_reason{0};
  Vec7 spark_feedforward_q_ik{Vec7::Zero()};
  Vec7 spark_feedforward_q{Vec7::Zero()};
  Vec7 spark_feedforward_qdot{Vec7::Zero()};
  Vec7 spark_feedforward_qddot{Vec7::Zero()};
  Vec7 spark_feedforward_jerk{Vec7::Zero()};
  bool headroom_valid{false};
  bool headroom_derivative_history_valid{false};
  double headroom_velocity{0.0};
  double headroom_acceleration{0.0};
  double headroom_jerk{0.0};
  double headroom_task{0.0};
  double headroom_raw{0.0};
  double headroom_filtered{0.0};
  double headroom_scale{0.0};
  int headroom_state{0};
  int headroom_dominant_source{0};
};

struct TelemetrySample {
  std::uint64_t sequence{0U};
  double control_time_seconds{0.0};
  IkAlgorithm algorithm{IkAlgorithm::kHierarchicalQp};
  ControlLevel control_level{ControlLevel::kVelocity};
  SolverBackend backend{SolverBackend::kQpoases};
  TargetMode mode{TargetMode::kHold};
  ArmAngleReferenceMode arm_angle_reference_mode{
      ArmAngleReferenceMode::kPico};
  bool paused{false};
  bool accepted{false};
  HoldReason hold_reason{HoldReason::kNone};
  bool left_target_stale{false};
  bool right_target_stale{false};
  bool otg_enabled{false};
  double left_position_error{0.0};
  double left_orientation_error{0.0};
  double right_position_error{0.0};
  double right_orientation_error{0.0};
  ArmIkSnapshot left_ik;
  ArmIkSnapshot right_ik;
  double cycle_time_us{0.0};
  double cycle_p99_us{0.0};
  std::uint64_t deadline_misses{0U};
  std::uint64_t control_failures{0U};
  bool left_hand_tcp_selected{false};
  bool right_hand_tcp_selected{false};
  bool hand_model_valid{false};
  bool hand_configured{false};
  bool hand_live{false};
  bool hand_stale{true};
  std::uint64_t hand_sequence{0U};
  std::uint64_t hand_datagrams{0U};
  std::uint64_t hand_accepted{0U};
  std::uint64_t hand_malformed{0U};
  std::uint64_t hand_crc_failures{0U};
  std::uint64_t hand_reordered{0U};
  double hand_frame_age_ms{std::numeric_limits<double>::infinity()};
  Vec20 left_hand_q{Vec20::Zero()};
  Vec20 right_hand_q{Vec20::Zero()};
  bool pico_configured{false};
  bool pico_enabled{false};
  bool pico_live{false};
  bool pico_stale{false};
  std::uint64_t pico_tracking_epoch{0U};
  std::uint64_t pico_sequence{0U};
  std::uint64_t pico_datagrams{0U};
  std::uint64_t pico_accepted{0U};
  std::uint64_t pico_malformed{0U};
  std::uint64_t pico_crc_failures{0U};
  std::uint64_t pico_reordered{0U};
  std::uint64_t pico_jump_rejections{0U};
  std::uint64_t pico_superseded{0U};
  std::uint64_t pico_epoch_resets{0U};
  std::uint64_t pico_resynchronizations{0U};
  std::uint64_t pico_reset_applies{0U};
  std::int64_t pico_left_source_timestamp_ns{0};
  std::int64_t pico_right_source_timestamp_ns{0};
  double pico_input_frequency_hz{0.0};
  double pico_frame_age_ms{0.0};
  double pico_receive_to_control_us{0.0};
  double pico_bridge_to_control_us{0.0};
  PicoEeTargetSource pico_ee_target_source{
      PicoEeTargetSource::kPacketTargets};
  PicoArmAngleReferenceSource pico_arm_angle_reference_source{
      PicoArmAngleReferenceSource::kPacketDirection};
  bool pico_ee_mapped_target_valid{false};
  bool left_pico_ee_reference_valid{false};
  bool left_pico_ee_reference_stale{false};
  double left_pico_ee_twist_linear_velocity{0.0};
  double left_pico_ee_twist_angular_velocity{0.0};
  double left_pico_ee_twist_lf_linear_velocity{0.0};
  double left_pico_ee_twist_lf_angular_velocity{0.0};
  double left_pico_ee_twist_hf_linear_velocity{0.0};
  double left_pico_ee_twist_hf_angular_velocity{0.0};
  double left_pico_ee_twist_final_linear_velocity{0.0};
  double left_pico_ee_twist_final_angular_velocity{0.0};
  double left_pico_ee_headroom_lf_scale{0.0};
  double left_pico_ee_headroom_hf_scale{0.0};
  double left_pico_ee_headroom_scale{0.0};
  double left_pico_ee_headroom_raw{0.0};
  double left_pico_ee_headroom_filtered{0.0};
  PicoEeHeadroomSource left_pico_ee_headroom_dominant_source{
      PicoEeHeadroomSource::kNone};
  bool right_pico_ee_reference_valid{false};
  bool right_pico_ee_reference_stale{false};
  double right_pico_ee_twist_linear_velocity{0.0};
  double right_pico_ee_twist_angular_velocity{0.0};
  double right_pico_ee_twist_lf_linear_velocity{0.0};
  double right_pico_ee_twist_lf_angular_velocity{0.0};
  double right_pico_ee_twist_hf_linear_velocity{0.0};
  double right_pico_ee_twist_hf_angular_velocity{0.0};
  double right_pico_ee_twist_final_linear_velocity{0.0};
  double right_pico_ee_twist_final_angular_velocity{0.0};
  double right_pico_ee_headroom_lf_scale{0.0};
  double right_pico_ee_headroom_hf_scale{0.0};
  double right_pico_ee_headroom_scale{0.0};
  double right_pico_ee_headroom_raw{0.0};
  double right_pico_ee_headroom_filtered{0.0};
  PicoEeHeadroomSource right_pico_ee_headroom_dominant_source{
      PicoEeHeadroomSource::kNone};
  PicoEeExecutionSnapshot left_pico_ee_execution;
  PicoEeExecutionSnapshot right_pico_ee_execution;
  Pose left_target_pose;
  Pose left_reference_pose;
  Pose left_actual_pose;
  Pose right_target_pose;
  Pose right_reference_pose;
  Pose right_actual_pose;
};

struct ViewerSnapshot {
  std::uint64_t sequence{0U};
  std::uint64_t last_processed_command_id{0U};
  double control_time_seconds{0.0};
  Vec7 left_q{Vec7::Zero()};
  Vec7 right_q{Vec7::Zero()};
  Vec20 left_hand_q{Vec20::Zero()};
  Vec20 right_hand_q{Vec20::Zero()};
  DualArmTargets targets;
  IkAlgorithm algorithm{IkAlgorithm::kHierarchicalQp};
  ControlLevel control_level{ControlLevel::kVelocity};
  SolverBackend backend{SolverBackend::kQpoases};
  TargetMode mode{TargetMode::kHold};
  bool paused{false};
  bool at_nominal_configuration{false};
  bool accepted{false};
  HoldReason hold_reason{HoldReason::kNone};
  bool left_target_stale{false};
  bool right_target_stale{false};
  bool otg_enabled{false};
  bool left_hand_tcp_selected{false};
  bool right_hand_tcp_selected{false};
  bool hand_model_valid{false};
  bool hand_configured{false};
  bool hand_live{false};
  bool hand_stale{true};
  std::uint64_t hand_sequence{0U};
  std::uint64_t hand_datagrams{0U};
  std::uint64_t hand_accepted{0U};
  std::uint64_t hand_malformed{0U};
  std::uint64_t hand_crc_failures{0U};
  std::uint64_t hand_reordered{0U};
  double hand_frame_age_ms{std::numeric_limits<double>::infinity()};
  double left_position_error{0.0};
  double left_orientation_error{0.0};
  double right_position_error{0.0};
  double right_orientation_error{0.0};
  SolverStatus left_solver_status{SolverStatus::kInvalidInput};
  SolverStatus right_solver_status{SolverStatus::kInvalidInput};
  ArmIkSnapshot left_ik;
  ArmIkSnapshot right_ik;
  double cycle_time_us{0.0};
  double cycle_p99_us{0.0};
  std::uint64_t deadline_misses{0U};
  std::uint64_t control_failures{0U};
  std::uint64_t snapshot_drops{0U};
  std::uint64_t telemetry_drops{0U};
  std::uint64_t joint_plot_samples{0U};
  std::uint64_t joint_plot_drops{0U};
  bool joint_plot_reference_derivatives_valid{false};
  bool joint_plot_actual_derivatives_valid{false};
  bool pico_configured{false};
  bool pico_enabled{false};
  bool pico_live{false};
  bool pico_stale{false};
  ArmAngleReferenceMode arm_angle_reference_mode{
      ArmAngleReferenceMode::kPico};
  std::uint64_t pico_tracking_epoch{0U};
  std::uint64_t pico_sequence{0U};
  std::uint64_t pico_datagrams{0U};
  std::uint64_t pico_accepted{0U};
  std::uint64_t pico_malformed{0U};
  std::uint64_t pico_crc_failures{0U};
  std::uint64_t pico_reordered{0U};
  std::uint64_t pico_jump_rejections{0U};
  std::uint64_t pico_superseded{0U};
  std::uint64_t pico_epoch_resets{0U};
  std::uint64_t pico_resynchronizations{0U};
  std::uint64_t pico_reset_applies{0U};
  std::int64_t pico_left_source_timestamp_ns{0};
  std::int64_t pico_right_source_timestamp_ns{0};
  double pico_input_frequency_hz{0.0};
  double pico_frame_age_ms{0.0};
  double pico_receive_to_control_us{0.0};
  double pico_bridge_to_control_us{0.0};
  PicoEeTargetSource pico_ee_target_source{
      PicoEeTargetSource::kPacketTargets};
  PicoArmAngleReferenceSource pico_arm_angle_reference_source{
      PicoArmAngleReferenceSource::kPacketDirection};
  bool pico_ee_mapped_target_valid{false};
  bool left_pico_ee_reference_valid{false};
  bool left_pico_ee_reference_stale{false};
  double left_pico_ee_twist_linear_velocity{0.0};
  double left_pico_ee_twist_angular_velocity{0.0};
  double left_pico_ee_twist_lf_linear_velocity{0.0};
  double left_pico_ee_twist_lf_angular_velocity{0.0};
  double left_pico_ee_twist_hf_linear_velocity{0.0};
  double left_pico_ee_twist_hf_angular_velocity{0.0};
  double left_pico_ee_twist_final_linear_velocity{0.0};
  double left_pico_ee_twist_final_angular_velocity{0.0};
  double left_pico_ee_headroom_lf_scale{0.0};
  double left_pico_ee_headroom_hf_scale{0.0};
  double left_pico_ee_headroom_scale{0.0};
  double left_pico_ee_headroom_raw{0.0};
  double left_pico_ee_headroom_filtered{0.0};
  PicoEeHeadroomSource left_pico_ee_headroom_dominant_source{
      PicoEeHeadroomSource::kNone};
  bool right_pico_ee_reference_valid{false};
  bool right_pico_ee_reference_stale{false};
  double right_pico_ee_twist_linear_velocity{0.0};
  double right_pico_ee_twist_angular_velocity{0.0};
  double right_pico_ee_twist_lf_linear_velocity{0.0};
  double right_pico_ee_twist_lf_angular_velocity{0.0};
  double right_pico_ee_twist_hf_linear_velocity{0.0};
  double right_pico_ee_twist_hf_angular_velocity{0.0};
  double right_pico_ee_twist_final_linear_velocity{0.0};
  double right_pico_ee_twist_final_angular_velocity{0.0};
  double right_pico_ee_headroom_lf_scale{0.0};
  double right_pico_ee_headroom_hf_scale{0.0};
  double right_pico_ee_headroom_scale{0.0};
  double right_pico_ee_headroom_raw{0.0};
  double right_pico_ee_headroom_filtered{0.0};
  PicoEeHeadroomSource right_pico_ee_headroom_dominant_source{
      PicoEeHeadroomSource::kNone};
  PicoEeExecutionSnapshot left_pico_ee_execution;
  PicoEeExecutionSnapshot right_pico_ee_execution;
  PicoUpperLimbSkeleton pico_upper_limb_skeleton;
  PicoUpperLimbSkeleton spark_upper_limb_skeleton;
};

enum class ViewerCommandType {
  kSetMode,
  kSetBackend,
  kSetIkAlgorithm,
  kSetControlLevel,
  kResetNominal,
  kSetManualTarget,
  kSetPaused,
  kTogglePicoTeleop,
  kTogglePicoArmAngleSource,
};

struct ViewerCommand {
  std::uint64_t id{0U};
  ViewerCommandType type{ViewerCommandType::kSetMode};
  TargetMode mode{TargetMode::kHold};
  SolverBackend backend{SolverBackend::kQpoases};
  IkAlgorithm algorithm{IkAlgorithm::kHierarchicalQp};
  ControlLevel control_level{ControlLevel::kVelocity};
  ArmSide side{ArmSide::kLeft};
  Pose target;
  double target_timestamp_seconds{
      std::numeric_limits<double>::quiet_NaN()};
  bool paused{false};
};

template <typename T>
class BoundedSpscQueue {
 public:
  explicit BoundedSpscQueue(std::size_t capacity) : storage_(capacity + 1U) {
    if (capacity == 0U) {
      throw std::invalid_argument("SPSC queue capacity must be positive");
    }
  }

  BoundedSpscQueue(const BoundedSpscQueue&) = delete;
  BoundedSpscQueue& operator=(const BoundedSpscQueue&) = delete;

  std::size_t capacity() const noexcept { return storage_.size() - 1U; }

  bool tryPush(const T& value) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = increment(head);
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;
    }
    storage_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool tryPop(T& value) noexcept {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return false;
    }
    value = storage_[tail];
    tail_.store(increment(tail), std::memory_order_release);
    return true;
  }

 private:
  std::size_t increment(std::size_t index) const noexcept {
    ++index;
    return index == storage_.size() ? 0U : index;
  }

  std::vector<T> storage_;
  alignas(64) std::atomic<std::size_t> head_{0U};
  alignas(64) std::atomic<std::size_t> tail_{0U};
};

using TelemetryBuffer = BoundedSpscQueue<TelemetrySample>;

enum class LatestPublishResult { kPublished, kSuperseded };

template <typename T>
class LatestSpscExchange {
 public:
  LatestSpscExchange() = default;

  LatestSpscExchange(const LatestSpscExchange&) = delete;
  LatestSpscExchange& operator=(const LatestSpscExchange&) = delete;

  LatestPublishResult publish(const T& value) noexcept {
    slots_[producer_slot_] = value;
    const std::uint32_t published_state =
        static_cast<std::uint32_t>(producer_slot_) | kDirtyBit;
    const std::uint32_t previous_state =
        middle_state_.exchange(published_state, std::memory_order_acq_rel);
    producer_slot_ = static_cast<std::size_t>(previous_state & kSlotMask);
    return (previous_state & kDirtyBit) != 0U
               ? LatestPublishResult::kSuperseded
               : LatestPublishResult::kPublished;
  }

  bool tryReadLatest(T& value) noexcept {
    if ((middle_state_.load(std::memory_order_acquire) & kDirtyBit) == 0U) {
      return false;
    }
    const std::uint32_t previous_state = middle_state_.exchange(
        static_cast<std::uint32_t>(consumer_slot_), std::memory_order_acq_rel);
    consumer_slot_ = static_cast<std::size_t>(previous_state & kSlotMask);
    value = slots_[consumer_slot_];
    return true;
  }

 private:
  static constexpr std::uint32_t kDirtyBit = 1U << 31U;
  static constexpr std::uint32_t kSlotMask = 0x03U;
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
                "latest-only exchange requires a lock-free 32-bit atomic");

  std::array<T, 3> slots_{};
  alignas(64) std::size_t producer_slot_{0U};
  alignas(64) std::size_t consumer_slot_{1U};
  alignas(64) std::atomic<std::uint32_t> middle_state_{2U};
};

template <typename T>
class LatestSnapshotExchange {
 public:
  explicit LatestSnapshotExchange(std::size_t capacity) : queue_(capacity) {}

  bool tryPublish(const T& snapshot) noexcept { return queue_.tryPush(snapshot); }

  bool tryReadLatest(T& latest) noexcept {
    T candidate;
    if (!queue_.tryPop(candidate)) {
      return false;
    }
    latest = candidate;
    while (queue_.tryPop(candidate)) {
      latest = candidate;
    }
    return true;
  }

 private:
  BoundedSpscQueue<T> queue_;
};

class CycleTimeWindow {
 public:
  explicit CycleTimeWindow(std::size_t capacity);
  void add(double duration_us) noexcept;
  double percentile99();
  std::size_t size() const noexcept { return size_; }

 private:
  std::vector<double> samples_;
  std::vector<double> scratch_;
  std::size_t cursor_{0U};
  std::size_t size_{0U};
};

}  // namespace tianji_mapped_palm
