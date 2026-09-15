#pragma once

#include "tianji_qp_ik/types.hpp"

#include <Eigen/Core>

#include <string>

namespace tianji_qp_ik {

enum class SolverBackend { kOsqp, kQpoases };
enum class IkAlgorithm {
  kHierarchicalQp,
  kNullspaceDls,
  kSparkGuidedVelocityQp,
  kSparkDirectVelocityQp,
  kSparkPoseVelocityQp,
  kSparkUpperQpoasesDirect,
  kSparkUpperQpoasesVelocityQp,
  kSparkUpperQpoasesCartesianOtgVelocityQp,
  kSparkUpperQpoasesFeedforwardVelocityQp,
  kSparkUpperQpoasesHeadroomFeedforwardVelocityQp,
};

inline bool usesSparkHeadroomFeedforwardVelocityQp(
    IkAlgorithm algorithm) noexcept {
  return algorithm ==
         IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp;
}

inline bool usesSparkFeedforwardVelocityQp(
    IkAlgorithm algorithm) noexcept {
  return algorithm ==
             IkAlgorithm::kSparkUpperQpoasesFeedforwardVelocityQp ||
         usesSparkHeadroomFeedforwardVelocityQp(algorithm);
}

inline bool usesSparkOtgConsistentVelocityQp(
    IkAlgorithm algorithm) noexcept {
  return algorithm ==
         IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp;
}

inline bool usesSparkVelocityQp(IkAlgorithm algorithm) noexcept {
  return algorithm == IkAlgorithm::kSparkGuidedVelocityQp ||
         algorithm == IkAlgorithm::kSparkDirectVelocityQp ||
         algorithm == IkAlgorithm::kSparkPoseVelocityQp ||
         algorithm == IkAlgorithm::kSparkUpperQpoasesVelocityQp ||
         usesSparkOtgConsistentVelocityQp(algorithm) ||
         usesSparkFeedforwardVelocityQp(algorithm);
}

inline bool usesSparkUpperQpoases(IkAlgorithm algorithm) noexcept {
  return algorithm == IkAlgorithm::kSparkUpperQpoasesDirect ||
         algorithm == IkAlgorithm::kSparkUpperQpoasesVelocityQp ||
         usesSparkOtgConsistentVelocityQp(algorithm) ||
         usesSparkFeedforwardVelocityQp(algorithm);
}

inline bool usesSparkUpperQpoasesDirect(IkAlgorithm algorithm) noexcept {
  return algorithm == IkAlgorithm::kSparkUpperQpoasesDirect;
}

inline bool usesSparkGuidance(IkAlgorithm algorithm) noexcept {
  return usesSparkVelocityQp(algorithm) ||
         usesSparkUpperQpoasesDirect(algorithm);
}

inline bool usesHierarchicalVelocityQp(IkAlgorithm algorithm) noexcept {
  return algorithm == IkAlgorithm::kHierarchicalQp ||
         usesSparkVelocityQp(algorithm);
}
enum class ControlLevel { kVelocity, kAcceleration };

struct ControllerConfig {
  double rate_hz{1000.0};
  bool model_state_only{false};
  bool initial_posture_enabled{false};
  Vec7 initial_left_q_rad{Vec7::Zero()};
  Vec7 initial_right_q_rad{Vec7::Zero()};
};

Vec7 configuredInitialPosture(const ControllerConfig& config,
                              const ArmLimits& limits, ArmSide side);

struct CartesianServoConfig {
  Eigen::Vector3d kp_position{Eigen::Vector3d::Constant(4.0)};
  Eigen::Vector3d kp_orientation{Eigen::Vector3d::Constant(3.0)};
  bool adaptive_gain_enabled{false};
  Eigen::Vector3d kp_position_near{Eigen::Vector3d::Constant(4.0)};
  Eigen::Vector3d kp_orientation_near{Eigen::Vector3d::Constant(3.0)};
  double position_gain_transition_start_m{0.02};
  double position_gain_transition_end_m{0.22};
  double orientation_gain_transition_start_rad{0.10};
  double orientation_gain_transition_end_rad{0.50};
  double kff_linear{0.0};
  double kff_angular{0.0};
  double feedforward_filter_cutoff_hz{15.0};
  double prediction_horizon_seconds{0.015};
  double target_timeout_seconds{0.075};
  double max_linear_velocity{0.30};
  double max_angular_velocity{1.00};
};

struct CartesianOtgConfig {
  bool enabled{false};
  bool translation_position_mode{false};
  bool translation_tracking_enabled{false};
  double translation_tracking_gain{30.0};
  double translation_stationary_velocity_threshold{1.0e-4};
  bool translation_prediction_enabled{false};
  double translation_prediction_horizon_seconds{0.015};
  bool stationary_hold_enabled{false};
  double stationary_hold_dwell_seconds{0.15};
  double translation_hold_enter_velocity_m_s{0.06};
  double translation_hold_exit_velocity_m_s{0.12};
  double translation_hold_exit_position_error_m{0.008};
  double orientation_hold_enter_velocity_rad_s{0.12};
  double orientation_hold_exit_velocity_rad_s{0.25};
  double orientation_hold_exit_error_rad{0.035};
  double translation_velocity_max{1.0};
  double translation_acceleration_max{3.0};
  double translation_jerk_max{20.0};
  double angular_velocity_max{3.0};
  double angular_acceleration_max{10.0};
  double angular_jerk_max{60.0};
  double orientation_gain{10.0};
  double orientation_settle_error_rad{1.0e-5};
  double angular_settle_velocity_rad_s{1.0e-4};
  double angular_settle_acceleration_rad_s2{1.0e-3};
};

struct CartesianOtgDynamicLimits {
  double translation_velocity_max{1.0};
  double translation_acceleration_max{3.0};
  double translation_jerk_max{20.0};
  double angular_velocity_max{3.0};
  double angular_acceleration_max{10.0};
  double angular_jerk_max{60.0};
};

struct CartesianOtgControlProfilesConfig {
  bool enabled{false};
  CartesianOtgDynamicLimits velocity;
  CartesianOtgDynamicLimits acceleration;
};

struct CartesianAccelerationServoConfig {
  double kp_position{100.0};
  double kd_position{20.0};
  double kp_orientation{70.0};
  double kd_orientation{16.0};
  double linear_limit{6.0};
  double angular_limit{20.0};
};

struct AccelerationQpConfig {
  double slack_weight_position{1.0e4};
  double slack_weight_orientation{3.0e3};
  double slack_linear_scale{5.0};
  double slack_angular_scale{15.0};
  double regularization{5.0e-4};
  double jerk_weight{2.0e-3};
  double posture_weight{5.0e-4};
  double posture_kp{0.5};
  double posture_kd{0.3};
  double arm_angle_weight{10.0};
  bool task_scaling_enabled{false};
  double task_scaling_min_position{0.0};
  double task_scaling_min_orientation{0.0};
  double task_scaling_weight_position{100.0};
  double task_scaling_weight_orientation{100.0};
  double equality_tolerance{1.0e-8};
};

struct JointAccelerationLimitConfig {
  double margin_rad{0.05};
  double velocity_scale{1.0};
  Vec7 max_acceleration_rad_s2{Vec7::Constant(20.0)};
  Vec7 braking_acceleration_rad_s2{Vec7::Constant(15.0)};
  bool hard_jerk_enabled{false};
  Vec7 max_jerk_rad_s3{Vec7::Constant(200.0)};
};

struct QpConfig {
  SolverBackend solver{SolverBackend::kQpoases};
  double position_weight{1.0};
  double orientation_weight{0.5};
  double regularization{1e-5};
  double nominal_weight{1e-3};
  double nominal_gain{0.2};
};

struct HierarchicalQpConfig {
  double lambda_reg{1e-4};
  double posture_weight{1e-3};
  double wrist_posture_weight_scale{1.0};
  double continuity_weight{1e-3};
  double jerk_weight{0.0};
  double nominal_gain{0.2};
  double arm_angle_weight{0.0};
  bool task_scaling_enabled{false};
  double task_scaling_min_position{0.75};
  double task_scaling_min_orientation{0.75};
  double task_scaling_weight_position{8000.0};
  double task_scaling_weight_orientation{3000.0};
  double slack_weight_position{1e4};
  double slack_weight_orientation{3e3};
  double slack_position_scale{1.0};
  double slack_orientation_scale{1.0};
  double equality_tolerance{1e-8};
};

struct DlsConfig {
  double damping{1e-3};
  double nominal_gain{0.2};
};

struct IterativeDlsConfig {
  bool posture_reference_enabled{false};
  int max_iterations{20};
  double position_tolerance_m{5.0e-4};
  double orientation_tolerance_rad{1.0e-2};
  double position_gain{1.0};
  double orientation_gain{1.0};
  double minimum_damping{1.0e-6};
  double maximum_damping{0.3};
  double singular_value_threshold{0.05};
  double maximum_step_norm_rad{0.35};
  double maximum_joint_step_rad{0.20};
  double minimum_merit_improvement{1.0e-10};
  double nominal_posture_gain{0.05};
  double arm_angle_gain{0.025};
  double maximum_goal_step_rad{0.05};
  double posture_reference_margin_rad{0.20};
};

struct SparkUpperQpoasesConfig {
  int calibration_frames{30};
  double minimum_scale{0.5};
  double maximum_scale{2.0};
  int stage1_max_iterations{10};
  int stage2_max_iterations{10};
  double convergence_delta{1.0e-4};
  double integration_step{1.0};
  double target_blend_seconds{0.15};
  double maximum_joint_rotation_jump_rad{0.75};
  double ik_cycle_budget_seconds{0.0045};
  double trust_region_rad{0.20};
  double damping{0.5};
  double joint_limit_margin_rad{0.05};
  double stage1_upper_direction_weight{5.0};
  double stage1_forearm_direction_weight{5.0};
  double stage1_palm_orientation_weight{5.0};
  double stage2_upper_direction_weight{5.0};
  double stage2_forearm_direction_weight{5.0};
  double stage2_palm_orientation_weight{5.0};
  double stage2_elbow_position_weight{5.0};
  double stage2_wrist_position_weight{5.0};
  double stage2_palm_position_weight{10.0};
  double posture_weight{2.0e-2};
  double joint_reference_weight{3.0e4};
  double joint_reference_attack_seconds{0.8};
  double joint_reference_release_seconds{0.4};
  double joint_reference_smoothness_weight{3.0e3};
  double posture_position_gain{4.0};
  double otg_position_tolerance_m{2.0e-3};
  double otg_orientation_tolerance_rad{2.0e-2};
  double otg_continuity_weight{1.0};
  bool enforce_hard_jerk_bounds{true};
};

struct SparkFeedforwardVelocityQpConfig {
  bool enabled{true};
  double alpha{0.70};
  double beta{0.16};
  int dt_median_window{31};
  double dt_min_ratio{0.5};
  double dt_max_ratio{1.5};
  double maximum_joint_jump_rad{0.35};
  double stale_velocity_decay_seconds{0.02};
  double source_stationary_velocity_rad_s{0.002};
  double velocity_reversal_decay{0.7225};
  double velocity_stationary_decay{0.9025};
  double target_braking_acceleration_scale{0.75};
  double palm_twist_filter_alpha{1.0};
  double palm_twist_lowpass_cutoff_hz{0.8};
  double reference_velocity_scale{1.0};
  double reference_acceleration_scale{1.0};
  double reference_jerk_scale{1.0};
  double reference_position_gain{10.0};
  double position_feedforward_gain{1.0};
  double orientation_feedforward_gain{1.0};
  double position_high_frequency_feedforward_gain{0.0};
  double orientation_high_frequency_feedforward_gain{0.0};
  double joint_position_gain{4.0};
  double attack_seconds{0.04};
  double release_seconds{0.05};
};

struct SparkHeadroomFeedforwardVelocityQpConfig {
  bool enabled{true};
  double joint_reference_smoothness_weight{3.0e3};
  double joint_reference_jerk_smoothness_weight{0.0};
  double low_headroom{0.10};
  double full_headroom{0.30};
  double reduction_time_seconds{0.020};
  double recovery_time_seconds{0.100};
  double jerk_usage_attack_seconds{0.050};
  double jerk_usage_release_seconds{0.150};
  double task_scaling_min_position{0.75};
  double task_scaling_min_orientation{0.75};
  double stationary_reference_hold_linear_velocity_m_s{0.01};
  double stationary_reference_hold_angular_velocity_rad_s{0.05};
  bool settled_hold_enabled{true};
  double settled_hold_dwell_seconds{0.30};
  double settled_hold_headroom_enter{0.05};
};

struct DlsPostureRuckigConfig {
  bool enabled{true};
  double velocity_scale{0.6};
  Vec7 max_velocity_rad_s{
      (Vec7() << 0.8, 0.8, 1.0, 1.0, 1.2, 1.2, 1.2).finished()};
  Vec7 max_acceleration_rad_s2{
      (Vec7() << 7.854, 7.854, 15.708, 15.708, 15.708, 15.708,
       15.708).finished()};
  Vec7 max_jerk_rad_s3{
      (Vec7() << 600.0, 600.0, 1000.0, 1000.0, 1000.0, 1000.0,
       1000.0).finished()};
  double validation_tolerance{1.0e-8};
};

struct JointLimitConfig {
  double margin_rad{0.05};
  double velocity_scale{0.20};
  Vec7 max_acceleration_rad_s2{Vec7::Constant(1.0e6)};
  Vec7 braking_acceleration_rad_s2{Vec7::Constant(1.0e6)};
  bool hard_jerk_enabled{false};
  Vec7 max_jerk_rad_s3{Vec7::Constant(1.0e6)};
};

struct OsqpConfig {
  int max_iterations{100};
  double absolute_tolerance{1e-8};
  double relative_tolerance{1e-8};
  bool polishing{false};
};

struct QpoasesConfig {
  int max_working_set_recalculations{50};
  double cpu_time_limit_seconds{0.0005};
};

struct SafetyConfig {
  double bound_tolerance{1e-8};
  double hessian_eigenvalue_tolerance{1e-10};
  double max_target_position_step{0.05};
  double max_target_orientation_step{0.20};
  double reference_tracking_warn_rad{1.0e6};
  double reference_tracking_stop_rad{2.0e6};
  double velocity_reference_tracking_warn_rad_s{1.0e6};
  double velocity_reference_tracking_stop_rad_s{2.0e6};
};

struct TrajectoryConfig {
  double circle_radius{0.06};
  double figure_eight_width{0.08};
  double figure_eight_height{0.05};
  double angular_amplitude{0.35};
  double frequency_hz{0.10};
};

struct PicoTeleopConfig {
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
};

struct ArmAngleConfig {
  bool enabled{true};
  bool nullspace_only{false};
  double kp_velocity{32.0};
  double max_velocity_rad_s{24.0};
  double kp_acceleration{80.0};
  double kd_acceleration{18.0};
  double max_acceleration_rad_s2{60.0};
  double minimum_radius_m{0.015};
  double full_weight_radius_m{0.050};
  // Keep the elbow plane above this observable radius when branch locking is
  // active. Defaults to the legacy geometric minimum in non-PICO profiles.
  double branch_lock_radius_m{0.015};
  double reference_rate_limit_rad_s{8.0};
  double reference_projection_hold_enter{0.05};
  double reference_projection_hold_exit{0.10};
  double error_branch_hysteresis_rad{0.20};
  // Keep the effective skeletal arm-direction reference inside a trackable
  // angular neighborhood of the modeled elbow plane. This prevents an
  // infeasible secondary task from winding up at the +/-pi branch cut.
  bool reference_governor_enabled{false};
  double reference_governor_enter_error_rad{1.0};
  double reference_governor_exit_error_rad{0.5};
  double reference_governor_tracking_error_rad{0.15};
  // Preserve the true PICO reference and constrain only model arm-angle
  // motion when the tracking error approaches this invariant envelope.
  bool tracking_envelope_enabled{false};
  double tracking_envelope_max_error_rad{1.50};
  double tracking_envelope_gain{8.0};
  double continuity_kp_velocity{2.0};
  double continuity_max_velocity_rad_s{0.8};
  double continuity_kp_acceleration{8.0};
  double continuity_kd_acceleration{4.0};
  double continuity_max_acceleration_rad_s2{8.0};
  double continuity_weight_scale{30.0};
  double joint_limit_soft_margin_rad{0.30};
  double joint_limit_recovery_gain_rad_s_per_rad{2.0};
  // Finite-distance branch protection used when the elbow-plane angle is
  // unobservable near a straight-arm configuration.
  bool branch_lock_enabled{true};
  double branch_lock_kp_velocity{8.0};
  double branch_lock_kp_acceleration{100.0};
  double branch_lock_kd_acceleration{20.0};
};

struct UpperArmOutwardConfig {
  double minimum_outward_distance_m{0.0};
  double velocity_gain{8.0};
  double acceleration_kp{100.0};
  double acceleration_kd{20.0};
};

struct QpIkConfig {
  ControllerConfig controller;
  ControlLevel control_level{ControlLevel::kVelocity};
  IkAlgorithm ik_algorithm{IkAlgorithm::kHierarchicalQp};
  CartesianServoConfig cartesian_servo;
  CartesianOtgConfig cartesian_otg;
  CartesianOtgControlProfilesConfig cartesian_otg_control_profiles;
  CartesianAccelerationServoConfig cartesian_acceleration;
  AccelerationQpConfig acceleration_qp;
  JointAccelerationLimitConfig joint_acceleration_limits;
  QpConfig qp;
  HierarchicalQpConfig hierarchical_qp;
  DlsConfig dls;
  IterativeDlsConfig iterative_dls;
  SparkUpperQpoasesConfig spark_upper_qpoases;
  SparkFeedforwardVelocityQpConfig spark_feedforward_velocity_qp;
  SparkHeadroomFeedforwardVelocityQpConfig
      spark_headroom_feedforward_velocity_qp;
  DlsPostureRuckigConfig dls_posture_ruckig;
  JointLimitConfig joint_limits;
  OsqpConfig osqp;
  QpoasesConfig qpoases;
  SafetyConfig safety;
  TrajectoryConfig trajectories;
  PicoTeleopConfig pico_teleop;
  ArmAngleConfig arm_angle;
  UpperArmOutwardConfig upper_arm_outward;
};

QpIkConfig loadConfig(const std::string& path);
CartesianOtgConfig cartesianOtgConfigForControlLevel(
    const QpIkConfig& config, ControlLevel level);
std::string toString(SolverBackend backend);
std::string toString(IkAlgorithm algorithm);
std::string toString(ControlLevel level);

}  // namespace tianji_qp_ik
