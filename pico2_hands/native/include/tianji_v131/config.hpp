// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#pragma once

#include "tianji_v131/realtime_control.hpp"
#include "tianji_v131/types.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

namespace tianji_v131 {

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
  // Keep this opt-in extension at the end so existing algorithm ordinals
  // remain stable for telemetry/API consumers that persist the enum value.
  kPicoEeDls,
  // Independent EE-only reproduction of dexhand_deploy's pinocchio_qp.
  // Keep new values appended for telemetry/API ordinal compatibility.
  kPicoEeDexhandQp,
  // EE-only v1.3.1 Cartesian OTG followed by the modern single-stage
  // constrained velocity QP. Keep appended for persisted telemetry ordinals.
  kPicoEeV131VelocityQp,
  // Opt-in v1.3.1 velocity QP with a strictly Cartesian-nullspace posture,
  // branch-continuity, and temporal-smoothing objective.
  kPicoEeV131VelocityQpAdaptiveNullspace,
};

inline bool usesSparkHeadroomFeedforwardVelocityQp(
    IkAlgorithm algorithm) noexcept {
  return algorithm ==
         IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp;
}

inline bool usesPicoEeDls(IkAlgorithm algorithm) noexcept {
  return algorithm == IkAlgorithm::kPicoEeDls;
}

inline bool usesPicoEeDexhandQp(IkAlgorithm algorithm) noexcept {
  return algorithm == IkAlgorithm::kPicoEeDexhandQp;
}

inline bool usesPicoEeV131VelocityQp(IkAlgorithm algorithm) noexcept {
  return algorithm == IkAlgorithm::kPicoEeV131VelocityQp ||
         algorithm ==
             IkAlgorithm::kPicoEeV131VelocityQpAdaptiveNullspace;
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

enum class SequentialTaskHqpMode {
  kLegacy,
  kShadow,
  kActive,
};

enum class SequentialCartesianTaskLayout {
  kSplitPositionOrientation,
  kCoupledCartesian,
};

enum class SecondaryTaskOrder {
  kPostureThenSmoothness,
  kSmoothnessThenPosture,
};

enum class FullJointTemporalEnvelopeConflictPolicy : std::uint8_t {
  kRelaxToSafety,
};

struct FullJointTemporalEnvelopeConfig {
  // The envelope is deliberately opt-in.  A disabled profile is a no-op and
  // preserves the existing safety bounds byte-for-byte.
  bool enabled{false};
  Vec7 max_acceleration_rad_s2{Vec7::Constant(1.0e6)};
  Vec7 max_jerk_rad_s3{Vec7::Constant(1.0e6)};
  FullJointTemporalEnvelopeConflictPolicy conflict_policy{
      FullJointTemporalEnvelopeConflictPolicy::kRelaxToSafety};
};

struct P1SmoothnessConfig {
  // Optional accepted-command-history jerk regularization for the Cartesian
  // P1 objective.  A disabled profile permits the zero default weight so the
  // existing command remains unchanged.
  bool enabled{false};
  double jerk_weight{0.0};
  Vec7 jerk_joint_scale{Vec7::Ones()};

  bool isValid() const noexcept {
    const bool weight_valid = std::isfinite(jerk_weight) &&
                              (enabled ? jerk_weight > 0.0
                                       : jerk_weight >= 0.0);
    return weight_valid && jerk_joint_scale.allFinite() &&
           (jerk_joint_scale.array() > 0.0).all();
  }
};

enum class CollisionMode {
  kDisabled,
  kDiagnostic,
  kHard,
};

enum class ReplayArmSelection {
  kDual,
  kLeft,
  kRight,
};

enum class RedundancyMpcMode : std::uint8_t {
  kDisabled,
  kShadow,
  kActive,
};

enum class RedundancyMpcPredictionModel : std::uint8_t {
  kAnalyticTaskScaleV1,
  kBoxConstrainedProjectorV2,
};

enum class RedundancyMpcFailureReason : std::uint8_t {
  kNone,
  kNonFiniteState,
  kInvalidDt,
  kLowNullspaceObservability,
  kNullspaceResidual,
  kFkFailure,
  kCollisionBackendFailure,
  kNonFiniteCost,
  kBudgetTimeout,
  kInvalidConfig,
  kTaskVelocityInfeasible,
  kExactCandidateSubsetInconclusive,
  kArmRateEqualityInfeasible,
  kProjectorSingularKkt,
  kProjectorIterationLimit,
  kProjectorResidualFailure,
  kBranchCrossing,
  kTaskResidualExceeded,
  kCollisionStopDistance,
};

struct RedundancyMpcConfig {
  RedundancyMpcMode mode{RedundancyMpcMode::kShadow};
  RedundancyMpcPredictionModel prediction_model{
      RedundancyMpcPredictionModel::kAnalyticTaskScaleV1};
  int horizon_steps{20};
  double horizon_dt_seconds{0.005};
  // Number of 200 Hz controller cycles between predictive updates.  A value
  // of two schedules each arm at 100 Hz while keeping the control loop at
  // 200 Hz.
  int update_period_cycles{1};
  double prediction_budget_seconds{0.00025};
  double projector_velocity_regularization{0.05};
  double projector_damping{1.0e-8};
  double projector_equality_tolerance{1.0e-8};
  double projector_bound_tolerance{1.0e-9};
  double projector_kkt_tolerance{1.0e-7};
  std::uint32_t projector_max_iterations{24U};
  double task_linear_reference_m_s{1.0};
  double task_angular_reference_rad_s{3.0};
  double dls_damping{1.0e-3};
  double minimum_nullspace_observability{1.0e-6};
  double maximum_nullspace_residual{1.0e-5};
  double arm_angle_rate_max_rad_s{0.8};
  double arm_angle_acceleration_max_rad_s2{8.0};
  double arm_angle_anchor_scale_rad{1.0};
  double arm_angle_rate_scale_rad_s{1.0};
  Vec7 acceleration_scale_rad_s2{Vec7::Constant(20.0)};
  Vec7 jerk_scale_rad_s3{Vec7::Constant(200.0)};
  Vec7 prediction_q_error_scale_rad{Vec7::Constant(0.05)};
  double prediction_arm_rate_error_scale_rad_s{0.8};
  double prediction_q_error_threshold{0.5};
  double prediction_arm_rate_error_threshold{0.5};
  std::uint32_t active_min_valid_cycles{200U};
  double fallback_decay{0.85};
  double joint_soft_margin_rad{0.30};
  double joint_barrier_power{2.0};
  double sigma_safe{0.05};
  double collision_safe_distance_m{0.05};
  double branch_soft_delta_rad{0.35};
  double weight_rate_delta{1.0};
  double weight_rate_second_delta{1.0};
  double weight_nominal_rate_tracking{0.0};
  double weight_arm_angle_anchor{0.10};
  double weight_acceleration{0.10};
  double weight_jerk{0.10};
  double weight_joint_margin{10.0};
  double weight_sigma{5.0};
  double weight_collision{10.0};
  double weight_branch{5.0};
  double weight_task_scale{25.0};
  double weight_task_orthogonal_residual{25.0};
  double weight_task_total_residual{25.0};
  double minimum_active_task_scale{0.70};
  double maximum_active_task_orthogonal_residual{0.35};
  double maximum_active_task_total_residual{0.50};
  double candidate_cost_hysteresis{0.01};
  std::uint32_t prediction_error_exit_cycles{3U};
  double collision_stop_distance_m{0.01};
  std::uint8_t collision_adaptive_max_samples{9U};
  double async_max_state_drift_rad{0.03};
  double async_max_task_twist_drift_normalized{0.20};
  // Opt-in exact collision queries for the predictive redundancy rollout.
  // Disabled by default so existing Shadow/Active profiles remain unchanged.
  bool collision_prediction_enabled{false};
  bool collision_cost_enabled{false};
  bool require_exact_evaluator_in_active{true};
  bool async_worker_enabled{false};
  double async_worker_prediction_budget_seconds{0.020};
  std::uint32_t async_result_max_age_cycles{6U};
  std::uint8_t collision_exact_candidate_limit{2U};
  std::array<std::uint8_t, 5> collision_sample_steps{
      {1U, 5U, 10U, 15U, 20U}};
  std::uint32_t collision_audit_period_updates{100U};
};

struct SequentialTaskHqpConfig {
  SequentialTaskHqpMode mode{SequentialTaskHqpMode::kLegacy};
  SequentialCartesianTaskLayout cartesian_task_layout{
      SequentialCartesianTaskLayout::kSplitPositionOrientation};
  CollisionMode collision_mode{CollisionMode::kDisabled};
  std::string collision_urdf_path;
  // Optional explicit allowed-collision matrix entries. Each entry is an
  // order-independent `geometry_a|geometry_b` name pair that is allowed to
  // touch and therefore excluded from safety rows; empty means no additional
  // exclusions beyond the backend's normal filtered candidate set.
  std::vector<std::string> collision_allowed_pairs;
  std::size_t collision_max_geometry_pairs{256U};
  bool collision_exclude_adjacent{true};
  bool collision_include_environment{false};
  // Cross-arm pairs are opt-in because the existing 7-DoF safety seam cannot
  // express relative two-arm velocity rows and they are more expensive.
  bool collision_include_inter_arm_diagnostic{false};
  double collision_activation_distance_m{0.05};
  double collision_release_distance_m{0.06};
  double collision_minimum_distance_m{0.01};
  double collision_damping_gain{10.0};
  double collision_normal_continuity_cosine{0.0};
  double collision_maximum_distance_jump_m{0.05};
  double collision_maximum_row_bound{1.0e6};
  double arm_budget_seconds{0.002};
  double combined_budget_seconds{0.004};
  double cycle_deadline_seconds{0.005};
  // Active-only recovery for a deadline-expired P0. Disabled by default so
  // existing SPARK, Legacy, Shadow, and Active profiles retain their behavior.
  bool p0_deadline_safe_brake_enabled{false};
  double shadow_arm_budget_seconds{0.001};
  double shadow_combined_budget_seconds{0.002};
  double preservation_epsilon{1.0e-6};
  // Dimensionless row factors used only by the Sequential HQP task adapter.
  // They normalize the physical units of each task family before the solver
  // forms its least-squares objective; the Legacy Weighted QP is untouched.
  double task_position_scale{1.0};
  double task_orientation_scale{1.0};
  double task_posture_scale{1.0};
  double task_smoothness_scale{1.0};
  // Active-only temporal regularization for the new sequential HQP.  Zero
  // keeps Legacy and Shadow numerically compatible with their prior output.
  double continuity_weight{0.0};
  // Optional primary-task temporal regularization.  It is independent from
  // `continuity_weight`, which is used by the redundant secondary tasks.
  // -1 (the compatibility default) inherits the legacy continuity weight;
  // an explicit zero preserves the current-cycle Cartesian demand exactly.
  double cartesian_continuity_weight{-1.0};
  // V1.2-only hard whole-joint temporal comfort envelope.  This nested
  // contract is intentionally absent from legacy/SPARK profiles.
  FullJointTemporalEnvelopeConfig full_joint_temporal_envelope;
  bool replay_enabled{false};
  std::string replay_csv_path;
  ReplayArmSelection replay_arm{ReplayArmSelection::kDual};
  Vec7 replay_q_nominal_left{Vec7::Zero()};
  Vec7 replay_q_nominal_right{Vec7::Zero()};
  bool replay_loop{false};
  double replay_source_timeout_seconds{0.100};
  double pico_to_replay_blend_seconds{0.300};
  double replay_to_pico_blend_seconds{0.200};
  // Kept for compatibility with early V1.1 profiles; new profiles should
  // use the directional transition durations above.
  double replay_blend_seconds{0.100};
  double replay_max_gap_seconds{0.100};
  // Optional stronger continuity preference for the identity posture stage.
  // Kept at the end to preserve the source ordering of the original public
  // aggregate fields.
  double posture_continuity_weight{0.0};
  // When enabled, the replay profile is an explicit EE-pose-only source and
  // must fail closed instead of accepting PICO/skeleton input.
  bool ee_pose_replay_only{false};
  // Optional stateful conditioning of replay-derived SE(3) feedforward.  It
  // is disabled by default so existing profiles remain command-transparent.
  bool ee_feedforward_filter_enabled{false};
  double ee_feedforward_filter_cutoff_hz{5.0};
  double ee_feedforward_low_gain{1.0};
  double ee_feedforward_high_gain{0.0};
  bool posture_task_enabled{true};
  SecondaryTaskOrder secondary_task_order{
      SecondaryTaskOrder::kPostureThenSmoothness};
  // -1 preserves the V1.1 policy/scaling composition. Positive values define
  // the final orientation/position least-squares row-weight ratio.
  double unified_orientation_to_position_weight_ratio{-1.0};
  // Soft P3 joint-velocity continuity row weight.  This is applied only to
  // the Sequential HQP smoothness task; Legacy/Shadow remain command-
  // compatible when the default value is used.
  double smoothness_task_weight{0.05};
  // Preserve the historical one-step acceleration prediction by default;
  // stable EE replay profiles may anchor P3 directly to qdot_prev.
  bool smoothness_predictive_target{true};
  // Optional Active-only arm-angle continuity task. It is appended after
  // Cartesian rows only when the posture slot is unused.
  bool arm_angle_task_enabled{false};
  double arm_angle_task_weight{1.0};
  // Comparison switch for the redundant secondary objective.  False keeps
  // the arm-angle scalar as its own Active level (A/B experiment A); true
  // fuses it with the smoothness rows at one weighted secondary level
  // (experiment B).  The default is false so enabling the new task alone
  // does not silently change its level semantics.
  bool arm_angle_fuse_with_smoothness{false};
  // Optional same-level P2 arm-angle rate continuity.  When enabled, the
  // provider adds a second weighted J_arm row around the previous command;
  // this lets the skeleton-angle target and minimum-change objective trade
  // off without allowing P3 to override the preserved P2 task.
  bool arm_angle_rate_smoothing_enabled{false};
  double arm_angle_rate_smoothing_weight{1.0};
  // V1.2-only reduced one-dimensional nullspace objective. Legacy and SPARK
  // profiles remain unchanged unless this explicit flag is enabled.
  bool arm_angle_nullspace_selection_enabled{false};
  // V1.2-only Active Cartesian beta selector. It is an explicit opt-in and
  // has no effect on Legacy, Shadow, or SPARK paths.
  bool cartesian_task_scaling_enabled{false};
  double cartesian_task_scaling_min_position{0.75};
  double cartesian_task_scaling_min_orientation{0.75};
  double cartesian_task_scaling_weight_position{8000.0};
  double cartesian_task_scaling_weight_orientation{3000.0};
  // Optional accepted-command-history jerk preference for the Cartesian P1
  // objective.  It is appended to preserve aggregate initialization order.
  P1SmoothnessConfig p1_smoothness;
  // Explicit Active-only Cartesian residual budget.  Disabled/zero keeps the
  // pre-existing strict P1 preservation behavior.
  bool cartesian_degradation_budget_enabled{false};
  double cartesian_degradation_ratio{0.0};
  double cartesian_degradation_position_floor{1.0e-6};
  double cartesian_degradation_orientation_floor{1.0e-6};
  // Maximum lower-level deviation from the P2 arm-angle optimum, expressed
  // in rad/s at the velocity IK level.
  double arm_angle_deviation_bound_rad_s{0.0};
};

struct ControllerConfig {
  double rate_hz{1000.0};
  bool model_state_only{false};
  bool initial_posture_enabled{false};
  Vec7 initial_left_q_rad{Vec7::Zero()};
  Vec7 initial_right_q_rad{Vec7::Zero()};
  // Optional Pinocchio URDF used by the direct PICO EE-DLS hot path. An
  // empty value preserves the MuJoCo evaluator fallback for other profiles.
  std::string pico_ee_dls_kinematics_urdf_path;
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

// Optional Cartesian target conditioning used by dexhand_deploy before its
// velocity-level IK.  It is deliberately independent from the existing
// TargetManager safety increment: this reproduces dexhand's stateful
// workspace compression and speed/acceleration limiting and is disabled by
// default so existing profiles remain command-transparent.
struct DexhandPreIkConditioningConfig {
  bool enabled{false};
  double rate_hz{60.0};
  Eigen::Vector3d translation_gain{Eigen::Vector3d::Ones()};
  double rotation_gain{1.0};
  Eigen::Vector3d workspace_relative_radii_m{0.42, 0.38, 0.38};
  double workspace_soft_zone_ratio{0.90};
  double maximum_linear_speed_m_s{0.36};
  double maximum_angular_speed_rad_s{1.55};
  double maximum_linear_acceleration_m_s2{3.5};
  double maximum_angular_acceleration_rad_s2{9.0};
};

// Dedicated settings for the EE-only velocity-level QP port from
// dexhand_deploy.  It intentionally does not reuse QpConfig or
// DlsPostureRuckigConfig: the former selects the existing qpOASES backend and
// the latter is a position-interface post-processor.
struct DexhandVelocityQpConfig {
  DexhandPreIkConditioningConfig pre_ik_conditioning;
  // When enabled, the controller may supply a Cartesian-reference servo
  // command (OTG twist plus pose feedback) directly to the QP.  False keeps
  // the original Dexhand pose-error/time-constant law unchanged.
  bool cartesian_reference_servo_enabled{false};
  bool dexhand_faithful_nominal{false};
  Vec7 nominal_left{
      (Vec7() << 1.10, -0.60, -1.52, -1.10, 0.0, 0.0, 0.0).finished()};
  Vec7 nominal_right{
      (Vec7() << -1.10, -0.60, 1.52, -1.10, 0.0, 0.0, 0.0).finished()};
  double position_time_constant_s{0.30};
  double orientation_time_constant_s{0.40};
  double max_linear_speed_m_s{0.25};
  double max_angular_speed_rad_s{1.0};
  Vec7 qp_joint_velocity_limits_rad_s{
      (Vec7() << 0.9599310886, 0.9599310886, 0.9599310886,
       0.9599310886, 0.9599310886, 0.9599310886, 0.9599310886)
          .finished()};
  double position_weight{1.0};
  double orientation_weight{0.45};
  double velocity_regularization_weight{0.02};
  double continuity_weight{0.06};
  double posture_weight{0.008};
  double posture_time_constant_s{2.5};
  double maximum_joint_step_rad{0.00596902599};
  double joint_limit_activation_margin_rad{0.2617993878};
  double joint_limit_velocity_damper_gain{4.0};
  double singular_value_threshold{0.05};
  double singularity_critical_threshold{0.015};
  double singularity_orientation_scale{0.15};
  double singularity_posture_multiplier{8.0};
  double singularity_velocity_multiplier{4.0};
  double singularity_escape_weight{0.03};
  double singularity_escape_speed_rad_s{0.15};
  int max_active_set_iterations{48};
  double active_set_tolerance{1.0e-9};
  bool ruckig_enabled{false};
  Vec7 ruckig_max_velocity_rad_s{
      (Vec7() << 0.8, 0.8, 1.0, 1.0, 1.2, 1.2, 1.2).finished()};
  Vec7 ruckig_max_acceleration_rad_s2{
      (Vec7() << 7.854, 7.854, 15.708, 15.708, 15.708, 15.708, 15.708)
          .finished()};
  Vec7 ruckig_max_jerk_rad_s3{
      (Vec7() << 600.0, 600.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0)
          .finished()};
  double ruckig_validation_tolerance{1.0e-8};
};

// Opt-in modern velocity-QP variant matching the v1.3.1 EE front end.  This
// block is intentionally separate from both HierarchicalQpConfig and
// DexhandVelocityQpConfig so selecting it cannot silently change either
// existing solver's objective or hard velocity box.
struct PicoEeV131VelocityQpConfig {
  bool enabled{false};
  std::string kinematics_urdf_path;
  Vec7 zero_posture_left{Vec7::Zero()};
  Vec7 zero_posture_right{Vec7::Zero()};
  double posture_time_constant_seconds{2.5};
  double velocity_regularization_weight{1.0e-4};
  double continuity_weight{1.0e-3};
  double jerk_trend_weight{2.0e-2};
  double posture_weight{8.0e-3};
  double wrist_posture_weight_scale{1.0};
  double slack_weight_position{3.0e4};
  double slack_weight_orientation{3.0e4};
  double slack_position_scale{3.0};
  double slack_orientation_scale{12.0};
  double task_scaling_min_position{0.75};
  double task_scaling_min_orientation{0.75};
  double task_scaling_weight_position{8000.0};
  double task_scaling_weight_orientation{3000.0};
  double equality_tolerance{1.0e-8};
  double singular_value_threshold{0.05};
  double singularity_critical_threshold{0.015};
  double singularity_orientation_scale{0.15};
  double singularity_posture_multiplier{8.0};
  double singularity_velocity_multiplier{4.0};
  double finite_difference_rad{1.0e-4};
  double singularity_escape_speed_rad_s{0.15};
  double singularity_escape_weight{0.03};
  double gradient_update_rate_hz{50.0};
  double gradient_max_age_seconds{0.04};
  double gradient_configuration_tolerance_rad{0.05};
  double warm_start_reset_threshold_rad{0.05};
  bool remove_dexhand_speed_box{true};
  bool remove_dexhand_cartesian_caps{true};
  bool adaptive_nullspace_enabled{false};
  double nullspace_reference_weight{8.0e-3};
  double nullspace_continuity_weight{1.0e-3};
  double nullspace_jerk_weight{2.0e-2};
  double branch_weight{5.0e-3};
  double branch_safe_margin_rad{0.15};
  double branch_soft_zone_rad{0.40};
  double branch_gain{2.0};
  double branch_health_floor{0.0};
  double nullspace_max_velocity_rad_s{0.30};
  double nullspace_max_acceleration_rad_s2{1.5};
  double nullspace_max_jerk_rad_s3{15.0};
  double position_error_full_health_m{0.05};
  double position_error_zero_health_m{0.20};
  double orientation_error_full_health_rad{0.15};
  double orientation_error_zero_health_rad{0.50};
  double task_scale_zero_health{0.80};
  double task_scale_full_health{0.95};
  bool vector_transport_enabled{false};
  double vector_continuity_weight{0.10};
  double vector_acceleration_weight{5.0e-5};
  double vector_jerk_weight{5.0e-10};
  double basis_rotation_full_health_rad_s{2.0};
  double basis_rotation_zero_health_rad_s{8.0};
  FullJointTemporalEnvelopeConfig temporal_envelope;
};

inline Vec7 dexhandNominal(const DexhandVelocityQpConfig& config,
                           ArmSide side) noexcept {
  if (config.dexhand_faithful_nominal) {
    return Vec7::Zero();
  }
  return side == ArmSide::kLeft ? config.nominal_left : config.nominal_right;
}

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
  // Sequential HQP will not start another task level when less than this
  // amount remains in either its per-arm or absolute reservation.  Legacy
  // QP callers ignore the field.
  double level_min_start_budget_seconds{0.0001};
  // A solved task with a larger weighted RMS residual is retained as the
  // verified command but reported as kDegradedTask.
  double degraded_normalized_residual{1.0};
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

enum class PicoEeTargetSource {
  kPacketTargets,
  kMappedCorrectedPalm,
};

enum class PicoArmAngleReferenceSource {
  kPacketDirection,
  kMappedSkeletonPlane,
  kDisabled,
};

struct PicoTeleopConfig {
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
  PicoEeTargetSource ee_target_source{PicoEeTargetSource::kPacketTargets};
  PicoArmAngleReferenceSource arm_angle_reference_source{
      PicoArmAngleReferenceSource::kPacketDirection};
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
  // Explicit mapped-skeleton scalar targets are rate-limited only when this
  // V1.2 opt-in is enabled. Legacy packet-direction/SPARK profiles retain
  // their historical direct-target behavior.
  bool standard_target_rate_limit_enabled{false};
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
  RealtimeControlConfig realtime;
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
  DexhandVelocityQpConfig dexhand_velocity_qp;
  PicoEeV131VelocityQpConfig pico_ee_v131_velocity_qp;
  JointLimitConfig joint_limits;
  OsqpConfig osqp;
  QpoasesConfig qpoases;
  SafetyConfig safety;
  TrajectoryConfig trajectories;
  PicoTeleopConfig pico_teleop;
  ArmAngleConfig arm_angle;
  UpperArmOutwardConfig upper_arm_outward;
  SequentialTaskHqpConfig sequential_task_hqp;
  RedundancyMpcConfig redundancy_mpc;
};

QpIkConfig loadConfig(const std::string& path);
CartesianOtgConfig cartesianOtgConfigForControlLevel(
    const QpIkConfig& config, ControlLevel level);
std::string toString(SolverBackend backend);
std::string toString(IkAlgorithm algorithm);
std::string toString(ControlLevel level);

}  // namespace tianji_v131
