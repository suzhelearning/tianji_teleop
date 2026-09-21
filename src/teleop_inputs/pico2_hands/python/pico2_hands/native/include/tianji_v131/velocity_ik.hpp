// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#pragma once

#include "tianji_v131/config.hpp"
#include "tianji_v131/types.hpp"

#include <array>
#include <memory>
#include <string_view>

namespace tianji_v131 {

enum class BoundSource {
  kVelocity,
  kPosition,
  kAcceleration,
  kJerk,
  kBraking,
  kTemporalAcceleration,
  kTemporalJerk
};

struct JointVelocityBounds {
  Vec7 lower{Vec7::Zero()};
  Vec7 upper{Vec7::Zero()};
  std::array<BoundSource, kArmDof> lower_source{};
  std::array<BoundSource, kArmDof> upper_source{};
};

enum class JointVelocityPostureSource {
  kLegacyNullspace,
  kSparkSoftQp,
  kSparkJointReference,
  // The Cartesian feedforward has already been generated from this same
  // reference. Keep the joint target as a soft phase/posture objective only;
  // unlike kSparkJointReference it must not be injected into Vdes again.
  kSparkFeedforwardJointReference,
};

struct JointVelocityPostureTask {
  bool active{false};
  Vec7 target{Vec7::Zero()};
  double activation{1.0};
  double weight{0.0};
  double smoothness_weight{0.0};
  double jerk_smoothness_weight{0.0};
  JointVelocityPostureSource source{
      JointVelocityPostureSource::kLegacyNullspace};
};

// Per-cycle data supplied by the v1.3.1 EE-only controller.  It is kept out
// of JointVelocityPostureTask so the existing SPARK/arm-angle routes cannot
// accidentally consume the new singularity objective.
struct PicoEeV131QpTask {
  bool active{false};
  Vec7 zero_posture{Vec7::Zero()};
  Vec7 velocity_scale{Vec7::Ones()};
  // Selects the strict-nullspace objective shape even when the per-cycle
  // adaptive preference is unavailable.  This prevents a fail-safe cycle
  // from silently restoring the legacy whole-vector posture objective.
  bool adaptive_nullspace_mode{false};
  bool adaptive_nullspace_active{false};
  Vec7 nullspace_basis{Vec7::Zero()};
  double alpha_command{0.0};
  double alpha_previous{0.0};
  double alpha_trend{0.0};
  double nullspace_reference_weight{0.0};
  double sigma_min{0.0};
  double activation{0.0};
  Vec7 gradient{Vec7::Zero()};
  bool gradient_valid{false};
  double gradient_age_seconds{0.0};
  double requested_sigma_dot{0.0};
  bool escape_active{false};
};

struct DualArmJointVelocityPostureTasks {
  JointVelocityPostureTask left;
  JointVelocityPostureTask right;
};

struct JointVelocityNullspaceResult {
  Vec7 value{Vec7::Zero()};
  bool active{false};
  double scale{0.0};
  double correction_norm{0.0};
  double cartesian_residual{0.0};
};

struct ArmIkInput {
  Vec7 q_measured{Vec7::Zero()};
  Vec7 q_ref{Vec7::Zero()};
  Vec7 qdot_prev{Vec7::Zero()};
  Vec7 qddot_prev{Vec7::Zero()};
  Vec6 slack_prev{Vec6::Zero()};
  Mat67 jacobian{Mat67::Zero()};
  Vec6 desired_twist{Vec6::Zero()};
  ArmLimits limits;
  JointVelocityBounds bounds;
  ScalarJointTask arm_angle_task;
  JointPositionGuard arm_angle_position_guard;
  JointVelocityPostureTask posture_task;
  PicoEeV131QpTask pico_ee_v131_task;
  LinearJointConstraint linear_constraint;
  LinearJointConstraint branch_lock_constraint;
  double dt{0.005};
};

struct ArmIkResult {
  SolverStatus status{SolverStatus::kInvalidInput};
  Vec7 qdot{Vec7::Zero()};
  Vec6 slack{Vec6::Zero()};
  int iterations{0};
  double solve_time_us{0.0};
  double equality_residual{0.0};
  int active_position_bound_count{0};
  int active_velocity_bound_count{0};
  int active_acceleration_bound_count{0};
  int active_jerk_bound_count{0};
  int active_braking_bound_count{0};
  double qdot_max_ratio{0.0};
  double task_scale_position{1.0};
  double task_scale_orientation{1.0};
  Vec7 zero_posture_velocity{Vec7::Zero()};
  double zero_posture_cost{0.0};
  double sigma_min{0.0};
  double singularity_activation{0.0};
  double singularity_gradient_age_seconds{0.0};
  double requested_sigma_dot{0.0};
  double achieved_sigma_dot{0.0};
  double singularity_escape_cost{0.0};
  bool singularity_escape_active{false};
  bool warm_start_mismatch_reset{false};
  bool qp_cold_start{false};
  bool qp_cold_retry{false};
  std::string_view detail{"not_solved"};
};

class IArmVelocityIk {
 public:
  virtual ~IArmVelocityIk() = default;
  virtual ArmIkResult solve(const ArmIkInput& input) = 0;
  virtual void reset() = 0;
  virtual IkAlgorithm algorithm() const noexcept = 0;
};

JointVelocityBounds computeJointVelocityBounds(
    const Vec7& q_measured, const Vec7& qdot_previous,
    const Vec7& qddot_previous, const ArmLimits& limits,
    const JointLimitConfig& config, double dt);

JointVelocityNullspaceResult refinePostureVelocityInNullspace(
    const Vec7& primary, const Mat67& cartesian_jacobian,
    const JointVelocityPostureTask& task, const Vec7& lower,
    const Vec7& upper, double tolerance,
    const LinearJointConstraint& linear_constraint = {},
    const LinearJointConstraint& secondary_linear_constraint = {}) noexcept;

std::unique_ptr<IArmVelocityIk> makeArmVelocityIk(
    IkAlgorithm algorithm, const QpIkConfig& config);

}  // namespace tianji_v131
