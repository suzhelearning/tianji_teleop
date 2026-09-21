#pragma once

#include "tianji_qp_ik/types.hpp"

#include <Eigen/Core>

#include <string_view>

namespace tianji_qp_ik {

enum class ArmDirectionReferenceSource {
  kPico,
  kDefaultDown,
  kPrevious,
  kDegenerate,
};

enum class ArmAngleReferenceMode {
  kPico,
  kDefaultDown,
  kOutwardOnly,
  kPicoOutward,
};

struct ArmDirectionReference {
  bool valid{false};
  Eigen::Vector3d direction{Eigen::Vector3d::Zero()};
  ArmDirectionReferenceSource source{
      ArmDirectionReferenceSource::kDegenerate};
  // Optional target shoulder-to-wrist axis. When present, `direction` and
  // this axis define the target arm plane, while the robot arm angle is
  // measured independently from the fixed world reference plane.
  bool shoulder_to_wrist_axis_valid{false};
  Eigen::Vector3d shoulder_to_wrist_axis{Eigen::Vector3d::Zero()};
};

struct DualArmDirectionReferences {
  ArmDirectionReference left;
  ArmDirectionReference right;
};

DualArmDirectionReferences defaultArmDirectionReferences() noexcept;
std::string_view toString(ArmDirectionReferenceSource source) noexcept;
std::string_view toString(ArmAngleReferenceMode mode) noexcept;
ArmAngleReferenceMode armAngleReferenceModeFromString(std::string_view value);
ArmAngleReferenceMode toggleArmAngleReferenceMode(
    ArmAngleReferenceMode mode) noexcept;
bool usesPicoArmDirection(ArmAngleReferenceMode mode) noexcept;
bool usesContinuityArmDirection(ArmAngleReferenceMode mode) noexcept;
bool usesOutwardArmBarrier(ArmAngleReferenceMode mode) noexcept;
DualArmDirectionReferences selectArmDirectionReferences(
    ArmAngleReferenceMode mode, bool pico_live,
    const DualArmDirectionReferences& pico_references) noexcept;

class ArmDirectionReferenceManager {
 public:
  explicit ArmDirectionReferenceManager(double rate_limit_rad_s);

  DualArmDirectionReferences update(
      const DualArmDirectionReferences& requested, double dt);
  void reset() noexcept;

 private:
  ArmDirectionReference updateOne(const ArmDirectionReference& requested,
                                  const ArmDirectionReference& previous,
                                  bool initialized, double dt) const;

  double rate_limit_rad_s_{0.0};
  bool initialized_{false};
  DualArmDirectionReferences previous_;
};

struct ArmAngleGeometryInput {
  Eigen::Vector3d shoulder_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d elbow_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d wrist_position{Eigen::Vector3d::Zero()};
  Mat37 shoulder_position_jacobian{Mat37::Zero()};
  Mat37 elbow_position_jacobian{Mat37::Zero()};
  Mat37 wrist_position_jacobian{Mat37::Zero()};
};

struct ArmAngleTask {
  bool active{false};
  Vec7 jacobian{Vec7::Zero()};
  // Physical shortest signed error in [-pi, pi], for diagnostics.
  double error_rad{0.0};
  // Physical shortest-path S1 error used by controllers. Hard joint dynamic
  // bounds limit the one-cycle command change at the antipodal branch cut.
  double control_error_rad{0.0};
  // Standard swivel angles measured from the fixed reference plane around
  // each arm's own shoulder-to-wrist axis.
  double robot_angle_rad{0.0};
  double target_angle_rad{0.0};
  // Time derivative of the final projected reference around the current
  // shoulder-wrist axis. This excludes motion of the modeled arm itself.
  double reference_rate_rad_s{0.0};
  double radius_m{0.0};
  double activation{0.0};
  Eigen::Vector3d projected_reference{Eigen::Vector3d::Zero()};
  double requested_reference_projection_norm{0.0};
  double jacobian_norm{0.0};
  bool reference_projection_held{false};
  // True when the model-aware reference governor is preventing the effective
  // PICO arm direction from accumulating an untrackable S1 error. The
  // effective reference may slide with the modeled elbow while this is set.
  bool reference_governor_held{false};
  ArmDirectionReferenceSource reference_source{
      ArmDirectionReferenceSource::kDegenerate};
  // When the elbow plane radius approaches zero, the signed angle is not
  // observable.  Keep a finite, Cartesian branch-lock differential around
  // the last reliable elbow-plane direction so the QP cannot silently cross
  // to the opposite elbow branch.
  bool branch_lock_active{false};
  Vec7 branch_lock_jacobian{Vec7::Zero()};
  double branch_lock_distance_m{0.0};
  bool branch_lock_constraint_active{false};
  double branch_lock_requested_lower{0.0};
  double branch_lock_effective_lower{0.0};
  bool branch_lock_feasibility_clipped{false};
  double shoulder_world_z{0.0};
  double elbow_world_z{0.0};
};

struct ArmAngleNullspaceResult {
  Vec7 value{Vec7::Zero()};
  bool active{false};
  double alpha{0.0};
  double before{0.0};
  double after{0.0};
  double cartesian_residual{0.0};
};

double estimateArmAngleJacobianDotTimesVelocity(
    const Vec7& current_jacobian, const Vec7& previous_jacobian,
    const Vec7& velocity, double dt, bool previous_valid) noexcept;

ArmAngleNullspaceResult refineArmAngleInNullspace(
    const Vec7& primary, const Mat67& cartesian_jacobian,
    const ScalarJointTask& task, const Vec7& lower, const Vec7& upper,
    double tolerance,
    const LinearJointConstraint& linear_constraint = {},
    const JointPositionGuard& position_guard = {},
    const LinearJointConstraint& secondary_linear_constraint = {}) noexcept;

// Blend a weighted arm-angle objective toward a joint-centering velocity near
// the soft joint-limit buffer. The post-solve null-space path already applies
// this guard; a weighted QP must receive the recovery target before solving.
ScalarJointTask applyArmAngleJointLimitRecovery(
    const ScalarJointTask& task, const Mat67& cartesian_jacobian,
    const JointPositionGuard& position_guard,
    double maximum_rate_rad_s) noexcept;

// The ordinary signed-angle task is ill-conditioned when the elbow-plane
// radius is small.  These helpers turn the finite branch-lock differential
// carried by ArmAngleTask into the same bounded lower inequality used by the
// existing outward barrier.
LinearJointConstraint makeArmAngleBranchLockVelocityConstraint(
    const ArmAngleTask& task, const Vec7& lower, const Vec7& upper,
    double gain) noexcept;

LinearJointConstraint makeArmAngleBranchLockAccelerationConstraint(
    const ArmAngleTask& task, const Vec7& qdot, double jdot_qdot,
    const Vec7& lower, const Vec7& upper, double kp, double kd) noexcept;

// Keep the physical arm-angle error inside a first-order invariant envelope.
// The returned row is two-sided and is relaxed into the interior of the
// current joint-velocity box if the requested recovery rate is unreachable.
LinearJointConstraint makeArmAngleTrackingEnvelopeVelocityConstraint(
    const ArmAngleTask& task, const Vec7& lower, const Vec7& upper,
    double maximum_error_rad, double gain) noexcept;

class ArmAngleTaskBuilder {
 public:
  ArmAngleTaskBuilder(ArmSide side, double minimum_radius_m,
                      double full_weight_radius_m,
                      double reference_rate_limit_rad_s,
                      double reference_projection_hold_enter = 0.05,
                      double reference_projection_hold_exit = 0.10,
                      double error_branch_hysteresis_rad = 0.20,
                      double branch_lock_radius_m = -1.0,
                      bool reference_governor_enabled = false,
                      double reference_governor_enter_error_rad = 1.0,
                      double reference_governor_exit_error_rad = 0.5,
                      double reference_governor_tracking_error_rad = 0.15);

  ArmAngleTask compute(const ArmAngleGeometryInput& geometry,
                       const ArmDirectionReference& reference, double dt);
  ArmAngleTask computeContinuity(const ArmAngleGeometryInput& geometry,
                                 double dt);
  void reset() noexcept;

 private:
  ArmSide side_{ArmSide::kLeft};
  double minimum_radius_m_{0.0};
  double full_weight_radius_m_{0.0};
  double branch_lock_radius_m_{0.0};
  double reference_rate_limit_rad_s_{0.0};
  double reference_projection_hold_enter_{0.0};
  double reference_projection_hold_exit_{0.0};
  double error_branch_hysteresis_rad_{0.0};
  bool reference_governor_enabled_{false};
  double reference_governor_enter_error_rad_{0.0};
  double reference_governor_exit_error_rad_{0.0};
  double reference_governor_tracking_error_rad_{0.0};
  double reference_governor_error_sign_{1.0};
  bool reference_projection_held_{false};
  bool reference_governor_held_{false};
  bool has_previous_projection_{false};
  Eigen::Vector3d previous_projection_{Eigen::Vector3d::Zero()};
  bool continuity_initialized_{false};
  Eigen::Vector3d continuity_reference_{Eigen::Vector3d::Zero()};
  bool current_direction_initialized_{false};
  Eigen::Vector3d current_direction_{Eigen::Vector3d::Zero()};
  bool control_error_initialized_{false};
  double previous_control_error_rad_{0.0};
  bool target_angle_initialized_{false};
  double previous_target_angle_rad_{0.0};
};

}  // namespace tianji_qp_ik
