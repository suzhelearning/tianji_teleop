#pragma once

#include "tianji_qp_ik/acceleration_ik.hpp"
#include "tianji_qp_ik/acceleration_qpoases_solver.hpp"
#include "tianji_qp_ik/arm_angle.hpp"
#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/iterative_pose_dls.hpp"
#include "tianji_qp_ik/joint_trajectory_limiter.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/safety.hpp"
#include "tianji_qp_ik/target_manager.hpp"
#include "tianji_qp_ik/upper_arm_outward.hpp"

#include <memory>

namespace tianji_qp_ik {

struct ArmAccelerationDiagnostics {
  bool accepted{false};
  bool fallback_applied{false};
  HoldReason hold_reason{HoldReason::kNonFiniteProblem};
  // Nominal Cartesian state evaluated from q_ref/qdot_ref.
  Pose current;
  // Actual feedback is retained for watchdogs, physical safety and telemetry.
  Pose tcp_actual;
  CartesianReference reference;
  Vec6 pose_error{Vec6::Zero()};
  Vec6 actual_pose_error{Vec6::Zero()};
  Vec6 model_twist{Vec6::Zero()};
  Vec6 desired_acceleration{Vec6::Zero()};
  Vec6 jdot_qdot{Vec6::Zero()};
  ArmAccelerationResult qp;
  Vec7 q_ref{Vec7::Zero()};
  Vec7 qdot_ref{Vec7::Zero()};
  Vec7 q_actual{Vec7::Zero()};
  Vec7 qdot_actual{Vec7::Zero()};
  JointAccelerationBounds bounds;
  int hold_joint_index{-1};
  double position_reference_error{0.0};
  double velocity_reference_error{0.0};
  double reference_scale{1.0};
  bool reference_frozen{false};
  ArmAngleTask arm_angle;
  bool arm_angle_task_active{false};
  double arm_angle_current_rate{0.0};
  double arm_angle_requested_acceleration{0.0};
  double arm_angle_achieved_acceleration{0.0};
  double arm_angle_acceleration_residual{0.0};
  UpperArmOutwardDiagnostics upper_arm_outward;
  bool dls_posture_reference_active{false};
  Vec7 dls_posture_reference{Vec7::Zero()};
  Vec7 dls_posture_velocity_reference{Vec7::Zero()};
  Vec7 dls_posture_acceleration_reference{Vec7::Zero()};
  bool dls_posture_ruckig_accepted{false};
  std::string_view dls_posture_ruckig_detail{"not_updated"};
  PoseDlsStatus dls_posture_status{PoseDlsStatus::kRejected};
  int dls_posture_iterations{0};
  double dls_posture_solve_time_us{0.0};
};

struct AccelerationControllerDiagnostics {
  bool accepted{false};
  HoldReason hold_reason{HoldReason::kNonFiniteProblem};
  ArmAccelerationDiagnostics left;
  ArmAccelerationDiagnostics right;
  double compute_time_us{0.0};
};

class DualArmAccelerationController {
 public:
  DualArmAccelerationController(MujocoRobot& robot, QpIkConfig config);
  DualArmAccelerationController(
      MujocoRobot& robot, QpIkConfig config,
      std::unique_ptr<IAccelerationQpSolver> left_solver,
      std::unique_ptr<IAccelerationQpSolver> right_solver);

  AccelerationControllerDiagnostics step(const DualArmReferences& references,
                                         double dt);
  AccelerationControllerDiagnostics step(
      const DualArmReferences& references,
      const DualArmDirectionReferences& arm_directions, double dt);
  void resetReferences();
  bool synchronizeReferencesToActual();
  const Vec7& positionReference(ArmSide side) const noexcept;
  const Vec7& velocityReference(ArmSide side) const noexcept;
  const Vec7& previousAcceleration(ArmSide side) const noexcept;
  ArmMotionState referenceState(ArmSide side) const noexcept;
  bool setReferenceState(ArmSide side, const ArmMotionState& motion);
  void setArmAngleReferenceMode(ArmAngleReferenceMode mode) noexcept;

 private:
  struct ArmState {
    Vec7 q_ref{Vec7::Zero()};
    Vec7 qdot_ref{Vec7::Zero()};
    Vec7 qddot_previous{Vec7::Zero()};
    bool solver_initialized{false};
    Vec7 previous_outward_jacobian{Vec7::Zero()};
    bool previous_outward_jacobian_valid{false};
    Vec7 previous_arm_angle_jacobian{Vec7::Zero()};
    bool previous_arm_angle_jacobian_valid{false};
    Vec7 previous_arm_branch_lock_jacobian{Vec7::Zero()};
    bool previous_arm_branch_lock_jacobian_valid{false};
    Vec7 dls_posture_reference{Vec7::Zero()};
    bool dls_posture_reference_initialized{false};
  };

  ArmState& state(ArmSide side) noexcept;
  const ArmState& state(ArmSide side) const noexcept;
  void freezeArm(ArmSide side, IAccelerationQpSolver& solver);
  bool referenceIsFinite(const CartesianReference& reference) const;

  MujocoRobot& robot_;
  QpIkConfig config_;
  IterativePoseDlsIk7 left_dls_posture_;
  IterativePoseDlsIk7 right_dls_posture_;
  JointTrajectoryLimiter7 left_dls_posture_ruckig_;
  JointTrajectoryLimiter7 right_dls_posture_ruckig_;
  std::unique_ptr<IAccelerationQpSolver> left_solver_;
  std::unique_ptr<IAccelerationQpSolver> right_solver_;
  ArmState left_state_;
  ArmState right_state_;
  ArmAngleTaskBuilder left_arm_angle_;
  ArmAngleTaskBuilder right_arm_angle_;
  ArmAngleReferenceMode arm_angle_reference_mode_{
      ArmAngleReferenceMode::kPico};
};

}  // namespace tianji_qp_ik
