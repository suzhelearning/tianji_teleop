#pragma once
#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/safety.hpp"
#include "tianji_qp_ik/target_manager.hpp"
#include "tianji_qp_ik/velocity_ik.hpp"
#include "tianji_qp_ik/pico_ee_franka_dls.hpp"
#include "tianji_qp_ik/ruckig_trajectory_limiter.hpp"
#include "tianji_qp_ik/dls_pinocchio_arm_kinematics.hpp"
#include <memory>
namespace tianji_qp_ik {
struct ArmReferenceState {
  Vec7 q_ref{Vec7::Zero()};
  Vec7 qdot_prev{Vec7::Zero()};
  Vec7 qddot_prev{Vec7::Zero()};
};
struct ArmControllerDiagnostics {
  bool accepted{false};
  HoldReason hold_reason{HoldReason::kNonFiniteProblem};
  // Command-model TCP pose used by the nominal Cartesian controller.
  Pose current;
  // Actual feedback state used only for supervision and physical safety.
  Pose tcp_actual;
  Pose target;
  CartesianReference reference;
  Vec6 pose_error{Vec6::Zero()};
  Vec6 actual_pose_error{Vec6::Zero()};
  ArmIkResult ik;
  Vec7 q_ref{Vec7::Zero()};
  Vec7 q_actual{Vec7::Zero()};
  SafetyDecision safety;
  bool dls_posture_reference_active{false};
  Vec7 dls_posture_goal{Vec7::Zero()};
  Vec7 dls_posture_reference{Vec7::Zero()};
  Vec7 dls_posture_velocity_reference{Vec7::Zero()};
  Vec7 dls_posture_velocity_target{Vec7::Zero()};
  bool dls_posture_ruckig_accepted{false};
  std::string_view dls_posture_ruckig_detail{"not_updated"};
  PoseDlsStatus dls_posture_status{PoseDlsStatus::kRejected};
  int dls_posture_iterations{0};
  int dls_posture_joint_projection_count{0};
  double dls_posture_solve_time_us{0.0};
  double dls_posture_initial_position_error_m{0.0};
  double dls_posture_initial_orientation_error_rad{0.0};
  double dls_posture_final_position_error_m{0.0};
  double dls_posture_final_orientation_error_rad{0.0};
  double dls_posture_goal_limit_margin_rad{0.0};
  double ee_ik_wall_time_us{0.0};
  double ee_ruckig_wall_time_us{0.0};
  double ee_ik_to_ruckig_wall_time_us{0.0};
  bool ee_ruckig_invoked{false};
  bool ee_pinocchio_kinematics{false};
};

struct ControllerDiagnostics {
  bool accepted{false};
  HoldReason hold_reason{HoldReason::kNonFiniteProblem};
  ArmControllerDiagnostics left;
  ArmControllerDiagnostics right;
  double compute_time_us{0.0};
};

class DualArmController {
 public:
  DualArmController(MujocoRobot& robot, QpIkConfig config);
  ControllerDiagnostics step(const DualArmTargets& targets, double dt);
  ControllerDiagnostics step(const DualArmReferences& references, double dt);
  void resetSolvers();
  bool beginSimulationSoftStart(SimulationSoftStartLimits limits = {});
  DlsPostureRuckigConfig trajectorySampleLimits(ArmSide side) const;
  bool synchronizeReferencesToActual();
  const Vec7& reference(ArmSide side) const noexcept;
  const Vec7& previousVelocity(ArmSide side) const noexcept;
  const Vec7& previousAcceleration(ArmSide side) const noexcept;
  ArmMotionState referenceState(ArmSide side) const noexcept;
  bool setReferenceState(ArmSide side, const ArmMotionState& motion);
 private:
  struct DlsState { bool valid{false}; };
  void initializeDls();
  ControllerDiagnostics stepDls(const DualArmTargets&, const DualArmReferences*, double);
  bool targetsAreFinite(const DualArmTargets& targets) const;
  bool referenceIsFinite(const CartesianReference& reference) const;
  ArmReferenceState& state(ArmSide side) noexcept;
  const ArmReferenceState& state(ArmSide side) const noexcept;
  void clearHistory();
  void clearHistory(ArmSide side);
  MujocoRobot& robot_;
  QpIkConfig config_;
  ArmReferenceState left_state_, right_state_;
  std::unique_ptr<PicoEeFrankaDlsIk7> left_franka_dls_, right_franka_dls_;
  std::unique_ptr<DlsPinocchioArmKinematics> dls_kinematics_;
  std::unique_ptr<RuckigTrajectoryLimiter7> left_smoother_, right_smoother_;
  DlsState left_dls_, right_dls_;
  bool simulation_soft_start_pending_{false};
  SimulationSoftStartLimits simulation_soft_start_limits_;
};
}  // namespace tianji_qp_ik
