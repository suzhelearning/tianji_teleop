#pragma once

#include "tianji_mapped_palm/arm_angle.hpp"
#include "tianji_mapped_palm/cartesian_feedforward_allocator.hpp"
#include "tianji_mapped_palm/cartesian_servo.hpp"
#include "tianji_mapped_palm/cartesian_task_allocator.hpp"
#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/iterative_pose_dls.hpp"
#include "tianji_mapped_palm/joint_trajectory_limiter.hpp"
#include "tianji_mapped_palm/mujoco_robot.hpp"
#include "tianji_mapped_palm/pico_mapped_arm_angle_vector_governor.hpp"
#include "tianji_mapped_palm/pico_ee_motion_confidence.hpp"
#include "tianji_mapped_palm/qp_solver.hpp"
#include "tianji_mapped_palm/safety.hpp"
#include "tianji_mapped_palm/target_manager.hpp"
#include "tianji_mapped_palm/upper_arm_outward.hpp"
#include "tianji_mapped_palm/velocity_ik.hpp"

#include <memory>

namespace tianji_mapped_palm {

struct ArmReferenceState {
  Vec7 q_ref{Vec7::Zero()};
  Vec7 qdot_prev{Vec7::Zero()};
  Vec7 qddot_prev{Vec7::Zero()};
  Vec6 slack_prev{Vec6::Zero()};
  Vec7 dls_posture_goal{Vec7::Zero()};
  bool dls_posture_initialized{false};
};

struct ArmControllerDiagnostics {
  bool accepted{false};
  bool fallback_applied{false};
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
  JointVelocityBounds bounds;
  double reference_error_max_abs{0.0};
  double reference_scale{1.0};
  bool reference_frozen{false};
  SafetyDecision safety;
  ArmAngleTask arm_angle;
  // True only when the selected IK backend actually consumes this task.
  bool arm_angle_task_active{false};
  double arm_angle_requested_rate{0.0};
  double arm_angle_current_rate{0.0};
  PicoMappedArmAngleVectorState mapped_vector_nullspace;
  PicoEeMotionEvidence motion_evidence;
  double redundancy_authority{1.0};
  CartesianFeedbackGainSchedule cartesian_feedback_gains;
  Vec6 cartesian_feedback_twist{Vec6::Zero()};
  Vec6 preallocation_feedforward_twist{Vec6::Zero()};
  Vec6 final_desired_twist{Vec6::Zero()};
  Vec6 cartesian_execution_residual{Vec6::Zero()};
  CartesianFeedforwardAllocationResult feedforward_allocation;
  CartesianTaskAllocationResult task_allocation;
  double feedforward_preview_actual_mismatch_rad_s{0.0};
  UpperArmOutwardDiagnostics upper_arm_outward;
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

  DualArmController(MujocoRobot& robot, QpIkConfig config,
                    IkAlgorithm algorithm,
                    std::unique_ptr<IArmVelocityIk> left,
                    std::unique_ptr<IArmVelocityIk> right);

  // Transitional constructor for the legacy Viewer and solver benchmark.
  DualArmController(MujocoRobot& robot, QpIkConfig config,
                    std::unique_ptr<IQpSolver7> left_solver,
                    std::unique_ptr<IQpSolver7> right_solver);

  ControllerDiagnostics step(const DualArmTargets& targets, double dt);
  ControllerDiagnostics step(
      const DualArmTargets& targets,
      const DualArmDirectionReferences& arm_directions, double dt);
  ControllerDiagnostics step(const DualArmReferences& references, double dt);
  ControllerDiagnostics step(
      const DualArmReferences& references,
      const DualArmDirectionReferences& arm_directions, double dt);
  ControllerDiagnostics step(
      const DualArmReferences& references,
      const DualArmDirectionReferences& arm_directions,
      const DualArmJointVelocityPostureTasks& posture_tasks, double dt);
  ControllerDiagnostics step(
      const DualArmReferences& references,
      const DualArmDirectionReferences& arm_directions,
      const DualArmCartesianFeedforwardDemand& feedforward, double dt);
  void resetSolvers();
  bool synchronizeReferencesToActual();
  void setAlgorithm(IkAlgorithm algorithm);
  void setArmAngleReferenceMode(ArmAngleReferenceMode mode) noexcept;
  IkAlgorithm algorithm() const noexcept { return algorithm_; }
  const Vec7& reference(ArmSide side) const noexcept;
  const Vec7& previousVelocity(ArmSide side) const noexcept;
  const Vec7& previousAcceleration(ArmSide side) const noexcept;
  ArmMotionState referenceState(ArmSide side) const noexcept;
  bool setReferenceState(ArmSide side, const ArmMotionState& motion);

 private:
  ControllerDiagnostics stepImpl(const DualArmTargets& targets,
                                 const DualArmReferences* references,
                                 const DualArmDirectionReferences& arm_directions,
                                 const DualArmJointVelocityPostureTasks*
                                     posture_tasks,
                                 const DualArmCartesianFeedforwardDemand*
                                     feedforward,
                                 double dt);
  bool targetsAreFinite(const DualArmTargets& targets) const;
  bool referenceIsFinite(const CartesianReference& reference) const;
  ArmReferenceState& state(ArmSide side) noexcept;
  const ArmReferenceState& state(ArmSide side) const noexcept;
  SafetyDecision validateArmResult(const ArmIkInput& input,
                                   const ArmIkResult& result,
                                   const Vec7& candidate) const;
  bool applyBoundedFallback(ArmSide side, const ArmIkInput& input,
                            double dt,
                            ArmControllerDiagnostics& diagnostics);
  void configureDlsPostureTask(
      ArmSide side, const Pose& target, bool stale,
      const ArmAngleTask& arm_angle, double dt, ArmIkInput& input,
      ArmControllerDiagnostics& diagnostics);
  void configureMappedArmAngleVectorTask(
      ArmSide side, bool stale, double dt, ArmIkInput& input,
      ArmControllerDiagnostics& diagnostics);
  void resetDlsPosture(ArmSide side);
  void clearHistory();
  void clearHistory(ArmSide side);

  MujocoRobot& robot_;
  QpIkConfig config_;
  IkAlgorithm algorithm_{IkAlgorithm::kHierarchicalQp};
  std::unique_ptr<IArmVelocityIk> left_ik_;
  std::unique_ptr<IArmVelocityIk> right_ik_;
  std::unique_ptr<CartesianFeedforwardAllocator7> left_ff_allocator_;
  std::unique_ptr<CartesianFeedforwardAllocator7> right_ff_allocator_;
  std::unique_ptr<CartesianTaskAllocator7> left_task_allocator_;
  std::unique_ptr<CartesianTaskAllocator7> right_task_allocator_;
  ArmReferenceState left_state_;
  ArmReferenceState right_state_;
  ArmAngleTaskBuilder left_arm_angle_;
  ArmAngleTaskBuilder right_arm_angle_;
  PicoMappedArmAngleVectorGovernor7 left_mapped_vector_governor_;
  PicoMappedArmAngleVectorGovernor7 right_mapped_vector_governor_;
  PicoEeMotionConfidence left_motion_confidence_;
  PicoEeMotionConfidence right_motion_confidence_;
  IterativePoseDlsIk7 left_dls_posture_;
  IterativePoseDlsIk7 right_dls_posture_;
  JointTrajectoryLimiter7 left_dls_posture_ruckig_;
  JointTrajectoryLimiter7 right_dls_posture_ruckig_;
  ArmAngleReferenceMode arm_angle_reference_mode_{
      ArmAngleReferenceMode::kPico};
};

std::unique_ptr<IQpSolver7> makeSolver(SolverBackend backend, const QpIkConfig& config);

}  // namespace tianji_mapped_palm
