#pragma once

#include "tianji_qp_ik/cartesian_otg.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"
#include "tianji_qp_ik/spark_feedforward_reference.hpp"
#include "tianji_qp_ik/spark_constraint_headroom.hpp"
#include "tianji_qp_ik/spark_palm_twist_estimator.hpp"
#include "tianji_qp_ik/spark_posture_reference.hpp"
#include "tianji_qp_ik/spark_upper_qpoases_ik.hpp"
#include "tianji_qp_ik/target_manager.hpp"
#include "tianji_qp_ik/velocity_ik.hpp"
#include "tianji_qp_ik/shared_root_pipeline.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <memory>

namespace tianji_qp_ik {

enum class SparkPostureGuideMode {
  kRuckig,
  kDirect,
  kJointReferenceVelocity,
  kOtgConsistentJointReferenceVelocity,
  kFeedforwardJointReferenceVelocity,
  kHeadroomFeedforwardJointReferenceVelocity,
  kDisabled
};

enum class SparkSettledHoldReason {
  kNone,
  kStale,
  kHeadroomExhausted,
};

std::string_view toString(SparkSettledHoldReason reason) noexcept;

struct SparkGuidanceArmDiagnostics {
  bool accepted{false};
  SparkUpperIkResult ik;
  SparkPostureReferenceResult reference;
  SparkUpperArmTarget target;
  Vec7 q_ik{Vec7::Zero()};
  SparkFeedforwardTargetDecision feedforward_target;
  SparkFeedforwardReferenceResult feedforward;
  SparkPalmTwistDecision palm_twist;
  SparkPalmTwistDecision motion_intent_twist;
  SparkConstraintHeadroomResult headroom;
  bool stationary_joint_reference_held{false};
  bool settled_hold_active{false};
  double settled_hold_dwell_seconds{0.0};
  SparkSettledHoldReason settled_hold_reason{SparkSettledHoldReason::kNone};
};

struct SparkGuidanceDiagnostics {
  bool reference_reset_required{false};
  // Zero on legacy dispatch. A completion requires this exact control cycle,
  // not merely the source sequence (multiple ticks may share one input frame).
  std::uint64_t shared_root_cycle{0};
  SharedRootState shared_root_state{SharedRootState::kUninitialized};
  bool accepted{false};
  bool target_valid{false};
  bool blend_active{false};
  double blend_progress{0.0};
  bool joint_takeover_active{false};
  bool joint_takeover_finished{false};
  double compute_time_us{0.0};
  std::string_view detail{"not_updated"};
  SparkGuidanceArmDiagnostics left;
  SparkGuidanceArmDiagnostics right;
  DualArmTargets cartesian_targets;
  DualArmReferences cartesian_references;
  bool cartesian_references_valid{false};
  bool feedforward_references_valid{false};
  DualArmJointVelocityPostureTasks posture_tasks;
};

class DualArmSparkGuidance {
 public:
  DualArmSparkGuidance(MujocoRobot& robot, const QpIkConfig& config,
                       const std::string& urdf_path,
                       SparkPostureGuideMode posture_mode =
                           SparkPostureGuideMode::kRuckig,
                       const SharedRootOptions* shared_root = nullptr);

  // Explicit experimental/offline integration. Legacy methods cannot silently
  // authorize this mode. Production profile enabling remains gated separately.
  bool updateSharedRootFrame(const PicoTeleopFrame&, std::int64_t now_ns);
  SparkGuidanceDiagnostics stepSharedRoot(
      const ArmMotionState& left_model, const ArmMotionState& right_model,
      double dt, std::int64_t now_ns, bool execution_authorized,
      bool model_state_valid);
  bool confirmSharedRootReference(std::uint64_t cycle, bool bilateral_accepted) noexcept;

  SparkUpperTargets updatePicoFrame(const PicoTeleopFrame& frame);
  bool startJointSpaceTakeover(const SparkUpperTargets& targets,
                               const ArmMotionState& left_model,
                               const ArmMotionState& right_model);
  SparkGuidanceDiagnostics stepJointSpaceTakeover(
      const ArmMotionState& left_model, const ArmMotionState& right_model,
      double dt);
  void cancelJointSpaceTakeover() noexcept;
  SparkGuidanceDiagnostics step(const ArmMotionState& left_model,
                                const ArmMotionState& right_model, double dt);
  void updateHeadroomFeedback(
      const SparkConstraintHeadroomFeedback& left,
      const SparkConstraintHeadroomFeedback& right, double dt) noexcept;
  bool reset(const ArmMotionState& left_model,
             const ArmMotionState& right_model) noexcept;
  // DLS/Ceres own motion validation; reset mapping histories, not unused SPARK OTG.
  bool resetMappingSession(const ArmMotionState& left_model,
                           const ArmMotionState& right_model) noexcept;
  void invalidateTarget(std::string_view detail = "spark_target_stale") noexcept;
  const SparkUpperTargets& latestTargets() const noexcept {
    return latest_targets_;
  }

 private:
  bool resetImpl(const ArmMotionState&, const ArmMotionState&, bool reset_reference) noexcept;
  struct ArmState {
    ArmState(ArmSide side, PinocchioArmKinematics& kinematics,
             const QpIkConfig& config, const ArmLimits& limits,
             double initial_dt);
    SparkUpperQpoasesIk7 ik;
    SparkPostureReference7 reference;
    CartesianReferenceGenerator otg;
    SparkFeedforwardReference7 feedforward;
    SparkPalmTwistEstimator palm_twist;
    SparkPalmTwistEstimator motion_intent_twist;
    SparkConstraintHeadroomGovernor headroom;
    SparkFeedforwardTargetDecision last_feedforward_target;
    ArmLimits arm_limits;
    Vec7 last_valid_q_ik{Vec7::Zero()};
    bool settled_hold_active{false};
    double settled_hold_dwell_seconds{0.0};
    SparkSettledHoldReason settled_hold_reason{SparkSettledHoldReason::kNone};
  };

  void restartBlend(const ArmMotionState& left_model,
                    const ArmMotionState& right_model);
  void updateBlend(const ArmMotionState& left_model,
                   const ArmMotionState& right_model, double dt);
  SparkGuidanceDiagnostics stepImpl(const ArmMotionState&, const ArmMotionState&, double);
  bool resetSharedRootControl(const ArmMotionState&, const ArmMotionState&);
  void initializeSharedRoot(const SharedRootOptions&, const std::string& urdf_path);

  MujocoRobot& robot_;
  QpIkConfig config_;
  SparkPostureGuideMode posture_mode_{SparkPostureGuideMode::kRuckig};
  PinocchioArmKinematics kinematics_;
  UpperSparkSkeletonScaler scaler_;
  ArmState left_;
  ArmState right_;
  SparkUpperTargets raw_targets_;
  SparkUpperTargets blend_start_targets_;
  SparkUpperTargets latest_targets_;
  SparkUpperTargets held_targets_;
  bool blend_restart_pending_{false};
  double blend_elapsed_seconds_{0.0};
  double joint_reference_phase_{0.0};
  std::uint64_t last_tracking_epoch_{0U};
  std::uint64_t latest_source_sequence_{0U};
  std::int64_t latest_source_timestamp_ns_{0};
  std::uint64_t latest_source_epoch_{0U};
  bool latest_source_discontinuity_{false};
  Pose latest_left_input_palm_;
  Pose latest_right_input_palm_;
  std::uint64_t last_feedforward_sequence_{0U};
  std::uint64_t last_palm_twist_sequence_{0U};
  bool joint_takeover_active_{false};
  Vec7 joint_takeover_left_goal_{Vec7::Zero()};
  Vec7 joint_takeover_right_goal_{Vec7::Zero()};
  std::unique_ptr<SharedRootPipeline> shared_root_;
  SharedRootContinuityOutput shared_output_;
  std::uint64_t shared_cycle_{0};
  bool shared_ack_ready_{false};
  bool shared_suppress_hold_{false};
  bool shared_intent_valid_{false};
};

}  // namespace tianji_qp_ik
