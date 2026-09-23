#pragma once

#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"
#include "tianji_qp_ik/shared_root_pipeline.hpp"
#include "tianji_qp_ik/target_manager.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace tianji_qp_ik {

struct SharedRootGuidanceArmDiagnostics {
  bool accepted{false};
  SharedRootArmTarget target;
};

struct SharedRootGuidanceDiagnostics {
  bool reference_reset_required{false};
  // Completion must match this control cycle, not merely the source sequence.
  std::uint64_t shared_root_cycle{0};
  SharedRootState shared_root_state{SharedRootState::kUninitialized};
  bool accepted{false};
  bool target_valid{false};
  double compute_time_us{0.0};
  std::string_view detail{"not_updated"};
  SharedRootGuidanceArmDiagnostics left;
  SharedRootGuidanceArmDiagnostics right;
  DualArmTargets cartesian_targets;
};

// Mapping only. The DLS controller owns IK, trajectory limiting, and bilateral
// acceptance; no reference history commits before that controller acknowledges.
class SharedRootGuidance {
 public:
  SharedRootGuidance(MujocoRobot& robot, const QpIkConfig& config,
                     const std::string& urdf_path,
                     const SharedRootOptions* shared_root);

  bool updateSharedRootFrame(const PicoTeleopFrame&, std::int64_t now_ns);
  SharedRootGuidanceDiagnostics stepSharedRoot(
      const ArmMotionState& left_model, const ArmMotionState& right_model,
      double dt, std::int64_t now_ns, bool execution_authorized,
      bool model_state_valid);
  bool confirmSharedRootReference(std::uint64_t cycle,
                                  bool bilateral_accepted) noexcept;
  bool reset(const ArmMotionState& left_model,
             const ArmMotionState& right_model) noexcept;
  bool resetMappingSession(const ArmMotionState& left_model,
                           const ArmMotionState& right_model) noexcept;
  void invalidateTarget(
      std::string_view detail = "shared_root_target_stale") noexcept;
  const SharedRootTargets& latestTargets() const noexcept {
    return latest_targets_;
  }

 private:
  void initializeSharedRoot(const SharedRootOptions&, const std::string& urdf_path,
                            const SharedRootShapeConfig& shape);

  MujocoRobot& robot_;
  PinocchioArmKinematics kinematics_;
  std::unique_ptr<SharedRootPipeline> shared_root_;
  SharedRootContinuityOutput shared_output_;
  SharedRootTargets latest_targets_;
  std::uint64_t latest_source_sequence_{0};
  std::uint64_t last_accepted_sequence_{0};
  std::uint64_t shared_cycle_{0};
  bool shared_ack_ready_{false};
};

}  // namespace tianji_qp_ik
