#pragma once

#include "tianji_v131/config.hpp"
#include "tianji_v131/types.hpp"

namespace tianji_v131 {

struct DexhandTargetConditioningDiagnostics {
  double requested_workspace_utilization{0.0};
  double workspace_utilization{0.0};
  bool workspace_soft_limited{false};
  double requested_linear_speed_m_s{0.0};
  double applied_linear_speed_m_s{0.0};
  bool linear_speed_limited{false};
  bool linear_acceleration_limited{false};
  double requested_angular_speed_rad_s{0.0};
  double applied_angular_speed_rad_s{0.0};
  bool angular_speed_limited{false};
  bool angular_acceleration_limited{false};
};

// Stateful reproduction of dexhand_deploy's TargetConditioner.  The class
// consumes an absolute base-frame pose and returns the pose that should be
// handed to velocity IK at one fixed-rate sample.  It intentionally owns the
// position/orientation and velocity state, because the acceleration limits in
// the source implementation are applied to the conditioned command rather
// than to the raw packet increment.
class DexhandTargetConditioner {
 public:
  DexhandTargetConditioner(const Pose& origin,
                           const DexhandPreIkConditioningConfig& config);

  Pose condition(const Pose& requested);
  void reset() noexcept;
  void synchronize(const Pose& pose);

  bool enabled() const noexcept { return config_.enabled; }
  const DexhandTargetConditioningDiagnostics& lastDiagnostics() const noexcept {
    return diagnostics_;
  }

 private:
  static void validate(const Pose& pose,
                       const DexhandPreIkConditioningConfig& config);

  Pose origin_;
  DexhandPreIkConditioningConfig config_;
  Pose position_state_;
  Eigen::Vector3d linear_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_{Eigen::Vector3d::Zero()};
  DexhandTargetConditioningDiagnostics diagnostics_;
};

}  // namespace tianji_v131
