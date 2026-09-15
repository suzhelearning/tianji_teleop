#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/types.hpp"

#include <string>

namespace tianji_qp_ik {

enum class TargetMode { kManual, kHold, kCircle, kFigureEight, kOrientationOnly, kCombined };

struct DualArmTargets {
  Pose left;
  Pose right;
  Vec6 left_twist{Vec6::Zero()};
  Vec6 right_twist{Vec6::Zero()};
  bool left_stale{false};
  bool right_stale{false};
};

struct CartesianReference {
  Pose pose;
  Vec6 twist{Vec6::Zero()};
  Vec6 acceleration{Vec6::Zero()};
  bool stale{false};
  bool valid{true};
};

struct DualArmReferences {
  CartesianReference left;
  CartesianReference right;
};

// Build an acceleration-controller reference directly from the sampled
// target when Cartesian OTG is intentionally bypassed.
DualArmReferences directReferences(const DualArmTargets& targets);

class TargetManager {
 public:
  TargetManager(QpIkConfig config, DualArmTargets initial_targets);

  void setMode(TargetMode mode, double start_time_seconds);
  TargetMode mode() const noexcept { return mode_; }
  void setManualTarget(ArmSide side, const Pose& requested);
  bool setManualTarget(ArmSide side, const Pose& requested,
                       double source_timestamp_seconds,
                       double receive_time_seconds);
  bool setManualTargets(const Pose& left_requested,
                        const Pose& right_requested,
                        double source_timestamp_seconds,
                        double receive_time_seconds);
  DualArmTargets sample(double time_seconds);

 private:
  struct ManualTargetState {
    Vec6 filtered_twist{Vec6::Zero()};
    Vec6 filtered_acceleration{Vec6::Zero()};
    double source_timestamp_seconds{0.0};
    double receive_time_seconds{0.0};
    bool has_frame{false};
  };

  Pose scriptedPose(const Pose& initial, double elapsed_seconds) const;
  Pose predictedManualPose(const Pose& target,
                           const ManualTargetState& state,
                           double time_seconds) const;
  Vec6 manualTwist(const ManualTargetState& state,
                   double time_seconds) const;
  void updateManualKinematics(ManualTargetState& state,
                              const Vec6& raw_twist,
                              double dt_seconds) const;
  bool manualTargetStale(const ManualTargetState& state,
                         double time_seconds) const;
  Pose limitManualIncrement(const Pose& current, const Pose& requested) const;

  QpIkConfig config_;
  DualArmTargets initial_;
  DualArmTargets manual_;
  DualArmTargets hold_;
  DualArmTargets last_output_;
  Vec6 filtered_left_twist_{Vec6::Zero()};
  Vec6 filtered_right_twist_{Vec6::Zero()};
  ManualTargetState left_manual_state_;
  ManualTargetState right_manual_state_;
  TargetMode mode_{TargetMode::kHold};
  double mode_start_time_seconds_{0.0};
  double last_sample_time_seconds_{0.0};
  bool has_sample_time_{false};
};

std::string toString(TargetMode mode);

}  // namespace tianji_qp_ik
