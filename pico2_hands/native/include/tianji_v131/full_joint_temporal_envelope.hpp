// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#pragma once

#include "tianji_v131/velocity_ik.hpp"

#include <cstdint>

namespace tianji_v131 {

struct FullJointTemporalEnvelopeApplication {
  JointVelocityBounds bounds;
  bool enabled{false};
  bool history_valid{false};
  bool relaxed{false};
  std::uint8_t relaxed_joint_mask{0U};
};

// Intersect the existing safety bounds with acceleration and jerk comfort
// intervals.  This adapter never widens the supplied safety interval.  When a
// temporal interval conflicts with a valid safety interval, the configured
// relax_to_safety policy keeps that joint's original safety interval intact.
FullJointTemporalEnvelopeApplication applyFullJointTemporalEnvelope(
    const JointVelocityBounds& base_bounds, const Vec7& qdot_previous,
    const Vec7& qddot_previous, double dt, bool history_valid,
    const FullJointTemporalEnvelopeConfig& config) noexcept;

struct FullJointTemporalEnvelopeUsage {
  double acceleration_ratio{0.0};
  double jerk_ratio{0.0};
};

FullJointTemporalEnvelopeUsage measureFullJointTemporalEnvelopeUsage(
    const Vec7& qdot, const Vec7& qdot_previous, const Vec7& qddot_previous,
    double dt, const FullJointTemporalEnvelopeConfig& config,
    bool history_valid) noexcept;

}  // namespace tianji_v131
