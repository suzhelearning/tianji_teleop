// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#include "tianji_v131/full_joint_temporal_envelope.hpp"

#include <algorithm>
#include <cmath>

namespace tianji_v131 {
namespace {

bool validInputs(const Vec7& qdot_previous, const Vec7& qddot_previous,
                double dt,
                const FullJointTemporalEnvelopeConfig& config) noexcept {
  return std::isfinite(dt) && dt > 0.0 && qdot_previous.allFinite() &&
         qddot_previous.allFinite() &&
         config.max_acceleration_rad_s2.allFinite() &&
         config.max_jerk_rad_s3.allFinite() &&
         (config.max_acceleration_rad_s2.array() > 0.0).all() &&
         (config.max_jerk_rad_s3.array() > 0.0).all();
}

}  // namespace

FullJointTemporalEnvelopeApplication applyFullJointTemporalEnvelope(
    const JointVelocityBounds& base_bounds, const Vec7& qdot_previous,
    const Vec7& qddot_previous, double dt, bool history_valid,
    const FullJointTemporalEnvelopeConfig& config) noexcept {
  FullJointTemporalEnvelopeApplication result;
  result.bounds = base_bounds;
  result.enabled = config.enabled;
  result.history_valid =
      config.enabled && history_valid &&
      validInputs(qdot_previous, qddot_previous, dt, config);
  if (!result.history_valid) {
    return result;
  }

  for (int index = 0; index < kArmDof; ++index) {
    const double qdot_previous_i = qdot_previous[index];
    const double qddot_previous_i = qddot_previous[index];
    const double acceleration_delta =
        config.max_acceleration_rad_s2[index] * dt;
    const double jerk_acceleration_delta =
        config.max_jerk_rad_s3[index] * dt;

    const double acceleration_lower =
        qdot_previous_i - acceleration_delta;
    const double acceleration_upper =
        qdot_previous_i + acceleration_delta;
    const double jerk_lower =
        qdot_previous_i +
        dt * (qddot_previous_i - jerk_acceleration_delta);
    const double jerk_upper =
        qdot_previous_i +
        dt * (qddot_previous_i + jerk_acceleration_delta);

    double temporal_lower = acceleration_lower;
    BoundSource temporal_lower_source = BoundSource::kTemporalAcceleration;
    if (jerk_lower > temporal_lower) {
      temporal_lower = jerk_lower;
      temporal_lower_source = BoundSource::kTemporalJerk;
    }
    double temporal_upper = acceleration_upper;
    BoundSource temporal_upper_source = BoundSource::kTemporalAcceleration;
    if (jerk_upper < temporal_upper) {
      temporal_upper = jerk_upper;
      temporal_upper_source = BoundSource::kTemporalJerk;
    }

    const double lower = std::max(base_bounds.lower[index], temporal_lower);
    const double upper = std::min(base_bounds.upper[index], temporal_upper);
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
      result.relaxed = true;
      result.relaxed_joint_mask = static_cast<std::uint8_t>(
          result.relaxed_joint_mask | (std::uint8_t{1U} << index));
      continue;
    }

    result.bounds.lower[index] = lower;
    result.bounds.upper[index] = upper;
    if (temporal_lower > base_bounds.lower[index]) {
      result.bounds.lower_source[static_cast<std::size_t>(index)] =
          temporal_lower_source;
    }
    if (temporal_upper < base_bounds.upper[index]) {
      result.bounds.upper_source[static_cast<std::size_t>(index)] =
          temporal_upper_source;
    }
  }
  return result;
}

FullJointTemporalEnvelopeUsage measureFullJointTemporalEnvelopeUsage(
    const Vec7& qdot, const Vec7& qdot_previous, const Vec7& qddot_previous,
    double dt, const FullJointTemporalEnvelopeConfig& config,
    bool history_valid) noexcept {
  FullJointTemporalEnvelopeUsage usage;
  if (!config.enabled || !history_valid ||
      !validInputs(qdot_previous, qddot_previous, dt, config) ||
      !qdot.allFinite()) {
    return usage;
  }
  for (int index = 0; index < kArmDof; ++index) {
    const double qddot = (qdot[index] - qdot_previous[index]) / dt;
    const double jerk = (qddot - qddot_previous[index]) / dt;
    if (!std::isfinite(qddot) || !std::isfinite(jerk)) {
      return {};
    }
    usage.acceleration_ratio = std::max(
        usage.acceleration_ratio,
        std::abs(qddot) / config.max_acceleration_rad_s2[index]);
    usage.jerk_ratio = std::max(
        usage.jerk_ratio,
        std::abs(jerk) / config.max_jerk_rad_s3[index]);
  }
  return usage;
}

}  // namespace tianji_v131
