#include "tianji_qp_ik/spark_qpoases_diagnostic.hpp"

#include <cmath>

namespace tianji_qp_ik {

SparkQpoasesDirectCommand makeDirectCommand(
    const SparkGuidanceDiagnostics& diagnostics) noexcept {
  SparkQpoasesDirectCommand result;
  if (!diagnostics.accepted || !diagnostics.left.accepted ||
      !diagnostics.right.accepted || !diagnostics.left.ik.accepted ||
      !diagnostics.right.ik.accepted) {
    result.detail = "bilateral_spark_ik_not_accepted";
    return result;
  }
  if (!diagnostics.left.q_ik.allFinite() ||
      !diagnostics.right.q_ik.allFinite()) {
    result.detail = "non_finite_spark_ik";
    return result;
  }
  result.left = diagnostics.left.q_ik;
  result.right = diagnostics.right.q_ik;
  result.accepted = true;
  result.detail = "spark_direct_command_accepted";
  return result;
}

SparkPostureVelocityResult makePostureVelocity(
    const Vec7& q_ik, const Vec7& q_model, double position_gain,
    const Vec7& velocity_limit) noexcept {
  SparkPostureVelocityResult result;
  if (!q_ik.allFinite() || !q_model.allFinite() ||
      !velocity_limit.allFinite() || !std::isfinite(position_gain) ||
      position_gain <= 0.0 || (velocity_limit.array() <= 0.0).any()) {
    result.detail = "invalid_posture_velocity_input";
    return result;
  }
  result.value = (position_gain * (q_ik - q_model))
                     .cwiseMax(-velocity_limit)
                     .cwiseMin(velocity_limit);
  if (!result.value.allFinite()) {
    result.value.setZero();
    result.detail = "non_finite_posture_velocity";
    return result;
  }
  result.accepted = true;
  result.detail = "posture_velocity_accepted";
  return result;
}

}  // namespace tianji_qp_ik
