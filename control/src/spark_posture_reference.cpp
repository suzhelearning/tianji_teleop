#include "tianji_qp_ik/spark_posture_reference.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace tianji_qp_ik {

SparkPostureReference7::SparkPostureReference7(
    DlsPostureRuckigConfig config, ArmLimits limits, double initial_dt,
    double posture_position_gain)
    : config_(std::move(config)),
      limits_(std::move(limits)),
      posture_position_gain_(posture_position_gain),
      limiter_(config_, limits_, initial_dt) {}

bool SparkPostureReference7::reset(
    const ArmMotionState& model_state) noexcept {
  initialized_ = limiter_.reset(model_state);
  return initialized_;
}

SparkPostureReferenceResult SparkPostureReference7::advance(
    const Vec7& target, const Vec7& q_model, double dt) {
  SparkPostureReferenceResult result;
  result.state = limiter_.state();
  if (!initialized_ || !q_model.allFinite() || !std::isfinite(dt) ||
      dt <= 0.0) {
    result.detail = "invalid_spark_posture_reference_input";
    return result;
  }

  const JointTrajectoryResult trajectory = limiter_.update(target, dt);
  result.state = trajectory.state;
  result.jerk = trajectory.jerk;
  result.velocity_ratio = trajectory.velocity_ratio;
  result.acceleration_ratio = trajectory.acceleration_ratio;
  result.jerk_ratio = trajectory.jerk_ratio;
  result.detail = trajectory.detail;
  if (!trajectory.accepted) {
    return result;
  }

  const Vec7 velocity_limit =
      (config_.velocity_scale * limits_.velocity)
          .cwiseMin(config_.max_velocity_rad_s);
  result.posture_velocity =
      trajectory.state.qdot +
      posture_position_gain_ * (trajectory.state.q - q_model);
  result.posture_velocity = result.posture_velocity
                                .array()
                                .max(-velocity_limit.array())
                                .min(velocity_limit.array())
                                .matrix();
  if (!result.posture_velocity.allFinite()) {
    result.detail = "invalid_spark_posture_velocity";
    return result;
  }
  result.accepted = true;
  return result;
}

SparkPostureReferenceResult SparkPostureReference7::update(
    const Vec7& q_spark, const Vec7& q_model, double dt) {
  return advance(q_spark, q_model, dt);
}

SparkPostureReferenceResult SparkPostureReference7::hold(
    const Vec7& q_model, double dt) {
  return advance(limiter_.state().q, q_model, dt);
}

}  // namespace tianji_qp_ik
