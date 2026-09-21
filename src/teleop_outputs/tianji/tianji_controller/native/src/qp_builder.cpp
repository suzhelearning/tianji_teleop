#include "tianji_qp_ik/qp_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {

QpBuilder::QpBuilder(QpConfig qp_config, JointLimitConfig joint_limit_config)
    : qp_config_(qp_config), joint_limit_config_(joint_limit_config) {}

QpProblem7 QpBuilder::build(const Mat67& jacobian, const Vec6& desired_twist, const Vec7& q,
                            const ArmLimits& limits, double dt) const {
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("QP time step must be finite and positive");
  }

  Vec6 squared_weights;
  squared_weights.head<3>().setConstant(qp_config_.position_weight * qp_config_.position_weight);
  squared_weights.tail<3>().setConstant(qp_config_.orientation_weight *
                                        qp_config_.orientation_weight);
  const Vec7 nominal = 0.5 * (limits.lower_position + limits.upper_position);
  const Vec7 qdot_nominal = -qp_config_.nominal_gain * (q - nominal);

  QpProblem7 result;
  result.H.noalias() = jacobian.transpose() * squared_weights.asDiagonal() * jacobian;
  result.H.diagonal().array() += qp_config_.regularization + qp_config_.nominal_weight;
  result.H = 0.5 * (result.H + result.H.transpose()).eval();
  result.g.noalias() = -jacobian.transpose() * squared_weights.asDiagonal() * desired_twist;
  result.g.noalias() -= qp_config_.nominal_weight * qdot_nominal;

  for (int index = 0; index < kArmDof; ++index) {
    const double velocity = joint_limit_config_.velocity_scale * limits.velocity[index];
    const double position_lower =
        (limits.lower_position[index] + joint_limit_config_.margin_rad - q[index]) / dt;
    const double position_upper =
        (limits.upper_position[index] - joint_limit_config_.margin_rad - q[index]) / dt;
    result.lower[index] = std::max(-velocity, position_lower);
    result.upper[index] = std::min(velocity, position_upper);
  }
  return result;
}

}  // namespace tianji_qp_ik
