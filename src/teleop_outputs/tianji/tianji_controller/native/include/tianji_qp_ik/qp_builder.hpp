#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/types.hpp"

namespace tianji_qp_ik {

class QpBuilder {
 public:
  QpBuilder(QpConfig qp_config, JointLimitConfig joint_limit_config);

  QpProblem7 build(const Mat67& jacobian, const Vec6& desired_twist, const Vec7& q,
                   const ArmLimits& limits, double dt) const;

 private:
  QpConfig qp_config_;
  JointLimitConfig joint_limit_config_;
};

}  // namespace tianji_qp_ik
