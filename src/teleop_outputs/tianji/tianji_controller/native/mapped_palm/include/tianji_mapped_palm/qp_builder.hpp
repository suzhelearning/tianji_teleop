#pragma once

#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/types.hpp"

namespace tianji_mapped_palm {

class QpBuilder {
 public:
  QpBuilder(QpConfig qp_config, JointLimitConfig joint_limit_config);

  QpProblem7 build(const Mat67& jacobian, const Vec6& desired_twist, const Vec7& q,
                   const ArmLimits& limits, double dt) const;

 private:
  QpConfig qp_config_;
  JointLimitConfig joint_limit_config_;
};

}  // namespace tianji_mapped_palm
