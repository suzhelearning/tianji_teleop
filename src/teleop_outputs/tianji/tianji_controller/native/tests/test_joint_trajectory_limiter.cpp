#include "tianji_qp_ik/joint_trajectory_limiter.hpp"

#include <gtest/gtest.h>

namespace tianji_qp_ik {
namespace {

ArmLimits limits() {
  ArmLimits value;
  value.lower_position = Vec7::Constant(-2.0);
  value.upper_position = Vec7::Constant(2.0);
  value.velocity = Vec7::Constant(1.0);
  return value;
}

DlsPostureRuckigConfig config() {
  DlsPostureRuckigConfig value;
  value.velocity_scale = 1.0;
  value.max_velocity_rad_s = Vec7::Constant(0.6);
  value.max_acceleration_rad_s2 = Vec7::Constant(2.0);
  value.max_jerk_rad_s3 = Vec7::Constant(10.0);
  return value;
}

TEST(JointTrajectoryLimiter, GeneratesBoundedContinuousStepResponse) {
  constexpr double kDt = 0.005;
  JointTrajectoryLimiter7 limiter(config(), limits(), kDt);
  ArmMotionState initial;
  ASSERT_TRUE(limiter.reset(initial));
  Vec7 target;
  target << 0.8, -0.7, 0.6, -0.5, 0.4, -0.3, 0.2;

  Vec7 previous_acceleration = Vec7::Zero();
  for (int sample = 0; sample < 1000; ++sample) {
    const JointTrajectoryResult result = limiter.update(target, kDt);
    ASSERT_TRUE(result.accepted) << result.detail;
    EXPECT_LE(result.velocity_ratio, 1.0 + 1e-8);
    EXPECT_TRUE((result.state.qdot.cwiseAbs().array() <= 0.6 + 1e-8).all());
    EXPECT_LE(result.acceleration_ratio, 1.0 + 1e-8);
    EXPECT_LE(result.jerk_ratio, 1.0 + 1e-8);
    EXPECT_TRUE(((result.state.qddot - previous_acceleration) / kDt)
                    .isApprox(result.jerk, 1e-10));
    previous_acceleration = result.state.qddot;
  }
  EXPECT_TRUE(limiter.state().q.isApprox(target, 1e-6));
  EXPECT_TRUE(limiter.state().qdot.isZero(1e-6));
  EXPECT_TRUE(limiter.state().qddot.isZero(1e-6));
}

TEST(JointTrajectoryLimiter, ResetSynchronizesAllReferenceDerivatives) {
  JointTrajectoryLimiter7 limiter(config(), limits(), 0.005);
  ArmMotionState initial;
  initial.q = Vec7::Constant(0.1);
  initial.qdot = Vec7::Constant(0.2);
  initial.qddot = Vec7::Constant(-0.3);
  ASSERT_TRUE(limiter.reset(initial));
  EXPECT_TRUE(limiter.state().q.isApprox(initial.q));
  EXPECT_TRUE(limiter.state().qdot.isApprox(initial.qdot));
  EXPECT_TRUE(limiter.state().qddot.isApprox(initial.qddot));
}

}  // namespace
}  // namespace tianji_qp_ik
