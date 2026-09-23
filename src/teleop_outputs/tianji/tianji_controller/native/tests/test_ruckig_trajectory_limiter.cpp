#include "tianji_qp_ik/ruckig_trajectory_limiter.hpp"

#include <gtest/gtest.h>

namespace tianji_qp_ik {
namespace {

TEST(RuckigTrajectorySoftStart, ExplicitPico2CapsBoundDerivativesAndPreserveDefaults) {
  ArmLimits bounds;
  bounds.lower_position=Vec7::Constant(-3.);bounds.upper_position=Vec7::Constant(3.);
  bounds.velocity=Vec7::Constant(4.);
  DlsPostureRuckigConfig cfg;
  cfg.velocity_scale=1.;cfg.max_velocity_rad_s=Vec7::Constant(4.);
  cfg.max_acceleration_rad_s2=Vec7::Constant(60.);cfg.max_jerk_rad_s3=Vec7::Constant(3000.);
  RuckigTrajectoryLimiter7 fast(cfg,bounds,.005), original(cfg,bounds,.005);
  ASSERT_TRUE(fast.reset(ArmMotionState{}));ASSERT_TRUE(original.reset(ArmMotionState{}));
  EXPECT_FALSE(fast.beginSoftStart({-1.,3.,12.}));
  EXPECT_FALSE(fast.beginSoftStart({1.4,0.,12.}));
  EXPECT_FALSE(fast.beginSoftStart({1.4,3.,NAN}));
  ASSERT_TRUE(fast.beginSoftStart({1.4,3.,12.}));ASSERT_TRUE(original.beginSoftStart());
  for(int n=0;n<200;++n) {
    const auto result=fast.update(Vec7::Constant(2.),.005,false);
    ASSERT_TRUE(result.accepted)<<result.detail;
    EXPECT_LE(result.state.qdot.cwiseAbs().maxCoeff(),1.4+1e-8);
    EXPECT_LE(result.state.qddot.cwiseAbs().maxCoeff(),3.+1e-8);
    EXPECT_LE(result.jerk.cwiseAbs().maxCoeff(),12.+1e-8);
    ASSERT_TRUE(original.update(Vec7::Constant(2.),.005,false).accepted);
  }
  EXPECT_GT(fast.state().q[0],original.state().q[0]);
  EXPECT_DOUBLE_EQ(fast.sampledLimits().max_velocity_rad_s[0],1.4);
  EXPECT_DOUBLE_EQ(original.sampledLimits().max_velocity_rad_s[0],.35);
  EXPECT_DOUBLE_EQ(original.sampledLimits().max_acceleration_rad_s2[0],.5);
  EXPECT_DOUBLE_EQ(original.sampledLimits().max_jerk_rad_s3[0],2.);
}

TEST(RuckigTrajectorySoftStart, SlowApproachThenRampWithoutStateReset) {
  ArmLimits bounds;
  bounds.lower_position=Vec7::Constant(-3.);bounds.upper_position=Vec7::Constant(3.);
  bounds.velocity=Vec7::Constant(4.);
  DlsPostureRuckigConfig cfg;
  cfg.velocity_scale=1.;cfg.max_velocity_rad_s=Vec7::Constant(4.);
  cfg.max_acceleration_rad_s2=Vec7::Constant(60.);cfg.max_jerk_rad_s3=Vec7::Constant(3000.);
  RuckigTrajectoryLimiter7 limiter(cfg,bounds,.005);
  ASSERT_FALSE(limiter.beginSoftStart());
  ArmMotionState state;
  ASSERT_TRUE(limiter.reset(state));ASSERT_TRUE(limiter.beginSoftStart());
  const Vec7 goal=Vec7::Constant(1.);
  for(int n=0;n<2400;++n) {
    auto previous=limiter.state();
    auto result=limiter.update(goal,.005);
    ASSERT_TRUE(result.accepted)<<result.detail;
    EXPECT_LE(result.jerk_ratio,1.+1e-8);
    EXPECT_LE((result.state.q-previous.q).cwiseAbs().maxCoeff(),4.*.005+1e-6);
    if(n<200) {
      EXPECT_DOUBLE_EQ(limiter.sampledLimits().max_velocity_rad_s[0],.35);
      EXPECT_DOUBLE_EQ(limiter.sampledLimits().max_acceleration_rad_s2[0],.5);
      EXPECT_DOUBLE_EQ(limiter.sampledLimits().max_jerk_rad_s3[0],2.);
      EXPECT_LE(result.state.qdot.cwiseAbs().maxCoeff(),.35+1e-8);
      EXPECT_LE(result.state.qddot.cwiseAbs().maxCoeff(),.5+1e-8);
      EXPECT_LE(result.jerk.cwiseAbs().maxCoeff(),2.+1e-8);
      EXPECT_TRUE(limiter.softStarting());
    }
  }
  EXPECT_FALSE(limiter.softStarting());
  EXPECT_TRUE(limiter.state().q.isApprox(goal,1e-6));
  EXPECT_DOUBLE_EQ(limiter.sampledLimits().max_velocity_rad_s[0],4.);
  EXPECT_TRUE(limiter.beginSoftStart()); // A later S gets its own slow approach.
  for(int n=0;n<200;++n) ASSERT_TRUE(limiter.update(goal,.005,false).accepted);
  EXPECT_TRUE(limiter.softStarting()); // A small IK increment alone cannot complete takeover.
  for(int n=0;n<130;++n) {
    ASSERT_TRUE(limiter.update(goal,.005).accepted);
    EXPECT_TRUE(limiter.softStarting());
  }
  EXPECT_GT(limiter.sampledLimits().max_velocity_rad_s[0],.35);
  EXPECT_LT(limiter.sampledLimits().max_velocity_rad_s[0],4.);
  for(int n=0;n<12;++n) ASSERT_TRUE(limiter.update(goal,.005).accepted);
  EXPECT_FALSE(limiter.softStarting()); // .2 s proximity dwell + .5 s ramp.
  auto invalid=goal;invalid[0]=4.;
  EXPECT_FALSE(limiter.update(invalid,.005).accepted);
}

ArmLimits limits() {
  ArmLimits value;
  value.lower_position = Vec7::Constant(-2.0);
  value.upper_position = Vec7::Constant(2.0);
  value.velocity = Vec7::Constant(1.0);
  return value;
}

ArmLimits fastLimits() {
  ArmLimits value;
  value.lower_position = Vec7::Constant(-10.0);
  value.upper_position = Vec7::Constant(10.0);
  value.velocity = Vec7::Constant(4.0);
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

DlsPostureRuckigConfig fastConfig() {
  DlsPostureRuckigConfig value;
  value.velocity_scale = 1.0;
  value.max_velocity_rad_s = Vec7::Constant(4.0);
  value.max_acceleration_rad_s2 = Vec7::Constant(60.0);
  value.max_jerk_rad_s3 = Vec7::Constant(3000.0);
  return value;
}

TEST(RuckigTrajectoryLimiter, GeneratesBoundedContinuousStepResponse) {
  constexpr double kDt = 0.005;
  RuckigTrajectoryLimiter7 limiter(config(), limits(), kDt);
  ArmMotionState initial;
  ASSERT_TRUE(limiter.reset(initial));
  Vec7 target;
  target << 0.8, -0.7, 0.6, -0.5, 0.4, -0.3, 0.2;

  Vec7 previous_acceleration = Vec7::Zero();
  for (int sample = 0; sample < 1000; ++sample) {
    const RuckigTrajectoryResult result = limiter.update(target, kDt);
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

TEST(RuckigTrajectoryLimiter, ResetSynchronizesAllReferenceDerivatives) {
  RuckigTrajectoryLimiter7 limiter(config(), limits(), 0.005);
  ArmMotionState initial;
  initial.q = Vec7::Constant(0.1);
  initial.qdot = Vec7::Constant(0.2);
  initial.qddot = Vec7::Constant(-0.3);
  ASSERT_TRUE(limiter.reset(initial));
  EXPECT_TRUE(limiter.state().q.isApprox(initial.q));
  EXPECT_TRUE(limiter.state().qdot.isApprox(initial.qdot));
  EXPECT_TRUE(limiter.state().qddot.isApprox(initial.qddot));
}

TEST(RuckigTrajectoryLimiter, ProjectsDiscreteVelocityOvershootAtTheLimit) {
  constexpr double kDt = 0.005;
  RuckigTrajectoryLimiter7 limiter(fastConfig(), fastLimits(), kDt);
  ArmMotionState initial;
  initial.qdot = Vec7::Constant(3.9);
  initial.qddot = Vec7::Constant(30.0);
  ASSERT_TRUE(limiter.reset(initial));

  const RuckigTrajectoryResult result = limiter.update(Vec7::Constant(1.0), kDt);
  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_LE(result.velocity_ratio, 1.0 + 1.0e-8);
  EXPECT_TRUE((result.state.qdot.cwiseAbs().array() <= 4.0 + 1.0e-8).all());
  EXPECT_LE(result.acceleration_ratio, 1.0 + 1.0e-8);
  EXPECT_LE(result.jerk_ratio, 1.0 + 1.0e-8);
}

TEST(RuckigTrajectoryLimiter, ClipsOutwardSamplesAtBothPositionLimits) {
  constexpr double kDt = 0.005;
  // The caller supplies model limits already reduced by the safety margin.
  ArmLimits safe = limits();
  safe.lower_position.array() += 0.05;
  safe.upper_position.array() -= 0.05;
  for (const double direction : {-1.0, 1.0}) {
    RuckigTrajectoryLimiter7 limiter(config(), safe, kDt);
    const Vec7 boundary = direction > 0.0 ? safe.upper_position
                                         : safe.lower_position;
    ArmMotionState initial;
    initial.q = boundary - Vec7::Constant(direction * 1e-6);
    initial.qdot = Vec7::Constant(direction * 0.1);
    ASSERT_TRUE(limiter.reset(initial));
    const auto result = limiter.update(boundary, kDt);
    ASSERT_TRUE(result.accepted) << result.detail;
    EXPECT_TRUE(result.state.q.isApprox(boundary, 1e-12));
    EXPECT_TRUE(limiter.state().q.isApprox(result.state.q, 1e-12));
    for (int sample = 0; sample < 20; ++sample) {
      const auto next = limiter.update(boundary, kDt);
      ASSERT_TRUE(next.accepted) << next.detail;
      EXPECT_TRUE((next.state.q.array() >= safe.lower_position.array()).all());
      EXPECT_TRUE((next.state.q.array() <= safe.upper_position.array()).all());
    }
  }
}

TEST(RuckigTrajectoryLimiter, SettledRoundoffDoesNotRejectOrPreventRestart) {
  constexpr double kDt = 0.005;
  auto cfg = fastConfig();
  cfg.max_acceleration_rad_s2.tail<4>().setConstant(90.0);
  cfg.max_jerk_rad_s3.tail<4>().setConstant(4500.0);
  RuckigTrajectoryLimiter7 limiter(cfg, fastLimits(), kDt);
  // Full-precision session01 state that previously returned Ruckig -111.
  ArmMotionState initial;
  initial.q << 0.49840709962988533, -1.71, -1.3804472364591933,
      -1.6837011307197856, 2.2945333554887162, 0.95946412590648855,
      -1.0129415201729244;
  initial.qdot << 0, 0, 0, 0, -1.1102230246251565e-16,
      5.5511151231257827e-16, 2.2204460492503131e-16;
  Vec7 target;
  target << 0.49840709962988539, -1.71, -1.3804472364591935,
      -1.6837011307197856, 2.2945333554887162, 0.95946412590648933,
      -1.0129415201729244;
  ASSERT_TRUE(limiter.reset(initial));
  for (int i = 0; i < 20; ++i) {
    const auto result = limiter.update(target, kDt);
    ASSERT_TRUE(result.accepted) << result.detail;
    EXPECT_LT((result.state.q - target).norm(), 1e-12);
    EXPECT_TRUE(result.state.qdot.isZero(1e-12));
    EXPECT_LE(result.jerk_ratio, 1.0 + 1e-8);
  }
  target.array() += 0.1;
  const auto result = limiter.update(target, kDt);
  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_GT(result.state.qdot.norm(), 1e-6);
}

TEST(RuckigTrajectoryLimiter, AtTargetWithRealVelocityStillBrakes) {
  RuckigTrajectoryLimiter7 limiter(config(), limits(), 0.005);
  ArmMotionState initial;
  initial.qdot.setConstant(0.1);
  ASSERT_TRUE(limiter.reset(initial));
  const auto result = limiter.update(initial.q, 0.005);
  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_GT(result.state.qdot.norm(), 0.1);
  EXPECT_LE(result.jerk_ratio, 1.0 + 1e-8);
}

TEST(RuckigTrajectoryLimiter, SettledAccumulatedRoundoffKeepsJerkCheck) {
  RuckigTrajectoryLimiter7 limiter(fastConfig(), fastLimits(), 0.005);
  ArmMotionState initial;
  initial.qdot.setConstant(4e-14);
  initial.qddot.setConstant(4e-11);
  ASSERT_TRUE(limiter.reset(initial));
  const auto result = limiter.update(initial.q, 0.005);
  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_TRUE(result.state.qdot.isZero(0.0));
  EXPECT_TRUE(result.state.qddot.isZero(0.0));
  EXPECT_TRUE(result.jerk.isApprox(-initial.qddot / 0.005, 1e-12));
  EXPECT_LE(result.jerk_ratio, 1.0 + 1e-8);
}

}  // namespace
}  // namespace tianji_qp_ik
