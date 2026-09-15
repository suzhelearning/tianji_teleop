#include "tianji_qp_ik/spark_posture_reference.hpp"

#include <gtest/gtest.h>

namespace tianji_qp_ik {
namespace {

ArmLimits testLimits() {
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-2.0);
  limits.upper_position = Vec7::Constant(2.0);
  limits.velocity = Vec7::Constant(1.0);
  return limits;
}

DlsPostureRuckigConfig testConfig() {
  DlsPostureRuckigConfig config;
  config.velocity_scale = 1.0;
  config.max_velocity_rad_s = Vec7::Constant(0.7);
  config.max_acceleration_rad_s2 = Vec7::Constant(3.0);
  config.max_jerk_rad_s3 = Vec7::Constant(20.0);
  return config;
}

TEST(SparkPostureReference, ProducesBoundedGuideAndResetsToModelState) {
  constexpr double kDt = 0.005;
  SparkPostureReference7 reference(testConfig(), testLimits(), kDt, 4.0);
  ArmMotionState model;
  model.q = Vec7::Constant(0.1);
  ASSERT_TRUE(reference.reset(model));

  const Vec7 target = Vec7::Constant(0.8);
  Vec7 previous_acceleration = Vec7::Zero();
  for (int sample = 0; sample < 200; ++sample) {
    const SparkPostureReferenceResult result =
        reference.update(target, model.q, kDt);
    ASSERT_TRUE(result.accepted) << result.detail;
    EXPECT_LE(result.velocity_ratio, 1.0 + 1.0e-8);
    EXPECT_LE(result.acceleration_ratio, 1.0 + 1.0e-8);
    EXPECT_LE(result.jerk_ratio, 1.0 + 1.0e-8);
    EXPECT_TRUE(result.posture_velocity.allFinite());
    EXPECT_TRUE((result.posture_velocity.cwiseAbs().array() <= 0.7 + 1.0e-8)
                    .all());
    previous_acceleration = result.state.qddot;
  }
  (void)previous_acceleration;

  model.q = Vec7::Constant(-0.2);
  model.qdot.setZero();
  model.qddot.setZero();
  ASSERT_TRUE(reference.reset(model));
  EXPECT_TRUE(reference.state().q.isApprox(model.q));
  EXPECT_TRUE(reference.state().qdot.isZero());
  EXPECT_TRUE(reference.state().qddot.isZero());
}

TEST(SparkPostureReference, HoldBrakesContinuouslyWithoutChangingLimits) {
  constexpr double kDt = 0.005;
  SparkPostureReference7 reference(testConfig(), testLimits(), kDt, 3.0);
  ArmMotionState model;
  ASSERT_TRUE(reference.reset(model));
  for (int sample = 0; sample < 80; ++sample) {
    ASSERT_TRUE(reference.update(Vec7::Constant(0.7), model.q, kDt).accepted);
  }
  const ArmMotionState before = reference.state();
  const SparkPostureReferenceResult held = reference.hold(model.q, kDt);
  ASSERT_TRUE(held.accepted) << held.detail;
  EXPECT_LE((held.state.q - before.q).cwiseAbs().maxCoeff(),
            0.7 * kDt + 1.0e-10);
  EXPECT_LE((held.state.qddot - before.qddot).cwiseAbs().maxCoeff(),
            20.0 * kDt + 1.0e-8);
  EXPECT_LE(held.velocity_ratio, 1.0 + 1.0e-8);
  EXPECT_LE(held.acceleration_ratio, 1.0 + 1.0e-8);
  EXPECT_LE(held.jerk_ratio, 1.0 + 1.0e-8);
}

}  // namespace
}  // namespace tianji_qp_ik
