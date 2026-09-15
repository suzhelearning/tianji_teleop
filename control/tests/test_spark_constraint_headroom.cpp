#include "tianji_qp_ik/spark_constraint_headroom.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace tianji_qp_ik {
namespace {

ArmLimits limits() {
  ArmLimits value;
  value.lower_position.setConstant(-3.0);
  value.upper_position.setConstant(3.0);
  value.velocity.setConstant(4.0);
  return value;
}

JointAccelerationLimitConfig dynamicLimits() {
  JointAccelerationLimitConfig value;
  value.max_acceleration_rad_s2.setConstant(20.0);
  value.max_jerk_rad_s3.setConstant(200.0);
  value.hard_jerk_enabled = true;
  return value;
}

SparkHeadroomFeedforwardVelocityQpConfig governorConfig() {
  SparkHeadroomFeedforwardVelocityQpConfig value;
  value.low_headroom = 0.10;
  value.full_headroom = 0.30;
  value.reduction_time_seconds = 0.020;
  value.recovery_time_seconds = 0.100;
  value.jerk_usage_attack_seconds = 0.050;
  value.jerk_usage_release_seconds = 0.150;
  value.task_scaling_min_position = 0.75;
  value.task_scaling_min_orientation = 0.75;
  return value;
}

SparkConstraintHeadroomFeedback feedback() {
  SparkConstraintHeadroomFeedback value;
  value.accepted = true;
  value.task_scale_position = 1.0;
  value.task_scale_orientation = 1.0;
  return value;
}

TEST(SparkConstraintHeadroom, StartsClosedUntilDerivativeHistoryExists) {
  SparkConstraintHeadroomGovernor governor(governorConfig(), limits(),
                                            dynamicLimits());
  const auto first = governor.update(feedback(), 0.005);
  EXPECT_FALSE(first.derivative_history_valid);
  EXPECT_DOUBLE_EQ(first.scale, 0.0);

  const auto second = governor.update(feedback(), 0.005);
  EXPECT_TRUE(second.derivative_history_valid);
  EXPECT_GT(second.filtered_headroom, 0.0);
  EXPECT_LE(second.scale, 1.0);
}

TEST(SparkConstraintHeadroom, SelectsTightestVelocityAndTaskHeadroom) {
  SparkConstraintHeadroomGovernor governor(governorConfig(), limits(),
                                            dynamicLimits());
  (void)governor.update(feedback(), 0.005);

  auto velocity_limited = feedback();
  velocity_limited.qdot[3] = 3.8;
  auto result = governor.update(velocity_limited, 0.005);
  EXPECT_NEAR(result.velocity_headroom, 0.05, 1.0e-12);
  EXPECT_EQ(result.dominant_source, SparkHeadroomSource::kVelocity);

  auto task_limited = feedback();
  task_limited.task_scale_position = 0.75;
  result = governor.update(task_limited, 0.005);
  EXPECT_DOUBLE_EQ(result.task_headroom, 0.0);
  EXPECT_EQ(result.dominant_source, SparkHeadroomSource::kTaskScaling);
}

TEST(SparkConstraintHeadroom, SelectsAccelerationAndJerkHeadroom) {
  SparkConstraintHeadroomGovernor governor(governorConfig(), limits(),
                                            dynamicLimits());
  auto acceleration_limited = feedback();
  acceleration_limited.qddot[1] = 19.0;
  (void)governor.update(acceleration_limited, 0.005);
  auto result = governor.update(acceleration_limited, 0.005);
  EXPECT_NEAR(result.acceleration_headroom, 0.05, 1.0e-12);
  EXPECT_EQ(result.dominant_source, SparkHeadroomSource::kAcceleration);

  governor.reset();
  (void)governor.update(feedback(), 0.005);
  auto jerk_limited = feedback();
  for (int step = 0; step < 100; ++step) {
    jerk_limited.qddot[2] = step % 2 == 0 ? 1.0 : 0.0;
    result = governor.update(jerk_limited, 0.005);
  }
  EXPECT_LT(result.jerk_headroom, 0.05);
  EXPECT_EQ(result.dominant_source, SparkHeadroomSource::kJerk);
}

TEST(SparkConstraintHeadroom,
     IgnoresOneCycleJerkSaturationButClosesOnPersistence) {
  SparkConstraintHeadroomGovernor governor(governorConfig(), limits(),
                                            dynamicLimits());
  for (int step = 0; step < 200; ++step) {
    (void)governor.update(feedback(), 0.005);
  }
  ASSERT_GT(governor.state().scale, 0.99);

  auto sample = feedback();
  sample.qddot[0] = 1.0;
  const auto one_spike = governor.update(sample, 0.005);
  EXPECT_GT(one_spike.jerk_headroom, 0.80);
  EXPECT_GT(one_spike.scale, 0.90);

  SparkConstraintHeadroomResult sustained;
  for (int step = 0; step < 150; ++step) {
    sample.qddot[0] = step % 2 == 0 ? 0.0 : 1.0;
    sustained = governor.update(sample, 0.005);
  }
  EXPECT_LT(sustained.jerk_headroom, 0.05);
  EXPECT_LT(sustained.scale, 0.10);

  sample.qddot.setZero();
  SparkConstraintHeadroomResult recovered;
  for (int step = 0; step < 300; ++step) {
    recovered = governor.update(sample, 0.005);
  }
  EXPECT_GT(recovered.jerk_headroom, 0.99);
  EXPECT_GT(recovered.scale, 0.99);
}

TEST(SparkConstraintHeadroom, ReducesFasterThanItRecovers) {
  SparkConstraintHeadroomGovernor governor(governorConfig(), limits(),
                                            dynamicLimits());
  for (int step = 0; step < 200; ++step) {
    (void)governor.update(feedback(), 0.005);
  }
  ASSERT_GT(governor.state().scale, 0.99);

  auto constrained = feedback();
  constrained.task_scale_position = 0.75;
  const double before_reduction = governor.state().filtered_headroom;
  const auto reduced = governor.update(constrained, 0.005);
  const double reduction = before_reduction - reduced.filtered_headroom;
  const auto recovered = governor.update(feedback(), 0.005);
  const double recovery = recovered.filtered_headroom -
                          reduced.filtered_headroom;
  EXPECT_GT(reduction, recovery);
}

TEST(SparkConstraintHeadroom, InvalidFeedbackClosesAndResetClearsState) {
  SparkConstraintHeadroomGovernor governor(governorConfig(), limits(),
                                            dynamicLimits());
  for (int step = 0; step < 200; ++step) {
    (void)governor.update(feedback(), 0.005);
  }
  auto invalid = feedback();
  invalid.accepted = false;
  invalid.qdot[0] = std::numeric_limits<double>::quiet_NaN();
  const auto result = governor.update(invalid, 0.005);
  EXPECT_EQ(result.state, SparkHeadroomState::kInvalid);
  EXPECT_LT(result.filtered_headroom, 1.0);

  governor.reset();
  EXPECT_DOUBLE_EQ(governor.state().scale, 0.0);
  EXPECT_FALSE(governor.state().derivative_history_valid);
}

}  // namespace
}  // namespace tianji_qp_ik
