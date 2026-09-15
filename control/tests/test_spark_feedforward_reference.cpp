#include "tianji_qp_ik/spark_feedforward_reference.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace tianji_qp_ik {
namespace {

SparkFeedforwardVelocityQpConfig testConfig() {
  SparkFeedforwardVelocityQpConfig config;
  config.alpha = 0.65;
  config.beta = 0.12;
  config.dt_median_window = 5;
  config.dt_min_ratio = 0.5;
  config.dt_max_ratio = 1.5;
  config.maximum_joint_jump_rad = 0.35;
  config.stale_velocity_decay_seconds = 0.02;
  config.source_stationary_velocity_rad_s = 0.05;
  config.velocity_reversal_decay = 0.10;
  config.velocity_stationary_decay = 0.25;
  config.target_braking_acceleration_scale = 0.75;
  config.attack_seconds = 0.02;
  config.release_seconds = 0.02;
  return config;
}

ArmLimits testLimits() {
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-4.0);
  limits.upper_position = Vec7::Constant(4.0);
  limits.velocity = Vec7::Constant(4.0);
  return limits;
}

SparkFeedforwardReference7 makeReference() {
  JointLimitConfig joint_limits;
  joint_limits.margin_rad = 0.0;
  joint_limits.velocity_scale = 1.0;
  joint_limits.max_acceleration_rad_s2 =
      (Vec7() << 60.0, 60.0, 60.0, 90.0, 90.0, 90.0, 90.0)
          .finished();
  joint_limits.braking_acceleration_rad_s2 =
      (Vec7() << 45.0, 45.0, 45.0, 67.5, 67.5, 67.5, 67.5)
          .finished();
  joint_limits.hard_jerk_enabled = true;
  joint_limits.max_jerk_rad_s3 =
      (Vec7() << 1000.0, 1000.0, 1000.0, 1500.0, 1500.0, 1500.0,
       1500.0)
          .finished();
  return SparkFeedforwardReference7(testConfig(), testLimits(),
                                    joint_limits, 0.005);
}

TEST(SparkFeedforwardReference,
     RespectsVelocityQpSafeMarginWhileBrakingAtLowerLimit) {
  SparkFeedforwardVelocityQpConfig config = testConfig();
  JointLimitConfig joint_limits;
  joint_limits.margin_rad = 0.05;
  joint_limits.velocity_scale = 1.0;
  joint_limits.max_acceleration_rad_s2 =
      (Vec7() << 60.0, 60.0, 60.0, 90.0, 90.0, 90.0, 90.0)
          .finished();
  joint_limits.braking_acceleration_rad_s2 =
      (Vec7() << 45.0, 45.0, 45.0, 67.5, 67.5, 67.5, 67.5)
          .finished();
  joint_limits.hard_jerk_enabled = true;
  joint_limits.max_jerk_rad_s3 =
      (Vec7() << 1000.0, 1000.0, 1000.0, 1500.0, 1500.0, 1500.0,
       1500.0)
          .finished();
  SparkFeedforwardReference7 reference(
      config, testLimits(), joint_limits, 0.005);
  ArmMotionState model;
  model.q = Vec7::Constant(-3.70);
  reference.reset(model, 85U);

  std::uint64_t sequence = 1U;
  std::int64_t timestamp = 1'000'000'000LL;
  Vec7 target = model.q;
  ASSERT_TRUE(reference
                  .acceptTarget(target, sequence, timestamp, 85U, model)
                  .accepted);

  constexpr double kSafeLower = -3.95;
  SparkFeedforwardReferenceResult result;
  for (int sample = 1; sample <= 30; ++sample) {
    ++sequence;
    timestamp += 10'000'000LL;
    target.setConstant(std::max(kSafeLower, -3.70 - 0.012 * sample));
    ASSERT_TRUE(reference
                    .acceptTarget(target, sequence, timestamp, 85U, model)
                    .accepted);
    for (int control_step = 0; control_step < 2; ++control_step) {
      result = reference.step(model, 0.005, true);
      EXPECT_GE(result.q.minCoeff(), kSafeLower - 1.0e-10);
      EXPECT_LE(result.qdot.cwiseAbs().maxCoeff(), 4.0 + 1.0e-9);
      EXPECT_LE(result.qddot.head<3>().cwiseAbs().maxCoeff(), 60.0 + 1.0e-9);
      EXPECT_LE(result.qddot.tail<4>().cwiseAbs().maxCoeff(), 90.0 + 1.0e-9);
      EXPECT_LE(result.jerk.head<3>().cwiseAbs().maxCoeff(), 1000.0 + 1.0e-9);
      EXPECT_LE(result.jerk.tail<4>().cwiseAbs().maxCoeff(), 1500.0 + 1.0e-9);
    }
  }

  for (int sample = 0; sample < 120; ++sample) {
    ++sequence;
    timestamp += 10'000'000LL;
    ASSERT_TRUE(reference
                    .acceptTarget(target, sequence, timestamp, 85U, model)
                    .accepted);
    for (int control_step = 0; control_step < 2; ++control_step) {
      result = reference.step(model, 0.005, true);
      EXPECT_GE(result.q.minCoeff(), kSafeLower - 1.0e-10);
    }
  }
  EXPECT_NEAR(result.q.minCoeff(), kSafeLower, 2.0e-3);
  EXPECT_LT(result.qdot.cwiseAbs().maxCoeff(), 0.02);
}

TEST(SparkFeedforwardReference, InitializesAtModelAndActivatesOnFirstTarget) {
  SparkFeedforwardReference7 reference = makeReference();
  ArmMotionState model;
  model.q = Vec7::Constant(0.1);
  reference.reset(model, 85U);

  const SparkFeedforwardTargetDecision decision =
      reference.acceptTarget(Vec7::Constant(0.2), 1U, 1'000'000'000LL, 85U,
                             model);
  const SparkFeedforwardReferenceResult result =
      reference.step(model, 0.005, true);

  EXPECT_TRUE(decision.accepted);
  EXPECT_FALSE(decision.dt_valid);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.state, SparkFeedforwardState::kActive);
  EXPECT_TRUE(result.q.allFinite());
  EXPECT_TRUE(result.qdot.allFinite());
  EXPECT_GT(result.activation, 0.0);
}

TEST(SparkFeedforwardReference,
     StaticJointTargetConvergesWithoutEstimatedSourceVelocity) {
  SparkFeedforwardReference7 reference = makeReference();
  ArmMotionState model;
  reference.reset(model, 85U);
  const Vec7 target = Vec7::Constant(0.20);
  ASSERT_TRUE(reference
                  .acceptTarget(target, 1U, 1'000'000'000LL, 85U, model)
                  .accepted);

  SparkFeedforwardReferenceResult result;
  for (int step = 0; step < 400; ++step) {
    result = reference.step(model, 0.005, true);
  }

  EXPECT_TRUE(result.q.isApprox(target, 2.0e-3))
      << "q=" << result.q.transpose();
  EXPECT_LT(result.qdot.cwiseAbs().maxCoeff(), 0.02);
}

TEST(SparkFeedforwardReference, RejectsDuplicateAndDroppedSourceTiming) {
  SparkFeedforwardReference7 reference = makeReference();
  ArmMotionState model;
  reference.reset(model, 85U);
  ASSERT_TRUE(reference
                  .acceptTarget(Vec7::Zero(), 1U, 1'000'000'000LL, 85U,
                                model)
                  .accepted);
  for (std::uint64_t sequence = 2; sequence <= 5; ++sequence) {
    Vec7 target = Vec7::Zero();
    target[0] = 0.01 * static_cast<double>(sequence - 1U);
    const auto decision = reference.acceptTarget(
        target, sequence,
        1'000'000'000LL +
            static_cast<std::int64_t>(sequence - 1U) * 11'000'000LL,
        85U, model);
    ASSERT_TRUE(decision.accepted);
    ASSERT_TRUE(decision.dt_valid);
  }

  const auto duplicate = reference.acceptTarget(
      Vec7::Constant(0.05), 5U, 1'044'000'000LL, 85U, model);
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_EQ(duplicate.detail, "feedforward_duplicate_source");

  const auto dropped = reference.acceptTarget(
      Vec7::Constant(0.06), 6U, 1'074'000'000LL, 85U, model);
  EXPECT_TRUE(dropped.accepted);
  EXPECT_FALSE(dropped.dt_valid);
  EXPECT_EQ(dropped.detail, "feedforward_source_dt_rejected");
  EXPECT_NEAR(dropped.median_dt_seconds, 0.011, 1.0e-12);
}

TEST(SparkFeedforwardReference,
     ReleasesOldDirectionFeedforwardOnStationaryAndReversingSamples) {
  SparkFeedforwardReference7 reference = makeReference();
  ArmMotionState model;
  reference.reset(model, 85U);
  ASSERT_TRUE(reference
                  .acceptTarget(Vec7::Zero(), 1U, 1'000'000'000LL, 85U,
                                model)
                  .accepted);

  SparkFeedforwardTargetDecision moving;
  for (std::uint64_t sequence = 2U; sequence <= 6U; ++sequence) {
    Vec7 target = Vec7::Zero();
    target[0] = 0.012 * static_cast<double>(sequence - 1U);
    moving = reference.acceptTarget(
        target, sequence,
        1'000'000'000LL +
            static_cast<std::int64_t>(sequence - 1U) * 10'000'000LL,
        85U, model);
  }
  ASSERT_GT(moving.estimated_velocity[0], 0.1);

  Vec7 stationary_target = Vec7::Zero();
  stationary_target[0] = 0.060;
  const auto stationary = reference.acceptTarget(
      stationary_target, 7U, 1'060'000'000LL, 85U, model);
  EXPECT_LT(std::abs(stationary.estimated_velocity[0]),
            std::abs(moving.estimated_velocity[0]) * 0.5);
  EXPECT_LT(stationary.feedforward_confidence[0], 0.5);

  Vec7 reversing_target = stationary_target;
  reversing_target[0] -= 0.012;
  const auto reversing = reference.acceptTarget(
      reversing_target, 8U, 1'070'000'000LL, 85U, model);
  EXPECT_LT(std::abs(reversing.estimated_velocity[0]),
            std::abs(stationary.estimated_velocity[0]));
  EXPECT_LT(reversing.feedforward_confidence[0], 0.5);
}

TEST(SparkFeedforwardReference,
     TargetRelativeBrakingAvoidsOvershootAndPreservesDerivativeBounds) {
  SparkFeedforwardReference7 reference = makeReference();
  ArmMotionState model;
  reference.reset(model, 85U);
  ASSERT_TRUE(reference
                  .acceptTarget(Vec7::Zero(), 1U, 1'000'000'000LL, 85U,
                                model)
                  .accepted);

  for (std::uint64_t sequence = 2U; sequence <= 11U; ++sequence) {
    Vec7 target = Vec7::Zero();
    target[0] = 0.02 * static_cast<double>(sequence - 1U);
    ASSERT_TRUE(reference
                    .acceptTarget(
                        target, sequence,
                        1'000'000'000LL +
                            static_cast<std::int64_t>(sequence - 1U) *
                                10'000'000LL,
                        85U, model)
                    .accepted);
    for (int control_step = 0; control_step < 2; ++control_step) {
      (void)reference.step(model, 0.005, true);
    }
  }

  Vec7 stopped_target = Vec7::Zero();
  stopped_target[0] = 0.20;
  double maximum_position = -1.0;
  double velocity_at_maximum = 0.0;
  double acceleration_at_maximum = 0.0;
  for (std::uint64_t sequence = 12U; sequence <= 71U; ++sequence) {
    ASSERT_TRUE(reference
                    .acceptTarget(
                        stopped_target, sequence,
                        1'000'000'000LL +
                            static_cast<std::int64_t>(sequence - 1U) *
                                10'000'000LL,
                        85U, model)
                    .accepted);
    for (int control_step = 0; control_step < 2; ++control_step) {
      const auto result = reference.step(model, 0.005, true);
      if (result.q[0] > maximum_position) {
        maximum_position = result.q[0];
        velocity_at_maximum = result.qdot[0];
        acceleration_at_maximum = result.qddot[0];
      }
      EXPECT_LE(result.qddot.head<3>().cwiseAbs().maxCoeff(), 60.0 + 1.0e-9);
      EXPECT_LE(result.jerk.head<3>().cwiseAbs().maxCoeff(), 1000.0 + 1.0e-9);
      EXPECT_LE(result.qdot.cwiseAbs().maxCoeff(), 4.0 + 1.0e-9);
    }
  }

  EXPECT_LE(maximum_position, 0.215)
      << "qdot_at_max=" << velocity_at_maximum
      << " qddot_at_max=" << acceleration_at_maximum;
}

TEST(SparkFeedforwardReference, UnwrapsJointAnglesAndRejectsBranchJump) {
  SparkFeedforwardReference7 reference = makeReference();
  ArmMotionState model;
  model.q[0] = 3.13;
  reference.reset(model, 85U);
  Vec7 first = model.q;
  ASSERT_TRUE(reference.acceptTarget(first, 1U, 1'000'000'000LL, 85U, model)
                  .accepted);

  Vec7 wrapped = first;
  wrapped[0] = -3.13;
  const auto continuous =
      reference.acceptTarget(wrapped, 2U, 1'011'000'000LL, 85U, model);
  ASSERT_TRUE(continuous.accepted);
  EXPECT_TRUE(continuous.dt_valid);
  EXPECT_NEAR(continuous.continuous_target[0],
              3.13 + (2.0 * std::numbers::pi - 6.26), 1.0e-12);

  Vec7 jump = wrapped;
  jump[1] = 0.5;
  const auto limited =
      reference.acceptTarget(jump, 3U, 1'022'000'000LL, 85U, model);
  EXPECT_TRUE(limited.accepted);
  EXPECT_TRUE(limited.jump_rejected);
  EXPECT_EQ(limited.detail, "feedforward_joint_jump_rate_limited");
  EXPECT_NEAR(limited.continuous_target[1], 0.35, 1.0e-12);

  const auto recovered =
      reference.acceptTarget(jump, 4U, 1'033'000'000LL, 85U, model);
  EXPECT_TRUE(recovered.accepted);
  EXPECT_FALSE(recovered.jump_rejected);
  EXPECT_NEAR(recovered.continuous_target[1], 0.5, 1.0e-12);
}

TEST(SparkFeedforwardReference, EnforcesAccelerationAndJerkDuringMotionAndStop) {
  SparkFeedforwardReference7 reference = makeReference();
  ArmMotionState model;
  reference.reset(model, 85U);
  ASSERT_TRUE(reference
                  .acceptTarget(Vec7::Zero(), 1U, 1'000'000'000LL, 85U,
                                model)
                  .accepted);
  Vec7 moving = Vec7::Constant(0.30);
  ASSERT_TRUE(reference
                  .acceptTarget(moving, 2U, 1'011'000'000LL, 85U, model)
                  .accepted);

  SparkFeedforwardReferenceResult result;
  for (int step = 0; step < 20; ++step) {
    result = reference.step(model, 0.005, true);
    EXPECT_LE(result.qddot.head<3>().cwiseAbs().maxCoeff(), 60.0 + 1.0e-9);
    EXPECT_LE(result.qddot.tail<4>().cwiseAbs().maxCoeff(), 90.0 + 1.0e-9);
    EXPECT_LE(result.jerk.head<3>().cwiseAbs().maxCoeff(), 1000.0 + 1.0e-9);
    EXPECT_LE(result.jerk.tail<4>().cwiseAbs().maxCoeff(), 1500.0 + 1.0e-9);
    EXPECT_LE(result.qdot.cwiseAbs().maxCoeff(), 4.0 + 1.0e-9);
  }

  for (int step = 0; step < 200; ++step) {
    result = reference.step(model, 0.005, false);
  }
  EXPECT_EQ(result.state, SparkFeedforwardState::kHold);
  EXPECT_NEAR(result.qdot.norm(), 0.0, 1.0e-9);
  EXPECT_NEAR(result.qddot.norm(), 0.0, 1.0e-9);
  EXPECT_NEAR(result.activation, 0.0, 1.0e-9);
}

TEST(SparkFeedforwardReference, EpochChangeResynchronizesToModel) {
  SparkFeedforwardReference7 reference = makeReference();
  ArmMotionState model;
  reference.reset(model, 85U);
  ASSERT_TRUE(reference
                  .acceptTarget(Vec7::Zero(), 1U, 1'000'000'000LL, 85U,
                                model)
                  .accepted);
  model.q = Vec7::Constant(-0.2);

  const auto decision = reference.acceptTarget(
      Vec7::Constant(0.4), 1U, 2'000'000'000LL, 86U, model);
  const auto result = reference.step(model, 0.005, true);

  EXPECT_TRUE(decision.epoch_reset);
  EXPECT_TRUE(decision.accepted);
  EXPECT_TRUE(result.q.isApprox(model.q, 1.0e-12));
  EXPECT_NEAR(result.qdot.norm(), 0.0, 1.0e-12);
}

}  // namespace
}  // namespace tianji_qp_ik
