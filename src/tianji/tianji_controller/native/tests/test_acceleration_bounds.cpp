#include "tianji_qp_ik/acceleration_bounds.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace tianji_qp_ik {
namespace {

ArmLimits wideLimits() {
  ArmLimits limits;
  limits.lower_position.setConstant(-100.0);
  limits.upper_position.setConstant(100.0);
  limits.velocity.setConstant(100.0);
  return limits;
}

JointAccelerationLimitConfig permissiveConfig() {
  JointAccelerationLimitConfig config;
  config.margin_rad = 0.0;
  config.velocity_scale = 1.0;
  config.max_acceleration_rad_s2.setConstant(1.0e6);
  config.braking_acceleration_rad_s2.setConstant(1.0e6);
  config.hard_jerk_enabled = false;
  config.max_jerk_rad_s3.setConstant(1.0e6);
  return config;
}

TEST(AccelerationBounds, AppliesConfiguredAccelerationLimit) {
  JointAccelerationLimitConfig config = permissiveConfig();
  config.max_acceleration_rad_s2.setConstant(20.0);
  const JointAccelerationBounds bounds = computeJointAccelerationBounds(
      Vec7::Zero(), Vec7::Zero(), Vec7::Zero(), wideLimits(), config, 0.005);
  EXPECT_TRUE(bounds.lower.isApprox(Vec7::Constant(-20.0)));
  EXPECT_TRUE(bounds.upper.isApprox(Vec7::Constant(20.0)));
}

TEST(AccelerationBounds, AppliesPerJointAccelerationAndJerkLimits) {
  JointAccelerationLimitConfig config = permissiveConfig();
  config.max_acceleration_rad_s2 << 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
      12.0;
  config.hard_jerk_enabled = true;
  config.max_jerk_rad_s3 << 100.0, 200.0, 300.0, 400.0, 500.0, 600.0,
      700.0;
  const JointAccelerationBounds bounds = computeJointAccelerationBounds(
      Vec7::Zero(), Vec7::Zero(), Vec7::Zero(), wideLimits(), config, 0.01);
  EXPECT_NEAR(bounds.upper[0], 1.0, 1e-12);
  EXPECT_NEAR(bounds.upper[1], 2.0, 1e-12);
  EXPECT_NEAR(bounds.upper[6], 7.0, 1e-12);
}

TEST(AccelerationBounds, ConvertsPredictedVelocityToAccelerationBound) {
  ArmLimits limits = wideLimits();
  limits.velocity.setOnes();
  Vec7 qdot_ref = Vec7::Zero();
  qdot_ref[0] = 0.9;
  const JointAccelerationBounds bounds = computeJointAccelerationBounds(
      Vec7::Zero(), qdot_ref, Vec7::Zero(), limits, permissiveConfig(), 0.1);
  EXPECT_NEAR(bounds.upper[0], 1.0, 1e-12);
}

TEST(AccelerationBounds, ConvertsSecondOrderPositionPrediction) {
  ArmLimits limits = wideLimits();
  limits.upper_position[0] = 1.0;
  Vec7 q_ref = Vec7::Zero();
  Vec7 qdot_ref = Vec7::Zero();
  q_ref[0] = 0.9;
  qdot_ref[0] = 0.5;
  const JointAccelerationBounds bounds = computeJointAccelerationBounds(
      q_ref, qdot_ref, Vec7::Zero(), limits, permissiveConfig(), 0.1);
  EXPECT_NEAR(bounds.upper[0], 10.0, 3e-4);
  EXPECT_LE(bounds.upper[0], 10.0);
}

TEST(AccelerationBounds, EnablesHardJerkOnlyWhenConfigured) {
  JointAccelerationLimitConfig config = permissiveConfig();
  config.hard_jerk_enabled = true;
  config.max_jerk_rad_s3.setConstant(3.0);
  Vec7 previous = Vec7::Constant(2.0);
  const JointAccelerationBounds bounded = computeJointAccelerationBounds(
      Vec7::Zero(), Vec7::Zero(), previous, wideLimits(), config, 0.1);
  EXPECT_NEAR(bounded.lower[0], 1.7, 1e-12);
  EXPECT_NEAR(bounded.upper[0], 2.3, 1e-12);

  config.hard_jerk_enabled = false;
  const JointAccelerationBounds disabled = computeJointAccelerationBounds(
      Vec7::Zero(), Vec7::Zero(), previous, wideLimits(), config, 0.1);
  EXPECT_LT(disabled.lower[0], -999.0);
  EXPECT_GT(disabled.upper[0], 999.0);
}

TEST(AccelerationBounds, PreservesVelocityFeasibilityWhileJerkLimited) {
  ArmLimits limits = wideLimits();
  limits.velocity.setOnes();
  JointAccelerationLimitConfig config = permissiveConfig();
  config.max_acceleration_rad_s2.setConstant(2.0);
  config.hard_jerk_enabled = true;
  config.max_jerk_rad_s3.setConstant(3.0);
  Vec7 q = Vec7::Zero();
  Vec7 qdot = Vec7::Zero();
  Vec7 qddot_previous = Vec7::Zero();
  constexpr double dt = 0.1;

  for (int step = 0; step < 80; ++step) {
    const JointAccelerationBounds bounds = computeJointAccelerationBounds(
        q, qdot, qddot_previous, limits, config, dt);
    ASSERT_LE(bounds.lower[0], bounds.upper[0] + 1e-12)
        << "step=" << step << " q=" << q[0] << " qdot=" << qdot[0]
        << " previous=" << qddot_previous[0];
    const double qddot = bounds.upper[0];
    EXPECT_LE(std::abs(qddot - qddot_previous[0]),
              config.max_jerk_rad_s3[0] * dt + 1e-12)
        << "step=" << step;
    q[0] += qdot[0] * dt + 0.5 * qddot * dt * dt;
    qdot[0] += qddot * dt;
    EXPECT_LE(qdot[0], limits.velocity[0] + 1e-12) << "step=" << step;
    qddot_previous[0] = qddot;
  }
}

TEST(AccelerationBounds, PreservesPositionFeasibilityWhileJerkLimited) {
  ArmLimits limits = wideLimits();
  limits.upper_position.setOnes();
  JointAccelerationLimitConfig config = permissiveConfig();
  config.max_acceleration_rad_s2.setConstant(2.0);
  config.braking_acceleration_rad_s2.setConstant(2.0);
  config.hard_jerk_enabled = true;
  config.max_jerk_rad_s3.setConstant(3.0);
  Vec7 q = Vec7::Zero();
  Vec7 qdot = Vec7::Zero();
  Vec7 qddot_previous = Vec7::Zero();
  constexpr double dt = 0.1;

  for (int step = 0; step < 80; ++step) {
    const JointAccelerationBounds bounds = computeJointAccelerationBounds(
        q, qdot, qddot_previous, limits, config, dt);
    ASSERT_LE(bounds.lower[0], bounds.upper[0] + 1e-12)
        << "step=" << step << " q=" << q[0] << " qdot=" << qdot[0]
        << " previous=" << qddot_previous[0];
    const double qddot = bounds.upper[0];
    EXPECT_LE(std::abs(qddot - qddot_previous[0]),
              config.max_jerk_rad_s3[0] * dt + 1e-12)
        << "step=" << step;
    q[0] += qdot[0] * dt + 0.5 * qddot * dt * dt;
    qdot[0] += qddot * dt;
    EXPECT_LE(q[0], limits.upper_position[0] + 1e-12) << "step=" << step;
    qddot_previous[0] = qddot;
  }
}

TEST(AccelerationBounds, AppliesBrakingVelocityEnvelope) {
  ArmLimits limits = wideLimits();
  limits.upper_position[0] = 1.0;
  JointAccelerationLimitConfig config = permissiveConfig();
  config.braking_acceleration_rad_s2.setConstant(2.0);
  Vec7 q = Vec7::Zero();
  Vec7 qdot_ref = Vec7::Zero();
  q[0] = 0.9;
  qdot_ref[0] = 0.6;
  const JointAccelerationBounds bounds = computeJointAccelerationBounds(
      q, qdot_ref, Vec7::Zero(), limits, config, 0.1);
  const double acceleration = bounds.upper[0];
  const double q_next = q[0] + qdot_ref[0] * 0.1 +
                        0.5 * acceleration * 0.1 * 0.1;
  const double qdot_next = qdot_ref[0] + acceleration * 0.1;
  EXPECT_LE(qdot_next,
            std::sqrt(2.0 * config.braking_acceleration_rad_s2[0] *
                      (limits.upper_position[0] - q_next)) +
                1e-12);
  EXPECT_LT(acceleration,
            (std::sqrt(2.0 * 2.0 * 0.1) - 0.6) / 0.1);
}

TEST(AccelerationBounds, PreservesBrakingEnvelopeAcrossIntegrationSteps) {
  ArmLimits limits = wideLimits();
  limits.lower_position[0] = -1.0;
  limits.upper_position[0] = 1.0;
  JointAccelerationLimitConfig config = permissiveConfig();
  config.max_acceleration_rad_s2.setConstant(20.0);
  config.braking_acceleration_rad_s2.setConstant(15.0);
  Vec7 q = Vec7::Zero();
  Vec7 qdot = Vec7::Zero();
  q[0] = 0.90;
  qdot[0] = 0.50;

  for (int step = 0; step < 80; ++step) {
    const JointAccelerationBounds bounds = computeJointAccelerationBounds(
        q, qdot, Vec7::Zero(), limits, config, 0.005);
    ASSERT_LE(bounds.lower[0], bounds.upper[0]) << "step=" << step;
    const double acceleration = bounds.upper[0];
    q[0] += qdot[0] * 0.005 + 0.5 * acceleration * 0.005 * 0.005;
    qdot[0] += acceleration * 0.005;
    const double distance = limits.upper_position[0] - q[0];
    ASSERT_GE(distance, -1e-12) << "step=" << step;
    EXPECT_LE(qdot[0],
              std::sqrt(2.0 * config.braking_acceleration_rad_s2[0] *
                        std::max(distance, 0.0)) +
                  1e-10)
        << "step=" << step;
  }
}

TEST(AccelerationBounds, PreservesInfeasibleHardIntersection) {
  ArmLimits limits = wideLimits();
  limits.lower_position[0] = -1.0;
  limits.upper_position[0] = 1.0;
  Vec7 q_ref = Vec7::Zero();
  q_ref[0] = 1.2;
  JointAccelerationLimitConfig config = permissiveConfig();
  config.max_acceleration_rad_s2.setConstant(1.0);
  const JointAccelerationBounds bounds = computeJointAccelerationBounds(
      q_ref, Vec7::Zero(), Vec7::Zero(), limits, config, 0.1);
  EXPECT_GT(bounds.lower[0], bounds.upper[0]);
}

TEST(AccelerationBounds, CollapsesRoundoffGapAtJerkViablePositionBoundary) {
  ArmLimits limits = wideLimits();
  limits.lower_position[2] = -3.1067;
  limits.upper_position[2] = 3.1067;
  limits.velocity[2] = 3.1416;
  Vec7 q = Vec7::Zero();
  Vec7 qdot = Vec7::Zero();
  Vec7 qddot_previous = Vec7::Zero();
  q[2] = -1.9792925800739392;
  qdot[2] = -3.0588030273988163;
  qddot_previous[2] = -61.559394533215475;
  JointAccelerationLimitConfig config = permissiveConfig();
  config.margin_rad = 0.05;
  config.max_acceleration_rad_s2.setConstant(90.0);
  config.braking_acceleration_rad_s2.setConstant(67.5);
  config.hard_jerk_enabled = true;
  config.max_jerk_rad_s3.setConstant(9000.0);

  const JointAccelerationBounds bounds = computeJointAccelerationBounds(
      q, qdot, qddot_previous, limits, config, 0.005);

  EXPECT_LE(bounds.lower[2], bounds.upper[2]);
  EXPECT_NEAR(bounds.lower[2], bounds.upper[2], 1e-7);
}

TEST(AccelerationBounds,
     SeedsAccelerationHistoryWhenResetStateIsNearBrakingBoundary) {
  ArmLimits limits = wideLimits();
  limits.lower_position[5] = -1.0472;
  limits.upper_position[5] = 1.0472;
  limits.velocity[5] = 4.0;
  Vec7 q = Vec7::Zero();
  Vec7 qdot = Vec7::Zero();
  q[5] = 0.80992196191674393;
  qdot[5] = 3.990005425148671;

  JointAccelerationLimitConfig config = permissiveConfig();
  config.margin_rad = 0.05;
  config.max_acceleration_rad_s2.setConstant(90.0);
  config.braking_acceleration_rad_s2.setConstant(67.5);
  config.hard_jerk_enabled = true;
  config.max_jerk_rad_s3.setConstant(1500.0);
  constexpr double kDt = 0.005;

  const JointAccelerationBounds reset_bounds = computeJointAccelerationBounds(
      q, qdot, Vec7::Zero(), limits, config, kDt);
  ASSERT_GT(reset_bounds.lower[5], reset_bounds.upper[5]);

  const Vec7 seeded = seedPreviousAccelerationForFeasibility(
      q, qdot, limits, config, kDt);
  const JointAccelerationBounds seeded_bounds = computeJointAccelerationBounds(
      q, qdot, seeded, limits, config, kDt);
  EXPECT_LE(seeded_bounds.lower[5], seeded_bounds.upper[5] + 1e-12);
  EXPECT_LT(seeded[5], 0.0);
  EXPECT_LE(std::abs(seeded[5]), config.max_acceleration_rad_s2[5]);
}

}  // namespace
}  // namespace tianji_qp_ik
