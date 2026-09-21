#include "tianji_qp_ik/velocity_ik.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace tianji_qp_ik {
namespace {

TEST(VelocityIk, IntersectsVelocityAndOneStepPositionBounds) {
  JointLimitConfig config;
  config.margin_rad = 0.05;
  config.velocity_scale = 1.0;
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-1.0);
  limits.upper_position = Vec7::Constant(1.0);
  limits.velocity = Vec7::Constant(3.0);
  Vec7 q = Vec7::Zero();
  q[0] = 0.949;

  const JointVelocityBounds bounds =
      computeJointVelocityBounds(q, Vec7::Zero(), Vec7::Zero(), limits,
                                 config, 0.005);

  EXPECT_NEAR(bounds.upper[0], 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(bounds.lower[1], -3.0);
  EXPECT_EQ(bounds.upper_source[0], BoundSource::kPosition);
  EXPECT_EQ(bounds.upper_source[1], BoundSource::kVelocity);
}

TEST(VelocityIk, ReportsPositionAsLowerBoundSourceNearLowerMargin) {
  JointLimitConfig config;
  config.margin_rad = 0.05;
  config.velocity_scale = 0.5;
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-1.0);
  limits.upper_position = Vec7::Constant(1.0);
  limits.velocity = Vec7::Constant(4.0);
  Vec7 q = Vec7::Zero();
  q[2] = -0.949;

  const JointVelocityBounds bounds =
      computeJointVelocityBounds(q, Vec7::Zero(), Vec7::Zero(), limits,
                                 config, 0.005);

  EXPECT_NEAR(bounds.lower[2], -0.2, 1e-12);
  EXPECT_EQ(bounds.lower_source[2], BoundSource::kPosition);
  EXPECT_DOUBLE_EQ(bounds.upper[2], 2.0);
  EXPECT_EQ(bounds.upper_source[2], BoundSource::kVelocity);
}

TEST(VelocityIk, IntersectsHardAccelerationBounds) {
  JointLimitConfig config;
  config.margin_rad = 0.05;
  config.velocity_scale = 1.0;
  config.max_acceleration_rad_s2.setConstant(20.0);
  config.braking_acceleration_rad_s2.setConstant(15.0);
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-1.0);
  limits.upper_position = Vec7::Constant(1.0);
  limits.velocity = Vec7::Constant(3.0);
  Vec7 q = Vec7::Zero();
  const Vec7 qdot_previous = Vec7::Constant(0.5);

  const JointVelocityBounds bounds = computeJointVelocityBounds(
      q, qdot_previous, Vec7::Zero(), limits, config, 0.005);

  EXPECT_NEAR(bounds.lower[1], 0.4, 1e-12);
  EXPECT_NEAR(bounds.upper[1], 0.6, 1e-12);
  EXPECT_EQ(bounds.lower_source[1], BoundSource::kAcceleration);
  EXPECT_EQ(bounds.upper_source[1], BoundSource::kAcceleration);
}

TEST(VelocityIk, AppliesPerJointAccelerationLimits) {
  JointLimitConfig config;
  config.margin_rad = 0.05;
  config.velocity_scale = 1.0;
  config.max_acceleration_rad_s2 << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0,
      70.0;
  config.braking_acceleration_rad_s2.setConstant(100.0);
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-10.0);
  limits.upper_position = Vec7::Constant(10.0);
  limits.velocity = Vec7::Constant(10.0);
  const JointVelocityBounds bounds = computeJointVelocityBounds(
      Vec7::Zero(), Vec7::Zero(), Vec7::Zero(), limits, config, 0.01);
  EXPECT_NEAR(bounds.upper[0], 0.1, 1e-12);
  EXPECT_NEAR(bounds.upper[1], 0.2, 1e-12);
  EXPECT_NEAR(bounds.upper[6], 0.7, 1e-12);
}

TEST(VelocityIk, IntersectsHardJointJerkBounds) {
  JointLimitConfig config;
  config.margin_rad = 0.05;
  config.velocity_scale = 1.0;
  config.max_acceleration_rad_s2.setConstant(100.0);
  config.braking_acceleration_rad_s2.setConstant(100.0);
  config.hard_jerk_enabled = true;
  config.max_jerk_rad_s3.setConstant(200.0);
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-10.0);
  limits.upper_position = Vec7::Constant(10.0);
  limits.velocity = Vec7::Constant(10.0);
  const Vec7 qdot_previous = Vec7::Constant(0.5);
  const Vec7 qddot_previous = Vec7::Constant(2.0);
  constexpr double dt = 0.01;

  const JointVelocityBounds bounds = computeJointVelocityBounds(
      Vec7::Zero(), qdot_previous, qddot_previous, limits, config, dt);

  // qddot may change by at most jerk * dt = 2 rad/s^2, hence the next
  // acceleration lies in [0, 4] and qdot in [0.5, 0.54] rad/s.
  EXPECT_NEAR(bounds.lower[0], 0.5, 1e-12);
  EXPECT_NEAR(bounds.upper[0], 0.54, 1e-12);
  EXPECT_EQ(bounds.lower_source[0], BoundSource::kJerk);
  EXPECT_EQ(bounds.upper_source[0], BoundSource::kJerk);
}

TEST(VelocityIk, ShrinksVelocityUsingBrakingDistance) {
  JointLimitConfig config;
  config.margin_rad = 0.05;
  config.velocity_scale = 1.0;
  config.max_acceleration_rad_s2.setConstant(1.0e6);
  config.braking_acceleration_rad_s2.setConstant(15.0);
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-1.0);
  limits.upper_position = Vec7::Constant(1.0);
  limits.velocity = Vec7::Constant(3.0);
  Vec7 q = Vec7::Zero();
  q[0] = 0.90;

  const JointVelocityBounds bounds = computeJointVelocityBounds(
      q, Vec7::Zero(), Vec7::Zero(), limits, config, 0.005);

  EXPECT_NEAR(bounds.upper[0], std::sqrt(1.5), 1e-12);
  EXPECT_EQ(bounds.upper_source[0], BoundSource::kBraking);
}

TEST(VelocityIk, PreservesPositionFeasibilityWithBoundedAcceleration) {
  JointLimitConfig config;
  config.margin_rad = 0.0;
  config.velocity_scale = 1.0;
  config.max_acceleration_rad_s2.setConstant(2.0);
  config.braking_acceleration_rad_s2.setConstant(2.0);
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-100.0);
  limits.upper_position = Vec7::Constant(1.0);
  limits.velocity = Vec7::Constant(100.0);
  Vec7 q = Vec7::Zero();
  Vec7 qdot_previous = Vec7::Zero();
  constexpr double dt = 0.1;

  for (int step = 0; step < 80; ++step) {
    const JointVelocityBounds bounds = computeJointVelocityBounds(
        q, qdot_previous, Vec7::Zero(), limits, config, dt);
    ASSERT_LE(bounds.lower[0], bounds.upper[0]) << "step=" << step;
    const double qdot = bounds.upper[0];
    EXPECT_LE(std::abs(qdot - qdot_previous[0]),
              config.max_acceleration_rad_s2[0] * dt + 1e-12)
        << "step=" << step;
    q[0] += qdot * dt;
    EXPECT_LE(q[0], limits.upper_position[0] + 1e-12) << "step=" << step;
    qdot_previous[0] = qdot;
  }
}

TEST(VelocityIk, PreservesJerkFeasibilityWhileApproachingAPositionLimit) {
  JointLimitConfig config;
  config.margin_rad = 0.0;
  config.velocity_scale = 1.0;
  config.max_acceleration_rad_s2.setConstant(2.0);
  config.braking_acceleration_rad_s2.setConstant(2.0);
  config.hard_jerk_enabled = true;
  config.max_jerk_rad_s3.setConstant(3.0);
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-100.0);
  limits.upper_position = Vec7::Constant(1.0);
  limits.velocity = Vec7::Constant(100.0);
  Vec7 q = Vec7::Zero();
  Vec7 qdot_previous = Vec7::Zero();
  Vec7 qddot_previous = Vec7::Zero();
  constexpr double dt = 0.1;

  for (int step = 0; step < 100; ++step) {
    const JointVelocityBounds bounds = computeJointVelocityBounds(
        q, qdot_previous, qddot_previous, limits, config, dt);
    ASSERT_LE(bounds.lower[0], bounds.upper[0]) << "step=" << step;
    const double qdot = bounds.upper[0];
    const double qddot = (qdot - qdot_previous[0]) / dt;
    EXPECT_LE(std::abs(qddot),
              config.max_acceleration_rad_s2[0] + 1e-12)
        << "step=" << step;
    EXPECT_LE(std::abs(qddot - qddot_previous[0]),
              config.max_jerk_rad_s3[0] * dt + 1e-12)
        << "step=" << step;
    q[0] += qdot * dt;
    EXPECT_LE(q[0], limits.upper_position[0] + 1e-12) << "step=" << step;
    qdot_previous[0] = qdot;
    qddot_previous[0] = qddot;
  }
}

TEST(VelocityIk, JerkAwareBoundsRemainFeasibleForInteriorCommands) {
  JointLimitConfig config;
  config.margin_rad = 0.05;
  config.velocity_scale = 1.0;
  config.max_acceleration_rad_s2.setConstant(60.0);
  config.braking_acceleration_rad_s2.setConstant(45.0);
  config.hard_jerk_enabled = true;
  config.max_jerk_rad_s3.setConstant(1000.0);
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-2.0);
  limits.upper_position = Vec7::Constant(2.0);
  limits.velocity = Vec7::Constant(3.0);
  Vec7 q = Vec7::Zero();
  Vec7 qdot_previous = Vec7::Zero();
  Vec7 qddot_previous = Vec7::Zero();
  constexpr double dt = 0.005;
  std::mt19937 generator(42U);
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  for (int step = 0; step < 20000; ++step) {
    const JointVelocityBounds bounds = computeJointVelocityBounds(
        q, qdot_previous, qddot_previous, limits, config, dt);
    ASSERT_LE(bounds.lower[0], bounds.upper[0])
        << "step=" << step << " q=" << q[0]
        << " qdot=" << qdot_previous[0]
        << " qddot=" << qddot_previous[0];
    const double blend = unit(generator);
    const double qdot =
        (1.0 - blend) * bounds.lower[0] + blend * bounds.upper[0];
    const double qddot = (qdot - qdot_previous[0]) / dt;
    ASSERT_LE(std::abs(qddot - qddot_previous[0]),
              config.max_jerk_rad_s3[0] * dt + 1e-8)
        << "step=" << step;
    q[0] += qdot * dt;
    ASSERT_GE(q[0], limits.lower_position[0] + config.margin_rad - 1e-8)
        << "step=" << step;
    ASSERT_LE(q[0], limits.upper_position[0] - config.margin_rad + 1e-8)
        << "step=" << step;
    qdot_previous[0] = qdot;
    qddot_previous[0] = qddot;
  }
}

TEST(VelocityIk, SafetyBoundOverridesConflictingAccelerationBound) {
  JointLimitConfig config;
  config.margin_rad = 0.05;
  config.velocity_scale = 1.0;
  config.max_acceleration_rad_s2.setConstant(20.0);
  config.braking_acceleration_rad_s2.setConstant(15.0);
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-1.0);
  limits.upper_position = Vec7::Constant(1.0);
  limits.velocity = Vec7::Constant(3.0);
  Vec7 q = Vec7::Zero();
  q[0] = 0.949;
  Vec7 qdot_previous = Vec7::Zero();
  qdot_previous[0] = 1.0;

  const JointVelocityBounds bounds = computeJointVelocityBounds(
      q, qdot_previous, Vec7::Zero(), limits, config, 0.005);

  EXPECT_LE(bounds.lower[0], bounds.upper[0]);
  EXPECT_NEAR(bounds.upper[0], 0.15, 1e-12);
  EXPECT_EQ(bounds.upper_source[0], BoundSource::kBraking);
}

TEST(VelocityIk,
     PostureReferenceUsesOnlyCartesianNullspaceAndPreservesHardBounds) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  const Vec7 primary =
      (Vec7() << 0.10, -0.20, 0.30, -0.40, 0.50, -0.60, 0.0).finished();
  JointVelocityPostureTask task;
  task.active = true;
  task.target = primary;
  task.target[6] = 0.8;

  Vec7 lower = Vec7::Constant(-1.0);
  Vec7 upper = Vec7::Constant(1.0);
  upper[6] = 0.25;
  const JointVelocityNullspaceResult result =
      refinePostureVelocityInNullspace(primary, jacobian, task, lower, upper,
                                       1.0e-10);

  ASSERT_TRUE(result.active);
  EXPECT_NEAR(result.value[6], 0.25, 1.0e-12);
  EXPECT_TRUE((jacobian * (result.value - primary)).isZero(1.0e-12));
  EXPECT_LE(result.value[6], upper[6]);
  EXPECT_NEAR(result.scale, 0.3125, 1.0e-12);
}

TEST(VelocityIk, PostureReferencePreservesOptionalLinearHardConstraint) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  JointVelocityPostureTask task;
  task.active = true;
  task.target[6] = -0.8;
  LinearJointConstraint outward;
  outward.active = true;
  outward.jacobian[6] = 1.0;
  outward.lower = -0.05;

  const JointVelocityNullspaceResult result =
      refinePostureVelocityInNullspace(
          Vec7::Zero(), jacobian, task, Vec7::Constant(-1.0),
          Vec7::Constant(1.0), 1.0e-10, outward);

  ASSERT_TRUE(result.active);
  EXPECT_NEAR(result.value[6], outward.lower, 1.0e-12);
  EXPECT_GE(outward.jacobian.dot(result.value), outward.lower - 1.0e-12);
  EXPECT_TRUE((jacobian * result.value).isZero(1.0e-12));
}

TEST(VelocityIk, FactoryCreatesTheSelectedAlgorithm) {
  QpIkConfig config;
  auto hierarchical = makeArmVelocityIk(IkAlgorithm::kHierarchicalQp, config);
  auto dls = makeArmVelocityIk(IkAlgorithm::kNullspaceDls, config);
  auto headroom = makeArmVelocityIk(
      IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp, config);

  ASSERT_NE(hierarchical, nullptr);
  ASSERT_NE(dls, nullptr);
  ASSERT_NE(headroom, nullptr);
  EXPECT_EQ(hierarchical->algorithm(), IkAlgorithm::kHierarchicalQp);
  EXPECT_EQ(dls->algorithm(), IkAlgorithm::kNullspaceDls);
  // Spark modes select guidance outside the shared hierarchical velocity-QP
  // backend, so the backend keeps reporting its solver identity.
  EXPECT_EQ(headroom->algorithm(), IkAlgorithm::kHierarchicalQp);
}

}  // namespace
}  // namespace tianji_qp_ik
