#include "tianji_qp_ik/upper_arm_outward.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace tianji_qp_ik {
namespace {

UpperArmOutwardConfig testConfig() {
  UpperArmOutwardConfig config;
  config.minimum_outward_distance_m = 0.0;
  config.velocity_gain = 8.0;
  config.acceleration_kp = 100.0;
  config.acceleration_kd = 20.0;
  return config;
}

TEST(UpperArmOutward, UsesLeftPositiveYAndRightNegativeY) {
  const Eigen::Vector3d shoulder = Eigen::Vector3d::Zero();
  const Mat37 shoulder_jacobian = Mat37::Zero();
  Mat37 elbow_jacobian = Mat37::Zero();
  elbow_jacobian(1, 2) = 0.5;

  const auto left = computeUpperArmOutwardState(
      ArmSide::kLeft, shoulder, Eigen::Vector3d(0.3, 0.1, -0.2),
      shoulder_jacobian, elbow_jacobian, 0.0);
  const auto right = computeUpperArmOutwardState(
      ArmSide::kRight, shoulder, Eigen::Vector3d(-0.4, -0.1, 0.7),
      shoulder_jacobian, elbow_jacobian, 0.0);

  ASSERT_TRUE(left.valid);
  ASSERT_TRUE(right.valid);
  EXPECT_NEAR(left.distance_m, 0.1, 1e-12);
  EXPECT_NEAR(right.distance_m, 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(left.jacobian[2], 0.5);
  EXPECT_DOUBLE_EQ(right.jacobian[2], -0.5);
}

TEST(UpperArmOutward, IgnoresElbowHeightAndForwardBackwardPosition) {
  const Mat37 zero = Mat37::Zero();
  const auto first = computeUpperArmOutwardState(
      ArmSide::kLeft, Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.4, 0.03, -0.5), zero, zero, 0.01);
  const auto second = computeUpperArmOutwardState(
      ArmSide::kLeft, Eigen::Vector3d::Zero(),
      Eigen::Vector3d(-0.8, 0.03, 1.2), zero, zero, 0.01);

  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_NEAR(first.distance_m, 0.02, 1e-12);
  EXPECT_NEAR(second.distance_m, 0.02, 1e-12);
}

TEST(UpperArmOutward, RejectsNonFiniteGeometryWithoutConstraint) {
  Mat37 elbow_jacobian = Mat37::Zero();
  elbow_jacobian(0, 0) = std::numeric_limits<double>::quiet_NaN();
  const auto state = computeUpperArmOutwardState(
      ArmSide::kLeft, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY(),
      Mat37::Zero(), elbow_jacobian, 0.0);
  const auto constraint = makeVelocityOutwardConstraint(
      state, Vec7::Constant(-1.0), Vec7::Constant(1.0), testConfig(), 0.005);

  EXPECT_FALSE(state.valid);
  EXPECT_FALSE(constraint.active);
}

TEST(UpperArmOutward, VelocityBarrierStopsInwardMotionAtPlane) {
  UpperArmOutwardState state;
  state.valid = true;
  state.distance_m = 0.0;
  state.jacobian[2] = 0.5;

  const auto constraint = makeVelocityOutwardConstraint(
      state, Vec7::Constant(-2.0), Vec7::Constant(2.0), testConfig(), 0.005);

  EXPECT_TRUE(constraint.active);
  EXPECT_DOUBLE_EQ(constraint.requested_lower, 0.0);
  EXPECT_DOUBLE_EQ(constraint.lower, 0.0);
  EXPECT_TRUE(std::isinf(constraint.upper));
  EXPECT_FALSE(constraint.feasibility_clipped);
}

TEST(UpperArmOutward, VelocityBarrierAllowsBoundedInwardMotionOutsidePlane) {
  UpperArmOutwardState state;
  state.valid = true;
  state.distance_m = 0.05;
  state.jacobian[0] = 1.0;

  const auto constraint = makeVelocityOutwardConstraint(
      state, Vec7::Constant(-2.0), Vec7::Constant(2.0), testConfig(), 0.005);

  EXPECT_NEAR(constraint.lower, -0.4, 1e-12);
}

TEST(UpperArmOutward, AccelerationBarrierIncludesBiasAndDamping) {
  UpperArmOutwardState state;
  state.valid = true;
  state.distance_m = 0.02;
  state.jacobian[0] = 1.0;
  Vec7 qdot = Vec7::Zero();
  qdot[0] = -0.3;

  const auto constraint = makeAccelerationOutwardConstraint(
      state, qdot, 0.4, Vec7::Constant(-20.0), Vec7::Constant(20.0),
      testConfig(), 0.005);

  EXPECT_NEAR(constraint.requested_lower,
              -0.4 - 20.0 * -0.3 - 100.0 * 0.02, 1e-12);
  EXPECT_NEAR(constraint.lower, 3.6, 1e-12);
}

TEST(UpperArmOutward, ClipsRecoveryToMaximumAchievableBoxValue) {
  UpperArmOutwardState state;
  state.valid = true;
  state.distance_m = -1.0;
  state.jacobian << 2.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Vec7 lower = Vec7::Constant(-1.0);
  Vec7 upper = Vec7::Constant(1.0);

  const auto constraint = makeVelocityOutwardConstraint(
      state, lower, upper, testConfig(), 0.005);

  EXPECT_TRUE(constraint.active);
  EXPECT_DOUBLE_EQ(constraint.requested_lower, 200.0);
  EXPECT_LT(constraint.lower, 5.0);
  EXPECT_GT(constraint.lower, 5.0 - 1.0e-4);
  EXPECT_TRUE(constraint.feasibility_clipped);
  EXPECT_TRUE(constraint.viability_clipped);
}

TEST(UpperArmOutward, VelocityBarrierPreventsOneStepPlaneCrossing) {
  UpperArmOutwardState state;
  state.valid = true;
  state.distance_m = 0.001;
  state.jacobian[0] = 1.0;

  const auto constraint = makeVelocityOutwardConstraint(
      state, Vec7::Constant(-20.0), Vec7::Constant(20.0), testConfig(),
      0.005);

  EXPECT_TRUE(constraint.active);
  EXPECT_GE(constraint.requested_lower, -0.2);
  EXPECT_DOUBLE_EQ(constraint.lower, constraint.requested_lower);
}

TEST(UpperArmOutward, AccelerationBarrierPreventsOneStepPlaneCrossing) {
  UpperArmOutwardState state;
  state.valid = true;
  state.distance_m = 0.001;
  state.jacobian[0] = 1.0;
  Vec7 qdot = Vec7::Zero();
  qdot[0] = -0.4;

  const auto constraint = makeAccelerationOutwardConstraint(
      state, qdot, 0.0, Vec7::Constant(-1000.0), Vec7::Constant(1000.0),
      testConfig(), 0.005);

  EXPECT_TRUE(constraint.active);
  EXPECT_NEAR(constraint.requested_lower, 80.0, 1e-12);
  EXPECT_NEAR(constraint.lower, 80.0, 1e-12);
}

TEST(UpperArmOutward,
     UsesInteriorRecoveryWhenSmoothAccelerationBarrierIsClipped) {
  UpperArmOutwardState state;
  state.valid = true;
  state.distance_m = 0.20;
  state.jacobian[0] = 1.0;
  Vec7 qdot = Vec7::Zero();
  qdot[0] = -2.0;

  const auto constraint = makeAccelerationOutwardConstraint(
      state, qdot, 0.0, Vec7::Constant(-5.0), Vec7::Constant(5.0),
      testConfig(), 0.005);

  EXPECT_TRUE(constraint.active);
  EXPECT_TRUE(constraint.feasibility_clipped);
  EXPECT_FALSE(constraint.viability_clipped);
  EXPECT_DOUBLE_EQ(constraint.requested_lower, 20.0);
  EXPECT_NEAR(constraint.lower, 4.5, 1e-12);
}

TEST(UpperArmOutward,
     ReportsUnrecoverableWhenOneStepViabilityExceedsAccelerationBox) {
  UpperArmOutwardState state;
  state.valid = true;
  state.distance_m = 0.001;
  state.jacobian[0] = 1.0;
  Vec7 qdot = Vec7::Zero();
  qdot[0] = -1.0;

  const auto constraint = makeAccelerationOutwardConstraint(
      state, qdot, 0.0, Vec7::Constant(-5.0), Vec7::Constant(5.0),
      testConfig(), 0.005);

  // The one-step plane invariant is no longer reachable inside the dynamic
  // joint box.  Keep commanding the strongest feasible outward recovery
  // instead of dropping the constraint and forcing a discontinuous hold.
  EXPECT_TRUE(constraint.active);
  EXPECT_TRUE(constraint.feasibility_clipped);
  EXPECT_TRUE(constraint.viability_clipped);
  EXPECT_GT(constraint.requested_lower, constraint.lower);
}

TEST(UpperArmOutward,
     UsesInteriorRecoveryWhenSmoothVelocityBarrierIsClipped) {
  UpperArmOutwardState state;
  state.valid = true;
  state.distance_m = -0.02;
  state.jacobian[0] = 1.0;

  UpperArmOutwardConfig config = testConfig();
  config.velocity_gain = 100.0;
  const auto constraint = makeVelocityOutwardConstraint(
      state, Vec7::Constant(-1.0), Vec7::Constant(1.0), config, 0.1);

  EXPECT_TRUE(constraint.active);
  EXPECT_TRUE(constraint.feasibility_clipped);
  EXPECT_FALSE(constraint.viability_clipped);
  EXPECT_DOUBLE_EQ(constraint.requested_lower, 2.0);
  EXPECT_NEAR(constraint.lower, 0.9, 1e-12);
}

}  // namespace
}  // namespace tianji_qp_ik
