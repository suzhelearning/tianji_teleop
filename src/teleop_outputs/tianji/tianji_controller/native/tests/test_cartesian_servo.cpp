#include "tianji_qp_ik/cartesian_servo.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace tianji_qp_ik {
namespace {

TEST(CartesianServo, AddsWorldFrameTargetVelocityBeforeClamping) {
  CartesianServoConfig config;
  config.kp_position = Eigen::Vector3d::Constant(2.0);
  config.kp_orientation = Eigen::Vector3d::Constant(3.0);
  config.kff_linear = 0.5;
  config.kff_angular = 0.25;
  config.max_linear_velocity = 100.0;
  config.max_angular_velocity = 100.0;

  Pose current;
  Pose desired;
  desired.position = {1.0, 0.0, 0.0};
  desired.rotation =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Vec6 target_twist = Vec6::Zero();
  target_twist.head<3>() = Eigen::Vector3d(0.4, 0.2, 0.0);
  target_twist.tail<3>() = Eigen::Vector3d(0.0, 0.0, 0.8);

  const Vec6 result =
      cartesianServoTwist(config, desired, current, target_twist);

  Vec6 expected;
  expected << 2.2, 0.1, 0.0, 0.0, 0.0, 0.8;
  EXPECT_TRUE(result.isApprox(expected, 1e-12));
}

TEST(CartesianServo, ClampsCombinedFeedbackAndFeedforwardTwist) {
  CartesianServoConfig config;
  config.kp_position = Eigen::Vector3d::Ones();
  config.kp_orientation = Eigen::Vector3d::Ones();
  config.kff_linear = 1.0;
  config.kff_angular = 1.0;
  config.max_linear_velocity = 2.0;
  config.max_angular_velocity = 3.0;

  Pose current;
  Pose desired;
  desired.position.x() = 1.0;
  Vec6 target_twist = Vec6::Zero();
  target_twist[0] = 3.0;
  target_twist[5] = 4.0;

  const Vec6 result =
      cartesianServoTwist(config, desired, current, target_twist);

  EXPECT_TRUE(result.head<3>().isApprox(Eigen::Vector3d(2.0, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(result.tail<3>().isApprox(Eigen::Vector3d(0.0, 0.0, 3.0), 1e-12));
}

TEST(CartesianServo, ReferencePathAddsReferenceTwistExactlyOnce) {
  CartesianServoConfig config;
  config.kp_position = Eigen::Vector3d::Constant(2.0);
  config.kp_orientation = Eigen::Vector3d::Constant(3.0);
  config.kff_linear = 100.0;
  config.kff_angular = 100.0;
  config.max_linear_velocity = 100.0;
  config.max_angular_velocity = 100.0;
  Pose measured;
  CartesianReference reference;
  reference.pose.position = {0.5, -0.2, 0.1};
  reference.pose.rotation =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  reference.twist << 0.3, 0.1, -0.2, 0.0, 0.0, 0.4;

  const Vec6 result =
      cartesianReferenceServoTwist(config, reference, measured);

  Vec6 expected;
  expected << 1.3, -0.3, 0.0, 0.0, 0.0, 1.0;
  EXPECT_TRUE(result.isApprox(expected, 1e-12));
}

TEST(CartesianServo, AdaptiveGainUsesNearAndFarLimitsSmoothly) {
  CartesianServoConfig config;
  config.adaptive_gain_enabled = true;
  config.kp_position = Eigen::Vector3d::Constant(14.0);
  config.kp_orientation = Eigen::Vector3d::Constant(10.0);
  config.kp_position_near = Eigen::Vector3d::Constant(4.0);
  config.kp_orientation_near = Eigen::Vector3d::Constant(2.0);
  config.position_gain_transition_start_m = 0.02;
  config.position_gain_transition_end_m = 0.22;
  config.orientation_gain_transition_start_rad = 0.10;
  config.orientation_gain_transition_end_rad = 0.50;
  config.max_linear_velocity = 100.0;
  config.max_angular_velocity = 100.0;
  CartesianReference reference;

  reference.pose.position.x() = 0.01;
  Vec6 near = cartesianReferenceServoTwist(config, reference, Pose{});
  EXPECT_NEAR(near.x(), 0.04, 1e-12);

  reference.pose.position.x() = 0.12;
  Vec6 middle = cartesianReferenceServoTwist(config, reference, Pose{});
  EXPECT_NEAR(middle.x(), 0.12 * 9.0, 1e-12);

  reference.pose.position.x() = 0.30;
  Vec6 far = cartesianReferenceServoTwist(config, reference, Pose{});
  EXPECT_NEAR(far.x(), 0.30 * 14.0, 1e-12);

  reference.pose = Pose{};
  reference.pose.rotation =
      Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  near = cartesianReferenceServoTwist(config, reference, Pose{});
  EXPECT_NEAR(near[5], 0.05 * 2.0, 1e-12);

  reference.pose.rotation =
      Eigen::AngleAxisd(0.30, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  middle = cartesianReferenceServoTwist(config, reference, Pose{});
  EXPECT_NEAR(middle[5], 0.30 * 6.0, 1e-12);
}

}  // namespace
}  // namespace tianji_qp_ik
