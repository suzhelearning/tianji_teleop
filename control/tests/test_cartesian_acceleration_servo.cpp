#include "tianji_qp_ik/cartesian_acceleration_servo.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace tianji_qp_ik {
namespace {

TEST(CartesianAccelerationServo, ComposesFeedforwardAndWorldFramePd) {
  CartesianAccelerationServoConfig config;
  config.kp_position = 10.0;
  config.kd_position = 4.0;
  config.kp_orientation = 8.0;
  config.kd_orientation = 3.0;
  config.linear_limit = 100.0;
  config.angular_limit = 100.0;
  CartesianReference reference;
  reference.pose.position = {0.2, -0.1, 0.05};
  reference.pose.rotation =
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  reference.twist << 0.5, 0.1, 0.0, 0.0, 0.0, 0.4;
  reference.acceleration << 0.3, -0.2, 0.1, 0.0, 0.0, 0.6;
  Pose measured_pose;
  Vec6 measured_twist;
  measured_twist << 0.2, -0.1, 0.0, 0.0, 0.0, 0.1;

  const Vec6 command = cartesianAccelerationCommand(
      config, reference, measured_pose, measured_twist);

  Vec6 expected;
  expected << 3.5, -0.4, 0.6, 0.0, 0.0, 3.5;
  EXPECT_TRUE(command.isApprox(expected, 1e-12));
}

TEST(CartesianAccelerationServo, ZeroErrorPassesReferenceAcceleration) {
  CartesianAccelerationServoConfig config;
  CartesianReference reference;
  reference.pose.position = {0.2, 0.1, 0.5};
  reference.acceleration << 1.0, 2.0, -1.0, 0.2, -0.3, 0.4;

  const Vec6 command = cartesianAccelerationCommand(
      config, reference, reference.pose, reference.twist);

  EXPECT_TRUE(command.isApprox(reference.acceleration, 1e-12));
}

TEST(CartesianAccelerationServo, ClampsAfterCompositionBySubspaceNorm) {
  CartesianAccelerationServoConfig config;
  config.kp_position = 100.0;
  config.kd_position = 20.0;
  config.kp_orientation = 70.0;
  config.kd_orientation = 16.0;
  config.linear_limit = 6.0;
  config.angular_limit = 20.0;
  CartesianReference reference;
  reference.pose.position = {1.0, 1.0, 1.0};
  reference.pose.rotation =
      Eigen::AngleAxisd(1.0, Eigen::Vector3d(1.0, 1.0, 0.0).normalized())
          .toRotationMatrix();

  const Vec6 command = cartesianAccelerationCommand(
      config, reference, Pose{}, Vec6::Zero());

  EXPECT_NEAR(command.head<3>().norm(), 6.0, 1e-12);
  EXPECT_NEAR(command.tail<3>().norm(), 20.0, 1e-12);
}

}  // namespace
}  // namespace tianji_qp_ik
