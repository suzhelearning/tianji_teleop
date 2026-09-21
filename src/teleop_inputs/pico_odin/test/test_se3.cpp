#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "pico_odin/se3.hpp"

namespace po = pico_odin;

TEST(Se3, ComposeWithInverseIsIdentity) {
  const po::Pose3 pelvis_T_odin{
    Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d(-0.25, 0.0, 0.08)};

  const auto identity = po::compose(pelvis_T_odin, po::inverse(pelvis_T_odin));
  EXPECT_LT(identity.translation.norm(), 1e-12);
  EXPECT_LT(po::angular_distance(identity.rotation, Eigen::Quaterniond::Identity()), 1e-12);
}

TEST(Se3, InterpolationUsesLinearTranslationAndSlerp) {
  const po::Pose3 start;
  const po::Pose3 finish{
    Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d(0.0, 0.0, 1.0)};

  const auto midpoint = po::interpolate(start, finish, 0.5);
  EXPECT_NEAR(midpoint.translation.z(), 0.5, 1e-12);
  EXPECT_NEAR(
    po::angular_distance(midpoint.rotation, Eigen::Quaterniond::Identity()),
    M_PI / 4.0, 1e-12);
}

TEST(Se3, PoseMessageRoundTripPreservesTransform) {
  const po::Pose3 expected{
    Eigen::Quaterniond(Eigen::AngleAxisd(0.23, Eigen::Vector3d(1.0, 2.0, 3.0).normalized())),
    Eigen::Vector3d(1.0, -2.0, 0.75)};

  const auto actual = po::from_pose_msg(po::to_pose_msg(expected));
  EXPECT_LT((actual.translation - expected.translation).norm(), 1e-12);
  EXPECT_LT(po::angular_distance(actual.rotation, expected.rotation), 1e-12);
}

TEST(Se3, MeanPoseHemisphereAlignsEquivalentQuaternions) {
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond negated(-q.w(), -q.x(), -q.y(), -q.z());
  const auto mean = po::mean_pose({
    po::Pose3{q, Eigen::Vector3d(1.0, 2.0, 3.0)},
    po::Pose3{negated, Eigen::Vector3d(3.0, 4.0, 5.0)}});

  EXPECT_LT(po::angular_distance(mean.rotation, q), 1e-12);
  EXPECT_LT((mean.translation - Eigen::Vector3d(2.0, 3.0, 4.0)).norm(), 1e-12);
}

TEST(Se3, CheckedPoseRejectsNonFiniteTranslation) {
  geometry_msgs::msg::Pose message;
  message.orientation.w = 1.0;
  message.position.x = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(po::from_pose_msg_checked(message), std::invalid_argument);
}

TEST(Se3, CheckedPoseRejectsNonNormalizedQuaternion) {
  geometry_msgs::msg::Pose message;
  message.orientation.w = 2.0;

  EXPECT_THROW(po::from_pose_msg_checked(message), std::invalid_argument);
}

TEST(Se3, CheckedPoseAcceptsNormalizedFinitePose) {
  const auto expected = po::Pose3{
    Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY())),
    Eigen::Vector3d(0.1, -0.2, 1.0)};

  const auto actual = po::from_pose_msg_checked(po::to_pose_msg(expected));

  EXPECT_LT(po::angular_distance(actual.rotation, expected.rotation), 1e-12);
  EXPECT_LT((actual.translation - expected.translation).norm(), 1e-12);
}

TEST(Se3, SkewSymmetricRepresentsTheCrossProductMatrix) {
  Eigen::Matrix3d expected;
  expected <<
    0.0, -3.0, 2.0,
    3.0, 0.0, -1.0,
    -2.0, 1.0, 0.0;

  EXPECT_TRUE(
    po::skew_symmetric(Eigen::Vector3d(1.0, 2.0, 3.0)).isApprox(expected));
}
