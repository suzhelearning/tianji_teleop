#include <gtest/gtest.h>

#include "pico_odin/pose_stationarity.hpp"

namespace po = pico_odin;

namespace {

po::Pose3 pose(double x, double yaw = 0.0) {
  return po::Pose3{
    Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d(x, 0.0, 0.0)};
}

}  // namespace

TEST(PoseStationarity, RequiresTwoMonotonicNearbySamples) {
  po::PoseStationarity detector;

  EXPECT_FALSE(detector.observe(1.0, pose(0.0)).valid_interval);
  EXPECT_FALSE(detector.observe(1.0, pose(0.0)).valid_interval);
  EXPECT_FALSE(detector.observe(2.0, pose(0.0)).valid_interval);
  EXPECT_TRUE(detector.observe(2.01, pose(0.0)).valid_interval);
}

TEST(PoseStationarity, ChecksLinearAndAngularRates) {
  po::PoseStationarity detector;
  detector.observe(1.0, pose(0.0));
  const auto still = detector.observe(1.1, pose(0.003, 0.005));
  EXPECT_TRUE(still.valid_interval);
  EXPECT_TRUE(still.stationary);
  EXPECT_NEAR(still.rate_hz, 10.0, 1e-9);

  const auto moving = detector.observe(1.2, pose(0.03, 0.03));
  EXPECT_TRUE(moving.valid_interval);
  EXPECT_FALSE(moving.stationary);
}

TEST(PoseStationarity, ResetRequiresASecondFreshSample) {
  po::PoseStationarity detector;
  detector.observe(1.0, pose(0.0));
  ASSERT_TRUE(detector.observe(1.1, pose(0.0)).valid_interval);

  detector.reset();

  EXPECT_FALSE(detector.observe(2.0, pose(0.0)).valid_interval);
  EXPECT_TRUE(detector.observe(2.1, pose(0.0)).valid_interval);
}
