#include <gtest/gtest.h>

#include "pico_odin/pose_stream_continuity.hpp"

namespace po = pico_odin;

namespace {

po::Pose3 pose(double x, double yaw = 0.0) {
  return po::Pose3{
    Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d(x, 0.0, 0.0)};
}

}  // namespace

TEST(PoseStreamContinuity, DetectsPoseOriginResetWithMonotonicTimestamps) {
  po::PoseStreamContinuity monitor;
  EXPECT_EQ(monitor.observe(10.0, 20.0, "odin", pose(1.0)), po::Discontinuity::kNone);

  EXPECT_EQ(
    monitor.observe(10.01, 20.01, "odin", pose(0.0)),
    po::Discontinuity::kPoseJump);
}

TEST(PoseStreamContinuity, DetectsFrameChangeAndReceiptGap) {
  po::PoseStreamContinuity monitor;
  monitor.observe(10.0, 20.0, "odin", pose(0.0));
  EXPECT_EQ(
    monitor.observe(10.01, 20.01, "new_odin", pose(0.0)),
    po::Discontinuity::kFrameChanged);

  monitor.reset();
  monitor.observe(10.0, 20.0, "odin", pose(0.0));
  EXPECT_EQ(
    monitor.observe(12.0, 22.0, "odin", pose(0.0)),
    po::Discontinuity::kReceiptGap);
}

TEST(PoseStreamContinuity, AcceptsPlausibleContinuousMotion) {
  po::PoseStreamContinuity monitor;
  monitor.observe(10.0, 20.0, "odin", pose(0.0));

  EXPECT_EQ(
    monitor.observe(10.1, 20.1, "odin", pose(0.08, 0.15)),
    po::Discontinuity::kNone);
}
