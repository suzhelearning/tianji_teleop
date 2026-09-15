#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "pico_odin/host_time_synchronizer.hpp"

namespace po = pico_odin;

namespace {

po::Pose3 pose(double x, double yaw = 0.0) {
  return po::Pose3{
    Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d(x, 0.0, 1.0)};
}

}  // namespace

TEST(HostTimeSynchronizer, WaitsForOdinSamplesThatBracketPicoReceiptTime) {
  po::HostTimeSynchronizer synchronizer;
  synchronizer.add_odin({0.99, pose(0.0)});

  EXPECT_FALSE(synchronizer.interpolate_for_pico(1.0).has_value());

  synchronizer.add_odin({1.01, pose(1.0)});
  const auto interpolated = synchronizer.interpolate_for_pico(1.0);

  ASSERT_TRUE(interpolated.has_value());
  EXPECT_NEAR(interpolated->translation.x(), 0.5, 1e-12);
}

TEST(HostTimeSynchronizer, EstimatesAndSmoothsPerRunReceiveLag) {
  po::HostTimeSynchronizerOptions options;
  options.lag_smoothing_alpha = 1.0;
  options.solver_options.min_receive_lag_pairs = 10;
  po::HostTimeSynchronizer synchronizer(options);
  constexpr double lag = 0.024;
  constexpr double dt = 0.02;
  for (int index = 0; index < 280; ++index) {
    const double time = index * dt;
    const double yaw = 0.28 * std::sin(2.0 * M_PI * time / 2.8);
    synchronizer.add_pico({time, pose(0.0, yaw)});
    synchronizer.add_odin({time + lag, pose(0.0, yaw)});
  }

  ASSERT_TRUE(synchronizer.update_receive_lag());
  EXPECT_NEAR(synchronizer.receive_lag_sec(), lag, 0.006);
  EXPECT_GT(synchronizer.receive_lag_correlation(), 0.25);
  EXPECT_TRUE(synchronizer.receive_lag_valid());
}

TEST(HostTimeSynchronizer, ResetDropsBuffersAndSessionLag) {
  po::HostTimeSynchronizerOptions options;
  options.lag_smoothing_alpha = 1.0;
  po::HostTimeSynchronizer synchronizer(options);
  synchronizer.add_odin({0.0, pose(0.0)});
  synchronizer.add_odin({0.02, pose(1.0)});

  synchronizer.reset();

  EXPECT_FALSE(synchronizer.interpolate_for_pico(0.01).has_value());
  EXPECT_FALSE(synchronizer.receive_lag_valid());
  EXPECT_DOUBLE_EQ(synchronizer.receive_lag_sec(), 0.0);
}

TEST(HostTimeSynchronizer, RejectsInvalidReceiveLagSearchOptions) {
  po::HostTimeSynchronizerOptions options;
  options.solver_options.receive_lag_step_sec = 0.0;
  EXPECT_THROW(
    { const po::HostTimeSynchronizer synchronizer(options); },
    std::invalid_argument);

  options = po::HostTimeSynchronizerOptions{};
  options.solver_options.max_receive_lag_sec = 0.0;
  EXPECT_THROW(
    { const po::HostTimeSynchronizer synchronizer(options); },
    std::invalid_argument);
}

TEST(HostTimeSynchronizer, SmoothsAcceptedLagChangesAndRejectsStaleTargets) {
  po::HostTimeSynchronizerOptions options;
  options.history_sec = 6.0;
  options.lag_smoothing_alpha = 0.5;
  options.solver_options.min_receive_lag_pairs = 10;
  po::HostTimeSynchronizer synchronizer(options);
  constexpr double dt = 0.02;
  for (int index = 0; index < 280; ++index) {
    const double time = index * dt;
    const double yaw = 0.28 * std::sin(2.0 * M_PI * time / 2.8);
    synchronizer.add_pico({time, pose(0.0, yaw)});
    synchronizer.add_odin({time + 0.02, pose(0.0, yaw)});
  }
  ASSERT_TRUE(synchronizer.update_receive_lag());
  const double first = synchronizer.receive_lag_sec();

  for (int index = 0; index < 280; ++index) {
    const double time = 6.0 + index * dt;
    const double yaw = 0.28 * std::sin(2.0 * M_PI * time / 2.8);
    synchronizer.add_pico({time, pose(0.0, yaw)});
    synchronizer.add_odin({time + 0.04, pose(0.0, yaw)});
  }
  ASSERT_TRUE(synchronizer.update_receive_lag());
  EXPECT_GT(synchronizer.receive_lag_sec(), first);
  EXPECT_LT(synchronizer.receive_lag_sec(), 0.04);
  EXPECT_FALSE(synchronizer.interpolate_for_pico(0.5).has_value());
}
