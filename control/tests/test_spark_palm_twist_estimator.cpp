#include "tianji_qp_ik/spark_palm_twist_estimator.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace tianji_qp_ik {
namespace {

SparkPalmTwistEstimatorConfig testConfig() {
  SparkPalmTwistEstimatorConfig config;
  config.filter_alpha = 1.0;
  config.dt_median_window = 5;
  config.dt_min_ratio = 0.5;
  config.dt_max_ratio = 1.5;
  config.linear_stationary_threshold_m_s = 1.0e-6;
  config.angular_stationary_threshold_rad_s = 1.0e-6;
  config.stationary_decay = 0.5;
  config.reversal_decay = 0.25;
  config.maximum_linear_velocity_m_s = 4.0;
  config.maximum_angular_velocity_rad_s = 8.0;
  config.lowpass_cutoff_hz = 1.0;
  return config;
}

Pose translated(double x) {
  Pose pose;
  pose.position.x() = x;
  return pose;
}

TEST(SparkPalmTwistEstimator, FirstFrameStartsAtZero) {
  SparkPalmTwistEstimator estimator(testConfig());

  const auto result = estimator.update(translated(0.2), 1U, 1'000'000'000LL,
                                       7U, false);

  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.reset);
  EXPECT_TRUE(result.twist.isZero(1.0e-12));
}

TEST(SparkPalmTwistEstimator, EstimatesConstantWorldLinearAndAngularTwist) {
  SparkPalmTwistEstimator estimator(testConfig());
  Pose first;
  ASSERT_TRUE(
      estimator.update(first, 1U, 1'000'000'000LL, 7U, false).accepted);

  Pose second = translated(0.01);
  second.rotation =
      Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto result = estimator.update(second, 2U, 1'010'000'000LL, 7U,
                                       false);

  ASSERT_TRUE(result.accepted);
  ASSERT_TRUE(result.dt_valid);
  EXPECT_NEAR(result.twist.x(), 1.0, 1.0e-9);
  EXPECT_NEAR(result.twist[5], 2.0, 1.0e-9);
}

TEST(SparkPalmTwistEstimator, DuplicateAndNonMonotonicFramesHoldTwist) {
  SparkPalmTwistEstimator estimator(testConfig());
  ASSERT_TRUE(estimator.update(translated(0.0), 1U, 1'000'000'000LL, 7U,
                               false)
                  .accepted);
  const auto moving = estimator.update(translated(0.01), 2U,
                                       1'010'000'000LL, 7U, false);
  ASSERT_TRUE(moving.dt_valid);

  const auto duplicate = estimator.update(translated(0.02), 2U,
                                          1'020'000'000LL, 7U, false);
  const auto backward = estimator.update(translated(0.02), 3U,
                                         1'005'000'000LL, 7U, false);

  EXPECT_FALSE(duplicate.accepted);
  EXPECT_FALSE(backward.accepted);
  EXPECT_TRUE(duplicate.twist.isApprox(moving.twist, 1.0e-12));
  EXPECT_TRUE(backward.twist.isApprox(moving.twist, 1.0e-12));
}

TEST(SparkPalmTwistEstimator, RejectsDropoutDtThenRecoversOnNextNormalFrame) {
  SparkPalmTwistEstimator estimator(testConfig());
  std::int64_t timestamp = 1'000'000'000LL;
  for (std::uint64_t sequence = 1U; sequence <= 5U; ++sequence) {
    const double x = 0.01 * static_cast<double>(sequence - 1U);
    const auto result = estimator.update(translated(x), sequence, timestamp,
                                         7U, false);
    ASSERT_TRUE(result.accepted);
    timestamp += 10'000'000LL;
  }

  const auto dropout = estimator.update(translated(0.07), 6U,
                                        1'070'000'000LL, 7U, false);
  EXPECT_TRUE(dropout.accepted);
  EXPECT_FALSE(dropout.dt_valid);
  EXPECT_NEAR(dropout.twist.x(), 1.0, 1.0e-9);

  const auto recovered = estimator.update(translated(0.08), 7U,
                                          1'080'000'000LL, 7U, false);
  EXPECT_TRUE(recovered.accepted);
  EXPECT_TRUE(recovered.dt_valid);
  EXPECT_NEAR(recovered.twist.x(), 1.0, 1.0e-9);
}

TEST(SparkPalmTwistEstimator, EpochAndDiscontinuityClearOldVelocity) {
  SparkPalmTwistEstimator estimator(testConfig());
  ASSERT_TRUE(estimator.update(translated(0.0), 1U, 1'000'000'000LL, 7U,
                               false)
                  .accepted);
  ASSERT_GT(estimator.update(translated(0.01), 2U, 1'010'000'000LL, 7U,
                             false)
                .twist.x(),
            0.0);

  const auto epoch = estimator.update(translated(0.5), 1U, 2'000'000'000LL,
                                      8U, false);
  EXPECT_TRUE(epoch.reset);
  EXPECT_TRUE(epoch.twist.isZero(1.0e-12));

  const auto discontinuity = estimator.update(
      translated(1.0), 2U, 2'010'000'000LL, 8U, true);
  EXPECT_TRUE(discontinuity.reset);
  EXPECT_TRUE(discontinuity.twist.isZero(1.0e-12));
}

TEST(SparkPalmTwistEstimator, ReversalIsAttenuatedAndNormsAreBounded) {
  SparkPalmTwistEstimatorConfig config = testConfig();
  config.maximum_linear_velocity_m_s = 0.5;
  config.maximum_angular_velocity_rad_s = 1.0;
  SparkPalmTwistEstimator estimator(config);
  ASSERT_TRUE(estimator.update(translated(0.0), 1U, 1'000'000'000LL, 7U,
                               false)
                  .accepted);
  const auto positive = estimator.update(translated(0.02), 2U,
                                         1'010'000'000LL, 7U, false);
  EXPECT_NEAR(positive.twist.head<3>().norm(), 0.5, 1.0e-12);

  const auto reversed = estimator.update(translated(0.01), 3U,
                                         1'020'000'000LL, 7U, false);
  EXPECT_LT(reversed.twist.x(), 0.0);
  EXPECT_LE(reversed.twist.head<3>().norm(), 0.5 + 1.0e-12);

  Pose rotated = translated(0.01);
  rotated.rotation =
      Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto bounded = estimator.update(rotated, 4U, 1'030'000'000LL, 7U,
                                        false);
  EXPECT_LE(bounded.twist.tail<3>().norm(), 1.0 + 1.0e-12);
}

TEST(SparkPalmTwistEstimator,
     SeparatesSteadyVelocityFromHighFrequencyFeedforward) {
  SparkPalmTwistEstimator estimator(testConfig());
  std::int64_t timestamp = 1'000'000'000LL;
  ASSERT_TRUE(estimator.update(translated(0.0), 1U, timestamp, 7U, false)
                  .accepted);
  SparkPalmTwistDecision steady;
  for (std::uint64_t sequence = 2U; sequence <= 302U; ++sequence) {
    timestamp += 10'000'000LL;
    steady = estimator.update(
        translated(0.01 * static_cast<double>(sequence - 1U)), sequence,
        timestamp, 7U, false);
  }
  EXPECT_NEAR(steady.twist.x(), 1.0, 1.0e-9);
  EXPECT_NEAR(steady.low_frequency_twist.x(), 1.0, 1.0e-6);
  EXPECT_NEAR(steady.high_frequency_twist.x(), 0.0, 1.0e-6);

  timestamp += 10'000'000LL;
  const auto reversal = estimator.update(translated(3.00), 303U, timestamp,
                                         7U, false);
  EXPECT_LT(reversal.high_frequency_twist.x(), 0.0);
  EXPECT_GT(std::abs(reversal.high_frequency_twist.x()), 1.0e-3);
  EXPECT_GT(reversal.low_frequency_twist.x(), 0.90);
}

}  // namespace
}  // namespace tianji_qp_ik
