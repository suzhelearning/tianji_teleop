#include <gtest/gtest.h>

#include <chrono>
#include <limits>

#include "pico_bridge/foot_imu_input_state.hpp"

namespace fusion = pico_bridge::fusion;

TEST(FootImuInputState, RejectImmediatelyRevokesAcceptedOrientation) {
  fusion::FootImuInputState state;
  const auto received_at = std::chrono::steady_clock::time_point(std::chrono::seconds(1));
  state.accept({0.0, 0.0, 0.0, 1.0}, received_at);
  ASSERT_TRUE(state.orientation().has_value());
  ASSERT_TRUE(state.is_fresh(received_at + std::chrono::milliseconds(50), 0.08));

  state.reject();

  EXPECT_FALSE(state.orientation().has_value());
  EXPECT_FALSE(state.is_fresh(received_at + std::chrono::milliseconds(51), 0.08));
}

TEST(FootImuInputState, FreshnessExpiresAtConfiguredAge) {
  fusion::FootImuInputState state;
  const auto received_at = std::chrono::steady_clock::time_point(std::chrono::seconds(1));
  state.accept({0.0, 0.0, 0.0, 1.0}, received_at);

  EXPECT_TRUE(state.is_fresh(received_at + std::chrono::milliseconds(79), 0.08));
  EXPECT_FALSE(state.is_fresh(received_at + std::chrono::milliseconds(81), 0.08));
}

TEST(FootImuInputState, PositionValidationRejectsNonFiniteCoordinates) {
  EXPECT_TRUE(fusion::is_finite_position({1.0, 2.0, 3.0}));
  EXPECT_FALSE(fusion::is_finite_position(
    {1.0, std::numeric_limits<double>::quiet_NaN(), 3.0}));
  EXPECT_FALSE(fusion::is_finite_position(
    {1.0, 2.0, std::numeric_limits<double>::infinity()}));
}
