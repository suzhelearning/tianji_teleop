#include <gtest/gtest.h>

#include "pico_bridge/auto_calibration_state.hpp"

TEST(AutoCalibrationState, RequiresSettlingAfterBothAcks)
{
  pico_bridge::AutoCalibrationState state;
  const auto generation = state.begin();
  EXPECT_FALSE(state.accept_zero_ack(generation, true, true));
  EXPECT_TRUE(state.accept_zero_ack(generation, false, true));
  EXPECT_TRUE(state.settling());
  EXPECT_FALSE(state.awaiting_fresh_imus());
  EXPECT_TRUE(state.finish_settling(generation));
  EXPECT_TRUE(state.awaiting_fresh_imus());
}

TEST(AutoCalibrationState, StaleTimerCannotFinishRestartedSettling)
{
  pico_bridge::AutoCalibrationState state;
  const auto old_generation = state.begin();
  EXPECT_FALSE(state.accept_zero_ack(old_generation, true, true));
  EXPECT_TRUE(state.accept_zero_ack(old_generation, false, true));
  const auto new_generation = state.begin();
  EXPECT_FALSE(state.finish_settling(old_generation));
  EXPECT_EQ(state.generation(), new_generation);
  EXPECT_FALSE(state.awaiting_fresh_imus());
}

TEST(AutoCalibrationState, IgnoresStaleGenerationAndCancelsOnFailure)
{
  pico_bridge::AutoCalibrationState state;
  const auto old_generation = state.begin();
  const auto new_generation = state.begin();
  EXPECT_FALSE(state.accept_zero_ack(old_generation, true, true));
  state.fail(new_generation);
  EXPECT_FALSE(state.awaiting_fresh_imus());
}

TEST(AutoCalibrationState, FailedAckDoesNotStartSampling)
{
  pico_bridge::AutoCalibrationState state;
  const auto generation = state.begin();
  EXPECT_FALSE(state.accept_zero_ack(generation, true, false));
  EXPECT_FALSE(state.awaiting_fresh_imus());
}
