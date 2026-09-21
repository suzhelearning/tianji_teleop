#include "tianji_qp_ik/telemetry.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

namespace tianji_qp_ik {
namespace {

TelemetrySample makeSample(std::uint64_t sequence) {
  TelemetrySample sample;
  sample.sequence = sequence;
  sample.control_time_seconds = static_cast<double>(sequence) * 0.001;
  sample.left_position_error = static_cast<double>(sequence);
  sample.right_position_error = -static_cast<double>(sequence);
  sample.cycle_time_us = static_cast<double>(sequence % 1000U);
  return sample;
}

void expectConsistent(const TelemetrySample& sample) {
  const double sequence = static_cast<double>(sample.sequence);
  EXPECT_DOUBLE_EQ(sample.control_time_seconds, sequence * 0.001);
  EXPECT_DOUBLE_EQ(sample.left_position_error, sequence);
  EXPECT_DOUBLE_EQ(sample.right_position_error, -sequence);
  EXPECT_DOUBLE_EQ(sample.cycle_time_us, static_cast<double>(sample.sequence % 1000U));
}

TEST(SnapshotExchangeTest, QueueIsBoundedAndNeverOverwritesUnreadSamples) {
  TelemetryBuffer queue(3U);
  EXPECT_EQ(queue.capacity(), 3U);
  EXPECT_TRUE(queue.tryPush(makeSample(1U)));
  EXPECT_TRUE(queue.tryPush(makeSample(2U)));
  EXPECT_TRUE(queue.tryPush(makeSample(3U)));
  EXPECT_FALSE(queue.tryPush(makeSample(4U)));

  TelemetrySample sample;
  ASSERT_TRUE(queue.tryPop(sample));
  EXPECT_EQ(sample.sequence, 1U);
  EXPECT_TRUE(queue.tryPush(makeSample(4U)));
  for (std::uint64_t expected = 2U; expected <= 4U; ++expected) {
    ASSERT_TRUE(queue.tryPop(sample));
    EXPECT_EQ(sample.sequence, expected);
    expectConsistent(sample);
  }
  EXPECT_FALSE(queue.tryPop(sample));
}

TEST(SnapshotExchangeTest, ConcurrentSnapshotsAreNotTornAndSlowConsumerCannotBlockProducer) {
  LatestSnapshotExchange<TelemetrySample> exchange(8U);
  constexpr std::uint64_t kAttempts = 200000U;
  std::atomic<bool> producer_done{false};
  std::atomic<std::uint64_t> drops{0U};

  const auto producer_start = std::chrono::steady_clock::now();
  std::thread producer([&] {
    for (std::uint64_t sequence = 1U; sequence <= kAttempts; ++sequence) {
      if (!exchange.tryPublish(makeSample(sequence))) {
        drops.fetch_add(1U, std::memory_order_relaxed);
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  std::uint64_t last_sequence = 0U;
  TelemetrySample latest;
  while (!producer_done.load(std::memory_order_acquire)) {
    if (exchange.tryReadLatest(latest)) {
      EXPECT_GT(latest.sequence, last_sequence);
      expectConsistent(latest);
      last_sequence = latest.sequence;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  producer.join();
  const auto producer_elapsed = std::chrono::steady_clock::now() - producer_start;
  while (exchange.tryReadLatest(latest)) {
    EXPECT_GT(latest.sequence, last_sequence);
    expectConsistent(latest);
    last_sequence = latest.sequence;
  }

  EXPECT_GT(drops.load(std::memory_order_relaxed), 0U);
  EXPECT_LT(producer_elapsed, std::chrono::seconds(2));
}

TEST(SnapshotExchangeTest, LatestOnlyExchangeReportsUnreadReplacement) {
  LatestSpscExchange<TelemetrySample> exchange;
  EXPECT_EQ(exchange.publish(makeSample(1U)), LatestPublishResult::kPublished);
  EXPECT_EQ(exchange.publish(makeSample(2U)), LatestPublishResult::kSuperseded);
  EXPECT_EQ(exchange.publish(makeSample(3U)), LatestPublishResult::kSuperseded);

  TelemetrySample latest;
  ASSERT_TRUE(exchange.tryReadLatest(latest));
  EXPECT_EQ(latest.sequence, 3U);
  expectConsistent(latest);
  EXPECT_FALSE(exchange.tryReadLatest(latest));

  EXPECT_EQ(exchange.publish(makeSample(4U)), LatestPublishResult::kPublished);
  ASSERT_TRUE(exchange.tryReadLatest(latest));
  EXPECT_EQ(latest.sequence, 4U);
  expectConsistent(latest);
}

TEST(SnapshotExchangeTest, CycleWindowReportsRollingP99) {
  CycleTimeWindow window(100U);
  for (int sample = 1; sample <= 100; ++sample) {
    window.add(static_cast<double>(sample));
  }
  EXPECT_EQ(window.size(), 100U);
  EXPECT_NEAR(window.percentile99(), 99.01, 1e-12);
  window.add(0.0);
  EXPECT_EQ(window.size(), 100U);
  EXPECT_NEAR(window.percentile99(), 99.01, 1e-12);
}

TEST(SnapshotExchangeTest, DrainsToNewestAlgorithmAndIkDiagnostics) {
  LatestSnapshotExchange<ViewerSnapshot> exchange(4U);
  ViewerSnapshot first;
  first.sequence = 10U;
  first.algorithm = IkAlgorithm::kNullspaceDls;
  first.left_ik.slack_position_norm = 0.4;
  ViewerSnapshot second;
  second.sequence = 11U;
  second.algorithm = IkAlgorithm::kHierarchicalQp;
  second.control_level = ControlLevel::kAcceleration;
  second.arm_angle_reference_mode = ArmAngleReferenceMode::kDefaultDown;
  second.at_nominal_configuration = true;
  second.left_ik.slack_position_norm = 0.02;
  second.right_ik.slack_orientation_norm = 0.03;
  second.left_ik.qddot_max_ratio = 0.7;
  second.left_ik.qdot_reference_error_max_abs = 0.04;
  second.left_ik.active_qddot_bounds = 2;
  second.left_ik.arm_angle_control_error_rad = 0.12;
  second.left_ik.dls_posture_reference_active = true;
  second.left_ik.dls_posture_status = 1;
  second.left_ik.dls_posture_goal_limit_margin_rad = 0.08;
  second.left_ik.dls_posture_qdot_error_max_abs = 0.03;
  second.joint_plot_samples = 123U;
  second.joint_plot_drops = 4U;
  second.joint_plot_reference_derivatives_valid = true;
  second.joint_plot_actual_derivatives_valid = true;
  ASSERT_TRUE(exchange.tryPublish(first));
  ASSERT_TRUE(exchange.tryPublish(second));

  ViewerSnapshot latest;
  ASSERT_TRUE(exchange.tryReadLatest(latest));
  EXPECT_EQ(latest.sequence, 11U);
  EXPECT_EQ(latest.algorithm, IkAlgorithm::kHierarchicalQp);
  EXPECT_EQ(latest.control_level, ControlLevel::kAcceleration);
  EXPECT_EQ(latest.arm_angle_reference_mode,
            ArmAngleReferenceMode::kDefaultDown);
  EXPECT_TRUE(latest.at_nominal_configuration);
  EXPECT_DOUBLE_EQ(latest.left_ik.slack_position_norm, 0.02);
  EXPECT_DOUBLE_EQ(latest.right_ik.slack_orientation_norm, 0.03);
  EXPECT_DOUBLE_EQ(latest.left_ik.qddot_max_ratio, 0.7);
  EXPECT_DOUBLE_EQ(latest.left_ik.qdot_reference_error_max_abs, 0.04);
  EXPECT_EQ(latest.left_ik.active_qddot_bounds, 2);
  EXPECT_DOUBLE_EQ(latest.left_ik.arm_angle_control_error_rad, 0.12);
  EXPECT_TRUE(latest.left_ik.dls_posture_reference_active);
  EXPECT_EQ(latest.left_ik.dls_posture_status, 1);
  EXPECT_DOUBLE_EQ(latest.left_ik.dls_posture_goal_limit_margin_rad, 0.08);
  EXPECT_DOUBLE_EQ(latest.left_ik.dls_posture_qdot_error_max_abs, 0.03);
  EXPECT_EQ(latest.joint_plot_samples, 123U);
  EXPECT_EQ(latest.joint_plot_drops, 4U);
  EXPECT_TRUE(latest.joint_plot_reference_derivatives_valid);
  EXPECT_TRUE(latest.joint_plot_actual_derivatives_valid);

  ViewerCommand command;
  command.type = ViewerCommandType::kSetControlLevel;
  command.control_level = ControlLevel::kAcceleration;
  EXPECT_EQ(command.control_level, ControlLevel::kAcceleration);

  command.type = ViewerCommandType::kTogglePicoArmAngleSource;
  EXPECT_EQ(command.type, ViewerCommandType::kTogglePicoArmAngleSource);
}

TEST(SnapshotExchangeTest, CarriesCompletePicoDiagnostics) {
  TelemetrySample telemetry;
  telemetry.pico_configured = true;
  telemetry.pico_enabled = true;
  telemetry.pico_live = true;
  telemetry.pico_stale = false;
  telemetry.pico_tracking_epoch = 9U;
  telemetry.pico_sequence = 7U;
  telemetry.pico_datagrams = 10U;
  telemetry.pico_accepted = 8U;
  telemetry.pico_malformed = 1U;
  telemetry.pico_crc_failures = 2U;
  telemetry.pico_reordered = 3U;
  telemetry.pico_jump_rejections = 4U;
  telemetry.pico_superseded = 5U;
  telemetry.pico_epoch_resets = 6U;
  telemetry.pico_resynchronizations = 7U;
  telemetry.pico_reset_applies = 8U;
  telemetry.pico_left_source_timestamp_ns = 123456789;
  telemetry.pico_right_source_timestamp_ns = 123456789;
  telemetry.pico_input_frequency_hz = 72.0;
  telemetry.pico_frame_age_ms = 14.0;
  telemetry.pico_receive_to_control_us = 500.0;
  telemetry.pico_bridge_to_control_us = 900.0;

  ViewerSnapshot snapshot;
  snapshot.pico_configured = telemetry.pico_configured;
  snapshot.pico_enabled = telemetry.pico_enabled;
  snapshot.pico_live = telemetry.pico_live;
  snapshot.pico_stale = telemetry.pico_stale;
  snapshot.pico_tracking_epoch = telemetry.pico_tracking_epoch;
  snapshot.pico_sequence = telemetry.pico_sequence;
  snapshot.pico_datagrams = telemetry.pico_datagrams;
  snapshot.pico_accepted = telemetry.pico_accepted;
  snapshot.pico_malformed = telemetry.pico_malformed;
  snapshot.pico_crc_failures = telemetry.pico_crc_failures;
  snapshot.pico_reordered = telemetry.pico_reordered;
  snapshot.pico_jump_rejections = telemetry.pico_jump_rejections;
  snapshot.pico_superseded = telemetry.pico_superseded;
  snapshot.pico_epoch_resets = telemetry.pico_epoch_resets;
  snapshot.pico_resynchronizations = telemetry.pico_resynchronizations;
  snapshot.pico_reset_applies = telemetry.pico_reset_applies;
  snapshot.pico_left_source_timestamp_ns =
      telemetry.pico_left_source_timestamp_ns;
  snapshot.pico_right_source_timestamp_ns =
      telemetry.pico_right_source_timestamp_ns;
  snapshot.pico_input_frequency_hz = telemetry.pico_input_frequency_hz;
  snapshot.pico_frame_age_ms = telemetry.pico_frame_age_ms;
  snapshot.pico_receive_to_control_us = telemetry.pico_receive_to_control_us;
  snapshot.pico_bridge_to_control_us = telemetry.pico_bridge_to_control_us;

  EXPECT_TRUE(snapshot.pico_configured);
  EXPECT_TRUE(snapshot.pico_enabled);
  EXPECT_TRUE(snapshot.pico_live);
  EXPECT_FALSE(snapshot.pico_stale);
  EXPECT_EQ(snapshot.pico_tracking_epoch, 9U);
  EXPECT_EQ(snapshot.pico_sequence, 7U);
  EXPECT_EQ(snapshot.pico_datagrams, 10U);
  EXPECT_EQ(snapshot.pico_accepted, 8U);
  EXPECT_EQ(snapshot.pico_malformed, 1U);
  EXPECT_EQ(snapshot.pico_crc_failures, 2U);
  EXPECT_EQ(snapshot.pico_reordered, 3U);
  EXPECT_EQ(snapshot.pico_jump_rejections, 4U);
  EXPECT_EQ(snapshot.pico_superseded, 5U);
  EXPECT_EQ(snapshot.pico_epoch_resets, 6U);
  EXPECT_EQ(snapshot.pico_resynchronizations, 7U);
  EXPECT_EQ(snapshot.pico_reset_applies, 8U);
  EXPECT_EQ(snapshot.pico_left_source_timestamp_ns, 123456789);
  EXPECT_EQ(snapshot.pico_right_source_timestamp_ns, 123456789);
  EXPECT_DOUBLE_EQ(snapshot.pico_input_frequency_hz, 72.0);
  EXPECT_DOUBLE_EQ(snapshot.pico_frame_age_ms, 14.0);
  EXPECT_DOUBLE_EQ(snapshot.pico_receive_to_control_us, 500.0);
  EXPECT_DOUBLE_EQ(snapshot.pico_bridge_to_control_us, 900.0);
}

}  // namespace
}  // namespace tianji_qp_ik
