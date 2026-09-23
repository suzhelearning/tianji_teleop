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

}  // namespace
}  // namespace tianji_qp_ik
