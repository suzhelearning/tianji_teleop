#include "tianji_qp_ik/pico_trace_recorder.hpp"

#include <gtest/gtest.h>

#include <unistd.h>

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace tianji_qp_ik {
namespace {

class TemporaryTracePath {
 public:
  explicit TemporaryTracePath(const std::string& label)
      : path_(std::filesystem::temp_directory_path() /
              ("tianji_" + label + "_" + std::to_string(getpid()) + "_" +
               std::to_string(std::chrono::steady_clock::now()
                                  .time_since_epoch()
                                  .count()) +
               ".tjvr")) {
    std::filesystem::remove(path_);
  }

  ~TemporaryTracePath() { std::filesystem::remove(path_); }

  const std::filesystem::path& path() const noexcept { return path_; }

 private:
  std::filesystem::path path_;
};

std::uint16_t readLe16(const std::vector<std::uint8_t>& bytes,
                       std::size_t offset) {
  return static_cast<std::uint16_t>(bytes.at(offset)) |
         (static_cast<std::uint16_t>(bytes.at(offset + 1U)) << 8U);
}

std::uint64_t readLe64(const std::vector<std::uint8_t>& bytes,
                       std::size_t offset) {
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < 8U; ++index) {
    value |= static_cast<std::uint64_t>(bytes.at(offset + index)) <<
             (8U * index);
  }
  return value;
}

std::vector<std::uint8_t> readFile(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open trace test output");
  }
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

std::vector<std::uint8_t> patternedPacket(std::size_t size,
                                          std::uint8_t seed) {
  std::vector<std::uint8_t> packet(size);
  for (std::size_t index = 0U; index < size; ++index) {
    packet[index] = static_cast<std::uint8_t>(seed + index);
  }
  return packet;
}

TEST(PicoTraceRecorder, WritesCompatibleHeaderTimingAndOriginalPackets) {
  TemporaryTracePath output("trace_format");
  const auto first = patternedPacket(656U, 3U);
  const auto second = patternedPacket(656U, 17U);

  PicoTraceRecorder recorder(output.path().string());
  ASSERT_TRUE(recorder.append(first.data(), first.size(), 1'000'000'000LL));
  ASSERT_TRUE(recorder.append(second.data(), second.size(), 1'011'000'000LL));
  recorder.finish();

  const PicoTraceRecorderStats stats = recorder.stats();
  EXPECT_EQ(stats.state, PicoTraceRecorderState::kFinalized);
  EXPECT_EQ(stats.records, 2U);
  EXPECT_EQ(stats.packet_size, 656U);

  const std::vector<std::uint8_t> bytes = readFile(output.path());
  ASSERT_EQ(bytes.size(), 16U + 2U * (8U + 656U));
  EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 4), "TJVT");
  EXPECT_EQ(readLe16(bytes, 4U), 1U);
  EXPECT_EQ(readLe16(bytes, 6U), 656U);
  EXPECT_EQ(readLe64(bytes, 8U), 2U);
  EXPECT_EQ(readLe64(bytes, 16U), 0U);
  EXPECT_TRUE(std::equal(first.begin(), first.end(), bytes.begin() + 24));
  EXPECT_EQ(readLe64(bytes, 24U + 656U), 11'000'000U);
  EXPECT_TRUE(std::equal(second.begin(), second.end(),
                         bytes.begin() + 32 + 656U));
}

TEST(PicoTraceRecorder, RefusesToOverwriteExistingDestination) {
  TemporaryTracePath output("trace_exists");
  {
    std::ofstream existing(output.path());
    ASSERT_TRUE(existing.good());
    existing << "keep";
  }

  EXPECT_THROW(PicoTraceRecorder(output.path().string()), std::runtime_error);
  const std::vector<std::uint8_t> bytes = readFile(output.path());
  EXPECT_EQ(std::string(bytes.begin(), bytes.end()), "keep");
}

TEST(PicoTraceRecorder, MixedPacketSizeFailsWithoutThrowingAndFinishIsSafe) {
  TemporaryTracePath output("trace_mixed");
  const auto first = patternedPacket(656U, 1U);
  const auto mixed = patternedPacket(400U, 2U);

  PicoTraceRecorder recorder(output.path().string());
  ASSERT_TRUE(recorder.append(first.data(), first.size(), 10LL));
  EXPECT_FALSE(recorder.append(mixed.data(), mixed.size(), 20LL));
  EXPECT_EQ(recorder.stats().state, PicoTraceRecorderState::kFailed);
  EXPECT_EQ(recorder.stats().records, 1U);
  recorder.finish();
  recorder.finish();
  EXPECT_EQ(recorder.stats().state, PicoTraceRecorderState::kFailed);
}

TEST(PicoTraceRecorder, NoPacketFinishRemovesReservedDestination) {
  TemporaryTracePath output("trace_empty");
  PicoTraceRecorder recorder(output.path().string());
  ASSERT_TRUE(std::filesystem::exists(output.path()));

  recorder.finish();

  EXPECT_EQ(recorder.stats().state, PicoTraceRecorderState::kNoValidPackets);
  EXPECT_FALSE(std::filesystem::exists(output.path()));
}

}  // namespace
}  // namespace tianji_qp_ik
