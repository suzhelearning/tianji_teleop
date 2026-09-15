#include "tianji_qp_ik/pico_udp_receiver.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace tianji_qp_ik {
namespace {

constexpr const char* kGoldenPacketHex =
    "544a56520100a000070000000000000009000000000000000b00000000000000"
    "0d000000000000000f000000000000000000f03f000000000000004000000000"
    "0000084000000000000000000000000000000000000000000000000000000000"
    "0000f03f000000000000f0bf00000000000000c000000000000008c000000000"
    "000000000000000000000000000000000000f03f00000000000000003740fb80";

std::vector<std::uint8_t> fromHex(const std::string& text) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(text.size() / 2);
  for (std::size_t index = 0; index < text.size(); index += 2) {
    bytes.push_back(static_cast<std::uint8_t>(
        std::stoul(text.substr(index, 2), nullptr, 16)));
  }
  return bytes;
}

std::uint32_t crc32(const std::uint8_t* bytes, std::size_t size) {
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= bytes[index];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xedb88320U & mask);
    }
  }
  return crc ^ 0xffffffffU;
}

void writeLe32(std::vector<std::uint8_t>* bytes, std::size_t offset,
               std::uint32_t value) {
  for (std::size_t index = 0; index < 4; ++index) {
    (*bytes)[offset + index] =
        static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

void writeLe16(std::vector<std::uint8_t>* bytes, std::size_t offset,
               std::uint16_t value) {
  (*bytes)[offset] = static_cast<std::uint8_t>(value & 0xffU);
  (*bytes)[offset + 1U] =
      static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void writeLe64(std::vector<std::uint8_t>* bytes, std::size_t offset,
               std::uint64_t value) {
  for (std::size_t index = 0; index < 8; ++index) {
    (*bytes)[offset + index] =
        static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

void writeDouble(std::vector<std::uint8_t>* bytes, std::size_t offset,
                 double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe64(bytes, offset, bits);
}

void refreshCrc(std::vector<std::uint8_t>* bytes) {
  const std::size_t crc_offset = bytes->size() - 4U;
  writeLe32(bytes, crc_offset, crc32(bytes->data(), crc_offset));
}

std::vector<std::uint8_t> packet(std::uint64_t sequence, std::uint64_t epoch,
                                 std::int64_t source_timestamp_ns);

std::vector<std::uint8_t> v2Packet(std::uint64_t sequence,
                                   std::uint64_t epoch,
                                   std::int64_t source_timestamp_ns) {
  auto bytes = packet(sequence, epoch, source_timestamp_ns);
  bytes.resize(kPicoTeleopPacketV2Size, 0U);
  writeLe16(&bytes, 4U, 2U);
  writeLe16(&bytes, 6U,
            static_cast<std::uint16_t>(kPicoTeleopPacketV2Size));
  writeLe32(&bytes, 40U, 0x3fU);
  writeDouble(&bytes, 156U, 0.0);
  writeDouble(&bytes, 164U, 0.0);
  writeDouble(&bytes, 172U, -1.0);
  writeDouble(&bytes, 180U, 0.0);
  writeDouble(&bytes, 188U, 0.0);
  writeDouble(&bytes, 196U, -1.0);
  refreshCrc(&bytes);
  return bytes;
}

std::vector<std::uint8_t> packet(std::uint64_t sequence, std::uint64_t epoch,
                                 std::int64_t source_timestamp_ns) {
  auto bytes = fromHex(kGoldenPacketHex);
  writeLe64(&bytes, 8, sequence);
  writeLe64(&bytes, 16, epoch);
  writeLe64(&bytes, 24, static_cast<std::uint64_t>(source_timestamp_ns));
  refreshCrc(&bytes);
  return bytes;
}

void sendDatagram(std::uint16_t port, const std::vector<std::uint8_t>& bytes) {
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    throw std::runtime_error("failed to create test UDP socket");
  }
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(port);
  destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  const ssize_t sent = sendto(socket_fd, bytes.data(), bytes.size(), 0,
                              reinterpret_cast<const sockaddr*>(&destination),
                              sizeof(destination));
  close(socket_fd);
  if (sent != static_cast<ssize_t>(bytes.size())) {
    throw std::runtime_error("failed to send test UDP datagram");
  }
}

bool waitUntil(const std::function<bool()>& predicate,
               std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return predicate();
}

std::uint64_t traceLe64(const std::vector<std::uint8_t>& bytes,
                        std::size_t offset) {
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < 8U; ++index) {
    value |= static_cast<std::uint64_t>(bytes.at(offset + index)) <<
             (8U * index);
  }
  return value;
}

std::filesystem::path receiverTracePath() {
  return std::filesystem::temp_directory_path() /
         ("tianji_receiver_trace_" + std::to_string(getpid()) + "_" +
          std::to_string(
              std::chrono::steady_clock::now().time_since_epoch().count()) +
          ".tjvr");
}

std::vector<std::uint8_t> readTraceBytes(
    const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

TEST(PicoUdpReceiver, ReceivesValidFrameAndStopsIdempotently) {
  LatestSpscExchange<PicoTeleopFrame> exchange;
  PicoUdpReceiver receiver(
      PicoUdpReceiverOptions{"127.0.0.1", 0U, 0.15, 0.60, ""}, exchange);
  receiver.start();
  ASSERT_NE(receiver.boundPort(), 0U);

  sendDatagram(receiver.boundPort(), v2Packet(1U, 9U, 1000000000LL));
  PicoTeleopFrame frame;
  ASSERT_TRUE(waitUntil([&] { return exchange.tryReadLatest(frame); },
                        std::chrono::milliseconds(100)));
  EXPECT_EQ(frame.sequence, 1U);
  EXPECT_EQ(frame.tracking_epoch, 9U);
  EXPECT_GT(frame.receive_monotonic_ns, 0);
  EXPECT_TRUE(frame.left_arm_direction.valid);
  EXPECT_TRUE(frame.right_arm_direction.valid);
  EXPECT_TRUE(frame.left_arm_direction.direction.isApprox(
      -Eigen::Vector3d::UnitZ()));

  receiver.stop();
  receiver.stop();
  EXPECT_EQ(receiver.boundPort(), 0U);
}

TEST(PicoUdpReceiver, RecordsProtocolValidPacketsBeforeStreamGate) {
  const std::filesystem::path trace_path = receiverTracePath();
  std::filesystem::remove(trace_path);
  LatestSpscExchange<PicoTeleopFrame> exchange;
  PicoUdpReceiverOptions options{"127.0.0.1", 0U, 0.15, 0.60, ""};
  options.record_path = trace_path.string();
  PicoUdpReceiver receiver(options, exchange);
  receiver.start();
  const std::uint16_t port = receiver.boundPort();

  const auto accepted_packet = packet(1U, 9U, 1'000'000'000LL);
  sendDatagram(port, accepted_packet);
  sendDatagram(port, std::vector<std::uint8_t>{1U, 2U, 3U});
  auto bad_crc = packet(2U, 9U, 1'010'000'000LL);
  bad_crc[44U] ^= 1U;
  sendDatagram(port, bad_crc);
  auto jump_rejected = packet(2U, 9U, 1'020'000'000LL);
  writeDouble(&jump_rejected, 44U, 1.151);
  refreshCrc(&jump_rejected);
  sendDatagram(port, jump_rejected);

  ASSERT_TRUE(waitUntil([&] { return receiver.stats().datagrams == 4U; },
                        std::chrono::milliseconds(200)));
  receiver.stop();
  const PicoReceiverStats stats = receiver.stats();
  EXPECT_EQ(stats.recording_state, PicoTraceRecorderState::kFinalized);
  EXPECT_EQ(stats.recorded_packets, 2U);
  EXPECT_EQ(stats.recording_packet_size, kPicoTeleopPacketV1Size);
  EXPECT_EQ(stats.accepted, 1U);
  EXPECT_EQ(stats.jump_rejections, 1U);

  const std::vector<std::uint8_t> bytes = readTraceBytes(trace_path);
  ASSERT_EQ(bytes.size(), 16U + 2U * (8U + kPicoTeleopPacketV1Size));
  EXPECT_EQ(traceLe64(bytes, 8U), 2U);
  EXPECT_TRUE(std::equal(accepted_packet.begin(), accepted_packet.end(),
                         bytes.begin() + 24));
  const std::size_t second_packet_offset =
      16U + 8U + kPicoTeleopPacketV1Size + 8U;
  EXPECT_TRUE(std::equal(jump_rejected.begin(), jump_rejected.end(),
                         bytes.begin() +
                             static_cast<std::ptrdiff_t>(second_packet_offset)));
  std::filesystem::remove(trace_path);
}

TEST(PicoUdpReceiver, CountsMalformedOrderingJumpAndEpochEventsExactly) {
  LatestSpscExchange<PicoTeleopFrame> exchange;
  PicoUdpReceiver receiver(
      PicoUdpReceiverOptions{"127.0.0.1", 0U, 0.15, 0.60, ""}, exchange);
  receiver.start();
  const std::uint16_t port = receiver.boundPort();

  sendDatagram(port, packet(1U, 9U, 1000000000LL));
  PicoTeleopFrame consumed;
  ASSERT_TRUE(waitUntil([&] { return exchange.tryReadLatest(consumed); },
                        std::chrono::milliseconds(100)));

  sendDatagram(port, std::vector<std::uint8_t>{1U, 2U, 3U});
  auto bad_crc = packet(2U, 9U, 1010000000LL);
  bad_crc[44] ^= 1U;
  sendDatagram(port, bad_crc);
  sendDatagram(port, packet(1U, 9U, 1010000000LL));
  auto jump = packet(2U, 9U, 1020000000LL);
  writeDouble(&jump, 44, 1.151);
  refreshCrc(&jump);
  sendDatagram(port, jump);
  sendDatagram(port, packet(1U, 10U, 2000000000LL));
  sendDatagram(port, packet(3U, 9U, 1030000000LL));

  ASSERT_TRUE(waitUntil([&] { return receiver.stats().datagrams == 7U; },
                        std::chrono::milliseconds(200)));
  const PicoReceiverStats stats = receiver.stats();
  EXPECT_EQ(stats.datagrams, 7U);
  EXPECT_EQ(stats.accepted, 2U);
  EXPECT_EQ(stats.malformed, 1U);
  EXPECT_EQ(stats.crc_failures, 1U);
  EXPECT_EQ(stats.reordered, 2U);
  EXPECT_EQ(stats.jump_rejections, 1U);
  EXPECT_EQ(stats.superseded, 0U);
  EXPECT_EQ(stats.epoch_resets, 2U);
  EXPECT_EQ(stats.tracking_epoch, 10U);
  EXPECT_EQ(stats.sequence, 1U);
  EXPECT_GT(stats.latest_receive_monotonic_ns, 0);
  receiver.stop();
}

TEST(PicoUdpReceiver, PublishesAndCountsStableClusterResynchronization) {
  LatestSpscExchange<PicoTeleopFrame> exchange;
  PicoUdpReceiver receiver(
      PicoUdpReceiverOptions{"127.0.0.1", 0U, 0.15, 0.60, ""}, exchange);
  receiver.start();
  const std::uint16_t port = receiver.boundPort();

  sendDatagram(port, packet(1U, 9U, 1000000000LL));
  PicoTeleopFrame frame;
  ASSERT_TRUE(waitUntil([&] { return exchange.tryReadLatest(frame); },
                        std::chrono::milliseconds(100)));
  ASSERT_FALSE(frame.stream_discontinuity);

  for (std::uint64_t sequence = 2U; sequence <= 4U; ++sequence) {
    auto candidate = packet(
        sequence, 9U,
        1000000000LL + static_cast<std::int64_t>(sequence) * 10000000LL);
    writeDouble(&candidate, 44U,
                1.28 + 0.01 * static_cast<double>(sequence));
    refreshCrc(&candidate);
    sendDatagram(port, candidate);
  }

  auto after_resynchronization = packet(5U, 9U, 1050000000LL);
  writeDouble(&after_resynchronization, 44U, 1.33);
  refreshCrc(&after_resynchronization);
  sendDatagram(port, after_resynchronization);

  ASSERT_TRUE(waitUntil([&] { return receiver.stats().datagrams == 5U; },
                        std::chrono::milliseconds(200)));
  ASSERT_TRUE(exchange.tryReadLatest(frame));
  EXPECT_EQ(frame.sequence, 5U);
  EXPECT_FALSE(frame.stream_discontinuity);
  EXPECT_EQ(frame.resynchronization_generation, 1U);

  const PicoReceiverStats stats = receiver.stats();
  EXPECT_EQ(stats.accepted, 3U);
  EXPECT_EQ(stats.jump_rejections, 2U);
  EXPECT_EQ(stats.resynchronizations, 1U);
  EXPECT_EQ(stats.epoch_resets, 1U);
  EXPECT_EQ(stats.superseded, 1U);
  receiver.stop();
}

TEST(PicoUdpReceiver, BurstPublishesNewestWithoutBlockingProducer) {
  LatestSpscExchange<PicoTeleopFrame> exchange;
  PicoUdpReceiver receiver(
      PicoUdpReceiverOptions{"127.0.0.1", 0U, 0.15, 0.60, ""}, exchange);
  receiver.start();
  const std::uint16_t port = receiver.boundPort();

  for (std::uint64_t sequence = 1U; sequence <= 32U; ++sequence) {
    sendDatagram(port,
                 packet(sequence, 9U,
                        1000000000LL + static_cast<std::int64_t>(sequence) *
                                           10000000LL));
  }
  ASSERT_TRUE(waitUntil([&] { return receiver.stats().accepted == 32U; },
                        std::chrono::milliseconds(300)));

  PicoTeleopFrame latest;
  ASSERT_TRUE(exchange.tryReadLatest(latest));
  EXPECT_EQ(latest.sequence, 32U);
  const PicoReceiverStats stats = receiver.stats();
  EXPECT_EQ(stats.datagrams, 32U);
  EXPECT_EQ(stats.superseded, 31U);
  EXPECT_NEAR(stats.input_frequency_hz, 100.0, 1e-12);
  receiver.stop();
}

}  // namespace
}  // namespace tianji_qp_ik
