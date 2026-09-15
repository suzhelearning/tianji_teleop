#include "tianji_qp_ik/wuji_hand_teleop_protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace tianji_qp_ik {
namespace {

WujiHandTeleopFrame sampleFrame() {
  WujiHandTeleopFrame frame;
  frame.sequence = 17U;
  frame.source_timestamp_ns = 123456;
  frame.left_source_timestamp_ns = 123450;
  frame.right_source_timestamp_ns = 123451;
  frame.left_valid = true;
  frame.right_valid = true;
  for (std::size_t index = 0; index < kWujiHandJointDof; ++index) {
    frame.left[index] = -0.5 + 0.01 * static_cast<double>(index);
    frame.right[index] = 0.5 + 0.01 * static_cast<double>(index);
  }
  return frame;
}

void updateCrc(std::vector<std::uint8_t>& bytes) {
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t index = 0U; index < bytes.size() - 4U; ++index) {
    crc ^= bytes[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
  }
  crc ^= 0xffffffffU;
  for (std::size_t index = 0U; index < 4U; ++index) {
    bytes[bytes.size() - 4U + index] =
        static_cast<std::uint8_t>(crc >> (8U * index));
  }
}

void setTimestamp(std::vector<std::uint8_t>& bytes, std::size_t offset,
                  std::int64_t timestamp_ns) {
  for (std::size_t index = 0U; index < 8U; ++index) {
    bytes[offset + index] = static_cast<std::uint8_t>(
        static_cast<std::uint64_t>(timestamp_ns) >> (8U * index));
  }
  updateCrc(bytes);
}

WujiHandTeleopFrame trajectoryFrame(std::uint64_t sequence,
                                   std::int64_t timestamp_ns,
                                   double left, double right) {
  WujiHandTeleopFrame frame;
  frame.sequence = sequence;
  frame.source_timestamp_ns = timestamp_ns;
  frame.left_source_timestamp_ns = timestamp_ns;
  frame.right_source_timestamp_ns = timestamp_ns;
  frame.left_valid = frame.right_valid = true;
  frame.left.fill(left);
  frame.right.fill(right);
  return frame;
}

TEST(WujiHandTeleopProtocol, RoundTripsBothSidesWithFixedSize) {
  const WujiHandTeleopFrame frame = sampleFrame();
  const std::vector<std::uint8_t> packet = encodeWujiHandTeleopPacket(frame);

  ASSERT_EQ(packet.size(), kWujiHandTeleopPacketSize);
  EXPECT_EQ(packet.size(), 364U);
  EXPECT_EQ(packet[4], 2U);
  const WujiHandPacketDecodeResult decoded =
      decodeWujiHandTeleopPacket(packet.data(), packet.size());

  ASSERT_TRUE(decoded.frame.has_value());
  EXPECT_EQ(decoded.error, WujiHandPacketError::kNone);
  EXPECT_EQ(decoded.frame->sequence, frame.sequence);
  EXPECT_EQ(decoded.frame->source_timestamp_ns, frame.source_timestamp_ns);
  EXPECT_EQ(decoded.frame->left_source_timestamp_ns, frame.left_source_timestamp_ns);
  EXPECT_EQ(decoded.frame->right_source_timestamp_ns, frame.right_source_timestamp_ns);
  EXPECT_TRUE(decoded.frame->left_valid);
  EXPECT_TRUE(decoded.frame->right_valid);
  EXPECT_EQ(decoded.frame->left, frame.left);
  EXPECT_EQ(decoded.frame->right, frame.right);
}

TEST(WujiHandTeleopProtocol, RejectsWrongMagicVersionSizeAndFlags) {
  const std::vector<std::uint8_t> packet =
      encodeWujiHandTeleopPacket(sampleFrame());

  auto expectError = [](std::vector<std::uint8_t> mutated,
                        WujiHandPacketError error) {
    const WujiHandPacketDecodeResult decoded =
        decodeWujiHandTeleopPacket(mutated.data(), mutated.size());
    EXPECT_FALSE(decoded.frame.has_value());
    EXPECT_EQ(decoded.error, error);
  };

  std::vector<std::uint8_t> wrong_magic = packet;
  wrong_magic[0] = static_cast<std::uint8_t>('X');
  expectError(wrong_magic, WujiHandPacketError::kWrongMagic);

  std::vector<std::uint8_t> wrong_version = packet;
  wrong_version[4] = 1U;
  expectError(wrong_version, WujiHandPacketError::kWrongVersion);

  std::vector<std::uint8_t> wrong_size = packet;
  wrong_size[6] = 0U;
  wrong_size[7] = 0U;
  expectError(wrong_size, WujiHandPacketError::kWrongDeclaredSize);

  std::vector<std::uint8_t> wrong_flags = packet;
  wrong_flags[5] = 0x80U;
  expectError(wrong_flags, WujiHandPacketError::kInvalidFlags);
}

TEST(WujiHandTeleopProtocol, RejectsCrcAndNonFiniteValues) {
  std::vector<std::uint8_t> packet =
      encodeWujiHandTeleopPacket(sampleFrame());
  packet.back() ^= 0x01U;
  const WujiHandPacketDecodeResult crc_result =
      decodeWujiHandTeleopPacket(packet.data(), packet.size());
  EXPECT_FALSE(crc_result.frame.has_value());
  EXPECT_EQ(crc_result.error, WujiHandPacketError::kCrcMismatch);

  WujiHandTeleopFrame invalid = sampleFrame();
  invalid.left[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(encodeWujiHandTeleopPacket(invalid), std::invalid_argument);
}

TEST(WujiHandTeleopProtocol, RejectsWrongInputSizeAndNullBytes) {
  const WujiHandPacketDecodeResult null_result =
      decodeWujiHandTeleopPacket(nullptr, kWujiHandTeleopPacketSize);
  EXPECT_FALSE(null_result.frame.has_value());
  EXPECT_EQ(null_result.error, WujiHandPacketError::kWrongSize);

  const std::array<std::uint8_t, 2> short_packet{};
  const WujiHandPacketDecodeResult short_result =
      decodeWujiHandTeleopPacket(short_packet.data(), short_packet.size());
  EXPECT_FALSE(short_result.frame.has_value());
  EXPECT_EQ(short_result.error, WujiHandPacketError::kWrongSize);
}

TEST(WujiHandTeleopProtocol, RejectsInvalidSideSourceMetadataWithValidCrc) {
  const auto frame = sampleFrame();
  const auto original = encodeWujiHandTeleopPacket(frame);
  for (std::size_t offset : {24U, 32U}) {
    for (std::int64_t invalid : {std::int64_t{0}, std::int64_t{-1},
                                frame.source_timestamp_ns + 1}) {
      auto packet = original;
      setTimestamp(packet, offset, invalid);
      EXPECT_EQ(decodeWujiHandTeleopPacket(packet.data(), packet.size()).error,
                WujiHandPacketError::kInvalidMetadata);
    }
  }
  auto packet = original;
  packet[5] = kWujiHandLeftValidFlag;  // Invalid right side must have timestamp zero.
  updateCrc(packet);
  EXPECT_EQ(decodeWujiHandTeleopPacket(packet.data(), packet.size()).error,
            WujiHandPacketError::kInvalidMetadata);
  setTimestamp(packet, 32U, 0);
  const auto decoded = decodeWujiHandTeleopPacket(packet.data(), packet.size());
  ASSERT_TRUE(decoded.frame);
  EXPECT_TRUE(decoded.frame->left_valid);
  EXPECT_FALSE(decoded.frame->right_valid);
  EXPECT_EQ(decoded.frame->left, frame.left);

  auto invalid = frame;
  invalid.left_source_timestamp_ns = 0;
  EXPECT_THROW(encodeWujiHandTeleopPacket(invalid), std::invalid_argument);
  invalid = frame;
  invalid.right_source_timestamp_ns = frame.source_timestamp_ns + 1;
  EXPECT_THROW(encodeWujiHandTeleopPacket(invalid), std::invalid_argument);
  invalid = frame;
  invalid.right_valid = false;
  EXPECT_THROW(encodeWujiHandTeleopPacket(invalid), std::invalid_argument);
}

TEST(WujiHandHistory, InterpolatesEachJointOnDelayedPublicationTimeline) {
  WujiHandHistory history;
  ASSERT_TRUE(history.observe(trajectoryFrame(1U, 1000000000, 0.0, 2.0)));
  auto next = trajectoryFrame(2U, 1010000000, 1.0, -2.0);
  // Retargeting callback age is not the interpolation timeline.
  next.left_source_timestamp_ns = 1001000000;
  next.right_source_timestamp_ns = 1002000000;
  ASSERT_TRUE(history.observe(next));
  const auto midpoint = history.sample(1015000000);
  ASSERT_TRUE(midpoint.left_valid);
  ASSERT_TRUE(midpoint.right_valid);
  for (std::size_t joint = 0U; joint < kWujiHandJointDof; ++joint) {
    EXPECT_DOUBLE_EQ(midpoint.left[joint], 0.5);
    EXPECT_DOUBLE_EQ(midpoint.right[joint], 0.0);
  }
  EXPECT_DOUBLE_EQ(history.sample(1010000000).left[0], 0.0);
  EXPECT_DOUBLE_EQ(history.sample(1020000000).left[0], 1.0);
}

TEST(WujiHandHistory, HoldsStartupAndLateEndpointsWithoutExtrapolation) {
  WujiHandHistory history;
  EXPECT_FALSE(history.sample(1000000000).left_valid);
  EXPECT_FALSE(history.sample(1000000000).right_valid);
  ASSERT_TRUE(history.observe(trajectoryFrame(1U, 1000000000, 0.25, -0.25)));
  EXPECT_DOUBLE_EQ(history.sample(1000000000).left[0], 0.25);
  EXPECT_DOUBLE_EQ(history.sample(1040000000).right[0], -0.25);
  // A delayed next packet is already behind the output timeline: hold it,
  // rather than extrapolating the velocity from the two available points.
  ASSERT_TRUE(history.observe(trajectoryFrame(2U, 1020000000, 0.75, -0.75)));
  EXPECT_DOUBLE_EQ(history.sample(1040000000).left[0], 0.75);
  EXPECT_DOUBLE_EQ(history.sample(1200000000).right[0], -0.75);
  history.clear();
  EXPECT_FALSE(history.sample(1200000000).left_valid);
  EXPECT_FALSE(history.sample(1200000000).right_valid);
  ASSERT_TRUE(history.observe(trajectoryFrame(3U, 1210000000, 0.9, -0.9)));
  EXPECT_DOUBLE_EQ(history.sample(1210000000).left[0], 0.9);
}

TEST(WujiHandHistory, MissingSidesNeverSupplyZeroEndpoints) {
  WujiHandHistory history;
  auto left = trajectoryFrame(1U, 1000000000, 0.2, 0.0);
  left.right_valid = false;
  left.right_source_timestamp_ns = 0;
  ASSERT_TRUE(history.observe(left));
  EXPECT_FALSE(history.sample(1010000000).right_valid);
  auto right = trajectoryFrame(2U, 1010000000, 0.0, -0.8);
  right.left_valid = false;
  right.left_source_timestamp_ns = 0;
  ASSERT_TRUE(history.observe(right));
  auto next = left;
  next.sequence = 3U;
  next.source_timestamp_ns = next.left_source_timestamp_ns = 1020000000;
  next.left.fill(0.6);
  ASSERT_TRUE(history.observe(next));
  const auto sample = history.sample(1020000000);
  ASSERT_TRUE(sample.left_valid);
  ASSERT_TRUE(sample.right_valid);
  EXPECT_DOUBLE_EQ(sample.left[0], 0.4);
  EXPECT_DOUBLE_EQ(sample.right[0], -0.8);
}

TEST(WujiHandHistory, RingWrapAndRejectedRegressionsPreserveNewestTrajectory) {
  WujiHandHistory history;
  for (std::uint64_t index = 1U; index <= 20U; ++index) {
    ASSERT_TRUE(history.observe(trajectoryFrame(
        index, 1000000000 + static_cast<std::int64_t>(index) * 10000000,
        static_cast<double>(index), -static_cast<double>(index))));
  }
  EXPECT_DOUBLE_EQ(history.sample(1205000000).left[0], 19.5);
  EXPECT_DOUBLE_EQ(history.sample(1220000000).right[0], -20.0);
  auto rollback = trajectoryFrame(21U, 1190000000, 99.0, 99.0);
  EXPECT_FALSE(history.observe(rollback));
  rollback.source_timestamp_ns = 1200000000;  // Same-time publication.
  EXPECT_FALSE(history.observe(rollback));
  rollback.source_timestamp_ns = 1210000000;  // Regressed callback source.
  EXPECT_FALSE(history.observe(rollback));
  EXPECT_DOUBLE_EQ(history.sample(1220000000).left[0], 20.0);
}

}  // namespace
}  // namespace tianji_qp_ik
