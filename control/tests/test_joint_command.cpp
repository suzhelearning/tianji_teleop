#include "tianji_qp_ik/joint_command.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

JointCommandFrame command() {
  JointCommandFrame frame;
  frame.sequence = 0x0102030405060708ULL;
  frame.source_timestamp_ns = 123456789;
  frame.pico_tracking_epoch = 42U;
  frame.flags = kJointCommandKnownFlags;
  for (std::size_t i = 0; i < frame.position_rad.size(); ++i) {
    frame.position_rad[i] = static_cast<double>(i) / 16.0;
  }
  return frame;
}

template <std::size_t Size>
void updateCrc(std::array<std::uint8_t, Size>& bytes) {
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t i = 0; i < bytes.size() - 4U; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
  }
  crc ^= 0xffffffffU;
  for (std::size_t i = 0; i < 4U; ++i) {
    bytes[bytes.size() - 4U + i] = static_cast<std::uint8_t>((crc >> (8U * i)) & 0xffU);
  }
}

TEST(JointCommand, ValidCrcCannotHideInvalidMetadataOrNonFinitePayload) {
  auto bytes = encodeJointCommandPacket(command());
  for (std::size_t i = 8; i < 16; ++i) {
    bytes[i] = 0U;
  }
  updateCrc(bytes);
  EXPECT_EQ(decodeJointCommandPacket(bytes.data(), bytes.size()).error,
            JointCommandPacketError::kInvalidMetadata);
  bytes = encodeJointCommandPacket(command());
  bytes[23] |= 0x80U;  // Negative signed source timestamp.
  updateCrc(bytes);
  EXPECT_EQ(decodeJointCommandPacket(bytes.data(), bytes.size()).error,
            JointCommandPacketError::kInvalidMetadata);
  bytes = encodeJointCommandPacket(command());
  // Last joint is +infinity, including when the packet is disabled.
  bytes[5] = 0U;
  for (std::size_t i = bytes.size() - 12U; i < bytes.size() - 6U; ++i) {
    bytes[i] = 0U;
  }
  bytes[bytes.size() - 6U] = 0xf0U;
  bytes[bytes.size() - 5U] = 0x7fU;
  updateCrc(bytes);
  EXPECT_EQ(decodeJointCommandPacket(bytes.data(), bytes.size()).error,
            JointCommandPacketError::kNonFiniteJointPosition);
}

TEST(JointCommand, AppliedPicoBridgeTimestampFailsClosedAtExpiryOrMissingClock) {
  constexpr std::int64_t sent = 1000000000;
  EXPECT_TRUE(jointCommandPicoBridgeFresh(sent, sent + 1));
  EXPECT_TRUE(jointCommandPicoBridgeFresh(sent, sent + 300000000));
  EXPECT_FALSE(jointCommandPicoBridgeFresh(sent, sent + 300000001));
  EXPECT_FALSE(jointCommandPicoBridgeFresh(0, sent));
  EXPECT_FALSE(jointCommandPicoBridgeFresh(-1, sent));
  EXPECT_FALSE(jointCommandPicoBridgeFresh(sent, sent - 1));
  EXPECT_FALSE(jointCommandPicoBridgeFresh(sent, sent));
}

TEST(JointCommand, RuntimeResetPermanentlyInhibitsArmsButStartupResetDoesNot) {
  JointCommandArmReadiness readiness;
  EXPECT_FALSE(readiness.update(true, true));
  EXPECT_TRUE(readiness.update(true, false));
  EXPECT_FALSE(readiness.update(false, true));
  // Dropping the disabled reset datagram cannot restore readiness.
  EXPECT_FALSE(readiness.update(true, false));
  EXPECT_FALSE(readiness.update(true, false));
  JointCommandArmReadiness restarted;
  EXPECT_TRUE(restarted.update(true, false));
}

TEST(JointCommand, WireLayoutAndRoundTripIncludeAllFourJointGroups) {
  const auto original = command();
  const auto bytes = encodeJointCommandPacket(original);
  ASSERT_EQ(bytes.size(), 468U);
  EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 4), "TJRC");
  EXPECT_EQ(bytes[4], 2U);
  EXPECT_EQ(bytes[5], 7U);
  EXPECT_EQ(bytes[6], 0xd4U);
  EXPECT_EQ(bytes[7], 0x01U);
  EXPECT_EQ(bytes[8], 0x08U);
  EXPECT_EQ(bytes[15], 0x01U);
  EXPECT_EQ(bytes[24], 42U);
  // Joint 16 is 1.0, little-endian IEEE754 at offset 32 + 16*8.
  EXPECT_EQ(bytes[166], 0xf0U);
  EXPECT_EQ(bytes[167], 0x3fU);
  const auto decoded = decodeJointCommandPacket(bytes.data(), bytes.size());
  ASSERT_TRUE(decoded.frame.has_value());
  EXPECT_EQ(decoded.frame->position_rad, original.position_rad);
  EXPECT_EQ(decoded.frame->sequence, original.sequence);
  EXPECT_EQ(decoded.frame->source_timestamp_ns, original.source_timestamp_ns);
  EXPECT_EQ(decoded.frame->pico_tracking_epoch, original.pico_tracking_epoch);
  EXPECT_EQ(decoded.frame->flags, original.flags);
}

TEST(JointCommand, DisabledHeartbeatIsValidButUnsafeMetadataCannotBeEncoded) {
  auto frame = command();
  frame.flags = 0U;
  auto bytes = encodeJointCommandPacket(frame);
  ASSERT_TRUE(decodeJointCommandPacket(bytes.data(), bytes.size()).frame);
  frame.flags = 8U;
  EXPECT_THROW(encodeJointCommandPacket(frame), std::invalid_argument);
  frame.flags = 0U;
  frame.sequence = 0U;
  EXPECT_THROW(encodeJointCommandPacket(frame), std::invalid_argument);
  frame.sequence = 1U;
  frame.source_timestamp_ns = 0;
  EXPECT_THROW(encodeJointCommandPacket(frame), std::invalid_argument);
  frame.source_timestamp_ns = 1;
  frame.position_rad.back() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(encodeJointCommandPacket(frame), std::invalid_argument);
}

TEST(JointCommand, RejectsTruncationUnknownFlagsAndCorruption) {
  auto bytes = encodeJointCommandPacket(command());
  EXPECT_EQ(decodeJointCommandPacket(nullptr, bytes.size()).error,
            JointCommandPacketError::kWrongSize);
  EXPECT_EQ(decodeJointCommandPacket(bytes.data(), bytes.size() - 1).error,
            JointCommandPacketError::kWrongSize);
  bytes[5] = 8U;
  EXPECT_EQ(decodeJointCommandPacket(bytes.data(), bytes.size()).error,
            JointCommandPacketError::kInvalidFlags);
  bytes[5] = kJointCommandKnownFlags;
  bytes[40] ^= 1U;
  EXPECT_EQ(decodeJointCommandPacket(bytes.data(), bytes.size()).error,
            JointCommandPacketError::kCrcMismatch);
}

TEST(JointCommand, RejectsLegacyAndUnknownVersionsEvenWithValidCrc) {
  auto bytes = encodeJointCommandPacket(command());
  for (std::uint8_t version : std::array<std::uint8_t, 3U>{0U, 1U, 3U}) {
    bytes[4] = version;
    updateCrc(bytes);
    EXPECT_EQ(decodeJointCommandPacket(bytes.data(), bytes.size()).error,
              JointCommandPacketError::kWrongVersion);
  }
  std::array<std::uint8_t, 308U> legacy{};
  std::copy_n(bytes.begin(), legacy.size() - 4U, legacy.begin());
  legacy[4] = 1U;
  legacy[5] = 3U;
  legacy[6] = 0x34U;
  legacy[7] = 0x01U;
  updateCrc(legacy);
  EXPECT_EQ(decodeJointCommandPacket(legacy.data(), legacy.size()).error,
            JointCommandPacketError::kWrongSize);
}

TEST(JointCommand, EachHandRequiresItsOwnSourceAndReceiveFreshness) {
  HandCommandFreshness left(ArmSide::kLeft);
  HandCommandFreshness right(ArmSide::kRight);
  WujiHandTeleopFrame frame;
  frame.sequence = 1U;
  frame.source_timestamp_ns = frame.receive_monotonic_ns = 1100000000;
  frame.left_source_timestamp_ns = 1000000000;
  frame.left_valid = true;
  EXPECT_FALSE(left.live(1100000000));
  left.observe(frame);
  right.observe(frame);
  EXPECT_TRUE(left.live(1149999999));
  EXPECT_FALSE(right.live(1100000000));
  EXPECT_FALSE(left.live(1150000000));  // Source expires before publication.

  frame.sequence = 2U;
  frame.source_timestamp_ns = frame.receive_monotonic_ns = 1140000000;
  frame.left_valid = false;
  frame.left_source_timestamp_ns = 0;
  frame.right_valid = true;
  frame.right_source_timestamp_ns = 1140000000;
  left.observe(frame);
  right.observe(frame);
  EXPECT_TRUE(left.live(1149999999));  // Absence does not immediately invalidate.
  EXPECT_FALSE(left.live(1150000000));
  EXPECT_TRUE(right.live(1150000000));
  EXPECT_FALSE(right.live(1139999999));  // Future source/receive clocks.
}

TEST(JointCommand, RepeatedPublicationsCannotRejuvenateFrozenSideSource) {
  HandCommandFreshness left(ArmSide::kLeft);
  HandCommandFreshness right(ArmSide::kRight);
  WujiHandTeleopFrame frame;
  frame.left_valid = frame.right_valid = true;
  frame.left_source_timestamp_ns = 1000000000;
  for (std::uint64_t index = 0U; index <= 20U; ++index) {
    const std::int64_t now = 1000000000 + static_cast<std::int64_t>(index) * 10000000;
    frame.sequence = index + 1U;
    frame.source_timestamp_ns = frame.receive_monotonic_ns = now;
    frame.right_source_timestamp_ns = now;
    left.observe(frame);
    right.observe(frame);
    EXPECT_EQ(left.live(now), index < 15U);
    EXPECT_TRUE(right.live(now));
  }
}

TEST(JointCommand, HandReadinessFailsClosedOnMalformedInputAndReset) {
  for (ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    HandCommandFreshness freshness(side);
    WujiHandTeleopFrame frame;
    frame.left_valid = frame.right_valid = true;
    frame.sequence = 1U;
    frame.source_timestamp_ns = frame.receive_monotonic_ns = 1000000000;
    frame.left_source_timestamp_ns = frame.right_source_timestamp_ns = 1000000000;
    freshness.observe(frame);
    ASSERT_TRUE(freshness.live(1000000000));
    freshness.reset();
    frame.sequence = 2U;
    frame.source_timestamp_ns = frame.receive_monotonic_ns = 1010000000;
    freshness.observe(frame);
    EXPECT_FALSE(freshness.live(1010000000));  // Repeated source cannot re-arm.
    auto& source = side == ArmSide::kLeft ? frame.left_source_timestamp_ns
                                        : frame.right_source_timestamp_ns;
    source = 1020000000;
    frame.sequence = 3U;
    frame.source_timestamp_ns = frame.receive_monotonic_ns = 1020000000;
    freshness.observe(frame);
    EXPECT_TRUE(freshness.live(1020000000));
    freshness.observe(frame);  // Replay fails closed.
    EXPECT_FALSE(freshness.live(1020000000));
    frame.sequence = 4U;
    frame.source_timestamp_ns = source = 1180000000;
    freshness.observe(frame);
    EXPECT_FALSE(freshness.live(1180000000));  // Old receiver clock.
    frame.sequence = 5U;
    frame.source_timestamp_ns = source = frame.receive_monotonic_ns = 1190000000;
    auto& positions = side == ArmSide::kLeft ? frame.left : frame.right;
    positions[0] = std::numeric_limits<double>::quiet_NaN();
    freshness.observe(frame);
    EXPECT_FALSE(freshness.live(1190000000));
    positions[0] = 0.0;
    source = 0;
    freshness.observe(frame);
    EXPECT_FALSE(freshness.live(1190000000));
    source = frame.source_timestamp_ns + 1;
    freshness.observe(frame);
    EXPECT_FALSE(freshness.live(1190000000));
  }
}

TEST(JointCommand, ExporterRejectsNonLoopbackDestinationWithoutSending) {
  EXPECT_THROW(JointCommandExporter("192.0.2.1", 15001U), std::invalid_argument);
  EXPECT_THROW(JointCommandExporter("localhost", 15001U), std::invalid_argument);
  EXPECT_THROW(JointCommandExporter("127.0.0.1", 0U), std::invalid_argument);
}

}  // namespace
}  // namespace tianji_qp_ik
