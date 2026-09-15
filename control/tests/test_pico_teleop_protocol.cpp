#include "tianji_qp_ik/pico_teleop_protocol.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
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
  if (text.size() % 2 != 0) {
    throw std::invalid_argument("hex length must be even");
  }
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
  (*bytes)[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
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
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe64(bytes, offset, bits);
}

void refreshCrc(std::vector<std::uint8_t>* bytes) {
  const std::size_t crc_offset = bytes->size() - 4U;
  writeLe32(bytes, crc_offset, crc32(bytes->data(), crc_offset));
}

std::vector<std::uint8_t> v2Packet() {
  auto bytes = fromHex(kGoldenPacketHex);
  bytes.resize(208U, 0U);
  writeLe16(&bytes, 4U, 2U);
  writeLe16(&bytes, 6U, 208U);
  writeLe32(&bytes, 40U, 0x3fU);
  writeDouble(&bytes, 156U, 0.0);
  writeDouble(&bytes, 164U, 0.0);
  writeDouble(&bytes, 172U, -2.0);
  writeDouble(&bytes, 180U, 0.0);
  writeDouble(&bytes, 188U, 3.0);
  writeDouble(&bytes, 196U, 0.0);
  refreshCrc(&bytes);
  return bytes;
}

std::vector<std::uint8_t> v3Packet() {
  auto bytes = v2Packet();
  bytes.resize(400U, 0U);
  writeLe16(&bytes, 4U, 3U);
  writeLe16(&bytes, 6U, 400U);
  writeLe32(&bytes, 40U, 0x7fU);
  for (std::size_t index = 0; index < 8U; ++index) {
    writeDouble(&bytes, 204U + index * 24U,
                static_cast<double>(index) + 0.1);
    writeDouble(&bytes, 212U + index * 24U,
                static_cast<double>(index) + 0.2);
    writeDouble(&bytes, 220U + index * 24U,
                static_cast<double>(index) + 0.3);
  }
  refreshCrc(&bytes);
  return bytes;
}

std::vector<std::uint8_t> v4Packet() {
  auto bytes = v3Packet();
  bytes.resize(656U, 0U);
  writeLe16(&bytes, 4U, 4U);
  writeLe16(&bytes, 6U, 656U);
  writeLe32(&bytes, 40U, 0xffU);
  for (std::size_t index = 0; index < 8U; ++index) {
    const Eigen::Quaterniond quaternion(
        Eigen::AngleAxisd(0.1 * static_cast<double>(index + 1U),
                          Eigen::Vector3d::UnitZ()));
    const std::size_t offset = 396U + index * 32U;
    writeDouble(&bytes, offset, quaternion.x());
    writeDouble(&bytes, offset + 8U, quaternion.y());
    writeDouble(&bytes, offset + 16U, quaternion.z());
    writeDouble(&bytes, offset + 24U, quaternion.w());
  }
  refreshCrc(&bytes);
  return bytes;
}

PicoTeleopFrame makeFrame(std::uint64_t sequence, std::uint64_t epoch = 9) {
  PicoTeleopFrame frame;
  frame.sequence = sequence;
  frame.tracking_epoch = epoch;
  frame.source_timestamp_ns = static_cast<std::int64_t>(sequence) + 10;
  frame.bridge_send_monotonic_ns = static_cast<std::int64_t>(sequence) + 20;
  return frame;
}

TEST(PicoTeleopProtocol, DecodesGoldenLittleEndianPacket) {
  const auto bytes = fromHex(kGoldenPacketHex);
  const PicoPacketDecodeResult result =
      decodePicoTeleopPacket(bytes.data(), bytes.size());

  ASSERT_EQ(result.error, PicoPacketError::kNone);
  ASSERT_TRUE(result.frame.has_value());
  EXPECT_EQ(result.frame->sequence, 7U);
  EXPECT_EQ(result.frame->tracking_epoch, 9U);
  EXPECT_EQ(result.frame->source_timestamp_ns, 11);
  EXPECT_EQ(result.frame->bridge_send_monotonic_ns, 13);
  EXPECT_EQ(result.frame->receive_monotonic_ns, 0);
  EXPECT_TRUE(result.frame->left.position.isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(result.frame->right.position.isApprox(
      Eigen::Vector3d(-1.0, -2.0, -3.0)));
  EXPECT_TRUE(result.frame->left.rotation.isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(result.frame->right.rotation.isApprox(
      Eigen::AngleAxisd(3.14159265358979323846, Eigen::Vector3d::UnitZ())
          .toRotationMatrix(),
      1e-12));
  EXPECT_FALSE(result.frame->left_arm_direction.valid);
  EXPECT_FALSE(result.frame->right_arm_direction.valid);
}

TEST(PicoTeleopProtocol, DecodesV2ArmDirectionsAndNormalizesThem) {
  const auto bytes = v2Packet();
  const PicoPacketDecodeResult result =
      decodePicoTeleopPacket(bytes.data(), bytes.size());

  ASSERT_EQ(result.error, PicoPacketError::kNone);
  ASSERT_TRUE(result.frame.has_value());
  ASSERT_TRUE(result.frame->left_arm_direction.valid);
  ASSERT_TRUE(result.frame->right_arm_direction.valid);
  EXPECT_TRUE(result.frame->left_arm_direction.direction.isApprox(
      -Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_TRUE(result.frame->right_arm_direction.direction.isApprox(
      Eigen::Vector3d::UnitY(), 1e-12));
}

TEST(PicoTeleopProtocol, DecodesV3ExactUpperLimbSkeleton) {
  const auto bytes = v3Packet();
  const PicoPacketDecodeResult result =
      decodePicoTeleopPacket(bytes.data(), bytes.size());

  ASSERT_EQ(result.error, PicoPacketError::kNone);
  ASSERT_TRUE(result.frame.has_value());
  ASSERT_TRUE(result.frame->upper_limb_skeleton.valid);
  for (std::size_t index = 0;
       index < result.frame->upper_limb_skeleton.points.size(); ++index) {
    EXPECT_TRUE(result.frame->upper_limb_skeleton.points[index].isApprox(
        Eigen::Vector3d(static_cast<double>(index) + 0.1,
                        static_cast<double>(index) + 0.2,
                        static_cast<double>(index) + 0.3),
        1e-12));
  }
}

TEST(PicoTeleopProtocol, DecodesPicoTrackerMainV4UpperLimbRotations) {
  const auto bytes = v4Packet();
  const PicoPacketDecodeResult result =
      decodePicoTeleopPacket(bytes.data(), bytes.size());

  ASSERT_EQ(result.error, PicoPacketError::kNone);
  ASSERT_TRUE(result.frame.has_value());
  ASSERT_TRUE(result.frame->upper_limb_skeleton.valid);
  ASSERT_TRUE(result.frame->upper_limb_skeleton.rotations_valid);
  for (std::size_t index = 0; index < 8U; ++index) {
    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd(0.1 * static_cast<double>(index + 1U),
                          Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    EXPECT_TRUE(result.frame->upper_limb_skeleton.rotations[index]
                    .toRotationMatrix()
                    .isApprox(expected, 1e-12));
  }
}

TEST(PicoTeleopProtocol, DecodesV4UserButtonFlag) {
  auto bytes = v4Packet();
  writeLe32(&bytes, 40U, 0x1ffU);
  refreshCrc(&bytes);

  const PicoPacketDecodeResult result =
      decodePicoTeleopPacket(bytes.data(), bytes.size());

  ASSERT_EQ(result.error, PicoPacketError::kNone);
  ASSERT_TRUE(result.frame.has_value());
  EXPECT_TRUE(result.frame->user_button_pressed);
}

TEST(PicoTeleopProtocol, RejectsV4UnknownFlagAboveUserButton) {
  auto bytes = v4Packet();
  writeLe32(&bytes, 40U, 0x2ffU);
  refreshCrc(&bytes);

  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kInvalidFlags);
}

TEST(PicoTeleopProtocol, V4WithoutRotationFlagUsesPositionFallback) {
  auto bytes = v4Packet();
  writeLe32(&bytes, 40U, 0x7fU);
  refreshCrc(&bytes);

  const PicoPacketDecodeResult result =
      decodePicoTeleopPacket(bytes.data(), bytes.size());
  ASSERT_EQ(result.error, PicoPacketError::kNone);
  ASSERT_TRUE(result.frame.has_value());
  EXPECT_TRUE(result.frame->upper_limb_skeleton.valid);
  EXPECT_FALSE(result.frame->upper_limb_skeleton.rotations_valid);
}

TEST(PicoTeleopProtocol, RejectsV4RotationsWithoutSkeletonPositions) {
  auto bytes = v4Packet();
  writeLe32(&bytes, 40U, 0xbfU);
  refreshCrc(&bytes);
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kInvalidFlags);
}

TEST(PicoTeleopProtocol, RejectsInvalidAdvertisedV4Rotation) {
  auto bytes = v4Packet();
  for (std::size_t offset = 396U; offset < 428U; offset += 8U) {
    writeDouble(&bytes, offset, 0.0);
  }
  refreshCrc(&bytes);
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kInvalidQuaternion);

  bytes = v4Packet();
  writeDouble(&bytes, 396U, std::nan(""));
  refreshCrc(&bytes);
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kNonFinitePose);
}

TEST(PicoTeleopProtocol, KeepsSkeletonInvalidForLegacyPackets) {
  const auto v1 = fromHex(kGoldenPacketHex);
  const auto v2 = v2Packet();

  ASSERT_TRUE(decodePicoTeleopPacket(v1.data(), v1.size()).frame.has_value());
  ASSERT_TRUE(decodePicoTeleopPacket(v2.data(), v2.size()).frame.has_value());
  EXPECT_FALSE(decodePicoTeleopPacket(v1.data(), v1.size())
                   .frame->upper_limb_skeleton.valid);
  EXPECT_FALSE(decodePicoTeleopPacket(v2.data(), v2.size())
                   .frame->upper_limb_skeleton.valid);
}

TEST(PicoTeleopProtocol, InvalidV2OptionalDirectionDoesNotRejectPoses) {
  auto bytes = v2Packet();
  writeDouble(&bytes, 156U, std::nan(""));
  refreshCrc(&bytes);

  const PicoPacketDecodeResult result =
      decodePicoTeleopPacket(bytes.data(), bytes.size());
  ASSERT_EQ(result.error, PicoPacketError::kNone);
  ASSERT_TRUE(result.frame.has_value());
  EXPECT_FALSE(result.frame->left_arm_direction.valid);
  EXPECT_TRUE(result.frame->right_arm_direction.valid);
}

TEST(PicoTeleopProtocol, RejectsMalformedHeaderAndCrc) {
  const auto golden = fromHex(kGoldenPacketHex);

  EXPECT_EQ(decodePicoTeleopPacket(golden.data(), golden.size() - 1).error,
            PicoPacketError::kWrongSize);

  auto bytes = golden;
  bytes[0] = static_cast<std::uint8_t>('X');
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kWrongMagic);

  bytes = golden;
  bytes[4] = 2;
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kWrongVersion);

  bytes = golden;
  bytes[6] = 159;
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kWrongDeclaredSize);

  bytes = golden;
  bytes[40] = 0x07;
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kInvalidFlags);

  bytes = golden;
  bytes[44] ^= 0x01U;
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kCrcMismatch);
}

TEST(PicoTeleopProtocol, RejectsInvalidMetadataPoseAndQuaternion) {
  const auto golden = fromHex(kGoldenPacketHex);

  for (const std::size_t offset : {std::size_t{8}, std::size_t{16},
                                   std::size_t{24}, std::size_t{32}}) {
    auto bytes = golden;
    writeLe64(&bytes, offset, 0);
    refreshCrc(&bytes);
    EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
              PicoPacketError::kInvalidMetadata)
        << "offset=" << offset;
  }

  auto bytes = golden;
  writeDouble(&bytes, 44, std::nan(""));
  refreshCrc(&bytes);
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kNonFinitePose);

  bytes = golden;
  for (std::size_t offset = 68; offset < 100; offset += 8) {
    writeDouble(&bytes, offset, 0.0);
  }
  refreshCrc(&bytes);
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kInvalidQuaternion);

  bytes = golden;
  writeDouble(&bytes, 92, 1.002);
  refreshCrc(&bytes);
  EXPECT_EQ(decodePicoTeleopPacket(bytes.data(), bytes.size()).error,
            PicoPacketError::kInvalidQuaternion);
}

TEST(PicoTeleopStreamGate, AcceptsFirstFrameAndRejectsOrderingErrors) {
  PicoTeleopStreamGate gate(0.15, 0.60);
  const PicoStreamDecision first = gate.evaluate(makeFrame(1));
  EXPECT_TRUE(first.accepted);
  EXPECT_TRUE(first.epoch_changed);
  EXPECT_EQ(first.reason, PicoStreamRejectReason::kNone);

  const PicoStreamDecision duplicate = gate.evaluate(makeFrame(1));
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_EQ(duplicate.reason, PicoStreamRejectReason::kOutOfOrder);

  const PicoStreamDecision older = gate.evaluate(makeFrame(0));
  EXPECT_FALSE(older.accepted);
  EXPECT_EQ(older.reason, PicoStreamRejectReason::kOutOfOrder);
}

TEST(PicoTeleopStreamGate, RejectsIndependentPositionAndOrientationJumps) {
  PicoTeleopStreamGate position_gate(0.15, 0.60);
  ASSERT_TRUE(position_gate.evaluate(makeFrame(1)).accepted);
  PicoTeleopFrame position_jump = makeFrame(2);
  position_jump.right.position.x() = 0.151;
  const PicoStreamDecision position = position_gate.evaluate(position_jump);
  EXPECT_FALSE(position.accepted);
  EXPECT_EQ(position.reason, PicoStreamRejectReason::kPositionJump);

  PicoTeleopStreamGate orientation_gate(0.15, 0.60);
  ASSERT_TRUE(orientation_gate.evaluate(makeFrame(1)).accepted);
  PicoTeleopFrame orientation_jump = makeFrame(2);
  orientation_jump.left.rotation =
      Eigen::AngleAxisd(0.601, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const PicoStreamDecision orientation = orientation_gate.evaluate(orientation_jump);
  EXPECT_FALSE(orientation.accepted);
  EXPECT_EQ(orientation.reason, PicoStreamRejectReason::kOrientationJump);
}

TEST(PicoTeleopStreamGate, ResynchronizesAfterThreeStableJumpFrames) {
  PicoTeleopStreamGate gate(0.15, 0.60);
  ASSERT_TRUE(gate.evaluate(makeFrame(1)).accepted);

  PicoTeleopFrame first_candidate = makeFrame(2);
  first_candidate.left.position.x() = 0.30;
  const PicoStreamDecision first = gate.evaluate(first_candidate);
  EXPECT_FALSE(first.accepted);
  EXPECT_EQ(first.reason, PicoStreamRejectReason::kPositionJump);

  PicoTeleopFrame second_candidate = makeFrame(3);
  second_candidate.left.position.x() = 0.31;
  const PicoStreamDecision second = gate.evaluate(second_candidate);
  EXPECT_FALSE(second.accepted);
  EXPECT_EQ(second.reason, PicoStreamRejectReason::kPositionJump);

  PicoTeleopFrame third_candidate = makeFrame(4);
  third_candidate.left.position.x() = 0.32;
  const PicoStreamDecision third = gate.evaluate(third_candidate);
  EXPECT_TRUE(third.accepted);
  EXPECT_FALSE(third.epoch_changed);
  EXPECT_EQ(third.reason, PicoStreamRejectReason::kNone);
  EXPECT_TRUE(third.stream_discontinuity);
}

TEST(PicoTeleopStreamGate, RestartsCandidateAfterAnotherDiscontinuity) {
  PicoTeleopStreamGate gate(0.15, 0.60);
  ASSERT_TRUE(gate.evaluate(makeFrame(1)).accepted);

  PicoTeleopFrame first_cluster = makeFrame(2);
  first_cluster.left.position.x() = 0.30;
  EXPECT_FALSE(gate.evaluate(first_cluster).accepted);

  PicoTeleopFrame new_cluster = makeFrame(3);
  new_cluster.left.position.x() = 0.60;
  EXPECT_FALSE(gate.evaluate(new_cluster).accepted);

  PicoTeleopFrame second_candidate = makeFrame(4);
  second_candidate.left.position.x() = 0.61;
  EXPECT_FALSE(gate.evaluate(second_candidate).accepted);

  PicoTeleopFrame third_candidate = makeFrame(5);
  third_candidate.left.position.x() = 0.62;
  const PicoStreamDecision decision = gate.evaluate(third_candidate);
  EXPECT_TRUE(decision.accepted);
  EXPECT_TRUE(decision.stream_discontinuity);
}

TEST(PicoTeleopStreamGate, ReturningToAcceptedAnchorCancelsCandidate) {
  PicoTeleopStreamGate gate(0.15, 0.60);
  ASSERT_TRUE(gate.evaluate(makeFrame(1)).accepted);

  PicoTeleopFrame jump = makeFrame(2);
  jump.left.position.x() = 0.30;
  EXPECT_FALSE(gate.evaluate(jump).accepted);

  PicoTeleopFrame recovered = makeFrame(3);
  recovered.left.position.x() = 0.01;
  const PicoStreamDecision decision = gate.evaluate(recovered);
  EXPECT_TRUE(decision.accepted);
  EXPECT_FALSE(decision.epoch_changed);
  EXPECT_FALSE(decision.stream_discontinuity);
}

TEST(PicoTeleopStreamGate, RejectedCandidatesStillAdvanceOrdering) {
  PicoTeleopStreamGate gate(0.15, 0.60);
  ASSERT_TRUE(gate.evaluate(makeFrame(1)).accepted);

  PicoTeleopFrame jump = makeFrame(3);
  jump.left.position.x() = 0.30;
  EXPECT_FALSE(gate.evaluate(jump).accepted);

  PicoTeleopFrame delayed = makeFrame(2);
  delayed.left.position.x() = 0.31;
  const PicoStreamDecision decision = gate.evaluate(delayed);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, PicoStreamRejectReason::kOutOfOrder);
}

TEST(PicoTeleopStreamGate, NewEpochClearsPendingCandidate) {
  PicoTeleopStreamGate gate(0.15, 0.60);
  ASSERT_TRUE(gate.evaluate(makeFrame(1, 9)).accepted);

  PicoTeleopFrame candidate = makeFrame(2, 9);
  candidate.left.position.x() = 0.30;
  EXPECT_FALSE(gate.evaluate(candidate).accepted);

  PicoTeleopFrame new_epoch = makeFrame(1, 10);
  new_epoch.left.position.x() = 1.0;
  const PicoStreamDecision transition = gate.evaluate(new_epoch);
  EXPECT_TRUE(transition.accepted);
  EXPECT_TRUE(transition.epoch_changed);
  EXPECT_FALSE(transition.stream_discontinuity);

  PicoTeleopFrame next = makeFrame(2, 10);
  next.left.position.x() = 1.01;
  const PicoStreamDecision normal = gate.evaluate(next);
  EXPECT_TRUE(normal.accepted);
  EXPECT_FALSE(normal.stream_discontinuity);
}

TEST(PicoTeleopStreamGate, AcceptsOnlyMonotonicallyNewerEpochs) {
  PicoTeleopStreamGate gate(0.15, 0.60);
  ASSERT_TRUE(gate.evaluate(makeFrame(10, 9)).accepted);

  PicoTeleopFrame new_epoch = makeFrame(1, 10);
  new_epoch.left.position.x() = 5.0;
  const PicoStreamDecision transition = gate.evaluate(new_epoch);
  EXPECT_TRUE(transition.accepted);
  EXPECT_TRUE(transition.epoch_changed);

  const PicoStreamDecision rollback = gate.evaluate(makeFrame(11, 9));
  EXPECT_FALSE(rollback.accepted);
  EXPECT_EQ(rollback.reason, PicoStreamRejectReason::kEpochRollback);

  PicoTeleopFrame zero_epoch = makeFrame(12, 0);
  const PicoStreamDecision zero = gate.evaluate(zero_epoch);
  EXPECT_FALSE(zero.accepted);
  EXPECT_EQ(zero.reason, PicoStreamRejectReason::kZeroEpoch);
}

TEST(PicoTeleopStreamGate, ResetForgetsAcceptedStream) {
  PicoTeleopStreamGate gate(0.15, 0.60);
  ASSERT_TRUE(gate.evaluate(makeFrame(10, 9)).accepted);
  PicoTeleopFrame candidate = makeFrame(11, 9);
  candidate.left.position.x() = 0.30;
  ASSERT_FALSE(gate.evaluate(candidate).accepted);
  gate.reset();

  const PicoStreamDecision restarted = gate.evaluate(makeFrame(1, 1));
  EXPECT_TRUE(restarted.accepted);
  EXPECT_TRUE(restarted.epoch_changed);
  EXPECT_FALSE(restarted.stream_discontinuity);
}

}  // namespace
}  // namespace tianji_qp_ik
