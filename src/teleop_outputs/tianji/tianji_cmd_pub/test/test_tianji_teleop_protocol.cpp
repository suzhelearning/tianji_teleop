#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "pico_bridge/tianji_teleop_protocol.hpp"

namespace pb = pico_bridge;

namespace {

pb::PicoSkeletonFrame neutralSkeleton()
{
  pb::PicoSkeletonFrame frame{};
  for (auto & pose : frame) {
    pose.orientation = Eigen::Quaterniond::Identity();
  }
  frame[pb::kPicoSpine2].position = {0.0, 0.0, 1.0};
  frame[pb::kPicoLeftShoulder].position = {0.0, 0.2, 1.5};
  frame[pb::kPicoRightShoulder].position = {0.0, -0.2, 1.5};
  frame[pb::kPicoLeftElbow].position = {0.25, 0.2, 1.2};
  frame[pb::kPicoRightElbow].position = {0.25, -0.2, 1.2};
  frame[pb::kPicoLeftWrist].position = {0.5, 0.2, 1.5};
  frame[pb::kPicoRightWrist].position = {0.5, -0.2, 1.5};
  frame[pb::kPicoLeftHand].position = {0.5, 0.4, 1.4};
  frame[pb::kPicoRightHand].position = {0.5, -0.4, 1.4};
  return frame;
}

std::string statusJson(std::int64_t stamp, std::uint64_t epoch = 9)
{
  return "{\"tracking_epoch\":" + std::to_string(epoch) +
    ",\"stream_valid\":true,\"source_frame_id\":\"pico\""
    ",\"source_stamp_ns\":" + std::to_string(stamp) +
    ",\"ik_frame_valid\":true,\"left\":{\"corrected\":true}"
    ",\"right\":{\"corrected\":true}}";
}

template<typename Integer>
Integer readLittleEndian(
  const std::array<std::uint8_t, pb::kTianjiTeleopPacketSize> & packet,
  std::size_t offset)
{
  using Unsigned = std::make_unsigned_t<Integer>;
  Unsigned value = 0;
  for (std::size_t byte = 0; byte < sizeof(Integer); ++byte) {
    value |= static_cast<Unsigned>(packet[offset + byte]) << (8U * byte);
  }
  return static_cast<Integer>(value);
}

double readDouble(
  const std::array<std::uint8_t, pb::kTianjiTeleopPacketSize> & packet,
  std::size_t offset)
{
  const std::uint64_t bits = readLittleEndian<std::uint64_t>(packet, offset);
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

TEST(TianjiTeleopProtocol, ParsesStrictCorrectedIkStatus)
{
  const auto status = pb::parse_corrected_ik_status(statusJson(11));

  EXPECT_TRUE(status.valid) << status.rejection_reason;
  EXPECT_EQ(status.source_stamp_ns, 11);
  EXPECT_EQ(status.tracking_epoch, 9U);
}

TEST(TianjiTeleopProtocol, RejectsMalformedAndIncompleteStatus)
{
  struct Case
  {
    std::string json;
    const char * reason;
  };
  const std::vector<Case> cases{
    {"{", "status_json_malformed"},
    {statusJson(11, 0), "tracking_epoch_invalid"},
    {"{\"tracking_epoch\":9,\"stream_valid\":true,"
     "\"source_frame_id\":\"world\",\"source_stamp_ns\":11,"
     "\"ik_frame_valid\":true,\"left\":{\"corrected\":true},"
     "\"right\":{\"corrected\":true}}", "source_frame_id_invalid"},
    {"{\"tracking_epoch\":9,\"stream_valid\":false,"
     "\"source_frame_id\":\"pico\",\"source_stamp_ns\":11,"
     "\"ik_frame_valid\":true,\"left\":{\"corrected\":true},"
     "\"right\":{\"corrected\":true}}", "stream_invalid"},
    {"{\"tracking_epoch\":9,\"stream_valid\":true,"
     "\"source_frame_id\":\"pico\",\"source_stamp_ns\":0,"
     "\"ik_frame_valid\":true,\"left\":{\"corrected\":true},"
     "\"right\":{\"corrected\":true}}", "source_stamp_invalid"},
    {"{\"tracking_epoch\":9,\"stream_valid\":true,"
     "\"source_frame_id\":\"pico\",\"source_stamp_ns\":11,"
     "\"ik_frame_valid\":false,\"left\":{\"corrected\":true},"
     "\"right\":{\"corrected\":true}}", "ik_frame_invalid"},
  };

  for (const auto & test_case : cases) {
    const auto status = pb::parse_corrected_ik_status(test_case.json);
    EXPECT_FALSE(status.valid) << test_case.reason;
    EXPECT_EQ(status.rejection_reason, test_case.reason);
  }
}

TEST(TianjiTeleopProtocol, PairsSkeletonThenStatusExactlyOnce)
{
  pb::TianjiTeleopBridgeCore core;
  auto outcome = core.ingest_skeleton(11, neutralSkeleton());
  EXPECT_FALSE(outcome.frame.has_value());
  EXPECT_EQ(outcome.rejection_reason, "awaiting_status");

  outcome = core.ingest_status(pb::parse_corrected_ik_status(statusJson(11)));
  ASSERT_TRUE(outcome.frame.has_value()) << outcome.rejection_reason;
  EXPECT_EQ(outcome.frame->sequence, 1U);
  EXPECT_EQ(outcome.frame->tracking_epoch, 9U);
  EXPECT_EQ(outcome.frame->source_timestamp_ns, 11);
  const auto expected = pb::map_pico_palms_to_tianji(neutralSkeleton());
  ASSERT_TRUE(expected.valid) << expected.rejection_reason;
  EXPECT_TRUE(outcome.frame->left_target.isApprox(expected.left_target, 1e-12));
  EXPECT_TRUE(outcome.frame->right_target.isApprox(expected.right_target, 1e-12));
  ASSERT_TRUE(outcome.frame->left_arm_direction.valid);
  ASSERT_TRUE(outcome.frame->right_arm_direction.valid);
  EXPECT_TRUE(outcome.frame->left_arm_direction.direction.isApprox(
    expected.left_arm_direction.direction, 1e-12));
  EXPECT_TRUE(outcome.frame->right_arm_direction.direction.isApprox(
    expected.right_arm_direction.direction, 1e-12));

  outcome = core.ingest_status(pb::parse_corrected_ik_status(statusJson(11)));
  EXPECT_FALSE(outcome.frame.has_value());
  EXPECT_EQ(outcome.rejection_reason, "duplicate_source_stamp");
}

TEST(TianjiTeleopProtocol, PassesConfiguredPicoWorldXOffsetToGeometry)
{
  pb::PicoPositionRetargetingConfig config;
  config.mode = pb::PicoPositionRetargetingMode::kPicoPalm;
  config.pico_world_x_offset_m = 0.25;
  pb::TianjiTeleopBridgeCore core(8, config);
  (void)core.ingest_skeleton(12, neutralSkeleton());

  const auto outcome = core.ingest_status(
    pb::parse_corrected_ik_status(statusJson(12)));

  ASSERT_TRUE(outcome.frame.has_value()) << outcome.rejection_reason;
  EXPECT_TRUE(outcome.frame->left_target.translation().isApprox(
    Eigen::Vector3d(0.75, 0.4, 1.021), 1e-12));
  EXPECT_TRUE(outcome.frame->right_target.translation().isApprox(
    Eigen::Vector3d(0.75, -0.4, 1.021), 1e-12));
}

TEST(TianjiTeleopProtocol, PassesConfiguredRobotArmReachScaleToGeometry)
{
  pb::PicoPositionRetargetingConfig config;
  config.robot_arm_reach_scale = 1.0;
  pb::TianjiTeleopBridgeCore core(8, config);
  (void)core.ingest_skeleton(13, neutralSkeleton());

  const auto outcome = core.ingest_status(
    pb::parse_corrected_ik_status(statusJson(13)));
  const auto expected = pb::map_pico_palms_to_tianji(neutralSkeleton(), config);

  ASSERT_TRUE(outcome.frame.has_value()) << outcome.rejection_reason;
  ASSERT_TRUE(expected.valid) << expected.rejection_reason;
  EXPECT_TRUE(outcome.frame->left_target.isApprox(expected.left_target, 1e-12));
  EXPECT_TRUE(outcome.frame->right_target.isApprox(expected.right_target, 1e-12));
}

TEST(TianjiTeleopProtocol, PairsStatusThenSkeletonExactlyOnce)
{
  pb::TianjiTeleopBridgeCore core;
  auto outcome = core.ingest_status(pb::parse_corrected_ik_status(statusJson(21)));
  EXPECT_FALSE(outcome.frame.has_value());
  EXPECT_EQ(outcome.rejection_reason, "awaiting_skeleton");

  outcome = core.ingest_skeleton(21, neutralSkeleton());
  ASSERT_TRUE(outcome.frame.has_value()) << outcome.rejection_reason;
  EXPECT_EQ(outcome.frame->sequence, 1U);
  EXPECT_EQ(outcome.frame->source_timestamp_ns, 21);
}

TEST(TianjiTeleopProtocol, RecoversAfterLargeSourceTimestampRollback)
{
  pb::TianjiTeleopBridgeCore core;

  auto outcome = core.ingest_skeleton(2'000'000'000LL, neutralSkeleton());
  EXPECT_FALSE(outcome.frame.has_value());
  outcome = core.ingest_status(
    pb::parse_corrected_ik_status(statusJson(2'000'000'000LL, 9U)));
  ASSERT_TRUE(outcome.frame.has_value()) << outcome.rejection_reason;
  EXPECT_EQ(outcome.frame->sequence, 1U);

  outcome = core.ingest_skeleton(100'000'000LL, neutralSkeleton());
  EXPECT_FALSE(outcome.frame.has_value());
  outcome = core.ingest_status(
    pb::parse_corrected_ik_status(statusJson(100'000'000LL, 10U)));
  ASSERT_TRUE(outcome.frame.has_value()) << outcome.rejection_reason;
  EXPECT_EQ(outcome.frame->sequence, 2U);
  EXPECT_EQ(outcome.frame->tracking_epoch, 10U);
}

TEST(TianjiTeleopProtocol, MatchesExistingStatusBeforeSkeletonCacheEviction)
{
  pb::TianjiTeleopBridgeCore core(2);
  (void)core.ingest_status(pb::parse_corrected_ik_status(statusJson(1)));
  (void)core.ingest_skeleton(2, neutralSkeleton());
  (void)core.ingest_skeleton(3, neutralSkeleton());

  const auto outcome = core.ingest_skeleton(1, neutralSkeleton());

  ASSERT_TRUE(outcome.frame.has_value()) << outcome.rejection_reason;
  EXPECT_EQ(outcome.frame->source_timestamp_ns, 1);
}

TEST(TianjiTeleopProtocol, RejectsInvalidStatusAndEvictedSkeleton)
{
  pb::TianjiTeleopBridgeCore core(2);
  auto invalid = pb::parse_corrected_ik_status(statusJson(1, 0));
  auto outcome = core.ingest_status(invalid);
  EXPECT_FALSE(outcome.frame.has_value());
  EXPECT_EQ(outcome.rejection_reason, "tracking_epoch_invalid");

  (void)core.ingest_skeleton(1, neutralSkeleton());
  (void)core.ingest_skeleton(2, neutralSkeleton());
  (void)core.ingest_skeleton(3, neutralSkeleton());
  outcome = core.ingest_status(pb::parse_corrected_ik_status(statusJson(1)));
  EXPECT_FALSE(outcome.frame.has_value());
  EXPECT_EQ(outcome.rejection_reason, "skeleton_cache_miss");
}

TEST(TianjiTeleopProtocol, RejectsMappedGeometryAsAnAtomicFrame)
{
  pb::TianjiTeleopBridgeCore core;
  auto invalid = neutralSkeleton();
  invalid[pb::kPicoLeftShoulder].position.y() = 0.01;
  invalid[pb::kPicoRightShoulder].position.y() = -0.01;
  (void)core.ingest_skeleton(31, invalid);

  const auto outcome = core.ingest_status(
    pb::parse_corrected_ik_status(statusJson(31)));

  EXPECT_FALSE(outcome.frame.has_value());
  EXPECT_EQ(outcome.rejection_reason, "shoulder_separation_out_of_range");
}

TEST(TianjiTeleopProtocol, EncodesV4ArmDirectionsSkeletonRotationsAndCrc)
{
  pb::TianjiTeleopWireFrame frame;
  frame.sequence = 7;
  frame.tracking_epoch = 9;
  frame.source_timestamp_ns = 11;
  frame.bridge_send_monotonic_ns = 13;
  frame.left_target.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  frame.left_target.linear() = Eigen::Matrix3d::Identity();
  frame.right_target.translation() = Eigen::Vector3d(-1.0, -2.0, -3.0);
  frame.right_target.linear() = Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)
    .toRotationMatrix();
  frame.left_arm_direction.valid = true;
  frame.left_arm_direction.direction = Eigen::Vector3d::UnitX();
  frame.right_arm_direction.valid = true;
  frame.right_arm_direction.direction = -Eigen::Vector3d::UnitZ();
  frame.upper_limb_skeleton.valid = true;
  for (std::size_t index = 0; index < frame.upper_limb_skeleton.points.size();
    ++index)
  {
    frame.upper_limb_skeleton.points[index] = Eigen::Vector3d(
      static_cast<double>(index) + 0.1,
      static_cast<double>(index) + 0.2,
      static_cast<double>(index) + 0.3);
    frame.upper_limb_skeleton.rotations[index] = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.1 * static_cast<double>(index + 1U),
                        Eigen::Vector3d::UnitZ()));
  }
  frame.upper_limb_skeleton.rotations_valid = true;

  const auto encoded = pb::encode_tianji_teleop_packet(frame);

  ASSERT_EQ(encoded.size(), 656U);
  EXPECT_EQ(encoded[0], static_cast<std::uint8_t>('T'));
  EXPECT_EQ(encoded[1], static_cast<std::uint8_t>('J'));
  EXPECT_EQ(encoded[2], static_cast<std::uint8_t>('V'));
  EXPECT_EQ(encoded[3], static_cast<std::uint8_t>('R'));
  EXPECT_EQ(readLittleEndian<std::uint16_t>(encoded, 4U), 4U);
  EXPECT_EQ(readLittleEndian<std::uint16_t>(encoded, 6U), 656U);
  EXPECT_EQ(readLittleEndian<std::uint32_t>(encoded, 40U), 0xffU);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 156U), 1.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 164U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 172U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 180U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 188U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 196U), -1.0);
  for (std::size_t index = 0; index < 8U; ++index) {
    EXPECT_DOUBLE_EQ(readDouble(encoded, 204U + index * 24U),
      static_cast<double>(index) + 0.1);
    EXPECT_DOUBLE_EQ(readDouble(encoded, 212U + index * 24U),
      static_cast<double>(index) + 0.2);
    EXPECT_DOUBLE_EQ(readDouble(encoded, 220U + index * 24U),
      static_cast<double>(index) + 0.3);
    const Eigen::Quaterniond expected =
      frame.upper_limb_skeleton.rotations[index].normalized();
    EXPECT_DOUBLE_EQ(readDouble(encoded, 396U + index * 32U), expected.x());
    EXPECT_DOUBLE_EQ(readDouble(encoded, 404U + index * 32U), expected.y());
    EXPECT_DOUBLE_EQ(readDouble(encoded, 412U + index * 32U), expected.z());
    EXPECT_DOUBLE_EQ(readDouble(encoded, 420U + index * 32U), expected.w());
  }
  EXPECT_EQ(
    readLittleEndian<std::uint32_t>(encoded, 652U),
    pb::tianji_teleop_crc32(encoded.data(), 652U));
}

TEST(TianjiTeleopProtocol, DoesNotAdvertiseRotationsWithoutSkeletonPositions)
{
  pb::TianjiTeleopWireFrame frame;
  frame.upper_limb_skeleton.rotations_valid = true;

  const auto encoded = pb::encode_tianji_teleop_packet(frame);

  EXPECT_EQ(
    readLittleEndian<std::uint32_t>(encoded, 40U) &
    pb::kTianjiTeleopUpperLimbRotationsValid,
    0U);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 396U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 404U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 412U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 420U), 1.0);
}

TEST(TianjiTeleopProtocol, DoesNotAdvertiseInvalidRotationPayload)
{
  pb::TianjiTeleopWireFrame frame;
  frame.upper_limb_skeleton.valid = true;
  frame.upper_limb_skeleton.rotations_valid = true;
  frame.upper_limb_skeleton.rotations[3].x() =
    std::numeric_limits<double>::quiet_NaN();

  const auto encoded = pb::encode_tianji_teleop_packet(frame);

  EXPECT_EQ(
    readLittleEndian<std::uint32_t>(encoded, 40U) &
    pb::kTianjiTeleopUpperLimbRotationsValid,
    0U);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 492U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 500U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 508U), 0.0);
  EXPECT_DOUBLE_EQ(readDouble(encoded, 516U), 1.0);
}
