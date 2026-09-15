#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "OdinFrameAssembler.hpp"
#include "logger.h"

namespace odin::sdk {

class OdinFrameAssemblerTestPeer {
 public:
  static size_t raw_payload_capacity(const OdinFrameAssembler & assembler) {
    return assembler.raw_point_state_.packet.payload.capacity();
  }
};

}  // namespace odin::sdk

namespace {

std::vector<uint8_t> raw_fragment(uint16_t udp_count, uint32_t frame_count) {
  using odin::sdk::OdinDataFrameHeader;
  using odin::sdk::OdinRawPoint;

  OdinDataFrameHeader header;
  header.version = 1;
  header.length = static_cast<uint16_t>(
    sizeof(OdinDataFrameHeader) + sizeof(OdinRawPoint<uint16_t>));
  header.dot_or_sample_count = 1;
  header.udp_count = udp_count;
  header.frame_count = frame_count;

  std::vector<uint8_t> wire(header.length, 0);
  std::memcpy(wire.data(), &header, sizeof(header));
  return wire;
}

TEST(OdinFrameAssembler, DropsPacketGapLargerThanOnePointCloudFrame) {
  ASSERT_EQ(log_config(LOG_LEVEL_WARN, std::printf), 0);
  odin::sdk::OdinFrameAssembler assembler(1, false);
  size_t callback_count = 0;
  assembler.SetPointCloudCallback(
    [&callback_count](const odin::sdk::OdinPointCloudPacket &) {
      ++callback_count;
    });

  const auto first = raw_fragment(1, 7);
  const auto impossible_gap = raw_fragment(400, 7);
  assembler.ProcessRawPointData(first.data(), first.size());
  assembler.ProcessRawPointData(impossible_gap.data(), impossible_gap.size());

  EXPECT_EQ(callback_count, 0U);
}

TEST(OdinFrameAssembler, PointFrameReserveDoesNotExceedLogicalPayloadLimit) {
  using odin::sdk::OdinRawPoint;

  odin::sdk::OdinFrameAssembler assembler(1, false);
  const auto fragment = raw_fragment(1, 8);
  assembler.ProcessRawPointData(fragment.data(), fragment.size());

  constexpr size_t kLogicalPayloadLimit =
    256U * 192U * sizeof(OdinRawPoint<uint16_t>);
  const size_t reserved_capacity =
    odin::sdk::OdinFrameAssemblerTestPeer::raw_payload_capacity(assembler);
  ASSERT_NE(reserved_capacity, 0U);
  EXPECT_LE(reserved_capacity, kLogicalPayloadLimit);
}

}  // namespace
