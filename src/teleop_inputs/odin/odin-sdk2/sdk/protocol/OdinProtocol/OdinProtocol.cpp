#include "OdinProtocol.hpp"
#include <cstring>

namespace odin {
namespace sdk {

bool OdinProtocolV1::Pack(const IPacket& packet, std::vector<uint8_t>& output) {
  const size_t payload_size = packet.GetPayloadSize();
  const size_t total_size = OdinConst::kHeaderSize + payload_size;

  output.resize(total_size);
  uint8_t* buf = output.data();

  // Build header
  buf[0] = OdinConst::kSof;
  buf[1] = static_cast<uint8_t>(packet.GetAttribute(PacketAttr::kVersion));
  if (buf[1] == 0) buf[1] = OdinConst::kVersion;

  // Length = total frame size (header + payload), little-endian
  buf[2] = static_cast<uint8_t>(total_size & 0xFF);
  buf[3] = static_cast<uint8_t>((total_size >> 8) & 0xFF);

  // Sequence (little-endian, uint32_t)
  uint32_t seq = packet.GetSequence();
  buf[4] = static_cast<uint8_t>(seq & 0xFF);
  buf[5] = static_cast<uint8_t>((seq >> 8) & 0xFF);
  buf[6] = static_cast<uint8_t>((seq >> 16) & 0xFF);
  buf[7] = static_cast<uint8_t>((seq >> 24) & 0xFF);

  // Command ID (little-endian)
  uint16_t cmd_id = static_cast<uint16_t>(packet.GetCommandId());
  buf[8] = static_cast<uint8_t>(cmd_id & 0xFF);
  buf[9] = static_cast<uint8_t>((cmd_id >> 8) & 0xFF);

  // Command type and send type
  buf[10] = static_cast<uint8_t>(packet.GetAttribute(PacketAttr::kCmdType));
  buf[11] = static_cast<uint8_t>(packet.GetAttribute(PacketAttr::kSendType));

  // CRC16 over first 12 bytes
  uint16_t crc16 = crc16_.ccitt(buf, OdinConst::kCrc16EndOffset);
  buf[12] = static_cast<uint8_t>(crc16 & 0xFF);
  buf[13] = static_cast<uint8_t>((crc16 >> 8) & 0xFF);

  // Copy payload
  if (payload_size > 0 && packet.GetPayload()) {
    std::memcpy(buf + OdinConst::kHeaderSize, packet.GetPayload(), payload_size);
  }

  // CRC32 over payload (or 0 if no payload)
  uint32_t crc32 = 0;
  if (payload_size > 0) {
    crc32 = crc32_.crc32(buf + OdinConst::kHeaderSize, payload_size);
  }
  buf[14] = static_cast<uint8_t>(crc32 & 0xFF);
  buf[15] = static_cast<uint8_t>((crc32 >> 8) & 0xFF);
  buf[16] = static_cast<uint8_t>((crc32 >> 16) & 0xFF);
  buf[17] = static_cast<uint8_t>((crc32 >> 24) & 0xFF);

  return true;
}

int OdinProtocolV1::Parse(const uint8_t* buffer, size_t length, std::unique_ptr<IPacket>& packet) {
  if (!buffer || length < OdinConst::kHeaderSize) {
    return 0;  // Incomplete
  }

  // Check SOF
  if (buffer[0] != OdinConst::kSof) {
    return -1;  // Invalid frame
  }

  // Parse header fields (little-endian)
  uint8_t version = buffer[1];
  // Length field is total frame size (header + payload)
  uint16_t total_len = static_cast<uint16_t>(buffer[2]) | (static_cast<uint16_t>(buffer[3]) << 8);
  uint32_t seq = static_cast<uint32_t>(buffer[4]) | (static_cast<uint32_t>(buffer[5]) << 8) |
                 (static_cast<uint32_t>(buffer[6]) << 16) | (static_cast<uint32_t>(buffer[7]) << 24);
  uint16_t cmd_id = static_cast<uint16_t>(buffer[8]) | (static_cast<uint16_t>(buffer[9]) << 8);
  uint8_t cmd_type = buffer[10];
  uint8_t send_type = buffer[11];
  uint16_t header_crc16 =
      static_cast<uint16_t>(buffer[12]) | (static_cast<uint16_t>(buffer[13]) << 8);
  uint32_t header_crc32 =
      static_cast<uint32_t>(buffer[14]) | (static_cast<uint32_t>(buffer[15]) << 8) |
      (static_cast<uint32_t>(buffer[16]) << 16) | (static_cast<uint32_t>(buffer[17]) << 24);

  // Validate total length
  if (total_len < OdinConst::kHeaderSize) {
    return -1;  // Invalid length
  }
  const size_t total_size = total_len;
  const size_t payload_len = total_len - OdinConst::kHeaderSize;
  if (length < total_size) {
    return 0;  // Incomplete, need more data
  }

  // Verify CRC16
  uint16_t calc_crc16 = crc16_.ccitt(buffer, OdinConst::kCrc16EndOffset);
  if (calc_crc16 != header_crc16) {
    return -1;  // CRC16 mismatch
  }

  // Verify CRC32 if payload exists
  if (payload_len > 0) {
    uint32_t calc_crc32 = crc32_.crc32(buffer + OdinConst::kHeaderSize, payload_len);
    if (calc_crc32 != header_crc32) {
      return -1;  // CRC32 mismatch
    }
  }

  // Create packet
  std::unique_ptr<OdinPacket> odin_pkt(new OdinPacket());
  odin_pkt->SetCommandId(cmd_id);
  odin_pkt->SetSequence(seq);
  odin_pkt->SetAttribute(PacketAttr::kVersion, version);
  odin_pkt->SetAttribute(PacketAttr::kCmdType, cmd_type);
  odin_pkt->SetAttribute(PacketAttr::kSendType, send_type);
  odin_pkt->SetAttribute(PacketAttr::kCrc16, header_crc16);
  odin_pkt->SetAttribute(PacketAttr::kCrc32, header_crc32);

  if (payload_len > 0) {
    odin_pkt->SetPayload(buffer + OdinConst::kHeaderSize, payload_len);
  }

  packet = std::move(odin_pkt);
  return static_cast<int>(total_size);
}

}  // namespace sdk
}  // namespace odin
