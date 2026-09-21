#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace odin {
namespace sdk {

/* ============================================================================
 * Abstract Packet Interface
 * ============================================================================
 *
 * Design Goals:
 *   - Protocol-agnostic packet representation
 *   - Easy migration from existing OdinCommandPacket
 *   - Extensible for future protocol versions
 */

/**
 * @brief Abstract packet interface
 * @note All protocol-specific packets should inherit from this
 */
class IPacket {
 public:
  virtual ~IPacket() = default;

  // Core identifiers (protocol-agnostic)
  virtual uint32_t GetCommandId() const = 0;
  virtual uint32_t GetSequence() const = 0;

  // Payload access
  virtual const uint8_t* GetPayload() const = 0;
  virtual size_t GetPayloadSize() const = 0;

  // For building packets
  virtual void SetCommandId(uint32_t id) = 0;
  virtual void SetSequence(uint32_t seq) = 0;
  virtual void SetPayload(const uint8_t* data, size_t size) = 0;

  // Protocol-specific attributes (optional implementation)
  virtual uint32_t GetAttribute(uint32_t key) const { return 0; }
  virtual void SetAttribute(uint32_t key, uint32_t value) {}

  // Clone for polymorphic copy
  virtual std::unique_ptr<IPacket> Clone() const = 0;
};

/* ============================================================================
 * Abstract Protocol Interface
 * ============================================================================ */

/**
 * @brief Abstract protocol encoder/decoder interface
 * @note Inherit this for different protocol implementations (V1, V2, etc.)
 */
class IProtocol {
 public:
  virtual ~IProtocol() = default;

  /**
   * @brief Pack a packet into wire format
   * @param packet  Packet to serialize
   * @param output  Output buffer (appended)
   * @return true on success
   */
  virtual bool Pack(const IPacket& packet, std::vector<uint8_t>& output) = 0;

  /**
   * @brief Parse raw data into a packet
   * @param buffer  Raw input data
   * @param length  Buffer length
   * @param packet  Output packet (created by protocol)
   * @return Bytes consumed (>0), 0 if incomplete, -1 on error
   */
  virtual int Parse(const uint8_t* buffer, size_t length, std::unique_ptr<IPacket>& packet) = 0;

  /**
   * @brief Create an empty packet for this protocol
   * @return New packet instance
   */
  virtual std::unique_ptr<IPacket> CreatePacket() = 0;

  // Protocol metadata
  virtual size_t GetHeaderSize() const = 0;
  virtual const char* GetProtocolName() const = 0;
  virtual uint32_t GetProtocolVersion() const = 0;
};

/* ============================================================================
 * Attribute Keys for IPacket::GetAttribute/SetAttribute
 * ============================================================================ */
namespace PacketAttr {
// Attribute keys
constexpr uint32_t kCmdType = 0x01;   // Command type (req/ack)
constexpr uint32_t kSendType = 0x02;  // Sender type (host/device)
constexpr uint32_t kVersion = 0x03;   // Protocol version
constexpr uint32_t kCrc16 = 0x10;     // CRC16 checksum
constexpr uint32_t kCrc32 = 0x11;     // CRC32 checksum

// Common attribute values
constexpr uint32_t kCmdTypeReq = 0x00;      // Request
constexpr uint32_t kCmdTypeAck = 0x01;      // Acknowledgement
constexpr uint32_t kSendTypeHost = 0x00;    // From host
constexpr uint32_t kSendTypeDevice = 0x01;  // From device
}  // namespace PacketAttr

}  // namespace sdk
}  // namespace odin
