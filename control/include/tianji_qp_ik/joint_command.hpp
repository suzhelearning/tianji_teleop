#pragma once

#include "tianji_qp_ik/types.hpp"
#include "tianji_qp_ik/wuji_hand_teleop_protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <netinet/in.h>

namespace tianji_qp_ik {

inline constexpr std::size_t kJointCommandPacketSize = 468U;
inline constexpr std::uint8_t kJointCommandPacketVersion = 2U;
inline constexpr std::uint8_t kJointCommandArmsReadyFlag = 1U;
inline constexpr std::uint8_t kJointCommandRightHandReadyFlag = 2U;
inline constexpr std::uint8_t kJointCommandLeftHandReadyFlag = 4U;
inline constexpr std::uint8_t kJointCommandKnownFlags =
    kJointCommandArmsReadyFlag | kJointCommandRightHandReadyFlag |
    kJointCommandLeftHandReadyFlag;
using JointCommandPacket = std::array<std::uint8_t, kJointCommandPacketSize>;

struct JointCommandFrame {
  std::uint8_t flags{0U};
  std::uint64_t sequence{0U};
  std::int64_t source_timestamp_ns{0};
  std::uint64_t pico_tracking_epoch{0U};
  // Left arm 7, right arm 7, left hand 20, right hand 20 in TJH2 joint order.
  std::array<double, 54U> position_rad{};
};

enum class JointCommandPacketError {
  kNone,
  kWrongSize,
  kWrongMagic,
  kWrongVersion,
  kWrongDeclaredSize,
  kInvalidFlags,
  kCrcMismatch,
  kInvalidMetadata,
  kNonFiniteJointPosition,
};

struct JointCommandDecodeResult {
  JointCommandPacketError error{JointCommandPacketError::kWrongSize};
  std::optional<JointCommandFrame> frame;
};

JointCommandPacket encodeJointCommandPacket(const JointCommandFrame& frame);
JointCommandDecodeResult decodeJointCommandPacket(
    const std::uint8_t* bytes, std::size_t size) noexcept;

// Bridge and exporter must share the local Linux monotonic clock. Receiver
// arrival time alone cannot establish freshness of a delayed datagram.
bool jointCommandPicoBridgeFresh(std::int64_t applied_bridge_send_monotonic_ns,
                                 std::int64_t now_monotonic_ns) noexcept;

// Tracks one applied side, not receiver-global traffic or cached model state.
class HandCommandFreshness {
 public:
  explicit HandCommandFreshness(ArmSide side) noexcept : side_(side) {}
  void observe(const WujiHandTeleopFrame& frame) noexcept;
  bool live(std::int64_t now_monotonic_ns) const noexcept;
  // Preserve source/sequence watermarks: repeats cannot re-arm after reset.
  void reset() noexcept { receive_monotonic_ns_ = 0; }

 private:
  const ArmSide side_;
  std::uint64_t sequence_{0U};
  std::int64_t source_timestamp_ns_{0};
  std::int64_t publication_timestamp_ns_{0};
  std::int64_t receive_monotonic_ns_{0};
};

// Once arms have been offered to an executor, a controller reset permanently
// inhibits them. A transient disabled UDP datagram is not a reliable latch.
class JointCommandArmReadiness {
 public:
  bool update(bool ready, bool reset) noexcept;

 private:
  bool ready_seen_{false};
  bool inhibited_{false};
};

// Socket preparation is explicit and loopback-only. send() never allocates on
// its success path; any failure aborts export rather than disguising a fault.
class JointCommandExporter {
 public:
  JointCommandExporter(const std::string& host, std::uint16_t port);
  ~JointCommandExporter();
  JointCommandExporter(const JointCommandExporter&) = delete;
  JointCommandExporter& operator=(const JointCommandExporter&) = delete;
  void send(const JointCommandFrame& frame);

 private:
  int socket_fd_{-1};
  sockaddr_in destination_{};
};

}  // namespace tianji_qp_ik
