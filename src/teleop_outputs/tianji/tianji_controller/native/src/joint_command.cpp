#include "tianji_qp_ik/joint_command.hpp"
#include "tianji_qp_ik/crc32.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace tianji_qp_ik {
namespace {

constexpr std::size_t kCrcOffset = kJointCommandPacketSize - sizeof(std::uint32_t);
static_assert(sizeof(double) == 8U && std::numeric_limits<double>::is_iec559);

template <typename UInt>
UInt readLe(const std::uint8_t* bytes) noexcept {
  std::uint64_t value = 0U;
  for (std::size_t i = 0U; i < sizeof(UInt); ++i) {
    value |= static_cast<std::uint64_t>(bytes[i]) << (8U * i);
  }
  return static_cast<UInt>(value);
}

template <typename UInt>
void writeLe(std::uint8_t* bytes, UInt value) noexcept {
  for (std::size_t i = 0U; i < sizeof(UInt); ++i) {
    bytes[i] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xffU);
  }
}

JointCommandPacketError validate(const JointCommandFrame& frame) noexcept {
  if ((frame.flags & ~kJointCommandKnownFlags) != 0U) {
    return JointCommandPacketError::kInvalidFlags;
  }
  if (frame.sequence == 0U || frame.source_timestamp_ns <= 0) {
    return JointCommandPacketError::kInvalidMetadata;
  }
  for (double value : frame.position_rad) {
    if (!std::isfinite(value)) {
      return JointCommandPacketError::kNonFiniteJointPosition;
    }
  }
  return JointCommandPacketError::kNone;
}

}  // namespace

JointCommandPacket encodeJointCommandPacket(const JointCommandFrame& frame) {
  if (validate(frame) != JointCommandPacketError::kNone) {
    throw std::invalid_argument("invalid TJRC joint command metadata or positions");
  }
  JointCommandPacket packet{};
  std::memcpy(packet.data(), "TJRC", 4U);
  packet[4] = kJointCommandPacketVersion;
  packet[5] = frame.flags;
  writeLe(packet.data() + 6U, static_cast<std::uint16_t>(packet.size()));
  writeLe(packet.data() + 8U, frame.sequence);
  writeLe(packet.data() + 16U,
          static_cast<std::uint64_t>(frame.source_timestamp_ns));
  writeLe(packet.data() + 24U, frame.pico_tracking_epoch);
  for (std::size_t i = 0U; i < frame.position_rad.size(); ++i) {
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &frame.position_rad[i], sizeof(bits));
    writeLe(packet.data() + 32U + 8U * i, bits);
  }
  writeLe(packet.data() + kCrcOffset, crc32(packet.data(), kCrcOffset));
  return packet;
}

JointCommandDecodeResult decodeJointCommandPacket(
    const std::uint8_t* bytes, std::size_t size) noexcept {
  JointCommandDecodeResult result;
  if (bytes == nullptr || size != kJointCommandPacketSize) {
    return result;
  }
  if (std::memcmp(bytes, "TJRC", 4U) != 0) {
    result.error = JointCommandPacketError::kWrongMagic;
  } else if (bytes[4] != kJointCommandPacketVersion) {
    result.error = JointCommandPacketError::kWrongVersion;
  } else if (readLe<std::uint16_t>(bytes + 6U) != kJointCommandPacketSize) {
    result.error = JointCommandPacketError::kWrongDeclaredSize;
  } else if ((bytes[5] & ~kJointCommandKnownFlags) != 0U) {
    result.error = JointCommandPacketError::kInvalidFlags;
  } else if (readLe<std::uint32_t>(bytes + kCrcOffset) != crc32(bytes, kCrcOffset)) {
    result.error = JointCommandPacketError::kCrcMismatch;
  } else {
    JointCommandFrame frame;
    frame.flags = bytes[5];
    frame.sequence = readLe<std::uint64_t>(bytes + 8U);
    const auto timestamp_bits = readLe<std::uint64_t>(bytes + 16U);
    std::memcpy(&frame.source_timestamp_ns, &timestamp_bits, sizeof(timestamp_bits));
    frame.pico_tracking_epoch = readLe<std::uint64_t>(bytes + 24U);
    for (std::size_t i = 0U; i < frame.position_rad.size(); ++i) {
      const auto bits = readLe<std::uint64_t>(bytes + 32U + 8U * i);
      std::memcpy(&frame.position_rad[i], &bits, sizeof(bits));
    }
    result.error = validate(frame);
    if (result.error == JointCommandPacketError::kNone) {
      result.frame = frame;
    }
  }
  return result;
}

bool jointCommandPicoBridgeFresh(std::int64_t applied_bridge_send_monotonic_ns,
                                 std::int64_t now_monotonic_ns) noexcept {
  return applied_bridge_send_monotonic_ns > 0 &&
         now_monotonic_ns > applied_bridge_send_monotonic_ns &&
         now_monotonic_ns - applied_bridge_send_monotonic_ns <= 300000000;
}

bool JointCommandArmReadiness::update(bool ready, bool reset) noexcept {
  inhibited_ = inhibited_ || (reset && ready_seen_);
  const bool output_ready = ready && !reset && !inhibited_;
  ready_seen_ = ready_seen_ || output_ready;
  return output_ready;
}

void HandCommandFreshness::observe(const WujiHandTeleopFrame& frame) noexcept {
  const auto generation = side_ == ArmSide::kLeft
      ? frame.left_revocation_generation : frame.right_revocation_generation;
  if (generation != revocation_generation_) {
    reset();
    revocation_generation_ = generation;
  }
  if (!(side_ == ArmSide::kLeft ? frame.left_valid : frame.right_valid)) {
    return;
  }
  const std::int64_t source_ns = side_ == ArmSide::kLeft
                                     ? frame.left_source_timestamp_ns
                                     : frame.right_source_timestamp_ns;
  if (frame.sequence <= sequence_ ||
      frame.source_timestamp_ns <= publication_timestamp_ns_ ||
      source_ns <= 0 || source_ns > frame.source_timestamp_ns ||
      source_ns < source_timestamp_ns_ || frame.receive_monotonic_ns <= 0) {
    receive_monotonic_ns_ = 0;
    return;
  }
  const auto& positions = side_ == ArmSide::kLeft ? frame.left : frame.right;
  for (double value : positions) {
    if (!std::isfinite(value)) {
      receive_monotonic_ns_ = 0;
      return;
    }
  }
  sequence_ = frame.sequence;
  publication_timestamp_ns_ = frame.source_timestamp_ns;
  if (source_ns == source_timestamp_ns_ && receive_monotonic_ns_ == 0) {
    return;
  }
  source_timestamp_ns_ = source_ns;
  receive_monotonic_ns_ = frame.receive_monotonic_ns;
}

bool HandCommandFreshness::live(std::int64_t now_monotonic_ns) const noexcept {
  constexpr std::int64_t timeout_ns = 150000000;
  return receive_monotonic_ns_ > 0 && source_timestamp_ns_ > 0 &&
         now_monotonic_ns >= receive_monotonic_ns_ &&
         now_monotonic_ns >= source_timestamp_ns_ &&
         now_monotonic_ns - receive_monotonic_ns_ < timeout_ns &&
         now_monotonic_ns - source_timestamp_ns_ < timeout_ns;
}

JointCommandExporter::JointCommandExporter(const std::string& host, std::uint16_t port) {
  destination_.sin_family = AF_INET;
  destination_.sin_port = htons(port);
  if (port == 0U || inet_pton(AF_INET, host.c_str(), &destination_.sin_addr) != 1 ||
      (ntohl(destination_.sin_addr.s_addr) >> 24U) != 127U) {
    throw std::invalid_argument("joint command export requires loopback IPv4 and port [1,65535]");
  }
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "joint command UDP socket");
  }
}

JointCommandExporter::~JointCommandExporter() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
}

void JointCommandExporter::send(const JointCommandFrame& frame) {
  const JointCommandPacket packet = encodeJointCommandPacket(frame);
  const ssize_t sent = ::sendto(socket_fd_, packet.data(), packet.size(), MSG_DONTWAIT,
                               reinterpret_cast<const sockaddr*>(&destination_),
                               sizeof(destination_));
  if (sent < 0) {
    throw std::system_error(errno, std::generic_category(), "joint command UDP send");
  }
  if (static_cast<std::size_t>(sent) != packet.size()) {
    throw std::runtime_error("joint command UDP send was incomplete");
  }
}

}  // namespace tianji_qp_ik
