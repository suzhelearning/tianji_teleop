#include "tianji_qp_ik/wuji_hand_teleop_protocol.hpp"
#include "tianji_qp_ik/crc32.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

constexpr std::size_t kVersionOffset = 4U;
constexpr std::size_t kFlagsOffset = 5U;
constexpr std::size_t kDeclaredSizeOffset = 6U;
constexpr std::size_t kSequenceOffset = 8U;
constexpr std::size_t kTimestampOffset = 16U;
constexpr std::size_t kLeftTimestampOffset = 24U;
constexpr std::size_t kRightTimestampOffset = 32U;
constexpr std::size_t kLeftOffset = 40U;
constexpr std::size_t kRightOffset =
    kLeftOffset + kWujiHandJointDof * sizeof(double);
constexpr std::size_t kCrcOffset = kWujiHandTeleopPacketSize - sizeof(std::uint32_t);

std::uint16_t readLe16(const std::uint8_t* bytes) noexcept {
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U);
}

std::uint32_t readLe32(const std::uint8_t* bytes) noexcept {
  std::uint32_t value = 0U;
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    value |= static_cast<std::uint32_t>(bytes[index]) << (8U * index);
  }
  return value;
}

std::uint64_t readLe64(const std::uint8_t* bytes) noexcept {
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    value |= static_cast<std::uint64_t>(bytes[index]) << (8U * index);
  }
  return value;
}

std::int64_t readLeInt64(const std::uint8_t* bytes) noexcept {
  const std::uint64_t bits = readLe64(bytes);
  std::int64_t value = 0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double readLeDouble(const std::uint8_t* bytes) noexcept {
  const std::uint64_t bits = readLe64(bytes);
  double value = 0.0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void writeLe16(std::uint8_t* bytes, std::uint16_t value) noexcept {
  bytes[0] = static_cast<std::uint8_t>(value & 0xffU);
  bytes[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void writeLe32(std::uint8_t* bytes, std::uint32_t value) noexcept {
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    bytes[index] = static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

void writeLe64(std::uint8_t* bytes, std::uint64_t value) noexcept {
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    bytes[index] = static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

void writeLeInt64(std::uint8_t* bytes, std::int64_t value) noexcept {
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe64(bytes, bits);
}

void writeLeDouble(std::uint8_t* bytes, double value) noexcept {
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe64(bytes, bits);
}

bool finiteJoints(const std::array<double, kWujiHandJointDof>& joints) noexcept {
  for (const double value : joints) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool validMetadata(const WujiHandTeleopFrame& frame) noexcept {
  const auto valid_side = [&frame](bool valid, std::int64_t timestamp) {
    return valid ? timestamp > 0 && timestamp <= frame.source_timestamp_ns
                 : timestamp == 0;
  };
  return frame.sequence > 0U && frame.source_timestamp_ns > 0 &&
         (frame.left_valid || frame.right_valid) &&
         valid_side(frame.left_valid, frame.left_source_timestamp_ns) &&
         valid_side(frame.right_valid, frame.right_source_timestamp_ns);
}

}  // namespace

std::vector<std::uint8_t> encodeWujiHandTeleopPacket(
    const WujiHandTeleopFrame& frame) {
  if (!validMetadata(frame)) {
    throw std::invalid_argument("invalid Wuji Hand teleop metadata");
  }
  if (!finiteJoints(frame.left) || !finiteJoints(frame.right)) {
    throw std::invalid_argument("Wuji Hand joint position contains NaN or infinity");
  }

  std::vector<std::uint8_t> packet(kWujiHandTeleopPacketSize, 0U);
  packet[0] = static_cast<std::uint8_t>('T');
  packet[1] = static_cast<std::uint8_t>('J');
  packet[2] = static_cast<std::uint8_t>('H');
  packet[3] = static_cast<std::uint8_t>('2');
  packet[kVersionOffset] = kWujiHandPacketVersion;
  packet[kFlagsOffset] =
      static_cast<std::uint8_t>((frame.left_valid ? kWujiHandLeftValidFlag : 0U) |
                                (frame.right_valid ? kWujiHandRightValidFlag : 0U));
  writeLe16(packet.data() + kDeclaredSizeOffset,
            static_cast<std::uint16_t>(kWujiHandTeleopPacketSize));
  writeLe64(packet.data() + kSequenceOffset, frame.sequence);
  writeLeInt64(packet.data() + kTimestampOffset, frame.source_timestamp_ns);
  writeLeInt64(packet.data() + kLeftTimestampOffset,
               frame.left_source_timestamp_ns);
  writeLeInt64(packet.data() + kRightTimestampOffset,
               frame.right_source_timestamp_ns);
  for (std::size_t index = 0U; index < kWujiHandJointDof; ++index) {
    writeLeDouble(packet.data() + kLeftOffset + index * sizeof(double),
                  frame.left[index]);
    writeLeDouble(packet.data() + kRightOffset + index * sizeof(double),
                  frame.right[index]);
  }
  writeLe32(packet.data() + kCrcOffset, crc32(packet.data(), kCrcOffset));
  return packet;
}

WujiHandPacketDecodeResult decodeWujiHandTeleopPacket(
    const std::uint8_t* bytes, std::size_t size) noexcept {
  WujiHandPacketDecodeResult result;
  if (bytes == nullptr || size != kWujiHandTeleopPacketSize) {
    result.error = WujiHandPacketError::kWrongSize;
    return result;
  }
  if (bytes[0] != static_cast<std::uint8_t>('T') ||
      bytes[1] != static_cast<std::uint8_t>('J') ||
      bytes[2] != static_cast<std::uint8_t>('H') ||
      bytes[3] != static_cast<std::uint8_t>('2')) {
    result.error = WujiHandPacketError::kWrongMagic;
    return result;
  }
  if (bytes[kVersionOffset] != kWujiHandPacketVersion) {
    result.error = WujiHandPacketError::kWrongVersion;
    return result;
  }
  if (readLe16(bytes + kDeclaredSizeOffset) != kWujiHandTeleopPacketSize) {
    result.error = WujiHandPacketError::kWrongDeclaredSize;
    return result;
  }
  const std::uint8_t flags = bytes[kFlagsOffset];
  if ((flags & ~kWujiHandKnownFlags) != 0U ||
      (flags & kWujiHandKnownFlags) == 0U) {
    result.error = WujiHandPacketError::kInvalidFlags;
    return result;
  }
  if (readLe32(bytes + kCrcOffset) != crc32(bytes, kCrcOffset)) {
    result.error = WujiHandPacketError::kCrcMismatch;
    return result;
  }

  WujiHandTeleopFrame frame;
  frame.sequence = readLe64(bytes + kSequenceOffset);
  frame.source_timestamp_ns = readLeInt64(bytes + kTimestampOffset);
  frame.left_source_timestamp_ns = readLeInt64(bytes + kLeftTimestampOffset);
  frame.right_source_timestamp_ns = readLeInt64(bytes + kRightTimestampOffset);
  frame.left_valid = (flags & kWujiHandLeftValidFlag) != 0U;
  frame.right_valid = (flags & kWujiHandRightValidFlag) != 0U;
  if (!validMetadata(frame)) {
    result.error = WujiHandPacketError::kInvalidMetadata;
    return result;
  }
  for (std::size_t index = 0U; index < kWujiHandJointDof; ++index) {
    frame.left[index] =
        readLeDouble(bytes + kLeftOffset + index * sizeof(double));
    frame.right[index] =
        readLeDouble(bytes + kRightOffset + index * sizeof(double));
  }
  if (!finiteJoints(frame.left) || !finiteJoints(frame.right)) {
    result.error = WujiHandPacketError::kNonFiniteJointPosition;
    return result;
  }

  result.error = WujiHandPacketError::kNone;
  result.frame = frame;
  return result;
}

void WujiHandHistory::SideHistory::append(
    std::int64_t timestamp_ns, std::int64_t source_ns,
    const std::array<double, kWujiHandJointDof>& position) noexcept {
  if (size == kCapacity) {
    begin = (begin + 1U) % kCapacity;
    --size;
  }
  points[(begin + size) % kCapacity] = Point{timestamp_ns, position};
  ++size;
  source_timestamp_ns = source_ns;
}

bool WujiHandHistory::SideHistory::sample(
    std::int64_t timestamp_ns,
    std::array<double, kWujiHandJointDof>& position) const noexcept {
  if (size == 0U) {
    return false;
  }
  const Point* lower = &points[begin];
  if (timestamp_ns <= lower->timestamp_ns) {
    position = lower->position;
    return true;
  }
  for (std::size_t index = 1U; index < size; ++index) {
    const Point& upper = points[(begin + index) % kCapacity];
    if (timestamp_ns < upper.timestamp_ns) {
      const double alpha = static_cast<double>(timestamp_ns - lower->timestamp_ns) /
                           static_cast<double>(upper.timestamp_ns - lower->timestamp_ns);
      for (std::size_t joint = 0U; joint < kWujiHandJointDof; ++joint) {
        position[joint] = (1.0 - alpha) * lower->position[joint] +
                          alpha * upper.position[joint];
      }
      return true;
    }
    lower = &upper;
  }
  position = lower->position;
  return true;
}

bool WujiHandHistory::observe(const WujiHandTeleopFrame& frame) noexcept {
  if (!validMetadata(frame) || !finiteJoints(frame.left) ||
      !finiteJoints(frame.right) || frame.sequence <= sequence_ ||
      frame.source_timestamp_ns <= publication_timestamp_ns_ ||
      (frame.left_valid &&
       frame.left_source_timestamp_ns < left_.source_timestamp_ns) ||
      (frame.right_valid &&
       frame.right_source_timestamp_ns < right_.source_timestamp_ns)) {
    return false;
  }
  if (frame.left_valid) {
    left_.append(frame.source_timestamp_ns, frame.left_source_timestamp_ns,
                 frame.left);
  }
  if (frame.right_valid) {
    right_.append(frame.source_timestamp_ns, frame.right_source_timestamp_ns,
                  frame.right);
  }
  sequence_ = frame.sequence;
  publication_timestamp_ns_ = frame.source_timestamp_ns;
  return true;
}

WujiHandSample WujiHandHistory::sample(std::int64_t now_monotonic_ns) const noexcept {
  WujiHandSample result;
  if (now_monotonic_ns <= 0) {
    return result;
  }
  const std::int64_t timestamp_ns =
      now_monotonic_ns - kWujiHandInterpolationDelayNs;
  result.left_valid = left_.sample(timestamp_ns, result.left);
  result.right_valid = right_.sample(timestamp_ns, result.right);
  return result;
}

void WujiHandHistory::clear() noexcept {
  left_.begin = right_.begin = 0U;
  left_.size = right_.size = 0U;
  left_.source_timestamp_ns = right_.source_timestamp_ns = 0;
  sequence_ = 0U;
  publication_timestamp_ns_ = 0;
}

}  // namespace tianji_qp_ik
