#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace tianji_mapped_palm {

inline constexpr std::size_t kWujiHandJointDof = 20U;
inline constexpr std::size_t kWujiHandTeleopPacketSize = 364U;
inline constexpr std::uint8_t kWujiHandPacketVersion = 2U;
inline constexpr int kWujiHandPublicationRateHz = 100;
inline constexpr std::int64_t kWujiHandInterpolationDelayNs = 10000000;
inline constexpr std::uint8_t kWujiHandLeftValidFlag = 1U << 0U;
inline constexpr std::uint8_t kWujiHandRightValidFlag = 1U << 1U;
inline constexpr std::uint8_t kWujiHandKnownFlags =
    kWujiHandLeftValidFlag | kWujiHandRightValidFlag;

struct WujiHandTeleopFrame {
  std::uint64_t sequence{0U};
  // Publication sample time, on the shared local monotonic clock.
  std::int64_t source_timestamp_ns{0};
  // Original ROS callback receive times, retained by repeated publications.
  std::int64_t left_source_timestamp_ns{0};
  std::int64_t right_source_timestamp_ns{0};
  std::array<double, kWujiHandJointDof> left{};
  std::array<double, kWujiHandJointDof> right{};
  bool left_valid{false};
  bool right_valid{false};
  // Receiver-local metadata; this field is never encoded on the wire.
  std::int64_t receive_monotonic_ns{0};
};

enum class WujiHandPacketError {
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

struct WujiHandPacketDecodeResult {
  WujiHandPacketError error{WujiHandPacketError::kWrongSize};
  std::optional<WujiHandTeleopFrame> frame;
};

std::vector<std::uint8_t> encodeWujiHandTeleopPacket(
    const WujiHandTeleopFrame& frame);

WujiHandPacketDecodeResult decodeWujiHandTeleopPacket(
    const std::uint8_t* bytes, std::size_t size) noexcept;

struct WujiHandSample {
  std::array<double, kWujiHandJointDof> left{};
  std::array<double, kWujiHandJointDof> right{};
  bool left_valid{false};
  bool right_valid{false};
};

// Controller-thread-owned bounded history; no allocation or extrapolation.
// Availability is not freshness: export must still check original source ages.
class WujiHandHistory {
 public:
  bool observe(const WujiHandTeleopFrame& frame) noexcept;
  WujiHandSample sample(std::int64_t now_monotonic_ns) const noexcept;
  void clear() noexcept;

 private:
  struct SideHistory {
    struct Point {
      std::int64_t timestamp_ns{0};
      std::array<double, kWujiHandJointDof> position{};
    };
    static constexpr std::size_t kCapacity = 8U;
    std::array<Point, kCapacity> points{};
    std::size_t begin{0U};
    std::size_t size{0U};
    std::int64_t source_timestamp_ns{0};

    void append(std::int64_t timestamp_ns, std::int64_t source_ns,
                const std::array<double, kWujiHandJointDof>& position) noexcept;
    bool sample(std::int64_t timestamp_ns,
                std::array<double, kWujiHandJointDof>& position) const noexcept;
  };
  SideHistory left_;
  SideHistory right_;
  std::uint64_t sequence_{0U};
  std::int64_t publication_timestamp_ns_{0};
};

}  // namespace tianji_mapped_palm
