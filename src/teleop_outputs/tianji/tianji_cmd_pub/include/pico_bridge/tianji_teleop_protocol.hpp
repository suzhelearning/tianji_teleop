#pragma once

#include "pico_bridge/tianji_teleop_geometry.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace pico_bridge {

inline constexpr std::size_t kTianjiTeleopPacketSize = 656;
inline constexpr std::uint16_t kTianjiTeleopProtocolVersion = 4;
inline constexpr std::uint32_t kTianjiTeleopRequiredFlags = 0x0f;
inline constexpr std::uint32_t kTianjiTeleopLeftArmDirectionValid = 1U << 4U;
inline constexpr std::uint32_t kTianjiTeleopRightArmDirectionValid = 1U << 5U;
inline constexpr std::uint32_t kTianjiTeleopUpperLimbSkeletonValid = 1U << 6U;
inline constexpr std::uint32_t kTianjiTeleopUpperLimbRotationsValid = 1U << 7U;
// 转发器/桥侧 A 键按下事件（TJVR flags bit8），viewer 用上升沿切换 teleop。
inline constexpr std::uint32_t kTianjiTeleopUserButtonPressedFlag = 1U << 8U;

struct CorrectedIkStatus
{
  bool valid{false};
  std::string rejection_reason;
  std::int64_t source_stamp_ns{0};
  std::uint64_t tracking_epoch{0};
};

struct TianjiTeleopWireFrame
{
  std::uint64_t sequence{0};
  std::uint64_t tracking_epoch{0};
  std::int64_t source_timestamp_ns{0};
  std::int64_t bridge_send_monotonic_ns{0};
  Eigen::Isometry3d left_target{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_target{Eigen::Isometry3d::Identity()};
  PicoArmDirection left_arm_direction;
  PicoArmDirection right_arm_direction;
  PicoUpperLimbSkeleton upper_limb_skeleton;
  bool user_button_pressed{false};
};

struct TianjiTeleopBridgeOutcome
{
  std::optional<TianjiTeleopWireFrame> frame;
  std::string rejection_reason;
};

CorrectedIkStatus parse_corrected_ik_status(std::string_view json_text);

class TianjiTeleopBridgeCore
{
public:
  explicit TianjiTeleopBridgeCore(
    std::size_t cache_capacity = 8,
    const PicoPositionRetargetingConfig & position_retargeting = {});
  void reset() noexcept;
  TianjiTeleopBridgeOutcome ingest_skeleton(
    std::int64_t source_stamp_ns, const PicoSkeletonFrame & skeleton);
  TianjiTeleopBridgeOutcome ingest_status(const CorrectedIkStatus & status);

private:
  TianjiTeleopBridgeOutcome match(std::int64_t source_stamp_ns);
  void rememberCompleted(std::int64_t source_stamp_ns);
  bool completed(std::int64_t source_stamp_ns) const;
  void trimSkeletons();
  void trimStatuses();

  std::size_t cache_capacity_;
  PicoPositionRetargetingConfig position_retargeting_;
  std::map<std::int64_t, PicoSkeletonFrame> skeletons_;
  std::map<std::int64_t, CorrectedIkStatus> statuses_;
  std::deque<std::int64_t> completed_stamps_;
  std::int64_t highest_skeleton_stamp_seen_{0};
  std::int64_t highest_status_stamp_seen_{0};
  std::uint64_t next_sequence_{1};
};

std::uint32_t tianji_teleop_crc32(const std::uint8_t * data, std::size_t size);
std::array<std::uint8_t, kTianjiTeleopPacketSize> encode_tianji_teleop_packet(
  const TianjiTeleopWireFrame & frame);

}  // namespace pico_bridge
