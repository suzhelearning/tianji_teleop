#pragma once

#include "tianji_qp_ik/arm_angle.hpp"
#include "tianji_qp_ik/types.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace tianji_qp_ik {

inline constexpr std::size_t kPicoTeleopPacketV1Size = 160;
inline constexpr std::size_t kPicoTeleopPacketV2Size = 208;
inline constexpr std::size_t kPicoTeleopPacketV3Size = 400;
inline constexpr std::size_t kPicoTeleopPacketV4Size = 656;
inline constexpr std::size_t kPicoTeleopMaximumPacketSize =
    kPicoTeleopPacketV4Size;
inline constexpr std::size_t kPicoUpperLimbPointCount = 8;
inline constexpr std::size_t kPicoLeftShoulderPoint = 0;
inline constexpr std::size_t kPicoLeftElbowPoint = 1;
inline constexpr std::size_t kPicoLeftWristPoint = 2;
inline constexpr std::size_t kPicoLeftHandPoint = 3;
inline constexpr std::size_t kPicoRightShoulderPoint = 4;
inline constexpr std::size_t kPicoRightElbowPoint = 5;
inline constexpr std::size_t kPicoRightWristPoint = 6;
inline constexpr std::size_t kPicoRightHandPoint = 7;
inline constexpr std::uint32_t kPicoTeleopUpperLimbSkeletonValid = 1U << 6U;
inline constexpr std::uint32_t kPicoTeleopUpperLimbRotationsValid = 1U << 7U;
inline constexpr std::uint32_t kPicoTeleopUserButtonPressedFlag = 1U << 8U;

struct PicoUpperLimbSkeleton {
  PicoUpperLimbSkeleton() {
    for (Eigen::Vector3d& point : points) {
      point.setZero();
    }
    for (Eigen::Quaterniond& rotation : rotations) {
      rotation.setIdentity();
    }
  }
  bool valid{false};
  bool rotations_valid{false};
  std::array<Eigen::Vector3d, kPicoUpperLimbPointCount> points{};
  std::array<Eigen::Quaterniond, kPicoUpperLimbPointCount> rotations{};
};

enum class PicoPacketError {
  kNone,
  kWrongSize,
  kWrongMagic,
  kWrongVersion,
  kWrongDeclaredSize,
  kInvalidFlags,
  kCrcMismatch,
  kInvalidMetadata,
  kNonFinitePose,
  kInvalidQuaternion,
};

struct PicoTeleopFrame {
  std::uint64_t sequence{0};
  std::uint64_t tracking_epoch{0};
  std::int64_t source_timestamp_ns{0};
  std::int64_t bridge_send_monotonic_ns{0};
  std::int64_t receive_monotonic_ns{0};
  Pose left;
  Pose right;
  ArmDirectionReference left_arm_direction;
  ArmDirectionReference right_arm_direction;
  PicoUpperLimbSkeleton upper_limb_skeleton;
  bool user_button_pressed{false};
  // Receiver-local metadata. This flag is never encoded on the wire.
  bool stream_discontinuity{false};
  // Monotonic receiver-local event generation. Persisted on later frames so a
  // latest-only exchange cannot erase a discontinuity before it is consumed.
  std::uint64_t resynchronization_generation{0U};
};

struct PicoPacketDecodeResult {
  PicoPacketError error{PicoPacketError::kWrongSize};
  std::optional<PicoTeleopFrame> frame;
};

PicoPacketDecodeResult decodePicoTeleopPacket(const std::uint8_t* bytes,
                                               std::size_t size) noexcept;

enum class PicoStreamRejectReason {
  kNone,
  kZeroEpoch,
  kEpochRollback,
  kOutOfOrder,
  kPositionJump,
  kOrientationJump,
};

struct PicoStreamDecision {
  bool accepted{false};
  bool epoch_changed{false};
  PicoStreamRejectReason reason{PicoStreamRejectReason::kNone};
  bool stream_discontinuity{false};
};

class PicoTeleopStreamGate {
 public:
  PicoTeleopStreamGate(double max_position_jump_m,
                       double max_orientation_jump_rad);
  PicoStreamDecision evaluate(const PicoTeleopFrame& frame);
  void reset() noexcept;

 private:
  PicoStreamRejectReason jumpReason(const Pose& left, const Pose& right,
                                    const Pose& previous_left,
                                    const Pose& previous_right) const noexcept;
  void startCandidate(const PicoTeleopFrame& frame) noexcept;
  void clearCandidate() noexcept;
  void accept(const PicoTeleopFrame& frame) noexcept;

  double max_position_jump_m_{0.0};
  double max_orientation_jump_rad_{0.0};
  bool initialized_{false};
  std::uint64_t current_epoch_{0};
  std::uint64_t last_observed_sequence_{0};
  Pose last_left_;
  Pose last_right_;
  bool candidate_active_{false};
  std::size_t candidate_count_{0};
  Pose candidate_left_;
  Pose candidate_right_;
};

}  // namespace tianji_qp_ik
