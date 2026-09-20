#include "tianji_qp_ik/pico_teleop_protocol.hpp"

#include "tianji_qp_ik/crc32.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

constexpr std::uint32_t kRequiredFlags = 0x0fU;
constexpr std::uint32_t kLeftArmDirectionValid = 1U << 4U;
constexpr std::uint32_t kRightArmDirectionValid = 1U << 5U;
constexpr std::uint32_t kUpperLimbSkeletonValid =
    kPicoTeleopUpperLimbSkeletonValid;
constexpr std::uint32_t kUpperLimbRotationsValid =
    kPicoTeleopUpperLimbRotationsValid;
constexpr std::uint32_t kV2KnownFlags = 0x3fU;
constexpr std::uint32_t kV3KnownFlags = 0x7fU;
constexpr std::uint32_t kUserButtonPressed =
    kPicoTeleopUserButtonPressedFlag;
constexpr std::uint32_t kV4KnownFlags = 0x1ffU;
constexpr double kQuaternionNormTolerance = 1e-3;
constexpr double kDirectionEpsilon = 1e-9;
constexpr std::size_t kDiscontinuityConfirmationFrames = 3;

std::uint16_t readLe16(const std::uint8_t* bytes) noexcept {
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U);
}

std::uint32_t readLe32(const std::uint8_t* bytes) noexcept {
  std::uint32_t value = 0;
  for (std::size_t index = 0; index < 4; ++index) {
    value |= static_cast<std::uint32_t>(bytes[index]) << (8U * index);
  }
  return value;
}

std::uint64_t readLe64(const std::uint8_t* bytes) noexcept {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    value |= static_cast<std::uint64_t>(bytes[index]) << (8U * index);
  }
  return value;
}

std::int64_t readLeInt64(const std::uint8_t* bytes) noexcept {
  const std::uint64_t bits = readLe64(bytes);
  std::int64_t value = 0;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double readLeDouble(const std::uint8_t* bytes) noexcept {
  const std::uint64_t bits = readLe64(bytes);
  double value = 0.0;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool decodePose(const std::uint8_t* bytes, Pose* pose,
                PicoPacketError* error) noexcept {
  pose->position = Eigen::Vector3d(readLeDouble(bytes), readLeDouble(bytes + 8),
                                   readLeDouble(bytes + 16));
  const double x = readLeDouble(bytes + 24);
  const double y = readLeDouble(bytes + 32);
  const double z = readLeDouble(bytes + 40);
  const double w = readLeDouble(bytes + 48);
  const Eigen::Quaterniond quaternion(w, x, y, z);

  if (!pose->position.allFinite() || !quaternion.coeffs().allFinite()) {
    *error = PicoPacketError::kNonFinitePose;
    return false;
  }
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) ||
      std::abs(norm - 1.0) > kQuaternionNormTolerance) {
    *error = PicoPacketError::kInvalidQuaternion;
    return false;
  }
  pose->rotation = quaternion.normalized().toRotationMatrix();
  if (!isProperRotation(pose->rotation)) {
    *error = PicoPacketError::kInvalidQuaternion;
    return false;
  }
  return true;
}

ArmDirectionReference decodeDirection(const std::uint8_t* bytes,
                                      bool declared_valid) noexcept {
  ArmDirectionReference result;
  if (!declared_valid) {
    return result;
  }
  result.direction = Eigen::Vector3d(
      readLeDouble(bytes), readLeDouble(bytes + 8), readLeDouble(bytes + 16));
  if (!result.direction.allFinite() ||
      result.direction.norm() <= kDirectionEpsilon) {
    result.direction.setZero();
    return result;
  }
  result.direction.normalize();
  result.valid = true;
  result.source = ArmDirectionReferenceSource::kPico;
  return result;
}

bool decodeQuaternion(const std::uint8_t* bytes,
                      Eigen::Quaterniond* result,
                      PicoPacketError* error) noexcept {
  const double x = readLeDouble(bytes);
  const double y = readLeDouble(bytes + 8U);
  const double z = readLeDouble(bytes + 16U);
  const double w = readLeDouble(bytes + 24U);
  const Eigen::Quaterniond quaternion(w, x, y, z);
  if (!quaternion.coeffs().allFinite()) {
    *error = PicoPacketError::kNonFinitePose;
    return false;
  }
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) ||
      std::abs(norm - 1.0) > kQuaternionNormTolerance) {
    *error = PicoPacketError::kInvalidQuaternion;
    return false;
  }
  *result = quaternion.normalized();
  return true;
}

bool exceedsPositionJump(const Pose& current, const Pose& previous,
                         double limit) noexcept {
  return (current.position - previous.position).norm() > limit;
}

}  // namespace

PicoPacketDecodeResult decodePicoTeleopPacket(const std::uint8_t* bytes,
                                               std::size_t size) noexcept {
  PicoPacketDecodeResult result;
  if (bytes == nullptr ||
      (size != kPicoTeleopPacketV1Size &&
       size != kPicoTeleopPacketV2Size &&
       size != kPicoTeleopPacketV3Size &&
       size != kPicoTeleopPacketV4Size)) {
    result.error = PicoPacketError::kWrongSize;
    return result;
  }
  if (bytes[0] != static_cast<std::uint8_t>('T') ||
      bytes[1] != static_cast<std::uint8_t>('J') ||
      bytes[2] != static_cast<std::uint8_t>('V') ||
      bytes[3] != static_cast<std::uint8_t>('R')) {
    result.error = PicoPacketError::kWrongMagic;
    return result;
  }
  const std::uint16_t version = readLe16(bytes + 4);
  const bool is_v1 = version == 1U && size == kPicoTeleopPacketV1Size;
  const bool is_v2 = version == 2U && size == kPicoTeleopPacketV2Size;
  const bool is_v3 = version == 3U && size == kPicoTeleopPacketV3Size;
  const bool is_v4 = version == 4U && size == kPicoTeleopPacketV4Size;
  if (!is_v1 && !is_v2 && !is_v3 && !is_v4) {
    result.error = PicoPacketError::kWrongVersion;
    return result;
  }
  if (readLe16(bytes + 6) != size) {
    result.error = PicoPacketError::kWrongDeclaredSize;
    return result;
  }
  const std::uint32_t flags = readLe32(bytes + 40);
  const std::uint32_t known_flags =
      is_v4 ? kV4KnownFlags : (is_v3 ? kV3KnownFlags : kV2KnownFlags);
  if ((flags & kRequiredFlags) != kRequiredFlags ||
      (flags & ~known_flags) != 0U ||
      (is_v1 && flags != kRequiredFlags) ||
      (is_v2 && (flags & kUpperLimbSkeletonValid) != 0U) ||
      (!is_v4 && (flags & kUpperLimbRotationsValid) != 0U) ||
      ((flags & kUpperLimbRotationsValid) != 0U &&
       (flags & kUpperLimbSkeletonValid) == 0U)) {
    result.error = PicoPacketError::kInvalidFlags;
    return result;
  }
  const std::size_t crc_offset = size - 4U;
  if (readLe32(bytes + crc_offset) != crc32(bytes, crc_offset)) {
    result.error = PicoPacketError::kCrcMismatch;
    return result;
  }

  PicoTeleopFrame frame;
  frame.sequence = readLe64(bytes + 8);
  frame.tracking_epoch = readLe64(bytes + 16);
  frame.source_timestamp_ns = readLeInt64(bytes + 24);
  frame.bridge_send_monotonic_ns = readLeInt64(bytes + 32);
  if (frame.sequence == 0 || frame.tracking_epoch == 0 ||
      frame.source_timestamp_ns <= 0 || frame.bridge_send_monotonic_ns <= 0) {
    result.error = PicoPacketError::kInvalidMetadata;
    return result;
  }
  frame.user_button_pressed =
      is_v4 && (flags & kUserButtonPressed) != 0U;
  if (!decodePose(bytes + 44, &frame.left, &result.error) ||
      !decodePose(bytes + 100, &frame.right, &result.error)) {
    return result;
  }
  if (is_v2 || is_v3 || is_v4) {
    frame.left_arm_direction = decodeDirection(
        bytes + 156, (flags & kLeftArmDirectionValid) != 0U);
    frame.right_arm_direction = decodeDirection(
        bytes + 180, (flags & kRightArmDirectionValid) != 0U);
  }
  if ((is_v3 || is_v4) && (flags & kUpperLimbSkeletonValid) != 0U) {
    for (std::size_t index = 0; index < kPicoUpperLimbPointCount; ++index) {
      Eigen::Vector3d& point = frame.upper_limb_skeleton.points[index];
      const std::size_t offset = 204U + index * 24U;
      point = Eigen::Vector3d(readLeDouble(bytes + offset),
                              readLeDouble(bytes + offset + 8U),
                              readLeDouble(bytes + offset + 16U));
      if (!point.allFinite()) {
        result.error = PicoPacketError::kNonFinitePose;
        return result;
      }
    }
    frame.upper_limb_skeleton.valid = true;
  }
  if (is_v4 && (flags & kUpperLimbRotationsValid) != 0U) {
    for (std::size_t index = 0; index < kPicoUpperLimbPointCount; ++index) {
      if (!decodeQuaternion(bytes + 396U + index * 32U,
                            &frame.upper_limb_skeleton.rotations[index],
                            &result.error)) {
        return result;
      }
    }
    frame.upper_limb_skeleton.rotations_valid = true;
  }

  result.error = PicoPacketError::kNone;
  result.frame = frame;
  return result;
}

PicoTeleopStreamGate::PicoTeleopStreamGate(
    double max_position_jump_m, double max_orientation_jump_rad, bool reject_pose_jumps)
    : max_position_jump_m_(max_position_jump_m),
      max_orientation_jump_rad_(max_orientation_jump_rad), reject_pose_jumps_(reject_pose_jumps) {
  if (!std::isfinite(max_position_jump_m_) || max_position_jump_m_ <= 0.0 ||
      !std::isfinite(max_orientation_jump_rad_) ||
      max_orientation_jump_rad_ <= 0.0) {
    throw std::invalid_argument("PICO stream jump limits must be finite and positive");
  }
}

PicoStreamDecision PicoTeleopStreamGate::evaluate(
    const PicoTeleopFrame& frame) {
  if (frame.tracking_epoch == 0) {
    return {false, false, PicoStreamRejectReason::kZeroEpoch};
  }
  if (!initialized_) {
    if (frame.sequence == 0) {
      return {false, false, PicoStreamRejectReason::kOutOfOrder};
    }
    accept(frame);
    return {true, true, PicoStreamRejectReason::kNone};
  }
  if (frame.tracking_epoch < current_epoch_) {
    return {false, false, PicoStreamRejectReason::kEpochRollback};
  }
  if (frame.tracking_epoch > current_epoch_) {
    if (frame.sequence == 0) {
      return {false, false, PicoStreamRejectReason::kOutOfOrder};
    }
    accept(frame);
    return {true, true, PicoStreamRejectReason::kNone};
  }
  if (frame.sequence <= last_observed_sequence_) {
    return {false, false, PicoStreamRejectReason::kOutOfOrder};
  }
  last_observed_sequence_ = frame.sequence;

  if (!reject_pose_jumps_) {
    accept(frame);
    return {true, false, PicoStreamRejectReason::kNone};
  }

  const PicoStreamRejectReason anchor_jump =
      jumpReason(frame.left, frame.right, last_left_, last_right_);
  if (anchor_jump == PicoStreamRejectReason::kNone) {
    accept(frame);
    return {true, false, PicoStreamRejectReason::kNone};
  }

  if (!candidate_active_) {
    startCandidate(frame);
    return {false, false, anchor_jump};
  }

  const PicoStreamRejectReason candidate_jump =
      jumpReason(frame.left, frame.right, candidate_left_, candidate_right_);
  if (candidate_jump != PicoStreamRejectReason::kNone) {
    startCandidate(frame);
    return {false, false, candidate_jump};
  }

  candidate_left_ = frame.left;
  candidate_right_ = frame.right;
  ++candidate_count_;
  if (candidate_count_ < kDiscontinuityConfirmationFrames) {
    return {false, false, anchor_jump};
  }

  accept(frame);
  return {true, false, PicoStreamRejectReason::kNone, true};
}

void PicoTeleopStreamGate::reset() noexcept {
  initialized_ = false;
  current_epoch_ = 0;
  last_observed_sequence_ = 0;
  last_left_ = Pose{};
  last_right_ = Pose{};
  clearCandidate();
}

PicoStreamRejectReason PicoTeleopStreamGate::jumpReason(
    const Pose& left, const Pose& right, const Pose& previous_left,
    const Pose& previous_right) const noexcept {
  if (exceedsPositionJump(left, previous_left, max_position_jump_m_) ||
      exceedsPositionJump(right, previous_right, max_position_jump_m_)) {
    return PicoStreamRejectReason::kPositionJump;
  }
  if (rotationDistance(left.rotation, previous_left.rotation) >
          max_orientation_jump_rad_ ||
      rotationDistance(right.rotation, previous_right.rotation) >
          max_orientation_jump_rad_) {
    return PicoStreamRejectReason::kOrientationJump;
  }
  return PicoStreamRejectReason::kNone;
}

void PicoTeleopStreamGate::startCandidate(
    const PicoTeleopFrame& frame) noexcept {
  candidate_active_ = true;
  candidate_count_ = 1;
  candidate_left_ = frame.left;
  candidate_right_ = frame.right;
}

void PicoTeleopStreamGate::clearCandidate() noexcept {
  candidate_active_ = false;
  candidate_count_ = 0;
  candidate_left_ = Pose{};
  candidate_right_ = Pose{};
}

void PicoTeleopStreamGate::accept(const PicoTeleopFrame& frame) noexcept {
  initialized_ = true;
  current_epoch_ = frame.tracking_epoch;
  last_observed_sequence_ = frame.sequence;
  last_left_ = frame.left;
  last_right_ = frame.right;
  clearCandidate();
}

}  // namespace tianji_qp_ik
