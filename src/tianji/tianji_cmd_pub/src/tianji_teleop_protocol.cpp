#include "pico_bridge/tianji_teleop_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <type_traits>

#include <yaml-cpp/yaml.h>

namespace pico_bridge {
namespace {

constexpr std::int64_t kSourceTimestampRollbackResetNs = 1'000'000'000LL;

CorrectedIkStatus rejectedStatus(const char * reason)
{
  CorrectedIkStatus status;
  status.rejection_reason = reason;
  return status;
}

template<typename Integer>
void writeLittleEndian(
  std::array<std::uint8_t, kTianjiTeleopPacketSize> & packet,
  std::size_t offset,
  Integer value)
{
  using Unsigned = std::make_unsigned_t<Integer>;
  Unsigned bits = static_cast<Unsigned>(value);
  for (std::size_t byte = 0; byte < sizeof(Integer); ++byte) {
    packet[offset + byte] = static_cast<std::uint8_t>(bits & 0xffU);
    bits >>= 8U;
  }
}

void writeDouble(
  std::array<std::uint8_t, kTianjiTeleopPacketSize> & packet,
  std::size_t offset,
  double value)
{
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  writeLittleEndian(packet, offset, bits);
}

void writePose(
  std::array<std::uint8_t, kTianjiTeleopPacketSize> & packet,
  std::size_t position_offset,
  std::size_t quaternion_offset,
  const Eigen::Isometry3d & pose)
{
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    writeDouble(packet, position_offset + static_cast<std::size_t>(axis) * 8U,
      pose.translation()[axis]);
  }
  const Eigen::Quaterniond quaternion(pose.linear());
  writeDouble(packet, quaternion_offset, quaternion.x());
  writeDouble(packet, quaternion_offset + 8U, quaternion.y());
  writeDouble(packet, quaternion_offset + 16U, quaternion.z());
  writeDouble(packet, quaternion_offset + 24U, quaternion.w());
}

void writeVector(
  std::array<std::uint8_t, kTianjiTeleopPacketSize> & packet,
  std::size_t offset, const Eigen::Vector3d & vector)
{
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    writeDouble(
      packet, offset + static_cast<std::size_t>(axis) * 8U, vector[axis]);
  }
}

void writeQuaternion(
  std::array<std::uint8_t, kTianjiTeleopPacketSize> & packet,
  std::size_t offset, const Eigen::Quaterniond & quaternion)
{
  const Eigen::Quaterniond normalized = quaternion.normalized();
  writeDouble(packet, offset, normalized.x());
  writeDouble(packet, offset + 8U, normalized.y());
  writeDouble(packet, offset + 16U, normalized.z());
  writeDouble(packet, offset + 24U, normalized.w());
}

bool rotationsEncodable(const PicoUpperLimbSkeleton & skeleton)
{
  return std::all_of(
    skeleton.rotations.begin(), skeleton.rotations.end(),
    [](const Eigen::Quaterniond & quaternion) {
      const double norm = quaternion.norm();
      return quaternion.coeffs().allFinite() && std::isfinite(norm) &&
             norm >= 1.0e-9;
    });
}

}  // namespace

CorrectedIkStatus parse_corrected_ik_status(std::string_view json_text)
{
  YAML::Node root;
  try {
    root = YAML::Load(std::string(json_text));
  } catch (const YAML::Exception &) {
    return rejectedStatus("status_json_malformed");
  }
  if (!root.IsMap()) {
    return rejectedStatus("status_json_malformed");
  }

  CorrectedIkStatus status;
  try {
    status.tracking_epoch = root["tracking_epoch"].as<std::uint64_t>();
  } catch (const YAML::Exception &) {
    return rejectedStatus("tracking_epoch_invalid");
  }
  if (status.tracking_epoch == 0) {
    return rejectedStatus("tracking_epoch_invalid");
  }

  try {
    if (!root["stream_valid"].as<bool>()) {
      return rejectedStatus("stream_invalid");
    }
  } catch (const YAML::Exception &) {
    return rejectedStatus("stream_invalid");
  }

  try {
    if (root["source_frame_id"].as<std::string>() != "pico") {
      return rejectedStatus("source_frame_id_invalid");
    }
  } catch (const YAML::Exception &) {
    return rejectedStatus("source_frame_id_invalid");
  }

  try {
    status.source_stamp_ns = root["source_stamp_ns"].as<std::int64_t>();
  } catch (const YAML::Exception &) {
    return rejectedStatus("source_stamp_invalid");
  }
  if (status.source_stamp_ns <= 0) {
    return rejectedStatus("source_stamp_invalid");
  }

  try {
    if (!root["ik_frame_valid"].as<bool>()) {
      return rejectedStatus("ik_frame_invalid");
    }
  } catch (const YAML::Exception &) {
    return rejectedStatus("ik_frame_invalid");
  }

  try {
    if (!root["left"]["corrected"].as<bool>()) {
      return rejectedStatus("left_not_corrected");
    }
  } catch (const YAML::Exception &) {
    return rejectedStatus("left_not_corrected");
  }

  try {
    if (!root["right"]["corrected"].as<bool>()) {
      return rejectedStatus("right_not_corrected");
    }
  } catch (const YAML::Exception &) {
    return rejectedStatus("right_not_corrected");
  }

  status.valid = true;
  return status;
}

TianjiTeleopBridgeCore::TianjiTeleopBridgeCore(
  std::size_t cache_capacity,
  const PicoPositionRetargetingConfig & position_retargeting)
: cache_capacity_(cache_capacity),
  position_retargeting_(position_retargeting)
{
  if (cache_capacity_ == 0) {
    throw std::invalid_argument("teleop cache capacity must be positive");
  }
}

void TianjiTeleopBridgeCore::reset() noexcept
{
  skeletons_.clear();
  statuses_.clear();
  completed_stamps_.clear();
  highest_skeleton_stamp_seen_ = 0;
  highest_status_stamp_seen_ = 0;
}

TianjiTeleopBridgeOutcome TianjiTeleopBridgeCore::ingest_skeleton(
  std::int64_t source_stamp_ns, const PicoSkeletonFrame & skeleton)
{
  if (source_stamp_ns <= 0) {
    return {{}, "source_stamp_invalid"};
  }
  const std::int64_t high_water_mark = std::max(
    highest_skeleton_stamp_seen_, highest_status_stamp_seen_);
  if (high_water_mark > source_stamp_ns &&
    high_water_mark - source_stamp_ns > kSourceTimestampRollbackResetNs)
  {
    reset();
  }
  if (completed(source_stamp_ns)) {
    return {{}, "duplicate_source_stamp"};
  }
  highest_skeleton_stamp_seen_ = std::max(
    highest_skeleton_stamp_seen_, source_stamp_ns);
  skeletons_[source_stamp_ns] = skeleton;
  if (statuses_.count(source_stamp_ns) != 0U) {
    return match(source_stamp_ns);
  }
  trimSkeletons();
  if (highest_status_stamp_seen_ > source_stamp_ns &&
    statuses_.count(source_stamp_ns) == 0U)
  {
    skeletons_.erase(source_stamp_ns);
    rememberCompleted(source_stamp_ns);
    return {{}, "status_cache_miss"};
  }
  return {{}, "awaiting_status"};
}

TianjiTeleopBridgeOutcome TianjiTeleopBridgeCore::ingest_status(
  const CorrectedIkStatus & status)
{
  if (!status.valid) {
    return {{}, status.rejection_reason.empty() ?
      "status_invalid" : status.rejection_reason};
  }
  const std::int64_t high_water_mark = std::max(
    highest_skeleton_stamp_seen_, highest_status_stamp_seen_);
  if (high_water_mark > status.source_stamp_ns &&
    high_water_mark - status.source_stamp_ns > kSourceTimestampRollbackResetNs)
  {
    reset();
  }
  if (completed(status.source_stamp_ns)) {
    return {{}, "duplicate_source_stamp"};
  }
  highest_status_stamp_seen_ = std::max(
    highest_status_stamp_seen_, status.source_stamp_ns);
  if (skeletons_.count(status.source_stamp_ns) != 0U) {
    statuses_[status.source_stamp_ns] = status;
    return match(status.source_stamp_ns);
  }
  if (highest_skeleton_stamp_seen_ > status.source_stamp_ns) {
    rememberCompleted(status.source_stamp_ns);
    return {{}, "skeleton_cache_miss"};
  }
  statuses_[status.source_stamp_ns] = status;
  trimStatuses();
  return {{}, "awaiting_skeleton"};
}

TianjiTeleopBridgeOutcome TianjiTeleopBridgeCore::match(
  std::int64_t source_stamp_ns)
{
  const auto skeleton_iterator = skeletons_.find(source_stamp_ns);
  const auto status_iterator = statuses_.find(source_stamp_ns);
  if (skeleton_iterator == skeletons_.end() || status_iterator == statuses_.end()) {
    return {{}, "pair_internal_error"};
  }

  const PicoShoulderMapResult mapped =
    map_pico_palms_to_tianji(
    skeleton_iterator->second, position_retargeting_);
  const CorrectedIkStatus status = status_iterator->second;
  skeletons_.erase(skeleton_iterator);
  statuses_.erase(status_iterator);
  rememberCompleted(source_stamp_ns);
  if (!mapped.valid) {
    return {{}, mapped.rejection_reason};
  }

  TianjiTeleopWireFrame frame;
  frame.sequence = next_sequence_++;
  frame.tracking_epoch = status.tracking_epoch;
  frame.source_timestamp_ns = source_stamp_ns;
  frame.left_target = mapped.left_target;
  frame.right_target = mapped.right_target;
  frame.left_arm_direction = mapped.left_arm_direction;
  frame.right_arm_direction = mapped.right_arm_direction;
  frame.upper_limb_skeleton = mapped.upper_limb_skeleton;
  return {frame, {}};
}

void TianjiTeleopBridgeCore::rememberCompleted(std::int64_t source_stamp_ns)
{
  if (completed(source_stamp_ns)) {
    return;
  }
  completed_stamps_.push_back(source_stamp_ns);
  while (completed_stamps_.size() > cache_capacity_) {
    completed_stamps_.pop_front();
  }
}

bool TianjiTeleopBridgeCore::completed(std::int64_t source_stamp_ns) const
{
  return std::find(
    completed_stamps_.begin(), completed_stamps_.end(), source_stamp_ns) !=
    completed_stamps_.end();
}

void TianjiTeleopBridgeCore::trimSkeletons()
{
  while (skeletons_.size() > cache_capacity_) {
    skeletons_.erase(skeletons_.begin());
  }
}

void TianjiTeleopBridgeCore::trimStatuses()
{
  while (statuses_.size() > cache_capacity_) {
    statuses_.erase(statuses_.begin());
  }
}

std::uint32_t tianji_teleop_crc32(const std::uint8_t * data, std::size_t size)
{
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^ (0xedb88320U &
        static_cast<std::uint32_t>(-static_cast<std::int32_t>(crc & 1U)));
    }
  }
  return crc ^ 0xffffffffU;
}

std::array<std::uint8_t, kTianjiTeleopPacketSize> encode_tianji_teleop_packet(
  const TianjiTeleopWireFrame & frame)
{
  std::array<std::uint8_t, kTianjiTeleopPacketSize> packet{};
  packet[0] = 'T';
  packet[1] = 'J';
  packet[2] = 'V';
  packet[3] = 'R';
  writeLittleEndian(packet, 4U, kTianjiTeleopProtocolVersion);
  writeLittleEndian(packet, 6U,
    static_cast<std::uint16_t>(kTianjiTeleopPacketSize));
  writeLittleEndian(packet, 8U, frame.sequence);
  writeLittleEndian(packet, 16U, frame.tracking_epoch);
  writeLittleEndian(packet, 24U, frame.source_timestamp_ns);
  writeLittleEndian(packet, 32U, frame.bridge_send_monotonic_ns);
  std::uint32_t flags = kTianjiTeleopRequiredFlags;
  if (frame.left_arm_direction.valid) {
    flags |= kTianjiTeleopLeftArmDirectionValid;
  }
  if (frame.right_arm_direction.valid) {
    flags |= kTianjiTeleopRightArmDirectionValid;
  }
  if (frame.upper_limb_skeleton.valid) {
    flags |= kTianjiTeleopUpperLimbSkeletonValid;
  }
  const bool rotations_valid = frame.upper_limb_skeleton.valid &&
    frame.upper_limb_skeleton.rotations_valid &&
    rotationsEncodable(frame.upper_limb_skeleton);
  if (rotations_valid) {
    flags |= kTianjiTeleopUpperLimbRotationsValid;
  }
  if (frame.user_button_pressed) {
    flags |= kTianjiTeleopUserButtonPressedFlag;
  }
  writeLittleEndian(packet, 40U, flags);
  writePose(packet, 44U, 68U, frame.left_target);
  writePose(packet, 100U, 124U, frame.right_target);
  writeVector(
    packet, 156U, frame.left_arm_direction.valid ?
    frame.left_arm_direction.direction : Eigen::Vector3d::Zero());
  writeVector(
    packet, 180U, frame.right_arm_direction.valid ?
    frame.right_arm_direction.direction : Eigen::Vector3d::Zero());
  for (std::size_t index = 0; index < kUpperLimbPointCount; ++index) {
    writeVector(packet, 204U + index * 24U,
      frame.upper_limb_skeleton.valid ?
      frame.upper_limb_skeleton.points[index] : Eigen::Vector3d::Zero());
  }
  for (std::size_t index = 0; index < kUpperLimbPointCount; ++index) {
    writeQuaternion(packet, 396U + index * 32U,
      rotations_valid ?
      frame.upper_limb_skeleton.rotations[index] :
      Eigen::Quaterniond::Identity());
  }
  writeLittleEndian(packet, 652U,
    tianji_teleop_crc32(packet.data(), 652U));
  return packet;
}

}  // namespace pico_bridge
