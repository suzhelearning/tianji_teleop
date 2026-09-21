#include "imu_ros2/im948_protocol.hpp"

#include <cmath>
#include <cstddef>

namespace imu_ros2
{

namespace
{

constexpr uint8_t kFrameBegin = 0x49;
constexpr uint8_t kFrameEnd = 0x4d;
constexpr uint8_t kMaxRxPayloadSize = 73;
constexpr uint8_t kMaxTxPayloadSize = 31;
constexpr size_t kPreambleSize = 50;

uint8_t checksum(const std::vector<uint8_t> & data, size_t begin, size_t end)
{
  uint8_t sum = 0;
  for (size_t i = begin; i < end; ++i) {
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  return sum;
}

uint16_t read_u16_le(const std::vector<uint8_t> & data, size_t offset)
{
  return static_cast<uint16_t>(
    static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8));
}

int16_t read_i16_le(const std::vector<uint8_t> & data, size_t offset)
{
  return static_cast<int16_t>(read_u16_le(data, offset));
}

uint32_t read_u32_le(const std::vector<uint8_t> & data, size_t offset)
{
  return static_cast<uint32_t>(data[offset]) |
         (static_cast<uint32_t>(data[offset + 1]) << 8) |
         (static_cast<uint32_t>(data[offset + 2]) << 16) |
         (static_cast<uint32_t>(data[offset + 3]) << 24);
}

bool consume_i16_triplet(
  const std::vector<uint8_t> & data,
  size_t & offset,
  int16_t & x,
  int16_t & y,
  int16_t & z)
{
  if (offset + 6 > data.size()) {
    return false;
  }
  x = read_i16_le(data, offset);
  y = read_i16_le(data, offset + 2);
  z = read_i16_le(data, offset + 4);
  offset += 6;
  return true;
}

}  // namespace

std::vector<uint8_t> Im948Protocol::pack_command(
  const std::vector<uint8_t> & payload,
  uint8_t target_device_address)
{
  if (payload.empty() || payload.size() > kMaxTxPayloadSize) {
    return {};
  }

  std::vector<uint8_t> packet(kPreambleSize, 0x00);
  packet[47] = 0xff;
  packet[48] = 0x00;
  packet[49] = 0xff;
  packet.push_back(kFrameBegin);
  packet.push_back(target_device_address);
  packet.push_back(static_cast<uint8_t>(payload.size()));
  packet.insert(packet.end(), payload.begin(), payload.end());
  packet.push_back(checksum(packet, kPreambleSize + 1, packet.size()));
  packet.push_back(kFrameEnd);
  return packet;
}

std::vector<uint8_t> Im948Protocol::make_set_parameters_command(
  uint8_t report_hz,
  uint16_t report_tag,
  uint8_t target_device_address,
  bool enable_compass)
{
  constexpr uint8_t kBarometerFilter = 2;
  const uint8_t filter_and_compass =
    static_cast<uint8_t>((kBarometerFilter << 1) | (enable_compass ? 1 : 0));
  const std::vector<uint8_t> payload = {
    0x12,
    5,
    255,
    0,
    filter_and_compass,
    report_hz,
    1,
    3,
    5,
    static_cast<uint8_t>(report_tag & 0xff),
    static_cast<uint8_t>((report_tag >> 8) & 0xff),
  };
  return pack_command(payload, target_device_address);
}

std::vector<uint8_t> Im948Protocol::make_wake_command(uint8_t target_device_address)
{
  return pack_command({0x03}, target_device_address);
}

std::vector<uint8_t> Im948Protocol::make_get_device_status_command(uint8_t target_device_address)
{
  return pack_command({0x10}, target_device_address);
}

std::vector<uint8_t> Im948Protocol::make_zero_z_axis_command(uint8_t target_device_address)
{
  return pack_command({0x05}, target_device_address);
}

std::vector<uint8_t> Im948Protocol::make_clear_world_axes_command(
  uint8_t target_device_address)
{
  return pack_command({0x06}, target_device_address);
}

std::vector<uint8_t> Im948Protocol::make_restore_world_axes_command(
  uint8_t target_device_address)
{
  return pack_command({0x08}, target_device_address);
}

std::vector<uint8_t> Im948Protocol::make_clear_ins_position_command(
  uint8_t target_device_address)
{
  return pack_command({0x13}, target_device_address);
}

std::vector<uint8_t> Im948Protocol::make_enable_auto_report_command(
  uint8_t target_device_address)
{
  return pack_command({0x19}, target_device_address);
}

Im948Protocol::Im948Protocol(uint8_t target_device_address)
: target_device_address_(target_device_address)
{
}

std::optional<Im948Event> Im948Protocol::input_byte(uint8_t byte)
{
  switch (receive_state_) {
    case ReceiveState::kBegin:
      if (byte == kFrameBegin) {
        payload_.clear();
        checksum_ = 0;
        address_ = 0;
        expected_length_ = 0;
        received_checksum_ = 0;
        receive_state_ = ReceiveState::kAddress;
      }
      break;
    case ReceiveState::kAddress:
      if (byte == 0xff) {
        receive_state_ = ReceiveState::kBegin;
        break;
      }
      address_ = byte;
      checksum_ = static_cast<uint8_t>(checksum_ + byte);
      receive_state_ = ReceiveState::kLength;
      break;
    case ReceiveState::kLength:
      if (byte == 0 || byte > kMaxRxPayloadSize) {
        receive_state_ = ReceiveState::kBegin;
        break;
      }
      expected_length_ = byte;
      checksum_ = static_cast<uint8_t>(checksum_ + byte);
      payload_.clear();
      payload_.reserve(expected_length_);
      receive_state_ = ReceiveState::kPayload;
      break;
    case ReceiveState::kPayload:
      payload_.push_back(byte);
      checksum_ = static_cast<uint8_t>(checksum_ + byte);
      if (payload_.size() >= expected_length_) {
        receive_state_ = ReceiveState::kChecksum;
      }
      break;
    case ReceiveState::kChecksum:
      received_checksum_ = byte;
      receive_state_ = checksum_ == received_checksum_ ? ReceiveState::kEnd : ReceiveState::kBegin;
      break;
    case ReceiveState::kEnd:
      receive_state_ = ReceiveState::kBegin;
      if (
        byte == kFrameEnd &&
        (target_device_address_ == 0xff || target_device_address_ == address_))
      {
        return parse_payload();
      }
      break;
  }

  return std::nullopt;
}

std::optional<Im948Event> Im948Protocol::parse_payload() const
{
  if (payload_.empty()) {
    return std::nullopt;
  }

  Im948Event event;
  if (payload_[0] == 0x11) {
    event.imu = parse_imu_payload();
  } else if (payload_[0] == 0x10) {
    event.battery = parse_device_status_payload();
  } else if (payload_[0] == 0x05 || payload_[0] == 0x06 || payload_[0] == 0x08) {
    event.command_ack = payload_[0];
  }

  if (!event.imu && !event.battery && !event.command_ack) {
    return std::nullopt;
  }

  return event;
}

std::optional<BatteryStatus> Im948Protocol::parse_device_status_payload() const
{
  if (payload_.size() < 15 || payload_[0] != 0x10) {
    return std::nullopt;
  }

  BatteryStatus status;
  status.charge_state = payload_[11];
  status.percentage = payload_[12];
  status.voltage_mv = read_u16_le(payload_, 13);
  return status;
}

std::optional<ImuSample> Im948Protocol::parse_imu_payload() const
{
  if (payload_.size() < 7 || payload_[0] != 0x11) {
    return std::nullopt;
  }

  const uint16_t report_tag = read_u16_le(payload_, 1);
  ImuSample sample;
  sample.device_time_ms = read_u32_le(payload_, 3);

  size_t offset = 7;
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;

  if ((report_tag & 0x0001) != 0) {
    if (!consume_i16_triplet(payload_, offset, x, y, z)) {
      return std::nullopt;
    }
  }

  if ((report_tag & kReportAccelerationWithGravity) != 0) {
    if (!consume_i16_triplet(payload_, offset, x, y, z)) {
      return std::nullopt;
    }
    sample.linear_acceleration_x = static_cast<double>(x) * kScaleAccel;
    sample.linear_acceleration_y = static_cast<double>(y) * kScaleAccel;
    sample.linear_acceleration_z = static_cast<double>(z) * kScaleAccel;
  }

  if ((report_tag & kReportAngularVelocity) != 0) {
    if (!consume_i16_triplet(payload_, offset, x, y, z)) {
      return std::nullopt;
    }
    sample.angular_velocity_x = static_cast<double>(x) * kScaleAngleSpeed * kDegToRad;
    sample.angular_velocity_y = static_cast<double>(y) * kScaleAngleSpeed * kDegToRad;
    sample.angular_velocity_z = static_cast<double>(z) * kScaleAngleSpeed * kDegToRad;
  }

  if ((report_tag & kReportMagneticField) != 0) {
    if (!consume_i16_triplet(payload_, offset, x, y, z)) {
      return std::nullopt;
    }
    sample.has_magnetic_field = true;
    sample.magnetic_field_x = static_cast<double>(x) * kScaleMag;
    sample.magnetic_field_y = static_cast<double>(y) * kScaleMag;
    sample.magnetic_field_z = static_cast<double>(z) * kScaleMag;
    sample.magnetic_field_magnitude = std::sqrt(
      sample.magnetic_field_x * sample.magnetic_field_x +
      sample.magnetic_field_y * sample.magnetic_field_y +
      sample.magnetic_field_z * sample.magnetic_field_z);
  }
  if ((report_tag & 0x0010) != 0) {
    offset += 8;
  }
  if (offset > payload_.size()) {
    return std::nullopt;
  }

  if ((report_tag & kReportQuaternion) != 0) {
    if (offset + 8 > payload_.size()) {
      return std::nullopt;
    }
    sample.orientation_w = static_cast<double>(read_i16_le(payload_, offset)) * kScaleQuat;
    sample.orientation_x = static_cast<double>(read_i16_le(payload_, offset + 2)) * kScaleQuat;
    sample.orientation_y = static_cast<double>(read_i16_le(payload_, offset + 4)) * kScaleQuat;
    sample.orientation_z = static_cast<double>(read_i16_le(payload_, offset + 6)) * kScaleQuat;
  }

  return sample;
}

}  // namespace imu_ros2
