#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "imu_ros2/im948_protocol.hpp"
#include "imu_ros2/orientation_reference.hpp"

namespace
{

void append_i16(std::vector<uint8_t> & data, int16_t value)
{
  data.push_back(static_cast<uint8_t>(value & 0xff));
  data.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
}

std::vector<uint8_t> make_frame(uint8_t address, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> frame;
  frame.push_back(0x49);
  frame.push_back(address);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  uint8_t checksum = 0;
  for (size_t i = 1; i < frame.size(); ++i) {
    checksum = static_cast<uint8_t>(checksum + frame[i]);
  }
  frame.push_back(checksum);
  frame.push_back(0x4d);
  return frame;
}

}  // namespace

TEST(Im948Protocol, PacksCommandLikeOriginalDriver)
{
  const std::vector<uint8_t> command = {0x03};

  const auto packet = imu_ros2::Im948Protocol::pack_command(command, 0xff);

  ASSERT_EQ(packet.size(), 56u);
  for (size_t i = 0; i < 47; ++i) {
    EXPECT_EQ(packet[i], 0x00);
  }
  EXPECT_EQ(packet[47], 0xff);
  EXPECT_EQ(packet[48], 0x00);
  EXPECT_EQ(packet[49], 0xff);
  EXPECT_EQ(packet[50], 0x49);
  EXPECT_EQ(packet[51], 0xff);
  EXPECT_EQ(packet[52], 0x01);
  EXPECT_EQ(packet[53], 0x03);
  EXPECT_EQ(packet[54], 0x03);
  EXPECT_EQ(packet[55], 0x4d);
}

TEST(Im948Protocol, PacksWorldAxesClearCommand)
{
  const auto packet = imu_ros2::Im948Protocol::make_clear_world_axes_command(0xff);

  ASSERT_EQ(packet.size(), 56u);
  EXPECT_EQ(packet[50], 0x49);
  EXPECT_EQ(packet[51], 0xff);
  EXPECT_EQ(packet[52], 0x01);
  EXPECT_EQ(packet[53], 0x06);
  EXPECT_EQ(packet[54], 0x06);
  EXPECT_EQ(packet[55], 0x4d);
}

TEST(Im948Protocol, PacksZAxisZeroCommand)
{
  const auto packet = imu_ros2::Im948Protocol::make_zero_z_axis_command(0xff);

  ASSERT_EQ(packet.size(), 56u);
  EXPECT_EQ(packet[50], 0x49);
  EXPECT_EQ(packet[51], 0xff);
  EXPECT_EQ(packet[52], 0x01);
  EXPECT_EQ(packet[53], 0x05);
  EXPECT_EQ(packet[54], 0x05);
  EXPECT_EQ(packet[55], 0x4d);
}

TEST(Im948Protocol, PacksWorldAxesRestoreCommand)
{
  const auto packet = imu_ros2::Im948Protocol::make_restore_world_axes_command(0xff);

  ASSERT_EQ(packet.size(), 56u);
  EXPECT_EQ(packet[50], 0x49);
  EXPECT_EQ(packet[51], 0xff);
  EXPECT_EQ(packet[52], 0x01);
  EXPECT_EQ(packet[53], 0x08);
  EXPECT_EQ(packet[54], 0x08);
  EXPECT_EQ(packet[55], 0x4d);
}

TEST(Im948Protocol, PacksInsPositionClearCommand)
{
  const auto packet = imu_ros2::Im948Protocol::make_clear_ins_position_command(0xff);

  ASSERT_EQ(packet.size(), 56u);
  EXPECT_EQ(packet[50], 0x49);
  EXPECT_EQ(packet[51], 0xff);
  EXPECT_EQ(packet[52], 0x01);
  EXPECT_EQ(packet[53], 0x13);
  EXPECT_EQ(packet[54], 0x13);
  EXPECT_EQ(packet[55], 0x4d);
}

TEST(Im948Protocol, PacksGetDeviceStatusCommand)
{
  const auto packet = imu_ros2::Im948Protocol::make_get_device_status_command(0xff);

  ASSERT_EQ(packet.size(), 56u);
  EXPECT_EQ(packet[50], 0x49);
  EXPECT_EQ(packet[51], 0xff);
  EXPECT_EQ(packet[52], 0x01);
  EXPECT_EQ(packet[53], 0x10);
  EXPECT_EQ(packet[54], 0x10);
  EXPECT_EQ(packet[55], 0x4d);
}

TEST(Im948Protocol, DefaultsSetParametersTo200Hz)
{
  const auto packet = imu_ros2::Im948Protocol::make_set_parameters_command();

  ASSERT_EQ(packet.size(), 66u);
  EXPECT_EQ(packet[50], 0x49);
  EXPECT_EQ(packet[51], 0xff);
  EXPECT_EQ(packet[52], 0x0b);
  EXPECT_EQ(packet[53], 0x12);
  EXPECT_EQ(packet[57], 0x05);
  EXPECT_EQ(packet[58], 200);
  EXPECT_EQ(packet[62], 0x2e);
  EXPECT_EQ(packet[63], 0x00);
  EXPECT_EQ(packet[65], 0x4d);
}

TEST(Im948Protocol, CanDisableCompassFusion)
{
  const auto packet = imu_ros2::Im948Protocol::make_set_parameters_command(
    200,
    imu_ros2::kDefaultReportTag,
    0xff,
    false);

  ASSERT_EQ(packet.size(), 66u);
  EXPECT_EQ(packet[57], 0x04);
}

TEST(Im948Protocol, CanUseVendorSixAxisReportTag)
{
  const auto packet = imu_ros2::Im948Protocol::make_set_parameters_command(
    60,
    imu_ros2::kReportAccelerationWithGravity | imu_ros2::kReportAngularVelocity |
    imu_ros2::kReportQuaternion,
    0xff,
    false);

  ASSERT_EQ(packet.size(), 66u);
  EXPECT_EQ(packet[57], 0x04);
  EXPECT_EQ(packet[58], 60);
  EXPECT_EQ(packet[62], 0x26);
  EXPECT_EQ(packet[63], 0x00);
}

TEST(Im948Protocol, ParsesSubscribedImuReport)
{
  imu_ros2::Im948Protocol protocol;
  std::vector<uint8_t> payload = {
    0x11,
    0x26, 0x00,
    0x78, 0x56, 0x34, 0x12,
  };
  append_i16(payload, 1000);
  append_i16(payload, -1000);
  append_i16(payload, 2000);
  append_i16(payload, 100);
  append_i16(payload, -200);
  append_i16(payload, 300);
  append_i16(payload, 16384);
  append_i16(payload, 1000);
  append_i16(payload, -1000);
  append_i16(payload, 500);

  std::optional<imu_ros2::Im948Event> event;
  for (const auto byte : make_frame(1, payload)) {
    auto parsed = protocol.input_byte(byte);
    if (parsed) {
      event = parsed;
    }
  }

  ASSERT_TRUE(event.has_value());
  ASSERT_TRUE(event->imu.has_value());
  const auto & sample = *event->imu;
  EXPECT_EQ(sample.device_time_ms, 0x12345678u);
  EXPECT_NEAR(sample.linear_acceleration_x, 1000.0 * imu_ros2::kScaleAccel, 1e-6);
  EXPECT_NEAR(sample.linear_acceleration_y, -1000.0 * imu_ros2::kScaleAccel, 1e-6);
  EXPECT_NEAR(sample.linear_acceleration_z, 2000.0 * imu_ros2::kScaleAccel, 1e-6);
  EXPECT_NEAR(
    sample.angular_velocity_x,
    100.0 * imu_ros2::kScaleAngleSpeed * imu_ros2::kDegToRad,
    1e-6);
  EXPECT_NEAR(
    sample.angular_velocity_y,
    -200.0 * imu_ros2::kScaleAngleSpeed * imu_ros2::kDegToRad,
    1e-6);
  EXPECT_NEAR(
    sample.angular_velocity_z,
    300.0 * imu_ros2::kScaleAngleSpeed * imu_ros2::kDegToRad,
    1e-6);
  EXPECT_NEAR(sample.orientation_w, 16384.0 * imu_ros2::kScaleQuat, 1e-6);
  EXPECT_NEAR(sample.orientation_x, 1000.0 * imu_ros2::kScaleQuat, 1e-6);
  EXPECT_NEAR(sample.orientation_y, -1000.0 * imu_ros2::kScaleQuat, 1e-6);
  EXPECT_NEAR(sample.orientation_z, 500.0 * imu_ros2::kScaleQuat, 1e-6);
  EXPECT_FALSE(sample.has_magnetic_field);
}

TEST(Im948Protocol, ParsesMagneticFieldAndMagnitude)
{
  imu_ros2::Im948Protocol protocol;
  std::vector<uint8_t> payload = {
    0x11,
    0x2e, 0x00,
    0x78, 0x56, 0x34, 0x12,
  };
  append_i16(payload, 1000);
  append_i16(payload, -1000);
  append_i16(payload, 2000);
  append_i16(payload, 100);
  append_i16(payload, -200);
  append_i16(payload, 300);
  append_i16(payload, 199);
  append_i16(payload, 265);
  append_i16(payload, 0);
  append_i16(payload, 16384);
  append_i16(payload, 1000);
  append_i16(payload, -1000);
  append_i16(payload, 500);

  std::optional<imu_ros2::Im948Event> event;
  for (const auto byte : make_frame(1, payload)) {
    auto parsed = protocol.input_byte(byte);
    if (parsed) {
      event = parsed;
    }
  }

  ASSERT_TRUE(event.has_value());
  ASSERT_TRUE(event->imu.has_value());
  const auto & sample = *event->imu;
  EXPECT_TRUE(sample.has_magnetic_field);
  EXPECT_NEAR(sample.magnetic_field_x, 199.0 * imu_ros2::kScaleMag, 1e-3);
  EXPECT_NEAR(sample.magnetic_field_y, 265.0 * imu_ros2::kScaleMag, 1e-3);
  EXPECT_NEAR(sample.magnetic_field_z, 0.0, 1e-6);
  EXPECT_NEAR(sample.magnetic_field_magnitude, 50.0, 0.1);
}

TEST(Im948Protocol, ParsesDeviceStatusBatteryReport)
{
  imu_ros2::Im948Protocol protocol;
  std::vector<uint8_t> payload(33, 0);
  payload[0] = 0x10;
  payload[5] = 200;
  payload[9] = 0x26;
  payload[10] = 0x00;
  payload[11] = 1;
  payload[12] = 87;
  payload[13] = 0x6a;
  payload[14] = 0x0e;

  std::optional<imu_ros2::Im948Event> event;
  for (const auto byte : make_frame(1, payload)) {
    auto parsed = protocol.input_byte(byte);
    if (parsed) {
      event = parsed;
    }
  }

  ASSERT_TRUE(event.has_value());
  ASSERT_TRUE(event->battery.has_value());
  EXPECT_FALSE(event->imu.has_value());
  EXPECT_EQ(event->battery->charge_state, 1u);
  EXPECT_EQ(event->battery->percentage, 87u);
  EXPECT_EQ(event->battery->voltage_mv, 3690u);
}

TEST(Im948Protocol, ParsesWorldAxesClearAcknowledgement)
{
  imu_ros2::Im948Protocol protocol;

  std::optional<imu_ros2::Im948Event> event;
  for (const auto byte : make_frame(1, {0x06})) {
    auto parsed = protocol.input_byte(byte);
    if (parsed) {
      event = parsed;
    }
  }

  ASSERT_TRUE(event.has_value());
  ASSERT_TRUE(event->command_ack.has_value());
  EXPECT_EQ(*event->command_ack, 0x06u);
  EXPECT_FALSE(event->imu.has_value());
  EXPECT_FALSE(event->battery.has_value());
}

TEST(Im948Protocol, RejectsBadChecksum)
{
  imu_ros2::Im948Protocol protocol;
  auto frame = make_frame(1, {0x03});
  frame[frame.size() - 2] ^= 0xff;

  for (const auto byte : frame) {
    EXPECT_FALSE(protocol.input_byte(byte).has_value());
  }
}

TEST(OrientationReference, PublishesNormalizedHardwareQuaternion)
{
  imu_ros2::OrientationReference reference;
  imu_ros2::Quaternion output;
  const imu_ros2::Quaternion yaw_90{
    0.0,
    0.0,
    std::sin(M_PI / 4.0),
    std::cos(M_PI / 4.0)};

  ASSERT_TRUE(reference.apply(yaw_90, output));

  EXPECT_NEAR(output.x, 0.0, 1e-9);
  EXPECT_NEAR(output.y, 0.0, 1e-9);
  EXPECT_NEAR(output.z, std::sin(M_PI / 4.0), 1e-9);
  EXPECT_NEAR(output.w, std::cos(M_PI / 4.0), 1e-9);
}

TEST(OrientationReference, ForcesPositiveWOnFirstValidFrame)
{
  imu_ros2::OrientationReference reference(true, false);
  imu_ros2::Quaternion output;
  const imu_ros2::Quaternion yaw_90_negative_w{
    0.0,
    0.0,
    -std::sin(M_PI / 4.0),
    -std::cos(M_PI / 4.0)};

  ASSERT_TRUE(reference.apply(yaw_90_negative_w, output));

  EXPECT_NEAR(output.x, 0.0, 1e-9);
  EXPECT_NEAR(output.y, 0.0, 1e-9);
  EXPECT_NEAR(output.z, std::sin(M_PI / 4.0), 1e-9);
  EXPECT_NEAR(output.w, std::cos(M_PI / 4.0), 1e-9);
}

TEST(OrientationReference, KeepsQuaternionContinuousAcrossFrames)
{
  imu_ros2::OrientationReference reference(false, true);
  imu_ros2::Quaternion output1;
  imu_ros2::Quaternion output2;
  const imu_ros2::Quaternion first{
    0.0,
    0.0,
    std::sin(M_PI / 6.0),
    std::cos(M_PI / 6.0)};
  const imu_ros2::Quaternion second_negative = {
    -first.x,
    -first.y,
    -first.z,
    -first.w};

  ASSERT_TRUE(reference.apply(first, output1));
  ASSERT_TRUE(reference.apply(second_negative, output2));

  EXPECT_NEAR(output2.x, output1.x, 1e-9);
  EXPECT_NEAR(output2.y, output1.y, 1e-9);
  EXPECT_NEAR(output2.z, output1.z, 1e-9);
  EXPECT_NEAR(output2.w, output1.w, 1e-9);
}

TEST(OrientationReference, RejectsInvalidIm948QuaternionSentinel)
{
  imu_ros2::OrientationReference reference;
  imu_ros2::Quaternion relative;
  const imu_ros2::Quaternion invalid{
    -imu_ros2::kScaleQuat,
    -imu_ros2::kScaleQuat,
    -imu_ros2::kScaleQuat,
    -imu_ros2::kScaleQuat};

  EXPECT_FALSE(reference.apply(invalid, relative));
}
