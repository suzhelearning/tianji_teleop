#ifndef IMU_ROS2__IM948_PROTOCOL_HPP_
#define IMU_ROS2__IM948_PROTOCOL_HPP_

#include <cstdint>
#include <optional>
#include <vector>

namespace imu_ros2
{

constexpr double kScaleAccel = 0.00478515625;
constexpr double kScaleQuat = 0.000030517578125;
constexpr double kScaleAngleSpeed = 0.06103515625;
constexpr double kScaleMag = 0.15106201171875;
constexpr double kDegToRad = 0.0174532925;
constexpr uint16_t kReportAccelerationWithGravity = 0x0002;
constexpr uint16_t kReportAngularVelocity = 0x0004;
constexpr uint16_t kReportMagneticField = 0x0008;
constexpr uint16_t kReportQuaternion = 0x0020;
constexpr uint16_t kDefaultReportTag =
  kReportAccelerationWithGravity | kReportAngularVelocity | kReportMagneticField |
  kReportQuaternion;

struct ImuSample
{
  uint32_t device_time_ms{};
  double linear_acceleration_x{};
  double linear_acceleration_y{};
  double linear_acceleration_z{};
  double angular_velocity_x{};
  double angular_velocity_y{};
  double angular_velocity_z{};
  double orientation_x{};
  double orientation_y{};
  double orientation_z{};
  double orientation_w{};
  bool has_magnetic_field{false};
  double magnetic_field_x{};  // Cx, uT
  double magnetic_field_y{};  // Cy, uT
  double magnetic_field_z{};  // Cz, uT
  double magnetic_field_magnitude{};  // |H| = sqrt(Cx^2 + Cy^2 + Cz^2), uT
};

struct BatteryStatus
{
  uint8_t charge_state{};
  uint8_t percentage{};
  uint16_t voltage_mv{};
};

struct Im948Event
{
  std::optional<ImuSample> imu;
  std::optional<BatteryStatus> battery;
  std::optional<uint8_t> command_ack;
};

class Im948Protocol
{
public:
  static std::vector<uint8_t> pack_command(
    const std::vector<uint8_t> & payload,
    uint8_t target_device_address = 0xff);

  static std::vector<uint8_t> make_set_parameters_command(
    uint8_t report_hz = 200,
    uint16_t report_tag = kDefaultReportTag,
    uint8_t target_device_address = 0xff,
    bool enable_compass = true);

  static std::vector<uint8_t> make_wake_command(uint8_t target_device_address = 0xff);

  static std::vector<uint8_t> make_get_device_status_command(
    uint8_t target_device_address = 0xff);

  static std::vector<uint8_t> make_zero_z_axis_command(
    uint8_t target_device_address = 0xff);

  static std::vector<uint8_t> make_clear_world_axes_command(
    uint8_t target_device_address = 0xff);

  static std::vector<uint8_t> make_restore_world_axes_command(
    uint8_t target_device_address = 0xff);

  static std::vector<uint8_t> make_clear_ins_position_command(
    uint8_t target_device_address = 0xff);

  static std::vector<uint8_t> make_enable_auto_report_command(
    uint8_t target_device_address = 0xff);

  explicit Im948Protocol(uint8_t target_device_address = 0xff);

  std::optional<Im948Event> input_byte(uint8_t byte);

private:
  enum class ReceiveState
  {
    kBegin,
    kAddress,
    kLength,
    kPayload,
    kChecksum,
    kEnd,
  };

  std::optional<Im948Event> parse_payload() const;
  std::optional<ImuSample> parse_imu_payload() const;
  std::optional<BatteryStatus> parse_device_status_payload() const;

  uint8_t target_device_address_;
  ReceiveState receive_state_{ReceiveState::kBegin};
  uint8_t checksum_{};
  uint8_t address_{};
  uint8_t expected_length_{};
  uint8_t received_checksum_{};
  std::vector<uint8_t> payload_;
};

}  // namespace imu_ros2

#endif  // IMU_ROS2__IM948_PROTOCOL_HPP_
