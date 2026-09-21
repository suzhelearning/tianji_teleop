#include <array>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "imu_ros2/im948_protocol.hpp"
#include "imu_ros2/orientation_reference.hpp"

namespace
{

constexpr double kMicroteslaToTesla = 1e-6;

class SerialPort
{
public:
  SerialPort(const std::string & port, int baudrate)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("open failed: " + std::string(std::strerror(errno)));
    }

    termios options{};
    if (::tcgetattr(fd_, &options) != 0) {
      close();
      throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
    }

    ::cfmakeraw(&options);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    const speed_t speed = to_speed_t(baudrate);
    if (::cfsetispeed(&options, speed) != 0 || ::cfsetospeed(&options, speed) != 0) {
      close();
      throw std::runtime_error("unsupported baudrate: " + std::to_string(baudrate));
    }

    if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
      close();
      throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
    }

    ::tcflush(fd_, TCIOFLUSH);
  }

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  ~SerialPort()
  {
    close();
  }

  void write_all(const std::vector<uint8_t> & data)
  {
    size_t written = 0;
    while (written < data.size()) {
      const ssize_t result = ::write(fd_, data.data() + written, data.size() - written);
      if (result > 0) {
        written += static_cast<size_t>(result);
        continue;
      }
      if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        wait_writable();
        continue;
      }
      throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
    }
    ::tcdrain(fd_);
  }

  ssize_t read_some(uint8_t * buffer, size_t buffer_size)
  {
    const ssize_t result = ::read(fd_, buffer, buffer_size);
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return 0;
    }
    if (result < 0) {
      throw std::runtime_error("serial read failed: " + std::string(std::strerror(errno)));
    }
    return result;
  }

private:
  static speed_t to_speed_t(int baudrate)
  {
    switch (baudrate) {
      case 9600:
        return B9600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      case 460800:
        return B460800;
      case 921600:
        return B921600;
      default:
        throw std::runtime_error("unsupported baudrate: " + std::to_string(baudrate));
    }
  }

  void wait_writable()
  {
    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(fd_, &write_fds);
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    const int result = ::select(fd_ + 1, nullptr, &write_fds, nullptr, &timeout);
    if (result < 0) {
      throw std::runtime_error("select failed: " + std::string(std::strerror(errno)));
    }
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  int fd_{-1};
};

sensor_msgs::msg::Imu to_msg(
  const imu_ros2::ImuSample & sample,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  imu_ros2::OrientationReference & orientation_reference,
  bool & has_valid_orientation)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  const imu_ros2::Quaternion raw_orientation{
    sample.orientation_x,
    sample.orientation_y,
    sample.orientation_z,
    sample.orientation_w};
  imu_ros2::Quaternion output_orientation;
  const bool has_orientation = orientation_reference.apply(raw_orientation, output_orientation);

  if (!has_orientation) {
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;
  } else {
    msg.orientation.x = output_orientation.x;
    msg.orientation.y = output_orientation.y;
    msg.orientation.z = output_orientation.z;
    msg.orientation.w = output_orientation.w;
    has_valid_orientation = true;
  }
  msg.linear_acceleration.x = sample.linear_acceleration_x;
  msg.linear_acceleration.y = sample.linear_acceleration_y;
  msg.linear_acceleration.z = sample.linear_acceleration_z;
  msg.angular_velocity.x = sample.angular_velocity_x;
  msg.angular_velocity.y = sample.angular_velocity_y;
  msg.angular_velocity.z = sample.angular_velocity_z;
  return msg;
}

sensor_msgs::msg::BatteryState to_msg(
  const imu_ros2::BatteryStatus & status,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.voltage = static_cast<float>(status.voltage_mv) / 1000.0F;
  msg.percentage = static_cast<float>(status.percentage) / 100.0F;
  msg.present = true;
  switch (status.charge_state) {
    case 1:
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
      break;
    case 2:
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
      break;
    default:
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
      break;
  }
  return msg;
}

sensor_msgs::msg::MagneticField to_magnetic_field_msg(
  const imu_ros2::ImuSample & sample,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::MagneticField msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.magnetic_field.x = sample.magnetic_field_x * kMicroteslaToTesla;
  msg.magnetic_field.y = sample.magnetic_field_y * kMicroteslaToTesla;
  msg.magnetic_field.z = sample.magnetic_field_z * kMicroteslaToTesla;
  return msg;
}

std_msgs::msg::Float64 to_magnetic_magnitude_msg(const imu_ros2::ImuSample & sample)
{
  std_msgs::msg::Float64 msg;
  msg.data = sample.magnetic_field_magnitude;
  return msg;
}

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("imu_node"),
    protocol_(static_cast<uint8_t>(declare_parameter<int>("target_address", 255)))
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baudrate_ = declare_parameter<int>("baudrate", 460800);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_node");
    const auto topic = declare_parameter<std::string>("topic", "imuData_raw");
    const auto battery_topic = declare_parameter<std::string>("battery_topic", "imuBattery");
    const auto magnetic_topic =
      declare_parameter<std::string>("magnetic_topic", "imuMagneticField");
    const auto magnetic_magnitude_topic =
      declare_parameter<std::string>("magnetic_magnitude_topic", "magnetic_field_magnitude");
    const auto zero_z_axis_service =
      declare_parameter<std::string>("zero_z_axis_service", "zero_z_axis");
    const auto clear_world_axes_service =
      declare_parameter<std::string>("clear_world_axes_service", "clear_world_axes");
    const auto restore_world_axes_service =
      declare_parameter<std::string>("restore_world_axes_service", "restore_world_axes");
    report_hz_ = declare_parameter<int>("report_hz", 200);
    battery_query_period_ms_ = declare_parameter<int>("battery_query_period_ms", 5000);
    log_magnetic_field_status_ = declare_parameter<bool>("log_magnetic_field_status", true);
    force_positive_w_ = declare_parameter<bool>("force_positive_w", true);
    use_quaternion_continuity_ = declare_parameter<bool>("use_quaternion_continuity", true);
    restore_world_axes_ = declare_parameter<bool>("restore_world_axes", false);
    clear_world_axes_ = declare_parameter<bool>("clear_world_axes", false);
    clear_ins_position_ = declare_parameter<bool>("clear_ins_position", true);
    enable_compass_ = declare_parameter<bool>("enable_compass", true);
    use_reliable_qos_ = declare_parameter<bool>("use_reliable_qos", true);
    command_ack_timeout_ms_ = declare_parameter<int>("command_ack_timeout_ms", 2000);
    target_address_ = static_cast<uint8_t>(get_parameter("target_address").as_int());
    if (command_ack_timeout_ms_ <= 0) {
      throw std::runtime_error("command_ack_timeout_ms must be positive");
    }
    orientation_reference_.configure(force_positive_w_, use_quaternion_continuity_);
    io_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    const auto imu_qos = use_reliable_qos_ ? rclcpp::QoS(10) : rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic, imu_qos);
    battery_publisher_ =
      create_publisher<sensor_msgs::msg::BatteryState>(battery_topic, rclcpp::QoS(10));
    magnetic_field_publisher_ =
      create_publisher<sensor_msgs::msg::MagneticField>(magnetic_topic, imu_qos);
    magnetic_magnitude_publisher_ =
      create_publisher<std_msgs::msg::Float64>(magnetic_magnitude_topic, imu_qos);
    zero_z_axis_service_ = create_service<std_srvs::srv::Trigger>(
      zero_z_axis_service,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        send_device_command_service(
          imu_ros2::Im948Protocol::make_zero_z_axis_command(target_address_),
          0x05,
          "z axis zero command sent",
          response);
      }, rmw_qos_profile_services_default, command_callback_group_);
    clear_world_axes_service_ = create_service<std_srvs::srv::Trigger>(
      clear_world_axes_service,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        send_device_command_service(
          imu_ros2::Im948Protocol::make_clear_world_axes_command(target_address_),
          0x06,
          "world xyz axes clear command sent",
          response);
      }, rmw_qos_profile_services_default, command_callback_group_);
    restore_world_axes_service_ = create_service<std_srvs::srv::Trigger>(
      restore_world_axes_service,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        send_device_command_service(
          imu_ros2::Im948Protocol::make_restore_world_axes_command(target_address_),
          0x08,
          "world axes restore command sent",
          response);
      }, rmw_qos_profile_services_default, command_callback_group_);
    serial_ = std::make_unique<SerialPort>(port_, baudrate_);

    send_startup_commands();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(2),
      [this]() {
        poll_serial();
      }, io_callback_group_);
    if (log_magnetic_field_status_) {
      magnetic_log_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          log_magnetic_field_status();
        }, io_callback_group_);
    }
    if (battery_query_period_ms_ > 0) {
      battery_query_timer_ = create_wall_timer(
        std::chrono::milliseconds(battery_query_period_ms_),
        [this]() {
          query_battery_status();
        }, io_callback_group_);
    }

    RCLCPP_INFO(
      get_logger(),
      "IM948 node reading %s at %d baud, publishing %s, %s, %s and %s, report_hz=%d, "
      "enable_compass=%s, imu_qos=%s",
      port_.c_str(),
      baudrate_,
      topic.c_str(),
      battery_topic.c_str(),
      magnetic_topic.c_str(),
      magnetic_magnitude_topic.c_str(),
      report_hz_,
      enable_compass_ ? "true" : "false",
      use_reliable_qos_ ? "reliable" : "best_effort");
    RCLCPP_INFO(
      get_logger(),
      "battery query %s (period_ms=%d)",
      battery_query_period_ms_ > 0 ? "enabled" : "disabled",
      battery_query_period_ms_);
  }

private:
  void send_startup_commands()
  {
    serial_->write_all(imu_ros2::Im948Protocol::make_set_parameters_command(
        static_cast<uint8_t>(report_hz_),
        imu_ros2::kDefaultReportTag,
        target_address_,
        enable_compass_));
    serial_->write_all(imu_ros2::Im948Protocol::make_wake_command(target_address_));
    if (restore_world_axes_) {
      serial_->write_all(imu_ros2::Im948Protocol::make_restore_world_axes_command(target_address_));
    }
    if (clear_world_axes_) {
      serial_->write_all(imu_ros2::Im948Protocol::make_clear_world_axes_command(target_address_));
    }
    if (clear_ins_position_) {
      serial_->write_all(imu_ros2::Im948Protocol::make_clear_ins_position_command(target_address_));
    }
    serial_->write_all(imu_ros2::Im948Protocol::make_enable_auto_report_command(target_address_));
    if (battery_query_period_ms_ > 0) {
      query_battery_status();
    }
  }

  void query_battery_status()
  {
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    serial_->write_all(imu_ros2::Im948Protocol::make_get_device_status_command(target_address_));
  }

  void log_magnetic_field_status()
  {
    if (!has_magnetic_field_magnitude_) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "[%s] magnetic field |H|=%.2f uT",
      frame_id_.c_str(),
      last_magnetic_field_magnitude_);
  }

  void send_device_command_service(
    const std::vector<uint8_t> & command,
    uint8_t command_id,
    const std::string & success_message,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    try {
      std::unique_lock<std::mutex> command_lock(command_mutex_);
      uint64_t ack_sequence = 0;
      {
        std::lock_guard<std::mutex> ack_lock(command_ack_mutex_);
        ack_sequence = command_ack_sequences_[command_id];
      }
      {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        serial_->write_all(command);
      }

      std::unique_lock<std::mutex> ack_lock(command_ack_mutex_);
      const bool acknowledged = command_ack_cv_.wait_for(
        ack_lock,
        std::chrono::milliseconds(command_ack_timeout_ms_),
        [this, command_id, ack_sequence]() {
          return command_ack_sequences_[command_id] > ack_sequence;
        });
      if (!acknowledged) {
        response->success = false;
        response->message =
          "timeout waiting for IM900 command ack 0x" + command_hex(command_id) +
          " on " + port_;
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }

      response->success = true;
      response->message = success_message + " confirmed (0x" + command_hex(command_id) + ")";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
  }

  static std::string command_hex(uint8_t command_id)
  {
    constexpr char kHex[] = "0123456789ABCDEF";
    std::string value(2, '0');
    value[0] = kHex[(command_id >> 4) & 0x0f];
    value[1] = kHex[command_id & 0x0f];
    return value;
  }

  void record_command_ack(uint8_t command_id)
  {
    {
      std::lock_guard<std::mutex> ack_lock(command_ack_mutex_);
      ++command_ack_sequences_[command_id];
    }
    command_ack_cv_.notify_all();
    RCLCPP_INFO(get_logger(), "IM900 acknowledged command 0x%s", command_hex(command_id).c_str());
  }

  void poll_serial()
  {
    std::array<uint8_t, 512> buffer{};
    try {
      ssize_t count = 0;
      {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        count = serial_->read_some(buffer.data(), buffer.size());
      }
      for (ssize_t i = 0; i < count; ++i) {
        const auto event = protocol_.input_byte(buffer[static_cast<size_t>(i)]);
        if (!event) {
          continue;
        }
        if (event->command_ack) {
          record_command_ack(*event->command_ack);
        }
        if (event->imu) {
          const auto stamp = now();
          const bool had_valid_orientation = has_valid_orientation_;
          publisher_->publish(
            to_msg(
              *event->imu,
              stamp,
              frame_id_,
              orientation_reference_,
              has_valid_orientation_));
          if (event->imu->has_magnetic_field) {
            magnetic_field_publisher_->publish(
              to_magnetic_field_msg(*event->imu, stamp, frame_id_));
            magnetic_magnitude_publisher_->publish(to_magnetic_magnitude_msg(*event->imu));
            last_magnetic_field_magnitude_ = event->imu->magnetic_field_magnitude;
            has_magnetic_field_magnitude_ = true;
          }
          if (has_valid_orientation_ && !had_valid_orientation) {
            RCLCPP_INFO(get_logger(), "received first valid hardware quaternion");
          }
          if (!has_valid_orientation_) {
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              5000,
              "IMU quaternion is invalid; publishing orientation as unavailable");
          }
        }
        if (event->battery) {
          battery_publisher_->publish(to_msg(*event->battery, now(), frame_id_));
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
    }
  }

  uint8_t target_address_{255};
  int baudrate_{460800};
  int report_hz_{200};
  int battery_query_period_ms_{5000};
  int command_ack_timeout_ms_{2000};
  bool log_magnetic_field_status_{true};
  bool restore_world_axes_{false};
  bool clear_world_axes_{false};
  bool clear_ins_position_{true};
  bool enable_compass_{true};
  bool use_reliable_qos_{true};
  bool force_positive_w_{true};
  bool use_quaternion_continuity_{true};
  bool has_valid_orientation_{false};
  bool has_magnetic_field_magnitude_{false};
  double last_magnetic_field_magnitude_{0.0};
  std::string port_;
  std::string frame_id_;
  imu_ros2::Im948Protocol protocol_;
  imu_ros2::OrientationReference orientation_reference_;
  std::mutex serial_mutex_;
  std::mutex command_mutex_;
  std::mutex command_ack_mutex_;
  std::condition_variable command_ack_cv_;
  std::array<uint64_t, 256> command_ack_sequences_{};
  std::unique_ptr<SerialPort> serial_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr magnetic_magnitude_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_z_axis_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_world_axes_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restore_world_axes_service_;
  rclcpp::CallbackGroup::SharedPtr io_callback_group_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr magnetic_log_timer_;
  rclcpp::TimerBase::SharedPtr battery_query_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ImuNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "imu_node failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
