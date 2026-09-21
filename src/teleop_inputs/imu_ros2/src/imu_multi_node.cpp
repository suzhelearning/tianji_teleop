#include <array>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "imu_ros2/device_time_mapper.hpp"
#include "imu_ros2/im948_protocol.hpp"
#include "imu_ros2/orientation_reference.hpp"

namespace
{

constexpr double kMicroteslaToTesla = 1e-6;

std::string sanitize_logger_suffix(std::string value)
{
  for (char & ch : value) {
    if (ch == '/') {
      ch = '_';
    }
  }
  return value;
}

class SerialPort
{
public:
  SerialPort(const std::string & port, int baudrate)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("open failed: " + std::string(std::strerror(errno)));
    }

    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      close();
      throw std::runtime_error(
        "serial port already in use (another imu_node/imu_multi_node?): " + port);
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

    serial_struct serinfo{};
    if (::ioctl(fd_, TIOCGSERIAL, &serinfo) == 0) {
      serinfo.flags |= ASYNC_LOW_LATENCY;
      ::ioctl(fd_, TIOCSSERIAL, &serinfo);
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
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
      fd_ = -1;
    }
  }

  int fd_{-1};
};

struct SharedConfig
{
  int baudrate{115200};
  int report_hz{110};
  int report_tag{imu_ros2::kDefaultReportTag};
  int battery_query_period_ms{5000};
  bool restore_world_axes{false};
  bool clear_world_axes{false};
  bool clear_ins_position{true};
  bool enable_compass{true};
  bool log_magnetic_field_status{true};
  bool use_reliable_qos{false};
  bool use_device_timestamp{true};
  bool coalesce_frames_per_poll{true};
  bool align_quaternion_hemisphere{true};
  bool force_positive_w{true};
  bool use_quaternion_continuity{true};
  int command_ack_timeout_ms{2000};
  uint8_t target_address{255};
  std::string imu_topic{"imuData_raw"};
  std::string battery_topic{"imuBattery"};
  std::string magnetic_topic{"imuMagneticField"};
  std::string magnetic_magnitude_topic{"magnetic_field_magnitude"};
  std::string zero_z_axis_service{"zero_z_axis"};
  std::string clear_world_axes_service{"clear_world_axes"};
  std::string restore_world_axes_service{"restore_world_axes"};
};

sensor_msgs::msg::Imu to_imu_msg(
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
    has_valid_orientation = false;
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

sensor_msgs::msg::BatteryState to_battery_msg(
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

class ImuChannel
{
public:
  ImuChannel(
    rclcpp::Node * node,
    std::string id,
    std::string frame_id,
    std::string port,
    bool append_blank_line_after_log,
    const SharedConfig & config,
    rclcpp::CallbackGroup::SharedPtr command_callback_group)
  : node_(node),
    logger_(node->get_logger().get_child(sanitize_logger_suffix(id))),
    id_(std::move(id)),
    port_(std::move(port)),
    frame_id_(std::move(frame_id)),
    append_blank_line_after_log_(append_blank_line_after_log),
    config_(config),
    protocol_(config.target_address)
  {
    orientation_reference_.configure(
      config_.force_positive_w,
      config_.use_quaternion_continuity);

    const auto imu_qos = config.use_reliable_qos ? rclcpp::QoS(10) : rclcpp::SensorDataQoS();
    const std::string imu_topic = id_ + "/" + config.imu_topic;
    const std::string battery_topic = id_ + "/" + config.battery_topic;
    const std::string magnetic_topic = id_ + "/" + config.magnetic_topic;
    const std::string magnetic_magnitude_topic = id_ + "/" + config.magnetic_magnitude_topic;
    const std::string ready_topic = id_ + "/ready";

    publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(imu_topic, imu_qos);
    battery_publisher_ =
      node_->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic, rclcpp::QoS(10));
    magnetic_field_publisher_ =
      node_->create_publisher<sensor_msgs::msg::MagneticField>(magnetic_topic, imu_qos);
    magnetic_magnitude_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64>(magnetic_magnitude_topic, imu_qos);
    ready_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(ready_topic, rclcpp::QoS(10));
    publish_ready(false);

    zero_z_axis_service_ = node_->create_service<std_srvs::srv::Trigger>(
      id_ + "/" + config.zero_z_axis_service,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        send_device_command(
          imu_ros2::Im948Protocol::make_zero_z_axis_command(config_.target_address),
          0x05,
          "z axis zero command sent",
          response);
      }, rmw_qos_profile_services_default, command_callback_group);
    clear_world_axes_service_ = node_->create_service<std_srvs::srv::Trigger>(
      id_ + "/" + config.clear_world_axes_service,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        send_device_command(
          imu_ros2::Im948Protocol::make_clear_world_axes_command(config_.target_address),
          0x06,
          "world xyz axes clear command sent",
          response);
      }, rmw_qos_profile_services_default, command_callback_group);
    restore_world_axes_service_ = node_->create_service<std_srvs::srv::Trigger>(
      id_ + "/" + config.restore_world_axes_service,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        send_device_command(
          imu_ros2::Im948Protocol::make_restore_world_axes_command(config_.target_address),
          0x08,
          "world axes restore command sent",
          response);
      }, rmw_qos_profile_services_default, command_callback_group);

    try_reconnect();
    if (config_.log_magnetic_field_status) {
      magnetic_log_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          log_magnetic_field_status();
        });
    }

    RCLCPP_INFO(
      logger_,
      "channel ready on %s at %d baud, topics %s, %s, %s and %s, report_hz=%d, "
      "report_tag=0x%04x, enable_compass=%s, imu_qos=%s, device_timestamp=%s, coalesce=%s",
      port_.c_str(),
      config_.baudrate,
      imu_topic.c_str(),
      battery_topic.c_str(),
      magnetic_topic.c_str(),
      magnetic_magnitude_topic.c_str(),
      config_.report_hz,
      config_.report_tag,
      config_.enable_compass ? "true" : "false",
      config_.use_reliable_qos ? "reliable" : "best_effort",
      config_.use_device_timestamp ? "true" : "false",
      config_.coalesce_frames_per_poll ? "true" : "false");
  }

  void poll_serial()
  {
    std::array<uint8_t, 2048> buffer{};
    std::optional<imu_ros2::ImuSample> latest_imu;
    std::optional<imu_ros2::BatteryStatus> latest_battery;
    try_reconnect();
    {
      std::lock_guard<std::mutex> serial_lock(serial_mutex_);
      if (!serial_) {
        return;
      }
    }
    try {
      while (true) {
        ssize_t count = 0;
        {
          std::lock_guard<std::mutex> serial_lock(serial_mutex_);
          if (!serial_) {
            return;
          }
          count = serial_->read_some(buffer.data(), buffer.size());
        }
        if (count <= 0) {
          break;
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
            if (config_.coalesce_frames_per_poll) {
              latest_imu = *event->imu;
            } else {
              publish_imu_sample(*event->imu);
            }
          }
          if (event->battery) {
            latest_battery = *event->battery;
          }
        }
      }
      if (latest_imu) {
        publish_imu_sample(*latest_imu);
      }
      if (latest_battery) {
        const rclcpp::Time stamp = node_->now();
        battery_publisher_->publish(to_battery_msg(*latest_battery, stamp, frame_id_));
      }
    } catch (const std::exception & error) {
      mark_disconnected(error.what());
      RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 2000, "%s", error.what());
    }
  }

  void query_battery_status()
  {
    try {
      try_reconnect();
      std::lock_guard<std::mutex> serial_lock(serial_mutex_);
      if (!serial_) {
        return;
      }
      serial_->write_all(
        imu_ros2::Im948Protocol::make_get_device_status_command(config_.target_address));
    } catch (const std::exception & error) {
      mark_disconnected(error.what());
      RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 2000, "%s", error.what());
    }
  }

  const std::string & id() const
  {
    return id_;
  }

  const std::string & port() const
  {
    return port_;
  }

private:
  void publish_ready(bool ready)
  {
    std_msgs::msg::Bool msg;
    msg.data = ready;
    ready_publisher_->publish(msg);
  }

  void try_reconnect()
  {
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    if (serial_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (last_reconnect_attempt_.time_since_epoch().count() != 0 &&
        now - last_reconnect_attempt_ < std::chrono::seconds(1)) {
      return;
    }
    last_reconnect_attempt_ = now;
    try {
      serial_ = std::make_unique<SerialPort>(port_, config_.baudrate);
      protocol_ = imu_ros2::Im948Protocol(config_.target_address);
      orientation_reference_.configure(config_.force_positive_w, config_.use_quaternion_continuity);
      has_valid_orientation_ = false;
      publish_ready(false);
      send_startup_commands();
      RCLCPP_INFO(logger_, "connected to %s; waiting for valid hardware quaternion", port_.c_str());
    } catch (const std::exception & error) {
      serial_.reset();
      has_valid_orientation_ = false;
      publish_ready(false);
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
        "IMU channel %s reconnect failed: %s", port_.c_str(), error.what());
    }
  }

  void mark_disconnected(const std::string & reason)
  {
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    serial_.reset();
    protocol_ = imu_ros2::Im948Protocol(config_.target_address);
    orientation_reference_.configure(config_.force_positive_w, config_.use_quaternion_continuity);
    has_valid_orientation_ = false;
    publish_ready(false);
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
      "IMU channel %s disconnected: %s; reconnecting", port_.c_str(), reason.c_str());
  }

  void log_magnetic_field_status()
  {
    if (!has_magnetic_field_magnitude_) {
      return;
    }
    if (append_blank_line_after_log_) {
      RCLCPP_INFO(
        logger_,
        "[%s] magnetic field |H|=%.2f uT\n",
        id_.c_str(),
        last_magnetic_field_magnitude_);
      return;
    }
    RCLCPP_INFO(
      logger_,
      "[%s] magnetic field |H|=%.2f uT",
      id_.c_str(),
      last_magnetic_field_magnitude_);
  }

  void publish_imu_sample(const imu_ros2::ImuSample & sample)
  {
    const rclcpp::Time stamp = config_.use_device_timestamp ?
      device_time_mapper_.to_stamp(sample.device_time_ms, node_->now()) :
      node_->now();
    const bool had_valid_orientation = has_valid_orientation_;
    auto msg = to_imu_msg(
      sample,
      stamp,
      frame_id_,
      orientation_reference_,
      has_valid_orientation_);
    publisher_->publish(msg);
    publish_ready(has_valid_orientation_);
    if (sample.has_magnetic_field) {
      magnetic_field_publisher_->publish(to_magnetic_field_msg(sample, stamp, frame_id_));
      magnetic_magnitude_publisher_->publish(to_magnetic_magnitude_msg(sample));
      last_magnetic_field_magnitude_ = sample.magnetic_field_magnitude;
      has_magnetic_field_magnitude_ = true;
    }
    if (has_valid_orientation_ && !had_valid_orientation) {
      RCLCPP_INFO(logger_, "received first valid hardware quaternion");
    }
    if (!has_valid_orientation_) {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *node_->get_clock(),
        5000,
        "IMU quaternion is invalid; publishing orientation as unavailable");
    }
  }

  void send_startup_commands()
  {
    serial_->write_all(imu_ros2::Im948Protocol::make_set_parameters_command(
      static_cast<uint8_t>(config_.report_hz),
      static_cast<uint16_t>(config_.report_tag),
      config_.target_address,
      config_.enable_compass));
    serial_->write_all(imu_ros2::Im948Protocol::make_wake_command(config_.target_address));
    if (config_.restore_world_axes) {
      serial_->write_all(
        imu_ros2::Im948Protocol::make_restore_world_axes_command(config_.target_address));
    }
    if (config_.clear_world_axes) {
      serial_->write_all(
        imu_ros2::Im948Protocol::make_clear_world_axes_command(config_.target_address));
    }
    if (config_.clear_ins_position) {
      serial_->write_all(
        imu_ros2::Im948Protocol::make_clear_ins_position_command(config_.target_address));
    }
    serial_->write_all(
      imu_ros2::Im948Protocol::make_enable_auto_report_command(config_.target_address));
    serial_->write_all(imu_ros2::Im948Protocol::make_get_device_status_command(config_.target_address));
  }

  void send_device_command(
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
        if (!serial_) {
          throw std::runtime_error("serial channel is disconnected");
        }
        serial_->write_all(command);
      }

      std::unique_lock<std::mutex> ack_lock(command_ack_mutex_);
      const bool acknowledged = command_ack_cv_.wait_for(
        ack_lock,
        std::chrono::milliseconds(config_.command_ack_timeout_ms),
        [this, command_id, ack_sequence]() {
          return command_ack_sequences_[command_id] > ack_sequence;
        });
      if (!acknowledged) {
        response->success = false;
        response->message =
          "timeout waiting for IM900 command ack 0x" + command_hex(command_id) +
          " on " + port_;
        RCLCPP_ERROR(logger_, "%s", response->message.c_str());
        return;
      }

      response->success = true;
      response->message = success_message + " confirmed (0x" + command_hex(command_id) + ")";
      RCLCPP_INFO(logger_, "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(logger_, "%s", error.what());
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
    RCLCPP_INFO(logger_, "IM900 acknowledged command 0x%s", command_hex(command_id).c_str());
  }

  rclcpp::Node * node_{nullptr};
  rclcpp::Logger logger_;
  std::string id_;
  std::string port_;
  std::string frame_id_;
  bool append_blank_line_after_log_{false};
  SharedConfig config_;
  imu_ros2::Im948Protocol protocol_;
  imu_ros2::OrientationReference orientation_reference_;
  imu_ros2::DeviceTimeMapper device_time_mapper_;
  bool has_valid_orientation_{false};
  std::chrono::steady_clock::time_point last_reconnect_attempt_;
  bool has_magnetic_field_magnitude_{false};
  double last_magnetic_field_magnitude_{0.0};
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_z_axis_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_world_axes_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restore_world_axes_service_;
  rclcpp::TimerBase::SharedPtr magnetic_log_timer_;
};

class ImuMultiNode : public rclcpp::Node
{
public:
  ImuMultiNode()
  : Node("imu_multi_node")
  {
    const auto ports = declare_parameter<std::vector<std::string>>(
      "ports",
      {"/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"});

    SharedConfig config;
    config.baudrate = declare_parameter<int>("baudrate", 115200);
    config.report_hz = declare_parameter<int>("report_hz", 110);
    config.report_tag = declare_parameter<int>("report_tag", imu_ros2::kDefaultReportTag);
    config.battery_query_period_ms = declare_parameter<int>("battery_query_period_ms", 5000);
    config.restore_world_axes = declare_parameter<bool>("restore_world_axes", false);
    config.clear_world_axes = declare_parameter<bool>("clear_world_axes", false);
    config.clear_ins_position = declare_parameter<bool>("clear_ins_position", true);
    config.enable_compass = declare_parameter<bool>("enable_compass", true);
    config.log_magnetic_field_status =
      declare_parameter<bool>("log_magnetic_field_status", true);
    config.use_reliable_qos = declare_parameter<bool>("use_reliable_qos", false);
    config.use_device_timestamp = declare_parameter<bool>("use_device_timestamp", true);
    config.coalesce_frames_per_poll = declare_parameter<bool>("coalesce_frames_per_poll", true);
    config.align_quaternion_hemisphere =
      declare_parameter<bool>("align_quaternion_hemisphere", true);
    // Keep the legacy tuning knobs accepted in this workspace even though the
    // upstream multi-node currently only exposes align_quaternion_hemisphere.
    config.force_positive_w = declare_parameter<bool>("force_positive_w", true);
    config.use_quaternion_continuity =
      declare_parameter<bool>("use_quaternion_continuity", config.align_quaternion_hemisphere);
    config.command_ack_timeout_ms = declare_parameter<int>("command_ack_timeout_ms", 2000);
    config.target_address = static_cast<uint8_t>(declare_parameter<int>("target_address", 255));
    config.imu_topic = declare_parameter<std::string>("imu_topic", "imuData_raw");
    config.battery_topic = declare_parameter<std::string>("battery_topic", "imuBattery");
    config.magnetic_topic = declare_parameter<std::string>("magnetic_topic", "imuMagneticField");
    config.magnetic_magnitude_topic =
      declare_parameter<std::string>("magnetic_magnitude_topic", "magnetic_field_magnitude");
    config.zero_z_axis_service =
      declare_parameter<std::string>("zero_z_axis_service", "zero_z_axis");
    config.clear_world_axes_service =
      declare_parameter<std::string>("clear_world_axes_service", "clear_world_axes");
    config.restore_world_axes_service =
      declare_parameter<std::string>("restore_world_axes_service", "restore_world_axes");

    const auto name_prefix = declare_parameter<std::string>("name_prefix", "imu");
    const auto channel_names =
      declare_parameter<std::vector<std::string>>("channel_names", std::vector<std::string>{});
    const auto frame_ids =
      declare_parameter<std::vector<std::string>>("frame_ids", std::vector<std::string>{});

    if (!channel_names.empty() && channel_names.size() != ports.size()) {
      throw std::runtime_error("channel_names size must match ports size");
    }
    if (!frame_ids.empty() && frame_ids.size() != ports.size()) {
      throw std::runtime_error("frame_ids size must match ports size");
    }
    if (config.command_ack_timeout_ms <= 0) {
      throw std::runtime_error("command_ack_timeout_ms must be positive");
    }
    if (config.report_tag < 0 || config.report_tag > 0xffff) {
      throw std::runtime_error("report_tag must be between 0 and 65535");
    }

    poll_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    for (size_t index = 0; index < ports.size(); ++index) {
      const std::string id = channel_names.empty()
        ? name_prefix + std::to_string(index)
        : channel_names[index];
      const std::string frame_id = frame_ids.empty() ? id : frame_ids[index];
      const bool append_blank_line_after_log = (index + 1 == ports.size());
      try {
        channels_.push_back(
          std::make_unique<ImuChannel>(
            this,
            id,
            frame_id,
            ports[index],
            append_blank_line_after_log,
            config,
            command_callback_group_));
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(),
          "failed to open channel %s on %s: %s",
          id.c_str(),
          ports[index].c_str(),
          error.what());
      }
    }

    if (channels_.empty()) {
      throw std::runtime_error("no IMU channels available");
    }

    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      [this]() {
        for (const auto & channel : channels_) {
          channel->poll_serial();
        }
      }, poll_callback_group_);
    battery_query_timer_ = create_wall_timer(
      std::chrono::milliseconds(config.battery_query_period_ms),
      [this]() {
        for (const auto & channel : channels_) {
          channel->query_battery_status();
        }
      }, poll_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "started %zu/%zu IMU channel(s), baudrate=%d report_hz=%d report_tag=0x%04x enable_compass=%s",
      channels_.size(),
      ports.size(),
      config.baudrate,
      config.report_hz,
      config.report_tag,
      config.enable_compass ? "true" : "false");
    for (const auto & channel : channels_) {
      RCLCPP_INFO(
        get_logger(),
        "  active: %s -> %s",
        channel->id().c_str(),
        channel->port().c_str());
    }
    if (channels_.size() < ports.size()) {
      RCLCPP_WARN(
        get_logger(),
        "some channels failed to open; run `ls -l /dev/ttyACM* /dev/ttyUSB*` and set the `ports` parameter");
    }
  }

private:
  std::vector<std::unique_ptr<ImuChannel>> channels_;
  rclcpp::CallbackGroup::SharedPtr poll_callback_group_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr battery_query_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ImuMultiNode>();
    // Four wrist reset services may wait for ACKs concurrently; reserve worker
    // capacity for the serial poll timer that receives those ACKs.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "imu_multi_node failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
