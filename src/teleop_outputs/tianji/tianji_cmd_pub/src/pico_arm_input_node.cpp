#include "pico_bridge/tianji_teleop_protocol.hpp"

#include <time.h>

#include <fstream>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <utility>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tianji_interfaces/msg/pico_arm_input.hpp>

namespace pico_bridge {
namespace {

std::int64_t monotonicNowNs()
{
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    throw std::runtime_error("clock_gettime failed");
  }
  return static_cast<std::int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
}

std::string readUuid(const char * path)
{
  std::ifstream stream(path);
  std::string value;
  if (!(stream >> value) || value.size() != 36) {
    throw std::runtime_error(std::string("Cannot read UUID: ") + path);
  }
  return value;
}

template<typename VectorMessage>
void copyVector(VectorMessage & output, const Eigen::Vector3d & vector)
{
  output.x = vector.x();
  output.y = vector.y();
  output.z = vector.z();
}

void copyQuaternion(
  geometry_msgs::msg::Quaternion & output, const Eigen::Quaterniond & quaternion)
{
  const auto normalized = quaternion.normalized();
  output.x = normalized.x();
  output.y = normalized.y();
  output.z = normalized.z();
  output.w = normalized.w();
}

void copyPose(geometry_msgs::msg::Pose & output, const Eigen::Isometry3d & pose)
{
  copyVector(output.position, pose.translation());
  copyQuaternion(output.orientation, Eigen::Quaterniond(pose.linear()));
}

bool waitingOutcome(const std::string & reason)
{
  return reason == "awaiting_status" || reason == "awaiting_skeleton";
}

struct SkeletonConversion
{
  bool valid{false};
  std::string rejection_reason;
  std::int64_t source_stamp_ns{0};
  PicoSkeletonFrame frame;
};

SkeletonConversion convertPoseArray(
  const geometry_msgs::msg::PoseArray & message)
{
  SkeletonConversion result;
  if (message.header.frame_id != "pico") {
    result.rejection_reason = "source_frame_id_invalid";
    return result;
  }
  if (message.poses.size() != kPicoSmplJointCount) {
    result.rejection_reason = "skeleton_pose_count_invalid";
    return result;
  }
  result.source_stamp_ns =
    static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(message.header.stamp.nanosec);
  if (result.source_stamp_ns <= 0) {
    result.rejection_reason = "source_stamp_invalid";
    return result;
  }

  for (std::size_t index = 0; index < message.poses.size(); ++index) {
    const auto & source = message.poses[index];
    PicoSkeletonPose & destination = result.frame[index];
    destination.position = {
      source.position.x, source.position.y, source.position.z};
    destination.orientation = Eigen::Quaterniond(
      source.orientation.w,
      source.orientation.x,
      source.orientation.y,
      source.orientation.z);
    if (!destination.position.allFinite() ||
      !destination.orientation.coeffs().allFinite())
    {
      result.rejection_reason = "skeleton_pose_non_finite";
      return result;
    }
    const double norm = destination.orientation.norm();
    if (!std::isfinite(norm) || norm < 1.0e-9) {
      result.rejection_reason = "skeleton_quaternion_invalid";
      return result;
    }
    destination.orientation.normalize();
  }
  result.valid = true;
  return result;
}

}  // namespace

class PicoArmInputNode : public rclcpp::Node
{
public:
  PicoArmInputNode()
  : Node("pico_arm_input"),
    skeleton_topic_(declare_parameter<std::string>(
      "skeleton_topic", "/pico/smpl_palm_corrected_ik")),
    status_topic_(declare_parameter<std::string>(
      "status_topic", "/pico/smpl_palm_corrected/status")),
    record_flag_topic_(declare_parameter<std::string>(
      "record_flag_topic", "/pico/record_flag")),
    diagnostics_topic_(declare_parameter<std::string>(
      "diagnostics_topic", "/pico/arm_input/status")),
    output_topic_(declare_parameter<std::string>("output_topic", "/pico/arm_input")),
    cache_capacity_(declare_parameter<int>("cache_capacity", 8)),
    position_retargeting_mode_(declare_parameter<std::string>(
      "position_retargeting_mode", "robot_arm_segments")),
    robot_arm_reach_scale_(declare_parameter<double>(
      "robot_arm_reach_scale", kDefaultRobotArmReachScale)),
    pico_world_x_offset_m_(declare_parameter<double>(
      "pico_world_x_offset_m", kDefaultPicoWorldXOffsetM)),
    core_(
      validatedCacheCapacity(cache_capacity_),
      validatedPositionRetargetingConfig(
        position_retargeting_mode_, robot_arm_reach_scale_,
        pico_world_x_offset_m_))
  {
    input_publisher_ = create_publisher<tianji_interfaces::msg::PicoArmInput>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    const auto skeleton_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    skeleton_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      skeleton_topic_, skeleton_qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        onSkeleton(*message);
      });
    status_subscription_ = create_subscription<std_msgs::msg::String>(
      status_topic_, status_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {
        onStatus(*message);
      });
    // A 键状态：driver 在收到 PICO 0x09 帧时发布 /pico/record_flag。
    // 桥侧保持消息携带的翻转状态，viewer 再检测该位的任意变化，
    // 切换联合遥操的暂停与继续接管状态。
    record_flag_subscription_ = create_subscription<std_msgs::msg::Bool>(
      record_flag_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        record_button_state_ = message->data;
        RCLCPP_INFO(get_logger(), "A button -> user_button_pressed=%d",
          record_button_state_ ? 1 : 0);
      });
    diagnostics_publisher_ = create_publisher<std_msgs::msg::String>(
      diagnostics_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publishDiagnostics();});
    start_monotonic_ns_ = monotonicNowNs();
  }

private:
  static std::size_t validatedCacheCapacity(int capacity)
  {
    if (capacity <= 0) {
      throw std::invalid_argument("cache_capacity must be positive");
    }
    return static_cast<std::size_t>(capacity);
  }

  static PicoPositionRetargetingConfig validatedPositionRetargetingConfig(
    const std::string & mode, double robot_arm_reach_scale,
    double pico_world_x_offset_m)
  {
    PicoPositionRetargetingConfig config;
    if (!std::isfinite(robot_arm_reach_scale) ||
      robot_arm_reach_scale <= 0.0 || robot_arm_reach_scale > 1.0)
    {
      throw std::invalid_argument(
              "robot_arm_reach_scale must be finite and in (0,1]");
    }
    if (!std::isfinite(pico_world_x_offset_m)) {
      throw std::invalid_argument("pico_world_x_offset_m must be finite");
    }
    if (mode == "robot_arm_segments") {
      config.mode = PicoPositionRetargetingMode::kRobotArmSegments;
    } else if (mode == "pico_palm") {
      config.mode = PicoPositionRetargetingMode::kPicoPalm;
    } else {
      throw std::invalid_argument(
              "position_retargeting_mode must be robot_arm_segments or pico_palm");
    }
    config.robot_arm_reach_scale = robot_arm_reach_scale;
    config.pico_world_x_offset_m = pico_world_x_offset_m;
    return config;
  }

  void onSkeleton(const geometry_msgs::msg::PoseArray & message)
  {
    ++skeleton_messages_;
    const SkeletonConversion converted = convertPoseArray(message);
    if (!converted.valid) {
      recordRejection(converted.rejection_reason);
      revokeInput();
      return;
    }
    handleOutcome(core_.ingest_skeleton(
      converted.source_stamp_ns, converted.frame));
  }

  void onStatus(const std_msgs::msg::String & message)
  {
    ++status_messages_;
    const auto status = parse_corrected_ik_status(message.data);
    if (!status.valid) {
      recordRejection(status.rejection_reason);
      revokeInput();
      return;
    }
    handleOutcome(core_.ingest_status(status));
  }

  void handleOutcome(const TianjiTeleopBridgeOutcome & outcome)
  {
    if (!outcome.frame.has_value()) {
      if (!waitingOutcome(outcome.rejection_reason)) {
        recordRejection(outcome.rejection_reason);
        if (outcome.rejection_reason != "duplicate_source_stamp" &&
          outcome.rejection_reason != "status_cache_miss" &&
          outcome.rejection_reason != "skeleton_cache_miss")
        {
          revokeInput();
        }
      }
      return;
    }

    const auto & frame = *outcome.frame;
    auto message = newMessage();
    message.valid = true;
    message.tracking_epoch = frame.tracking_epoch;
    message.source_timestamp_ns = frame.source_timestamp_ns;
    copyPose(message.left_target, frame.left_target);
    copyPose(message.right_target, frame.right_target);
    copyVector(message.left_arm_direction, frame.left_arm_direction.direction);
    copyVector(message.right_arm_direction, frame.right_arm_direction.direction);
    message.left_arm_direction_valid = frame.left_arm_direction.valid;
    message.right_arm_direction_valid = frame.right_arm_direction.valid;
    message.upper_limb_valid = frame.upper_limb_skeleton.valid;
    message.upper_limb_rotations_valid = frame.upper_limb_skeleton.rotations_valid;
    for (std::size_t index = 0; index < message.upper_limb_points.size(); ++index) {
      copyVector(message.upper_limb_points[index], frame.upper_limb_skeleton.points[index]);
      copyQuaternion(
        message.upper_limb_rotations[index], frame.upper_limb_skeleton.rotations[index]);
    }
    input_publisher_->publish(message);
    stream_valid_ = true;
    ++frames_published_;
    latest_tracking_epoch_ = frame.tracking_epoch;
    latest_source_stamp_ns_ = frame.source_timestamp_ns;
  }

  tianji_interfaces::msg::PicoArmInput newMessage()
  {
    tianji_interfaces::msg::PicoArmInput message;
    message.boot_id = boot_id_;
    message.session_id = session_id_;
    message.revocation_generation = revocation_generation_;
    message.sequence = ++latest_sequence_;
    message.published_monotonic_ns = monotonicNowNs();
    message.user_button_pressed = record_button_state_;
    return message;
  }

  void revokeInput()
  {
    core_.reset();
    if (stream_valid_) {
      // Carry a discontinuity into subsequent valid samples as well: latest-only
      // delivery may coalesce the invalid sample before the consumer observes it.
      ++revocation_generation_;
    }
    stream_valid_ = false;
    auto message = newMessage();
    message.valid = false;
    message.tracking_epoch = latest_tracking_epoch_;
    message.source_timestamp_ns = latest_source_stamp_ns_;
    input_publisher_->publish(message);
    ++invalid_frames_published_;
  }

  void recordRejection(const std::string & reason)
  {
    ++rejections_;
    last_rejection_reason_ = reason.empty() ? "unknown" : reason;
  }

  void publishDiagnostics()
  {
    const std::int64_t now_ns = monotonicNowNs();
    const double elapsed_seconds = static_cast<double>(
      now_ns - start_monotonic_ns_) * 1.0e-9;
    const double output_frequency_hz = elapsed_seconds > 0.0 ?
      static_cast<double>(frames_published_) / elapsed_seconds : 0.0;
    std::ostringstream json;
    json << '{'
         << "\"skeleton_messages\":" << skeleton_messages_ << ','
         << "\"status_messages\":" << status_messages_ << ','
         << "\"frames_published\":" << frames_published_ << ','
         << "\"invalid_frames_published\":" << invalid_frames_published_ << ','
         << "\"valid\":" << (stream_valid_ ? "true" : "false") << ','
         << "\"rejections\":" << rejections_ << ','
         << "\"output_frequency_hz\":" << output_frequency_hz << ','
         << "\"position_retargeting_mode\":\""
         << position_retargeting_mode_ << "\","
         << "\"robot_arm_reach_scale\":" << robot_arm_reach_scale_ << ','
         << "\"pico_world_x_offset_m\":" << pico_world_x_offset_m_ << ','
         << "\"sequence\":" << latest_sequence_ << ','
         << "\"tracking_epoch\":" << latest_tracking_epoch_ << ','
         << "\"source_stamp_ns\":" << latest_source_stamp_ns_ << ','
         << "\"last_rejection_reason\":\"" << last_rejection_reason_ << "\""
         << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    diagnostics_publisher_->publish(message);
  }

  std::string skeleton_topic_;
  std::string status_topic_;
  std::string record_flag_topic_;
  std::string diagnostics_topic_;
  std::string output_topic_;
  int cache_capacity_{8};
  std::string position_retargeting_mode_{"robot_arm_segments"};
  double robot_arm_reach_scale_{kDefaultRobotArmReachScale};
  double pico_world_x_offset_m_{kDefaultPicoWorldXOffsetM};
  TianjiTeleopBridgeCore core_;
  const std::string boot_id_{readUuid("/proc/sys/kernel/random/boot_id")};
  const std::string session_id_{readUuid("/proc/sys/kernel/random/uuid")};
  std::uint64_t revocation_generation_{0};
  bool stream_valid_{false};
  rclcpp::Publisher<tianji_interfaces::msg::PicoArmInput>::SharedPtr input_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
    skeleton_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    record_flag_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher_;
  bool record_button_state_{false};
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::uint64_t skeleton_messages_{0};
  std::uint64_t status_messages_{0};
  std::uint64_t frames_published_{0};
  std::uint64_t rejections_{0};
  std::uint64_t invalid_frames_published_{0};
  std::uint64_t latest_sequence_{0};
  std::uint64_t latest_tracking_epoch_{0};
  std::int64_t latest_source_stamp_ns_{0};
  std::int64_t start_monotonic_ns_{0};
  std::string last_rejection_reason_;
};

}  // namespace pico_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<pico_bridge::PicoArmInputNode>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "pico_arm_input: %s\n", error.what());
    rclcpp::shutdown();
    return 2;
  }
}
