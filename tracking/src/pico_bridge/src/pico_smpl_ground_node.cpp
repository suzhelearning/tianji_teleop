#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "pico_bridge/smpl_ground_alignment.hpp"

namespace {

pico_bridge::SmplPose from_message(const geometry_msgs::msg::Pose & message) {
  return {
    {message.position.x, message.position.y, message.position.z},
    {message.orientation.x, message.orientation.y,
     message.orientation.z, message.orientation.w}};
}

}  // namespace

class PicoSmplGroundNode : public rclcpp::Node {
 public:
  PicoSmplGroundNode() : Node("pico_smpl_ground") {
    raw_topic_ = declare_parameter<std::string>("raw_topic", "/pico/smpl_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/pico/smpl");
    world_reset_topic_ = declare_parameter<std::string>(
      "world_reset_topic", "/pico/world_reset");
    ground_ready_topic_ = declare_parameter<std::string>(
      "ground_ready_topic", "/pico/smpl_ground_ready");
    output_frame_ = declare_parameter<std::string>("output_frame", "pico_ground");
    const auto extents = declare_parameter<std::vector<double>>(
      "foot_half_extents", {0.115, 0.050, 0.022});
    const int stable_window = declare_parameter<int>("stable_window_frames", 30);
    const double tolerance = declare_parameter<double>(
      "stability_tolerance_m", 0.02);
    const bool require_reset = declare_parameter<bool>("require_world_reset", true);
    if (raw_topic_.empty() || output_topic_.empty() || world_reset_topic_.empty() ||
        ground_ready_topic_.empty() || output_frame_.empty()) {
      throw std::invalid_argument("SMPL ground topics and output frame must not be empty");
    }
    if (raw_topic_ == output_topic_) {
      throw std::invalid_argument("raw_topic and output_topic must differ");
    }
    if (extents.size() != 3 || stable_window <= 0) {
      throw std::invalid_argument(
        "foot_half_extents must contain 3 values and stable_window_frames must be positive");
    }
    pico_bridge::GroundAlignmentOptions options;
    options.foot_half_extents = {extents[0], extents[1], extents[2]};
    options.stable_window_frames = static_cast<std::size_t>(stable_window);
    options.stability_tolerance_m = tolerance;
    options.require_world_reset = require_reset;
    alignment_ = std::make_unique<pico_bridge::SmplGroundAlignment>(options);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, sensor_qos);
    auto ready_qos = rclcpp::QoS(1);
    ready_qos.reliable().transient_local();
    ground_ready_publisher_ =
      create_publisher<std_msgs::msg::Bool>(ground_ready_topic_, ready_qos);
    publish_ground_ready(false);
    raw_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      raw_topic_, sensor_qos,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
        on_raw_skeleton(*message);
      });
    world_reset_subscription_ = create_subscription<std_msgs::msg::Float32>(
      world_reset_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Float32::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        alignment_->reset();
        publish_ground_ready(false);
        RCLCPP_INFO(
          get_logger(),
          "PICO world reset received; canonical SMPL paused until stable feet lock the floor");
      });
    RCLCPP_INFO(
      get_logger(),
      "SMPL ground normalizer: %s -> %s, frame=%s, stable_frames=%d, require_A=%s",
      raw_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str(), stable_window,
      require_reset ? "true" : "false");
  }

 private:
  void on_raw_skeleton(const geometry_msgs::msg::PoseArray & message) {
    if (message.poses.size() != 24) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring raw SMPL frame with %zu poses; expected 24", message.poses.size());
      return;
    }
    std::vector<pico_bridge::SmplPose> poses;
    poses.reserve(message.poses.size());
    for (const auto & pose : message.poses) poses.push_back(from_message(pose));

    geometry_msgs::msg::PoseArray output;
    double locked_floor = 0.0;
    bool just_locked = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool was_locked = alignment_->locked();
      const bool accepted = alignment_->observe(poses);
      if (!accepted && alignment_->locked()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring invalid raw SMPL frame after floor lock");
        return;
      }
      if (!alignment_->locked()) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for PICO A and stable post-reset feet before publishing /pico/smpl");
        return;
      }
      just_locked = !was_locked;
      locked_floor = *alignment_->floor_height();
      const auto transformed = alignment_->transform(poses);
      output = message;
      output.header.frame_id = output_frame_;
      for (std::size_t index = 0; index < transformed.size(); ++index) {
        output.poses[index].position.z = transformed[index].position.z;
      }
    }
    if (just_locked) {
      publish_ground_ready(true);
      RCLCPP_INFO(
        get_logger(), "Canonical SMPL floor locked at raw Z=%.4f m; publishing frame %s",
        locked_floor, output_frame_.c_str());
    }
    publisher_->publish(output);
  }

  void publish_ground_ready(bool ready) {
    std_msgs::msg::Bool message;
    message.data = ready;
    ground_ready_publisher_->publish(message);
  }

  std::mutex mutex_;
  std::string raw_topic_, output_topic_, world_reset_topic_, ground_ready_topic_, output_frame_;
  std::unique_ptr<pico_bridge::SmplGroundAlignment> alignment_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ground_ready_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr raw_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr world_reset_subscription_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PicoSmplGroundNode>());
  rclcpp::shutdown();
  return 0;
}
