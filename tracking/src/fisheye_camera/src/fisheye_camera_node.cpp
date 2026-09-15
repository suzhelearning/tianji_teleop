#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "fisheye_camera/v4l2_camera.hpp"

namespace fisheye_camera {

struct CameraConfig {
  std::string device_path;
  std::string cam_name;
  std::unordered_map<std::string, int> settings;
};

// Parse "device:name[:key=val...]"
static CameraConfig parse_camera_string(const std::string& cam_str) {
  CameraConfig cfg;
  std::vector<std::string> parts;
  size_t start = 0;
  while (start < cam_str.size()) {
    auto pos = cam_str.find(':', start);
    if (pos == std::string::npos) {
      parts.push_back(cam_str.substr(start));
      break;
    }
    parts.push_back(cam_str.substr(start, pos - start));
    start = pos + 1;
  }

  if (parts.size() < 2) {
    throw std::runtime_error("Camera string must be 'device:name[:key=val...]', got: " + cam_str);
  }

  cfg.device_path = parts[0];
  cfg.cam_name = parts[1];

  for (size_t i = 2; i < parts.size(); ++i) {
    auto eq = parts[i].find('=');
    if (eq == std::string::npos) continue;
    std::string key = parts[i].substr(0, eq);
    std::string val = parts[i].substr(eq + 1);

    if (key == "auto_exposure" || key == "auto_wb") {
      cfg.settings[key] = (val == "1" || val == "true" || val == "on") ? 1 : 0;
    } else if (key == "exposure" || key == "gain" || key == "wb_temperature") {
      cfg.settings[key] = std::stoi(val);
    } else if (key == "rotate") {
      // Rotation not supported in zero-copy mode, log warning later
      cfg.settings[key] = std::stoi(val);
    }
  }

  return cfg;
}

class FisheyeCameraNode : public rclcpp::Node {
public:
  FisheyeCameraNode() : Node("fisheye_camera_node"), running_(true) {
    declare_parameter("usb_cameras", std::vector<std::string>{});
    declare_parameter("width", 640);
    declare_parameter("height", 480);
    declare_parameter("fps", 60);
    declare_parameter("jpeg_quality", 80);
    declare_parameter("record", false);
    declare_parameter("record_dir", std::string("./recordings"));

    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    fps_ = get_parameter("fps").as_int();

    auto cam_strings = get_parameter("usb_cameras").as_string_array();
    if (cam_strings.empty()) {
      RCLCPP_ERROR(get_logger(), "No cameras specified. Set usb_cameras parameter.");
      return;
    }

    // Recording: warn that it's not implemented
    if (get_parameter("record").as_bool()) {
      RCLCPP_WARN(get_logger(), "MP4 recording is not yet supported in C++ node, ignoring record=true");
    }

    // QoS: BEST_EFFORT, VOLATILE, depth=1
    auto qos = rclcpp::QoS(1)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile);

    for (const auto& cam_str : cam_strings) {
      auto cfg = parse_camera_string(cam_str);

      if (cfg.settings.count("rotate") && cfg.settings["rotate"] != 0) {
        RCLCPP_WARN(get_logger(),
          "[%s] Rotation not supported in zero-copy V4L2 mode, ignoring rotate=%d",
          cfg.cam_name.c_str(), cfg.settings["rotate"]);
      }

      auto pub = create_publisher<sensor_msgs::msg::CompressedImage>(
        "/" + cfg.cam_name + "/image/compressed", qos);

      RCLCPP_INFO(get_logger(), "Camera configured: %s @ %s",
        cfg.cam_name.c_str(), cfg.device_path.c_str());

      threads_.emplace_back(&FisheyeCameraNode::camera_thread, this, cfg, pub);
    }
  }

  ~FisheyeCameraNode() override {
    running_ = false;
    for (auto& t : threads_) {
      if (t.joinable()) t.join();
    }
  }

private:
  void camera_thread(
      CameraConfig cfg,
      rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub) {
    const std::string tag = "[" + cfg.cam_name + "]";
    constexpr int kMaxReconnect = 5;
    constexpr int kMaxFailures = 30;

    V4L2Camera cam;
    if (!open_camera(cam, cfg, tag)) {
      return;
    }

    // Pre-allocate message
    auto msg = sensor_msgs::msg::CompressedImage();
    msg.header.frame_id = cfg.cam_name;
    msg.format = "jpeg";

    int frame_count = 0;
    auto fps_timer = std::chrono::steady_clock::now();
    int fail_count = 0;

    while (running_.load(std::memory_order_relaxed)) {
      FrameData frame;
      if (!cam.grab_frame(frame)) {
        ++fail_count;
        if (fail_count >= kMaxFailures) {
          RCLCPP_WARN(get_logger(), "%s %d consecutive grab failures, reconnecting...",
            tag.c_str(), kMaxFailures);
          cam.close();
          if (!reconnect(cam, cfg, tag, kMaxReconnect)) {
            RCLCPP_ERROR(get_logger(), "%s Reconnect failed, stopping.", tag.c_str());
            return;
          }
          fail_count = 0;
        }
        continue;
      }
      fail_count = 0;

      // Publish: assign JPEG bytes directly from mmap buffer
      msg.header.stamp = now();
      msg.data.assign(frame.ptr, frame.ptr + frame.length);
      pub->publish(msg);

      cam.release_frame();

      // FPS logging every 2 seconds
      ++frame_count;
      auto elapsed = std::chrono::steady_clock::now() - fps_timer;
      if (elapsed >= std::chrono::seconds(2)) {
        double secs = std::chrono::duration<double>(elapsed).count();
        RCLCPP_INFO(get_logger(), "%s FPS: %.1f", tag.c_str(), frame_count / secs);
        frame_count = 0;
        fps_timer = std::chrono::steady_clock::now();
      }
    }

    cam.close();
    RCLCPP_INFO(get_logger(), "%s Stopped.", tag.c_str());
  }

  bool open_camera(V4L2Camera& cam, const CameraConfig& cfg, const std::string& tag) {
    if (!cam.open(cfg.device_path, width_, height_, fps_)) {
      RCLCPP_ERROR(get_logger(), "%s Failed to open %s", tag.c_str(), cfg.device_path.c_str());
      return false;
    }
    cam.apply_controls(cfg.settings);
    RCLCPP_INFO(get_logger(), "%s Opened: %s %dx%d @ %dfps [V4L2 mmap passthrough]",
      tag.c_str(), cfg.device_path.c_str(), width_, height_, fps_);
    return true;
  }

  bool reconnect(V4L2Camera& cam, const CameraConfig& cfg,
                 const std::string& tag, int max_retries) {
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
      RCLCPP_INFO(get_logger(), "%s Reconnect attempt %d/%d...",
        tag.c_str(), attempt, max_retries);
      std::this_thread::sleep_for(std::chrono::seconds(3));
      if (open_camera(cam, cfg, tag)) {
        RCLCPP_INFO(get_logger(), "%s Reconnected.", tag.c_str());
        return true;
      }
    }
    return false;
  }

  std::atomic<bool> running_;
  int width_;
  int height_;
  int fps_;
  std::vector<std::thread> threads_;
};

}  // namespace fisheye_camera

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fisheye_camera::FisheyeCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
