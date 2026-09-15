/**
 * @file post_process_node.cpp
 * @brief Unified ROS1/ROS2 post-processing node
 *
 * Uses ros_compat.hpp wrappers to minimize version-specific code.
 * Only service calls need #if/#endif due to different message generation.
 */

#include "utility/ros_compat.hpp"

#if ROS_VERSION_MAJOR == 1
#include <std_msgs/String.h>
#include "odin_ros_driver_rev1/GetCalibration.h"
#else
#include <std_msgs/msg/string.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include "odin_ros_driver_rev1/srv/get_calibration.hpp"
#endif

// cv_bridge header compatibility: Ubuntu 24.04 uses .hpp, earlier versions use .h
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <filesystem>

#include "color_undistort.hpp"
#include "disparity_to_pointcloud.hpp"
#include "jpeg_decoder.h"
#include "rawcloud_render.h"
#include "stereo_rectifier.hpp"
#include "yaml_config_loader.hpp"

using namespace ros_compat;

// Global Flags
static bool g_send_image_raw = false;
static bool g_send_image2_raw = false;
static bool g_send_cloud_render = false;
static bool g_send_ess_output = false;
static bool g_send_depth_pointcloud = false;

// Device info structure for multi-device support
struct DeviceInfo {
  std::string sn;
  std::string prefix;
  std::string ip;
};

// Parse device info JSON: {"sn":"xxx","prefix":"manifold/odin/xxx/","ip":"192.168.1.251"}
static bool ParseDeviceInfoJson(const std::string& json, DeviceInfo& info) {
  auto getValue = [&json](const std::string& key) -> std::string {
    std::string search = "\"" + key + "\":\"";
    size_t start = json.find(search);
    if (start == std::string::npos) return "";
    start += search.length();
    size_t end = json.find('"', start);
    if (end == std::string::npos) return "";
    return json.substr(start, end - start);
  };
  info.sn = getValue("sn");
  info.prefix = getValue("prefix");
  info.ip = getValue("ip");
  return !info.sn.empty() && !info.prefix.empty();
}

// Queue Item Types
struct ImageItem {
  Time stamp;
  CompressedImageConstPtr msg;
};

struct CloudItem {
  Time stamp;
  PointCloud2ConstPtr msg;
};

struct DecodedImageItem {
  Time stamp;
  Header header;
  cv::Mat image;
};

// Forward declaration
class PostProcessNode;

// DeviceProcessor - handles processing for a single device
class DeviceProcessor {
 public:
  DeviceProcessor(PostProcessNode* parent, const DeviceInfo& info);
  ~DeviceProcessor() {
    RC_LOG_INFO("DeviceProcessor destroying: %s, resetting pub/sub...", device_info_.sn.c_str());
    // Explicitly reset publishers and subscribers to ensure DDS cleanup
    image_sub_ = Subscriber<CompressedImage>();
    image2_sub_ = Subscriber<CompressedImage>();
    cloud_sub_ = Subscriber<PointCloud2>();
    disparity_sub_ = Subscriber<DisparityImage>();
    
    cloud_pub_ = Publisher<PointCloud2>();
    depth_cloud_pub_ = Publisher<PointCloud2>();
    image_pub_ = Publisher<Image>();
    image2_pub_ = Publisher<Image>();
    image_undistort_pub_ = Publisher<Image>();
    image2_undistort_pub_ = Publisher<Image>();
    left_rect_pub_ = Publisher<Image>();
    right_rect_pub_ = Publisher<Image>();
    left_info_pub_ = Publisher<CameraInfo>();
    right_info_pub_ = Publisher<CameraInfo>();
    
    RC_LOG_INFO("DeviceProcessor destroyed: %s", device_info_.sn.c_str());
  }

  const std::string& sn() const { return device_info_.sn; }
  const std::string& prefix() const { return device_info_.prefix; }
  bool isInitialized() const { return initialized_; }

  void imageCallback(const CompressedImageConstPtr& img);
  void imageCallback2(const CompressedImageConstPtr& img);
  void cloudCallback(const PointCloud2ConstPtr& cloud);
  void disparityCallback(const DisparityImageConstPtr& disp_msg);

  // Reinitialize all resolution-dependent components with new image dimensions
  void ReinitComponents(int img_width, int img_height);

 private:
  bool FetchCalibrationFromService(std::string& yaml_content_out);
  void InitializeComponents(const std::string& yaml_content);
  void SetupPubSub();

  template <typename T>
  void trimQueue(std::queue<T>& q) {
    while (static_cast<int>(q.size()) > max_queue_size_) q.pop();
  }
  void trimDecodedQueue(std::queue<DecodedImageItem>& q) {
    while (static_cast<int>(q.size()) > max_queue_size_) q.pop();
  }
  void tryProcessPair();
  void tryProcessEssPair();
  void processEssStereoPair(const DecodedImageItem& left, const DecodedImageItem& right);
  void renderCloud(const CompressedImageConstPtr& img, const PointCloud2ConstPtr& cloud);
  bool decodeCompressedImage(const CompressedImageConstPtr& img, cv::Mat& decoded);

  PostProcessNode* parent_;
  DeviceInfo device_info_;

  // Publishers
  Publisher<PointCloud2> cloud_pub_, depth_cloud_pub_;
  Publisher<Image> image_pub_, image2_pub_;
  Publisher<Image> image_undistort_pub_, image2_undistort_pub_;
  Publisher<Image> left_rect_pub_, right_rect_pub_;
  Publisher<CameraInfo> left_info_pub_, right_info_pub_;

  // Subscribers
  Subscriber<CompressedImage> image_sub_, image2_sub_;
  Subscriber<PointCloud2> cloud_sub_;
  Subscriber<DisparityImage> disparity_sub_;

  // Processing components
  rawCloudRender render_;
  colorUndistort m_undistort_0_, m_undistort_1_;
  odin::StereoRectifier stereo_rectifier_;
  odin::DisparityToPointCloud disp_to_cloud_;

  bool render_initialized_ = false;
  bool undistort_0_initialized_ = false;
  bool undistort_1_initialized_ = false;
  bool ess_initialized_ = false;
  bool initialized_ = false;  // Overall initialization status

  std::string yaml_content_;  // Stored calibration YAML for reinit on resolution change
  int current_img_width_ = 0;
  int current_img_height_ = 0;

  int ess_output_width_ = 960;
  int ess_output_height_ = 576;
  double ess_output_fov_ = 90.0;
  int max_queue_size_ = 10;
  Duration sync_tol_;
  std::string output_frame_id_;

  std::queue<ImageItem> image_queue_;
  std::queue<CloudItem> cloud_queue_;
  std::mutex mutex_;

  std::queue<DecodedImageItem> left_image_queue_, right_image_queue_;
  std::mutex ess_mutex_;

  cv::Mat latest_left_rect_;
  std::mutex disp_color_mutex_;

  // JPEG decoder (hardware or software)
  std::unique_ptr<odin_decoder::JpegDecoder> jpeg_decoder_;
};

// PostProcessNode Class - manages multiple DeviceProcessors
class PostProcessNode : public Node {
 public:
  PostProcessNode() : Node("postprocess_node") {
    // Load global parameters
    max_queue_size_ = param<int>("max_queue_size", 10);
    ess_output_width_ = param<int>("ess_output_width", 960);
    ess_output_height_ = param<int>("ess_output_height", 576);
    ess_output_fov_ = param<double>("ess_output_fov", 90.0);
    sync_tolerance_ms_ = param<int>("sync_tolerance_ms", 10);
    output_frame_id_ = param<std::string>("output_frame_id", "");
    // Topic prefix base for driver management topics (default: "manifold")
    topic_prefix_base_ = param<std::string>("topic_prefix", "manifold");

    // Start listening for device_online messages
    StartDeviceDiscovery();

    RC_LOG_INFO("PostProcessNode initialized, waiting for devices...");
  }

  // Accessor methods for DeviceProcessor
  int max_queue_size() const { return max_queue_size_; }
  int ess_output_width() const { return ess_output_width_; }
  int ess_output_height() const { return ess_output_height_; }
  double ess_output_fov() const { return ess_output_fov_; }
  int sync_tolerance_ms() const { return sync_tolerance_ms_; }
  const std::string& output_frame_id() const { return output_frame_id_; }

 private:
  // Helper to build driver topic name: "/{prefix}/driver/{suffix}" or "/driver/{suffix}" if prefix is empty
  // Unified with device topic format: /{prefix}/{model}/device{N}/...
  std::string BuildDriverTopicName(const std::string &suffix) const {
    if (topic_prefix_base_.empty()) {
      return "/driver/" + suffix;
    }
    return "/" + topic_prefix_base_ + "/driver/" + suffix;
  }

  void StartDeviceDiscovery() {
#if ROS_VERSION_MAJOR == 1
    // Subscribe to device_online topic for multi-device support
    // Use larger queue and transport hints to ensure all messages are received
    device_online_sub_ = nh().subscribe<std_msgs::String>(
        BuildDriverTopicName("device_online"), 100,
        [this](const boost::shared_ptr<std_msgs::String const>& msg) {
          OnDeviceInfoReceived(msg->data);
        });
    // Subscribe to device_offline topic for cleanup
    device_offline_sub_ = nh().subscribe<std_msgs::String>(
        BuildDriverTopicName("device_offline"), 10,
        [this](const boost::shared_ptr<std_msgs::String const>& msg) {
          OnDeviceOffline(msg->data);
        });
    // Subscribe to resolution_change topic for dynamic resolution switching
    resolution_change_sub_ = nh().subscribe<std_msgs::String>(
        BuildDriverTopicName("resolution_change"), 10,
        [this](const boost::shared_ptr<std_msgs::String const>& msg) {
          OnResolutionChange(msg->data);
        });
#else
    // Subscribe to device_online topic with transient_local QoS to receive latched messages
    device_online_sub_ = create_subscription<std_msgs::msg::String>(
        BuildDriverTopicName("device_online"), rclcpp::QoS(10).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) { OnDeviceInfoReceived(msg->data); });
    // Subscribe to device_offline topic for cleanup
    device_offline_sub_ = create_subscription<std_msgs::msg::String>(
        BuildDriverTopicName("device_offline"), rclcpp::QoS(10).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) { OnDeviceOffline(msg->data); });
    // Subscribe to resolution_change topic for dynamic resolution switching.
    // TRANSIENT_LOCAL matches the main driver publisher so a late-joining
    // post_process_node immediately receives the current resolution rather
    // than waiting for the next resolution event (which only fires when
    // the stream is restarted).
    resolution_change_sub_ = create_subscription<std_msgs::msg::String>(
        BuildDriverTopicName("resolution_change"),
        rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) { OnResolutionChange(msg->data); });
#endif
  }

  void OnDeviceInfoReceived(const std::string& json) {
    RC_LOG_INFO("Received device_online: %s", json.c_str());

    DeviceInfo info;
    if (!ParseDeviceInfoJson(json, info)) {
      RC_LOG_WARN("Failed to parse device_online JSON: %s", json.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(processors_mutex_);
      if (processors_.find(info.sn) != processors_.end()) {
        // Device already registered
        return;
      }
      // Mark as pending to avoid duplicate processing
      processors_[info.sn] = nullptr;
    }

    RC_LOG_INFO("New device discovered: sn=%s, prefix=%s, ip=%s", info.sn.c_str(),
                info.prefix.c_str(), info.ip.c_str());

    // Create processor in a separate thread to avoid blocking callback
    std::thread([this, info]() {
      auto processor = std::make_unique<DeviceProcessor>(this, info);

      std::lock_guard<std::mutex> lock(processors_mutex_);
      processors_[info.sn] = std::move(processor);
      RC_LOG_INFO("DeviceProcessor created for %s, total devices: %zu", info.sn.c_str(),
                  processors_.size());
    }).detach();
  }

  void OnResolutionChange(const std::string& json) {
    // JSON format: {"sn":"P030300006","width":640,"height":544}
    auto getStr = [&json](const std::string& key) -> std::string {
      std::string search = "\"" + key + "\":\"";
      size_t start = json.find(search);
      if (start == std::string::npos) return "";
      start += search.length();
      size_t end = json.find('"', start);
      if (end == std::string::npos) return "";
      return json.substr(start, end - start);
    };
    auto getInt = [&json](const std::string& key) -> int {
      std::string search = "\"" + key + "\":";
      size_t start = json.find(search);
      if (start == std::string::npos) return 0;
      start += search.length();
      return std::atoi(json.c_str() + start);
    };

    std::string sn = getStr("sn");
    int width = getInt("width");
    int height = getInt("height");
    if (sn.empty() || width <= 0 || height <= 0) {
      RC_LOG_WARN("Invalid resolution_change JSON: %s", json.c_str());
      return;
    }

    RC_LOG_INFO("Resolution change for %s: %dx%d", sn.c_str(), width, height);
    std::lock_guard<std::mutex> lock(processors_mutex_);
    auto it = processors_.find(sn);
    if (it != processors_.end() && it->second) {
      it->second->ReinitComponents(width, height);
    }
  }

  void OnDeviceOffline(const std::string& sn) {
    std::unique_ptr<DeviceProcessor> processor;

    {
      std::lock_guard<std::mutex> lock(processors_mutex_);
      auto it = processors_.find(sn);
      if (it == processors_.end()) {
        RC_LOG_WARN("Device offline notification for unknown device: %s", sn.c_str());
        return;
      }

      RC_LOG_INFO("Device offline: %s, removing DeviceProcessor", sn.c_str());
      processor = std::move(it->second);
      processors_.erase(it);
    }

    // Processor destructor handles cleanup (outside lock to avoid deadlock)
    processor.reset();
    RC_LOG_INFO("DeviceProcessor removed for %s, remaining devices: %zu", sn.c_str(),
                processors_.size());
  }

  // Global parameters
  int max_queue_size_ = 10;
  int ess_output_width_ = 960;
  int ess_output_height_ = 576;
  double ess_output_fov_ = 90.0;
  int sync_tolerance_ms_ = 10;
  std::string output_frame_id_;
  std::string topic_prefix_base_ = "manifold";  // Topic prefix base for driver management topics

  // Device management
  std::map<std::string, std::unique_ptr<DeviceProcessor>> processors_;
  std::mutex processors_mutex_;

#if ROS_VERSION_MAJOR == 1
  ros::Subscriber device_online_sub_;
  ros::Subscriber device_offline_sub_;
  ros::Subscriber resolution_change_sub_;
#else
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr device_online_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr device_offline_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr resolution_change_sub_;
#endif
};

// ============================================================================
// DeviceProcessor Implementation
// ============================================================================

DeviceProcessor::DeviceProcessor(PostProcessNode* parent, const DeviceInfo& info)
    : parent_(parent), device_info_(info), sync_tol_(durationFromMs(parent->sync_tolerance_ms())) {
  max_queue_size_ = parent->max_queue_size();
  ess_output_width_ = parent->ess_output_width();
  ess_output_height_ = parent->ess_output_height();
  ess_output_fov_ = parent->ess_output_fov();
  output_frame_id_ = parent->output_frame_id();

  // Initialize JPEG decoder (hardware or software)
  jpeg_decoder_ = odin_decoder::CreateDecoder();
  RC_LOG_INFO("[%s] JPEG decoder: %s", device_info_.sn.c_str(), jpeg_decoder_->GetName().c_str());

  std::string yaml_content;
  bool calib_ok = FetchCalibrationFromService(yaml_content);

  if (calib_ok) {
    yaml_content_ = yaml_content;  // Store for reinit on resolution change
    InitializeComponents(yaml_content);
  } else {
    RC_LOG_WARN("No calibration for device %s, advanced features disabled",
                device_info_.sn.c_str());
  }

  // Always setup pub/sub - basic image decoding works without calibration
  SetupPubSub();

  initialized_ = true;
  RC_LOG_INFO("DeviceProcessor initialized for %s (calib=%s)", device_info_.sn.c_str(),
              calib_ok ? "ok" : "none");
}

bool DeviceProcessor::FetchCalibrationFromService(std::string& yaml_content_out) {
  std::string service_name = device_info_.prefix + "get_calibration";
  RC_LOG_INFO("Waiting for calibration service: %s", service_name.c_str());
#if ROS_VERSION_MAJOR == 1
  if (!ros::service::waitForService(service_name, ros::Duration(60.0))) {
    RC_LOG_ERROR("Calibration service not available after 60 seconds");
    return false;
  }
  RC_LOG_INFO("Calibration service available, sending request...");
  odin_ros_driver_rev1::GetCalibration srv;
  if (!ros::service::call(service_name, srv)) {
    RC_LOG_ERROR("Failed to call calibration service");
    return false;
  }
  if (!srv.response.success) {
    RC_LOG_ERROR("Calibration service returned failure: %s", srv.response.message.c_str());
    return false;
  }
  RC_LOG_INFO("Received calibration data (%zu bytes)", srv.response.yaml_content.size());
  yaml_content_out = srv.response.yaml_content;
#else
  auto client = parent_->create_client<odin_ros_driver_rev1::srv::GetCalibration>(service_name);
  int wait_count = 0;
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (++wait_count > 60) {
      RC_LOG_ERROR("Calibration service not available after 60 seconds");
      return false;
    }
    RC_LOG_INFO("Waiting for calibration service... (%d/60)", wait_count);
  }
  RC_LOG_INFO("Calibration service available, sending request...");
  auto request = std::make_shared<odin_ros_driver_rev1::srv::GetCalibration::Request>();
  auto future = client->async_send_request(request);
  // Use wait_for instead of spin_until_future_complete (works in separate thread)
  auto status = future.wait_for(std::chrono::seconds(30));
  if (status != std::future_status::ready) {
    RC_LOG_ERROR("Calibration service call timed out");
    return false;
  }
  auto response = future.get();
  if (!response->success) {
    RC_LOG_ERROR("Calibration service returned failure: %s", response->message.c_str());
    return false;
  }
  RC_LOG_INFO("Received calibration data (%zu bytes)", response->yaml_content.size());
  yaml_content_out = response->yaml_content;
#endif
  return true;
}

void DeviceProcessor::InitializeComponents(const std::string& yaml_content) {
  int w = current_img_width_;
  int h = current_img_height_;
  bool use_resolution = (w > 0 && h > 0);

  if (use_resolution) {
    render_initialized_ = render_.initFromString(yaml_content, w, h);
  } else {
    render_initialized_ = render_.initFromString(yaml_content);
  }
  if (!render_initialized_) {
    RC_LOG_WARN("[%s] Failed to init rawCloudRender", device_info_.sn.c_str());
  }

  if (use_resolution) {
    undistort_0_initialized_ = m_undistort_0_.initFromString(yaml_content, w, h, 0);
  } else {
    undistort_0_initialized_ = m_undistort_0_.initFromString(yaml_content);
  }
  if (!undistort_0_initialized_) {
    RC_LOG_WARN("[%s] Failed to init m_undistort_0", device_info_.sn.c_str());
  }

  if (use_resolution) {
    undistort_1_initialized_ = m_undistort_1_.initFromString(yaml_content, w, h, 1);
  } else {
    undistort_1_initialized_ = m_undistort_1_.initFromString(yaml_content, "cam_1");
  }
  if (!undistort_1_initialized_) {
    RC_LOG_WARN("[%s] Failed to init m_undistort_1", device_info_.sn.c_str());
  }

  if (use_resolution) {
    ess_initialized_ = stereo_rectifier_.initFromString(yaml_content, w, h, ess_output_width_,
                                                        ess_output_height_, ess_output_fov_);
  } else {
    ess_initialized_ = stereo_rectifier_.initFromString(yaml_content, ess_output_width_,
                                                        ess_output_height_, ess_output_fov_);
  }
  if (!ess_initialized_) {
    RC_LOG_WARN("[%s] Failed to init StereoRectifier", device_info_.sn.c_str());
  } else {
    disp_to_cloud_.setParameters(stereo_rectifier_.getFocalLength(),
                                 stereo_rectifier_.getBaseline(), ess_output_width_,
                                 ess_output_height_);

    auto pc_config_loader = std::make_shared<YAMLConfigLoader>();
    if (pc_config_loader->loadFromString(yaml_content)) {
      std::vector<double> Tcl_0 = pc_config_loader->getVector<double>("Tcl_0");
      if (Tcl_0.size() >= 16) {
        disp_to_cloud_.setExtrinsic(Tcl_0);
      }
      bool transform_to_lidar =
          pc_config_loader->getValue<bool>("pointcloud.transform_to_lidar", false);
      std::string pc_frame_id = pc_config_loader->getValue<std::string>("pointcloud.frame_id",
                                                                        std::string("left_camera"));
      disp_to_cloud_.setTransformToLidar(transform_to_lidar);
      disp_to_cloud_.setOutputFrameId(pc_frame_id);
    }
  }
}

void DeviceProcessor::ReinitComponents(int img_width, int img_height) {
  if (yaml_content_.empty()) {
    RC_LOG_WARN("[%s] Cannot reinit: no calibration data available", device_info_.sn.c_str());
    return;
  }
  if (img_width == current_img_width_ && img_height == current_img_height_) {
    return;  // No change
  }
  RC_LOG_INFO("[%s] Resolution changed %dx%d -> %dx%d, reinitializing components...",
              device_info_.sn.c_str(), current_img_width_, current_img_height_, img_width,
              img_height);
  current_img_width_ = img_width;
  current_img_height_ = img_height;
  InitializeComponents(yaml_content_);
}

void DeviceProcessor::SetupPubSub() {
  const std::string& prefix = device_info_.prefix;

  cloud_pub_ = parent_->advertise<PointCloud2>(prefix + "cloud/render", 5);
  image_pub_ = parent_->advertise<Image>(prefix + "camera0/raw", 5);
  image2_pub_ = parent_->advertise<Image>(prefix + "camera1/raw", 5);
  image_undistort_pub_ = parent_->advertise<Image>(prefix + "camera0/undistort", 5);
  image2_undistort_pub_ = parent_->advertise<Image>(prefix + "camera1/undistort", 5);
  left_rect_pub_ = parent_->advertise<Image>(prefix + "camera0/rect", 5);
  right_rect_pub_ = parent_->advertise<Image>(prefix + "camera1/rect", 5);
  left_info_pub_ = parent_->advertise<CameraInfo>(prefix + "camera0/camera_info", 5);
  right_info_pub_ = parent_->advertise<CameraInfo>(prefix + "camera1/camera_info", 5);
  depth_cloud_pub_ = parent_->advertise<PointCloud2>(prefix + "stereo/depth", 5);

  image_sub_ = parent_->subscribe<CompressedImage>(
      prefix + "camera0/compressed", 5,
      [this](const CompressedImageConstPtr& msg) { imageCallback(msg); });
  image2_sub_ = parent_->subscribe<CompressedImage>(
      prefix + "camera1/compressed", 5,
      [this](const CompressedImageConstPtr& msg) { imageCallback2(msg); });
  cloud_sub_ = parent_->subscribe<PointCloud2>(
      prefix + "cloud/raw", 5, [this](const PointCloud2ConstPtr& msg) { cloudCallback(msg); });
  disparity_sub_ = parent_->subscribe<DisparityImage>(
      prefix + "stereo/disparity", 5,
      [this](const DisparityImageConstPtr& msg) { disparityCallback(msg); });
}

void DeviceProcessor::imageCallback(const CompressedImageConstPtr& img) {
  if (!g_send_image_raw) return;

  cv::Mat decoded;
  if (!decodeCompressedImage(img, decoded)) {
    return;
  }

  // Check for resolution change and reinitialize components if needed
  if (decoded.cols != current_img_width_ || decoded.rows != current_img_height_) {
    ReinitComponents(decoded.cols, decoded.rows);
  }

  // Clone to prevent decoder buffer reuse issues
  cv::Mat safe_decoded = decoded.clone();

  cv_bridge::CvImage cv_img;
  cv_img.header = img->header;
  cv_img.encoding = "bgr8";
  cv_img.image = safe_decoded;
  Image image_msg;
  cv_img.toImageMsg(image_msg);
  image_pub_.publish(image_msg);

  if (undistort_0_initialized_) {
    cv_bridge::CvImage undistort_img;
    undistort_img.header = img->header;
    m_undistort_0_.process(safe_decoded, undistort_img);
    undistort_img.header = img->header;
    Image undistort_msg;
    undistort_img.toImageMsg(undistort_msg);
    image_undistort_pub_.publish(undistort_msg);
  }

  if (g_send_cloud_render && render_initialized_) {
    std::lock_guard<std::mutex> lock(mutex_);
    image_queue_.push({img->header.stamp, img});
    trimQueue(image_queue_);
    tryProcessPair();
  }

  if (g_send_ess_output && ess_initialized_) {
    std::lock_guard<std::mutex> lock(ess_mutex_);
    left_image_queue_.push({img->header.stamp, img->header, safe_decoded.clone()});
    trimDecodedQueue(left_image_queue_);
    tryProcessEssPair();
  }
}

void DeviceProcessor::imageCallback2(const CompressedImageConstPtr& img) {
  if (!g_send_image2_raw) return;

  cv::Mat decoded;
  if (!decodeCompressedImage(img, decoded)) {
    return;
  }

  // Check for resolution change and reinitialize components if needed
  if (decoded.cols != current_img_width_ || decoded.rows != current_img_height_) {
    ReinitComponents(decoded.cols, decoded.rows);
  }

  // Clone to prevent decoder buffer reuse issues
  cv::Mat safe_decoded = decoded.clone();

  cv_bridge::CvImage cv_img;
  cv_img.header = img->header;
  cv_img.encoding = "bgr8";
  cv_img.image = safe_decoded;
  Image image_msg;
  cv_img.toImageMsg(image_msg);
  image2_pub_.publish(image_msg);

  if (undistort_1_initialized_) {
    cv_bridge::CvImage undistort_img;
    undistort_img.header = img->header;
    m_undistort_1_.process(safe_decoded, undistort_img);
    undistort_img.header = img->header;
    Image undistort_msg;
    undistort_img.toImageMsg(undistort_msg);
    image2_undistort_pub_.publish(undistort_msg);
  }

  if (g_send_ess_output && ess_initialized_) {
    std::lock_guard<std::mutex> lock(ess_mutex_);
    right_image_queue_.push({img->header.stamp, img->header, safe_decoded.clone()});
    trimDecodedQueue(right_image_queue_);
    tryProcessEssPair();
  }
}

void DeviceProcessor::cloudCallback(const PointCloud2ConstPtr& cloud) {
  if (!g_send_image_raw || !g_send_cloud_render || !render_initialized_) return;

  std::lock_guard<std::mutex> lock(mutex_);
  cloud_queue_.push({cloud->header.stamp, cloud});
  trimQueue(cloud_queue_);
  tryProcessPair();
}

void DeviceProcessor::disparityCallback(const DisparityImageConstPtr& disp_msg) {
  if (!g_send_depth_pointcloud || !ess_initialized_) return;

  try {
    cv::Mat disparity(disp_msg->image.height, disp_msg->image.width, CV_32FC1,
                      const_cast<uint8_t*>(disp_msg->image.data.data()));

    cv::Mat color_image;
    {
      std::lock_guard<std::mutex> lock(disp_color_mutex_);
      if (!latest_left_rect_.empty()) {
        color_image = latest_left_rect_.clone();
      }
    }

    PointCloud2 cloud_msg;
    cloud_msg.header = disp_msg->header;
    if (disp_to_cloud_.convertFromMsg(*disp_msg, color_image, cloud_msg)) {
      depth_cloud_pub_.publish(cloud_msg);
    }
  } catch (const std::exception& e) {
    RC_LOG_ERROR("[%s] Disparity conversion error: %s", device_info_.sn.c_str(), e.what());
  }
}

void DeviceProcessor::tryProcessPair() {
  while (!image_queue_.empty() && !cloud_queue_.empty()) {
    auto& img_item = image_queue_.front();
    auto& cloud_item = cloud_queue_.front();
    Duration dt = absDiff(img_item.stamp, cloud_item.stamp);

    if (dt <= sync_tol_) {
      renderCloud(img_item.msg, cloud_item.msg);
      image_queue_.pop();
      cloud_queue_.pop();
    } else {
      if (img_item.stamp < cloud_item.stamp) {
        image_queue_.pop();
      } else {
        cloud_queue_.pop();
      }
    }
  }
}

void DeviceProcessor::tryProcessEssPair() {
  while (!left_image_queue_.empty() && !right_image_queue_.empty()) {
    auto& left_item = left_image_queue_.front();
    auto& right_item = right_image_queue_.front();
    Duration dt = absDiff(left_item.stamp, right_item.stamp);

    if (dt <= sync_tol_) {
      processEssStereoPair(left_item, right_item);
      left_image_queue_.pop();
      right_image_queue_.pop();
    } else {
      if (left_item.stamp < right_item.stamp) {
        left_image_queue_.pop();
      } else {
        right_image_queue_.pop();
      }
    }
  }
}

void DeviceProcessor::processEssStereoPair(const DecodedImageItem& left,
                                           const DecodedImageItem& right) {
  cv::Mat left_rect, right_rect;
  stereo_rectifier_.rectify(left.image, right.image, left_rect, right_rect);

  if (g_send_depth_pointcloud) {
    std::lock_guard<std::mutex> lock(disp_color_mutex_);
    latest_left_rect_ = left_rect.clone();
  }

  cv::Mat left_rgb, right_rgb;
  cv::cvtColor(left_rect, left_rgb, cv::COLOR_BGR2RGB);
  cv::cvtColor(right_rect, right_rgb, cv::COLOR_BGR2RGB);

  Header header = left.header;
  header.frame_id = "left_camera";
  cv_bridge::CvImage left_cv_img(header, "rgb8", left_rgb);
  Image left_msg;
  left_cv_img.toImageMsg(left_msg);
  left_rect_pub_.publish(left_msg);

  header.frame_id = "right_camera";
  cv_bridge::CvImage right_cv_img(header, "rgb8", right_rgb);
  Image right_msg;
  right_cv_img.toImageMsg(right_msg);
  right_rect_pub_.publish(right_msg);

  auto left_info = stereo_rectifier_.getLeftCameraInfo();
  auto right_info = stereo_rectifier_.getRightCameraInfo();
  left_info.header = left.header;
  left_info.header.frame_id = "left_camera";
  right_info.header = left.header;
  right_info.header.frame_id = "right_camera";
  left_info_pub_.publish(left_info);
  right_info_pub_.publish(right_info);
}

void DeviceProcessor::renderCloud(const CompressedImageConstPtr& img,
                                  const PointCloud2ConstPtr& cloud) {
  cv::Mat decoded;
  if (!decodeCompressedImage(img, decoded)) return;

  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  cloud_xyz.reserve(cloud->width * cloud->height);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    cloud_xyz.emplace_back(*iter_x, *iter_y, *iter_z);
  }

  pcl::PointCloud<pcl::PointXYZRGBA> colored_cloud;
  render_.render(decoded, cloud_xyz, colored_cloud);
  if (colored_cloud.empty()) return;

  PointCloud2 output_msg;
  pcl::toROSMsg(colored_cloud, output_msg);
  output_msg.header = cloud->header;
  if (!output_frame_id_.empty()) output_msg.header.frame_id = output_frame_id_;
  cloud_pub_.publish(output_msg);
}

bool DeviceProcessor::decodeCompressedImage(const CompressedImageConstPtr& img, cv::Mat& decoded) {
  if (jpeg_decoder_->Decode(img->data.data(), img->data.size(), decoded)) {
    return true;
  }
  RC_LOG_ERROR("[%s] Failed to decode compressed image", device_info_.sn.c_str());
  return false;
}

#if ROS_VERSION_MAJOR == 1

int main(int argc, char** argv) {
  ros::init(argc, argv, "postprocess_node");
  ros::NodeHandle pnh("~");

  int post_process = 0;
  pnh.param("register_keys/post_process", post_process, 0);
  if (post_process == 0) {
    ROS_INFO("all post process disabled");
    return 0;
  }

  int val = 0;
  pnh.param("register_keys/send_image_raw", val, 0);
  g_send_image_raw = (val == 1);
  pnh.param("register_keys/send_image2_raw", val, 0);
  g_send_image2_raw = (val == 1);
  pnh.param("register_keys/send_cloud_render", val, 0);
  g_send_cloud_render = (val == 1);
  pnh.param("register_keys/send_ess_output", val, 0);
  g_send_ess_output = (val == 1);
  pnh.param("register_keys/send_depth_pointcloud", val, 0);
  g_send_depth_pointcloud = (val == 1);

  ROS_INFO("Flags: image_raw=%d, image2_raw=%d, cloud_render=%d, ess=%d, depth_pc=%d",
           g_send_image_raw, g_send_image2_raw, g_send_cloud_render, g_send_ess_output,
           g_send_depth_pointcloud);

  try {
    PostProcessNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in PostProcessNode: %s", e.what());
    return 1;
  }
  return 0;
}

#else  // ROS2

std::string getPackageSourceDirectory() {
  rcpputils::fs::path current_file(__FILE__);
  auto path = current_file.parent_path();
  while (!path.empty() && !rcpputils::fs::exists(path / "package.xml")) {
    path = path.parent_path();
  }
  if (path.empty()) {
    throw std::runtime_error("Failed to locate package root directory");
  }
  return path.string();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string package_path = getPackageSourceDirectory();
  std::string config_file = package_path + "/config/control_command.yaml";

  YAML::Node config = YAML::LoadFile(config_file);
  if (!config["register_keys"]) {
    throw std::runtime_error("Missing 'register_keys' section");
  }
  YAML::Node keys = config["register_keys"];

  int post_process = keys["post_process"] ? keys["post_process"].as<int>() : 0;
  if (post_process == 0) {
    std::cout << "post_process is disabled" << std::endl;
    return 0;
  }

  g_send_image_raw = keys["send_image_raw"] && keys["send_image_raw"].as<int>() == 1;
  g_send_image2_raw = keys["send_image2_raw"] && keys["send_image2_raw"].as<int>() == 1;
  g_send_cloud_render = keys["send_cloud_render"] && keys["send_cloud_render"].as<int>() == 1;
  g_send_ess_output = keys["send_ess_output"] && keys["send_ess_output"].as<int>() == 1;
  g_send_depth_pointcloud =
      keys["send_depth_pointcloud"] && keys["send_depth_pointcloud"].as<int>() == 1;

  std::cout << "Flags: image_raw=" << g_send_image_raw << ", image2_raw=" << g_send_image2_raw
            << ", cloud_render=" << g_send_cloud_render << ", ess=" << g_send_ess_output
            << ", depth_pc=" << g_send_depth_pointcloud << std::endl;

  auto node = std::make_shared<PostProcessNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#endif