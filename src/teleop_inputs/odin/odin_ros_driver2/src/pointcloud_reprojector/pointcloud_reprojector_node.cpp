/**
 * @file pointcloud_reprojector_node.cpp
 * @brief Standalone ROS node for point cloud to image reprojection
 * 
 * This node subscribes to device_online to auto-discover topic prefix,
 * then subscribes to cloud and image topics for reprojection.
 * 
 * Features:
 * - Auto-discovery via device_online topic
 * - Auto-fetch calibration via get_calibration service
 * - Raw mode: cloud/raw + camera0/raw -> depth/overlay
 * - Slam mode: cloud/slam + camera0/raw + odometry -> depth/overlay (interface ready)
 * 
 * Usage:
 *   ros2 run odin_ros_driver_rev1 pointcloud_reprojector_node
 *   ros2 run odin_ros_driver_rev1 pointcloud_reprojector_node --ros-args -p mode:=slam
 */

#include "pointcloud_reprojector.hpp"
#include "yaml_config_loader.hpp"

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <atomic>
#include <sstream>
#include <iostream>

#if defined(ODIN_ROS2)
#include <rcpputils/filesystem_helper.hpp>
#else
#include <ros/package.h>
#endif

// cv_bridge header compatibility
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#if defined(ODIN_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "odin_ros_driver_rev1/srv/get_calibration.hpp"

using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using ImageMsg = sensor_msgs::msg::Image;
using OdometryMsg = nav_msgs::msg::Odometry;
using StringMsg = std_msgs::msg::String;

#else
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "odin_ros_driver_rev1/GetCalibration.h"

using PointCloud2Msg = sensor_msgs::PointCloud2;
using ImageMsg = sensor_msgs::Image;
using OdometryMsg = nav_msgs::Odometry;
using StringMsg = std_msgs::String;
#endif

namespace odin {

// Parse a string field from JSON
static bool ParseJsonStringField(const std::string& json, const std::string& key,
                                  std::string& value_out) {
  std::string search = "\"" + key + "\":\"";
  size_t start = json.find(search);
  if (start == std::string::npos) return false;
  start += search.length();
  size_t end = json.find('"', start);
  if (end == std::string::npos) return false;
  value_out = json.substr(start, end - start);
  return !value_out.empty();
}

// Parse an int field from JSON
static int ParseJsonIntField(const std::string& json, const std::string& key) {
  std::string search = "\"" + key + "\":";
  size_t start = json.find(search);
  if (start == std::string::npos) return 0;
  start += search.length();
  return std::atoi(json.c_str() + start);
}

// Parse device info JSON: {"sn":"xxx","prefix":"manifold/odin/xxx/","ip":"192.168.1.251"}
static bool ParseDeviceInfoJson(const std::string& json, std::string& prefix_out,
                                 std::string& sn_out) {
  ParseJsonStringField(json, "sn", sn_out);
  return ParseJsonStringField(json, "prefix", prefix_out);
}

class PointCloudReprojectorNode {
 public:
#if defined(ODIN_ROS2)
  explicit PointCloudReprojectorNode(rclcpp::Node::SharedPtr node) : node_(node) {
    // Declare parameters
    node_->declare_parameter<std::string>("mode", "raw");
    node_->declare_parameter<double>("min_depth", 0.1);
    node_->declare_parameter<double>("max_depth", 50.0);
    node_->declare_parameter<int>("point_size", 2);
    node_->declare_parameter<bool>("use_depth_color", true);
    node_->declare_parameter<int>("sync_queue_size", 10);
    node_->declare_parameter<double>("sync_slop", 0.1);
    node_->declare_parameter<std::string>("topic_prefix", "manifold");

    init();
  }
#else
  explicit PointCloudReprojectorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
      : nh_(nh), pnh_(pnh) {
    init();
  }
#endif

 private:
  void init() {
    loadParameters();
    startDeviceDiscovery();

#if defined(ODIN_ROS2)
    RCLCPP_INFO(node_->get_logger(), "PointCloudReprojectorNode initialized");
    RCLCPP_INFO(node_->get_logger(), "  Mode: %s", mode_str_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  Waiting for device_online...");
#else
    ROS_INFO("PointCloudReprojectorNode initialized");
    ROS_INFO("  Mode: %s", mode_str_.c_str());
    ROS_INFO("  Waiting for device_online...");
#endif
  }

  void loadParameters() {
#if defined(ODIN_ROS2)
    mode_str_ = node_->get_parameter("mode").as_string();
    min_depth_ = node_->get_parameter("min_depth").as_double();
    max_depth_ = node_->get_parameter("max_depth").as_double();
    point_size_ = node_->get_parameter("point_size").as_int();
    use_depth_color_ = node_->get_parameter("use_depth_color").as_bool();
    sync_queue_size_ = node_->get_parameter("sync_queue_size").as_int();
    sync_slop_ = node_->get_parameter("sync_slop").as_double();
    topic_prefix_base_ = node_->get_parameter("topic_prefix").as_string();
#else
    pnh_.param<std::string>("mode", mode_str_, "raw");
    pnh_.param<double>("min_depth", min_depth_, 0.1);
    pnh_.param<double>("max_depth", max_depth_, 50.0);
    pnh_.param<int>("point_size", point_size_, 2);
    pnh_.param<bool>("use_depth_color", use_depth_color_, true);
    pnh_.param<int>("sync_queue_size", sync_queue_size_, 10);
    pnh_.param<double>("sync_slop", sync_slop_, 0.1);
    pnh_.param<std::string>("topic_prefix", topic_prefix_base_, "manifold");
#endif

    // Parse mode
    if (mode_str_ == "slam") {
      mode_ = ReprojectorMode::kSlamToImage;
    } else {
      mode_ = ReprojectorMode::kRawToImage;
    }
  }

  // Helper to build driver topic name: "/{prefix}/driver/{suffix}" or "/driver/{suffix}" if prefix is empty
  // Unified with device topic format: /{prefix}/{model}/device{N}/...
  std::string buildDriverTopicName(const std::string &suffix) const {
    if (topic_prefix_base_.empty()) {
      return "/driver/" + suffix;
    }
    return "/" + topic_prefix_base_ + "/driver/" + suffix;
  }

  void startDeviceDiscovery() {
#if defined(ODIN_ROS2)
    // Subscribe to device_online with transient_local QoS to receive latched messages
    device_online_sub_ = node_->create_subscription<StringMsg>(
        buildDriverTopicName("device_online"), rclcpp::QoS(10).transient_local(),
        [this](const StringMsg::SharedPtr msg) { onDeviceInfoReceived(msg->data); });
    // Subscribe to resolution_change for dynamic resolution updates.
    // TRANSIENT_LOCAL matches the main driver publisher so late-joining
    // reprojector instances immediately pick up the current resolution
    // instead of being stuck on YAML defaults until the next resolution event.
    resolution_change_sub_ = node_->create_subscription<StringMsg>(
        buildDriverTopicName("resolution_change"),
        rclcpp::QoS(1).transient_local(),
        [this](const StringMsg::SharedPtr msg) { onResolutionChange(msg->data); });
#else
    device_online_sub_ = nh_.subscribe<StringMsg>(
        buildDriverTopicName("device_online"), 10,
        [this](const boost::shared_ptr<StringMsg const>& msg) { onDeviceInfoReceived(msg->data); });
    resolution_change_sub_ = nh_.subscribe<StringMsg>(
        buildDriverTopicName("resolution_change"), 10,
        [this](const boost::shared_ptr<StringMsg const>& msg) { onResolutionChange(msg->data); });
#endif
  }

  void onResolutionChange(const std::string& json) {
    // JSON format: {"sn":"P030300006","width":640,"height":544}
    std::string sn;
    ParseJsonStringField(json, "sn", sn);
    int width = ParseJsonIntField(json, "width");
    int height = ParseJsonIntField(json, "height");

    if (sn.empty() || width <= 0 || height <= 0) {
#if defined(ODIN_ROS2)
      RCLCPP_WARN(node_->get_logger(), "Invalid resolution_change JSON: %s", json.c_str());
#else
      ROS_WARN("Invalid resolution_change JSON: %s", json.c_str());
#endif
      return;
    }

    // Only handle if SN matches our device
    if (!device_sn_.empty() && sn != device_sn_) {
      return;
    }

    // Skip if no real change
    if (width == current_img_width_ && height == current_img_height_) {
      return;
    }

#if defined(ODIN_ROS2)
    RCLCPP_INFO(node_->get_logger(), "Resolution change for %s: %dx%d -> %dx%d",
                sn.c_str(), current_img_width_, current_img_height_, width, height);
#else
    ROS_INFO("Resolution change for %s: %dx%d -> %dx%d", sn.c_str(),
             current_img_width_, current_img_height_, width, height);
#endif

    current_img_width_ = width;
    current_img_height_ = height;

    // Re-initialize reprojector with new dimensions if we already have calibration
    if (!yaml_content_.empty()) {
      setupReprojector();
    }
  }

  void onDeviceInfoReceived(const std::string& json) {
    if (initialized_.load()) return;  // Already initialized

    std::string prefix;
    std::string sn;
    if (!ParseDeviceInfoJson(json, prefix, sn)) {
#if defined(ODIN_ROS2)
      RCLCPP_WARN(node_->get_logger(), "Failed to parse device_online: %s", json.c_str());
#else
      ROS_WARN("Failed to parse device_online: %s", json.c_str());
#endif
      return;
    }

    topic_prefix_ = prefix;
    device_sn_ = sn;
#if defined(ODIN_ROS2)
    RCLCPP_INFO(node_->get_logger(), "Device discovered, sn: %s, prefix: %s",
                device_sn_.c_str(), topic_prefix_.c_str());
#else
    ROS_INFO("Device discovered, sn: %s, prefix: %s", device_sn_.c_str(), topic_prefix_.c_str());
#endif

    // Fetch calibration asynchronously
    fetchCalibrationAsync();
  }

  void fetchCalibrationAsync() {
#if defined(ODIN_ROS2)
    calibration_client_ = node_->create_client<odin_ros_driver_rev1::srv::GetCalibration>(
        topic_prefix_ + "get_calibration");

    if (!calibration_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "Calibration service not available yet, retrying...");
      // Retry after a short delay using a timer
      retry_timer_ = node_->create_wall_timer(
          std::chrono::seconds(1),
          [this]() {
            retry_timer_->cancel();
            fetchCalibrationAsync();
          });
      return;
    }

    auto request = std::make_shared<odin_ros_driver_rev1::srv::GetCalibration::Request>();
    calibration_client_->async_send_request(
        request,
        [this](rclcpp::Client<odin_ros_driver_rev1::srv::GetCalibration>::SharedFuture future) {
          onCalibrationResponse(future);
        });
#else
    // ROS1: use synchronous fetch
    if (fetchCalibration()) {
      setupReprojector();
      if (!initialized_.load()) {
        setupPubSub();
        initialized_.store(true);
      }
    }
#endif
  }

#if defined(ODIN_ROS2)
  void onCalibrationResponse(
      rclcpp::Client<odin_ros_driver_rev1::srv::GetCalibration>::SharedFuture future) {
    try {
      auto response = future.get();
      if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Calibration service returned failure");
        return;
      }

      yaml_content_ = response->yaml_content;
      RCLCPP_INFO(node_->get_logger(), "Calibration fetched successfully");

      // Now setup reprojector and pub/sub
      setupReprojector();
      if (!initialized_.load()) {
        setupPubSub();
        initialized_.store(true);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get calibration: %s", e.what());
    }
  }
#endif

  bool fetchCalibration() {
#if defined(ODIN_ROS2)
    return true;  // ROS2 uses async version
#else
    ros::ServiceClient client = nh_.serviceClient<odin_ros_driver_rev1::GetCalibration>(
        topic_prefix_ + "get_calibration");

    if (!client.waitForExistence(ros::Duration(5.0))) {
      ROS_ERROR("Calibration service not available");
      return false;
    }

    odin_ros_driver_rev1::GetCalibration srv;
    if (!client.call(srv) || !srv.response.success) {
      ROS_ERROR("Failed to call calibration service");
      return false;
    }

    yaml_content_ = srv.response.yaml_content;
    ROS_INFO("Calibration fetched successfully");
#endif

    return true;
  }

  void setupReprojector() {
    // Parse calibration YAML
    auto config_loader = std::make_shared<YAMLConfigLoader>();
    if (!config_loader->loadFromString(yaml_content_)) {
#if defined(ODIN_ROS2)
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse calibration YAML");
#else
      ROS_ERROR("Failed to parse calibration YAML");
#endif
      return;
    }

    // Use resolution from resolution_change topic if available, otherwise yaml/default
    int img_width = current_img_width_ > 0
                        ? current_img_width_
                        : config_loader->getValue<int>("image_width", 640);
    int img_height = current_img_height_ > 0
                         ? current_img_height_
                         : config_loader->getValue<int>("image_height", 544);
    current_img_width_ = img_width;
    current_img_height_ = img_height;

#if defined(ODIN_ROS2)
    RCLCPP_INFO(node_->get_logger(), "Initializing reprojector at %dx%d", img_width, img_height);
#else
    ROS_INFO("Initializing reprojector at %dx%d", img_width, img_height);
#endif

    // Initialize reprojector from YAML
    if (!reprojector_.initFromYaml(yaml_content_, 0, img_width, img_height)) {
#if defined(ODIN_ROS2)
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize reprojector from YAML");
#else
      ROS_ERROR("Failed to initialize reprojector from YAML");
#endif
      return;
    }

    // Get extrinsic Tcl_0 (camera <- lidar)
    std::vector<double> Tcl_0 = config_loader->getVector<double>("Tcl_0");
    if (Tcl_0.size() >= 16) {
      reprojector_.setStaticExtrinsic(Tcl_0);
    } else {
#if defined(ODIN_ROS2)
      RCLCPP_WARN(node_->get_logger(), "Tcl_0 not found or invalid, using identity");
#else
      ROS_WARN("Tcl_0 not found or invalid, using identity");
#endif
    }

    // Configure reprojector
    reprojector_.setMode(mode_);
    reprojector_.setDepthRange(min_depth_, max_depth_);
    reprojector_.setPointSize(point_size_);
    reprojector_.setUseDepthColor(use_depth_color_);
  }

  void setupPubSub() {
#if defined(ODIN_ROS2)
    auto qos = rclcpp::QoS(10).reliable();

    // Publishers (only overlay, no depth)
    overlay_pub_ = node_->create_publisher<ImageMsg>(topic_prefix_ + "reprojection/overlay", qos);

    RCLCPP_INFO(node_->get_logger(), "Publishing to: %sreprojection/overlay", topic_prefix_.c_str());

    // Setup subscribers based on mode
    if (mode_ == ReprojectorMode::kRawToImage) {
      setupRawModeSync();
    } else {
      setupSlamModeSync();
    }
#else
    // Publishers (only overlay, no depth)
    overlay_pub_ = nh_.advertise<ImageMsg>(topic_prefix_ + "reprojection/overlay", 10);

    ROS_INFO("Publishing to: %sreprojection/overlay", topic_prefix_.c_str());

    // Setup subscribers based on mode
    if (mode_ == ReprojectorMode::kRawToImage) {
      setupRawModeSyncROS1();
    } else {
      setupSlamModeSyncROS1();
    }
#endif
  }

#if defined(ODIN_ROS2)
  void setupRawModeSync() {
    // Sync mode: use message_filters to synchronize cloud and RGB
    std::string cloud_topic = topic_prefix_ + "cloud/raw";
    std::string image_topic = topic_prefix_ + "camera0/undistort";

    RCLCPP_INFO(node_->get_logger(), "Raw mode (sync): subscribing to %s and %s",
                cloud_topic.c_str(), image_topic.c_str());

    // Match publishers (reliable, volatile, depth>=5). Use larger queue (30)
    // to absorb 30fps RGB + 10fps cloud rate mismatch within sync window.
    auto qos = rclcpp::QoS(30)
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    cloud_sub_.subscribe(node_, cloud_topic, qos.get_rmw_qos_profile());
    image_sub_.subscribe(node_, image_topic, qos.get_rmw_qos_profile());

    // Sync queue size = 10, allow larger time tolerance
    raw_sync_ = std::make_shared<RawSync>(RawSyncPolicy(10), cloud_sub_, image_sub_);
    raw_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
    raw_sync_->registerCallback(std::bind(&PointCloudReprojectorNode::rawSyncCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));
  }

  void setupSlamModeSync() {
    // Sync mode for SLAM: synchronize cloud, RGB, and odom
    std::string cloud_topic = topic_prefix_ + "cloud/slam";
    std::string image_topic = topic_prefix_ + "camera0/undistort";
    std::string odom_topic = topic_prefix_ + "odometry";

    RCLCPP_INFO(node_->get_logger(), "Slam mode (sync): subscribing to %s, %s, %s",
                cloud_topic.c_str(), image_topic.c_str(), odom_topic.c_str());

    // Match publishers (reliable, volatile). Cloud/image depth=5, odom depth=100.
    auto qos_cloud_img = rclcpp::QoS(30)
                             .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                             .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    auto qos_odom = rclcpp::QoS(100)
                        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    cloud_sub_.subscribe(node_, cloud_topic, qos_cloud_img.get_rmw_qos_profile());
    image_sub_.subscribe(node_, image_topic, qos_cloud_img.get_rmw_qos_profile());
    odom_sub_.subscribe(node_, odom_topic, qos_odom.get_rmw_qos_profile());

    slam_sync_ = std::make_shared<SlamSync>(SlamSyncPolicy(10), 
                                             cloud_sub_, image_sub_, odom_sub_);
    slam_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
    slam_sync_->registerCallback(std::bind(&PointCloudReprojectorNode::slamSyncCallback, this,
                                           std::placeholders::_1, std::placeholders::_2,
                                           std::placeholders::_3));
  }

  void rawSyncCallback(const PointCloud2Msg::ConstSharedPtr& cloud_msg,
                       const ImageMsg::ConstSharedPtr& image_msg) {
    processRaw(cloud_msg, image_msg);
  }

  void slamSyncCallback(const PointCloud2Msg::ConstSharedPtr& cloud_msg,
                        const ImageMsg::ConstSharedPtr& image_msg,
                        const OdometryMsg::ConstSharedPtr& odom_msg) {
    processSlam(cloud_msg, image_msg, odom_msg);
  }

  void processRaw(const PointCloud2Msg::ConstSharedPtr& cloud_msg,
                  const ImageMsg::ConstSharedPtr& image_msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
      cv::Mat depth_out, overlay_out;

      if (reprojector_.reproject(*cloud_msg, cv_ptr->image, depth_out, overlay_out)) {
        publishResults(overlay_out, image_msg->header);
      }
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void processSlam(const PointCloud2Msg::ConstSharedPtr& cloud_msg,
                   const ImageMsg::ConstSharedPtr& image_msg,
                   const OdometryMsg::ConstSharedPtr& odom_msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
      cv::Mat depth_out, overlay_out;

      // Convert odom to 4x4 transform
      std::vector<double> T_body_world = odomToTransform(odom_msg);

      if (reprojector_.reprojectWithPose(*cloud_msg, T_body_world, cv_ptr->image, 
                                          depth_out, overlay_out)) {
        publishResults(overlay_out, image_msg->header);
      }
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  template<typename OdomPtr>
  std::vector<double> odomToTransform(const OdomPtr& odom) {
    std::vector<double> T(16, 0.0);
    double qx = odom->pose.pose.orientation.x;
    double qy = odom->pose.pose.orientation.y;
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    double tx = odom->pose.pose.position.x;
    double ty = odom->pose.pose.position.y;
    double tz = odom->pose.pose.position.z;

    double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    double xy = qx * qy, xz = qx * qz, yz = qy * qz;
    double wx = qw * qx, wy = qw * qy, wz = qw * qz;

    T[0] = 1.0 - 2.0 * (yy + zz); T[1] = 2.0 * (xy - wz); T[2] = 2.0 * (xz + wy); T[3] = tx;
    T[4] = 2.0 * (xy + wz); T[5] = 1.0 - 2.0 * (xx + zz); T[6] = 2.0 * (yz - wx); T[7] = ty;
    T[8] = 2.0 * (xz - wy); T[9] = 2.0 * (yz + wx); T[10] = 1.0 - 2.0 * (xx + yy); T[11] = tz;
    T[12] = 0.0; T[13] = 0.0; T[14] = 0.0; T[15] = 1.0;
    return T;
  }

  void publishResults(const cv::Mat& overlay, const std_msgs::msg::Header& header) {
    if (!overlay.empty()) {
      cv_bridge::CvImage overlay_msg;
      overlay_msg.header = header;
      overlay_msg.encoding = "bgr8";
      overlay_msg.image = overlay;
      overlay_pub_->publish(*overlay_msg.toImageMsg());
    }
  }

#else
  // ROS1 implementations
  void setupRawModeSyncROS1() {
    std::string cloud_topic = topic_prefix_ + "cloud/raw";
    std::string image_topic = topic_prefix_ + "camera0/undistort";

    ROS_INFO("Raw mode: subscribing to %s and %s", cloud_topic.c_str(), image_topic.c_str());

    cloud_sub_ros1_.subscribe(nh_, cloud_topic, 30);
    image_sub_ros1_.subscribe(nh_, image_topic, 30);

    raw_sync_ros1_ = std::make_shared<RawSyncROS1>(RawSyncPolicyROS1(sync_queue_size_), 
                                                    cloud_sub_ros1_, image_sub_ros1_);
    raw_sync_ros1_->registerCallback(boost::bind(&PointCloudReprojectorNode::rawSyncCallbackROS1, 
                                                  this, _1, _2));
  }

  void setupSlamModeSyncROS1() {
    std::string cloud_topic = topic_prefix_ + "cloud/slam";
    std::string image_topic = topic_prefix_ + "camera0/undistort";
    std::string odom_topic = topic_prefix_ + "odometry";

    ROS_INFO("Slam mode: subscribing to %s, %s, %s", 
             cloud_topic.c_str(), image_topic.c_str(), odom_topic.c_str());

    cloud_sub_ros1_.subscribe(nh_, cloud_topic, 30);
    image_sub_ros1_.subscribe(nh_, image_topic, 30);
    odom_sub_ros1_.subscribe(nh_, odom_topic, 100);

    slam_sync_ros1_ = std::make_shared<SlamSyncROS1>(SlamSyncPolicyROS1(sync_queue_size_),
                                                      cloud_sub_ros1_, image_sub_ros1_, odom_sub_ros1_);
    slam_sync_ros1_->registerCallback(boost::bind(&PointCloudReprojectorNode::slamSyncCallbackROS1,
                                                   this, _1, _2, _3));
  }

  void rawSyncCallbackROS1(const PointCloud2Msg::ConstPtr& cloud_msg,
                           const ImageMsg::ConstPtr& image_msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
      cv::Mat depth_out, overlay_out;

      if (reprojector_.reproject(*cloud_msg, cv_ptr->image, depth_out, overlay_out)) {
        publishResultsROS1(overlay_out, image_msg->header);
      }
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void slamSyncCallbackROS1(const PointCloud2Msg::ConstPtr& cloud_msg,
                            const ImageMsg::ConstPtr& image_msg,
                            const OdometryMsg::ConstPtr& odom_msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
      cv::Mat depth_out, overlay_out;

      std::vector<double> T_body_world = odomToTransformROS1(odom_msg);

      if (reprojector_.reprojectWithPose(*cloud_msg, T_body_world, cv_ptr->image,
                                          depth_out, overlay_out)) {
        publishResultsROS1(overlay_out, image_msg->header);
      }
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  std::vector<double> odomToTransformROS1(const OdometryMsg::ConstPtr& odom) {
    std::vector<double> T(16, 0.0);
    double qx = odom->pose.pose.orientation.x;
    double qy = odom->pose.pose.orientation.y;
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    double tx = odom->pose.pose.position.x;
    double ty = odom->pose.pose.position.y;
    double tz = odom->pose.pose.position.z;

    double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    double xy = qx * qy, xz = qx * qz, yz = qy * qz;
    double wx = qw * qx, wy = qw * qy, wz = qw * qz;

    T[0] = 1.0 - 2.0 * (yy + zz); T[1] = 2.0 * (xy - wz); T[2] = 2.0 * (xz + wy); T[3] = tx;
    T[4] = 2.0 * (xy + wz); T[5] = 1.0 - 2.0 * (xx + zz); T[6] = 2.0 * (yz - wx); T[7] = ty;
    T[8] = 2.0 * (xz - wy); T[9] = 2.0 * (yz + wx); T[10] = 1.0 - 2.0 * (xx + yy); T[11] = tz;
    T[12] = 0.0; T[13] = 0.0; T[14] = 0.0; T[15] = 1.0;
    return T;
  }

  void publishResultsROS1(const cv::Mat& overlay, const std_msgs::Header& header) {
    if (!overlay.empty()) {
      cv_bridge::CvImage overlay_msg;
      overlay_msg.header = header;
      overlay_msg.encoding = "bgr8";
      overlay_msg.image = overlay;
      overlay_pub_.publish(overlay_msg.toImageMsg());
    }
  }
#endif

 private:
#if defined(ODIN_ROS2)
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<StringMsg>::SharedPtr device_online_sub_;
  rclcpp::Subscription<StringMsg>::SharedPtr resolution_change_sub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr overlay_pub_;

  // Calibration service client and retry timer
  rclcpp::Client<odin_ros_driver_rev1::srv::GetCalibration>::SharedPtr calibration_client_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

  // Message filters for time synchronization
  message_filters::Subscriber<PointCloud2Msg> cloud_sub_;
  message_filters::Subscriber<ImageMsg> image_sub_;
  message_filters::Subscriber<OdometryMsg> odom_sub_;

  // Raw mode sync (2 topics)
  using RawSyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, ImageMsg>;
  using RawSync = message_filters::Synchronizer<RawSyncPolicy>;
  std::shared_ptr<RawSync> raw_sync_;

  // Slam mode sync (3 topics)
  using SlamSyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, ImageMsg, OdometryMsg>;
  using SlamSync = message_filters::Synchronizer<SlamSyncPolicy>;
  std::shared_ptr<SlamSync> slam_sync_;

#else
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber device_online_sub_;
  ros::Subscriber resolution_change_sub_;
  ros::Publisher overlay_pub_;

  // Message filters for time synchronization
  message_filters::Subscriber<PointCloud2Msg> cloud_sub_ros1_;
  message_filters::Subscriber<ImageMsg> image_sub_ros1_;
  message_filters::Subscriber<OdometryMsg> odom_sub_ros1_;

  // Raw mode sync (2 topics)
  using RawSyncPolicyROS1 = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, ImageMsg>;
  using RawSyncROS1 = message_filters::Synchronizer<RawSyncPolicyROS1>;
  std::shared_ptr<RawSyncROS1> raw_sync_ros1_;

  // Slam mode sync (3 topics)
  using SlamSyncPolicyROS1 = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, ImageMsg, OdometryMsg>;
  using SlamSyncROS1 = message_filters::Synchronizer<SlamSyncPolicyROS1>;
  std::shared_ptr<SlamSyncROS1> slam_sync_ros1_;
#endif

  // Core reprojector
  PointCloudReprojector reprojector_;

  // State
  std::atomic<bool> initialized_{false};
  std::string topic_prefix_;
  std::string device_sn_;
  std::string yaml_content_;
  int current_img_width_{0};
  int current_img_height_{0};

  // Parameters
  std::string mode_str_;
  ReprojectorMode mode_;
  double min_depth_;
  double max_depth_;
  int point_size_;
  bool use_depth_color_;
  int sync_queue_size_;
  double sync_slop_;
  std::string topic_prefix_base_ = "manifold";  // Topic prefix base for driver management topics
};

}  // namespace odin

// Helper: locate package source directory. Used to load config/control_command.yaml.
// ROS2: walk up from __FILE__ until package.xml is found.
// ROS1: use ros::package::getPath.
static std::string getPackageSourceDirectory() {
#if defined(ODIN_ROS2)
  rcpputils::fs::path current_file(__FILE__);
  auto path = current_file.parent_path();
  while (!path.empty() && !rcpputils::fs::exists(path / "package.xml")) {
    auto parent = path.parent_path();
    if (parent == path) break;
    path = parent;
  }
  if (path.empty() || !rcpputils::fs::exists(path / "package.xml")) {
    return "";
  }
  return path.string();
#else
  return ros::package::getPath("odin_ros_driver_rev1");
#endif
}

// Load reprojection options from config/control_command.yaml.
// Returns true if reprojection should be enabled; sets mode_out to "raw" or "slam".
static bool loadReprojectionConfig(std::string& mode_out) {
  mode_out = "raw";
  std::string pkg_path = getPackageSourceDirectory();
  if (pkg_path.empty()) {
    std::cout << "[reprojector] package dir not found, using defaults (enable=1, mode=raw)"
              << std::endl;
    return true;
  }
  std::string config_file = pkg_path + "/config/control_command.yaml";
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    if (!config["register_keys"]) {
      std::cout << "[reprojector] register_keys missing, using defaults" << std::endl;
      return true;
    }
    YAML::Node keys = config["register_keys"];
    int enable = keys["enable_reprojection"] ? keys["enable_reprojection"].as<int>() : 1;
    if (keys["reprojection_mode"]) {
      mode_out = keys["reprojection_mode"].as<std::string>();
    }
    std::cout << "[reprojector] enable_reprojection=" << enable
              << ", reprojection_mode=" << mode_out << std::endl;
    return enable != 0;
  } catch (const std::exception& e) {
    std::cerr << "[reprojector] Failed to load " << config_file << ": " << e.what()
              << ", using defaults" << std::endl;
    return true;
  }
}

#if defined(ODIN_ROS2)
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Load reprojection config from yaml
  std::string yaml_mode;
  bool enable = loadReprojectionConfig(yaml_mode);
  if (!enable) {
    std::cout << "[reprojector] reprojection disabled by config, exiting" << std::endl;
    rclcpp::shutdown();
    return 0;
  }

  // Pass yaml_mode as parameter override so it is applied before declare_parameter
  // inside the node's constructor (default value will be ignored in favor of override).
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("mode", yaml_mode)});
  auto node = std::make_shared<rclcpp::Node>("pointcloud_reprojector_node", options);
  odin::PointCloudReprojectorNode reprojector_node(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#else
int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_reprojector_node");

  // Load reprojection config from yaml
  std::string yaml_mode;
  bool enable = loadReprojectionConfig(yaml_mode);
  if (!enable) {
    ROS_INFO("[reprojector] reprojection disabled by config, exiting");
    return 0;
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // Inject yaml mode as private parameter default; can still be overridden via cmd-line
  if (!pnh.hasParam("mode")) {
    pnh.setParam("mode", yaml_mode);
  }
  odin::PointCloudReprojectorNode reprojector_node(nh, pnh);
  ros::spin();
  return 0;
}
#endif
