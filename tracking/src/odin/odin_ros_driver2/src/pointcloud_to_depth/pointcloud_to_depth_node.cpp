// @file pointcloud2depth_node.cpp
// @brief Multi-device ROS node that projects each Odin device's dToF point
//        cloud onto its undistorted camera image to produce a dense depth
//        map (data + visualizations).
//
// Architecture mirrors post_process_node:
//   - The top-level Pointcloud2DepthNode owns the global parameter set and
//     manages a map of per-device DeviceWorker objects keyed by serial number.
//   - device_online    -> spawn a new DeviceWorker for that sn
//   - device_offline   -> destroy the DeviceWorker (DDS pubs/subs released)
//   - resolution_change-> dispatched to the matching DeviceWorker
//
// Each DeviceWorker holds its own:
//   - get_calibration client and YAML content
//   - low-res projection canvas (Kcl, out_width/out_height)
//   - cloud/raw + camera0/undistort ApproximateTime synchronizer
//   - three publishers: depth / depth_color / depth_overlay
//   - first-frame latency one-shot log
//
// Published topics per device (under the discovered device prefix):
//   {prefix}pointcloud2depth/depth          sensor_msgs/Image  32FC1 (full-res)
//   {prefix}pointcloud2depth/depth_color    sensor_msgs/Image  bgr8  (jet)
//   {prefix}pointcloud2depth/depth_overlay  sensor_msgs/Image  bgr8  (jet on RGB)
//
// The plain-types projection core (pointcloud2depth_node.hpp) is intentionally
// header-only and free of PCL/Eigen/OpenCV/ROS so it can be lifted into the
// device-side SDK as a pure C/C++ module without modification.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#if defined(ODIN_ROS2)
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "odin_ros_driver_rev1/srv/get_calibration.hpp"

using StringMsg      = std_msgs::msg::String;
using ImageMsg       = sensor_msgs::msg::Image;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
#else
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include "odin_ros_driver_rev1/GetCalibration.h"

using StringMsg      = std_msgs::String;
using ImageMsg       = sensor_msgs::Image;
using PointCloud2Msg = sensor_msgs::PointCloud2;
#endif

#include "pointcloud_to_depth_node.hpp"
#include "yaml_config_loader.hpp"

namespace odin {

// ---------------------------------------------------------------------------
// Small JSON helpers (same flavor as pointcloud_reprojector_node).
// ---------------------------------------------------------------------------
static bool ParseJsonStringField(const std::string& json, const std::string& key,
                                 std::string& value_out) {
  const std::string search = "\"" + key + "\":\"";
  size_t start = json.find(search);
  if (start == std::string::npos) return false;
  start += search.length();
  const size_t end = json.find('"', start);
  if (end == std::string::npos) return false;
  value_out = json.substr(start, end - start);
  return !value_out.empty();
}

static int ParseJsonIntField(const std::string& json, const std::string& key) {
  const std::string search = "\"" + key + "\":";
  const size_t start = json.find(search);
  if (start == std::string::npos) return 0;
  return std::atoi(json.c_str() + start + search.length());
}

// ---------------------------------------------------------------------------
// Per-device metadata parsed out of the driver's device_online JSON.
// ---------------------------------------------------------------------------
struct DeviceInfo {
  std::string sn;       // device serial number
  std::string prefix;   // already includes leading + trailing '/'
};

static bool ParseDeviceInfoJson(const std::string& json, DeviceInfo& info) {
  ParseJsonStringField(json, "sn", info.sn);
  ParseJsonStringField(json, "prefix", info.prefix);
  return !info.sn.empty() && !info.prefix.empty();
}

// ---------------------------------------------------------------------------
// Shared tunables. The manager loads these once from ROS params, then passes
// a const reference into every per-device worker so behavior stays uniform.
// ---------------------------------------------------------------------------
struct Pointcloud2DepthConfig {
  // Projection
  float scale               = 7.0f;   // low-res canvas = img / scale
  float z_min               = 0.1f;
  float z_max               = 50.0f;
  int   dilate_radius       = 1;
  bool  use_z_buffer        = false;
  float edge_grad_threshold = 0.75f;  // Sobel threshold on full-res depth (m)
  // Jet colormap normalization (m). vis_depth_min < 0 -> per-frame auto.
  float vis_depth_min = 0.1f;
  float vis_depth_max = 5.0f;
  // Synchronization
  int    sync_queue_size = 10;
  double sync_slop_sec   = 0.1;
  // Topic naming (suffixes appended to the per-device prefix)
  std::string cloud_topic_suffix  = "cloud/raw";
  std::string image_topic_suffix  = "camera0/undistort";
  std::string output_topic_suffix = "pointcloud2depth";
};

// ---------------------------------------------------------------------------
// DeviceWorker: encapsulates everything needed to process one device. One
// instance per discovered serial number; lifetime managed by the manager
// via device_online / device_offline.
// ---------------------------------------------------------------------------
class DeviceWorker {
 public:
#if defined(ODIN_ROS2)
  DeviceWorker(rclcpp::Node::SharedPtr node, DeviceInfo info,
               const Pointcloud2DepthConfig& cfg)
      : node_(std::move(node)), info_(std::move(info)), cfg_(cfg) {
    logInfo("Worker created (prefix=" + info_.prefix + ")");
    fetchCalibrationAsync();
  }
#else
  DeviceWorker(ros::NodeHandle& nh, DeviceInfo info,
               const Pointcloud2DepthConfig& cfg)
      : nh_(nh), info_(std::move(info)), cfg_(cfg) {
    logInfo("Worker created (prefix=" + info_.prefix + ")");
    fetchCalibrationAsync();
  }
#endif

  ~DeviceWorker() {
    logInfo("Worker destroying");
#if defined(ODIN_ROS2)
    // Reset pubs/subs explicitly so topics disappear immediately on unplug.
    depth_pub_.reset();
    depth_color_pub_.reset();
    depth_overlay_pub_.reset();
    calibration_client_.reset();
    if (retry_timer_) retry_timer_->cancel();
    retry_timer_.reset();
    sync_.reset();
    cloud_sub_.unsubscribe();
    image_sub_.unsubscribe();
#else
    sync_.reset();
    cloud_sub_.unsubscribe();
    image_sub_.unsubscribe();
    depth_pub_.shutdown();
    depth_color_pub_.shutdown();
    depth_overlay_pub_.shutdown();
#endif
  }

  DeviceWorker(const DeviceWorker&) = delete;
  DeviceWorker& operator=(const DeviceWorker&) = delete;

  const std::string& sn()     const { return info_.sn; }
  const std::string& prefix() const { return info_.prefix; }

  // Manager dispatches resolution_change here. Safe to call before the worker
  // finishes calibrating; rebuildKcl picks up the new dimensions later.
  void onResolutionChange(int width, int height) {
    if (width <= 0 || height <= 0) return;
    if (width == img_width_ && height == img_height_) return;
    logInfo("Resolution change: " + std::to_string(img_width_) + "x" +
            std::to_string(img_height_) + " -> " +
            std::to_string(width) + "x" + std::to_string(height));
    img_width_  = width;
    img_height_ = height;
    if (!yaml_content_.empty()) {
      rebuildKcl();
    }
  }

 private:
  void fetchCalibrationAsync() {
#if defined(ODIN_ROS2)
    calibration_client_ = node_->create_client<odin_ros_driver_rev1::srv::GetCalibration>(
        info_.prefix + "get_calibration");
    if (!calibration_client_->wait_for_service(std::chrono::seconds(1))) {
      logWarn("get_calibration not available yet, retrying in 1s...");
      retry_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
        retry_timer_->cancel();
        fetchCalibrationAsync();
      });
      return;
    }
    auto request = std::make_shared<odin_ros_driver_rev1::srv::GetCalibration::Request>();
    calibration_client_->async_send_request(
        request,
        [this](rclcpp::Client<odin_ros_driver_rev1::srv::GetCalibration>::SharedFuture f) {
          try {
            auto response = f.get();
            if (!response->success) {
              logError("get_calibration returned failure");
              return;
            }
            yaml_content_ = response->yaml_content;
            logInfo("Calibration fetched.");
            rebuildKcl();
            setupPubSub();
          } catch (const std::exception& e) {
            logError(std::string("get_calibration future exception: ") + e.what());
          }
        });
#else
    ros::ServiceClient client = nh_.serviceClient<odin_ros_driver_rev1::GetCalibration>(
        info_.prefix + "get_calibration");
    if (!client.waitForExistence(ros::Duration(5.0))) {
      logError("get_calibration service not available");
      return;
    }
    odin_ros_driver_rev1::GetCalibration srv;
    if (!client.call(srv) || !srv.response.success) {
      logError("Failed to call get_calibration");
      return;
    }
    yaml_content_ = srv.response.yaml_content;
    logInfo("Calibration fetched.");
    rebuildKcl();
    setupPubSub();
#endif
  }

  // Parse YAML, resolve cam key by resolution, build DepthParams and the two Kcls.
  bool rebuildKcl() {
    YAMLConfigLoader loader;
    if (!loader.loadFromString(yaml_content_)) {
      logError("Failed to parse calibration YAML");
      return false;
    }

    // Pick resolution: prefer dynamic resolution_change, else YAML default.
    int img_w = img_width_  > 0 ? img_width_  : loader.getValue<int>("image_width", 640);
    int img_h = img_height_ > 0 ? img_height_ : loader.getValue<int>("image_height", 544);
    img_width_  = img_w;
    img_height_ = img_h;

    // Resolve cam key (uses the same resolver as the existing reprojector).
    YAML::Node root = YAML::Load(yaml_content_);
    const std::string cam_key = yaml_utils::ResolveCamKey(root, /*cam_id=*/0, img_w, img_h);
    if (cam_key.empty()) {
      logError("No calibration node found for cam_0 at " + std::to_string(img_w) +
               "x" + std::to_string(img_h));
      return false;
    }
    YAML::Node cam = root[cam_key];

    depth_core::DepthParams params{};
    params.A11 = static_cast<float>(yaml_utils::safeGet<double>(cam, "A11", 500.0));
    params.A12 = static_cast<float>(yaml_utils::safeGet<double>(cam, "A12", 0.0));
    params.A22 = static_cast<float>(yaml_utils::safeGet<double>(cam, "A22", 500.0));
    params.u0  = static_cast<float>(yaml_utils::safeGet<double>(cam, "u0",  img_w / 2.0));
    params.v0  = static_cast<float>(yaml_utils::safeGet<double>(cam, "v0",  img_h / 2.0));

    // Extrinsic Tcl_0 (camera <- lidar), row-major, 16 doubles in YAML.
    std::vector<double> tcl_v = loader.getVector<double>("Tcl_0");
    if (tcl_v.size() < 16) {
      logWarn("Tcl_0 missing/invalid in calibration. Using identity.");
      for (int i = 0; i < 16; ++i) params.Tcl[i] = (i % 5 == 0) ? 1.0f : 0.0f;
    } else {
      for (int i = 0; i < 16; ++i) params.Tcl[i] = static_cast<float>(tcl_v[i]);
    }

    params.scale         = cfg_.scale;
    params.out_width     = std::max(1, static_cast<int>(img_w / cfg_.scale));
    params.out_height    = std::max(1, static_cast<int>(img_h / cfg_.scale));
    params.z_min         = cfg_.z_min;
    params.z_max         = cfg_.z_max;
    params.dilate_radius = cfg_.dilate_radius;

    out_width_  = params.out_width;
    out_height_ = params.out_height;

    depth_core::compute_kcl(params, Kcl_depth_);

    {
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "Calibration ready. img=%dx%d -> depth=%dx%d (scale=%.2f) "
                    "fx=%.2f fy=%.2f cx=%.2f cy=%.2f z=[%.2f,%.2f] dilate=%d",
                    img_w, img_h, params.out_width, params.out_height, params.scale,
                    params.A11, params.A22, params.u0, params.v0,
                    params.z_min, params.z_max, params.dilate_radius);
      logInfo(buf);
    }
    return true;
  }

  void setupPubSub() {
    if (initialized_.exchange(true)) return;  // already set up

    const std::string depth_topic         = info_.prefix + cfg_.output_topic_suffix + "/depth";
    const std::string depth_color_topic   = info_.prefix + cfg_.output_topic_suffix + "/depth_color";
    const std::string depth_overlay_topic = info_.prefix + cfg_.output_topic_suffix + "/depth_overlay";

#if defined(ODIN_ROS2)
    // Image streams use RELIABLE + KEEP_LAST(5) + VOLATILE: this matches the
    // RViz Image display default (RELIABLE) so RViz can subscribe at all,
    // while the small KEEP_LAST queue prevents stale-frame backpressure that
    // caused the previous stutter (1296x1600 32FC1 = ~8 MB/frame; a deep
    // queue would accumulate and the viewer would always lag behind).
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    depth_pub_         = node_->create_publisher<ImageMsg>(depth_topic,         qos);
    depth_color_pub_   = node_->create_publisher<ImageMsg>(depth_color_topic,   qos);
    depth_overlay_pub_ = node_->create_publisher<ImageMsg>(depth_overlay_topic, qos);

    auto sub_qos = rclcpp::QoS(30)
                       .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                       .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    cloud_sub_.subscribe(node_, info_.prefix + cfg_.cloud_topic_suffix, sub_qos.get_rmw_qos_profile());
    image_sub_.subscribe(node_, info_.prefix + cfg_.image_topic_suffix, sub_qos.get_rmw_qos_profile());

    sync_ = std::make_shared<Sync>(SyncPolicy(cfg_.sync_queue_size), cloud_sub_, image_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(cfg_.sync_slop_sec));
    sync_->registerCallback(
        std::bind(&DeviceWorker::syncCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
#else
    depth_pub_         = nh_.advertise<ImageMsg>(depth_topic,         10);
    depth_color_pub_   = nh_.advertise<ImageMsg>(depth_color_topic,   10);
    depth_overlay_pub_ = nh_.advertise<ImageMsg>(depth_overlay_topic, 10);

    cloud_sub_.subscribe(nh_, info_.prefix + cfg_.cloud_topic_suffix, 30);
    image_sub_.subscribe(nh_, info_.prefix + cfg_.image_topic_suffix, 30);
    sync_ = std::make_shared<Sync>(SyncPolicy(cfg_.sync_queue_size), cloud_sub_, image_sub_);
    sync_->registerCallback(boost::bind(&DeviceWorker::syncCallback, this, _1, _2));
#endif

    logInfo("Publishing: " + depth_topic);
    logInfo("Publishing: " + depth_color_topic);
    logInfo("Publishing: " + depth_overlay_topic);
  }

  // --------------------------------------------------------------------
  // Hot path
  // --------------------------------------------------------------------
#if defined(ODIN_ROS2)
  void syncCallback(const PointCloud2Msg::ConstSharedPtr& cloud,
                    const ImageMsg::ConstSharedPtr& image) {
    processFrame(*cloud, image);
  }
#else
  void syncCallback(const PointCloud2Msg::ConstPtr& cloud,
                    const ImageMsg::ConstPtr& image) {
    processFrame(*cloud, image);
  }
#endif

  template <typename ImagePtr>
  void processFrame(const PointCloud2Msg& cloud, const ImagePtr& image_msg) {
    if (out_width_ <= 0 || out_height_ <= 0) return;  // not yet calibrated

    // One-shot end-to-end timing: from the moment we enter the synchronized
    // callback to right after the last image is published. Useful to compare
    // against budget without spamming the log every frame.
    const auto frame_t0 = std::chrono::steady_clock::now();

    // ---- Extract xyz array from PointCloud2 (assumes float32 x/y/z fields) ----
    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto& f : cloud.fields) {
      if      (f.name == "x") x_off = f.offset;
      else if (f.name == "y") y_off = f.offset;
      else if (f.name == "z") z_off = f.offset;
    }
    if (x_off < 0 || y_off < 0 || z_off < 0) {
      logWarn("PointCloud2 missing x/y/z fields");
      return;
    }

    const uint8_t* data    = cloud.data.data();
    const size_t   step    = cloud.point_step;
    const size_t   n_total = static_cast<size_t>(cloud.width) * cloud.height;

    std::vector<float> xyz;
    xyz.reserve(n_total * 3);
    for (size_t i = 0; i < n_total; ++i) {
      const uint8_t* p = data + i * step;
      const float x = *reinterpret_cast<const float*>(p + x_off);
      const float y = *reinterpret_cast<const float*>(p + y_off);
      const float z = *reinterpret_cast<const float*>(p + z_off);
      xyz.push_back(x);
      xyz.push_back(y);
      xyz.push_back(z);
    }

    // ---- Decode RGB image for overlay color sampling ----
    cv::Mat rgb;
    try {
      rgb = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      logWarn(std::string("cv_bridge exception: ") + e.what());
      return;
    }

    // ---- Safety net for late-joining startup ----
    // The driver may publish resolution_change BEFORE this node subscribes
    // (even with TRANSIENT_LOCAL there are races at first launch). The
    // synchronized image is the ground truth for current resolution, so if
    // it disagrees with our internal state we resize the projection canvas.
    if (!rgb.empty() && !yaml_content_.empty() &&
        (rgb.cols != img_width_ || rgb.rows != img_height_)) {
      logInfo("Image dims differ from internal state (" +
              std::to_string(img_width_) + "x" + std::to_string(img_height_) +
              " -> " + std::to_string(rgb.cols) + "x" + std::to_string(rgb.rows) +
              "), rebuilding Kcl");
      img_width_  = rgb.cols;
      img_height_ = rgb.rows;
      rebuildKcl();
      if (out_width_ <= 0 || out_height_ <= 0) return;
    }

    if (rgb.empty()) return;

    // ---- Depth path: low-res depth map via plain-types core ----
    std::vector<float> depth_buf(static_cast<size_t>(out_width_) * out_height_, 0.0f);
    const uint32_t n_pts = static_cast<uint32_t>(n_total);
    const int n_written = depth_core::pointcloud_to_depth(
        xyz.data(), n_pts, Kcl_depth_,
        out_width_, out_height_, cfg_.dilate_radius,
        cfg_.z_min, cfg_.z_max, cfg_.use_z_buffer, depth_buf.data());

    // ---- Step 3: nearest-neighbor upsample to full image resolution ----
    // Per spec: project on a low-res canvas (W/scale x H/scale), then NN-upsample
    // to the camera's full resolution to produce a dense depth map aligned with rgb.
    cv::Mat depth_lowres(out_height_, out_width_, CV_32FC1, depth_buf.data());
    cv::Mat depth_full;
    cv::resize(depth_lowres, depth_full, cv::Size(rgb.cols, rgb.rows),
               0, 0, cv::INTER_NEAREST);

    // ---- Step 4: Sobel edge suppression on FULL-RES depth ----
    // We run Sobel on the upsampled depth (not the low-res grid) on purpose:
    // doing it on low-res produces an edge mask that, after NN-upsample, is
    // ~scale pixels wide on the full image, which visibly over-erases depth
    // along discontinuities. Full-res Sobel keeps the wipeout band to ~2 px,
    // matching the visual fidelity the doc spec / customer expects.
    if (cfg_.edge_grad_threshold > 0.0f) {
      cv::Mat gx, gy, grad;
      cv::Sobel(depth_full, gx, CV_32F, 1, 0, 3);
      cv::Sobel(depth_full, gy, CV_32F, 0, 1, 3);
      cv::magnitude(gx, gy, grad);
      cv::Mat edge_mask = grad > cfg_.edge_grad_threshold;  // 8UC1, 0 or 255
      depth_full.setTo(0.0f, edge_mask);
    }

    // ---- Publish depth (32FC1, meters, full resolution; data product) ----
    publishImage(depth_full, "32FC1", image_msg->header, depth_pub_);

    // ---- Build jet-color visualization (black where depth is invalid) ----
    // This is the key disambiguation: in 32FC1 grayscale, both "near" and
    // "no depth" look dark. Jet colormap makes "valid but near" colored
    // (red/yellow) while keeping invalid pixels truly black.
    cv::Mat jet_color, valid_mask;
    makeDepthJet(depth_full, jet_color, valid_mask);

    // ---- Publish depth_color (pure jet visualization, OpenCV-style) ----
    publishImage(jet_color, "bgr8", image_msg->header, depth_color_pub_);

    // ---- Publish depth_overlay (jet blended onto camera image) ----
    cv::Mat depth_overlay = blendJetOnImage(jet_color, valid_mask, rgb);
    publishImage(depth_overlay, "bgr8", image_msg->header, depth_overlay_pub_);

    // One-shot end-to-end latency log (subscription -> all publishes done).
    if (!timing_logged_.exchange(true)) {
      const auto dt = std::chrono::steady_clock::now() - frame_t0;
      const double ms =
          std::chrono::duration<double, std::milli>(dt).count();
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "First-frame pipeline latency: %.3f ms (n_pts=%u, %dx%d)",
                    ms, static_cast<unsigned>(n_pts), rgb.cols, rgb.rows);
      logInfo(buf);
    }

    (void)n_written;  // for future stats
  }

  // Render the depth map as an OpenCV-style jet visualization where:
  //   - valid pixels (depth > 0) are mapped to the JET colormap by
  //     min-max-normalizing within the current frame
  //   - invalid pixels (depth == 0) stay solid black so the customer can
  //     immediately tell "no depth here" from "near object"
  // Outputs:
  //   color_out : BGR8 of the same size as depth, black where invalid
  //   mask_out  : 8UC1 (0 / 255) marking valid pixels for downstream blending
  void makeDepthJet(const cv::Mat& depth, cv::Mat& color_out, cv::Mat& mask_out) {
    color_out = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC3);
    mask_out  = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1);
    if (depth.empty()) return;

    // 1) Valid mask (vectorized): mask_out is 8UC1, 255 where depth > 0.
    cv::compare(depth, 0.0f, mask_out, cv::CMP_GT);

    // 2) Pick the normalization range.
    //    - Default: fixed [cfg_.vis_depth_min, cfg_.vis_depth_max] (absolute coloring,
    //      a given physical depth always maps to the same color).
    //    - If cfg_.vis_depth_min < 0: per-frame auto-normalize (relative coloring;
    //      maximizes contrast but the same depth changes color across frames).
    float dmin, dmax;
    if (cfg_.vis_depth_min >= 0.0f && cfg_.vis_depth_max > cfg_.vis_depth_min) {
      dmin = cfg_.vis_depth_min;
      dmax = cfg_.vis_depth_max;
    } else {
      // cv::minMaxLoc is SIMD-accelerated and supports a mask, so we get the
      // per-frame valid-only [min, max] in one vectorized pass.
      double dmin_d = 0.0, dmax_d = 0.0;
      cv::minMaxLoc(depth, &dmin_d, &dmax_d, nullptr, nullptr, mask_out);
      if (dmax_d <= dmin_d) {
        // No valid pixels (or all identical) -> nothing meaningful to render.
        if (dmax_d == 0.0 && dmin_d == 0.0) return;
        dmax_d = dmin_d + 1e-3;
      }
      dmin = static_cast<float>(dmin_d);
      dmax = static_cast<float>(dmax_d);
    }

    // 3) Clamp depth into [dmin, dmax] then linearly map to 8U [1..255] via a
    //    single SIMD-accelerated convertTo (much faster than the per-pixel
    //    loop we used to do here).
    //
    //    The clamp leaves invalid pixels at dmin (from cv::max), so they would
    //    map to 1 (dark blue) after convertTo. We wipe those back to 0 using
    //    the valid mask so the final image keeps invalid pixels truly black.
    cv::Mat clamped;
    cv::max(depth, dmin, clamped);
    cv::min(clamped, dmax, clamped);
    const double alpha = 254.0 / (dmax - dmin);
    const double beta  = 1.0 - 254.0 * static_cast<double>(dmin) / (dmax - dmin);
    cv::Mat d8;
    clamped.convertTo(d8, CV_8UC1, alpha, beta);

    // 4) Apply JET colormap, then force invalid pixels to pure black so the
    //    customer can distinguish "no depth here" from "near object".
    cv::applyColorMap(d8, color_out, cv::COLORMAP_JET);
    cv::Mat invalid_mask;
    cv::bitwise_not(mask_out, invalid_mask);
    color_out.setTo(cv::Scalar(0, 0, 0), invalid_mask);
  }

  // Alpha-blend a jet-colored depth image onto a BGR camera image where the
  // mask is valid. Used by depth_overlay to verify alignment.
  cv::Mat blendJetOnImage(const cv::Mat& jet_color, const cv::Mat& mask,
                          const cv::Mat& rgb) {
    if (rgb.empty()) return cv::Mat();
    if (jet_color.empty() || mask.empty()) return rgb.clone();

    // Vectorized whole-image alpha blend, then keep the original RGB on
    // invalid pixels using the valid mask. This is ~5-10x faster than the
    // per-pixel scan it replaces because cv::addWeighted is SIMD-optimized.
    const double alpha = 0.55;
    cv::Mat blended;
    cv::addWeighted(jet_color, alpha, rgb, 1.0 - alpha, 0.0, blended);
    // Restore original RGB where the depth mask is invalid.
    rgb.copyTo(blended, ~mask);
    return blended;
  }

  // --------------------------------------------------------------------
  // Logging + publish helpers (ROS1/ROS2 thin wrappers)
  // --------------------------------------------------------------------
#if defined(ODIN_ROS2)
  void logInfo (const std::string& s) { RCLCPP_INFO (node_->get_logger(), "[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logWarn (const std::string& s) { RCLCPP_WARN (node_->get_logger(), "[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logError(const std::string& s) { RCLCPP_ERROR(node_->get_logger(), "[%s] %s", info_.sn.c_str(), s.c_str()); }

  template <typename HeaderT>
  void publishImage(const cv::Mat& img, const std::string& encoding,
                    const HeaderT& header,
                    rclcpp::Publisher<ImageMsg>::SharedPtr& pub) {
    if (!pub || img.empty()) return;
    cv_bridge::CvImage out;
    out.header   = header;
    out.encoding = encoding;
    out.image    = img;
    pub->publish(*out.toImageMsg());
  }
#else
  void logInfo (const std::string& s) { ROS_INFO ("[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logWarn (const std::string& s) { ROS_WARN ("[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logError(const std::string& s) { ROS_ERROR("[%s] %s", info_.sn.c_str(), s.c_str()); }

  template <typename HeaderT>
  void publishImage(const cv::Mat& img, const std::string& encoding,
                    const HeaderT& header, ros::Publisher& pub) {
    if (img.empty()) return;
    cv_bridge::CvImage out;
    out.header   = header;
    out.encoding = encoding;
    out.image    = img;
    pub.publish(out.toImageMsg());
  }
#endif

  // --------------------------------------------------------------------
  // Members
  // --------------------------------------------------------------------
#if defined(ODIN_ROS2)
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<ImageMsg>::SharedPtr depth_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr depth_color_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr depth_overlay_pub_;
  rclcpp::Client<odin_ros_driver_rev1::srv::GetCalibration>::SharedPtr calibration_client_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

  message_filters::Subscriber<PointCloud2Msg> cloud_sub_;
  message_filters::Subscriber<ImageMsg> image_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, ImageMsg>;
  using Sync       = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;
#else
  ros::NodeHandle& nh_;
  ros::Publisher depth_pub_;
  ros::Publisher depth_color_pub_;
  ros::Publisher depth_overlay_pub_;

  message_filters::Subscriber<PointCloud2Msg> cloud_sub_;
  message_filters::Subscriber<ImageMsg> image_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, ImageMsg>;
  using Sync       = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;
#endif

  DeviceInfo info_;
  const Pointcloud2DepthConfig& cfg_;

  // Per-device state
  std::atomic<bool> initialized_{false};
  std::atomic<bool> timing_logged_{false};  // first-frame latency log guard
  std::string yaml_content_;

  // Resolution (updated dynamically via resolution_change or YAML defaults)
  int img_width_  = 0;
  int img_height_ = 0;
  int out_width_  = 0;
  int out_height_ = 0;

  // Precomputed projection matrix (plain types): (K / scale) * Tcl
  float Kcl_depth_[16] = {0};
};

// ---------------------------------------------------------------------------
// Pointcloud2DepthNode: manager. Owns global parameters, listens for
// device_online / device_offline / resolution_change, and spawns / destroys
// per-device DeviceWorker instances.
// ---------------------------------------------------------------------------
class Pointcloud2DepthNode {
 public:
#if defined(ODIN_ROS2)
  explicit Pointcloud2DepthNode(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    declareParameters();
    init();
  }
#else
  Pointcloud2DepthNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    init();
  }
#endif

  ~Pointcloud2DepthNode() {
    std::lock_guard<std::mutex> lock(workers_mutex_);
    workers_.clear();
  }

 private:
#if defined(ODIN_ROS2)
  void declareParameters() {
    node_->declare_parameter<std::string>("topic_prefix", "manifold");
    node_->declare_parameter<double>("scale", 7.0);
    node_->declare_parameter<double>("z_min", 0.1);
    node_->declare_parameter<double>("z_max", 50.0);
    node_->declare_parameter<int>("dilate_radius", 1);
    node_->declare_parameter<bool>("use_z_buffer", false);
    node_->declare_parameter<double>("edge_grad_threshold", 0.75);
    node_->declare_parameter<double>("vis_depth_min", 0.1);
    node_->declare_parameter<double>("vis_depth_max", 5.0);
    node_->declare_parameter<std::string>("cloud_topic_suffix", "cloud/raw");
    node_->declare_parameter<std::string>("image_topic_suffix", "camera0/undistort");
    node_->declare_parameter<std::string>("output_topic_suffix", "pointcloud2depth");
    node_->declare_parameter<int>("sync_queue_size", 10);
    node_->declare_parameter<double>("sync_slop_sec", 0.1);
  }
#endif

  void loadParameters() {
#if defined(ODIN_ROS2)
    topic_prefix_base_       = node_->get_parameter("topic_prefix").as_string();
    cfg_.scale               = static_cast<float>(node_->get_parameter("scale").as_double());
    cfg_.z_min               = static_cast<float>(node_->get_parameter("z_min").as_double());
    cfg_.z_max               = static_cast<float>(node_->get_parameter("z_max").as_double());
    cfg_.dilate_radius       = node_->get_parameter("dilate_radius").as_int();
    cfg_.use_z_buffer        = node_->get_parameter("use_z_buffer").as_bool();
    cfg_.edge_grad_threshold = static_cast<float>(node_->get_parameter("edge_grad_threshold").as_double());
    cfg_.vis_depth_min       = static_cast<float>(node_->get_parameter("vis_depth_min").as_double());
    cfg_.vis_depth_max       = static_cast<float>(node_->get_parameter("vis_depth_max").as_double());
    cfg_.cloud_topic_suffix  = node_->get_parameter("cloud_topic_suffix").as_string();
    cfg_.image_topic_suffix  = node_->get_parameter("image_topic_suffix").as_string();
    cfg_.output_topic_suffix = node_->get_parameter("output_topic_suffix").as_string();
    cfg_.sync_queue_size     = node_->get_parameter("sync_queue_size").as_int();
    cfg_.sync_slop_sec       = node_->get_parameter("sync_slop_sec").as_double();
#else
    pnh_.param<std::string>("topic_prefix", topic_prefix_base_, "manifold");
    double scale_d = 7.0, zmin_d = 0.1, zmax_d = 50.0, edge_d = 0.75;
    double vmin_d = 0.1, vmax_d = 5.0;
    pnh_.param<double>("scale", scale_d, 7.0);
    pnh_.param<double>("z_min", zmin_d, 0.1);
    pnh_.param<double>("z_max", zmax_d, 50.0);
    pnh_.param<double>("edge_grad_threshold", edge_d, 0.75);
    pnh_.param<double>("vis_depth_min", vmin_d, 0.1);
    pnh_.param<double>("vis_depth_max", vmax_d, 5.0);
    cfg_.scale               = static_cast<float>(scale_d);
    cfg_.z_min               = static_cast<float>(zmin_d);
    cfg_.z_max               = static_cast<float>(zmax_d);
    cfg_.edge_grad_threshold = static_cast<float>(edge_d);
    cfg_.vis_depth_min       = static_cast<float>(vmin_d);
    cfg_.vis_depth_max       = static_cast<float>(vmax_d);
    pnh_.param<int>("dilate_radius", cfg_.dilate_radius, 1);
    pnh_.param<bool>("use_z_buffer", cfg_.use_z_buffer, false);
    pnh_.param<std::string>("cloud_topic_suffix", cfg_.cloud_topic_suffix, "cloud/raw");
    pnh_.param<std::string>("image_topic_suffix", cfg_.image_topic_suffix, "camera0/undistort");
    pnh_.param<std::string>("output_topic_suffix", cfg_.output_topic_suffix, "pointcloud2depth");
    pnh_.param<int>("sync_queue_size", cfg_.sync_queue_size, 10);
    pnh_.param<double>("sync_slop_sec", cfg_.sync_slop_sec, 0.1);
#endif
  }

  void init() {
    loadParameters();
    startDeviceDiscovery();
    logInfo("Pointcloud2DepthNode initialized. Waiting for devices...");
  }

  // Driver-side management topics use the unified /{prefix}/driver/<suffix> form.
  std::string buildDriverTopicName(const std::string& suffix) const {
    if (topic_prefix_base_.empty()) return "/driver/" + suffix;
    return "/" + topic_prefix_base_ + "/driver/" + suffix;
  }

  void startDeviceDiscovery() {
#if defined(ODIN_ROS2)
    // TRANSIENT_LOCAL on all three subscriptions to mirror the driver-side
    // publishers, so this node correctly picks up latched device_online and
    // resolution_change messages even when it starts AFTER the driver.
    device_online_sub_ = node_->create_subscription<StringMsg>(
        buildDriverTopicName("device_online"),
        rclcpp::QoS(10).transient_local(),
        [this](const StringMsg::SharedPtr msg) { onDeviceOnline(msg->data); });
    device_offline_sub_ = node_->create_subscription<StringMsg>(
        buildDriverTopicName("device_offline"),
        rclcpp::QoS(10).transient_local(),
        [this](const StringMsg::SharedPtr msg) { onDeviceOffline(msg->data); });
    resolution_change_sub_ = node_->create_subscription<StringMsg>(
        buildDriverTopicName("resolution_change"),
        rclcpp::QoS(1).transient_local(),
        [this](const StringMsg::SharedPtr msg) { onResolutionChange(msg->data); });
#else
    device_online_sub_ = nh_.subscribe<StringMsg>(
        buildDriverTopicName("device_online"), 100,
        [this](const boost::shared_ptr<StringMsg const>& msg) { onDeviceOnline(msg->data); });
    device_offline_sub_ = nh_.subscribe<StringMsg>(
        buildDriverTopicName("device_offline"), 10,
        [this](const boost::shared_ptr<StringMsg const>& msg) { onDeviceOffline(msg->data); });
    resolution_change_sub_ = nh_.subscribe<StringMsg>(
        buildDriverTopicName("resolution_change"), 10,
        [this](const boost::shared_ptr<StringMsg const>& msg) { onResolutionChange(msg->data); });
#endif
  }

  // ---- Hotplug callbacks ------------------------------------------------
  void onDeviceOnline(const std::string& json) {
    DeviceInfo info;
    if (!ParseDeviceInfoJson(json, info)) {
      logWarn("Failed to parse device_online: " + json);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(workers_mutex_);
      auto it = workers_.find(info.sn);
      if (it != workers_.end()) {
        // Already known (or pending). Re-announcements are common with
        // TRANSIENT_LOCAL latched topics; treat as no-op.
        return;
      }
      // Reserve the slot with a nullptr so concurrent device_online
      // re-announcements don't race to spawn duplicate workers.
      workers_[info.sn] = nullptr;
    }

    logInfo("New device online: sn=" + info.sn + " prefix=" + info.prefix);

    // Construct the worker in a detached thread: in ROS1 this lets the
    // synchronous get_calibration service call block without freezing the
    // subscriber thread; in ROS2 it just avoids long-running work inside
    // the executor callback.
    std::thread([this, info]() {
#if defined(ODIN_ROS2)
      auto worker = std::make_unique<DeviceWorker>(node_, info, cfg_);
#else
      auto worker = std::make_unique<DeviceWorker>(nh_, info, cfg_);
#endif
      std::lock_guard<std::mutex> lock(workers_mutex_);
      workers_[info.sn] = std::move(worker);
      logInfo("DeviceWorker registered for " + info.sn +
              " (total devices: " + std::to_string(workers_.size()) + ")");
    }).detach();
  }

  void onDeviceOffline(const std::string& sn) {
    std::unique_ptr<DeviceWorker> dying;  // destruct outside the lock
    {
      std::lock_guard<std::mutex> lock(workers_mutex_);
      auto it = workers_.find(sn);
      if (it == workers_.end()) {
        logWarn("device_offline for unknown sn: " + sn);
        return;
      }
      dying = std::move(it->second);
      workers_.erase(it);
    }
    if (dying) {
      logInfo("DeviceWorker removed for " + sn);
    }
    // dying.reset() happens here, outside the lock, to avoid blocking other
    // hotplug callbacks while DDS tears down pubs/subs.
  }

  void onResolutionChange(const std::string& json) {
    std::string sn;
    ParseJsonStringField(json, "sn", sn);
    const int width  = ParseJsonIntField(json, "width");
    const int height = ParseJsonIntField(json, "height");
    if (sn.empty() || width <= 0 || height <= 0) {
      logWarn("Invalid resolution_change JSON: " + json);
      return;
    }

    std::lock_guard<std::mutex> lock(workers_mutex_);
    auto it = workers_.find(sn);
    if (it == workers_.end()) {
      // Resolution change arrived before device_online: ignore. The worker
      // will pick up the right resolution from the synchronized image via
      // the safety net inside processFrame.
      return;
    }
    if (it->second) {
      it->second->onResolutionChange(width, height);
    }
  }

  // ---- Logging (manager-level; workers prefix their own SN) ------------
#if defined(ODIN_ROS2)
  void logInfo (const std::string& s) { RCLCPP_INFO (node_->get_logger(), "%s", s.c_str()); }
  void logWarn (const std::string& s) { RCLCPP_WARN (node_->get_logger(), "%s", s.c_str()); }
  void logError(const std::string& s) { RCLCPP_ERROR(node_->get_logger(), "%s", s.c_str()); }
#else
  void logInfo (const std::string& s) { ROS_INFO ("%s", s.c_str()); }
  void logWarn (const std::string& s) { ROS_WARN ("%s", s.c_str()); }
  void logError(const std::string& s) { ROS_ERROR("%s", s.c_str()); }
#endif

  // ---- Members ----------------------------------------------------------
#if defined(ODIN_ROS2)
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<StringMsg>::SharedPtr device_online_sub_;
  rclcpp::Subscription<StringMsg>::SharedPtr device_offline_sub_;
  rclcpp::Subscription<StringMsg>::SharedPtr resolution_change_sub_;
#else
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber device_online_sub_;
  ros::Subscriber device_offline_sub_;
  ros::Subscriber resolution_change_sub_;
#endif

  Pointcloud2DepthConfig cfg_;
  std::string topic_prefix_base_ = "manifold";

  std::map<std::string, std::unique_ptr<DeviceWorker>> workers_;
  std::mutex workers_mutex_;
};

}  // namespace odin

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
#if defined(ODIN_ROS2)
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pointcloud2depth_node");
  odin::Pointcloud2DepthNode manager(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
#else
  ros::init(argc, argv, "pointcloud2depth_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  odin::Pointcloud2DepthNode manager(nh, pnh);
  ros::spin();
#endif
  return 0;
}
