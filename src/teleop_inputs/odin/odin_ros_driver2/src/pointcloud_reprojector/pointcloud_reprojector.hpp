#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

#if defined(ODIN_ROS2)
#include <sensor_msgs/msg/point_cloud2.hpp>
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
#else
#include <sensor_msgs/PointCloud2.h>
using PointCloud2Msg = sensor_msgs::PointCloud2;
#endif

namespace odin {

/**
 * @brief Reprojector mode for point cloud to image reprojection
 * 重投影模式
 */
enum class ReprojectorMode {
  kRawToImage,   // cloud_raw + image -> depth/overlay (static extrinsic)
  kSlamToImage   // cloud_slam + image + odom -> depth/overlay (dynamic pose)
};

/**
 * @brief Point cloud to image reprojector
 * 点云重投影到图像模块
 * 
 * Supports two modes:
 * - Raw mode: cloud/raw (lidar frame) + image -> depth/overlay
 * - Slam mode: cloud/slam (world frame) + image + odom -> depth/overlay
 * 
 * Output:
 * - Depth image (CV_32FC1, meters) for 3DGS depth supervision
 * - Overlay image (CV_8UC3) for calibration verification
 */
class PointCloudReprojector {
 public:
  PointCloudReprojector() = default;

  /**
   * @brief Initialize with camera intrinsics
   * @param fx Focal length x
   * @param fy Focal length y
   * @param cx Principal point x
   * @param cy Principal point y
   * @param width Image width
   * @param height Image height
   */
  bool init(double fx, double fy, double cx, double cy, int width, int height);

  /**
   * @brief Initialize from YAML calibration string
   * @param yaml_content YAML calibration content
   * @param cam_id Camera ID (0 for left camera)
   * @param img_width Image width
   * @param img_height Image height
   */
  bool initFromYaml(const std::string& yaml_content, int cam_id, int img_width, int img_height);

  /**
   * @brief Set static extrinsic (camera to lidar/body, does not change per frame)
   * @param T_cam_lidar 4x4 transformation matrix (row-major)
   *        This is the fixed transform from lidar/body frame to camera frame
   */
  void setStaticExtrinsic(const std::vector<double>& T_cam_lidar);

  /**
   * @brief Set reprojector mode
   */
  void setMode(ReprojectorMode mode) { mode_ = mode; }

  /**
   * @brief Get current mode
   */
  ReprojectorMode getMode() const { return mode_; }

  /**
   * @brief Set depth range for filtering
   */
  void setDepthRange(double min_depth, double max_depth) {
    min_depth_ = min_depth;
    max_depth_ = max_depth;
  }

  /**
   * @brief Set point size for overlay visualization
   */
  void setPointSize(int size) { point_size_ = size; }

  /**
   * @brief Set color mode for overlay
   * @param use_depth_color If true, color by depth; if false, use fixed color
   */
  void setUseDepthColor(bool use_depth_color) { use_depth_color_ = use_depth_color; }

  /**
   * @brief Reproject point cloud to image (for raw mode, uses static extrinsic)
   * @param cloud Input point cloud (in lidar/body frame)
   * @param rgb_image Input RGB image (for overlay output)
   * @param depth_out Output depth image (CV_32FC1, meters)
   * @param overlay_out Output overlay image (CV_8UC3)
   * @return true if successful
   * 
   * Transform: P_cam = T_cam_lidar * P_lidar
   */
  bool reproject(const PointCloud2Msg& cloud, const cv::Mat& rgb_image,
                 cv::Mat& depth_out, cv::Mat& overlay_out);

  /**
   * @brief Reproject with dynamic pose (for SLAM mode, cloud in world frame)
   * @param cloud Input point cloud (in world frame)
   * @param T_body_world Current body pose in world frame (from odom, 4x4 row-major)
   * @param rgb_image Input RGB image
   * @param depth_out Output depth image
   * @param overlay_out Output overlay image
   * @return true if successful
   * 
   * Transform: P_cam = T_cam_lidar * inv(T_body_world) * P_world
   */
  bool reprojectWithPose(const PointCloud2Msg& cloud, 
                         const std::vector<double>& T_body_world,
                         const cv::Mat& rgb_image,
                         cv::Mat& depth_out, cv::Mat& overlay_out);

  // Getters
  bool isInitialized() const { return initialized_; }
  int getWidth() const { return width_; }
  int getHeight() const { return height_; }

  void printInfo() const;

 private:
  // Project a 3D point to 2D image coordinates
  bool projectPoint(float x, float y, float z, float& u, float& v, float& depth) const;

  // Get color for depth visualization (jet colormap)
  cv::Vec3b getDepthColor(float depth) const;

  // Compute combined transform for world frame points
  // T_cam_world = T_cam_lidar * inv(T_body_world)
  Eigen::Matrix4d computeCamWorldTransform(const std::vector<double>& T_body_world) const;

  // Camera intrinsics
  double fx_ = 500.0;
  double fy_ = 500.0;
  double cx_ = 320.0;
  double cy_ = 272.0;
  int width_ = 640;
  int height_ = 544;

  // Depth range
  double min_depth_ = 0.1;   // meters
  double max_depth_ = 50.0;  // meters

  // Visualization settings
  int point_size_ = 2;
  bool use_depth_color_ = true;

  // Configuration
  ReprojectorMode mode_ = ReprojectorMode::kRawToImage;

  bool initialized_ = false;
  bool extrinsic_set_ = false;

  // Static extrinsic: T_cam_lidar (camera <- lidar/body)
  // For raw cloud: P_cam = T_cam_lidar * P_lidar
  Eigen::Matrix4d T_cam_lidar_ = Eigen::Matrix4d::Identity();

  // Current frame transform (updated per frame for SLAM mode)
  // Mutable because reprojectWithPose updates it before calling internal reproject
  mutable Eigen::Matrix4d T_cam_cloud_ = Eigen::Matrix4d::Identity();
};

}  // namespace odin
