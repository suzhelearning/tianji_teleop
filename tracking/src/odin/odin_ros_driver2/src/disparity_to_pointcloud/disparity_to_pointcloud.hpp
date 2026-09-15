#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <string>

#if defined(ODIN_ROS2)
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using ImageMsg = sensor_msgs::msg::Image;
using DisparityImageMsg = stereo_msgs::msg::DisparityImage;
#else
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
using PointCloud2Msg = sensor_msgs::PointCloud2;
using ImageMsg = sensor_msgs::Image;
using DisparityImageMsg = stereo_msgs::DisparityImage;
#endif

namespace odin {

class DisparityToPointCloud {
 public:
  DisparityToPointCloud() = default;

  // Initialize with stereo parameters
  bool init(double focal, double baseline, double cx, double cy, int width, int height);

  // Set extrinsic transformation (camera to lidar)
  void setExtrinsic(const std::vector<double>& Tcl_0);

  // Enable/disable coordinate transformation
  void setTransformToLidar(bool enable) { transform_to_lidar_ = enable; }
  void setOutputFrameId(const std::string& frame_id) { output_frame_id_ = frame_id; }

  // Set parameters from StereoRectifier output
  void setParameters(double focal, double baseline, int out_width, int out_height);

  // Convert disparity image to point cloud
  // color_image is optional (for coloring points)
  bool convert(const cv::Mat& disparity, const cv::Mat& color_image, PointCloud2Msg& cloud_out,
               const std::string& frame_id = "left_camera");

  // Convert from DisparityImage message
  bool convertFromMsg(const DisparityImageMsg& disp_msg, const cv::Mat& color_image,
                      PointCloud2Msg& cloud_out);

  // Getters
  double getFocal() const { return focal_; }
  double getBaseline() const { return baseline_; }
  bool isInitialized() const { return initialized_; }

  void printInfo() const;

 private:
  // Build lookup tables for fast conversion
  void buildLUT(int w, int h, float f, float cx, float cy);

  double focal_ = 480.0;
  double baseline_ = 0.0512;
  double cx_ = 480.0;
  double cy_ = 288.0;
  int width_ = 960;
  int height_ = 576;

  double min_depth_ = 0.1;   // meters
  double max_depth_ = 50.0;  // meters
  double min_disparity_ = 0.1;

  bool initialized_ = false;

  // LUT for fast coordinate computation
  std::vector<float> u_lut_;  // (u - cx) / f for each column
  std::vector<float> v_lut_;  // (v - cy) / f for each row
  float fb_ = 0.0f;           // focal * baseline (cached)
  int lut_width_ = 0;
  int lut_height_ = 0;

  // Extrinsic transformation (camera to lidar)
  bool transform_to_lidar_ = false;
  bool extrinsic_set_ = false;
  std::string output_frame_id_ = "left_camera";

  // Pre-computed inverse transformation matrix elements: T_cam_to_lidar = Tcl_0^(-1)
  // R^T elements (row-major)
  double r00_ = 1.0, r01_ = 0.0, r02_ = 0.0;
  double r10_ = 0.0, r11_ = 1.0, r12_ = 0.0;
  double r20_ = 0.0, r21_ = 0.0, r22_ = 1.0;
  // -R^T * t elements
  double tx_ = 0.0, ty_ = 0.0, tz_ = 0.0;
};

}  // namespace odin
