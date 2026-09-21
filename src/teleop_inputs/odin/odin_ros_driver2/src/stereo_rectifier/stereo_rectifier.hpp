#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <memory>

#if defined(ODIN_ROS2)
#include <sensor_msgs/msg/camera_info.hpp>
#else
#include <sensor_msgs/CameraInfo.h>
#endif

namespace odin {

#if defined(ODIN_ROS2)
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
#else
using CameraInfoMsg = sensor_msgs::CameraInfo;
#endif

struct PolyFisheyeParams {
  int width = 640;
  int height = 544;
  double fx = 200.0;  // A11
  double fy = 200.0;  // A22
  double cx = 320.0;  // u0
  double cy = 272.0;  // v0
  double skew = 0.0;  // A12
  double k2 = 0.0, k3 = 0.0, k4 = 0.0, k5 = 0.0, k6 = 0.0, k7 = 0.0;
};

class StereoRectifier {
 public:
  StereoRectifier() = default;
  ~StereoRectifier() = default;

  bool initFromString(const std::string& yaml_content, int out_width = 960, int out_height = 576,
                      double fov_deg = 90.0);
  bool initFromString(const std::string& yaml_content, int img_width, int img_height,
                      int out_width, int out_height, double fov_deg);

  void rectify(const cv::Mat& left_raw, const cv::Mat& right_raw, cv::Mat& left_rect,
               cv::Mat& right_rect) const;

  void rectifyLeft(const cv::Mat& raw, cv::Mat& rect) const;
  void rectifyRight(const cv::Mat& raw, cv::Mat& rect) const;

  CameraInfoMsg getLeftCameraInfo() const;
  CameraInfoMsg getRightCameraInfo() const;

  double getBaseline() const { return baseline_; }
  double getFocalLength() const { return f_new_; }
  int getOutputWidth() const { return out_width_; }
  int getOutputHeight() const { return out_height_; }
  bool isInitialized() const { return initialized_; }

  void printInfo() const;

 private:
  bool loadYamlFromString(const std::string& yaml_content, int img_width = 0, int img_height = 0);
  void computeRectification();
  void generateRectifyMaps();

  double polyfisheyeR(double theta, const PolyFisheyeParams& params) const;

  bool initialized_ = false;

  // Stereo extrinsics
  Eigen::Matrix4d T_cl_cr_;  // left to right camera transform
  double baseline_ = 0.0;

  // Rectification rotation matrices
  Eigen::Matrix3d R1_;  // left camera rectification
  Eigen::Matrix3d R2_;  // right camera rectification

  // Camera intrinsics
  PolyFisheyeParams cam0_;  // left camera
  PolyFisheyeParams cam1_;  // right camera

  // Output parameters
  int out_width_ = 960;
  int out_height_ = 576;
  double fov_deg_ = 90.0;
  double f_new_ = 480.0;   // new focal length after rectification
  Eigen::Matrix3d K_new_;  // new intrinsic matrix

  // Rectification maps
  cv::Mat map1_l_, map2_l_;
  cv::Mat map1_r_, map2_r_;
};

}  // namespace odin
