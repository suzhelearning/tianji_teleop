#pragma once

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class rawCloudRender {
 public:
  rawCloudRender() = default;

  bool initFromString(const std::string& yamlContent);
  bool initFromString(const std::string& yamlContent, int img_width, int img_height);
  void render(const cv::Mat& image_bgr, const pcl::PointCloud<pcl::PointXYZ>& cloud_xyz,
              pcl::PointCloud<pcl::PointXYZRGBA>& colored_cloud);

  void print_camera_calib();

 private:
  bool initFromStringWithKey(const std::string& yamlContent, const std::string& cam_key);

  std::string model_type_;
  std::string camera_name_;
  int image_width_;
  int image_height_;
  int frame_size_;
  bool opencv_available_;

  // 4x4 transformation matrix (T_camera_lidar)
  Eigen::Matrix4f T_camera_lidar_;

  float k2_;
  float k3_;
  float k4_;
  float k5_;
  float k6_;
  float k7_;
  float p1_;
  float p2_;
  float A11_fx_;
  float A12_skew_;
  float A22_fy_;
  float u0_cx_;
  float v0_cy_;
  bool isFast_;
  int numDiff_;
  float maxIncidentAngle_;
};
