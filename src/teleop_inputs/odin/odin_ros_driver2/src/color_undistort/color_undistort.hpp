#pragma once
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
// cv_bridge header compatibility: Ubuntu 24.04 uses .hpp, earlier versions use .h
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/core.hpp>

#include "camera/instrinsic.h"
class colorUndistort {
 public:
  colorUndistort() = default;
  bool initFromString(const std::string& yamlContent, std::string sensor = "cam_0");
  bool initFromString(const std::string& yamlContent, int img_width, int img_height, int cam_id = 0);
  int process(const cv::Mat& input, cv_bridge::CvImage& output);

 private:
  MTSDK::MTCAMERA::intrinsic m_intrinsic;
  cv::Mat m_undistort_map_x;
  cv::Mat m_undistort_map_y;
  cv::Mat m_output_buffer;  // Pre-allocated output buffer
  bool isInit = false;
};