
#include "color_undistort.hpp"
#include "polynomial_camera.hpp"
#include "yaml_config_loader.hpp"
#include <iostream>

using yaml_utils::safeGet;

bool colorUndistort::initFromString(const std::string& yamlContent, int img_width, int img_height,
                                    int cam_id) {
  YAML::Node config = YAML::Load(yamlContent);
  std::string resolved = yaml_utils::ResolveCamKey(config, cam_id, img_width, img_height);
  if (resolved.empty()) {
    std::cerr << "[colorUndistort] Error: no calibration for cam_" << cam_id << " at " << img_width
              << "x" << img_height << std::endl;
    return false;
  }
  return initFromString(yamlContent, resolved);
}

bool colorUndistort::initFromString(const std::string& yamlContent, std::string sensor) {
  YAML::Node config = YAML::Load(yamlContent);
  std::string resolved = sensor;
  if (!config[sensor]) {
    // Try wildcard fallback: extract cam_id from sensor (e.g. "cam_0" → 0)
    int cam_id = 0;
    auto pos = sensor.find_last_of('_');
    if (pos != std::string::npos) cam_id = std::atoi(sensor.c_str() + pos + 1);
    resolved = yaml_utils::ResolveCamKey(config, cam_id, 0, 0);
    if (resolved.empty()) {
      std::cerr << "[colorUndistort] Error: sensor '" << sensor << "' not found in yaml"
                << std::endl;
      return false;
    }
  }
  YAML::Node cam_node = config[resolved];
  m_intrinsic.width = safeGet<int>(cam_node, "image_width", 640);
  m_intrinsic.height = safeGet<int>(cam_node, "image_height", 544);
  m_intrinsic.fx = safeGet<float>(cam_node, "A11", 200.0f);
  m_intrinsic.fy = safeGet<float>(cam_node, "A22", 200.0f);
  m_intrinsic.cx = safeGet<float>(cam_node, "u0", 320.0f);
  m_intrinsic.cy = safeGet<float>(cam_node, "v0", 272.0f);
  m_intrinsic.skew = safeGet<float>(cam_node, "A12", 0.0f);
  m_intrinsic.k[2] = safeGet<float>(cam_node, "k2", 0.0f);
  m_intrinsic.k[3] = safeGet<float>(cam_node, "k3", 0.0f);
  m_intrinsic.k[4] = safeGet<float>(cam_node, "k4", 0.0f);
  m_intrinsic.k[5] = safeGet<float>(cam_node, "k5", 0.0f);
  m_intrinsic.k[6] = safeGet<float>(cam_node, "k6", 0.0f);
  m_intrinsic.k[7] = safeGet<float>(cam_node, "k7", 0.0f);
  m_intrinsic.p[1] = safeGet<float>(cam_node, "p1", 0.0f);
  m_intrinsic.p[2] = safeGet<float>(cam_node, "p2", 0.0f);

  cv::Mat map_x_f32(m_intrinsic.height, m_intrinsic.width, CV_32F);
  cv::Mat map_y_f32(m_intrinsic.height, m_intrinsic.width, CV_32F);

  std::unique_ptr<mini_vikit::PolynomialCamera> polynoCam =
      std::make_unique<mini_vikit::PolynomialCamera>(
          m_intrinsic.width, m_intrinsic.height, m_intrinsic.fx, m_intrinsic.fy, m_intrinsic.cx,
          m_intrinsic.cy, m_intrinsic.skew, m_intrinsic.k[2], m_intrinsic.k[3], m_intrinsic.k[4],
          m_intrinsic.k[5], m_intrinsic.k[6], m_intrinsic.k[7]);

  for (int v_out = 0; v_out < m_intrinsic.height; ++v_out) {
    for (int u_out = 0; u_out < m_intrinsic.width; ++u_out) {
      double x_norm = (u_out - polynoCam->cx()) / polynoCam->fx();
      double y_norm = (v_out - polynoCam->cy()) / polynoCam->fy();

      // remove skew
      x_norm = x_norm - y_norm * polynoCam->skew() / polynoCam->fx();

      Eigen::Vector2d uv(x_norm, y_norm);
      Eigen::Vector2d distorted_pixel = polynoCam->world2cam(uv);

      map_x_f32.at<float>(v_out, u_out) = static_cast<float>(distorted_pixel[0]);
      map_y_f32.at<float>(v_out, u_out) = static_cast<float>(distorted_pixel[1]);
    }
  }

  // Convert to fixed-point format for faster remap
  // CV_16SC2 stores integer coords, nninterpolation=true for INTER_NEAREST
  cv::convertMaps(map_x_f32, map_y_f32, m_undistort_map_x, m_undistort_map_y, CV_16SC2, true);

  // Pre-allocate output buffer
  m_output_buffer.create(m_intrinsic.height, m_intrinsic.width, CV_8UC3);

  isInit = true;
  return true;
}

int colorUndistort::process(const cv::Mat& input, cv_bridge::CvImage& output) {
  if (!isInit) {
    return -1;
  }

  // Use pre-allocated buffer, INTER_NEAREST for faster processing
  cv::remap(input, m_output_buffer, m_undistort_map_x, m_undistort_map_y, cv::INTER_NEAREST,
            cv::BORDER_CONSTANT);
  output.encoding = "bgr8";
  output.image = m_output_buffer;  // No clone - caller must not modify
  return 0;
}
