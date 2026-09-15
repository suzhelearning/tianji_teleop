#include "pointcloud_reprojector.hpp"
#include "yaml_config_loader.hpp"
#include <iostream>
#include <cmath>

#if defined(ODIN_ROS2)
#include <sensor_msgs/point_cloud2_iterator.hpp>
#else
#include <sensor_msgs/point_cloud2_iterator.h>
#endif

namespace odin {

bool PointCloudReprojector::init(double fx, double fy, double cx, double cy, int width, int height) {
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  width_ = width;
  height_ = height;
  initialized_ = true;

  printInfo();
  return true;
}

bool PointCloudReprojector::initFromYaml(const std::string& yaml_content, int cam_id,
                                          int img_width, int img_height) {
  try {
    YAML::Node config = YAML::Load(yaml_content);
    std::string resolved = yaml_utils::ResolveCamKey(config, cam_id, img_width, img_height);
    if (resolved.empty()) {
      std::cerr << "[PointCloudReprojector] Error: no calibration for cam_" << cam_id 
                << " at " << img_width << "x" << img_height << std::endl;
      return false;
    }

    YAML::Node cam_node = config[resolved];
    fx_ = yaml_utils::safeGet<double>(cam_node, "A11", 500.0);
    fy_ = yaml_utils::safeGet<double>(cam_node, "A22", 500.0);
    cx_ = yaml_utils::safeGet<double>(cam_node, "u0", img_width / 2.0);
    cy_ = yaml_utils::safeGet<double>(cam_node, "v0", img_height / 2.0);
    width_ = img_width;
    height_ = img_height;
    initialized_ = true;

    printInfo();
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[PointCloudReprojector] YAML parse error: " << e.what() << std::endl;
    return false;
  }
}

void PointCloudReprojector::setStaticExtrinsic(const std::vector<double>& T_cam_lidar) {
  if (T_cam_lidar.size() < 16) {
    std::cerr << "[PointCloudReprojector] Invalid extrinsic size: " << T_cam_lidar.size() << std::endl;
    return;
  }

  // T_cam_lidar is 4x4 row-major: [R | t; 0 0 0 1]
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      T_cam_lidar_(i, j) = T_cam_lidar[i * 4 + j];
    }
  }

  // Also set as current transform (for raw cloud case)
  T_cam_cloud_ = T_cam_lidar_;
  extrinsic_set_ = true;

  std::cout << "[PointCloudReprojector] Static extrinsic set (lidar->cam):" << std::endl;
  std::cout << T_cam_lidar_ << std::endl;
}

Eigen::Matrix4d PointCloudReprojector::computeCamWorldTransform(
    const std::vector<double>& T_body_world) const {
  // T_body_world: body pose in world frame (from odom)
  // We need: T_cam_world = T_cam_lidar * inv(T_body_world)
  // Because: P_cam = T_cam_lidar * P_body = T_cam_lidar * inv(T_body_world) * P_world
  
  Eigen::Matrix4d T_bw;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      T_bw(i, j) = T_body_world[i * 4 + j];
    }
  }

  // inv(T_body_world) = T_world_body
  Eigen::Matrix4d T_world_body = T_bw.inverse();

  // T_cam_world = T_cam_lidar * T_world_body
  return T_cam_lidar_ * T_world_body;
}

bool PointCloudReprojector::projectPoint(float x, float y, float z, 
                                          float& u, float& v, float& depth) const {
  // Transform point to camera frame using current transform
  float x_cam, y_cam, z_cam;
  if (extrinsic_set_) {
    // P_cam = T_cam_cloud * P_cloud
    x_cam = static_cast<float>(T_cam_cloud_(0,0) * x + T_cam_cloud_(0,1) * y + T_cam_cloud_(0,2) * z + T_cam_cloud_(0,3));
    y_cam = static_cast<float>(T_cam_cloud_(1,0) * x + T_cam_cloud_(1,1) * y + T_cam_cloud_(1,2) * z + T_cam_cloud_(1,3));
    z_cam = static_cast<float>(T_cam_cloud_(2,0) * x + T_cam_cloud_(2,1) * y + T_cam_cloud_(2,2) * z + T_cam_cloud_(2,3));
  } else {
    // Assume point cloud is already in camera frame
    x_cam = x;
    y_cam = y;
    z_cam = z;
  }

  // Check if point is in front of camera
  if (z_cam <= 0) {
    return false;
  }

  // Check depth range
  if (z_cam < min_depth_ || z_cam > max_depth_) {
    return false;
  }

  // Project to image plane: u = fx * X/Z + cx, v = fy * Y/Z + cy
  u = static_cast<float>(fx_ * x_cam / z_cam + cx_);
  v = static_cast<float>(fy_ * y_cam / z_cam + cy_);
  depth = z_cam;

  // Check if within image bounds
  if (u < 0 || u >= width_ || v < 0 || v >= height_) {
    return false;
  }

  return true;
}

cv::Vec3b PointCloudReprojector::getDepthColor(float depth) const {
  // Normalize depth to [0, 1]
  float depth_norm = static_cast<float>((depth - min_depth_) / (max_depth_ - min_depth_));
  depth_norm = std::max(0.0f, std::min(1.0f, depth_norm));

  // Jet colormap: blue (far) -> green -> red (near)
  cv::Vec3b color;
  if (depth_norm < 0.25f) {
    // Blue to cyan
    float t = depth_norm / 0.25f;
    color[0] = 255;                          // B
    color[1] = static_cast<uint8_t>(255 * t); // G
    color[2] = 0;                             // R
  } else if (depth_norm < 0.5f) {
    // Cyan to green
    float t = (depth_norm - 0.25f) / 0.25f;
    color[0] = static_cast<uint8_t>(255 * (1 - t)); // B
    color[1] = 255;                                  // G
    color[2] = 0;                                    // R
  } else if (depth_norm < 0.75f) {
    // Green to yellow
    float t = (depth_norm - 0.5f) / 0.25f;
    color[0] = 0;                             // B
    color[1] = 255;                           // G
    color[2] = static_cast<uint8_t>(255 * t); // R
  } else {
    // Yellow to red
    float t = (depth_norm - 0.75f) / 0.25f;
    color[0] = 0;                                    // B
    color[1] = static_cast<uint8_t>(255 * (1 - t)); // G
    color[2] = 255;                                  // R
  }

  return color;
}

bool PointCloudReprojector::reproject(const PointCloud2Msg& cloud, const cv::Mat& rgb_image,
                                       cv::Mat& depth_out, cv::Mat& overlay_out) {
  if (!initialized_) {
    std::cerr << "[PointCloudReprojector] Not initialized" << std::endl;
    return false;
  }

  // Initialize output images
  depth_out = cv::Mat::zeros(height_, width_, CV_32FC1);

  // Always use white background, only color the projected point cloud positions
  overlay_out = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255, 255, 255));

  // Keep a reference to RGB for color sampling (resize if needed)
  cv::Mat rgb_resized;
  if (!rgb_image.empty()) {
    if (rgb_image.rows != height_ || rgb_image.cols != width_) {
      cv::resize(rgb_image, rgb_resized, cv::Size(width_, height_));
    } else {
      rgb_resized = rgb_image;
    }
  }

  // Find field offsets in point cloud
  int x_offset = -1, y_offset = -1, z_offset = -1;

  for (const auto& field : cloud.fields) {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    std::cerr << "[PointCloudReprojector] Point cloud missing xyz fields" << std::endl;
    return false;
  }

  // Iterate through points
  const uint8_t* data_ptr = cloud.data.data();
  size_t point_step = cloud.point_step;
  size_t num_points = cloud.width * cloud.height;

  int projected_count = 0;

  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t* point_ptr = data_ptr + i * point_step;

    float x = *reinterpret_cast<const float*>(point_ptr + x_offset);
    float y = *reinterpret_cast<const float*>(point_ptr + y_offset);
    float z = *reinterpret_cast<const float*>(point_ptr + z_offset);

    // Skip invalid points
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (x == 0 && y == 0 && z == 0) {
      continue;
    }

    float u, v, depth;
    if (!projectPoint(x, y, z, u, v, depth)) {
      continue;
    }

    int ui = static_cast<int>(u);
    int vi = static_cast<int>(v);

    // Update depth image (keep closest point)
    float& current_depth = depth_out.at<float>(vi, ui);
    if (current_depth == 0 || depth < current_depth) {
      current_depth = depth;
    }

    // Update overlay image - sample RGB color from image at projected point
    // Black background with RGB-colored point cloud projection
    cv::Vec3b color;
    if (!rgb_resized.empty()) {
      color = rgb_resized.at<cv::Vec3b>(vi, ui);
    } else {
      color = cv::Vec3b(255, 255, 255);  // White fallback
    }

    if (point_size_ <= 1) {
      overlay_out.at<cv::Vec3b>(vi, ui) = color;
    } else {
      cv::circle(overlay_out, cv::Point(ui, vi), point_size_, 
                 cv::Scalar(color[0], color[1], color[2]), -1);
    }

    ++projected_count;
  }


  return projected_count > 0;
}

bool PointCloudReprojector::reprojectWithPose(const PointCloud2Msg& cloud,
                                               const std::vector<double>& T_body_world,
                                               const cv::Mat& rgb_image,
                                               cv::Mat& depth_out, cv::Mat& overlay_out) {
  if (!initialized_) {
    std::cerr << "[PointCloudReprojector] Not initialized" << std::endl;
    return false;
  }

  if (!extrinsic_set_) {
    std::cerr << "[PointCloudReprojector] Static extrinsic not set, call setStaticExtrinsic first" << std::endl;
    return false;
  }

  if (T_body_world.size() < 16) {
    std::cerr << "[PointCloudReprojector] Invalid pose size: " << T_body_world.size() << std::endl;
    return false;
  }

  // Compute T_cam_world for this frame
  // T_cam_cloud_ will be used by projectPoint()
  T_cam_cloud_ = computeCamWorldTransform(T_body_world);

  // Now call reproject which uses T_cam_cloud_
  return reproject(cloud, rgb_image, depth_out, overlay_out);
}

void PointCloudReprojector::printInfo() const {
  std::cout << "========== PointCloudReprojector Info ==========" << std::endl;
  std::cout << "  Camera intrinsics:" << std::endl;
  std::cout << "    fx = " << fx_ << ", fy = " << fy_ << std::endl;
  std::cout << "    cx = " << cx_ << ", cy = " << cy_ << std::endl;
  std::cout << "    size = " << width_ << " x " << height_ << std::endl;
  std::cout << "  Depth range: " << min_depth_ << " - " << max_depth_ << " m" << std::endl;
  std::cout << "  Mode: " << (mode_ == ReprojectorMode::kRawToImage ? "raw" : "slam") << std::endl;
  std::cout << "  Extrinsic set: " << (extrinsic_set_ ? "yes" : "no") << std::endl;
  std::cout << "================================================" << std::endl;
}

}  // namespace odin
