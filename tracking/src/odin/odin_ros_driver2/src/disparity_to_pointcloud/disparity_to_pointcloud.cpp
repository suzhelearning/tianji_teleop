#include "disparity_to_pointcloud.hpp"
#include <iostream>
#include <cmath>

#if defined(ODIN_ROS2)
#include <sensor_msgs/point_cloud2_iterator.hpp>
#else
#include <sensor_msgs/point_cloud2_iterator.h>
#endif

namespace odin {

void DisparityToPointCloud::setExtrinsic(const std::vector<double>& Tcl_0) {
  if (Tcl_0.size() < 16) {
    std::cerr << "[DisparityToPointCloud] Invalid Tcl_0 size: " << Tcl_0.size() << std::endl;
    return;
  }

  // Tcl_0 is T_lidar_to_cam (4x4 row-major)
  // We need T_cam_to_lidar = Tcl_0^(-1)
  // For a rigid transform: T^(-1) = [R^T, -R^T*t; 0, 1]

  // Extract R (rotation matrix) from Tcl_0
  // Tcl_0 layout: [R11 R12 R13 tx; R21 R22 R23 ty; R31 R32 R33 tz; 0 0 0 1]
  double R00 = Tcl_0[0], R01 = Tcl_0[1], R02 = Tcl_0[2];
  double R10 = Tcl_0[4], R11 = Tcl_0[5], R12 = Tcl_0[6];
  double R20 = Tcl_0[8], R21 = Tcl_0[9], R22 = Tcl_0[10];
  double t0 = Tcl_0[3], t1 = Tcl_0[7], t2 = Tcl_0[11];

  // Compute R^T (transpose)
  r00_ = R00;
  r01_ = R10;
  r02_ = R20;
  r10_ = R01;
  r11_ = R11;
  r12_ = R21;
  r20_ = R02;
  r21_ = R12;
  r22_ = R22;

  // Compute -R^T * t
  tx_ = -(r00_ * t0 + r01_ * t1 + r02_ * t2);
  ty_ = -(r10_ * t0 + r11_ * t1 + r12_ * t2);
  tz_ = -(r20_ * t0 + r21_ * t1 + r22_ * t2);

  extrinsic_set_ = true;

  std::cout << "[DisparityToPointCloud] Extrinsic set (cam->lidar):" << std::endl;
  std::cout << "  R^T = [" << r00_ << ", " << r01_ << ", " << r02_ << "]" << std::endl;
  std::cout << "        [" << r10_ << ", " << r11_ << ", " << r12_ << "]" << std::endl;
  std::cout << "        [" << r20_ << ", " << r21_ << ", " << r22_ << "]" << std::endl;
  std::cout << "  t   = [" << tx_ << ", " << ty_ << ", " << tz_ << "]" << std::endl;
}

void DisparityToPointCloud::buildLUT(int w, int h, float f, float cx, float cy) {
  if (w == lut_width_ && h == lut_height_) {
    return;  // LUT already built for this size
  }

  u_lut_.resize(w);
  v_lut_.resize(h);

  for (int u = 0; u < w; ++u) {
    u_lut_[u] = (u - cx) / f;
  }
  for (int v = 0; v < h; ++v) {
    v_lut_[v] = (v - cy) / f;
  }

  lut_width_ = w;
  lut_height_ = h;
}

bool DisparityToPointCloud::init(double focal, double baseline, double cx, double cy, int width,
                                 int height) {
  focal_ = focal;
  baseline_ = baseline;
  cx_ = cx;
  cy_ = cy;
  width_ = width;
  height_ = height;
  initialized_ = true;

  // Pre-build LUT
  fb_ = static_cast<float>(focal * baseline);
  buildLUT(width, height, static_cast<float>(focal), static_cast<float>(cx),
           static_cast<float>(cy));

  printInfo();
  return true;
}

void DisparityToPointCloud::setParameters(double focal, double baseline, int out_width,
                                          int out_height) {
  focal_ = focal;
  baseline_ = baseline;
  cx_ = out_width / 2.0;
  cy_ = out_height / 2.0;
  width_ = out_width;
  height_ = out_height;
  initialized_ = true;

  // Pre-build LUT
  fb_ = static_cast<float>(focal * baseline);
  buildLUT(out_width, out_height, static_cast<float>(focal), static_cast<float>(cx_),
           static_cast<float>(cy_));
}

bool DisparityToPointCloud::convert(const cv::Mat& disparity, const cv::Mat& color_image,
                                    PointCloud2Msg& cloud_out, const std::string& frame_id) {
  if (!initialized_) {
    std::cerr << "[DisparityToPointCloud] Not initialized" << std::endl;
    return false;
  }

  if (disparity.empty()) {
    std::cerr << "[DisparityToPointCloud] Empty disparity image" << std::endl;
    return false;
  }

  int h = disparity.rows;
  int w = disparity.cols;

  // Scale parameters if disparity size differs from expected
  double scale_x = static_cast<double>(w) / width_;
  double scale_y = static_cast<double>(h) / height_;
  float f_use = static_cast<float>(focal_ * scale_x);
  float cx_use = static_cast<float>(cx_ * scale_x);
  float cy_use = static_cast<float>(cy_ * scale_y);

  // Rebuild LUT if size changed
  float fb_use = f_use * static_cast<float>(baseline_);
  buildLUT(w, h, f_use, cx_use, cy_use);

  // Count valid points first (using pointer access for speed)
  int valid_count = 0;
  const float min_disp = static_cast<float>(min_disparity_);
  const float min_z = static_cast<float>(min_depth_);
  const float max_z = static_cast<float>(max_depth_);

  for (int v = 0; v < h; ++v) {
    const float* disp_row = disparity.ptr<float>(v);
    for (int u = 0; u < w; ++u) {
      float d = disp_row[u];
      if (d > min_disp) {
        float Z = fb_use / d;
        if (Z > min_z && Z < max_z) {
          ++valid_count;
        }
      }
    }
  }

  if (valid_count == 0) {
    std::cerr << "[DisparityToPointCloud] No valid points" << std::endl;
    return false;
  }

  // Prepare color image if available
  cv::Mat color_scaled;
  bool has_color = !color_image.empty();
  if (has_color && (color_image.rows != h || color_image.cols != w)) {
    cv::resize(color_image, color_scaled, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
  } else if (has_color) {
    color_scaled = color_image;
  }

  // Setup PointCloud2 message
  // Use configured frame_id if transform is enabled, otherwise use provided frame_id
  std::string actual_frame_id =
      (transform_to_lidar_ && !output_frame_id_.empty()) ? output_frame_id_ : frame_id;
  cloud_out.header.frame_id = actual_frame_id;
  cloud_out.height = 1;
  cloud_out.width = valid_count;
  cloud_out.is_dense = true;
  cloud_out.is_bigendian = false;

  // Define fields: x, y, z, rgb
#if defined(ODIN_ROS2)
  sensor_msgs::PointCloud2Modifier modifier(cloud_out);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(valid_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_out, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_out, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud_out, "rgb");
#else
  sensor_msgs::PointCloud2Modifier modifier(cloud_out);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(valid_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_out, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_out, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud_out, "rgb");
#endif

  // Fill point cloud (using LUT + pointer access for speed)
  for (int v = 0; v < h; ++v) {
    const float* disp_row = disparity.ptr<float>(v);
    const float v_factor = v_lut_[v];

    for (int u = 0; u < w; ++u) {
      float d = disp_row[u];
      if (d <= min_disp) continue;

      float Z = fb_use / d;
      if (Z <= min_z || Z >= max_z) continue;

      // Use LUT for X, Y calculation (much faster)
      float X_cam = u_lut_[u] * Z;
      float Y_cam = v_factor * Z;
      float Z_cam = Z;

      // Apply coordinate transformation if enabled
      if (transform_to_lidar_ && extrinsic_set_) {
        // P_lidar = R^T * P_cam + t_cam_to_lidar
        *iter_x = static_cast<float>(r00_ * X_cam + r01_ * Y_cam + r02_ * Z_cam + tx_);
        *iter_y = static_cast<float>(r10_ * X_cam + r11_ * Y_cam + r12_ * Z_cam + ty_);
        *iter_z = static_cast<float>(r20_ * X_cam + r21_ * Y_cam + r22_ * Z_cam + tz_);
      } else {
        *iter_x = X_cam;
        *iter_y = Y_cam;
        *iter_z = Z_cam;
      }

      // Set color
      if (has_color) {
        cv::Vec3b bgr = color_scaled.at<cv::Vec3b>(v, u);
        iter_rgb[0] = bgr[2];  // R
        iter_rgb[1] = bgr[1];  // G
        iter_rgb[2] = bgr[0];  // B
        iter_rgb[3] = 255;     // A
      } else {
        // Depth-based coloring (red=near, blue=far)
        float depth_norm = static_cast<float>((Z - min_depth_) / (max_depth_ - min_depth_));
        depth_norm = std::max(0.0f, std::min(1.0f, depth_norm));
        iter_rgb[0] = static_cast<uint8_t>(255 * (1.0f - depth_norm));  // R
        iter_rgb[1] = 0;                                                // G
        iter_rgb[2] = static_cast<uint8_t>(255 * depth_norm);           // B
        iter_rgb[3] = 255;                                              // A
      }

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_rgb;
    }
  }

  return true;
}

bool DisparityToPointCloud::convertFromMsg(const DisparityImageMsg& disp_msg,
                                           const cv::Mat& color_image, PointCloud2Msg& cloud_out) {
  // Extract disparity from message
  cv::Mat disparity;

#if defined(ODIN_ROS2)
  int h = disp_msg.image.height;
  int w = disp_msg.image.width;
  disparity = cv::Mat(h, w, CV_32FC1, const_cast<uint8_t*>(disp_msg.image.data.data())).clone();

  // Use parameters from message if available
  if (disp_msg.f > 0 && disp_msg.t != 0) {
    focal_ = disp_msg.f;
    baseline_ = std::abs(disp_msg.t);
    cx_ = w / 2.0;
    cy_ = h / 2.0;
    width_ = w;
    height_ = h;
  }

  std::string frame_id = disp_msg.header.frame_id;
#else
  int h = disp_msg.image.height;
  int w = disp_msg.image.width;
  disparity = cv::Mat(h, w, CV_32FC1, const_cast<uint8_t*>(disp_msg.image.data.data())).clone();

  if (disp_msg.f > 0 && disp_msg.T != 0) {
    focal_ = disp_msg.f;
    baseline_ = std::abs(disp_msg.T);
    cx_ = w / 2.0;
    cy_ = h / 2.0;
    width_ = w;
    height_ = h;
  }

  std::string frame_id = disp_msg.header.frame_id;
#endif

  if (frame_id.empty()) {
    frame_id = "left_camera";
  }

  return convert(disparity, color_image, cloud_out, frame_id);
}

void DisparityToPointCloud::printInfo() const {
  std::cout << "========== DisparityToPointCloud Info ==========" << std::endl;
  std::cout << "  Focal length: " << focal_ << " px" << std::endl;
  std::cout << "  Baseline:     " << baseline_ * 1000.0 << " mm" << std::endl;
  std::cout << "  Principal pt: (" << cx_ << ", " << cy_ << ")" << std::endl;
  std::cout << "  Image size:   " << width_ << " x " << height_ << std::endl;
  std::cout << "  Depth range:  " << min_depth_ << " - " << max_depth_ << " m" << std::endl;
  std::cout << "  Transform:    " << (transform_to_lidar_ ? "camera->lidar" : "none") << std::endl;
  std::cout << "  Output frame: " << output_frame_id_ << std::endl;
  std::cout << "================================================" << std::endl;
}

}  // namespace odin
