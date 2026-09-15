#include "stereo_rectifier.hpp"
#include "yaml_config_loader.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>

namespace odin {

using yaml_utils::safeGet;

// Helper to load camera params with defaults
static void loadCamParams(const YAML::Node& cam_node, PolyFisheyeParams& params) {
  params.width = safeGet<int>(cam_node, "image_width", 640);
  params.height = safeGet<int>(cam_node, "image_height", 544);
  params.fx = safeGet<double>(cam_node, "A11", 200.0);
  params.fy = safeGet<double>(cam_node, "A22", 200.0);
  params.cx = safeGet<double>(cam_node, "u0", 320.0);
  params.cy = safeGet<double>(cam_node, "v0", 272.0);
  params.skew = safeGet<double>(cam_node, "A12", 0.0);
  params.k2 = safeGet<double>(cam_node, "k2", 0.0);
  params.k3 = safeGet<double>(cam_node, "k3", 0.0);
  params.k4 = safeGet<double>(cam_node, "k4", 0.0);
  params.k5 = safeGet<double>(cam_node, "k5", 0.0);
  params.k6 = safeGet<double>(cam_node, "k6", 0.0);
  params.k7 = safeGet<double>(cam_node, "k7", 0.0);
}

bool StereoRectifier::initFromString(const std::string& yaml_content, int img_width, int img_height,
                                     int out_width, int out_height, double fov_deg) {
  out_width_ = out_width;
  out_height_ = out_height;
  fov_deg_ = fov_deg;

  if (!loadYamlFromString(yaml_content, img_width, img_height)) {
    std::cerr << "[StereoRectifier] Failed to parse yaml content" << std::endl;
    return false;
  }

  computeRectification();
  generateRectifyMaps();

  initialized_ = true;
  std::cout << "[StereoRectifier] Initialized successfully from string" << std::endl;
  printInfo();

  return true;
}

bool StereoRectifier::initFromString(const std::string& yaml_content, int out_width, int out_height,
                                     double fov_deg) {
  out_width_ = out_width;
  out_height_ = out_height;
  fov_deg_ = fov_deg;

  if (!loadYamlFromString(yaml_content, 0, 0)) {
    std::cerr << "[StereoRectifier] Failed to parse yaml content" << std::endl;
    return false;
  }

  computeRectification();
  generateRectifyMaps();

  initialized_ = true;
  std::cout << "[StereoRectifier] Initialized successfully from string" << std::endl;
  printInfo();

  return true;
}

bool StereoRectifier::loadYamlFromString(const std::string& yaml_content, int img_width,
                                         int img_height) {
  YAML::Node config = YAML::Load(yaml_content);

  // Load T_cl_cr (left to right camera transform)
  if (!config["T_cl_cr"]) {
    std::cerr << "[StereoRectifier] Error: Missing T_cl_cr in yaml" << std::endl;
    return false;
  }

  std::vector<double> T_vec = config["T_cl_cr"].as<std::vector<double>>();
  if (T_vec.size() != 16) {
    std::cerr << "[StereoRectifier] Error: T_cl_cr should have 16 elements" << std::endl;
    return false;
  }

  T_cl_cr_ = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(T_vec.data());

  // Resolve camera keys by resolution (fallback to cam_0/cam_1 for old format)
  std::string cam0_key = yaml_utils::ResolveCamKey(config, 0, img_width, img_height);
  std::string cam1_key = yaml_utils::ResolveCamKey(config, 1, img_width, img_height);

  // Load cam_0 (left camera)
  if (cam0_key.empty()) {
    std::cerr << "[StereoRectifier] Error: Missing cam_0 in yaml" << std::endl;
    return false;
  }
  loadCamParams(config[cam0_key], cam0_);

  // Load cam_1 (right camera)
  if (cam1_key.empty()) {
    std::cerr << "[StereoRectifier] Error: Missing cam_1 in yaml, stereo disabled" << std::endl;
    return false;
  }
  loadCamParams(config[cam1_key], cam1_);

  return true;
}

void StereoRectifier::computeRectification() {
  // Extract rotation and translation from T_cl_cr
  Eigen::Matrix3d R = T_cl_cr_.block<3, 3>(0, 0);
  Eigen::Vector3d T = T_cl_cr_.block<3, 1>(0, 3);

  // Baseline is the norm of translation vector
  baseline_ = T.norm();

  // Build rectification coordinate system
  // e1: along baseline direction (x-axis)
  Eigen::Vector3d e1 = T / baseline_;

  // e2: perpendicular to e1 and original z-axis
  Eigen::Vector3d e2 = Eigen::Vector3d(0, 0, 1).cross(e1);
  double e2_norm = e2.norm();
  if (e2_norm < 1e-6) {
    // If e1 is parallel to z, use y-axis instead
    e2 = Eigen::Vector3d(0, 1, 0);
  } else {
    e2 = e2 / e2_norm;
  }

  // e3: perpendicular to e1 and e2
  Eigen::Vector3d e3 = e1.cross(e2);

  // Rectification rotation matrix (world to rectified)
  Eigen::Matrix3d R_rect;
  R_rect.row(0) = e1.transpose();
  R_rect.row(1) = e2.transpose();
  R_rect.row(2) = e3.transpose();

  // Left camera rectification rotation
  R1_ = R_rect;

  // Right camera rectification rotation
  // Right camera needs to first rotate to left camera frame (R.T), then to rectified frame
  R2_ = R_rect * R.transpose();

  // Compute new intrinsic matrix
  double fov_rad = fov_deg_ * M_PI / 180.0;
  f_new_ = 0.5 * out_width_ / std::tan(fov_rad / 2.0);

  K_new_ << f_new_, 0, out_width_ / 2.0, 0, f_new_, out_height_ / 2.0, 0, 0, 1;
}

double StereoRectifier::polyfisheyeR(double theta, const PolyFisheyeParams& params) const {
  double theta2 = theta * theta;
  double theta3 = theta2 * theta;
  double theta4 = theta3 * theta;
  double theta5 = theta4 * theta;
  double theta6 = theta5 * theta;
  double theta7 = theta6 * theta;

  return theta + params.k2 * theta2 + params.k3 * theta3 + params.k4 * theta4 + params.k5 * theta5 +
         params.k6 * theta6 + params.k7 * theta7;
}

void StereoRectifier::generateRectifyMaps() {
  // Allocate maps
  map1_l_.create(out_height_, out_width_, CV_32F);
  map2_l_.create(out_height_, out_width_, CV_32F);
  map1_r_.create(out_height_, out_width_, CV_32F);
  map2_r_.create(out_height_, out_width_, CV_32F);

  Eigen::Matrix3d K_new_inv = K_new_.inverse();
  Eigen::Matrix3d R1_inv = R1_.transpose();
  Eigen::Matrix3d R2_inv = R2_.transpose();

  double max_angle_rad = 120.0 * M_PI / 180.0;

  // Generate left camera map
  for (int v = 0; v < out_height_; ++v) {
    for (int u = 0; u < out_width_; ++u) {
      // Step 1: output pixel -> rectified normalized coordinates
      Eigen::Vector3d p_rect = K_new_inv * Eigen::Vector3d(u, v, 1);

      // Step 2: rectified coordinates -> original camera coordinates
      Eigen::Vector3d p_cam = R1_inv * p_rect;

      // Step 3: compute incident angle and azimuth
      double X = p_cam.x();
      double Y = p_cam.y();
      double Z = p_cam.z();
      double r_xy = std::sqrt(X * X + Y * Y);

      float u_in, v_in;
      if (r_xy < 1e-10) {
        // On optical axis
        u_in = static_cast<float>(cam0_.cx);
        v_in = static_cast<float>(cam0_.cy);
      } else {
        double theta = std::atan2(r_xy, Z);
        double phi = std::atan2(Y, X);

        if (theta > max_angle_rad) {
          u_in = -1.0f;
          v_in = -1.0f;
        } else {
          // Step 4: apply polyfisheye distortion
          double r_distorted = polyfisheyeR(theta, cam0_);

          // Step 5: compute distorted normalized coordinates
          double x_d = r_distorted * std::cos(phi);
          double y_d = r_distorted * std::sin(phi);

          // Step 6: convert to pixel coordinates
          u_in = static_cast<float>(cam0_.fx * x_d + cam0_.skew * y_d + cam0_.cx);
          v_in = static_cast<float>(cam0_.fy * y_d + cam0_.cy);
        }
      }

      map1_l_.at<float>(v, u) = u_in;
      map2_l_.at<float>(v, u) = v_in;
    }
  }

  // Generate right camera map
  for (int v = 0; v < out_height_; ++v) {
    for (int u = 0; u < out_width_; ++u) {
      Eigen::Vector3d p_rect = K_new_inv * Eigen::Vector3d(u, v, 1);
      Eigen::Vector3d p_cam = R2_inv * p_rect;

      double X = p_cam.x();
      double Y = p_cam.y();
      double Z = p_cam.z();
      double r_xy = std::sqrt(X * X + Y * Y);

      float u_in, v_in;
      if (r_xy < 1e-10) {
        u_in = static_cast<float>(cam1_.cx);
        v_in = static_cast<float>(cam1_.cy);
      } else {
        double theta = std::atan2(r_xy, Z);
        double phi = std::atan2(Y, X);

        if (theta > max_angle_rad) {
          u_in = -1.0f;
          v_in = -1.0f;
        } else {
          double r_distorted = polyfisheyeR(theta, cam1_);
          double x_d = r_distorted * std::cos(phi);
          double y_d = r_distorted * std::sin(phi);
          u_in = static_cast<float>(cam1_.fx * x_d + cam1_.skew * y_d + cam1_.cx);
          v_in = static_cast<float>(cam1_.fy * y_d + cam1_.cy);
        }
      }

      map1_r_.at<float>(v, u) = u_in;
      map2_r_.at<float>(v, u) = v_in;
    }
  }

  std::cout << "[StereoRectifier] Rectification maps generated" << std::endl;
}

void StereoRectifier::rectify(const cv::Mat& left_raw, const cv::Mat& right_raw, cv::Mat& left_rect,
                              cv::Mat& right_rect) const {
  if (!initialized_) {
    left_rect = left_raw.clone();
    right_rect = right_raw.clone();
    return;
  }

  cv::remap(left_raw, left_rect, map1_l_, map2_l_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  cv::remap(right_raw, right_rect, map1_r_, map2_r_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void StereoRectifier::rectifyLeft(const cv::Mat& raw, cv::Mat& rect) const {
  if (!initialized_) {
    rect = raw.clone();
    return;
  }
  cv::remap(raw, rect, map1_l_, map2_l_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void StereoRectifier::rectifyRight(const cv::Mat& raw, cv::Mat& rect) const {
  if (!initialized_) {
    rect = raw.clone();
    return;
  }
  cv::remap(raw, rect, map1_r_, map2_r_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

CameraInfoMsg StereoRectifier::getLeftCameraInfo() const {
  CameraInfoMsg info;
  info.width = out_width_;
  info.height = out_height_;
  info.distortion_model = "plumb_bob";

#if defined(ODIN_ROS2)
  // ROS2: lowercase field names
  info.d = {0, 0, 0, 0, 0};
  info.k = {K_new_(0, 0), K_new_(0, 1), K_new_(0, 2), K_new_(1, 0), K_new_(1, 1),
            K_new_(1, 2), K_new_(2, 0), K_new_(2, 1), K_new_(2, 2)};
  info.r = {R1_(0, 0), R1_(0, 1), R1_(0, 2), R1_(1, 0), R1_(1, 1),
            R1_(1, 2), R1_(2, 0), R1_(2, 1), R1_(2, 2)};
  info.p = {f_new_, 0, out_width_ / 2.0, 0, 0, f_new_, out_height_ / 2.0, 0, 0, 0, 1, 0};
#else
  // ROS1: uppercase field names
  info.D = {0, 0, 0, 0, 0};
  info.K = {K_new_(0, 0), K_new_(0, 1), K_new_(0, 2), K_new_(1, 0), K_new_(1, 1),
            K_new_(1, 2), K_new_(2, 0), K_new_(2, 1), K_new_(2, 2)};
  info.R = {R1_(0, 0), R1_(0, 1), R1_(0, 2), R1_(1, 0), R1_(1, 1),
            R1_(1, 2), R1_(2, 0), R1_(2, 1), R1_(2, 2)};
  info.P = {f_new_, 0, out_width_ / 2.0, 0, 0, f_new_, out_height_ / 2.0, 0, 0, 0, 1, 0};
#endif

  return info;
}

CameraInfoMsg StereoRectifier::getRightCameraInfo() const {
  CameraInfoMsg info;
  info.width = out_width_;
  info.height = out_height_;
  info.distortion_model = "plumb_bob";

  double Tx = -f_new_ * baseline_;

#if defined(ODIN_ROS2)
  // ROS2: lowercase field names
  info.d = {0, 0, 0, 0, 0};
  info.k = {K_new_(0, 0), K_new_(0, 1), K_new_(0, 2), K_new_(1, 0), K_new_(1, 1),
            K_new_(1, 2), K_new_(2, 0), K_new_(2, 1), K_new_(2, 2)};
  info.r = {R2_(0, 0), R2_(0, 1), R2_(0, 2), R2_(1, 0), R2_(1, 1),
            R2_(1, 2), R2_(2, 0), R2_(2, 1), R2_(2, 2)};
  info.p = {f_new_, 0, out_width_ / 2.0, Tx, 0, f_new_, out_height_ / 2.0, 0, 0, 0, 1, 0};
#else
  // ROS1: uppercase field names
  info.D = {0, 0, 0, 0, 0};
  info.K = {K_new_(0, 0), K_new_(0, 1), K_new_(0, 2), K_new_(1, 0), K_new_(1, 1),
            K_new_(1, 2), K_new_(2, 0), K_new_(2, 1), K_new_(2, 2)};
  info.R = {R2_(0, 0), R2_(0, 1), R2_(0, 2), R2_(1, 0), R2_(1, 1),
            R2_(1, 2), R2_(2, 0), R2_(2, 1), R2_(2, 2)};
  info.P = {f_new_, 0, out_width_ / 2.0, Tx, 0, f_new_, out_height_ / 2.0, 0, 0, 0, 1, 0};
#endif

  return info;
}

void StereoRectifier::printInfo() const {
  std::cout << "========== StereoRectifier Info ==========" << std::endl;
  std::cout << "  Input size:  " << cam0_.width << " x " << cam0_.height << std::endl;
  std::cout << "  Output size: " << out_width_ << " x " << out_height_ << std::endl;
  std::cout << "  Output FOV:  " << fov_deg_ << " deg" << std::endl;
  std::cout << "  New focal:   " << f_new_ << " px" << std::endl;
  std::cout << "  Baseline:    " << baseline_ * 1000.0 << " mm" << std::endl;
  std::cout << "  Left  P[0,3]: 0" << std::endl;
  std::cout << "  Right P[0,3]: " << -f_new_ * baseline_ << std::endl;
  std::cout << "==========================================" << std::endl;
}

}  // namespace odin
