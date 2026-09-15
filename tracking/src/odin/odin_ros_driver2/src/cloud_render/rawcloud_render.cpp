#include "rawcloud_render.h"
#include "yaml_config_loader.hpp"
#include "fisheye.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/io/pcd_io.h>

#include <sys/stat.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>

bool rawCloudRender::initFromString(const std::string& yamlContent, int img_width, int img_height) {
  YAML::Node root = YAML::Load(yamlContent);
  std::string cam_key = yaml_utils::ResolveCamKey(root, 0, img_width, img_height);
  if (cam_key.empty()) {
    std::cerr << "[rawCloudRender] Error: no calibration for cam_0 at " << img_width << "x"
              << img_height << std::endl;
    return false;
  }
  return initFromStringWithKey(yamlContent, cam_key);
}

bool rawCloudRender::initFromString(const std::string& yamlContent) {
  YAML::Node root = YAML::Load(yamlContent);
  std::string cam_key = yaml_utils::ResolveCamKey(root, 0, 0, 0);
  if (cam_key.empty()) {
    std::cerr << "[rawCloudRender] Error: no cam_0 calibration found in yaml" << std::endl;
    return false;
  }
  return initFromStringWithKey(yamlContent, cam_key);
}

bool rawCloudRender::initFromStringWithKey(const std::string& yamlContent,
                                           const std::string& cam_key) {
  auto cam_config_loader = std::make_shared<YAMLConfigLoader>();
  if (!cam_config_loader->loadFromString(yamlContent)) {
    std::cerr << "[rawCloudRender] Failed to parse YAML content." << std::endl;
    return false;
  }

  model_type_ = cam_config_loader->getValue<std::string>(cam_key + ".model_type", std::string(""));
  camera_name_ =
      cam_config_loader->getValue<std::string>(cam_key + ".camera_name", std::string(""));
  image_width_ =
      static_cast<int>(cam_config_loader->getValue<double>(cam_key + ".image_width", 0.0));
  image_height_ =
      static_cast<int>(cam_config_loader->getValue<double>(cam_key + ".image_height", 0.0));

  if (image_width_ <= 0 || image_height_ <= 0) {
    std::cerr << "[rawCloudRender] Error: invalid image_width/image_height in yaml." << std::endl;
    return false;
  }
  frame_size_ = image_width_ * image_height_;

  std::vector<double> SE3T = cam_config_loader->getVector<double>("Tcl_0");

  if (SE3T.size() >= 16) {
    T_camera_lidar_ << static_cast<float>(SE3T[0]), static_cast<float>(SE3T[1]),
        static_cast<float>(SE3T[2]), static_cast<float>(SE3T[3]), static_cast<float>(SE3T[4]),
        static_cast<float>(SE3T[5]), static_cast<float>(SE3T[6]), static_cast<float>(SE3T[7]),
        static_cast<float>(SE3T[8]), static_cast<float>(SE3T[9]), static_cast<float>(SE3T[10]),
        static_cast<float>(SE3T[11]), static_cast<float>(SE3T[12]), static_cast<float>(SE3T[13]),
        static_cast<float>(SE3T[14]), static_cast<float>(SE3T[15]);
  } else {
    std::cerr << "[rawCloudRender] Error: Tcl_0/T_camera_lidar size is " << SE3T.size()
              << ", expected >= 12." << std::endl;
    return false;
  }

  k2_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".k2", 0.0));
  k3_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".k3", 0.0));
  k4_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".k4", 0.0));
  k5_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".k5", 0.0));
  k6_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".k6", 0.0));
  k7_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".k7", 0.0));

  p1_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".p1", 0.0));
  p2_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".p2", 0.0));

  A11_fx_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".A11", 0.0));
  A12_skew_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".A12", 0.0));
  A22_fy_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".A22", 0.0));
  u0_cx_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".u0", 0.0));
  v0_cy_ = static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".v0", 0.0));

  maxIncidentAngle_ =
      static_cast<float>(cam_config_loader->getValue<double>(cam_key + ".maxIncidentAngle", 120.0));

  return true;
}

void rawCloudRender::render(const cv::Mat& image_bgr,
                            const pcl::PointCloud<pcl::PointXYZ>& cloud_xyz,
                            pcl::PointCloud<pcl::PointXYZRGBA>& colored_cloud) {
  if (image_bgr.empty() || image_bgr.type() != CV_8UC3 || image_bgr.rows != image_height_ ||
      image_bgr.cols != image_width_) {
    std::cerr << "mal-formed image"
              << " image[actual height]: " << image_bgr.rows
              << " image[actual width]: " << image_bgr.cols
              << " image[calib height]: " << image_height_
              << " image[calib width]: " << image_width_ << std::endl;
    return;
  }

  const int total_point_num = static_cast<int>(cloud_xyz.size());
  colored_cloud.reserve(total_point_num);
  colored_cloud.clear();

  camera::FisheyeProjection proj;
  const double intrinsic[5] = {
      static_cast<double>(A11_fx_), static_cast<double>(A12_skew_), static_cast<double>(A22_fy_),
      static_cast<double>(u0_cx_),  static_cast<double>(v0_cy_),
  };
  const double distortion[6] = {
      static_cast<double>(k2_), static_cast<double>(k3_), static_cast<double>(k4_),
      static_cast<double>(k5_), static_cast<double>(k6_), static_cast<double>(k7_),
  };

  int valid_point_num = 0;
  for (int idx = 0; idx < total_point_num; ++idx) {
    // projection
    const pcl::PointXYZ& pt = cloud_xyz.points[static_cast<size_t>(idx)];
    const float x = pt.x;
    const float y = pt.y;
    const float z = pt.z;

    Eigen::Vector4f pt_homo(x, y, z, 1.0f);

    Eigen::Vector4f pt_cam_homo = T_camera_lidar_ * pt_homo;

    // Visibility check
    if (pt_cam_homo.z() <= 0.1f) {
      continue;
    }

    const Eigen::Vector3d pt_cam_xyz = pt_cam_homo.template head<3>().cast<double>();
    const Eigen::Matrix<double, 3, 1> pt3d = pt_cam_xyz / pt_cam_xyz.z();
    const Eigen::Matrix<double, 2, 1> uv = proj(intrinsic, distortion, pt3d);
    const int u = static_cast<int>(uv(0));
    const int v = static_cast<int>(uv(1));

    if (u < 0 || v < 0 || u >= image_width_ || v >= image_height_) {
      continue;
    }

    const cv::Vec3b color = image_bgr.at<cv::Vec3b>(v, u);

    pcl::PointXYZRGBA pcl_point;
    pcl_point.x = x;
    pcl_point.y = y;
    pcl_point.z = z;
    pcl_point.r = color[2];
    pcl_point.g = color[1];
    pcl_point.b = color[0];
    pcl_point.a = 255;
    colored_cloud.push_back(pcl_point);

    valid_point_num++;
  }
}

void rawCloudRender::print_camera_calib() {
  std::cout << model_type_ << std::endl;
  std::cout << camera_name_ << std::endl;
  std::cout << image_width_ << std::endl;
  std::cout << image_height_ << std::endl;

  std::cout << "T_camera_lidar" << std::endl;
  std::cout << T_camera_lidar_(0, 0) << " " << T_camera_lidar_(0, 1) << " " << T_camera_lidar_(0, 2)
            << " " << T_camera_lidar_(0, 3) << std::endl;
  std::cout << T_camera_lidar_(1, 0) << " " << T_camera_lidar_(1, 1) << " " << T_camera_lidar_(1, 2)
            << " " << T_camera_lidar_(1, 3) << std::endl;
  std::cout << T_camera_lidar_(2, 0) << " " << T_camera_lidar_(2, 1) << " " << T_camera_lidar_(2, 2)
            << " " << T_camera_lidar_(2, 3) << std::endl;
  std::cout << T_camera_lidar_(3, 0) << " " << T_camera_lidar_(3, 1) << " " << T_camera_lidar_(3, 2)
            << " " << T_camera_lidar_(3, 3) << std::endl;

  std::cout << "cam" << std::endl;
  std::cout << k2_ << std::endl;
  std::cout << k3_ << std::endl;
  std::cout << k4_ << std::endl;
  std::cout << k5_ << std::endl;
  std::cout << k6_ << std::endl;
  std::cout << k7_ << std::endl;

  std::cout << A11_fx_ << std::endl;
  std::cout << A12_skew_ << std::endl;
  std::cout << A22_fy_ << std::endl;
  std::cout << u0_cx_ << std::endl;
  std::cout << v0_cy_ << std::endl;
}