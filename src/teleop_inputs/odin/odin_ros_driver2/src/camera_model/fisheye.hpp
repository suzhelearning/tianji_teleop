#pragma once

#include <Eigen/Core>
#include "traits.hpp"

namespace camera {

/**
 * @brief Pinhole projection + fisheye distortion (compatible with OpenCV maybe)
 * https://docs.opencv.org/4.5.2/db/d58/group__calib3d__fisheye.html
 */
struct FisheyeProjection {
  template <typename T, typename T2>
  auto operator()(const T* const intrinsic, const T* const distortion,
                  const Eigen::Matrix<T2, 3, 1>& point_3d) const
      -> Eigen::Matrix<decltype(T() * T2()), 2, 1> {
    // const auto r = point_3d.template head<2>().norm();
    // const auto theta = atan2(r, abs(point_3d.z()));
    // const auto theta2 = pow(theta, 2);
    // const auto theta4 = pow(theta, 4);
    // const auto theta6 = pow(theta, 6);
    // const auto theta8 = pow(theta, 8);

    // const auto& k1 = distortion[0];
    // const auto& k2 = distortion[1];
    // const auto& k3 = distortion[2];
    // const auto& k4 = distortion[3];

    // const auto theta_d = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    // const auto pt_d = ((theta_d / r) * point_3d.template head<2>()).eval();

    // const auto& fx = intrinsic[0];
    // const auto& fy = intrinsic[1];
    // const auto& cx = intrinsic[2];
    // const auto& cy = intrinsic[3];

    // return {fx * pt_d[0] + cx, fy * pt_d[1] + cy};

    const auto& k2_ = distortion[0];
    const auto& k3_ = distortion[1];
    const auto& k4_ = distortion[2];
    const auto& k5_ = distortion[3];
    const auto& k6_ = distortion[4];
    const auto& k7_ = distortion[5];

    const auto& fx_ = intrinsic[0];
    const auto& skew_ = intrinsic[1];
    const auto& fy_ = intrinsic[2];
    const auto& cx_ = intrinsic[3];
    const auto& cy_ = intrinsic[4];

    // double xd, yd;
    // const double r = sqrt( xyz( 1 ) * xyz( 1 ) + xyz( 0 ) * xyz( 0 ));
    const auto r = point_3d.template head<2>().norm();

    // if (r < 1e-8)
    // {
    //   return uv;
    // }

    // const double theta = acos( xyz( 2 ) / xyz.norm( ) );
    const auto theta = acos(point_3d.z() / point_3d.norm());

    // const auto thetad = thetad_from_theta(theta);
    const auto theta2 = theta * theta;
    const auto theta3 = theta2 * theta;
    const auto theta4 = theta3 * theta;
    const auto theta5 = theta4 * theta;
    const auto theta6 = theta5 * theta;
    const auto theta7 = theta6 * theta;
    const auto thetad = theta + k2_ * theta2 + k3_ * theta3 + k4_ * theta4 + k5_ * theta5 +
                        k6_ * theta6 + k7_ * theta7;

    const auto scaling = thetad / r;
    // xd = xyz[0] * scaling;
    // yd = xyz[1] * scaling;
    // px[0] = xd*fx_ + yd*skew_ + cx_;
    // px[1] = yd*fy_ + cy_;
    const auto xd = point_3d.x() * scaling;
    const auto yd = point_3d.y() * scaling;

    // if (abs(point_3d.x() - (-3.55252935277652)) < 1e-4 &&
    //     abs(point_3d.y() - 1.239844212294181) < 1e-4 &&
    //     abs(point_3d.z() - 5.805775403317319) < 1e-4) {
    //       std::cout << "xd yd: " << xd << " " << yd << std::endl;
    //       std::cout << fx_ << " " << skew_ << " " << fy_ << " " << cx_ << " " << cy_ <<
    //       std::endl;
    //     }

    return {xd * fx_ + yd * skew_ + cx_, yd * fy_ + cy_};
  }
};

// traits
template <>
struct CameraModelTraits<FisheyeProjection> {
  static constexpr int num_intrinsic_params = 5;   // was 4
  static constexpr int num_distortion_params = 6;  // was 4

  static std::string projection_model() { return "pinhole"; }

  static std::string distortion_model() { return "fisheye"; }

  static Eigen::Vector4d init_intrinsic(int width, int height,
                                        const Eigen::VectorXd& pinhole_intrinsic) {
    return pinhole_intrinsic;
  }

  static Eigen::Matrix<double, 4, 1> init_distortion() {
    return Eigen::Matrix<double, 4, 1>::Zero();
  }
};

}  // namespace camera