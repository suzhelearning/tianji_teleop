#include "tianji_qp_ik/so3.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

Eigen::Vector3d skewVector(const Eigen::Matrix3d& rotation) {
  return {rotation(2, 1) - rotation(1, 2), rotation(0, 2) - rotation(2, 0),
          rotation(1, 0) - rotation(0, 1)};
}

Eigen::Vector3d nearPiAxis(const Eigen::Matrix3d& rotation) {
  Eigen::Vector3d axis;
  const Eigen::Vector3d diagonal = rotation.diagonal();
  Eigen::Index index = 0;
  diagonal.maxCoeff(&index);

  if (index == 0) {
    axis.x() = std::sqrt(std::max(0.0, 0.5 * (rotation(0, 0) + 1.0)));
    const double denominator = std::max(4.0 * axis.x(), 1e-15);
    axis.y() = (rotation(0, 1) + rotation(1, 0)) / denominator;
    axis.z() = (rotation(0, 2) + rotation(2, 0)) / denominator;
  } else if (index == 1) {
    axis.y() = std::sqrt(std::max(0.0, 0.5 * (rotation(1, 1) + 1.0)));
    const double denominator = std::max(4.0 * axis.y(), 1e-15);
    axis.x() = (rotation(0, 1) + rotation(1, 0)) / denominator;
    axis.z() = (rotation(1, 2) + rotation(2, 1)) / denominator;
  } else {
    axis.z() = std::sqrt(std::max(0.0, 0.5 * (rotation(2, 2) + 1.0)));
    const double denominator = std::max(4.0 * axis.z(), 1e-15);
    axis.x() = (rotation(0, 2) + rotation(2, 0)) / denominator;
    axis.y() = (rotation(1, 2) + rotation(2, 1)) / denominator;
  }

  if (axis.norm() < 1e-12) {
    axis = Eigen::Vector3d::UnitX();
  } else {
    axis.normalize();
  }
  if (axis.dot(skewVector(rotation)) < 0.0) {
    axis = -axis;
  }
  return axis;
}

}  // namespace

bool isProperRotation(const Eigen::Matrix3d& rotation, double tolerance) noexcept {
  if (!rotation.allFinite() || !std::isfinite(tolerance) || tolerance < 0.0) {
    return false;
  }
  const double orthogonality_error =
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
  return orthogonality_error <= tolerance &&
         std::abs(rotation.determinant() - 1.0) <= tolerance;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d& rotation) {
  if (!isProperRotation(rotation)) {
    throw std::invalid_argument("SO(3) logarithm received an invalid rotation matrix");
  }

  const double cosine = std::clamp(0.5 * (rotation.trace() - 1.0), -1.0, 1.0);
  const double angle = std::acos(cosine);
  const Eigen::Vector3d skew = skewVector(rotation);

  if (angle < 1e-7) {
    const double angle_squared = angle * angle;
    return (0.5 + angle_squared / 12.0) * skew;
  }
  if (kPi - angle < 1e-5) {
    return angle * nearPiAxis(rotation);
  }
  return (angle / (2.0 * std::sin(angle))) * skew;
}

Vec6 poseErrorWorld(const Pose& desired, const Pose& current) {
  Vec6 error;
  error.head<3>() = desired.position - current.position;
  error.tail<3>() = so3Log(desired.rotation * current.rotation.transpose());
  return error;
}

double rotationDistance(const Eigen::Matrix3d& first, const Eigen::Matrix3d& second) {
  return so3Log(first * second.transpose()).norm();
}

}  // namespace tianji_qp_ik
